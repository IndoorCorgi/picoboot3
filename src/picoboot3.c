/*
 * Copyright (c) 2024 Indoor Corgi
 *
 * SPDX-License-Identifier: MIT
 */

#include "picoboot3.h"

#include <stdio.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "picoboot3.h"

#define RECEIVE_BUFFER_SIZE 4200
#define MAX_READ_BYTES 4096  // For read command

// To determine interface
#define NO_INTERFACE 0
#define UART_INTERFACE 1
#define I2C_INTERFACE 2

// Command codes
#define NO_COMMAND 0
#define READY_BUSY_COMMAND 0x1
#define VERSION_COMMAND 0x2
#define READ_COMMAND 0x10
#define PROGRAM_COMMAND 0x20
#define ERASE_COMMAND 0x30
#define GO_TO_APPCODE_COMMAND 0x40
#define FLASH_SIZE_COMMAND 0x50
#define ACTIVATE_COMMAND 0xA5

// To determine what data to be sent to the host
#define SEND_NONE 0
#define SEND_READY_BUSY 1
#define SEND_FLASH_DATA 2
#define SEND_ACTIVATION_RESPONSE 3
#define SEND_VERSION 4
#define SEND_FLASH_SIZE 5

// Vector table offset
#if PICO_RP2040
#define VTOR_OFFSET M0PLUS_VTOR_OFFSET
#elif PICO_RP2350
#define VTOR_OFFSET M33_VTOR_OFFSET
#endif

typedef struct {
  uint8_t command;
  uint8_t flash_address[4];
  uint8_t num_of_bytes[2];
} read_command_t;

typedef struct {
  uint8_t command;
  uint8_t flash_address[4];
  uint8_t num_of_bytes[2];
  uint8_t data[FLASH_SECTOR_SIZE];
} program_command_t;

typedef struct {
  uint8_t command;
  uint8_t sector[2];
} erase_command_t;

int activated_interface = NO_INTERFACE;  // Set activated interface
const uint8_t activation_response[] = PICOBOOT3_ACTIVATION_RESPONSE;
const uint8_t version[] = {PICOBOOT3_MAJOR_VERSION,
                           PICOBOOT3_MINOR_VERSION,
                           PICOBOOT3_PATCH_VERSION};
uint8_t ready = 1;  // 1:ready, 0:busy(program or erase is running)

uint8_t uart_receive_buffer[RECEIVE_BUFFER_SIZE];
int uart_receive_counter = 0;            // Stores the number of bytes of received data
uint32_t uart_last_receive_time_ms = 0;  // Stores the last receive time
uint8_t i2c_receive_buffer[RECEIVE_BUFFER_SIZE];
int i2c_receive_counter = 0;           // Stores the number of bytes of received data
int i2c_select_send_data = SEND_NONE;  // Select what data to be sent to master
int i2c_send_counter = 0;              // Stores the number of bytes of sent data

// To pass parameters to picoboot3_reserved_command_handler()
uint8_t reserved_command = NO_COMMAND;
program_command_t reserved_program_command;
erase_command_t reserved_erase_command;

// Branches to application code and does not return
// Before calling this, deinit all used resources.
void picoboot3_go_to_appcode() {
  asm volatile(
      "ldr r0, =%[appcode]\n"
      "ldr r1, =%[vtor]\n"
      "str r0, [r1]\n"
      "ldmia r0, {r0, r1}\n"
      "msr msp, r0\n"
      "bx r1\n"
      :
      : [appcode] "i"(XIP_BASE + PICOBOOT3_APPCODE_OFFSET), [vtor] "i"(PPB_BASE + VTOR_OFFSET)
      :);
}

void picoboot3_bootsel_init() {
  gpio_init(PICOBOOT3_BOOTSEL3_PIN);
  if (PICOBOOT3_BOOTSEL3_PULLUP) {
    gpio_pull_up(PICOBOOT3_BOOTSEL3_PIN);
  }
}

void picoboot3_bootsel_deinit() {
  gpio_deinit(PICOBOOT3_BOOTSEL3_PIN);
}

bool picoboot3_bootsel_is_bootloader() {
  if (PICOBOOT3_BOOTSEL3_VAL_TO_START_BOOTLOADER == gpio_get(PICOBOOT3_BOOTSEL3_PIN)) {
    return true;
  }
  return false;
}

void picoboot3_debug_uart_init() {
#if PICOBOOT3_DEBUG_UART
  stdio_uart_init_full(PICOBOOT3_DEBUG_UART_INST,
                       PICOBOOT3_DEBUG_UART_BAUDRATE,
                       PICOBOOT3_DEBUG_UART_TX_PIN,
                       PICOBOOT3_DEBUG_UART_RX_PIN);
#endif
}

void picoboot3_debug_uart_deinit() {
#if PICOBOOT3_DEBUG_UART
  stdio_uart_deinit_full(PICOBOOT3_DEBUG_UART_INST,
                         PICOBOOT3_DEBUG_UART_TX_PIN,
                         PICOBOOT3_DEBUG_UART_RX_PIN);
#endif
}

void picoboot3_uart_init() {
  gpio_init(PICOBOOT3_UART_TX_PIN);
  gpio_init(PICOBOOT3_UART_RX_PIN);
  gpio_set_function(PICOBOOT3_UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(PICOBOOT3_UART_RX_PIN, GPIO_FUNC_UART);
  uart_init(PICOBOOT3_UART_INST, PICOBOOT3_UART_BAUDRATE);
  int uart_irq = PICOBOOT3_UART_INST == uart0 ? UART0_IRQ : UART1_IRQ;
  irq_set_exclusive_handler(uart_irq, picoboot3_uart_rx_handler);
  irq_set_enabled(uart_irq, true);
  uart_set_irqs_enabled(PICOBOOT3_UART_INST, true, false);
}

void picoboot3_uart_deinit() {
  int uart_irq = PICOBOOT3_UART_INST == uart0 ? UART0_IRQ : UART1_IRQ;
  irq_set_enabled(uart_irq, false);
  irq_remove_handler(uart_irq, picoboot3_uart_rx_handler);
  uart_deinit(PICOBOOT3_UART_INST);
  gpio_deinit(PICOBOOT3_UART_TX_PIN);
  gpio_deinit(PICOBOOT3_UART_RX_PIN);
}

// UART interrupt handler
void picoboot3_uart_rx_handler() {
  uint16_t num_of_bytes;

  while (uart_is_readable(PICOBOOT3_UART_INST)) {
    uart_receive_buffer[uart_receive_counter++] = uart_getc(PICOBOOT3_UART_INST);
    uart_last_receive_time_ms = to_ms_since_boot(get_absolute_time());

    if (uart_receive_buffer[0] == ACTIVATE_COMMAND) {
      if (activated_interface != NO_INTERFACE && activated_interface != UART_INTERFACE) {
        uart_receive_counter = 0;
        continue;  // Only one interface is able to be acvive
      }
    } else {
      if (activated_interface != UART_INTERFACE) {
        uart_receive_counter = 0;
        continue;  // Other commands are valid after activate command
      }
    }

    if (!ready && uart_receive_buffer[0] != READY_BUSY_COMMAND) {
      uart_receive_counter = 0;
      continue;  // If busy, accept ready/busy command only
    }

    switch (uart_receive_buffer[0]) {
      case READY_BUSY_COMMAND:
        uart_putc_raw(PICOBOOT3_UART_INST, ready);
        uart_receive_counter = 0;
        break;

      case VERSION_COMMAND:
        for (int i = 0; i < sizeof(version); i++) {
          uart_putc_raw(PICOBOOT3_UART_INST, version[i]);
        }
        uart_receive_counter = 0;
        break;

      case READ_COMMAND:
        if (uart_receive_counter < sizeof(read_command_t)) break;
        read_command_t* read_command = (read_command_t*)uart_receive_buffer;
        num_of_bytes = *(uint16_t*)read_command->num_of_bytes;
        uint32_t flash_address = *(uint32_t*)read_command->flash_address;
        if (num_of_bytes < 1 || num_of_bytes > MAX_READ_BYTES) break;
        for (int i = 0; i < num_of_bytes; i++) {
          while (!uart_is_writable(PICOBOOT3_UART_INST));
          uart_putc_raw(PICOBOOT3_UART_INST, *(uint8_t*)(XIP_BASE + flash_address++));
        }
        uart_receive_counter = 0;
        break;

      case PROGRAM_COMMAND:
        if (uart_receive_counter < 7) break;
        num_of_bytes = *(uint16_t*)(uart_receive_buffer + 5);
        if (uart_receive_counter < 7 + num_of_bytes) break;
        memcpy(&reserved_program_command, uart_receive_buffer, sizeof(program_command_t));
        reserved_command = PROGRAM_COMMAND;
        uart_receive_counter = 0;
        ready = 0;
        break;

      case ERASE_COMMAND:
        if (uart_receive_counter < sizeof(erase_command_t)) break;
        memcpy(&reserved_erase_command, uart_receive_buffer, sizeof(erase_command_t));
        reserved_command = ERASE_COMMAND;
        ready = 0;
        uart_receive_counter = 0;
        break;

      case GO_TO_APPCODE_COMMAND:
        reserved_command = GO_TO_APPCODE_COMMAND;
        ready = 0;
        uart_receive_counter = 0;
        break;

      case FLASH_SIZE_COMMAND:
        for (int i = 0; i < 4; i++) {
          uart_putc_raw(PICOBOOT3_UART_INST, (PICO_FLASH_SIZE_BYTES >> (i * 8)) & 0xFF);
        }
        uart_receive_counter = 0;
        break;

      case ACTIVATE_COMMAND:
        activated_interface = UART_INTERFACE;
        for (int i = 0; i < sizeof(activation_response); i++) {
          uart_putc_raw(PICOBOOT3_UART_INST, activation_response[i]);
        }
        uart_receive_counter = 0;
        break;

      default:
        uart_receive_counter = 0;  // Unknown command. Clear buffer.
        break;
    }
  }
}

// Claer UART receive counter if timeout
// Call this in the main loop
void picoboot3_uart_timeout_handler() {
  if (uart_receive_counter > 0) {
    if (uart_last_receive_time_ms + PICOBOOT3_UART_TIMEOUT_MS <
        to_ms_since_boot(get_absolute_time())) {
      uart_receive_counter = 0;
    }
  }
}

void picoboot3_i2c_init() {
  gpio_init(PICOBOOT3_I2C_SDA_PIN);
  gpio_set_function(PICOBOOT3_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICOBOOT3_I2C_SDA_PIN);

  gpio_init(PICOBOOT3_I2C_SCL_PIN);
  gpio_set_function(PICOBOOT3_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICOBOOT3_I2C_SCL_PIN);

  i2c_slave_init(PICOBOOT3_I2C_INST, PICOBOOT3_I2C_ADDRESS, &picoboot3_i2c_slave_handler);
}

void picoboot3_i2c_deinit() {
  i2c_slave_deinit(PICOBOOT3_I2C_INST);
  gpio_deinit(PICOBOOT3_I2C_SDA_PIN);
  gpio_deinit(PICOBOOT3_I2C_SCL_PIN);
}

// I2C interrupt handler
void picoboot3_i2c_slave_handler(i2c_inst_t* i2c, i2c_slave_event_t event) {
  switch (event) {
    case I2C_SLAVE_RECEIVE:
      if (i2c_receive_counter < RECEIVE_BUFFER_SIZE) {
        i2c_receive_buffer[i2c_receive_counter++] = i2c_read_byte_raw(i2c);
      } else {
        i2c_read_byte_raw(i2c);
      }
      break;
    case I2C_SLAVE_REQUEST:
      switch (i2c_select_send_data) {
        case SEND_READY_BUSY:
          i2c_write_byte_raw(i2c, ready);
          break;

        case SEND_FLASH_DATA:
          i2c_write_byte_raw(i2c, *(uint8_t*)(XIP_BASE + i2c_send_counter++));
          if (i2c_send_counter >= PICO_FLASH_SIZE_BYTES) {
            i2c_send_counter = 0;
          }
          break;

        case SEND_VERSION:
          i2c_write_byte_raw(i2c, version[i2c_send_counter++]);
          if (i2c_send_counter >= sizeof(version)) {
            i2c_send_counter = 0;
          }
          break;

        case SEND_ACTIVATION_RESPONSE:
          i2c_write_byte_raw(i2c, activation_response[i2c_send_counter++]);
          if (i2c_send_counter >= sizeof(activation_response)) {
            i2c_send_counter = 0;
          }
          break;

        case SEND_FLASH_SIZE:
          i2c_write_byte_raw(i2c, (PICO_FLASH_SIZE_BYTES >> (i2c_send_counter++ * 8)) & 0xFF);
          if (i2c_send_counter >= 4) {
            i2c_send_counter = 0;
          }
          break;

        default:
          i2c_write_byte_raw(i2c, 0);
          break;
      }
      break;
    case I2C_SLAVE_FINISH:
      if (i2c_receive_counter > 0) picoboot3_i2c_command_handler(I2C_INTERFACE);
      i2c_receive_counter = 0;
      break;
    default:
      break;
  }
}

// Handle received commands from I2C
// Called by picoboot3_i2c_slave_handler
void picoboot3_i2c_command_handler() {
  uint16_t num_of_bytes;

  if (i2c_receive_buffer[0] == ACTIVATE_COMMAND) {
    if (activated_interface != NO_INTERFACE && activated_interface != I2C_INTERFACE)
      return;  // Only one interface is able to be acvive
  } else {
    if (activated_interface != I2C_INTERFACE)
      return;  // Other commands are valid after activate command
  }

  if (!ready && i2c_receive_buffer[0] != READY_BUSY_COMMAND)
    return;  // If busy, accept ready/busy command only

  switch (i2c_receive_buffer[0]) {
    case READY_BUSY_COMMAND:
      if (i2c_receive_counter != 1) break;
      i2c_select_send_data = SEND_READY_BUSY;
      break;

    case VERSION_COMMAND:
      if (i2c_receive_counter != 1) break;
      i2c_select_send_data = SEND_VERSION;
      i2c_send_counter = 0;
      break;

    case READ_COMMAND:
      if (i2c_receive_counter != sizeof(read_command_t)) break;
      read_command_t* read_command = (read_command_t*)i2c_receive_buffer;
      num_of_bytes = *(uint16_t*)read_command->num_of_bytes;
      uint32_t flash_address = *(uint32_t*)read_command->flash_address;
      if (num_of_bytes < 1 || num_of_bytes > MAX_READ_BYTES) break;
      i2c_select_send_data = SEND_FLASH_DATA;
      i2c_send_counter = flash_address;
      break;

    case PROGRAM_COMMAND:
      if (i2c_receive_counter < 7) break;
      num_of_bytes = *(uint16_t*)(i2c_receive_buffer + 5);
      if (i2c_receive_counter < 7 + num_of_bytes) break;
      memcpy(&reserved_program_command, i2c_receive_buffer, sizeof(program_command_t));
      i2c_select_send_data = SEND_READY_BUSY;
      reserved_command = PROGRAM_COMMAND;
      ready = 0;
      break;

    case ERASE_COMMAND:
      if (i2c_receive_counter != sizeof(erase_command_t)) break;
      memcpy(&reserved_erase_command, i2c_receive_buffer, sizeof(erase_command_t));
      i2c_select_send_data = SEND_READY_BUSY;
      reserved_command = ERASE_COMMAND;
      ready = 0;
      break;

    case GO_TO_APPCODE_COMMAND:
      if (i2c_receive_counter != 1) break;
      i2c_select_send_data = SEND_READY_BUSY;
      reserved_command = GO_TO_APPCODE_COMMAND;
      ready = 0;
      break;

    case FLASH_SIZE_COMMAND:
      if (i2c_receive_counter != 1) break;
      i2c_select_send_data = SEND_FLASH_SIZE;
      i2c_send_counter = 0;
      break;

    case ACTIVATE_COMMAND:
      if (i2c_receive_counter != 1) break;
      activated_interface = I2C_INTERFACE;
      i2c_select_send_data = SEND_ACTIVATION_RESPONSE;
      i2c_send_counter = 0;
      break;

    default:
      break;
  }
}

// Handle program, erase and go_to_appcode command here as they take time
// Call this in the main loop
void picoboot3_reserved_command_handler() {
  switch (reserved_command) {
    case GO_TO_APPCODE_COMMAND:
      picoboot3_bootsel_deinit();
      picoboot3_uart_deinit();
      picoboot3_i2c_deinit();
      picoboot3_debug_uart_deinit();
      picoboot3_go_to_appcode();
      break;

    case PROGRAM_COMMAND:
      uint32_t flash_address = *(uint32_t*)reserved_program_command.flash_address;
      uint16_t num_of_bytes = *(uint16_t*)reserved_program_command.num_of_bytes;

      if (PICOBOOT3_APPCODE_OFFSET > flash_address ||
          PICO_FLASH_SIZE_BYTES <= flash_address ||
          flash_address % FLASH_PAGE_SIZE != 0) {
        reserved_command = NO_COMMAND;
        ready = 1;
        break;
      }
      if (num_of_bytes == 0 || num_of_bytes % FLASH_PAGE_SIZE != 0) {
        reserved_command = NO_COMMAND;
        ready = 1;
        break;
      }

      // Even if running in SRAM, disable interrupt to avoid failure on I2C
      uint32_t interrupt_status = save_and_disable_interrupts();
      flash_range_program(flash_address,
                          (const uint8_t*)&reserved_program_command.data,
                          num_of_bytes);
      restore_interrupts(interrupt_status);
      reserved_command = NO_COMMAND;
      ready = 1;
      break;

    case ERASE_COMMAND:
      uint16_t sector = *(uint16_t*)reserved_erase_command.sector;

      // The sector is used by picoboot3 or out of flash size
      if (PICOBOOT3_APPCODE_OFFSET / FLASH_SECTOR_SIZE > sector ||
          PICO_FLASH_SIZE_BYTES / FLASH_SECTOR_SIZE <= sector) {
        reserved_command = NO_COMMAND;
        ready = 1;
        break;
      }
      flash_range_erase(FLASH_SECTOR_SIZE * sector, FLASH_SECTOR_SIZE);
      reserved_command = NO_COMMAND;
      ready = 1;
      break;

    default:
      break;
  }
}
