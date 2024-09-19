/*
 * Copyright (c) 2024 Indoor Corgi
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef PICOBOOT3_H
#define PICOBOOT3_H

#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "pico/stdlib.h"

// -----------------
// Configurations

// 4 bytes data to allow the host to recognize the bootloader
#define PICOBOOT3_ACTIVATION_RESPONSE {0x70, 0x62, 0x74, 0x33}

// A version code to inform the host of the bootloader capability
#define PICOBOOT3_MAJOR_VERSION 1
#define PICOBOOT3_MINOR_VERSION 0
#define PICOBOOT3_PATCH_VERSION 0

// GPIO settings for switching between user applications or bootloaders
#define PICOBOOT3_BOOTSEL3_PIN 22
#define PICOBOOT3_BOOTSEL3_PULLUP 1
#define PICOBOOT3_BOOTSEL3_READ_DELAY_MS 5
#define PICOBOOT3_BOOTSEL3_VAL_TO_START_BOOTLOADER 0

// Offset of the application code
// Bootloader binary size must be smaller than this
#define PICOBOOT3_APPCODE_OFFSET 32 * 1024

// UART setting to communicate with the host
#define PICOBOOT3_UART_INST uart_default
#define PICOBOOT3_UART_TX_PIN PICO_DEFAULT_UART_TX_PIN
#define PICOBOOT3_UART_RX_PIN PICO_DEFAULT_UART_RX_PIN
#define PICOBOOT3_UART_BAUDRATE 500000
#define PICOBOOT3_UART_TIMEOUT_MS 1000

// I2C setting to communicate with the host
#define PICOBOOT3_I2C_INST i2c_default
#define PICOBOOT3_I2C_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#define PICOBOOT3_I2C_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN
#define PICOBOOT3_I2C_ADDRESS 0x5E

// UART for printf debug
// If it's enabled, you can use printf after picoboot3_debug_uart_init()
// Use different uart instanse from above
#define PICOBOOT3_DEBUG_UART 0  // Set 1 to enable
#define PICOBOOT3_DEBUG_UART_INST uart1
#define PICOBOOT3_DEBUG_UART_BAUDRATE 115200
#define PICOBOOT3_DEBUG_UART_TX_PIN 8
#define PICOBOOT3_DEBUG_UART_RX_PIN 9

// -----------------

void picoboot3_go_to_appcode();
void picoboot3_bootsel_init();
void picoboot3_bootsel_deinit();
bool picoboot3_bootsel_is_bootloader();
void picoboot3_debug_uart_init();
void picoboot3_debug_uart_deinit();
void picoboot3_uart_init();
void picoboot3_uart_deinit();
void picoboot3_uart_rx_handler();
void picoboot3_uart_timeout_handler();
void picoboot3_i2c_init();
void picoboot3_i2c_deinit();
void picoboot3_i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event);
void picoboot3_i2c_command_handler();
void picoboot3_reserved_command_handler();

#endif