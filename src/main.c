/*
 * Copyright (c) 2024 Indoor Corgi
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "picoboot3.h"

int main() {
  picoboot3_bootsel_init();
  sleep_ms(PICOBOOT3_BOOTSEL3_READ_DELAY_MS);
  if (!picoboot3_bootsel_is_bootloader()) {
    picoboot3_bootsel_deinit();
    picoboot3_go_to_appcode();
  }

  picoboot3_debug_uart_init();
  picoboot3_uart_init();
  picoboot3_i2c_init();

  while (1) {
    picoboot3_reserved_command_handler();
    picoboot3_uart_timeout_handler();
  }
}
