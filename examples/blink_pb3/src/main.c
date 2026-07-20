
#include "hardware/watchdog.h"
#include "pico/status_led.h"
#include "pico/stdlib.h"

int main() {
  status_led_init();

  // Blink LED 10 times
  for (int i = 0; i < 10; i++) {
    status_led_set_state(1);
    sleep_ms(1000);
    status_led_set_state(0);
    sleep_ms(1000);
  }

  // Example to enter picoboot3 from your application without holding BOOTSEL3 pin
  watchdog_hw->scratch[0] = 1;  // Notify picoboot3 to enter bootloader mode
  watchdog_reboot(0, 0, 10);    // Reoot by watchdog timeout
  while (1) {
    continue;
  }
}