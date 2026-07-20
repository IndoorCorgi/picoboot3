
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

int main() {
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  // Blink LED 10 times
  for (int i = 0; i < 10; i++) {
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_ms(1000);
  }

  // Example to enter picoboot3 from your application without holding BOOTSEL3 pin
  watchdog_hw->scratch[0] = 1;  // Notify picoboot3 to enter bootloader mode
  watchdog_reboot(0, 0, 10);    // Reoot by watchdog timeout
  while (1) {
    continue;
  }
}