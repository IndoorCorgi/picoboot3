## Overview

An example blink project to run with Picoboot3. 
After blinking LED 10 times, enter picoboot3 without holding BOOTSEL3 pin. 

This example is for SDK 2.3.0 or above. Example for SDK 2.0.0 - 2.2.0 is [here](../blink_pb3_sdk2.0.0_2.2.0/). 

## Key Changes

- Created linker script override files in linker_script_XXXX directory. 
- Described flash address with 32KB offset in the memory_flash.incl file. 
- Keep section_boot2.incl blank to removed boot2 for RP2040. 
- Added pico_add_linker_script_override_path(...) to the CMakeLists.txt to load override files. 


## How to Build

If you use VS Code and Raspberry Pi Pico extension, 
open this directory and import as Raspberry Pi Pico project. 

Change PICO_BOARD value in [CMakeLists.txt](CMakeLists.txt) to match your board. 
(e.g. pico2)
~~~
set(PICO_BOARD pico CACHE STRING "Board type")
~~~

Now build it as you normally would. 
For example:
~~~
cmake -B build -G Ninja -DPICO_SDK_PATH=your_sdk_path
ninja -C build
~~~


## Write Firmware via UART

~~~
picoboot3 -f build/blink_pb3.bin -a
~~~