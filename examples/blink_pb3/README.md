## Overview

An example blink project to run with Picoboot3. 


## Key Changes

- Copied memmap_default.ld from SDK
- Added 32KB offset in the ld file
- Removed boot2 in the ld file
- Changed CMakeLists.txt to load modified ld file


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