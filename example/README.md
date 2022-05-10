# This is an example showing how to use simavr-nRF24 (and simavr-spi-dispatcher).

*Warning: Because of a bug in simavr the internal pullup needed for the IRQ-pin of both nRF24 is not enabled inside the AVR, so if you want to test this code on real hardware you need to add an external pullup!*

## Licence and disclaimer
AGPLv3+ and NO WARRANTY!

## Prerequisites
You need libsimavr and the simavr-headers inside folder "sim". Symlinks are fine (create a symlink to *folder* "sim", not symlinks to the files inside).  
You need the following files in your working directory: main.c, nRF.h, nRF_config.h, nRF_defs.h, nRF_internals.h, nRF.c, spi_dispatcher.h, spi_dispatcher.c  
You will also need libelf installed on your system (Debian: `sudo apt install libelf1`).

## How to compile
```
gcc -Wall -Wextra -Werror -I./sim main.c spi_dispatcher.c nRF.c -L. -lsimavr -lelf -o example -Wl,-rpath,.
```
The `-Wl`-stuff tells GCC to write into the binary that needed libraries are in the same directory (.) as the binary.

## How to execute
```
./example
```
The simulation will run forever until interrupted with Ctrl+C. Change the code, recompile and execute to experiment with different settings like nRF-loglevel, number of lost packets, ...

## What does this do exactly??
This simulates the communication between two ATmega328P using two nRF24L01+ (configured for 2Mbps, 2 Bytes CRC, ACK with payload). The first AVR ("PTX" for "primary transmitter" with a 10MHz clock/crystal) will send packets of 32 continuous bytes. The second AVR ("PRX" for "primary receiver" with a 8MHz clock/crystal) will receive those packets, invert the bytes (XOR 0xff) and send them back as ACK-payload.  Both AVR will spit out informations on UART0 (38400 8N1 TX-only). The pins used to connect to the nRF-modules are hardcoded (CE=PD5, CSN=PD6, IRQ=PD7), don't change this part in `main.c` or everything will explode... Note that both AVR have different clock frequencies.

## What can i try??
You can change the loglevel of the nRF-modules, simulate lost (ACK- or regular) packets, enable/disable logging to disk, ... You can also mess with other settings and see what happens, expect warnings and/or errors depending on what you do.
