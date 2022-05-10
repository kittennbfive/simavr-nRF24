# simavr-nRF24

## What is this?
This code allows to use/simulate the nRF24L01+ 2,4GHz wireless modules with [simavr](https://github.com/buserror/simavr).

## Licence and disclaimer
AGPLv3+ and NO WARRANTY! The code was quite a challenge to write because the nRF24 are not simple devices (if you look at the internal workings). Some features are still missing and the whole thing should be considered experimental.

## Overview
To use this code in a meaningful way you need to have at least two AVR in your simavr-project. This works perfectly fine even if the AVR have different clocks, see `/example` for a howto. Please note that this code has only been tested for a simple point-to-point link between two AVR, although it should work for more than two AVR/nRF24. The code allows to simulate lost data- or ACK-packets, this is really important because it will happen with real hardware. You can also log the activity of an nRF to disk, although this feature is still incomplete (the plan is to have the same output as for my [gr-nrf24-sniffer](https://github.com/kittennbfive/gr-nrf24-sniffer) that allows snooping on *real* hardware using a SDR and GNU Radio). You can also set different log-levels to see what is happening "inside" the nRF (shown on screen but can be redirected to disk too).

## public API
```
void nRF_global_init(void);
void nRF_stop_on_error(const bool yesno);
void nRF_log_to_file(nRF_t * nRF, char const * const filename);
void nRF_set_log_level(const nRF_log_level_t level);
void nRF_set_lost_packets(const uint32_t lost_packets, const uint32_t lost_acks);
nRF_t * make_new_nRF(void);
void nRF_init(struct avr_t * avr, nRF_t * const nRF, char const * const name);
void nRF_connect(nRF_t * const nRF, avr_irq_t * pin_ce_irq, avr_irq_t * pin_irq_irq);
void csn_nRF(void * nRF, uint32_t value);
uint8_t spi_nRF(nRF_t * nRF, const uint8_t rx);
void nRF_cleanup(void);
```

### nRF_global_init
This function must be called before any other function to set some things up.

### nRF_stop_on_error
This function allows you to tell the code to stop (or not) if an error occurs, like reading/writing an invalid register of an nRF or ... This feature is really useful for complex code, else some error might produce other errors and your screen will be flood with (meaningless) warnings/errors from the simulator and maybe from your own code.

### nRF_log_to_file
You can enable logging of TX-activity to disk for a given nRF (pointer returned by `make_new_nRF()` which must be called before this function).

### nRF_set_log_level
Used to set the verbosity of the code, possible values are NRF_LOG_ERROR, NRF_LOG_WARNING (default), NRF_LOG_VERBOSE (some informations about what is going on), NRF_LOG_DEBUG (*lots* of internal stuff for debugging).

### nRF_set_lost_packets
If you want the code to simulate lost packets call this function before starting the simulation. Approximately one of N ACK- or data-packets will be "lost" for a specified argument of N. Set this to 0 if you want to perfectly stable RF-link without any lost packets (default).

### make_new_nRF
This *creates* a new nRF to be connected to an AVR and returns a pointer to an internal data structure.

### nRF_init
This *initializes* a created nRF and give it a name used for logging on screen (to be able to distinguish betweens multiple nRF).

### nRF_connect
This *connects* an initialized nRF to an AVR. You need to specify the nRF (pointer to the internal opaque data structure as returned by `make_new_nRF()`), the CE-pin-IRQ (used to enable RX/TX) and the IRQ-pin-IRQ (used to signal events from the nRF to the AVR) as returned by `avr_io_getirq()`.

### csn_nRF and spi_nRF
Those are the callbacks you need to provide to the SPI-dispatcher, see documentation there and code in `/example`. It should be possible to use this code without the SPI-dispatcher but you might need to write some glue-code.

### nRF_cleanup
To be called once the simulation has finished, prints some statistics and cleans up some internal stuff.

