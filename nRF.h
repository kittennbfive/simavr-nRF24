#ifndef __NRF_H__
#define __NRF_H__
#include <stdint.h>
#include <stdbool.h>

#include "sim_avr.h"
#include "sim_irq.h"

#include "nRF_internals.h"
#include "nRF_config.h"

/*
public header for simavr-nRF24

(c) 2022 by kittennbfive

AGPLv3+ and NO WARRANTY!

version 11.05.22 00:54
*/

typedef enum
{
	NRF_LOG_ERROR=0, //always printed
	NRF_LOG_WARNING, //default
	NRF_LOG_VERBOSE,
	NRF_LOG_DEBUG
} nRF_log_level_t;

void nRF_global_init(void);
void nRF_stop_on_error(const bool yesno);
void nRF_log_to_file(nRF_t * const nRF, char const * const filename);
void nRF_set_log_level(const nRF_log_level_t level);
void nRF_set_lost_packets(const uint32_t lost_packets, const uint32_t lost_acks);
nRF_t * make_new_nRF(void);
void nRF_init(struct avr_t * avr, nRF_t * const nRF, char const * const name);
void nRF_connect(nRF_t * const nRF, avr_irq_t * pin_ce_irq, avr_irq_t * pin_irq_irq);
void csn_nRF(void * nRF, uint32_t value);
uint8_t spi_nRF(nRF_t * nRF, const uint8_t rx);
void nRF_cleanup(void);

#endif
