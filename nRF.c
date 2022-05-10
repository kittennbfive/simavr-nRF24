#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <err.h>
#include <sys/time.h>

#include "nRF.h"
#include "nRF_internals.h"
#include "nRF_defs.h"

#include "sim_avr.h"
#include "avr_spi.h"
#include "avr_ioport.h"
#include "sim_time.h"

/*
simavr-nRF24

This code simulates one (or more, a single one does not make much sense) nRF24L01+. To be used with simavr.

Please read the fine manual.

(c) 2022 by kittennbfive

AGPLv3+ and NO WARRANTY!

version 11.05.22 00:54
*/

static nRF_log_level_t loglevel=NRF_LOG_WARNING;
static bool stop_on_error=false;

#define LOG(level, msg, ...) \
do \
{ \
	if(level==NRF_LOG_ERROR && stop_on_error) \
		errx(1, msg, ##__VA_ARGS__); \
	if(loglevel>=level) \
		printf(msg, ##__VA_ARGS__); \
} while(0)


static nRF_t * modules[NB_NRF_MAX];
static uint8_t nb_modules=0;

static config_lost_packets_t lost;

static packets_stats_t stats;

enum
{
	NRF24_CE_IN=0,
	NRF24_IRQ_OUT,

	NRF24_IRQ_COUNT
};

static const char * irq_names[NRF24_IRQ_COUNT]={
	[NRF24_CE_IN]="nRF_CE",
	[NRF24_IRQ_OUT]="nRF_IRQ"
};

static const uint8_t regs_len_bytes[30]={1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 5, 5, 1, 1, 1, 1, 5, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1};

static void handle_pin_IRQ(nRF_t * nRF);
static void update_nRF(nRF_t * nRF);
static void update_fifo_status(nRF_t * nRF);
static void do_TX(nRF_t * nRF);
static void do_TX_ack(nRF_t * nRF);
static avr_cycle_count_t cb_delay_timer(avr_t * avr, avr_cycle_count_t when, void * param);
static avr_cycle_count_t cb_tx_finished(avr_t * avr, avr_cycle_count_t when, void * param);
static avr_cycle_count_t cb_ard_elapsed(avr_t * avr, avr_cycle_count_t when, void * param);

static void finish_spi(nRF_t * const nRF)
{
	LOG(NRF_LOG_DEBUG, "nRF %s: finish_spi called\n", nRF->name);

	switch(nRF->state_spi)
	{
		case NRF_SPI_IDLE: //nothing to do
			break;

		case NRF_SPI_READ_REGISTER:  //nothing to do
			break;

		case NRF_SPI_WRITE_REGISTER:
			switch(nRF->spi_reg_index)
			{
				case REG_STATUS: //special handling for interrupt flags
					if(nRF->spi_value&(1<<TX_DS))
						nRF->regs[REG_STATUS]&=~(1<<TX_DS);
					if(nRF->spi_value&(1<<RX_DR))
						nRF->regs[REG_STATUS]&=~(1<<RX_DR);
					if(nRF->spi_value&(1<<MAX_RT))
						nRF->regs[REG_STATUS]&=~(1<<MAX_RT);
					break;
				case REG_RF_CH:
					nRF->regs[REG_RF_CH]=nRF->spi_value;
					nRF->regs[REG_OBSERVE_TX]&=~(0b1111<<PLOS_CNT);
					break;
				default:
					nRF->regs[nRF->spi_reg_index]=nRF->spi_value;
					break;
			}
			break;

		case NRF_SPI_W_TX_PAYLOAD:
			nRF->fifo_tx[nRF->fifo_tx_entries].PID=nRF->PID;
			nRF->PID=(nRF->PID+1)&3;
			nRF->fifo_tx_entries++;
			update_fifo_status(nRF);
			nRF->tx_in_progress=false;
			nRF->tx_finished=false;
			nRF->ard_has_elapsed=false;
			nRF->nb_retries=0;
			nRF->rx_ack_timeout=false;
			break;

		case NRF_SPI_R_RX_PAYLOAD:
			memmove(&nRF->fifo_rx[0], &nRF->fifo_rx[1], 2*sizeof(packet_rx_t));
			nRF->fifo_rx_entries--;
			update_fifo_status(nRF);
			break;

		case NRF_SPI_READ_LENGTH_PAYLOAD: //nothing to do
			break;

		case NRF_SPI_WRITE_ACK_PAYLOAD:
			nRF->fifo_tx_entries++;
			update_fifo_status(nRF);
			break;
	}

	nRF->state_spi=NRF_SPI_IDLE;

	update_nRF(nRF);
	handle_pin_IRQ(nRF);
}

static void update_nRF(nRF_t * const nRF)
{
	switch(nRF->state)
	{
		case NRF_POWER_DOWN:
			if(nRF->regs[REG_CONFIG]&(1<<PWR_UP))
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: waking up...\n", nRF->name);
				nRF->state=NRF_START_UP;
				nRF->state_next=NRF_STANDBY1;
				avr_cycle_timer_register(nRF->avr, MS_TO_CYCLES(nRF->avr, 1.5), &cb_delay_timer, nRF);
			}
			break;

		case NRF_START_UP:
			break;

		case NRF_STANDBY1:
			if(!(nRF->regs[REG_CONFIG]&(1<<PWR_UP)))
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going to power down\n", nRF->name);
				nRF->state=NRF_POWER_DOWN;
			}
			else if(!(nRF->regs[REG_CONFIG]&(1<<PRIM_RX)) && nRF->pin_CE && nRF->fifo_tx_entries)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going to TX-mode\n", nRF->name);
				nRF->state=NRF_TX_SETTLING;
				nRF->state_next=NRF_TX_MODE;
				avr_cycle_timer_register(nRF->avr, US_TO_CYCLES(nRF->avr, 10+130), &cb_delay_timer, nRF); //HACK, TODO: check if CE was high for >=10µs
			}
			else if(nRF->regs[REG_CONFIG]&(1<<PRIM_RX) && nRF->pin_CE)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going to RX-mode\n", nRF->name);
				nRF->state=NRF_RX_SETTLING;
				nRF->state_next=NRF_RX_MODE;
				avr_cycle_timer_register(nRF->avr, US_TO_CYCLES(nRF->avr, 130), &cb_delay_timer, nRF);
			}
			else if(!(nRF->regs[REG_CONFIG]&(1<<PRIM_RX)) && nRF->pin_CE && nRF->fifo_tx_entries==0)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: no packets to TX, going into Standby2\n", nRF->name);
				nRF->state=NRF_STANDBY2;
			}
			else
				LOG(NRF_LOG_DEBUG, "nRF %s: no action, remaining in Standby1, CE=%u CSN=%u IRQ=%u\n", nRF->name, nRF->pin_CE, nRF->pin_CSN, nRF->pin_IRQ);
			break;

		case NRF_RX_SETTLING:
			if(!(nRF->regs[REG_CONFIG]&(1<<PWR_UP)))
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going to power down\n", nRF->name);
				nRF->state=NRF_POWER_DOWN;
				nRF->state_next=NRF_POWER_DOWN; //there might be a timer firing that will set state to state_next, so write both
			}
			else if(!nRF->pin_CE)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: RX Settling aborted because CE went low, going into Standby1\n", nRF->name);
				nRF->state=NRF_STANDBY1;
				nRF->state_next=NRF_STANDBY1;
			}
			break;

		case NRF_TX_SETTLING:
			if(!(nRF->regs[REG_CONFIG]&(1<<PWR_UP)))
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going to power down\n", nRF->name);
				nRF->state=NRF_POWER_DOWN;
				nRF->state_next=NRF_POWER_DOWN; //there might be a timer firing that will set state to state_next, so write both
			}
			break;

		case NRF_TX_MODE:
			if(nRF->tx_finished && nRF->tx_wait_for_ack)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going into RX-mode to receive ACK, registering cb_delay_timer\n", nRF->name);
				nRF->tx_finished=false;
				nRF->state=NRF_RX_SETTLING_FOR_ACK;
				nRF->state_next=NRF_RX_MODE_FOR_ACK;
				avr_cycle_timer_register(nRF->avr, US_TO_CYCLES(nRF->avr, 130), &cb_delay_timer, nRF);
			}
			else if(nRF->tx_finished && !nRF->pin_CE)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going to Standby1-mode\n", nRF->name);
				nRF->tx_finished=false;
				nRF->state=NRF_STANDBY1;
			}
			else if(!(nRF->regs[REG_CONFIG]&(1<<PWR_UP)))
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going to power down\n", nRF->name);
				nRF->state=NRF_POWER_DOWN;
			}
			else if(nRF->pin_CE && nRF->fifo_tx_entries==0)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: no packets to TX, going into Standby2\n", nRF->name);
				nRF->state=NRF_STANDBY2;
			}
			break;

		case NRF_RX_MODE:
			if(!nRF->pin_CE)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: leaving RX-mode for Standby1\n", nRF->name);
				nRF->state=NRF_STANDBY1;
			}
			else if(!(nRF->regs[REG_CONFIG]&(1<<PWR_UP)))
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going to power down\n", nRF->name);
				nRF->state=NRF_POWER_DOWN;
			}
			break;

		case NRF_STANDBY2:
			if(!(nRF->regs[REG_CONFIG]&(1<<PWR_UP)))
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going to power down\n", nRF->name);
				nRF->state=NRF_POWER_DOWN;
			}
			else if(nRF->pin_CE && nRF->fifo_tx_entries)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: going to TX-mode\n", nRF->name);
				nRF->state=NRF_TX_SETTLING;
				nRF->state_next=NRF_TX_MODE;
				avr_cycle_timer_register(nRF->avr, US_TO_CYCLES(nRF->avr, 130), &cb_delay_timer, nRF);
			}
			else if(nRF->tx_wait_for_ack)
			{
				if(nRF->ard_has_elapsed)
				{
					nRF->tx_wait_for_ack=false; //prevent cb_delay_timer to register cb_rx_ack_timeout before the new transmission has even started
					LOG(NRF_LOG_VERBOSE, "nRF %s: ARD has elapsed\n", nRF->name);
					if(nRF->nb_retries==(nRF->regs[REG_SETUP_RETR]&(0b1111<<ARC)))
					{
						LOG(NRF_LOG_VERBOSE, "nRF %s: ARC reached, setting MAX_RT, going into Standby1\n", nRF->name);
						nRF->regs[REG_STATUS]|=(1<<MAX_RT);
						handle_pin_IRQ(nRF);
						nRF->state=NRF_STANDBY1;
					}
					else
					{
						LOG(NRF_LOG_VERBOSE, "nRF %s: going into TX to send again\n", nRF->name);
						nRF->nb_retries++;
						nRF->state=NRF_TX_SETTLING;
						nRF->state_next=NRF_TX_MODE;
						avr_cycle_timer_register(nRF->avr, US_TO_CYCLES(nRF->avr, 130), &cb_delay_timer, nRF);
					}
				}
			}
			break;

		case NRF_RX_SETTLING_FOR_ACK: //TODO check CE here?
			break;

		case NRF_RX_MODE_FOR_ACK:
			if(nRF->tx_ack_received)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: ACK received, going into Standby1\n", nRF->name);
				nRF->state=NRF_STANDBY1;
				nRF->tx_wait_for_ack=false;
				nRF->tx_receive_ack_from=NULL;
				nRF->rx_send_ack_to=NULL;
			}
			else if(nRF->rx_ack_timeout)
			{
				LOG(NRF_LOG_VERBOSE, "nRF %s: timeout while waiting for ACK, going into Standby2\n", nRF->name);
				nRF->rx_ack_timeout=false;
				nRF->state=NRF_STANDBY2;
			}
			break;

		case NRF_TX_SETTLING_FOR_ACK: //TODO check CE here?
			break;

		case NRF_TX_MODE_FOR_ACK:
			if(nRF->tx_finished)
			{
				nRF->tx_finished=false;
				if(nRF->pin_CE)
				{
					LOG(NRF_LOG_VERBOSE, "nRF %s: ACK transmitted, going back to RX-mode\n", nRF->name);
					nRF->state=NRF_RX_SETTLING;
					nRF->state_next=NRF_RX_MODE;
					avr_cycle_timer_register(nRF->avr, US_TO_CYCLES(nRF->avr, 130), &cb_delay_timer, nRF);
				}
				else
				{
					LOG(NRF_LOG_VERBOSE, "nRF %s: ACK transmitted, CE is low, going into Standby1\n", nRF->name);
					nRF->state=NRF_STANDBY1;
				}
			}
			break;
	}

	do_TX(nRF);
	do_TX_ack(nRF);
}

static void update_fifo_status(nRF_t * const nRF)
{
	//TODO: STATUS_TX_REUSE unimplemented

	if(nRF->fifo_rx_entries==0)
	{
		nRF->regs[REG_STATUS]&=~(1<<RX_DR);
		nRF->regs[REG_STATUS]|=(0b111<<RX_P_NO); //"RX FIFO empty"
	}
	else
	{
		nRF->regs[REG_STATUS]&=~(0b111<<RX_P_NO);
		nRF->regs[REG_STATUS]|=(1<<RX_DR)|(nRF->fifo_rx[0].pipe<<RX_P_NO);
	}

	if(nRF->fifo_tx_entries==3)
		nRF->regs[REG_STATUS]|=(1<<TX_FULL);
	else
		nRF->regs[REG_STATUS]&=~(1<<TX_FULL);

	switch(nRF->fifo_tx_entries)
	{
		case 0:
			nRF->regs[REG_FIFO_STATUS]|=(1<<FIFO_TX_EMPTY);
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_TX_FULL);
			break;

		case 1:
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_TX_EMPTY);
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_TX_FULL);
			break;

		case 2:
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_TX_EMPTY);
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_TX_FULL);
			break;

		case 3:
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_TX_EMPTY);
			nRF->regs[REG_FIFO_STATUS]|=(1<<FIFO_TX_FULL);
			break;

		default:
			errx(1, "nRF: internal error: update_fifo_status: fifo_tx_entries>3 for nRF %s", nRF->name);
	}

	switch(nRF->fifo_rx_entries)
	{
		case 0:
			nRF->regs[REG_FIFO_STATUS]|=(1<<FIFO_RX_EMPTY);
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_RX_FULL);
			break;

		case 1:
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_RX_EMPTY);
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_RX_FULL);
			break;

		case 2:
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_RX_EMPTY);
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_RX_FULL);
			break;

		case 3:
			nRF->regs[REG_FIFO_STATUS]&=~(1<<FIFO_RX_EMPTY);
			nRF->regs[REG_FIFO_STATUS]|=(1<<FIFO_RX_FULL);
			break;

		default:
			errx(1, "nRF: internal error: update_fifo_status: fifo_rx_entries>3 for nRF %s", nRF->name);
	}
}

static void handle_pin_IRQ(nRF_t * const nRF)
{
	bool IRQ=1;

	if(!(nRF->regs[REG_CONFIG]&(1<<MASK_RX_DR)) && nRF->regs[REG_STATUS]&(1<<RX_DR))
		IRQ=0;
	if(!(nRF->regs[REG_CONFIG]&(1<<MASK_TX_DS)) && nRF->regs[REG_STATUS]&(1<<TX_DS))
		IRQ=0;
	if(!(nRF->regs[REG_CONFIG]&(1<<MASK_MAX_RT)) && nRF->regs[REG_STATUS]&(1<<MAX_RT))
		IRQ=0;

	nRF->pin_IRQ=IRQ;

	LOG(NRF_LOG_DEBUG, "handle_pin_IRQ nRF %s: IRQ set to %u\n", nRF->name, nRF->pin_IRQ);

	avr_raise_irq(nRF->irq+NRF24_IRQ_OUT, nRF->pin_IRQ);
}

static void log_to_file(nRF_t * const nRF, const bool is_ack_packet, const uint8_t bytes_payload) //TODO improve this
{
	if(!is_ack_packet)
		fprintf(nRF->log, "[%10.3fms] [delta %7.3fms] TX %2u bytes\n", CYCLES_TO_MS_FLOAT(nRF->avr, nRF->avr->cycle), CYCLES_TO_MS_FLOAT(nRF->avr, nRF->avr->cycle-nRF->avr_cycle_last_tx), bytes_payload);
	else
		fprintf(nRF->log, "[%10.3fms] [delta %7.3fms] ACK %2u bytes\n", CYCLES_TO_MS_FLOAT(nRF->avr, nRF->avr->cycle), CYCLES_TO_MS_FLOAT(nRF->avr, nRF->avr->cycle-nRF->avr_cycle_last_tx), bytes_payload);

	nRF->avr_cycle_last_tx=nRF->avr->cycle;
}

static void do_TX(nRF_t * const nRF)
{
	if(nRF->state!=NRF_TX_MODE || nRF->tx_in_progress || nRF->state_spi!=NRF_SPI_IDLE)
		return;

	nRF->packet_being_sent.PID=nRF->fifo_tx[0].PID;
	nRF->packet_being_sent.regular_packet.nb_bytes_addr=nRF->fifo_tx[0].regular_packet.nb_bytes_addr;
	nRF->packet_being_sent.regular_packet.addr=nRF->fifo_tx[0].regular_packet.addr;
	nRF->packet_being_sent.nb_bytes=nRF->fifo_tx[0].nb_bytes;
	memcpy(nRF->packet_being_sent.data, nRF->fifo_tx[0].data, nRF->fifo_tx[0].nb_bytes);
	nRF->packet_being_sent_valid=true;

	uint8_t bytes_addr=nRF->packet_being_sent.regular_packet.nb_bytes_addr;
	uint8_t bytes_payload=nRF->packet_being_sent.nb_bytes;
	uint8_t bytes_crc=(nRF->regs[REG_CONFIG]&(1<<CRCO))?2:1;
	uint32_t data_rate=(nRF->regs[REG_RF_SETUP]&(1<<RF_DR_LOW))?250E3:((nRF->regs[REG_RF_SETUP]&(1<<RF_DR_HIGH))?2E6:1E6);
	uint32_t time_on_air_us=1.0E6*(8*(1+bytes_addr+bytes_payload+bytes_crc)+9)/data_rate;

	LOG(NRF_LOG_VERBOSE, "nRF %s: transmitting %u bytes of payload, time on air is %u µs\n", nRF->name, bytes_payload, time_on_air_us);

	if(nRF->log_tx_to_file)
	{
		LOG(NRF_LOG_VERBOSE, "nRF %s: logging TX to file\n", nRF->name);
		log_to_file(nRF, false, bytes_payload);
	}

	avr_cycle_timer_register(nRF->avr, US_TO_CYCLES(nRF->avr, time_on_air_us), &cb_tx_finished, nRF);

	nRF->tx_in_progress=true;
}

static void do_TX_ack(nRF_t * const nRF)
{
	if(nRF->state!=NRF_TX_MODE_FOR_ACK || nRF->tx_in_progress || nRF->state_spi!=NRF_SPI_IDLE)
		return;

	if(!nRF->last_rx_valid)
		errx(1, "nRF: internal error: do_TX_ack: last_rx_valid==false for nRF %s", nRF->name);

	if(nRF->regs[REG_FEATURE]&(1<<EN_ACK_PAY) && nRF->fifo_tx_entries)
	{
		LOG(NRF_LOG_DEBUG, "nRF %s: EN_ACK_PAY enabled, pending ACK-payload will be sent\n", nRF->name);

		uint8_t i;
		bool found=false;
		for(i=0; i<nRF->fifo_tx_entries; i++)
		{
			if(nRF->fifo_tx[i].ack_packet.pipe==nRF->last_rx.pipe)
			{
				found=true;
				break;
			}
		}

		if(!found)
			errx(1, "nRF: internal error: do_TX_ack: not found for nRF %s", nRF->name);

		nRF->packet_being_sent.ack_packet.pipe=nRF->fifo_tx[i].ack_packet.pipe;
		nRF->packet_being_sent.nb_bytes=nRF->fifo_tx[i].nb_bytes;
		memcpy(nRF->packet_being_sent.data, nRF->fifo_tx[i].data, nRF->fifo_tx[i].nb_bytes);
		nRF->packet_being_sent_valid=true;

		if(i!=2)
			memmove(&nRF->fifo_tx[i], &nRF->fifo_tx[i+1], (2-i)*sizeof(packet_tx_t));
		nRF->fifo_tx_entries--;
		update_fifo_status(nRF);

	}
	else
	{
		LOG(NRF_LOG_DEBUG, "nRF %s: EN_ACK_PAY not enabled or no pending ACK-payload, sending empty ACK\n", nRF->name);
		nRF->packet_being_sent.ack_packet.pipe=nRF->last_rx.pipe;
		nRF->packet_being_sent.nb_bytes=0;
		nRF->packet_being_sent_valid=true;
	}

	uint8_t bytes_addr=(nRF->regs[REG_SETUP_AW]&(0b11<<AW))+2;
	uint8_t bytes_payload=nRF->packet_being_sent.nb_bytes;
	uint8_t bytes_crc=(nRF->regs[REG_CONFIG]&(1<<CRCO))?2:1;
	uint32_t data_rate=(nRF->regs[REG_RF_SETUP]&(1<<RF_DR_LOW))?250E3:((nRF->regs[REG_RF_SETUP]&(1<<RF_DR_HIGH))?2E6:1E6);
	uint32_t time_on_air_us=1.0E6*(8*(1+bytes_addr+bytes_payload+bytes_crc)+9)/data_rate;

	LOG(NRF_LOG_VERBOSE, "nRF %s: transmitting ACK with %u bytes payload to %s, time on air is %u µs\n", nRF->name, bytes_payload, nRF->rx_send_ack_to->name, time_on_air_us);

	if(nRF->log_tx_to_file)
	{
		LOG(NRF_LOG_VERBOSE, "nRF %s: logging TX ACK to file\n", nRF->name);
		log_to_file(nRF, true, bytes_payload);
	}

	avr_cycle_timer_register(nRF->avr, US_TO_CYCLES(nRF->avr, time_on_air_us), &cb_tx_finished, nRF);

	nRF->tx_in_progress=true;
}

static void handle_tx_ack(nRF_t * const nRF_PTX, nRF_t * const nRF_PRX)
{
	LOG(NRF_LOG_DEBUG, "handle_tx_ack: setting PRX to TX-settling, registering timer cb_delay_timer\n");

	nRF_PTX->tx_receive_ack_from=nRF_PRX;

	nRF_PRX->state=NRF_TX_SETTLING_FOR_ACK;
	nRF_PRX->state_next=NRF_TX_MODE_FOR_ACK;
	nRF_PRX->rx_send_ack=true;
	nRF_PRX->rx_send_ack_to=nRF_PTX;

	avr_cycle_timer_register(nRF_PRX->avr, US_TO_CYCLES(nRF_PRX->avr, 130), &cb_delay_timer, nRF_PRX);
}

static void dispatch_sent_packet(nRF_t * const nRF)
{
	LOG(NRF_LOG_DEBUG, "dispatch_sent_packet: searching for receiver for packet from %s\n", nRF->name);

	uint8_t i;
	bool found=false;
	for(i=0; i<nb_modules; i++)
	{
		if(modules[i]==nRF)
			continue;

		if(
				modules[i]->state==NRF_RX_MODE //module is in RX mode
			&&	modules[i]->regs[REG_RF_CH]==nRF->regs[REG_RF_CH] //module is listening on same channel
			&&	(modules[i]->regs[REG_RF_SETUP]&0b00101000)==(nRF->regs[REG_RF_SETUP]&0b00101000) //module is using same speed
			&&	(modules[i]->regs[REG_CONFIG]&(1<<CRCO))==(nRF->regs[REG_CONFIG]&(1<<CRCO)) //module is using same CRC-setting
		  )
		{
			uint64_t nb_bytes_addr=(modules[i]->regs[REG_SETUP_AW]&(0b11<<AW))+2;
			uint64_t addr_mask=(1UL<<(8*nb_bytes_addr))-1;
			uint64_t addr_pipes[5];
			addr_pipes[0]=(modules[i]->regs[REG_RX_ADDR_P0])&addr_mask;
			addr_pipes[1]=(modules[i]->regs[REG_RX_ADDR_P1])&addr_mask;
			addr_pipes[2]=((addr_pipes[1]&0xffffffff00)|modules[i]->regs[REG_RX_ADDR_P2])&addr_mask;
			addr_pipes[3]=((addr_pipes[1]&0xffffffff00)|modules[i]->regs[REG_RX_ADDR_P3])&addr_mask;
			addr_pipes[4]=((addr_pipes[1]&0xffffffff00)|modules[i]->regs[REG_RX_ADDR_P4])&addr_mask;
			addr_pipes[5]=((addr_pipes[1]&0xffffffff00)|modules[i]->regs[REG_RX_ADDR_P5])&addr_mask;
			uint8_t pipe;

			for(pipe=0; pipe<5; pipe++)
			{
				//addr match for some pipe and this pipe enabled on RX side?
				if(nRF->packet_being_sent.regular_packet.addr==addr_pipes[pipe] && modules[i]->regs[REG_EN_RXADDR]&(1<<pipe))
				{
					found=true;
					break;
				}
			}

			if(found)
			{
				bool discard_packet=false;

				if(modules[i]->last_rx_valid && modules[i]->last_rx.PID==nRF->packet_being_sent.PID && modules[i]->last_rx.nb_bytes==nRF->packet_being_sent.nb_bytes && modules[i]->last_rx.pipe==pipe && !memcmp(modules[i]->last_rx.data, nRF->packet_being_sent.data, nRF->packet_being_sent.nb_bytes))
				{
					LOG(NRF_LOG_VERBOSE, "nRF %s: dropping duplicate packet with %u bytes payload\n", modules[i]->name, nRF->packet_being_sent.nb_bytes);
					discard_packet=true;
				}

				if(modules[i]->fifo_rx_entries<3)
				{
					if(!discard_packet)
					{
						modules[i]->fifo_rx[modules[i]->fifo_rx_entries].PID=nRF->packet_being_sent.PID;
						modules[i]->fifo_rx[modules[i]->fifo_rx_entries].pipe=pipe;
						modules[i]->fifo_rx[modules[i]->fifo_rx_entries].nb_bytes=nRF->packet_being_sent.nb_bytes;
						memcpy(modules[i]->fifo_rx[modules[i]->fifo_rx_entries].data, nRF->packet_being_sent.data, nRF->packet_being_sent.nb_bytes);
						modules[i]->fifo_rx_entries++;

						modules[i]->last_rx.PID=nRF->packet_being_sent.PID;
						modules[i]->last_rx.pipe=pipe;
						modules[i]->last_rx.nb_bytes=nRF->packet_being_sent.nb_bytes;
						memcpy(modules[i]->last_rx.data, nRF->packet_being_sent.data, nRF->packet_being_sent.nb_bytes);
						modules[i]->last_rx_valid=true;

						modules[i]->regs[REG_STATUS]|=(1<<RX_DR);
						update_fifo_status(modules[i]);
						LOG(NRF_LOG_DEBUG, "nRF %s has a new packet, fifo_rx_entries is %u\n", modules[i]->name, modules[i]->fifo_rx_entries);
					}

					if(modules[i]->regs[REG_EN_AA]&(1<<pipe))
					{
						if(lost.lose_acks && (rand()%lost.divider_acks)==0)
						{
							lost.nb_lost_acks++;
							LOG(NRF_LOG_VERBOSE, "nRF %s: simulating lost ACK-packet, total %u lost\n", nRF->name, lost.nb_lost_acks);
						}
						else
							handle_tx_ack(nRF, modules[i]);
					}
					else
						LOG(NRF_LOG_WARNING, "WARNING: auto-ACK disabled for pipe %u on %s, not sending ACK\n", pipe, modules[i]->name);
				}
				else
					LOG(NRF_LOG_WARNING, "WARNING: nRF %s has no free RX-slot and will miss a packet send by nRF %s\n", modules[i]->name, nRF->name);
			}
		}
	}
	if(!found)
		LOG(NRF_LOG_WARNING, "WARNING: no receiver found for packet from nRF %s\n", nRF->name);

}

static void cb_ce(struct avr_irq_t * irq, uint32_t value, void * param) //RX/TX-enable
{
	(void)irq;

	nRF_t * nRF=(nRF_t*)param;

	nRF->pin_CE=value;
	update_nRF(nRF);
}

static avr_cycle_count_t cb_tx_finished(avr_t * avr, avr_cycle_count_t when, void * param)
{
	(void)avr;
	(void)when;

	nRF_t * nRF=(nRF_t*)param;

	LOG(NRF_LOG_DEBUG, "cb_tx_finished called for nRF %s in state %u\n", nRF->name, nRF->state);

	if(!nRF->packet_being_sent_valid)
		errx(1, "nRF: internal error: cb_tx_finished: packet_being_sent_valid==false for nRF %s", nRF->name);

	nRF->tx_in_progress=false;

	if(nRF->rx_send_ack) //is this an ACK-packet from a PRX?
	{
		nRF->tx_finished=true;

		LOG(NRF_LOG_DEBUG, "cb_tx_finished: this is an ACK-packet from PRX\n");

		if(nRF->rx_send_ack_to->ard_has_elapsed)
		{
			LOG(NRF_LOG_WARNING, "WARNING: nRF %s timed-out while receiving ACK from %s - did you set ARD correctly?\n", nRF->rx_send_ack_to->name, nRF->name);
			nRF->rx_send_ack=false;
			update_nRF(nRF->rx_send_ack_to);
			return 0;
		}

		if(nRF->rx_send_ack_to->state!=NRF_RX_MODE_FOR_ACK)
		{
			LOG(NRF_LOG_WARNING, "WARNING: nRF %s is not in RX-mode (but mode %u) and will miss the ACK from %s - did you set ARD correctly?\n", nRF->rx_send_ack_to->name, nRF->rx_send_ack_to->state, nRF->name);
			return 0;
		}

		nRF->rx_send_ack_to->tx_ack_received=true;
		nRF->rx_send_ack_to->regs[REG_STATUS]&=~(1<<TX_FULL);
		nRF->rx_send_ack_to->regs[REG_STATUS]|=(1<<TX_DS);
		update_fifo_status(nRF);
		handle_pin_IRQ(nRF->rx_send_ack_to);
		nRF->regs[REG_STATUS]|=(1<<RX_DR);
		handle_pin_IRQ(nRF);
		stats.nb_acks++;

		if(nRF->packet_being_sent.nb_bytes)
		{
			LOG(NRF_LOG_DEBUG, "ACK has payload\n");

			if(nRF->rx_send_ack_to->fifo_rx_entries==3) //TODO confirm with datasheet how to behave
				LOG(NRF_LOG_WARNING, "WARNING: nRF %s: no free space in RX fifo for ACK-packet payload, data is lost\n", nRF->rx_send_ack_to->name);
			else
			{
				nRF->rx_send_ack_to->fifo_rx[nRF->rx_send_ack_to->fifo_rx_entries].pipe=nRF->packet_being_sent.ack_packet.pipe;
				nRF->rx_send_ack_to->fifo_rx[nRF->rx_send_ack_to->fifo_rx_entries].nb_bytes=nRF->packet_being_sent.nb_bytes;
				memcpy(nRF->rx_send_ack_to->fifo_rx[nRF->rx_send_ack_to->fifo_rx_entries].data, nRF->packet_being_sent.data, nRF->packet_being_sent.nb_bytes);
				nRF->rx_send_ack_to->fifo_rx_entries++;
				update_fifo_status(nRF->rx_send_ack_to);
				nRF->rx_send_ack_to->regs[REG_STATUS]|=(1<<TX_DS)|(1<<RX_DR);
				handle_pin_IRQ(nRF->rx_send_ack_to);

			}
		}

		nRF->packet_being_sent_valid=false;

		LOG(NRF_LOG_DEBUG, "cb_tx_finished: ACK-received, removing packet from TX-fifo\n");
		memmove(&nRF->rx_send_ack_to->fifo_tx[0], &nRF->rx_send_ack_to->fifo_tx[1], 2*sizeof(packet_tx_t));
		nRF->rx_send_ack_to->fifo_tx_entries--;
		nRF->rx_send_ack_to->regs[REG_STATUS]|=(1<<TX_DS);
		update_fifo_status(nRF->rx_send_ack_to);
		handle_pin_IRQ(nRF->rx_send_ack_to);
		stats.nb_packets++;

	}
	else //no, this is a regular packet from a PTX
	{
		LOG(NRF_LOG_DEBUG, "cb_tx_finished: this is a regular packet\n");

		if(lost.lose_packets && (rand()%lost.divider_packets)==0)
		{
			lost.nb_lost_packets++;
			LOG(NRF_LOG_VERBOSE, "nRF %s: simulating lost packet, total %u lost\n", nRF->name, lost.nb_lost_packets);
		}
		else
			dispatch_sent_packet(nRF);

		nRF->packet_being_sent_valid=false;

		//signal to state machine
		nRF->tx_finished=true;
		nRF->tx_in_progress=false;

		if(nRF->regs[REG_SETUP_RETR]&(0b1111<<ARC)) //is auto-retransmit enabled? -> wait for ACK
		{
			nRF->tx_wait_for_ack=true;
			nRF->tx_ack_received=false;
			nRF->rx_ack_timeout=false;
			nRF->ard_has_elapsed=false;

			uint16_t auto_retransmit_delay_us=((nRF->regs[REG_SETUP_RETR]>>ARD)+1)*250;
			avr_cycle_timer_register(nRF->avr, US_TO_CYCLES(nRF->avr, auto_retransmit_delay_us), &cb_ard_elapsed, nRF);

			LOG(NRF_LOG_DEBUG, "cb_tx_finished: we need to wait for ACK, setting variables, registering timer cb_ard_elapsed for ARD %u µs\n", auto_retransmit_delay_us);
		}
		else //we are done with this packet
		{
			LOG(NRF_LOG_DEBUG, "cb_tx_finished: we are done with this packet, removing from TX-fifo\n");
			//remove sent entry from FIFO
			memmove(&nRF->fifo_tx[0], &nRF->fifo_tx[1], 2*sizeof(packet_tx_t));
			nRF->fifo_tx_entries--;
			nRF->regs[REG_STATUS]|=(1<<TX_DS);
			update_fifo_status(nRF);
			handle_pin_IRQ(nRF);
			stats.nb_packets++;
		}
	}

	LOG(NRF_LOG_DEBUG, "cb_tx_finished: calling update_nRF for %s\n", nRF->name);
	update_nRF(nRF);

	LOG(NRF_LOG_DEBUG, "end of cb_tx_finished\n");

	return 0; //stop timer
}

static avr_cycle_count_t cb_rx_ack_timeout(avr_t * avr, avr_cycle_count_t when, void * param)
{
	(void)avr;
	(void)when;

	LOG(NRF_LOG_DEBUG, "cb_rx_ack_timeout fired\n");

	nRF_t * nRF=(nRF_t*)param;

	if(nRF->tx_receive_ack_from)
		LOG(NRF_LOG_DEBUG, "cb_rx_ack_timeout: tx_in_progress for %s is %u\n", nRF->tx_receive_ack_from->name, nRF->tx_receive_ack_from->tx_in_progress);
	else
		LOG(NRF_LOG_DEBUG, "cb_rx_ack_timeout: nRF->tx_receive_ack_from is NULL for nRF %s\n", nRF->name);

	if(nRF->ard_has_elapsed)
	{
		LOG(NRF_LOG_DEBUG, "cb_rx_ack_timeout: ard has elapsed is true, setting rx_ack_timeout\n");
		nRF->rx_ack_timeout=true;
	}
	else if(!nRF->tx_receive_ack_from || !nRF->tx_receive_ack_from->tx_in_progress)
	{
		LOG(NRF_LOG_DEBUG, "cb_rx_ack_timeout: nRF->rx_ack_timeout set to true\n");
		nRF->rx_ack_timeout=true;
	}
	else
		LOG(NRF_LOG_DEBUG, "cb_rx_ack_timeout: tx is in progress, not setting timeout\n");

	//two times in case ARD is too small and ARC has been reached to allow to go into Standby2 and then set MAX_RT and go into Standby1 - a bit ugly but yeah...
	update_nRF(nRF);
	update_nRF(nRF);

	return 0;
}

static avr_cycle_count_t cb_ard_elapsed(avr_t * avr, avr_cycle_count_t when, void * param)
{
	(void)avr;
	(void)when;

	LOG(NRF_LOG_DEBUG, "cb_ard_elapsed fired\n");

	nRF_t * nRF=(nRF_t*)param;

	nRF->ard_has_elapsed=true;

	if(nRF->tx_receive_ack_from && nRF->tx_receive_ack_from->tx_in_progress)
	{
		nRF->tx_receive_ack_from->tx_in_progress=false;
		nRF->rx_ack_timeout=true;
		LOG(NRF_LOG_WARNING, "WARNING: ARD for nRF %s elapsed while nRF %s was still transmitting, ACK is lost\n", nRF->name, nRF->tx_receive_ack_from->name);
	}

	update_nRF(nRF);

	return 0;
}

static avr_cycle_count_t cb_delay_timer(avr_t * avr, avr_cycle_count_t when, void * param)
{
	(void)avr;
	(void)when;

	nRF_t * nRF=(nRF_t*)param;

	LOG(NRF_LOG_DEBUG, "cb_delay_timer fired for %s, old state was %u, new is %u\n", nRF->name, nRF->state, nRF->state_next);

	nRF->state=nRF->state_next;

	if(nRF->tx_wait_for_ack) //PTX waiting for ack
	{
		LOG(NRF_LOG_DEBUG, "cb_delay_timer: registering timer cb_rx_ack_timeout 250µs for %s\n", nRF->name);
		avr_cycle_timer_register(nRF->avr, US_TO_CYCLES(nRF->avr, 250), &cb_rx_ack_timeout, nRF); //see footnote datasheet p. 59
	}

	update_nRF(nRF);

	return 0; //stop timer
}

////////////////////////////////////////////////////////////////////////

//public functions

void nRF_global_init(void)
{
	lost.lose_packets=false;
	lost.lose_acks=false;
	lost.nb_lost_packets=0;
	lost.nb_lost_acks=0;

	stats.nb_packets=0;
	stats.nb_acks=0;
}

void nRF_stop_on_error(const bool yesno)
{
	stop_on_error=yesno;
}

void nRF_set_log_level(const nRF_log_level_t level)
{
	loglevel=level;
}

void nRF_log_to_file(nRF_t * const nRF, char const * const filename)
{
	nRF->log=fopen(filename, "w");
	if(nRF->log==NULL)
		err(1, "nRF %s: creating logfile %s failed", nRF->name, filename);
	nRF->log_tx_to_file=true;

	fprintf(nRF->log, "LOGFILE FOR nRF %s\n", nRF->name);

	printf("nRF %s: logging enabled\n", nRF->name);
}

void nRF_set_lost_packets(const uint32_t lost_packets, const uint32_t lost_acks)
{
	if(lost_packets)
	{
		lost.lose_packets=true;
		lost.divider_packets=lost_packets;
		printf("nRF: simulating 1 lost packet for %u packets sent\n", lost_packets);
	}

	if(lost_acks)
	{
		lost.lose_acks=true;
		lost.divider_acks=lost_acks;
		printf("nRF: simulating 1 lost ACK-packet for %u ACK-packets sent\n", lost_acks);
	}
}

nRF_t * make_new_nRF(void)
{
	if(nb_modules==NB_NRF_MAX)
		errx(1, "make_new_nRF: no more space in modules[], increase NB_NRF_MAX");

	nRF_t * ptr=malloc(sizeof(nRF_t));

	modules[nb_modules++]=ptr;

	return ptr;
}

void nRF_init(struct avr_t * avr, nRF_t * const nRF, char const * const name)
{
	nRF->avr=avr;

	nRF->irq=avr_alloc_irq(&avr->irq_pool, 0, NRF24_IRQ_COUNT, irq_names);

	avr_irq_register_notify(nRF->irq+NRF24_CE_IN, &cb_ce, nRF);

	strncpy(nRF->name, name, NRF_SZ_NAME);

	nRF->pin_CSN=1;
	nRF->pin_CE=0;
	nRF->pin_IRQ=1;

	//default values taken from datasheet
	nRF->regs[REG_CONFIG]=(1<<EN_CRC);
	nRF->regs[REG_EN_AA]=(1<<ENAA_P0)|(1<<ENAA_P1)|(1<<ENAA_P2)|(1<<ENAA_P3)|(1<<ENAA_P4)|(1<<ENAA_P5);
	nRF->regs[REG_EN_RXADDR]=(1<<ERX_P0)|(1<<ERX_P1);
	nRF->regs[REG_SETUP_AW]=(0b11<<AW);
	nRF->regs[REG_SETUP_RETR]=(0b11<<ARC);
	nRF->regs[REG_RF_CH]=2;
	nRF->regs[REG_RF_SETUP]=(1<<RF_DR_HIGH)|(0b11<<RF_PWR);
	nRF->regs[REG_STATUS]=(0b111<<RX_P_NO);
	nRF->regs[REG_OBSERVE_TX]=0;
	nRF->regs[REG_RPD]=0;
	nRF->regs[REG_RX_ADDR_P0]=0xe7e7e7e7e7;
	nRF->regs[REG_RX_ADDR_P1]=0xc2c2c2c2c2;
	nRF->regs[REG_RX_ADDR_P2]=0xc3;
	nRF->regs[REG_RX_ADDR_P3]=0xc4;
	nRF->regs[REG_RX_ADDR_P4]=0xc5;
	nRF->regs[REG_RX_ADDR_P5]=0xc6;
	nRF->regs[REG_TX_ADDR]=0xe7e7e7e7e7;
	nRF->regs[REG_RX_PW_P0]=0;
	nRF->regs[REG_RX_PW_P1]=0;
	nRF->regs[REG_RX_PW_P2]=0;
	nRF->regs[REG_RX_PW_P3]=0;
	nRF->regs[REG_RX_PW_P4]=0;
	nRF->regs[REG_RX_PW_P5]=0;
	nRF->regs[REG_FIFO_STATUS]=(1<<FIFO_TX_EMPTY)|(1<<FIFO_RX_EMPTY);
	nRF->regs[REG_DYNPD]=0; //TODO: not checked by code
	nRF->regs[REG_FEATURE]=0; //TODO: only partially checked by code

	nRF->state=NRF_POWER_DOWN;

	nRF->PID=0;

	nRF->fifo_rx_entries=0;

	nRF->fifo_tx_entries=0;

	nRF->tx_in_progress=false;

	nRF->tx_finished=false;

	nRF->tx_wait_for_ack=false;

	nRF->tx_ack_received=false;

	nRF->rx_send_ack=false;

	nRF->rx_send_ack_to=NULL;

	nRF->tx_receive_ack_from=NULL;

	nRF->last_rx_valid=false;

	nRF->packet_being_sent_valid=false;

	nRF->log=NULL;
	nRF->log_tx_to_file=false;
	nRF->avr_cycle_last_tx=0;
}

void nRF_connect(nRF_t * const nRF, avr_irq_t * pin_ce_irq, avr_irq_t * pin_irq_irq)
{
	avr_connect_irq(pin_ce_irq, nRF->irq+NRF24_CE_IN);
	avr_connect_irq(nRF->irq+NRF24_IRQ_OUT, pin_irq_irq);

	avr_raise_irq(nRF->irq+NRF24_IRQ_OUT, 1);
}

void csn_nRF(void * nRF, uint32_t value) //SPI
{
	((nRF_t*)nRF)->pin_CSN=value;
	if(((nRF_t*)nRF)->pin_CSN==1)
		finish_spi((nRF_t*)nRF);
}

uint8_t spi_nRF(nRF_t * nRF, const uint8_t rx)
{
	uint8_t ret;

	switch(nRF->state_spi)
	{
		case NRF_SPI_IDLE:
			ret=nRF->regs[REG_STATUS];

			if(rx==nRF_NOP) //nop, send status
				;
			else if((rx&0xe0)==0) //R_REGISTER
			{
				uint8_t reg_to_read=rx&0x1f;
				nRF->spi_reg_index=reg_to_read;
				nRF->spi_value=nRF->regs[reg_to_read];
				nRF->spi_nb_bytes=0;
				nRF->spi_length_bytes=regs_len_bytes[reg_to_read];
				if(nRF->spi_length_bytes==0)
					LOG(NRF_LOG_ERROR, "ERROR: nRF %s: tried to read inexistent register 0x%02x\n", nRF->name, reg_to_read);
				nRF->state_spi=NRF_SPI_READ_REGISTER;
			}
			else if((rx&0xe0)==0x20) //W_REGISTER
			{
				uint8_t reg_to_write=rx&0x1f;
				nRF->spi_reg_index=reg_to_write;
				nRF->spi_nb_bytes=0;
				nRF->spi_value=0;
				nRF->spi_length_bytes=regs_len_bytes[reg_to_write];
				if(nRF->spi_length_bytes==0)
					LOG(NRF_LOG_ERROR, "ERROR: nRF %s: tried to write to inexistent register 0x%02x\n", nRF->name, reg_to_write);
				nRF->state_spi=NRF_SPI_WRITE_REGISTER;
			}
			else if(rx==R_RX_PAYLOAD)
			{
				LOG(NRF_LOG_DEBUG, "nRF %s: command R_RX_PAYLOAD\n", nRF->name);
				if(nRF->fifo_rx_entries==0)
					LOG(NRF_LOG_ERROR, "ERROR: nRF %s: no entries in RX fifo\n", nRF->name);
				nRF->fifo_rx_readpos=0;
				nRF->state_spi=NRF_SPI_R_RX_PAYLOAD;
			}
			else if(rx==W_TX_PAYLOAD)
			{
				LOG(NRF_LOG_DEBUG, "nRF %s: command W_TX_PAYLOAD\n", nRF->name);
				if(nRF->fifo_tx_entries==3)
				{
					LOG(NRF_LOG_ERROR, "ERROR: nRF %s: no space in TX fifo\n", nRF->name);
					return ret;
				}
				nRF->fifo_tx[nRF->fifo_tx_entries].regular_packet.nb_bytes_addr=(nRF->regs[REG_SETUP_AW]&(0b11<<AW))+2;
				nRF->fifo_tx[nRF->fifo_tx_entries].regular_packet.addr=nRF->regs[REG_TX_ADDR];
				nRF->fifo_tx[nRF->fifo_tx_entries].nb_bytes=0;
				nRF->state_spi=NRF_SPI_W_TX_PAYLOAD;
			}
			else if(rx==FLUSH_TX)
			{
				LOG(NRF_LOG_DEBUG, "nRF %s: flush TX\n", nRF->name);
				nRF->fifo_tx_entries=0;
				update_fifo_status(nRF);
			}
			else if(rx==FLUSH_RX)
			{
				LOG(NRF_LOG_DEBUG, "nRF %s: flush RX\n", nRF->name);
				nRF->fifo_rx_entries=0;
				nRF->regs[REG_STATUS]|=(0b111<<RX_P_NO); //RX FIFO empty
				update_fifo_status(nRF);
			}
			else if(rx==REUSE_TX_PL)
				errx(1, "nRF: unimplemented command: REUSE_TX_PL");
			else if(rx==R_RX_PL_WID)
			{
				//LOG(NRF_LOG_DEBUG, "nRF %s: command R_RX_PL_WID\n", nRF->name); //if polling is used this will flood the screen...
				nRF->state_spi=NRF_SPI_READ_LENGTH_PAYLOAD;
			}
			else if((rx&0xf8)==W_ACK_PAYLOAD)
			{
				uint8_t pipe=rx&0x07;
				LOG(NRF_LOG_DEBUG, "nRF %s: command W_ACK_PAYLOAD pipe %u\n", nRF->name, pipe);
				if(nRF->fifo_tx_entries==3)
				{
					LOG(NRF_LOG_ERROR, "ERROR: nRF %s: no space for ACK in TX fifo\n", nRF->name);
					return ret;
				}
				nRF->fifo_tx[nRF->fifo_tx_entries].ack_packet.pipe=pipe;
				nRF->state_spi=NRF_SPI_WRITE_ACK_PAYLOAD;
			}
			else if(rx==W_TX_PAYLOAD_NOACK) //TODO
				errx(1, "nRF: unimplemented command: W_TX_PAYLOAD_NOACK");
			else
				LOG(NRF_LOG_ERROR, "ERROR: nRF %s: unknown command 0x%02x\n", nRF->name, rx);
			break;

		case NRF_SPI_READ_REGISTER:
			if(nRF->spi_nb_bytes < nRF->spi_length_bytes)
				ret=(nRF->spi_value>>(8*nRF->spi_nb_bytes++))&0xff;
			else
			{
				LOG(NRF_LOG_WARNING, "WARNING: nRF %s: tried to read more bytes than available from register 0x%02x, returning 0xff\n", nRF->name, nRF->spi_reg_index);
				ret=0xff;
			}
			break;

		case NRF_SPI_WRITE_REGISTER:
			ret=0xff;
			if(nRF->spi_nb_bytes < nRF->spi_length_bytes)
				nRF->spi_value|=(uint64_t)rx<<(8*nRF->spi_nb_bytes++);
			else
				LOG(NRF_LOG_WARNING, "WARNING: nRF %s: tried to write more bytes than possible to register 0x%02x, ignoring\n", nRF->name, nRF->spi_reg_index);
			break;

		case NRF_SPI_W_TX_PAYLOAD:
			ret=0xff;
			if(nRF->fifo_tx[nRF->fifo_tx_entries].nb_bytes==32)
			{
				LOG(NRF_LOG_ERROR, "ERROR: nRF %s: TX fifo overflow, tried to write more than 32 bytes\n", nRF->name);
				return ret;
			}
			nRF->fifo_tx[nRF->fifo_tx_entries].data[nRF->fifo_tx[nRF->fifo_tx_entries].nb_bytes++]=rx;
			break;

		case NRF_SPI_R_RX_PAYLOAD:
			if(nRF->fifo_rx_readpos==nRF->fifo_rx[0].nb_bytes)
			{
				LOG(NRF_LOG_ERROR, "ERROR: nRF %s: no more bytes in RX fifo\n", nRF->name);
				return 0xff;
			}
			ret=nRF->fifo_rx[0].data[nRF->fifo_rx_readpos++];
			break;

		case NRF_SPI_READ_LENGTH_PAYLOAD:
			if(nRF->fifo_rx_entries==0)
				ret=0;
			else
			{
				LOG(NRF_LOG_DEBUG, "nRF %s: payload %u bytes\n", nRF->name, nRF->fifo_rx[0].nb_bytes);
				ret=nRF->fifo_rx[0].nb_bytes;
			}
			break;

		case NRF_SPI_WRITE_ACK_PAYLOAD:
			ret=0xff;
			if(nRF->fifo_tx[nRF->fifo_tx_entries].nb_bytes==32)
			{
				LOG(NRF_LOG_ERROR, "ERROR: nRF %s: fifo ACK payload overflow, tried to write more than 32 bytes\n", nRF->name);
				return ret;
			}
			nRF->fifo_tx[nRF->fifo_tx_entries].data[nRF->fifo_tx[nRF->fifo_tx_entries].nb_bytes++]=rx;
			LOG(NRF_LOG_DEBUG, "nRF %s: SPI_WRITE_ACK_PAYLOAD %u bytes written, last was 0x%02x\n", nRF->name, nRF->fifo_tx[nRF->fifo_tx_entries].nb_bytes, rx);
			break;
	}

	return ret;
}

void nRF_cleanup(void)
{
	printf("nRF: simulated loss of %u packets and %u ACK-packets\n", lost.nb_lost_packets, lost.nb_lost_acks);
	printf("nRF: %u packets and %u ACK-packets successfully transmitted\n", stats.nb_packets, stats.nb_acks);

	uint8_t i;
	for(i=0; i<nb_modules; i++)
	{
		if(modules[i]->log)
			fclose(modules[i]->log);
		free(modules[i]);
	}
}
