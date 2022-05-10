#ifndef __NRF_INTERNALS_H__
#define __NRF_INTERNALS_H__
#include <stdint.h>
#include <stdbool.h>

#include "sim_avr.h"
#include "sim_irq.h"

#include "nRF_config.h"

/*
internal stuff for simavr-nRF24

Do not change anything here!

(c) 2022 by kittennbfive

AGPLv3+ and NO WARRANTY!

version 10.05.22 19:49
*/

#define MS_TO_CYCLES(avr, ms) (avr_cycle_count_t)(((ms)*1E-3)/(1.0/avr->frequency))
#define US_TO_CYCLES(avr, us) (avr_cycle_count_t)(((us)*1E-6)/(1.0/avr->frequency))
#define CYCLES_TO_MS_FLOAT(avr, cycles) ((cycles)*(1.0/avr->frequency)*1E3)

typedef enum
{
	NRF_POWER_DOWN, //0
	NRF_START_UP, //1
	NRF_STANDBY1, //2
	NRF_RX_SETTLING, //3
	NRF_RX_MODE, //4
	NRF_TX_SETTLING, //5
	NRF_TX_MODE, //6
	NRF_STANDBY2, //7
	NRF_RX_SETTLING_FOR_ACK, //8
	NRF_RX_MODE_FOR_ACK, //9
	NRF_TX_SETTLING_FOR_ACK, //10
	NRF_TX_MODE_FOR_ACK //11
} state_nRF_t;

typedef enum
{
	NRF_SPI_IDLE,
	NRF_SPI_READ_REGISTER,
	NRF_SPI_WRITE_REGISTER,
	NRF_SPI_W_TX_PAYLOAD,
	NRF_SPI_R_RX_PAYLOAD,
	NRF_SPI_READ_LENGTH_PAYLOAD,
	NRF_SPI_WRITE_ACK_PAYLOAD
} state_spi_nRF_t;

typedef struct
{
	union
	{
		struct
		{
			uint8_t pipe;
		} ack_packet;  //ACK in PRX-mode

		struct
		{
			uint8_t nb_bytes_addr;
			uint64_t addr;
		} regular_packet; //PTX-mode
	} ;

	uint8_t PID;
	uint8_t nb_bytes;
	uint8_t data[32];
} packet_tx_t;

typedef struct
{
	uint8_t PID;
	uint8_t pipe;
	uint8_t nb_bytes;
	uint8_t data[32];
} packet_rx_t;

struct nRF_struct;

typedef struct nRF_struct
{
	struct avr_t * avr;

	avr_irq_t *	irq;

	char name[NRF_SZ_NAME];

	state_nRF_t state;
	state_nRF_t state_next;

	state_spi_nRF_t state_spi;

	uint8_t spi_reg_index;
	uint64_t spi_value;
	uint8_t spi_length_bytes;
	uint8_t spi_nb_bytes;

	uint8_t PID;

	bool pin_CE;
	bool pin_CSN;
	bool pin_IRQ;

	uint64_t regs[30]; //some regs are 40 bits wide, so make everything 64 bits for simplicity for now...

	packet_tx_t fifo_tx[3];
	uint8_t fifo_tx_entries;

	bool tx_in_progress;
	bool tx_finished;

	bool tx_wait_for_ack;
	bool tx_ack_received;
	bool ard_has_elapsed;
	uint8_t nb_retries;
	bool rx_ack_timeout;
	bool rx_send_ack;
	struct nRF_struct * rx_send_ack_to;
	struct nRF_struct * tx_receive_ack_from;

	packet_rx_t fifo_rx[3];
	uint8_t fifo_rx_entries;
	uint8_t fifo_rx_readpos;

	packet_tx_t packet_being_sent;
	bool packet_being_sent_valid; //contains an actual packet

	packet_rx_t last_rx;
	bool last_rx_valid; //contains an actual packet

	FILE *log;
	bool log_tx_to_file;
	avr_cycle_count_t avr_cycle_last_tx;
} nRF_t;

typedef struct
{
	bool lose_packets;
	uint32_t divider_packets;
	uint32_t nb_lost_packets;

	bool lose_acks;
	uint32_t divider_acks;
	uint32_t nb_lost_acks;
} config_lost_packets_t;


typedef struct
{
	uint32_t nb_packets;
	uint32_t nb_acks;
} packets_stats_t;

#endif
