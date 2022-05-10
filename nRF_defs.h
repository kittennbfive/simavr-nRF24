#ifndef __NRF_DEFS_H__
#define __NRF_DEFS_H__

/*
definitions for nRF24L01+

part of simavr-nRF24

Do not change anything here!

(c) 2022 by kittennbfive

AGPLv3+ and NO WARRANTY!

version 11.05.22 01:02
*/

//COMMANDS
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3 //unimplemented
#define R_RX_PL_WID 0x60
#define W_ACK_PAYLOAD 0xA8
#define W_TX_PAYLOAD_NOACK 0xB0 //unimplemented
#define nRF_NOP 0xFF

//REGISTER
#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_OBSERVE_TX 0x08
#define REG_RPD 0x09
#define REG_RX_ADDR_P0 0x0A
#define REG_RX_ADDR_P1 0x0B
#define REG_RX_ADDR_P2 0x0C
#define REG_RX_ADDR_P3 0x0D
#define REG_RX_ADDR_P4 0x0E
#define REG_RX_ADDR_P5 0x0F
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11
#define REG_RX_PW_P1 0x12
#define REG_RX_PW_P2 0x13
#define REG_RX_PW_P3 0x14
#define REG_RX_PW_P4 0x15
#define REG_RX_PW_P5 0x16
#define REG_FIFO_STATUS 0x17
//some registers are reserved!
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D

//BITS
//REG_CONFIG
#define MASK_RX_DR 6
#define MASK_TX_DS 5
#define MASK_MAX_RT 4
#define EN_CRC 3
#define CRCO 2
#define PWR_UP 1
#define PRIM_RX 0
//EN_AA
#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0
//EN_RXADDR
#define ERX_P5 5
#define ERX_P4 4
#define ERX_P3 3
#define ERX_P2 2
#define ERX_P1 1
#define ERX_P0 0
//SETUP_AW
#define AW 0 //1:0
//SETUP_RETR
#define ARD 4 //7:4
#define ARC 0 //3:0
//RF_SETUP
#define RF_DR_LOW 5 //sic!
#define RF_DR_HIGH 3
#define RF_PWR 1 //2:1
#define LNA_HCURR 0
//STATUS
#define RX_DR 6
#define TX_DS 5
#define MAX_RT 4
#define RX_P_NO 1 //3:1
#define TX_FULL 0
//OBSERVE_TX
#define PLOS_CNT 4 //7:4
#define ARC_CNT 0 //3:0
//FIFO_STATUS
#define FIFO_TX_REUSE 6
#define FIFO_TX_FULL 5 //duplicate with REG_FIFO bit 0 so prefix FIFO_ added
#define FIFO_TX_EMPTY 4
#define FIFO_RX_FULL 1
#define FIFO_RX_EMPTY 0
//DYNPD
#define DPL_P5 5
#define DPL_P4 4
#define DPL_P3 3
#define DPL_P2 2
#define DPL_P1 1
#define DPL_P0 0
//FEATURE
#define EN_DPL 2
#define EN_ACK_PAY 1
#define EN_DYN_ACK 0

#endif
