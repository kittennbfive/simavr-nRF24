#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <stdbool.h>
#include <time.h>

#include "sim_avr.h"
#include "sim_elf.h"
#include "avr_spi.h"
#include "avr_ioport.h"

#include "spi_dispatcher.h"
#include "nRF.h"

/*
This is an example showing how to use simavr-nRF24 (and simavr-spi-dispatcher). It also shows how to simulate two AVR running with different frequencies.

Please read the fine manual.

(c) 2022 by kittennbfive

AGPLv3+ and NO WARRANTY!

version 11.05.22 01:54
*/

volatile bool run=true;

static void sigint_handler(int sig)
{
	(void)sig;
	run=false;
	printf("\n");
}

int main(void)
{
	signal(SIGINT, &sigint_handler);

	avr_t *avr1, *avr2;
	elf_firmware_t firmware1, firmware2;

	if(elf_read_firmware("avr1.elf", &firmware1))
	{
		printf("elf_read_firmware 1 failed\n");
		return 1;
	}

	avr1 = avr_make_mcu_by_name("atmega328p");
	if (!avr1) {
		printf("avr_make_mcu_by_name 1 failed\n");
		return 1;
	}

	if(elf_read_firmware("avr2.elf", &firmware2))
	{
		printf("elf_read_firmware 2 failed\n");
		return 1;
	}

	avr2 = avr_make_mcu_by_name("atmega328p");
	if (!avr2) {
		printf("avr_make_mcu_by_name 2 failed\n");
		return 1;
	}

	avr_init(avr1);
	avr_init(avr2);

	avr1->frequency=10000000;
	avr2->frequency=8000000;

	avr_load_firmware(avr1, &firmware1);
	avr_load_firmware(avr2, &firmware2);

	nRF_global_init();

	nRF_stop_on_error(true); //change this to experiment

	nRF_set_lost_packets(0, 0); //change this to experiment

	nRF_set_log_level(NRF_LOG_WARNING); //change this to experiment

	nRF_t * nRF1=make_new_nRF();
	nRF_init(avr1, nRF1, "nRF1");
	nRF_connect(nRF1, avr_io_getirq(avr1, AVR_IOCTL_IOPORT_GETIRQ('D'), 5), avr_io_getirq(avr1, AVR_IOCTL_IOPORT_GETIRQ('D'), 7));

	//nRF_log_to_file(nRF1, "log_nRF1.txt"); //add this to experiment

	nRF_t * nRF2=make_new_nRF();
	nRF_init(avr2, nRF2, "nRF2");
	nRF_connect(nRF2, avr_io_getirq(avr2, AVR_IOCTL_IOPORT_GETIRQ('D'), 5), avr_io_getirq(avr2, AVR_IOCTL_IOPORT_GETIRQ('D'), 7));

	//nRF_log_to_file(nRF2, "log_nRF2.txt"); //add this to experiment

	spi_dispatcher_t * dispatch1=make_new_spi_dispatcher(avr1, "avr1");
	init_spi_dispatcher(dispatch1, "nRF", avr_io_getirq(avr1, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_OUTPUT), avr_io_getirq(avr1, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT), \
						nRF1, avr_io_getirq(avr1, AVR_IOCTL_IOPORT_GETIRQ('D'), 6), &csn_nRF, &spi_nRF);

	spi_dispatcher_t * dispatch2=make_new_spi_dispatcher(avr2, "avr2");
	init_spi_dispatcher(dispatch2, "nRF", avr_io_getirq(avr2, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_OUTPUT), avr_io_getirq(avr2, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT), \
						nRF2, avr_io_getirq(avr2, AVR_IOCTL_IOPORT_GETIRQ('D'), 6), &csn_nRF, &spi_nRF);

	printf("starting simulation - interrupt with Ctrl+C\n");

	//f_avr1/f_avr2=5/4 - see https://github.com/buserror/simavr/issues/476
	const int multiplier_avr1=4;
	const int multiplier_avr2=5;

	int state1, state2;
	do
	{
		if(multiplier_avr1*avr1->cycle < multiplier_avr2*avr2->cycle)
			state1=avr_run(avr1);
		else
			state2=avr_run(avr2);
	} while(state1!=cpu_Done && state1!=cpu_Crashed && state2!=cpu_Done && state2!=cpu_Crashed && run);

	avr_terminate(avr1);
	avr_terminate(avr2);

	nRF_cleanup();

	printf("simulation finished\n");

	return 0;
}
