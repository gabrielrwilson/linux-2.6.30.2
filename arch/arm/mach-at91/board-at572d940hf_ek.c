/*
 * linux/arch/arm/mach-at91/board-at572d940hf_ek.c
 *
 * Copyright (C) 2008 Atmel Antonio R. Costa <costa.antonior@gmail.com>
 * Copyright (C) 2005 SAN People
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/ds1305.h>
#include <linux/irq.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91sam9_smc.h>

#include "sam9_smc.h"
#include "generic.h"


static void __init eb_map_io(void)
{
	/* Initialize processor: 12.500 MHz crystal */
	at572d940hf_initialize(12000000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 on ttyS1. (Rx & Tx only) */
	at91_register_uart(AT572D940HF_ID_US0, 1, 0);

	/* USART1 on ttyS2. (Rx & Tx only) */
	at91_register_uart(AT572D940HF_ID_US1, 2, 0);

	/* USART2 on ttyS3. (Tx & Rx only */
	at91_register_uart(AT572D940HF_ID_US2, 3, 0);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init eb_init_irq(void)
{
	at572d940hf_init_interrupts(NULL);
}


/*
 * USB Host Port
 */
static struct at91_usbh_data __initdata eb_usbh_data = {
	.ports		= 2,
};


/*
 * USB Device Port
 */
static struct at91_udc_data __initdata eb_udc_data = {
	.vbus_pin	= 0,		/* no VBUS detection,UDC always on */
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};


/*
 * MCI (SD/MMC)
 */
static struct at91_mmc_data __initdata eb_mmc_data = {
	.wire4		= 1,
/*	.det_pin	= ... not connected */
/*	.wp_pin		= ... not connected */
/*	.vcc_pin	= ... not connected */
};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata eb_eth_data = {
	.phy_irq_pin	= AT91_PIN_PB25,
	.is_rmii	= 1,
};


/*
 * NAND flash
 */
static struct mtd_partition __initdata eb_nand_partition[] = {
	{
		.name	= "Partition 1",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL,
	}
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(eb_nand_partition);
	return eb_nand_partition;
}

static struct atmel_nand_data __initdata eb_nand_data = {
	.ale		= 22,
	.cle		= 21,
/*	.det_pin	= ... not connected */
/*	.rdy_pin	= AT91_PIN_PC16, */
	.enable_pin	= AT91_PIN_PA15,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_AT91_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};

static struct sam9_smc_config __initdata eb_nand_smc_config = {
	.ncs_read_setup		= 3,
	.nrd_setup		= 3,
	.ncs_write_setup	= 3,
	.nwe_setup		= 3,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 10,
	.write_cycle		= 10,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 1,
};

static void __init eb_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (eb_nand_data.bus_width_16)
		eb_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		eb_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &eb_nand_smc_config);

	at91_add_device_nand(&eb_nand_data);
}


/*
 * SPI devices
 */
static struct resource rtc_resources[] = {
	[0] = {
		.start	= AT572D940HF_ID_IRQ1,
		.end	= AT572D940HF_ID_IRQ1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct ds1305_platform_data ds1306_data = {
	.is_ds1306	= true,
	.en_1hz		= false,
};

static struct spi_board_info eb_spi_devices[] = {
	{	/* RTC Dallas DS1306 */
		.modalias	= "rtc-ds1305",
		.chip_select	= 3,
		.mode		= SPI_CS_HIGH | SPI_CPOL | SPI_CPHA,
		.max_speed_hz	= 500000,
		.bus_num	= 0,
		.irq		= AT572D940HF_ID_IRQ1,
		.platform_data	= (void *) &ds1306_data,
	},
#if defined(CONFIG_MTD_AT91_DATAFLASH_CARD)
	{	/* Dataflash card */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
#endif
};

static void __init eb_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* USB Host */
	at91_add_device_usbh(&eb_usbh_data);
	/* USB Device */
	at91_add_device_udc(&eb_udc_data);
	/* I2C */
	at91_add_device_i2c(NULL, 0);
	/* NAND */
	eb_add_device_nand();
	/* SPI */
	at91_add_device_spi(eb_spi_devices, ARRAY_SIZE(eb_spi_devices));
	/* MMC */
	at91_add_device_mmc(0, &eb_mmc_data);
	/* Ethernet */
	at91_add_device_eth(&eb_eth_data);
	/* mAgic */
	at91_add_device_mAgic();
}

MACHINE_START(AT572D940HFEB, "Atmel AT91D940HF-EB")
	/* Maintainer: Atmel <costa.antonior@gmail.com> */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= eb_map_io,
	.init_irq	= eb_init_irq,
	.init_machine	= eb_board_init,
MACHINE_END
