#pragma once
/*
 * (C) Copyright 2001
 * Gerald Van Baren, Custom IDEAS, vanbaren@cideas.com.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SPI_GENERIC_H_
#define _SPI_GENERIC_H_

struct spi_ctrlr;

/*-----------------------------------------------------------------------
 * Representation of a SPI slave, i.e. what we're communicating with.
 *
 *   bus:	ID of the bus that the slave is attached to.
 *   cs:	ID of the chip select connected to the slave.
 *   ctrlr:	Pointer to SPI controller structure.
 */
struct spi_slave {
	unsigned int	bus;
	unsigned int	cs;
	const struct spi_ctrlr *ctrlr;
};

enum spi_polarity {
    SPI_POLARITY_LOW,
    SPI_POLARITY_HIGH
};

enum spi_clock_phase {
    SPI_CLOCK_PHASE_FIRST,
    SPI_CLOCK_PHASE_SECOND
};

enum spi_wire_mode {
    SPI_4_WIRE_MODE,
    SPI_3_WIRE_MODE
};

struct spi_cfg {
	/* CLK phase - 0: Phase first, 1: Phase second */
	enum spi_clock_phase clk_phase;
	/* CLK polarity - 0: Low, 1: High */
	enum spi_polarity clk_polarity;
	/* CS polarity - 0: Low, 1: High */
	enum spi_polarity cs_polarity;
	/* Wire mode - 0: 4-wire, 1: 3-wire */
	enum spi_wire_mode wire_mode;
	/* Data bit length. */
	unsigned int data_bit_length;
};
#endif	/* _SPI_GENERIC_H_ */
