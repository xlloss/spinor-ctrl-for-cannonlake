/*
 * This file is part of the coreboot project.
 *
 * Copyright 2017 Google Inc.
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
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <windows.h>

#include "gspi.h"
#include "memctl.h"

/* GSPI Memory Mapped Registers */
#define SSCR0			0x0	/* SSP Control Register 0 */
#define   SSCR0_EDSS_0		(0 << 20)
#define   SSCR0_EDSS_1		(1 << 20)
#define   SSCR0_SCR_SHIFT	(8)
#define   SSCR0_SCR_MASK	(0xFFF)
#define   SSCR0_SSE_DISABLE	(0 << 7)
#define   SSCR0_SSE_ENABLE	(1 << 7)
#define   SSCR0_ECS_ON_CHIP	(0 << 6)
#define   SSCR0_FRF_MOTOROLA	(0 << 4)
#define   SSCR0_DSS_SHIFT	(0)
#define   SSCR0_DSS_MASK	(0xF)
#define SSCR1			0x4	/* SSP Control Register 1 */
#define   SSCR1_IFS_LOW	(0 << 16)
#define   SSCR1_IFS_HIGH	(1 << 16)
#define   SSCR1_SPH_FIRST	(0 << 4)
#define   SSCR1_SPH_SECOND	(1 << 4)
#define   SSCR1_SPO_LOW	(0 << 3)
#define   SSCR1_SPO_HIGH	(1 << 3)
#define SSSR			0x8	/* SSP Status Register */
#define   SSSR_TUR		(1 << 21)  /* Tx FIFO underrun */
#define   SSSR_TINT		(1 << 19)  /* Rx Time-out interrupt */
#define   SSSR_PINT		(1 << 18)  /* Peripheral trailing byte
					      interrupt */
#define   SSSR_ROR		(1 << 7)   /* Rx FIFO Overrun */
#define   SSSR_BSY		(1 << 4)   /* SSP Busy */
#define   SSSR_RNE		(1 << 3)   /* Receive FIFO not empty */
#define   SSSR_TNF		(1 << 2)   /* Transmit FIFO not full */
#define SSDR			0x10	/* SSP Data Register */
#define SSTO			0x28	/* SSP Time out */
#define SITF			0x44	/* SPI Transmit FIFO */
#define   SITF_LEVEL_SHIFT	(16)
#define   SITF_LEVEL_MASK	(0x3f)
#define   SITF_LWM_SHIFT	(8)
#define   SITF_LWM_MASK	(0x3f)
#define   SITF_LWM(x)		((((x) - 1) & SITF_LWM_MASK) << SITF_LWM_SHIFT)
#define   SITF_HWM_SHIFT	(0)
#define   SITF_HWM_MASK	(0x3f)
#define   SITF_HWM(x)		((((x) - 1) & SITF_HWM_MASK) << SITF_HWM_SHIFT)
#define SIRF			0x48	/* SPI Receive FIFO */
#define   SIRF_LEVEL_SHIFT	(8)
#define   SIRF_LEVEL_MASK	(0x3f)
#define   SIRF_WM_SHIFT	(0)
#define   SIRF_WM_MASK		(0x3f)
#define   SIRF_WM(x)		((((x) - 1) & SIRF_WM_MASK) << SIRF_WM_SHIFT)

/* GSPI Additional Registers */
#define CLOCKS			0x200	/* Clocks */
#define   CLOCKS_UPDATE	(1 << 31)
#define   CLOCKS_N_SHIFT	(16)
#define   CLOCKS_N_MASK	(0x7fff)
#define   CLOCKS_M_SHIFT	(1)
#define   CLOCKS_M_MASK	(0x7fff)
#define   CLOCKS_DISABLE	(0 << 0)
#define   CLOCKS_ENABLE	(1 << 0)
#define RESETS			0x204	/* Resets */
#define   DMA_RESET		(0 << 2)
#define   DMA_ACTIVE		(1 << 2)
#define   CTRLR_RESET		(0 << 0)
#define   CTRLR_ACTIVE		(3 << 0)
#define ACTIVELTR_VALUE	0x210	/* Active LTR */
#define IDLELTR_VALUE		0x214	/* Idle LTR Value */
#define TX_BIT_COUNT		0x218	/* Tx Bit Count */
#define RX_BIT_COUNT		0x21c	/* Rx Bit Count */
#define SSP_REG		0x220	/* SSP Reg */
#define   DMA_FINISH_DISABLE	(1 << 0)
#define SPI_CS_CONTROL		0x224	/* SPI CS Control */
#define   CS_0_POL_SHIFT	(12)
#define   CS_0_POL_MASK		(1 << CS_0_POL_SHIFT)
#define   CS_POL_LOW		(0)
#define   CS_POL_HIGH		(1)
#define   CS_0			(0 << 8)
#define   CS_STATE_SHIFT	(1)
#define   CS_STATE_MASK		(1 << CS_STATE_SHIFT)
#define   CS_V1_STATE_LOW	(0)
#define   CS_V1_STATE_HIGH	(1)
#define   CS_MODE_HW		(0 << 0)
#define   CS_MODE_SW		(1 << 0)

#define GSPI_DATA_BIT_LENGTH		(8)
#define GSPI_BASE 0xFE0F9000

static uint32_t gspi_get_bus_clk_mhz(unsigned int gspi_bus)
{
    uint32_t speed_mhz;

	return speed_mhz;
}

static unsigned int gspi_get_bus_base_addr()
{
	return GSPI_BASE;
}

/* Parameters for GSPI controller operation. */
struct gspi_ctrlr_params {
	unsigned int mmio_base;
	unsigned int gspi_bus;
	uint8_t *in;
	size_t bytesin;
	const uint8_t *out;
	size_t bytesout;
};

static uint32_t gspi_read_mmio_reg(const struct gspi_ctrlr_params *p,
					uint32_t offset)
{
    uint32_t data = 0;
    int ret;

	assert(p->mmio_base != 0);
    ret = ReadMemDW(p->mmio_base + offset, &data);
    if (ret) {
        printf("%s fail\r\n", __func__);
        return 0;
    }

    return data;
}

static void gspi_write_mmio_reg(const struct gspi_ctrlr_params *p,
				uint32_t offset, uint32_t value)
{
    int ret;

	assert(p->mmio_base != 0);
	ret = WriteMemDW(p->mmio_base + offset, value);
    if (ret != MEM_SUCCESSFUL)
        printf("%s fail\r\n", __func__);
}

static int gspi_ctrlr_params_init(struct gspi_ctrlr_params *p)
{
	memset(p, 0, sizeof(*p));

	p->mmio_base = gspi_get_bus_base_addr();
	if (!p->mmio_base) {
		printf("%s: Base addr is 0 for GSPI\n", __func__);
		return -1;
	}

	return 0;
}

enum cs_assert {
	CS_ASSERT,
	CS_DEASSERT,
};

/*
 * SPI_CS_CONTROL bit definitions based on GSPI_VERSION_x:
 *
 * VERSION_2 (CNL GSPI controller):
 * Polarity: Indicates inactive polarity of chip-select
 * State   : Indicates assert/de-assert of chip-select
 *
 * Default (SKL/KBL GSPI controller):
 * Polarity: Indicates active polarity of chip-select
 * State   : Indicates low/high output state of chip-select
 */
static uint32_t gspi_csctrl_state_v2(uint32_t pol, enum cs_assert cs_assert)
{
	return cs_assert;
}

static uint32_t gspi_csctrl_state_v1(uint32_t pol, enum cs_assert cs_assert)
{
	uint32_t state;

	if (pol == CS_POL_HIGH)
		state = (cs_assert == CS_ASSERT) ? CS_V1_STATE_HIGH :
			CS_V1_STATE_LOW;
	else
		state = (cs_assert == CS_ASSERT) ? CS_V1_STATE_LOW :
			CS_V1_STATE_HIGH;

	return state;
}

static uint32_t gspi_csctrl_state(uint32_t pol, enum cs_assert cs_assert)
{
#ifdef SOC_INTEL_COMMON_BLOCK_GSPI_VERSION_2
    return gspi_csctrl_state_v2(pol, cs_assert);
#else
	return gspi_csctrl_state_v1(pol, cs_assert);
#endif
}

static uint32_t gspi_csctrl_polarity_v2(enum spi_polarity active_pol)
{
	/* Polarity field indicates cs inactive polarity */
	if (active_pol == SPI_POLARITY_LOW)
		return CS_POL_HIGH;
	return CS_POL_LOW;
}

static uint32_t gspi_csctrl_polarity_v1(enum spi_polarity active_pol)
{
	/* Polarity field indicates cs active polarity */
	if (active_pol == SPI_POLARITY_LOW)
		return CS_POL_LOW;
	return CS_POL_HIGH;
}

static uint32_t gspi_csctrl_polarity(enum spi_polarity active_pol)
{
#ifdef SOC_INTEL_COMMON_BLOCK_GSPI_VERSION_2
    return gspi_csctrl_polarity_v2(active_pol);
#else
	return gspi_csctrl_polarity_v1(active_pol);
#endif
}

static void __gspi_cs_change(const struct gspi_ctrlr_params *p,
				enum cs_assert cs_assert)
{
	uint32_t cs_ctrl, pol;
	cs_ctrl = gspi_read_mmio_reg(p, SPI_CS_CONTROL);

	cs_ctrl &= ~CS_STATE_MASK;

	pol = !!(cs_ctrl & CS_0_POL_MASK);
	cs_ctrl |= gspi_csctrl_state(pol, cs_assert) << CS_STATE_SHIFT;

	gspi_write_mmio_reg(p, SPI_CS_CONTROL, cs_ctrl);
}

static int gspi_cs_change(const struct spi_slave *dev,
    enum cs_assert cs_assert)
{
	struct gspi_ctrlr_params params, *p;

	p = &params;

	if (gspi_ctrlr_params_init(p))
		return -1;

	__gspi_cs_change(p, cs_assert);

	return 0;
}

int gspi_get_soc_spi_cfg(struct spi_cfg *cfg)
{
	cfg->clk_phase = SPI_CLOCK_PHASE_FIRST;
	cfg->clk_polarity = SPI_POLARITY_LOW;
	cfg->cs_polarity = SPI_POLARITY_LOW;
	cfg->wire_mode = SPI_4_WIRE_MODE;
	cfg->data_bit_length = GSPI_DATA_BIT_LENGTH;

	return 0;
}

int gspi_cs_assert(const struct spi_slave *dev)
{
	return gspi_cs_change(dev, CS_ASSERT);
}

void gspi_cs_deassert(const struct spi_slave *dev)
{
	gspi_cs_change(dev, CS_DEASSERT);
}

static uint32_t gspi_get_clk_div(unsigned int gspi_bus)
{
    /*
	const uint32_t ref_clk_mhz =
		CONFIG_SOC_INTEL_COMMON_BLOCK_GSPI_CLOCK_MHZ;
	uint32_t gspi_clk_mhz = gspi_get_bus_clk_mhz(gspi_bus);

	if (!gspi_clk_mhz)
		gspi_clk_mhz = 1;

	assert(ref_clk_mhz != 0);
	return (DIV_ROUND_UP(ref_clk_mhz, gspi_clk_mhz) - 1) & SSCR0_SCR_MASK;
    */

    return 0xC00;
}

int gspi_ctrlr_setup(const struct spi_slave *dev)
{
	struct spi_cfg cfg;
	uint32_t cs_ctrl, sscr0, sscr1, clocks, sitf, sirf, pol;
	struct gspi_ctrlr_params params, *p;
    int status;

    status = Init_memctl();
    if (status != MEM_SUCCESSFUL) {
        printf("Init_memctl fail\r\n");
        return -1;
    }

	p = &params;
	/* Only chip select 0 is supported. */
	if (dev->cs != 0) {
		printf("%s: Invalid CS value: cs=%u.\n", __func__, dev->cs);
		return -1;
	}

	if (gspi_ctrlr_params_init(p))
		return -1;

	/* Obtain SPI bus configuration for the device. */
	if (gspi_get_soc_spi_cfg(&cfg)) {
		printf("%s: Failed to get config\n", __func__);
		return -1;
	}

	/* Take controller out of reset, keeping DMA in reset. */
	gspi_write_mmio_reg(p, RESETS, CTRLR_ACTIVE | DMA_RESET);

	/*
	 * CS control:
	 * - Set SW mode.
	 * - Set chip select to 0.
	 * - Set polarity based on device configuration.
	 * - Do not assert CS.
	 */
	cs_ctrl = CS_MODE_SW | CS_0;
	pol = gspi_csctrl_polarity(cfg.cs_polarity);
	cs_ctrl |= pol << CS_0_POL_SHIFT;
	cs_ctrl |= gspi_csctrl_state(pol, CS_DEASSERT) << CS_STATE_SHIFT;
    gspi_write_mmio_reg(p, SPI_CS_CONTROL, cs_ctrl);

	/* Disable SPI controller. */
	gspi_write_mmio_reg(p, SSCR0, SSCR0_SSE_DISABLE);

	/*
	 * SSCR0 configuration:
	 * clk_div - Based on reference clock and expected clock frequency.
	 * data bit length - assumed to be 8, hence EDSS = 0.
	 * ECS - Use on-chip clock
	 * FRF - Frame format set to Motorola SPI
	 */
	sscr0 = gspi_get_clk_div(p->gspi_bus) << SSCR0_SCR_SHIFT;
	assert(GSPI_DATA_BIT_LENGTH == 8);
	sscr0 |= ((GSPI_DATA_BIT_LENGTH - 1) << SSCR0_DSS_SHIFT) | SSCR0_EDSS_0;
	sscr0 |= SSCR0_ECS_ON_CHIP | SSCR0_FRF_MOTOROLA;
    gspi_write_mmio_reg(p, SSCR0, sscr0);

	/*
	 * SSCR1 configuration:
	 * - Chip select polarity
	 * - Clock phase setting
	 * - Clock polarity
	 */
	sscr1 = (cfg.cs_polarity == SPI_POLARITY_LOW) ? SSCR1_IFS_LOW :
		SSCR1_IFS_HIGH;
	sscr1 |= (cfg.clk_phase == SPI_CLOCK_PHASE_FIRST) ? SSCR1_SPH_FIRST :
		SSCR1_SPH_SECOND;
	sscr1 |= (cfg.clk_polarity == SPI_POLARITY_LOW) ? SSCR1_SPO_LOW :
		SSCR1_SPO_HIGH;
    gspi_write_mmio_reg(p, SSCR1, sscr1);

	/*
	 * Program m/n divider.
	 * Set m and n to 1, so that this divider acts as a pass-through.
	 */
	clocks = (1 << CLOCKS_N_SHIFT) | (1 << CLOCKS_M_SHIFT) | CLOCKS_ENABLE |
		 CLOCKS_UPDATE;
    //clocks = CLOCKS_ENABLE;
	gspi_write_mmio_reg(p, CLOCKS, clocks);
	Sleep(1);

	/*
	 * Tx FIFO Threshold.
	 * Low watermark threshold = 1
	 * High watermark threshold = 1
	 */
	sitf = SITF_LWM(1) | SITF_HWM(1);
	gspi_write_mmio_reg(p, SITF, sitf);

	/* Rx FIFO Threshold (set to 1). */
	sirf = SIRF_WM(1);
	gspi_write_mmio_reg(p, SIRF, sirf);

	/* Enable GSPI controller. */
	sscr0 |= SSCR0_SSE_ENABLE;
    gspi_write_mmio_reg(p, SSCR0, sscr0);

	return 0;
}

static uint32_t gspi_read_status(const struct gspi_ctrlr_params *p)
{
	return gspi_read_mmio_reg(p, SSSR);
}

static void gspi_clear_status(const struct gspi_ctrlr_params *p)
{
	const uint32_t sssr = SSSR_TUR | SSSR_TINT | SSSR_PINT | SSSR_ROR;
	gspi_write_mmio_reg(p, SSSR, sssr);
}

static bool gspi_rx_fifo_empty(const struct gspi_ctrlr_params *p)
{
	return !(gspi_read_status(p) & SSSR_RNE);
}

static bool gspi_tx_fifo_full(const struct gspi_ctrlr_params *p)
{
	return !(gspi_read_status(p) & SSSR_TNF);
}

static bool gspi_rx_fifo_overrun(const struct gspi_ctrlr_params *p)
{
	if (gspi_read_status(p) & SSSR_ROR) {
		printf("%s:GSPI receive FIFO overrun!\n", __func__);
		return true;
	}

	return false;
}

/* Read SSDR and return lowest byte. */
static uint8_t gspi_read_byte(const struct gspi_ctrlr_params *p)
{
	return gspi_read_mmio_reg(p, SSDR) & 0xFF;
}

/* Write 32-bit word with "data" in lowest byte to SSDR. */
static void gspi_write_byte(const struct gspi_ctrlr_params *p, uint8_t data)
{
	return gspi_write_mmio_reg(p, SSDR, data);
}

static void gspi_read_data(struct gspi_ctrlr_params *p)
{
	*(p->in) = gspi_read_byte(p);
	p->in++;
	p->bytesin--;
}

static void gspi_write_data(struct gspi_ctrlr_params *p)
{
	gspi_write_byte(p, *(p->out));
	p->out++;
	p->bytesout--;
}

static void gspi_read_dummy(struct gspi_ctrlr_params *p)
{
	gspi_read_byte(p);
	p->bytesin--;
}

static void gspi_write_dummy(struct gspi_ctrlr_params *p)
{
	gspi_write_byte(p, 0);
	p->bytesout--;
}

static int gspi_ctrlr_flush(const struct gspi_ctrlr_params *p)
{
	const uint32_t timeout_ms = 500;

	/* Wait 500ms to allow Rx FIFO to be empty. */
    Sleep(500);
	while (!gspi_rx_fifo_empty(p)) {
        Sleep(100);
		gspi_read_byte(p);
	}

	return 0;
}

static int __gspi_xfer(struct gspi_ctrlr_params *p)
{
	/*
	 * If bytesin is non-zero, then use gspi_read_data to perform
	 * byte-by-byte read of data from SSDR and save it to "in" buffer. Else
	 * discard the read data using gspi_read_dummy.
	 */
	void (*fn_read)(struct gspi_ctrlr_params *p) = gspi_read_data;

	/*
	 * If bytesout is non-zero, then use gspi_write_data to perform
	 * byte-by-byte write of data from "out" buffer to SSDR. Else, use
	 * gspi_write_dummy to write dummy "0" data to SSDR in order to trigger
	 * read from slave.
	 */
	void (*fn_write)(struct gspi_ctrlr_params *p) = gspi_write_data;

	if (!p->bytesin) {
		p->bytesin = p->bytesout;
		fn_read = gspi_read_dummy;
	}

	if (!p->bytesout) {
		p->bytesout = p->bytesin;
		fn_write = gspi_write_dummy;
	}

	while (p->bytesout > 0 || p->bytesin > 0) {
		if (p->bytesout && !gspi_tx_fifo_full(p))
			fn_write(p);

		if (p->bytesin && !gspi_rx_fifo_empty(p)) {
			if (gspi_rx_fifo_overrun(p)) {
                printf("%s gspi_rx_fifo_overrun fail\r\n", __func__);
				return -1;
            }
			fn_read(p);
		}
	}

	return 0;
}

int gspi_ctrlr_xfer(const void *dout, size_t bytesout,
			   void *din, size_t bytesin)
{
	struct gspi_ctrlr_params params;
	struct gspi_ctrlr_params *p;

    p = &params;
	/*
	 * Assumptions about in and out transfers:
	 * 1. Both bytesin and bytesout cannot be 0.
	 * 2. If both bytesin and bytesout are non-zero, then they should be
	 * equal i.e. if both in and out transfers are to be done in same
	 * transaction, then they should be equal in length.
	 * 3. Buffer corresponding to non-zero bytes (bytesin/bytesout) cannot
	 * be NULL.
	 */
	if (!bytesin && !bytesout) {
		printf("%s: Both in and out bytes cannot be zero!\n", __func__);
		return -1;
	} else if (bytesin && bytesout && (bytesin != bytesout)) {
		printf("%s: bytesin(%zd) != bytesout(%zd)\n",
		       __func__, bytesin, bytesout);
		return -1;
	}

	if ((bytesin && !din) || (bytesout && !dout)) {
		printf("%s: in/out buffer is NULL!\n", __func__);
		return -1;
	}

	if (gspi_ctrlr_params_init(p)) {
        printf("%s gspi_ctrlr_params_init fail\r\n", __func__);
		return -1;
    }

	/* Flush out any stale data in Rx FIFO. */
	if (gspi_ctrlr_flush(p)) {
        printf("%s gspi_ctrlr_flush fail\r\n", __func__);
		return -1;
    }

	/* Clear status bits. */
	gspi_clear_status(p);

	p->in = (uint8_t *)din;
	p->bytesin = bytesin;
	p->out = (uint8_t *)dout;
	p->bytesout = bytesout;

	return __gspi_xfer(p);
}
