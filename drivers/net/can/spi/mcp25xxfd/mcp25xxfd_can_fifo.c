// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 */

/* here we define and configure the fifo layout */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "mcp25xxfd_can.h"
#include "mcp25xxfd_can_fifo.h"
#include "mcp25xxfd_can_priv.h"
#include "mcp25xxfd_cmd.h"

static int mcp25xxfd_can_fifo_get_address(struct mcp25xxfd_can_priv *cpriv)
{
	int fifo, ret;

	/* we need to move out of config mode to force address computation */
	ret = mcp25xxfd_can_switch_mode(cpriv->priv, &cpriv->regs.con,
					MCP25XXFD_CAN_CON_MODE_INT_LOOPBACK);
	if (ret)
		return ret;

	/* and get back into config mode */
	ret = mcp25xxfd_can_switch_mode(cpriv->priv, &cpriv->regs.con,
					MCP25XXFD_CAN_CON_MODE_CONFIG);
	if (ret)
		return ret;

	/* read address and config back in */
	for (fifo = 1; fifo < 32; fifo++) {
		ret = mcp25xxfd_cmd_read(cpriv->priv->spi,
					 MCP25XXFD_CAN_FIFOUA(fifo),
					 &cpriv->fifos.info[fifo].offset);
		if (ret)
			return ret;
	}

	return 0;
}

static int mcp25xxfd_can_fifo_setup_config(struct mcp25xxfd_can_priv *cpriv,
					   struct mcp25xxfd_fifo *desc,
					   u32 flags, u32 flags_last)
{
	u32 val;
	int i, p, f, c, ret;

	/* now setup the fifos themselves */
	for (i = 0, f = desc->start, c = desc->count, p = 31;
	     c > 0; i++, f++, p--, c--) {
		/* select the effective value */
		val = (c > 1) ? flags : flags_last;

		/* write the config to the controller in one go */
		ret = mcp25xxfd_cmd_write(cpriv->priv->spi,
					  MCP25XXFD_CAN_FIFOCON(f), val);
		if (ret)
			return ret;
	}

	return 0;
}

static int mcp25xxfd_can_fifo_setup_rx(struct mcp25xxfd_can_priv *cpriv)
{
	u32 rx_flags = MCP25XXFD_CAN_FIFOCON_FRESET |     /* reset FIFO */
		MCP25XXFD_CAN_FIFOCON_RXTSEN |            /* RX timestamps */
		MCP25XXFD_CAN_FIFOCON_TFERFFIE |          /* FIFO Full */
		MCP25XXFD_CAN_FIFOCON_TFHRFHIE |          /* FIFO Half Full*/
		MCP25XXFD_CAN_FIFOCON_TFNRFNIE |          /* FIFO not empty */
		(cpriv->fifos.payload_mode <<
		 MCP25XXFD_CAN_FIFOCON_PLSIZE_SHIFT) |
		(0 << MCP25XXFD_CAN_FIFOCON_FSIZE_SHIFT); /* 1 FIFO deep */
	/* enable overflow int on last fifo */
	u32 rx_flags_last = rx_flags | MCP25XXFD_CAN_FIFOCON_RXOVIE;

	return mcp25xxfd_can_fifo_setup_config(cpriv, &cpriv->fifos.rx,
					       rx_flags, rx_flags_last);
}

static int mcp25xxfd_can_fifo_setup_rxfilter(struct mcp25xxfd_can_priv *cpriv)
{
	u8 filter_con[32];
	int c, f;

	/* clear the filters and filter mappings for all filters */
	memset(filter_con, 0, sizeof(filter_con));

	/* and now set up the rx filters */
	for (c = 0, f = cpriv->fifos.rx.start; c < cpriv->fifos.rx.count;
	     c++, f++) {
		/* set up filter config - we can use the mask of filter 0 */
		filter_con[c] = MCP25XXFD_CAN_FIFOCON_FLTEN(0) |
			(f << MCP25XXFD_CAN_FILCON_SHIFT(0));
	}

	/* and set up filter control */
	return mcp25xxfd_cmd_write_regs(cpriv->priv->spi,
					MCP25XXFD_CAN_FLTCON(0),
					(u32 *)filter_con, sizeof(filter_con));
}

static int mcp25xxfd_can_fifo_compute(struct mcp25xxfd_can_priv *cpriv)
{
	int rx_memory_available;

	/* default settings as per MTU/CANFD */
	switch (cpriv->can.dev->mtu) {
	case CAN_MTU:
		/* mtu is 8 */
		cpriv->fifos.payload_size = 8;
		cpriv->fifos.payload_mode = MCP25XXFD_CAN_TXQCON_PLSIZE_8;

		break;
	case CANFD_MTU:
		/* wish there was a way to have hw filters
		 * that can separate based on length ...
		 */
		/* MTU is 64 */
		cpriv->fifos.payload_size = 64;
		cpriv->fifos.payload_mode = MCP25XXFD_CAN_TXQCON_PLSIZE_64;

		break;
	default:
		return -EINVAL;
	}

	/* compute effective sizes */
	cpriv->fifos.rx.size = sizeof(struct mcp25xxfd_can_obj_rx) +
		cpriv->fifos.payload_size;

	/* calculate evailable memory for RX_fifos */
	rx_memory_available = MCP25XXFD_SRAM_SIZE;

	/* calculate possible amount of RX fifos */
	cpriv->fifos.rx.count = rx_memory_available / cpriv->fifos.rx.size;

	/* so now calculate effective number of rx-fifos
	 * there are only 31 fifos available in total,
	 * so we need to limit ourselves
	 */
	if (cpriv->fifos.rx.count > 31)
		cpriv->fifos.rx.count = 31;

	/* define the layout now that we have gotten everything */
	cpriv->fifos.rx.start = 1;

	return 0;
}

static int mcp25xxfd_can_fifo_clear_regs(struct mcp25xxfd_can_priv *cpriv,
					 u32 start, u32 end)
{
	size_t len = end - start;
	u8 *data = kzalloc(len, GFP_KERNEL);
	int ret;

	if (!data)
		return -ENOMEM;

	ret = mcp25xxfd_cmd_write_regs(cpriv->priv->spi,
				       start, (u32 *)data, len);

	kfree(data);

	return ret;
}

static int mcp25xxfd_can_fifo_clear(struct mcp25xxfd_can_priv *cpriv)
{
	int ret;

	memset(&cpriv->fifos.info, 0, sizeof(cpriv->fifos.info));
	memset(&cpriv->fifos.rx, 0, sizeof(cpriv->fifos.rx));

	/* clear FIFO config */
	ret = mcp25xxfd_can_fifo_clear_regs(cpriv, MCP25XXFD_CAN_FIFOCON(1),
					    MCP25XXFD_CAN_FIFOCON(32));
	if (ret)
		return ret;

	/* clear the filter mask - match any frame with every filter */
	return mcp25xxfd_can_fifo_clear_regs(cpriv, MCP25XXFD_CAN_FLTCON(0),
					     MCP25XXFD_CAN_FLTCON(32));
}

int mcp25xxfd_can_fifo_setup(struct mcp25xxfd_can_priv *cpriv)
{
	int ret;

	/* clear fifo config */
	ret = mcp25xxfd_can_fifo_clear(cpriv);
	if (ret)
		return ret;

	/* compute fifos counts */
	ret = mcp25xxfd_can_fifo_compute(cpriv);
	if (ret)
		return ret;

	/* configure TEF */
	cpriv->regs.tefcon = 0;
	ret = mcp25xxfd_cmd_write(cpriv->priv->spi, MCP25XXFD_CAN_TEFCON,
				  cpriv->regs.tefcon);
	if (ret)
		return ret;

	/* TXQueue disabled */
	ret = mcp25xxfd_cmd_write(cpriv->priv->spi, MCP25XXFD_CAN_TXQCON, 0);
	if (ret)
		return ret;

	/* configure FIFOS themselves */
	ret = mcp25xxfd_can_fifo_setup_rx(cpriv);
	if (ret)
		return ret;
	ret = mcp25xxfd_can_fifo_setup_rxfilter(cpriv);
	if (ret)
		return ret;

	/* get fifo addresses */
	ret = mcp25xxfd_can_fifo_get_address(cpriv);
	if (ret)
		return ret;

	/* add the can info to debugfs */
	mcp25xxfd_can_debugfs_setup(cpriv);

	return 0;
}

void mcp25xxfd_can_fifo_release(struct mcp25xxfd_can_priv *cpriv)
{
	mcp25xxfd_can_fifo_clear(cpriv);
	mcp25xxfd_can_debugfs_remove(cpriv);
}
