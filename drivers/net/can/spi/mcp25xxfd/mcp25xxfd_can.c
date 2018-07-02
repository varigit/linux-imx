// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 */

/* controller details
 *
 *  It has 32 FIFOs (of up to 32 CAN-frames).
 *
 * There are 4 Fifo types which can get configured:
 * * TEF - Transmission Event Fifo - which consumes FIFO 0
 *   even if it is not configured
 * * Tansmission Queue - for up to 32 Frames.
 *   this queue reorders CAN frames to get transmitted following the
 *   typical CAN dominant/recessive rules on the can bus itself.
 *   This FIFO is optional.
 * * TX FIFO: generic TX fifos that can contain arbitrary data
 *   and which come with a configurable priority for transmission
 *   It is also possible to have the Controller automatically trigger
 *   a transfer when a Filter Rule for a RTR frame matches.
 *   Each of these fifos in principle can get configured for distinct
 *   dlc sizes (8 thru 64 bytes)
 * * RX FIFO: generic RX fifo which is filled via filter-rules.
 *   Each of these fifos in principle can get configured for distinct
 *   dlc sizes (8 thru 64 bytes)
 *   Unfortunately there is no filter rule that would allow triggering
 *   on different frame sizes, so for all practical purposes the
 *   RX fifos have to be of the same size (unless one wants to experience
 *   lost data).
 * When a Can Frame is transmitted from the TX Queue or an individual
 * TX FIFO then a small TEF Frame can get added to the TEF FIFO queue
 * to log the Transmission of the frame - this includes ID, Flags
 * (including a custom identifier/index) and the timestamp (see below).
 *
 * The controller provides an optional free running counter with a divider
 * for timestamping of RX frames as well as for TEF entries.
 */

/* Implementation notes:
 *
 * Right now we only use the CAN controller block to put us into deep sleep
 * this means that the oscillator clock is turned off.
 * So this is the only thing that we implement here right now
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>

#include "mcp25xxfd_can.h"
#include "mcp25xxfd_clock.h"
#include "mcp25xxfd_cmd.h"
#include "mcp25xxfd_priv.h"
#include "mcp25xxfd_regs.h"

static int mcp25xxfd_can_get_mode(struct mcp25xxfd_priv *priv, u32 *reg)
{
	int ret;

	ret = mcp25xxfd_cmd_read(priv->spi, MCP25XXFD_CAN_CON, reg);
	if (ret)
		return ret;

	return (*reg & MCP25XXFD_CAN_CON_OPMOD_MASK) >>
		MCP25XXFD_CAN_CON_OPMOD_SHIFT;
}

static int mcp25xxfd_can_switch_mode(struct mcp25xxfd_priv *priv,
				     u32 *reg, int mode)
{
	u32 dummy;
	int ret, i;

	/* get the current mode/register - if reg is NULL
	 * when the can controller is not setup yet
	 * typically by calling mcp25xxfd_can_sleep_mode
	 * (this only happens during initialization phase)
	 */
	if (reg) {
		ret = mcp25xxfd_can_get_mode(priv, reg);
		if (ret < 0)
			return ret;
	} else {
		/* alternatively use dummy */
		dummy = 0;
		reg = &dummy;
	}

	/* compute the effective mode in osc*/
	*reg &= ~(MCP25XXFD_CAN_CON_REQOP_MASK |
		  MCP25XXFD_CAN_CON_OPMOD_MASK);
	*reg |= (mode << MCP25XXFD_CAN_CON_REQOP_SHIFT) |
		(mode << MCP25XXFD_CAN_CON_OPMOD_SHIFT);

	/* if the opmode is sleep then the oscilator will be disabled
	 * and also not ready, so fake this change
	 */
	if (mode == MCP25XXFD_CAN_CON_MODE_SLEEP)
		mcp25xxfd_clock_fake_sleep(priv);

	/* request the mode switch */
	ret = mcp25xxfd_cmd_write(priv->spi, MCP25XXFD_CAN_CON, *reg);
	if (ret)
		return ret;

	/* if we are in now sleep mode then return immediately
	 * the controller does not respond back!
	 */
	if (mode == MCP25XXFD_CAN_CON_MODE_SLEEP)
		return 0;

	/* wait for it to stabilize/switch mode
	 * we assume 256 rounds should be enough as this is > 12ms
	 * at 1MHz Can Bus speed without any extra overhead
	 *
	 * The assumption here is that it depends on bus activity
	 * how long it takes the controller to switch modes
	 */
	for (i = 0; i < 256; i++) {
		/* get the mode */
		ret = mcp25xxfd_can_get_mode(priv, reg);
		if (ret < 0)
			return ret;
		/* check that we have reached our mode */
		if (ret == mode)
			return 0;
	}

	dev_err(&priv->spi->dev, "Failed to switch to mode %u in time\n",
		mode);
	return -ETIMEDOUT;
}

static int mcp25xxfd_can_probe_modeswitch(struct mcp25xxfd_priv *priv)
{
	u32 mode_data;
	int ret;

	/* so we should be in config mode now, so move to INT_LOOPBACK */
	ret = mcp25xxfd_can_switch_mode(priv, &mode_data,
					MCP25XXFD_CAN_CON_MODE_INT_LOOPBACK);
	if (ret) {
		dev_err(&priv->spi->dev,
			"Failed to switch into loopback mode\n");
		return ret;
	}

	/* and back into config mode */
	ret = mcp25xxfd_can_switch_mode(priv, &mode_data,
					MCP25XXFD_CAN_CON_MODE_CONFIG);
	if (ret) {
		dev_err(&priv->spi->dev,
			"Failed to switch back to config mode\n");
		return ret;
	}

	/* so we have checked basic functionality successfully */
	return 0;
}

int mcp25xxfd_can_sleep_mode(struct mcp25xxfd_priv *priv)
{
	return mcp25xxfd_can_switch_mode(priv, NULL,
					 MCP25XXFD_CAN_CON_MODE_SLEEP);
}

int mcp25xxfd_can_probe(struct mcp25xxfd_priv *priv)
{
	struct spi_device *spi = priv->spi;
	u32 mode_data;
	int mode, ret;

	/* read TXQCON - the TXEN bit should always read as 1 */
	ret = mcp25xxfd_cmd_read(spi, MCP25XXFD_CAN_TXQCON, &mode_data);
	if (ret)
		return ret;
	if ((mode_data & MCP25XXFD_CAN_TXQCON_TXEN) == 0) {
		dev_err(&spi->dev,
			"Register TXQCON does not have bit TXEN set - reads as %08x - this may be a problem with spi bus signal quality - try reducing spi-clock speed if this can get reproduced",
			mode_data);
		return -EINVAL;
	}

	/* try to get the current mode */
	mode = mcp25xxfd_can_get_mode(priv, &mode_data);
	if (mode < 0)
		return mode;

	/* we would expect to be in config mode, as a SPI-reset should
	 * have moved us into config mode.
	 * But then the documentation says that SPI-reset may only work
	 * reliably when already in config mode
	 */

	/* so if we are in config mode then everything is fine
	 * and we check that a mode switch works propperly
	 */
	if (mode == MCP25XXFD_CAN_CON_MODE_CONFIG)
		return mcp25xxfd_can_probe_modeswitch(priv);

	/* if the bitfield is 0 then there is something is wrong */
	if (!mode_data) {
		dev_err(&spi->dev,
			"got controller config register reading as 0\n");
		return -EINVAL;
	}

	/* any other mode is unexpected */
	dev_err(&spi->dev,
		"Found controller in unexpected mode %i - register reads as %08x\n",
		mode, mode_data);

	/* so try to move to config mode
	 * if this fails, then everything is lost and the controller
	 * is not identified
	 * This action MAY be destructive if a different device is connected
	 * but note that the first hurdle (oscillator) was already
	 * successful - so we should be safe...
	 */
	ret = mcp25xxfd_can_switch_mode(priv, &mode_data,
					MCP25XXFD_CAN_CON_MODE_CONFIG);
	if (ret) {
		dev_err(&priv->spi->dev,
			"Mode did not switch to config as expected - could not identify controller - register reads as %08x\n",
			mode_data);
		return -EINVAL;
	}
	/* check that modeswitch is really working */
	return mcp25xxfd_can_probe_modeswitch(priv);
}
