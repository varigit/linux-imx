/* SPDX-License-Identifier: GPL-2.0 */

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 */

#ifndef __MCP25XXFD_CAN_H
#define __MCP25XXFD_CAN_H

#include "mcp25xxfd_priv.h"

/* to put us to sleep fully we need the CAN controller to enter sleep mode */
int mcp25xxfd_can_sleep_mode(struct mcp25xxfd_priv *priv);

/* probe the can controller */
int mcp25xxfd_can_probe(struct mcp25xxfd_priv *priv);

#endif /* __MCP25XXFD_CAN_H */
