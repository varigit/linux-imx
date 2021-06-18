/*
 * Copyright (C) 2021 Variscite Ltd.
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef IMX8_SC_PAD_API_H
#define IMX8_SC_PAD_API_H

/*
 * Defines for the wakeup trigger types
 */
#define SC_PAD_WAKEUP_OFF       0U	/* Off */
#define SC_PAD_WAKEUP_CLEAR     1U	/* Clears pending flag */
#define SC_PAD_WAKEUP_LOW_LVL   4U	/* Low level */
#define SC_PAD_WAKEUP_FALL_EDGE 5U	/* Falling edge */
#define SC_PAD_WAKEUP_RISE_EDGE 6U	/* Rising edge */
#define SC_PAD_WAKEUP_HIGH_LVL  7U	/* High-level */

#endif	/* IMX8_SC_PAD_API_H */
