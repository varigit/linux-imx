// SPDX-License-Identifier: GPL-2.0+
/*
 * PHY driver for MXL86110 and MXL86111
 *
 * V1.0.0
 *
 * Copyright 2023 MaxLinear Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/etherdevice.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/module.h>
#include <linux/bitfield.h>

#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#else
#define KERNEL_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))
#endif

#define MXL8611x_DRIVER_DESC	"MXL86110 PHY driver"
#define MXL8611x_DRIVER_VER		"1.0.0"

/* PHY IDs */
#define PHY_ID_MXL86110		0xC1335580
#define PHY_ID_MXL86111		0xC1335588

/* required to access extended registers */
#define MXL8611X_EXTD_REG_ADDR_OFFSET	0x1E
#define MXL8611X_EXTD_REG_ADDR_DATA		0x1F
#define PHY_IRQ_ENABLE_REG				0x12
#define PHY_IRQ_ENABLE_REG_WOL			BIT(6)

/* only 1 page for MXL86110 */
#define MXL86110_DEFAULT_PAGE	0

/* different pages for EXTD access for MXL86111*/
/* SerDes/PHY Control Access Register - COM_EXT_SMI_SDS_PHY */
#define MXL86111_EXT_SMI_SDS_PHY_REG				0xA000
#define MXL86111_EXT_SMI_SDS_PHYSPACE_MASK			BIT(1)
#define MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE			(0x1 << 1)
#define MXL86111_EXT_SMI_SDS_PHYUTP_SPACE			(0x0 << 1)
#define MXL86111_EXT_SMI_SDS_PHY_AUTO	(0xFF)

/* SyncE Configuration Register - COM_EXT SYNCE_CFG */
#define MXL8611X_EXT_SYNCE_CFG_REG						0xA012
#define MXL8611X_EXT_SYNCE_CFG_CLK_FRE_SEL				BIT(4)
#define MXL8611X_EXT_SYNCE_CFG_EN_SYNC_E_DURING_LNKDN	BIT(5)
#define MXL8611X_EXT_SYNCE_CFG_EN_SYNC_E				BIT(6)
#define MXL8611X_EXT_SYNCE_CFG_CLK_SRC_SEL_MASK			GENMASK(3, 1)
#define MXL8611X_EXT_SYNCE_CFG_CLK_SRC_SEL_125M_PLL		0
#define MXL8611X_EXT_SYNCE_CFG_CLK_SRC_SEL_25M			4

/* WOL registers */
#define MXL86110_WOL_MAC_ADDR_HIGH_EXTD_REG		0xA007 /* high-> FF:FF                   */
#define MXL86110_WOL_MAC_ADDR_MIDDLE_EXTD_REG	0xA008 /*    middle-> :FF:FF <-middle    */
#define MXL86110_WOL_MAC_ADDR_LOW_EXTD_REG		0xA009 /*                   :FF:FF <-low */

#define MXL8611X_EXT_WOL_CFG_REG				0xA00A
#define MXL8611X_EXT_WOL_CFG_WOLE_MASK			BIT(3)
#define MXL8611X_EXT_WOL_CFG_WOLE_DISABLE		0
#define MXL8611X_EXT_WOL_CFG_WOLE_ENABLE		BIT(3)

/* RGMII register */
#define MXL8611X_EXT_RGMII_CFG1_REG							0xA003
/* delay can be adjusted in steps of about 150ps */
#define MXL8611X_EXT_RGMII_CFG1_NO_DELAY					0
#define MXL8611X_EXT_RGMII_CFG1_RX_DELAY_MAX				(0xF << 10)
#define MXL8611X_EXT_RGMII_CFG1_RX_DELAY_MIN				(0x1 << 10)
#define MXL8611X_EXT_RGMII_CFG1_RX_DELAY_DEFAULT			(0xF << 10)

#define MXL8611X_EXT_RGMII_CFG1_RX_DELAY_MASK				GENMASK(13, 10)
#define MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_MAX				(0xF << 0)
#define MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_MIN				(0x1 << 0)
#define MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_MASK			GENMASK(3, 0)
#define MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_DEFAULT			(0x1 << 0)

#define MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_DELAY_MAX		(0xF << 4)
#define MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_DELAY_MIN		(0x1 << 4)
#define MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_DEFAULT		(0xF << 4)
#define MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_DELAY_MASK	GENMASK(7, 4)

#define MXL8611X_EXT_RGMII_CFG1_FULL_MASK \
			((MXL8611X_EXT_RGMII_CFG1_RX_DELAY_MASK) | \
			(MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_MASK) | \
			(MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_DELAY_MASK))

/* EXT Sleep Control register */
#define MXL8611x_UTP_EXT_SLEEP_CTRL_REG					0x27
#define MXL8611x_UTP_EXT_SLEEP_CTRL_EN_SLEEP_SW_OFF		0
#define MXL8611x_UTP_EXT_SLEEP_CTRL_EN_SLEEP_SW_MASK	BIT(15)

/* LED registers and defines */
#define MXL8611X_LED0_CFG_REG 0xA00C
#define MXL8611X_LED1_CFG_REG 0xA00D
#define MXL8611X_LED2_CFG_REG 0xA00E

#define MXL8611X_LEDX_CFG_TRAFFIC_ACT_BLINK_IND		BIT(13)
#define MXL8611X_LEDX_CFG_LINK_UP_FULL_DUPLEX_ON	BIT(12)
#define MXL8611X_LEDX_CFG_LINK_UP_HALF_DUPLEX_ON	BIT(11)
#define MXL8611X_LEDX_CFG_LINK_UP_TX_ACT_ON			BIT(10)	/* LED 0,1,2 default */
#define MXL8611X_LEDX_CFG_LINK_UP_RX_ACT_ON			BIT(9)	/* LED 0,1,2 default */
#define MXL8611X_LEDX_CFG_LINK_UP_TX_ON				BIT(8)
#define MXL8611X_LEDX_CFG_LINK_UP_RX_ON				BIT(7)
#define MXL8611X_LEDX_CFG_LINK_UP_1GB_ON			BIT(6) /* LED 2 default */
#define MXL8611X_LEDX_CFG_LINK_UP_100MB_ON			BIT(5) /* LED 1 default */
#define MXL8611X_LEDX_CFG_LINK_UP_10MB_ON			BIT(4) /* LED 0 default */
#define MXL8611X_LEDX_CFG_LINK_UP_COLLISION			BIT(3)
#define MXL8611X_LEDX_CFG_LINK_UP_1GB_BLINK			BIT(2)
#define MXL8611X_LEDX_CFG_LINK_UP_100MB_BLINK		BIT(1)
#define MXL8611X_LEDX_CFG_LINK_UP_10MB_BLINK		BIT(0)

#define MXL8611X_LED_BLINK_CFG_REG						0xA00F
#define MXL8611X_LED_BLINK_CFG_FREQ_MODE1_2HZ			0
#define MXL8611X_LED_BLINK_CFG_FREQ_MODE1_4HZ			BIT(0)
#define MXL8611X_LED_BLINK_CFG_FREQ_MODE1_8HZ			BIT(1)
#define MXL8611X_LED_BLINK_CFG_FREQ_MODE1_16HZ			(BIT(1) | BIT(0))
#define MXL8611X_LED_BLINK_CFG_FREQ_MODE2_2HZ			0
#define MXL8611X_LED_BLINK_CFG_FREQ_MODE2_4HZ			BIT(2)
#define MXL8611X_LED_BLINK_CFG_FREQ_MODE2_8HZ			BIT(3)
#define MXL8611X_LED_BLINK_CFG_FREQ_MODE2_16HZ			(BIT(3) | BIT(2))
#define MXL8611X_LED_BLINK_CFG_DUTY_CYCLE_50_PERC_ON	0
#define MXL8611X_LED_BLINK_CFG_DUTY_CYCLE_67_PERC_ON	(BIT(4))
#define MXL8611X_LED_BLINK_CFG_DUTY_CYCLE_75_PERC_ON	(BIT(5))
#define MXL8611X_LED_BLINK_CFG_DUTY_CYCLE_83_PERC_ON	(BIT(5) | BIT(4))
#define MXL8611X_LED_BLINK_CFG_DUTY_CYCLE_50_PERC_OFF	(BIT(6))
#define MXL8611X_LED_BLINK_CFG_DUTY_CYCLE_33_PERC_ON	(BIT(6) | BIT(4))
#define MXL8611X_LED_BLINK_CFG_DUTY_CYCLE_25_PERC_ON	(BIT(6) | BIT(5))
#define MXL8611X_LED_BLINK_CFG_DUTY_CYCLE_17_PERC_ON	(BIT(6) | BIT(5) | BIT(4))

/* Specific Status Register - PHY_STAT */
#define MXL86111_PHY_STAT_REG				0x11
#define MXL86111_PHY_STAT_SPEED_MASK		GENMASK(15, 14)
#define MXL86111_PHY_STAT_SPEED_OFFSET		14
#define MXL86111_PHY_STAT_SPEED_10M			0x0
#define MXL86111_PHY_STAT_SPEED_100M		0x1
#define MXL86111_PHY_STAT_SPEED_1000M		0x2
#define MXL86111_PHY_STAT_DPX_OFFSET		13
#define MXL86111_PHY_STAT_DPX				BIT(13)
#define MXL86111_PHY_STAT_LSRT				BIT(10)

/* 3 phy reg page modes,auto mode combines utp and fiber mode*/
#define MXL86111_MODE_FIBER					0x1
#define MXL86111_MODE_UTP					0x2
#define MXL86111_MODE_AUTO					0x3

/* FIBER Auto-Negotiation link partner ability - SDS_AN_LPA */
#define MXL86111_SDS_AN_LPA_PAUSE			(0x3 << 7)
#define MXL86111_SDS_AN_LPA_ASYM_PAUSE		(0x2 << 7)

/* Chip Configuration Register - COM_EXT_CHIP_CFG */
#define MXL86111_EXT_CHIP_CFG_REG			0xA001
#define MXL86111_EXT_CHIP_CFG_RXDLY_ENABLE	BIT(8)
#define MXL86111_EXT_CHIP_CFG_SW_RST_N_MODE	BIT(15)

#define MXL86111_EXT_CHIP_CFG_MODE_SEL_MASK				GENMASK(2, 0)
#define MXL86111_EXT_CHIP_CFG_MODE_UTP_TO_RGMII			0
#define MXL86111_EXT_CHIP_CFG_MODE_FIBER_TO_RGMII		1
#define MXL86111_EXT_CHIP_CFG_MODE_UTP_FIBER_TO_RGMII	2
#define MXL86111_EXT_CHIP_CFG_MODE_UTP_TO_SGMII			3
#define MXL86111_EXT_CHIP_CFG_MODE_SGPHY_TO_RGMAC		4
#define MXL86111_EXT_CHIP_CFG_MODE_SGMAC_TO_RGPHY		5
#define MXL86111_EXT_CHIP_CFG_MODE_UTP_TO_FIBER_AUTO	6
#define MXL86111_EXT_CHIP_CFG_MODE_UTP_TO_FIBER_FORCE	7

/* Miscellaneous Control Register - COM_EXT _MISC_CFG */
#define MXL86111_EXT_MISC_CONFIG_REG					0xA006
#define MXL86111_EXT_MISC_CONFIG_FIB_SPEED_SEL			BIT(0)
#define MXL86111_EXT_MISC_CONFIG_FIB_SPEED_SEL_1000BX	(0x1 << 0)
#define MXL86111_EXT_MISC_CONFIG_FIB_SPEED_SEL_100BX	(0x0 << 0)

/* Phy fiber Link timer cfg2 Register - EXT_SDS_LINK_TIMER_CFG2 */
#define MXL86111_EXT_SDS_LINK_TIMER_CFG2_REG			0xA5
#define MXL86111_EXT_SDS_LINK_TIMER_CFG2_EN_AUTOSEN		BIT(15)

/* default values of PHY register, required for Dual Media mode */
#define MII_BMSR_DEFAULT_VAL			0x7949
#define MII_ESTATUS_DEFAULT_VAL			0x2000

/* Timeout in ms for PHY SW reset check in STD_CTRL/SDS_CTRL */
#define BMCR_RESET_TIMEOUT		500

/* ******************************************************** */
/* Customer specific configuration START					*/
/* Adapt here if other than default values are required!	*/
/* ******************************************************** */

/* disable auto sleep feature */
#define MXL8611x_UTP_DISABLE_AUTO_SLEEP_FEATURE_CUSTOM		0

/* SYNCE/clkout feature */
#define MXL8611X_CLOCK_DISABLE							0
#define MXL8611X_CLOCK_FREQ_25M							1
#define MXL8611X_CLOCK_FREQ_125M						2
#define MXL8611X_CLOCK_DEFAULT							3

#define MXL8611X_EXT_SYNCE_CFG_CLOCK_FREQ_CUSTOM	MXL8611X_CLOCK_DEFAULT

/* Adjust RGMII timing based on min/max range defined above */
#define MXL8611X_EXT_RGMII_CFG1_RX_DELAY_CUSTOM \
			MXL8611X_EXT_RGMII_CFG1_RX_DELAY_DEFAULT
#define MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_CUSTOM	\
			MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_DEFAULT
#define MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_CUSTOM \
			MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_DEFAULT

/* ******************************************************** */
/* Customer specific configuration END						*/
/* ******************************************************** */

/**
 * mxlphy_write_extended_reg() - write to a PHY's extended register
 * @phydev: pointer to a &struct phy_device
 * @regnum: register number to write
 * @val: value to write to @regnum
 *
 * NOTE:The calling function must have taken the MDIO bus lock.
 *
 * returns 0 or negative error code
 */
static int mxlphy_write_extended_reg(struct phy_device *phydev, u16 regnum, u16 val)
{
	int ret;

	ret = __phy_write(phydev, MXL8611X_EXTD_REG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	return __phy_write(phydev, MXL8611X_EXTD_REG_ADDR_DATA, val);
}

/**
 * mxlphy_read_extended_reg() - write to a PHY's extended register
 * @phydev: pointer to a &struct phy_device
 * @regnum: register number to write
 * @val: value to write to @regnum
 *
 * NOTE:The calling function must have taken the MDIO bus lock.
 *
 * returns the value of regnum reg or negative error code
 */
static int mxlphy_read_extended_reg(struct phy_device *phydev, u16 regnum)
{
	int ret;

	ret = __phy_write(phydev, MXL8611X_EXTD_REG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;
	return __phy_read(phydev, MXL8611X_EXTD_REG_ADDR_DATA);
}

/**
 * mxlphy_read_extended_reg() - write to a PHY's extended register
 * @phydev: pointer to a &struct phy_device
 * @regnum: register number to write
 * @val: value to write to @regnum
 *
 * returns the value of regnum reg or negative error code
 */
static int mxlphy_locked_read_extended_reg(struct phy_device *phydev, u16 regnum)
{
	int ret;

	mutex_lock(&phydev->mdio.bus->mdio_lock);
	ret = mxlphy_read_extended_reg(phydev, regnum);
	mutex_unlock(&phydev->mdio.bus->mdio_lock);

	return ret;
}

/**
 * mxlphy_modify_extended_reg() - modify bits of a PHY's extended register
 * @phydev: pointer to the phy_device
 * @regnum: register number to write
 * @mask: bit mask of bits to clear
 * @set: bit mask of bits to set
 *
 * NOTE: register value = (old register value & ~mask) | set.
 * The caller must have taken the MDIO bus lock.
 *
 * returns 0 or negative error code
 */
static int mxlphy_modify_extended_reg(struct phy_device *phydev, u16 regnum, u16 mask,
			    u16 set)
{
	int ret;

	ret = __phy_write(phydev, MXL8611X_EXTD_REG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	return __phy_modify(phydev, MXL8611X_EXTD_REG_ADDR_DATA, mask, set);
}

/**
 * mxlphy_locked_modify_extended_reg() - modify bits of a PHY's extended register
 * @phydev: pointer to the phy_device
 * @regnum: register number to write
 * @mask: bit mask of bits to clear
 * @set: bit mask of bits to set
 *
 * NOTE: register value = (old register value & ~mask) | set.
 *
 * returns 0 or negative error code
 */
static int mxlphy_locked_modify_extended_reg(struct phy_device *phydev, u16 regnum,
				      u16 mask, u16 set)
{
	int ret;

	mutex_lock(&phydev->mdio.bus->mdio_lock);
	ret = mxlphy_modify_extended_reg(phydev, regnum, mask, set);
	mutex_unlock(&phydev->mdio.bus->mdio_lock);

	return ret;
}

/**
 * mxlphy_get_wol() - report if wake-on-lan is enabled
 * @phydev: pointer to the phy_device
 * @wol: a pointer to a &struct ethtool_wolinfo
 */
static void mxlphy_get_wol(struct phy_device *phydev, struct ethtool_wolinfo *wol)
{
	int value;

	wol->supported = WAKE_MAGIC;
	wol->wolopts = 0;
	value = mxlphy_locked_read_extended_reg(phydev, MXL8611X_EXT_WOL_CFG_REG);
	if ((value >= 0) && (value & MXL8611X_EXT_WOL_CFG_WOLE_MASK))
		wol->wolopts |= WAKE_MAGIC;
}

/**
 * mxlphy_set_wol() - enable/disable wake-on-lan
 * @phydev: pointer to the phy_device
 * @wol: a pointer to a &struct ethtool_wolinfo
 *
 * Configures the WOL Magic Packet MAC
 * returns 0 or negative errno code
 */
static int mxlphy_set_wol(struct phy_device *phydev, struct ethtool_wolinfo *wol)
{
	struct net_device *netdev;
	int page_to_restore;
	const u8 *mac;
	int ret = 0;

	if (wol->wolopts & WAKE_MAGIC) {
		netdev = phydev->attached_dev;
		if (!netdev)
			return -ENODEV;

		mac = (const u8 *)netdev->dev_addr;
		if (!is_valid_ether_addr(mac))
			return -EINVAL;

		page_to_restore = phy_select_page(phydev, MXL86110_DEFAULT_PAGE);
		if (page_to_restore < 0)
			goto error;

		/* Configure the MAC address of the WOL magic packet */
		ret = mxlphy_write_extended_reg(phydev, MXL86110_WOL_MAC_ADDR_HIGH_EXTD_REG,
										((mac[0] << 8) | mac[1]));
		if (ret < 0)
			goto error;
		ret = mxlphy_write_extended_reg(phydev, MXL86110_WOL_MAC_ADDR_MIDDLE_EXTD_REG,
										((mac[2] << 8) | mac[3]));
		if (ret < 0)
			goto error;
		ret = mxlphy_write_extended_reg(phydev, MXL86110_WOL_MAC_ADDR_LOW_EXTD_REG,
										((mac[4] << 8) | mac[5]));
		if (ret < 0)
			goto error;

		ret = mxlphy_modify_extended_reg(phydev, MXL8611X_EXT_WOL_CFG_REG, MXL8611X_EXT_WOL_CFG_WOLE_MASK,
				 MXL8611X_EXT_WOL_CFG_WOLE_ENABLE);
		if (ret < 0)
			goto error;

		ret = __phy_modify(phydev, PHY_IRQ_ENABLE_REG, 0,
				   PHY_IRQ_ENABLE_REG_WOL);
		if (ret < 0)
			goto error;

		phydev_info(phydev, "%s, WOL Magic packet MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
				__func__, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	} else {
		page_to_restore = phy_select_page(phydev, MXL86110_DEFAULT_PAGE);
		if (page_to_restore < 0)
			goto error;

		ret = mxlphy_modify_extended_reg(phydev, MXL8611X_EXT_WOL_CFG_REG, MXL8611X_EXT_WOL_CFG_WOLE_MASK,
				MXL8611X_EXT_WOL_CFG_WOLE_DISABLE);

		ret = __phy_modify(phydev, PHY_IRQ_ENABLE_REG,
				   PHY_IRQ_ENABLE_REG_WOL, 0);
		if (ret < 0)
			goto error;
	}

error:
	return phy_restore_page(phydev, page_to_restore, ret);
}

/**
 * mxl86110_read_page() - read reg page
 * @phydev: pointer to the phy_device
 *
 * returns current reg space of MxL86110
 * (only MXL86111_EXT_SMI_SDS_PHYUTP_SPACE supported) or negative errno code
 */
static int mxl86110_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, MXL8611X_EXTD_REG_ADDR_OFFSET);
};

/**
 * mxl86110_write_page() - write reg page
 * @phydev: pointer to the phy_device
 * @page: The reg page to write
 *
 * returns current reg space of MxL86110
 * (only MXL86111_EXT_SMI_SDS_PHYUTP_SPACE supported) or negative errno code
 */
static int mxl86110_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, MXL8611X_EXTD_REG_ADDR_OFFSET, page);
};

/**
 * mxl8611x_led_cfg() - applies LED configuration from device tree
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl8611x_led_cfg(struct phy_device *phydev)
{
	int ret = 0;
	int i;
	char propname[25];
	struct device_node *node = phydev->mdio.dev.of_node;
	u32 val;

	/* Loop through three the LED registers */
	for (i = 0; i < 3; i++) {
		/* Read property from device tree */
		snprintf(propname, 25, "mxl-8611x,led%d_cfg", i);
		if (of_property_read_u32(node, propname, &val))
			continue;

		/* Update PHY LED register */
		ret = mxlphy_write_extended_reg(phydev, MXL8611X_LED0_CFG_REG + i, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/**
 * mxl8611x_synce_clk_cfg() - applies syncE/clk output configuration
 * @phydev: pointer to the phy_device
 *
 * Custom settings can be defined in custom config section of the driver
 * returns 0 or negative errno code
 */
static int mxl8611x_synce_clk_cfg(struct phy_device *phydev)
{
	u16 mask = 0, value = 0;
	int ret;

	switch (MXL8611X_EXT_SYNCE_CFG_CLOCK_FREQ_CUSTOM) {
	case MXL8611X_CLOCK_DISABLE:
		mask = MXL8611X_EXT_SYNCE_CFG_EN_SYNC_E;
		value = 0;
		break;
	case MXL8611X_CLOCK_FREQ_25M:
		value = MXL8611X_EXT_SYNCE_CFG_EN_SYNC_E |
				FIELD_PREP(MXL8611X_EXT_SYNCE_CFG_CLK_SRC_SEL_MASK,
				MXL8611X_EXT_SYNCE_CFG_CLK_SRC_SEL_25M);
		mask = MXL8611X_EXT_SYNCE_CFG_EN_SYNC_E |
		       MXL8611X_EXT_SYNCE_CFG_CLK_SRC_SEL_MASK |
		       MXL8611X_EXT_SYNCE_CFG_CLK_FRE_SEL;
		break;
	case MXL8611X_CLOCK_FREQ_125M:
		value = MXL8611X_EXT_SYNCE_CFG_EN_SYNC_E |
				MXL8611X_EXT_SYNCE_CFG_CLK_FRE_SEL |
				FIELD_PREP(MXL8611X_EXT_SYNCE_CFG_CLK_SRC_SEL_MASK,
				MXL8611X_EXT_SYNCE_CFG_CLK_SRC_SEL_125M_PLL);
		mask = MXL8611X_EXT_SYNCE_CFG_EN_SYNC_E |
		       MXL8611X_EXT_SYNCE_CFG_CLK_SRC_SEL_MASK |
		       MXL8611X_EXT_SYNCE_CFG_CLK_FRE_SEL;
		break;
	case MXL8611X_CLOCK_DEFAULT:
		phydev_info(phydev, "%s, default clock cfg\n", __func__);
		return 0;
	default:
		phydev_info(phydev, "%s, invalid clock cfg: %d\n", __func__, MXL8611X_EXT_SYNCE_CFG_CLOCK_FREQ_CUSTOM);
		return -EINVAL;
	}

	phydev_info(phydev, "%s, clock cfg mask:%d, value: %d\n", __func__, mask, value);

	/* Write clock output configuration */
	ret = mxlphy_locked_modify_extended_reg(phydev, MXL8611X_EXT_SYNCE_CFG_REG,
					 mask, value);
	if (ret < 0)
		return ret;

	return 0;
}

/**
 * mxl86110_config_init() - initialize the PHY
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl86110_config_init(struct phy_device *phydev)
{
	int page_to_restore, ret = 0;
	unsigned int val = 0;
	bool disable_rxdly = false;

	page_to_restore = phy_select_page(phydev, MXL86110_DEFAULT_PAGE);
	if (page_to_restore < 0)
		goto error;

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII:
		/* no delay, will write 0 */
		val = MXL8611X_EXT_RGMII_CFG1_NO_DELAY;
		disable_rxdly = true;
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		val = MXL8611X_EXT_RGMII_CFG1_RX_DELAY_CUSTOM;
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val = MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_CUSTOM |
				MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_CUSTOM;
		disable_rxdly = true;
		break;
	case PHY_INTERFACE_MODE_RGMII_ID:
		val = MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_CUSTOM |
				MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_CUSTOM;
		val |= MXL8611X_EXT_RGMII_CFG1_RX_DELAY_CUSTOM;
		break;
	default:
		ret = -EINVAL;
		goto error;
	}
	ret = mxlphy_modify_extended_reg(phydev, MXL8611X_EXT_RGMII_CFG1_REG,
					MXL8611X_EXT_RGMII_CFG1_FULL_MASK, val);
	if (ret < 0)
		goto error;

	if (ret < 0)
		goto error;

	if (MXL8611x_UTP_DISABLE_AUTO_SLEEP_FEATURE_CUSTOM == 1) {
		/* disable auto sleep */
		ret = mxlphy_modify_extended_reg(phydev, MXL8611x_UTP_EXT_SLEEP_CTRL_REG,
					MXL8611x_UTP_EXT_SLEEP_CTRL_EN_SLEEP_SW_MASK,
					MXL8611x_UTP_EXT_SLEEP_CTRL_EN_SLEEP_SW_OFF);
		if (ret < 0)
			goto error;
	}

	/* Disable RXDLY (RGMII Rx Clock Delay) */
	if (disable_rxdly)
	{
		ret = mxlphy_modify_extended_reg(phydev, MXL86111_EXT_CHIP_CFG_REG,
						 MXL86111_EXT_CHIP_CFG_RXDLY_ENABLE, 0);
		if (ret < 0)
			goto error;
	}

	ret = mxl8611x_led_cfg(phydev);
	if (ret < 0)
		goto error;

error:
	return phy_restore_page(phydev, page_to_restore, ret);
}

struct mxl86111_priv {
	/* dual_media_advertising used for Dual Media mode (MXL86111_EXT_SMI_SDS_PHY_AUTO) */
	__ETHTOOL_DECLARE_LINK_MODE_MASK(dual_media_advertising);

	/* MXL86111_MODE_FIBER / MXL86111_MODE_UTP / MXL86111_MODE_AUTO*/
	u8 reg_page_mode;
	u8 strap_mode; /* 8 working modes  */
	/* current reg page of mxl86111 phy:
	 * MXL86111_EXT_SMI_SDS_PHYUTP_SPACE
	 * MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE
	 * MXL86111_EXT_SMI_SDS_PHY_AUTO
	 */
	u8 reg_page;
};

/**
 * mxl86110_probe() - read chip config then set suitable reg_page_mode
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl86110_probe(struct phy_device *phydev)
{
	int ret;

	/* configure syncE / clk output */
	ret = mxl8611x_synce_clk_cfg(phydev);
	if (ret < 0)
		return ret;

	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
/**
 * mii_bmcr_encode_fixed - encode fixed speed/duplex settings to a BMCR value
 * @speed: a SPEED_* value
 * @duplex: a DUPLEX_* value
 *
 * Encode the speed and duplex to a BMCR value. 2500, 1000, 100 and 10 Mbps are
 * supported. 2500Mbps is encoded to 1000Mbps. Other speeds are encoded as 10
 * Mbps. Unknown duplex values are encoded to half-duplex.
 *
 * NOTE: this function is only needed for kernel versions < 6.0.0, since it is
 * defined in new kernel versions anyway.
 */
static inline u16 mii_bmcr_encode_fixed(int speed, int duplex)
{
	u16 bmcr;

	switch (speed) {
	case SPEED_2500:
	case SPEED_1000:
		bmcr = BMCR_SPEED1000;
		break;

	case SPEED_100:
		bmcr = BMCR_SPEED100;
		break;

	case SPEED_10:
	default:
		bmcr = BMCR_SPEED10;
		break;
	}

	if (duplex == DUPLEX_FULL)
		bmcr |= BMCR_FULLDPLX;

	return bmcr;
}
#endif

/**
 * mxlphy_config_eee_advert - disable unwanted eee mode advertisement
 * @phydev: pointer to the phy_device
 *
 * NOTE: The caller must have taken the MDIO bus lock.
 *
 * identical to genphy_config_eee_advert, but __phy_modify_mmd_changed without mdio lock
 *
 * Writes MDIO_AN_EEE_ADV after disabling unsupported energy
 * efficent ethernet modes. Returns 0 if the PHY's advertisement hasn't
 * changed, and 1 if it has changed.
 */
int mxlphy_config_eee_advert(struct phy_device *phydev)
{
	int err;

	/* Nothing to disable */
	if (!phydev->eee_broken_modes)
		return 0;

	err = __phy_modify_mmd_changed(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV,
				     phydev->eee_broken_modes, 0);
	/* If the call failed, we assume that EEE is not supported */
	return err < 0 ? 0 : err;
}

static struct phy_driver mxl_phy_drvs[] = {
	{
		PHY_ID_MATCH_EXACT(PHY_ID_MXL86110),
		.name			= "MXL86110 Gigabit Ethernet",
		.probe			= mxl86110_probe,
		.config_init	= mxl86110_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_page		= mxl86110_read_page,
		.write_page		= mxl86110_write_page,
		.read_status	= genphy_read_status,
		.get_wol		= mxlphy_get_wol,
		.set_wol		= mxlphy_set_wol,
		.suspend		= genphy_suspend,
		.resume			= genphy_resume,
	}
};

module_phy_driver(mxl_phy_drvs);

MODULE_DESCRIPTION(MXL8611x_DRIVER_DESC);
MODULE_VERSION(MXL8611x_DRIVER_DESC);
MODULE_LICENSE("GPL");

static const struct mdio_device_id __maybe_unused mxl_tbl[] = {
	{ PHY_ID_MATCH_EXACT(PHY_ID_MXL86110) },
	{  }
};

MODULE_DEVICE_TABLE(mdio, mxl_tbl);
