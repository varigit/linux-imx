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
#include <linux/phy.h>
#include <linux/module.h>
#include <linux/bitfield.h>

#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#else
#define KERNEL_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))
#endif

#define MXL8611x_DRIVER_DESC	"MXL86111 and MXL86110 PHY driver"
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

/* Adjust default LED settings */
#define MXL8611X_LED0_CFG_CUSTOM		(MXL8611X_LEDX_CFG_LINK_UP_TX_ACT_ON | \
										MXL8611X_LEDX_CFG_LINK_UP_RX_ACT_ON | \
										MXL8611X_LEDX_CFG_LINK_UP_10MB_ON)
#define MXL8611X_LED1_CFG_CUSTOM		(MXL8611X_LEDX_CFG_LINK_UP_TX_ACT_ON | \
										MXL8611X_LEDX_CFG_LINK_UP_RX_ACT_ON | \
										MXL8611X_LEDX_CFG_LINK_UP_100MB_ON)
#define MXL8611X_LED2_CFG_CUSTOM		(MXL8611X_LEDX_CFG_LINK_UP_TX_ACT_ON | \
										MXL8611X_LEDX_CFG_LINK_UP_RX_ACT_ON | \
										MXL8611X_LEDX_CFG_LINK_UP_1GB_ON)
#define MXL8611X_LED_BLINK_CFG_CUSTOM	(MXL8611X_LED_BLINK_CFG_FREQ_MODE1_8HZ | \
										MXL8611X_LED_BLINK_CFG_FREQ_MODE2_4HZ | \
										MXL8611X_LED_BLINK_CFG_DUTY_CYCLE_50_PERC_ON)

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
 * mxlphy_locked_write_extended_reg() - write to a PHY's extended register
 * @phydev: pointer to a &struct phy_device
 * @regnum: register number to write
 * @val: value to write to @regnum
 *
 * returns 0 or negative error code
 */
static int mxlphy_locked_write_extended_reg(struct phy_device *phydev, u16 regnum,
				     u16 val)
{
	int ret;

	phy_lock_mdio_bus(phydev);
	ret = mxlphy_write_extended_reg(phydev, regnum, val);
	phy_unlock_mdio_bus(phydev);

	return ret;
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

	phy_lock_mdio_bus(phydev);
	ret = mxlphy_read_extended_reg(phydev, regnum);
	phy_unlock_mdio_bus(phydev);

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

	phy_lock_mdio_bus(phydev);
	ret = mxlphy_modify_extended_reg(phydev, regnum, mask, set);
	phy_unlock_mdio_bus(phydev);

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
 * mxl8611x_led_cfg() - applies LED configuration
 * @phydev: pointer to the phy_device
 *
 * Custom settings can be defined in custom config section of the driver
 * returns 0 or negative errno code
 */
static int mxl8611x_led_cfg(struct phy_device *phydev)
{
	int ret = 0;

	/* Write default LED settings. Can be adapted if target platform requires
	 * different LED behaviour
	 */
	ret = mxlphy_write_extended_reg(phydev, MXL8611X_LED0_CFG_REG,
									MXL8611X_LED0_CFG_CUSTOM);
	if (ret < 0)
		return ret;

	ret = mxlphy_write_extended_reg(phydev, MXL8611X_LED1_CFG_REG,
									MXL8611X_LED1_CFG_CUSTOM);
	if (ret < 0)
		return ret;

	ret = mxlphy_write_extended_reg(phydev, MXL8611X_LED2_CFG_REG,
									MXL8611X_LED2_CFG_CUSTOM);
	if (ret < 0)
		return ret;

	ret = mxlphy_write_extended_reg(phydev, MXL8611X_LED_BLINK_CFG_REG,
									MXL8611X_LED_BLINK_CFG_CUSTOM);
	return ret;
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

	page_to_restore = phy_select_page(phydev, MXL86110_DEFAULT_PAGE);
	if (page_to_restore < 0)
		goto error;

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII:
		/* no delay, will write 0 */
		val = MXL8611X_EXT_RGMII_CFG1_NO_DELAY;
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		val = MXL8611X_EXT_RGMII_CFG1_RX_DELAY_CUSTOM;
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val = MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_CUSTOM |
				MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_CUSTOM;
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
 * mxl86111_read_page() - read reg page
 * @phydev: pointer to the phy_device
 *
 * returns current reg space of mxl86111 (MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE/
 * MXL86111_EXT_SMI_SDS_PHYUTP_SPACE) or negative errno code
 */
static int mxl86111_read_page(struct phy_device *phydev)
{
	int old_page;

	old_page = mxlphy_read_extended_reg(phydev, MXL86111_EXT_SMI_SDS_PHY_REG);
	if (old_page < 0)
		return old_page;

	if ((old_page & MXL86111_EXT_SMI_SDS_PHYSPACE_MASK) == MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE)
		return MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE;

	return MXL86111_EXT_SMI_SDS_PHYUTP_SPACE;
};

/**
 * mxl86111_locked_read_page() - read reg page
 * @phydev: pointer to the phy_device
 *
 * returns current reg space of mxl86111 (MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE/
 * MXL86111_EXT_SMI_SDS_PHYUTP_SPACE) or negative errno code
 */
static int mxl86111_locked_read_page(struct phy_device *phydev)
{
	int old_page;

	phy_lock_mdio_bus(phydev);
	old_page = mxl86111_read_page(phydev);
	phy_unlock_mdio_bus(phydev);

	return old_page;
};

/**
 * mxl86111_write_page() - Set reg page
 * @phydev: pointer to the phy_device
 * @page: The reg page to set (MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE/MXL86111_EXT_SMI_SDS_PHYUTP_SPACE)
 *
 * returns 0 or negative errno code
 */
static int mxl86111_write_page(struct phy_device *phydev, int page)
{
	int mask = MXL86111_EXT_SMI_SDS_PHYSPACE_MASK;
	int set;

	if ((page & MXL86111_EXT_SMI_SDS_PHYSPACE_MASK) == MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE)
		set = MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE;
	else
		set = MXL86111_EXT_SMI_SDS_PHYUTP_SPACE;

	return mxlphy_modify_extended_reg(phydev, MXL86111_EXT_SMI_SDS_PHY_REG, mask, set);
};

/**
 * mxl86111_modify_bmcr_paged - modify bits of the PHY's BMCR register of a given page
 * @phydev: pointer to the phy_device
 * @page: The reg page to operate (MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE/MXL86111_EXT_SMI_SDS_PHYUTP_SPACE)
 * @mask: bit mask of bits to clear
 * @set: bit mask of bits to set
 *
 * NOTE: new register value = (old register value & ~mask) | set.
 * MxL86111 has 2 register spaces (utp/fiber) and 3 modes (utp/fiber/auto).
 * Each space has its MII_BMCR.
 *
 * returns 0 or negative errno code
 */
static int mxl86111_modify_bmcr_paged(struct phy_device *phydev, int page,
				    u16 mask, u16 set)
{
	int bmcr_timeout = BMCR_RESET_TIMEOUT;
	int page_to_restore;
	int ret = 0;

	page_to_restore = phy_select_page(phydev, page & MXL86111_EXT_SMI_SDS_PHYSPACE_MASK);
	if (page_to_restore < 0)
		goto err_restore_page;

	ret = __phy_modify(phydev, MII_BMCR, mask, set);
	if (ret < 0)
		goto err_restore_page;

	/* In case of BMCR_RESET, check until reset bit is cleared */
	if (set == BMCR_RESET) {
		while (bmcr_timeout--) {
			usleep_range(1000, 1050);
			ret = __phy_read(phydev, MII_BMCR);
			if (ret < 0)
				goto err_restore_page;

			if (!(ret & BMCR_RESET))
				return phy_restore_page(phydev, page_to_restore, 0);
		}
		phydev_warn(phydev, "%s, BMCR reset not completed until timeout", __func__);
		ret = -EBUSY;
	}

err_restore_page:
	return phy_restore_page(phydev, page_to_restore, ret);
}

/**
 * mxl86111_modify_bmcr - modify bits of a PHY's BMCR register
 * @phydev: pointer to the phy_device
 * @mask: bit mask of bits to clear
 * @set: bit mask of bits to set
 *
 * NOTE: new register value = (old register value & ~mask) | set.
 * MxL86111 has 2 register spaces (utp/fiber) and 3 mode (utp/fiber/poll)
 * each space has its MII_BMCR. poll mode combines utp and fiber.
 *
 * returns 0 or negative errno code
 */
static int mxl86111_modify_bmcr(struct phy_device *phydev, u16 mask, u16 set)
{
	struct mxl86111_priv *priv = phydev->priv;
	int ret = 0;

	if (priv->reg_page != MXL86111_EXT_SMI_SDS_PHY_AUTO) {
		ret = mxl86111_modify_bmcr_paged(phydev, priv->reg_page, mask, set);
		if (ret < 0)
			return ret;
	} else {
		ret = mxl86111_modify_bmcr_paged(phydev, ret, mask, set);
		if (ret < 0)
			return ret;
	}
	return 0;
}

/**
 * mxl86111_set_fiber_features() -  setup fiber mode features.
 * @phydev: pointer to the phy_device
 * @dst: a pointer to store fiber mode features
 */
static void mxl86111_set_fiber_features(struct phy_device *phydev,
					  unsigned long *dst)
{
	linkmode_set_bit(ETHTOOL_LINK_MODE_100baseFX_Full_BIT, dst);
	linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseX_Full_BIT, dst);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, dst);
	linkmode_set_bit(ETHTOOL_LINK_MODE_FIBRE_BIT, dst);
}

/**
 * mxlphy_utp_read_abilities - read PHY abilities from Clause 22 registers
 * @phydev: pointer to the phy_device
 *
 * NOTE: Read the PHY abilities and set phydev->supported.
 * The caller must have taken the MDIO bus lock.
 *
 * returns 0 or negative errno code
 */
static int mxlphy_utp_read_abilities(struct phy_device *phydev)
{
	struct mxl86111_priv *priv = phydev->priv;
	int val, page;

	linkmode_set_bit_array(phy_basic_ports_array,
			       ARRAY_SIZE(phy_basic_ports_array),
			       phydev->supported);

	val = __phy_read(phydev, MII_BMSR);
	if (val < 0)
		return val;

	/* In case of dual mode media, we might not have access to the utp page.
	 * Therefore we use the default register setting of the device to
	 * 'extract' supported modes.
	 */
	page = mxl86111_read_page(phydev);
	if (page < 0)
		return page;

	if ((priv->reg_page == MXL86111_EXT_SMI_SDS_PHY_AUTO) && (page == MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE))
		val = MII_BMSR_DEFAULT_VAL;

	phydev_info(phydev, "%s, MII_BMSR: 0x%04X\n", __func__, val);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, phydev->supported,
			 val & BMSR_ANEGCAPABLE);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT, phydev->supported,
			 val & BMSR_10FULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT, phydev->supported,
			 val & BMSR_10HALF);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, phydev->supported,
			 val & BMSR_100FULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT, phydev->supported,
			 val & BMSR_100HALF);

	if (val & BMSR_ESTATEN) {
		val = __phy_read(phydev, MII_ESTATUS);
		if (val < 0)
			return val;

	if ((priv->reg_page == MXL86111_EXT_SMI_SDS_PHY_AUTO) && (page == MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE))
		val = MII_ESTATUS_DEFAULT_VAL;

	phydev_info(phydev, "%s, MII_ESTATUS: 0x%04X\n", __func__, val);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
				 phydev->supported, val & ESTATUS_1000_TFULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
				 phydev->supported, val & ESTATUS_1000_THALF);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseX_Full_BIT,
			 phydev->supported, val & ESTATUS_1000_XFULL);
	}

	return 0;
}

/**
 * mxl86111_get_features_paged() -  read supported link modes for a given page
 * @phydev: pointer to the phy_device
 * @page: The reg page to operate (MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE/MXL86111_EXT_SMI_SDS_PHYUTP_SPACE)
 *
 * returns 0 or negative errno code
 */
static int mxl86111_get_features_paged(struct phy_device *phydev, int page)
{
	struct mxl86111_priv *priv = phydev->priv;
	int page_to_restore;
	int ret = 0;

	page &= MXL86111_EXT_SMI_SDS_PHYSPACE_MASK;
	page_to_restore = phy_select_page(phydev, page);
	if (page_to_restore < 0)
		goto err_restore_page;

	if ((page == MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE) && (priv->reg_page != MXL86111_EXT_SMI_SDS_PHY_AUTO)) {
		linkmode_zero(phydev->supported);
		mxl86111_set_fiber_features(phydev, phydev->supported);
	} else {
		ret = mxlphy_utp_read_abilities(phydev);
		if (ret < 0)
			goto err_restore_page;
	}

err_restore_page:
	return phy_restore_page(phydev, page_to_restore, ret);
}

/**
 * mxl86111_get_features - select the reg space then call mxl86111_get_features_paged
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl86111_get_features(struct phy_device *phydev)
{
	struct mxl86111_priv *priv = phydev->priv;
	int ret;

	if (priv->reg_page != MXL86111_EXT_SMI_SDS_PHY_AUTO) {
		ret = mxl86111_get_features_paged(phydev, priv->reg_page);
	} else {
		/* read current page in Dual Media mode, since PHY HW is controlling the page select */
		ret = mxl86111_locked_read_page(phydev);
		if (ret < 0)
			return ret;

		ret = mxl86111_get_features_paged(phydev, ret);
		if (ret < 0)
			return ret;

		/* add fiber mode features to phydev->supported */
		mxl86111_set_fiber_features(phydev, phydev->supported);
	}
	return ret;
}

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

/**
 * mxl86111_probe() - read chip config then set reg_page_mode, port and page
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl86111_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct mxl86111_priv *priv;
	int chip_config;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;

	// TM: Debugging of pinstrap mode
	ret = mxlphy_locked_modify_extended_reg(phydev, MXL86111_EXT_CHIP_CFG_REG,
					 MXL86111_EXT_CHIP_CFG_MODE_SEL_MASK, MXL86111_EXT_CHIP_CFG_MODE_UTP_TO_RGMII);
	if (ret < 0)
		return ret;
	// TM: Debugging of pinstrap mode
	ret = mxlphy_locked_modify_extended_reg(phydev, MXL86111_EXT_CHIP_CFG_REG,
				MXL86111_EXT_CHIP_CFG_SW_RST_N_MODE, 0);
	if (ret < 0)
		return ret;

	chip_config = mxlphy_locked_read_extended_reg(phydev, MXL86111_EXT_CHIP_CFG_REG);
	if (chip_config < 0)
		return chip_config;

	priv->strap_mode = chip_config & MXL86111_EXT_CHIP_CFG_MODE_SEL_MASK;
	switch (priv->strap_mode) {
	case MXL86111_EXT_CHIP_CFG_MODE_UTP_TO_SGMII:
	case MXL86111_EXT_CHIP_CFG_MODE_UTP_TO_RGMII:
		phydev->port = PORT_TP;
		priv->reg_page = MXL86111_EXT_SMI_SDS_PHYUTP_SPACE;
		priv->reg_page_mode = MXL86111_MODE_UTP;
		break;
	case MXL86111_EXT_CHIP_CFG_MODE_FIBER_TO_RGMII:
	case MXL86111_EXT_CHIP_CFG_MODE_SGPHY_TO_RGMAC:
	case MXL86111_EXT_CHIP_CFG_MODE_SGMAC_TO_RGPHY:
		phydev->port = PORT_FIBRE;
		priv->reg_page = MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE;
		priv->reg_page_mode = MXL86111_MODE_FIBER;
		break;
	case MXL86111_EXT_CHIP_CFG_MODE_UTP_FIBER_TO_RGMII:
	case MXL86111_EXT_CHIP_CFG_MODE_UTP_TO_FIBER_AUTO:
	case MXL86111_EXT_CHIP_CFG_MODE_UTP_TO_FIBER_FORCE:
		phydev->port = PORT_NONE;
		priv->reg_page = MXL86111_EXT_SMI_SDS_PHY_AUTO;
		priv->reg_page_mode = MXL86111_MODE_AUTO;
		break;

	}
	/* set default reg space when it is not Dual Media mode */
	if (priv->reg_page != MXL86111_EXT_SMI_SDS_PHY_AUTO) {
		ret = mxlphy_locked_write_extended_reg(phydev,
						MXL86111_EXT_SMI_SDS_PHY_REG,
						priv->reg_page);
		if (ret < 0)
			return ret;
	}

	phydev_info(phydev, "%s, pinstrap mode: %d\n", __func__, priv->strap_mode);

	/* configure syncE / clk output - can be defined in custom config section */
	ret = mxl8611x_synce_clk_cfg(phydev);
	if (ret < 0)
		return ret;

	return 0;
}

/**
 * mxlphy_check_and_restart_aneg - Enable and restart auto-negotiation
 * @phydev: pointer to the phy_device
 * @restart: bool if aneg restart is requested
 *
 * NOTE: The caller must have taken the MDIO bus lock.
 *
 * identical to genphy_check_and_restart_aneg, but phy_read without mdio lock
 *
 * returns 0 or negative errno code
 */
static int mxlphy_check_and_restart_aneg(struct phy_device *phydev, bool restart)
{
	int ret;

	if (!restart) {
		ret = __phy_read(phydev, MII_BMCR);
		if (ret < 0)
			return ret;

		/* Advertisement did not change, but aneg maybe was never to begin with
		 * or phy was in isolation state
		 */
		if (!(ret & BMCR_ANENABLE) || (ret & BMCR_ISOLATE))
			restart = true;
	}
	/* Enable and restart Autonegotiation */
	if (restart)
		return __phy_modify(phydev, MII_BMCR, BMCR_ISOLATE,
				    BMCR_ANENABLE | BMCR_ANRESTART);

	return 0;
}

/**
 * mxlphy_config_advert - sanitize and advertise auto-negotiation parameters
 * @phydev: pointer to the phy_device
 *
 * NOTE: Writes MII_ADVERTISE with the appropriate values,
 * Returns < 0 on error, 0 if the PHY's advertisement hasn't changed,
 * and > 0 if it has changed.
 *
 * identical to genphy_config_advert, but phy_read without mdio lock
 *
 * NOTE: The caller must have taken the MDIO bus lock.
 *
 * returns 0 or negative errno code
 */
static int mxlphy_config_advert(struct phy_device *phydev)
{
	int err, bmsr, changed = 0;
	u32 adv;

	/* Only allow advertising what this PHY supports */
	linkmode_and(phydev->advertising, phydev->advertising,
		     phydev->supported);

	adv = linkmode_adv_to_mii_adv_t(phydev->advertising);

	/* Setup standard advertisement */
	err = __phy_modify_changed(phydev, MII_ADVERTISE,
				   ADVERTISE_ALL | ADVERTISE_100BASE4 |
				   ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM,
				   adv);
	if (err < 0)
		return err;
	if (err > 0)
		changed = 1;

	bmsr = __phy_read(phydev, MII_BMSR);
	if (bmsr < 0)
		return bmsr;

	/* Per 802.3-2008, Section 22.2.4.2.16 Extended status all
	 * 1000Mbits/sec capable PHYs shall have the BMSR_ESTATEN bit set to a
	 * logical 1.
	 */
	if (!(bmsr & BMSR_ESTATEN))
		return changed;

	adv = linkmode_adv_to_mii_ctrl1000_t(phydev->advertising);

	err = __phy_modify_changed(phydev, MII_CTRL1000,
				   ADVERTISE_1000FULL | ADVERTISE_1000HALF,
				   adv);
	if (err < 0)
		return err;
	if (err > 0)
		changed = 1;

	return changed;
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
 * mxlphy_setup_master_slave - set master/slave capabilities
 * @phydev: pointer to the phy_device
 *
 * NOTE: The caller must have taken the MDIO bus lock.
 *
 * identical to genphy_setup_master_slave, but phy_read without mdio lock
 *
 * returns 0 or negative errno code
 */
static int mxlphy_setup_master_slave(struct phy_device *phydev)
{
	u16 ctl = 0;

	if (!phydev->is_gigabit_capable)

		return 0;

	switch (phydev->master_slave_set) {
	case MASTER_SLAVE_CFG_MASTER_PREFERRED:
		ctl |= CTL1000_PREFER_MASTER;
		break;
	case MASTER_SLAVE_CFG_SLAVE_PREFERRED:
		break;
	case MASTER_SLAVE_CFG_MASTER_FORCE:
		ctl |= CTL1000_AS_MASTER;
		fallthrough;
	case MASTER_SLAVE_CFG_SLAVE_FORCE:
		ctl |= CTL1000_ENABLE_MASTER;
		break;
	case MASTER_SLAVE_CFG_UNKNOWN:
	case MASTER_SLAVE_CFG_UNSUPPORTED:
		return 0;
	default:
		phydev_warn(phydev, "Unsupported Master/Slave mode\n");
		return -EOPNOTSUPP;
	}

	return __phy_modify_changed(phydev, MII_CTRL1000,
				    (CTL1000_ENABLE_MASTER | CTL1000_AS_MASTER |
				    CTL1000_PREFER_MASTER), ctl);
}

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

/**
 * mxlphy_config_aneg_utp - restart auto-negotiation or write BMCR
 * @phydev: pointer to the phy_device
 * @changed: bool if autoneg is requested
 *
 * If auto-negotiation is enabled, advertising will be configure,
 * and then auto-negotiation will be restarted. Otherwise write the BMCR.
 *
 * NOTE: The caller must have taken the MDIO bus lock.
 *
 * returns 0 or negative errno code
 */
static int mxlphy_config_aneg_utp(struct phy_device *phydev, bool changed)
{
	struct mxl86111_priv *priv = phydev->priv;
	int err;
	u16 ctl;

	/* identical to genphy_config_eee_advert, but __phy_modify_mmd_changed without mdio lock */
	if (mxlphy_config_eee_advert(phydev))
		changed = true;

	/* identical to genphy_setup_master_slave, but phy_read without mdio lock */
	err = mxlphy_setup_master_slave(phydev);
	if (err < 0)
		return err;
	else if (err)
		changed = true;

	if (phydev->autoneg != AUTONEG_ENABLE) {
		/* configures/forces speed/duplex, empty rate selection would result in 10M */
		ctl = mii_bmcr_encode_fixed(phydev->speed, phydev->duplex);
		/* In dual media mode prevent 1000BASE-T half duplex due to risk of permanent blocking*/
		if (priv->reg_page_mode == MXL86111_MODE_AUTO) {
			if (ctl & BMCR_SPEED1000) {
				phydev_err(phydev, "%s, Error: no supported fixed UTP rate configured - setting ignored\n", __func__);
				return -EINVAL;
			}
		}

		return __phy_modify(phydev, MII_BMCR, ~(BMCR_LOOPBACK |
				    BMCR_ISOLATE | BMCR_PDOWN), ctl);
	}

	/* At least one valid rate must be advertised in Dual Media mode, since it could
	 * lead to permanent blocking of the UTP path otherwise. The PHY HW is controlling
	 * the page select autonomously in this mode.
	 * If mode switches to Fiber reg page when a link partner is connected, while UTP
	 * link is down, it can never switch back to UTP if the link cannot be established
	 * anymore due to invalid configuration. A HW reset would be required to re-enable
	 * UTP mode. Therefore prevent this situation and ignore invalid speed configuration.
	 * Returning a negative return code (e.g. EINVAL) results in stack trace dumps from
	 * some PAL functions.
	 */
	if (priv->reg_page_mode == MXL86111_MODE_AUTO) {
		ctl = 0;
		if (linkmode_test_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT, phydev->advertising))
			ctl |= ADVERTISE_10HALF;
		if (linkmode_test_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT, phydev->advertising))
			ctl |= ADVERTISE_10FULL;
		if (linkmode_test_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT, phydev->advertising))
			ctl |= ADVERTISE_100HALF;
		if (linkmode_test_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, phydev->advertising))
			ctl |= ADVERTISE_100FULL;
		if (linkmode_test_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, phydev->advertising))
			ctl |= ADVERTISE_1000FULL;

		if (ctl == 0) {
			phydev_err(phydev, "%s, Error: no supported UTP rate configured - setting ignored\n", __func__);
			return -EINVAL;
		}
	}

	/* identical to genphy_config_advert, but phy_read without mdio lock */
	err = mxlphy_config_advert(phydev);
	if (err < 0)
		return err;
	else if (err)
		changed = true;

	return mxlphy_check_and_restart_aneg(phydev, changed);
}

/**
 * mxl86111_fiber_setup_forced - configures/forces speed for fiber mode
 * @phydev: pointer to the phy_device
 *
 * NOTE: The caller must have taken the MDIO bus lock.
 *
 * returns 0 or negative errno code
 */
static int mxl86111_fiber_setup_forced(struct phy_device *phydev)
{
	u16 val;
	int ret;

	if (phydev->speed == SPEED_1000)
		val = MXL86111_EXT_MISC_CONFIG_FIB_SPEED_SEL_1000BX;
	else if (phydev->speed == SPEED_100)
		val = MXL86111_EXT_MISC_CONFIG_FIB_SPEED_SEL_100BX;
	else
		return -EINVAL;

	ret =  __phy_modify(phydev, MII_BMCR, BMCR_ANENABLE, 0);
	if (ret < 0)
		return ret;

	ret =  mxlphy_modify_extended_reg(phydev, MXL86111_EXT_MISC_CONFIG_REG,
				MXL86111_EXT_MISC_CONFIG_FIB_SPEED_SEL, val);
	if (ret < 0)
		return ret;

	ret =  mxlphy_modify_extended_reg(phydev, MXL86111_EXT_SDS_LINK_TIMER_CFG2_REG,
				MXL86111_EXT_SDS_LINK_TIMER_CFG2_EN_AUTOSEN, 0);
	if (ret < 0)
		return ret;

	return mxlphy_modify_extended_reg(phydev, MXL86111_EXT_CHIP_CFG_REG,
				MXL86111_EXT_CHIP_CFG_SW_RST_N_MODE, 0);
}

/**
 * mxl86111_fiber_config_aneg - restart auto-negotiation or write
 * MXL86111_EXT_MISC_CONFIG_REG.
 * @phydev: pointer to the phy_device
 *
 * NOTE: The caller must have taken the MDIO bus lock.
 *
 * returns 0 or negative errno code
 */
static int mxl86111_fiber_config_aneg(struct phy_device *phydev)
{
	int ret, bmcr;
	u16 adv;
	bool changed = false;

	if (phydev->autoneg != AUTONEG_ENABLE)
		return mxl86111_fiber_setup_forced(phydev);

	/* enable fiber auto sensing */
	ret =  mxlphy_modify_extended_reg(phydev, MXL86111_EXT_SDS_LINK_TIMER_CFG2_REG,
				0, MXL86111_EXT_SDS_LINK_TIMER_CFG2_EN_AUTOSEN);
	if (ret < 0)
		return ret;

	ret =  mxlphy_modify_extended_reg(phydev, MXL86111_EXT_CHIP_CFG_REG,
				MXL86111_EXT_CHIP_CFG_SW_RST_N_MODE, 0);
	if (ret < 0)
		return ret;

	bmcr = __phy_read(phydev, MII_BMCR);
	if (bmcr < 0)
		return bmcr;

	/* For fiber forced mode, power down/up to re-aneg */
	if (!(bmcr & BMCR_ANENABLE)) {
		__phy_modify(phydev, MII_BMCR, 0, BMCR_PDOWN);
		usleep_range(1000, 1050);
		__phy_modify(phydev, MII_BMCR, BMCR_PDOWN, 0);
	}

	adv = linkmode_adv_to_mii_adv_x(phydev->advertising,
					ETHTOOL_LINK_MODE_1000baseX_Full_BIT);

	/* Setup fiber advertisement */
	ret = __phy_modify_changed(phydev, MII_ADVERTISE,
				   ADVERTISE_1000XHALF | ADVERTISE_1000XFULL |
				   ADVERTISE_1000XPAUSE |
				   ADVERTISE_1000XPSE_ASYM,
				   adv);
	if (ret < 0)
		return ret;

	if (ret > 0)
		changed = true;

	/* identical to genphy_check_and_restart_aneg, but phy_read without mdio lock */
	return mxlphy_check_and_restart_aneg(phydev, changed);
}

/**
 * mxl86111_config_aneg_paged() - configure advertising for a givenpage
 * @phydev: pointer to the phy_device
 * @page: The reg page to operate (MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE/MXL86111_EXT_SMI_SDS_PHYUTP_SPACE)
 *
 * returns 0 or negative errno code
 */
static int mxl86111_config_aneg_paged(struct phy_device *phydev, int page)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(fiber_supported);
	struct mxl86111_priv *priv = phydev->priv;
	int page_to_restore;
	int ret = 0;

	page &= MXL86111_EXT_SMI_SDS_PHYSPACE_MASK;

	page_to_restore = phy_select_page(phydev, page);
	if (page_to_restore < 0)
		goto err_restore_page;

	/* In case of Dual Media mode (MXL86111_EXT_SMI_SDS_PHY_AUTO),
	 * update phydev->advertising.
	 */
	if (priv->reg_page == MXL86111_EXT_SMI_SDS_PHY_AUTO) {
		/* prepare the general fiber supported modes */
		linkmode_zero(fiber_supported);
		mxl86111_set_fiber_features(phydev, fiber_supported);

		/* configure fiber_supported, then prepare advertising */
		if (page == MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE) {
			linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT,
					 fiber_supported);
			linkmode_set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT,
					 fiber_supported);
			linkmode_and(phydev->advertising,
				     priv->dual_media_advertising, fiber_supported);
		} else {
			/* ETHTOOL_LINK_MODE_Autoneg_BIT is also used in UTP mode */
			linkmode_clear_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
					   fiber_supported);
			linkmode_andnot(phydev->advertising,
					priv->dual_media_advertising,
					fiber_supported);
		}
	}

	if (priv->reg_page_mode == MXL86111_MODE_AUTO) {
		/* read current page in Dual Media mode as late as possible,
		 * since PHY HW is controlling the page select
		 */
		ret = mxl86111_read_page(phydev);
		if (ret < 0)
			return ret;
		if (ret != page)
			phydev_warn(phydev, "%s, Warning: Dual Media mode - page changed during configuration: page: %d, new: %d, reg_page: %d\n", __func__, page, ret, priv->reg_page);
	}

	if (page == MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE)
		ret = mxl86111_fiber_config_aneg(phydev);
	else
		ret = mxlphy_config_aneg_utp(phydev, false);

err_restore_page:
	return phy_restore_page(phydev, page_to_restore, ret);
}

/**
 * mxl86111_config_aneg() - set reg page and then call mxl86111_config_aneg_paged
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl86111_config_aneg(struct phy_device *phydev)
{
	struct mxl86111_priv *priv = phydev->priv;
	int ret;

	if (priv->reg_page != MXL86111_EXT_SMI_SDS_PHY_AUTO) {
		ret = mxl86111_config_aneg_paged(phydev, priv->reg_page);
		if (ret < 0)
			return ret;
	} else {
		/* In Dual Media mode (MXL86111_EXT_SMI_SDS_PHY_AUTO)
		 * phydev->advertising need to be saved at first run, since it contains the
		 * advertising which supported by both mac and mxl86111(utp and fiber).
		 */
		if (linkmode_empty(priv->dual_media_advertising)) {
			linkmode_copy(priv->dual_media_advertising,
				      phydev->advertising);
		}
		/* read current page in Dual Media mode, since PHY HW is controlling the page select */
		ret = mxl86111_locked_read_page(phydev);
		if (ret < 0)
			return ret;
		ret = mxl86111_config_aneg_paged(phydev, ret);
		if (ret < 0)
			return ret;

		/* we don't known which mode will link, so restore
		 * phydev->advertising as default value.
		 */
		linkmode_copy(phydev->advertising, priv->dual_media_advertising);
	}
	return 0;
}

/**
 * mxl86111_aneg_done_paged() - determines the auto negotiation result of a given page.
 * @phydev: pointer to the phy_device
 * @page: The reg page to operate (MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE/MXL86111_EXT_SMI_SDS_PHYUTP_SPACE)
 *
 * returns 0 (no link) or 1 (fiber or utp link) or negative errno code
 */
static int mxl86111_aneg_done_paged(struct phy_device *phydev, int page)
{
	int page_to_restore;
	int ret = 0;
	int link;

	page_to_restore = phy_select_page(phydev, page & MXL86111_EXT_SMI_SDS_PHYSPACE_MASK);
	if (page_to_restore < 0)
		goto err_restore_page;

	ret = __phy_read(phydev, MXL86111_PHY_STAT_REG);
	if (ret < 0)
		goto err_restore_page;

	link = !!(ret & MXL86111_PHY_STAT_LSRT);
	ret = link;

err_restore_page:
	return phy_restore_page(phydev, page_to_restore, ret);
}

/**
 * mxl86111_aneg_done() - determines the auto negotiation result
 * @phydev: pointer to the phy_device
 *
 * returns 0 (no link) or 1 (fiber or utp link) or negative errno code
 */
static int mxl86111_aneg_done(struct phy_device *phydev)
{
	struct mxl86111_priv *priv = phydev->priv;

	int link;
	int ret;

	if (priv->reg_page != MXL86111_EXT_SMI_SDS_PHY_AUTO) {
		link = mxl86111_aneg_done_paged(phydev, priv->reg_page);
	} else {
		/* read current page in Dual Media mode, since PHY HW is controlling the page select */
		ret = mxl86111_locked_read_page(phydev);
		if (ret < 0)
			return ret;
		link = mxl86111_aneg_done_paged(phydev, ret);
	}

	return link;
}

/**
 * mxl86111_config_init() - initialize the PHY
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl86111_config_init(struct phy_device *phydev)
{
	int page_to_restore, ret = 0;
	unsigned int val = 0;

	page_to_restore = phy_select_page(phydev, MXL86111_EXT_SMI_SDS_PHYUTP_SPACE);
	if (page_to_restore < 0)
		goto err_restore_page;

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII:
		/* no delay, will write 0 */
		val = MXL8611X_EXT_RGMII_CFG1_NO_DELAY;
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		val = MXL8611X_EXT_RGMII_CFG1_RX_DELAY_CUSTOM;
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val = MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_CUSTOM |
				MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_CUSTOM;
		break;
	case PHY_INTERFACE_MODE_RGMII_ID:
		val = MXL8611X_EXT_RGMII_CFG1_TX_1G_DELAY_CUSTOM |
				MXL8611X_EXT_RGMII_CFG1_TX_10MB_100MB_CUSTOM;
		val |= MXL8611X_EXT_RGMII_CFG1_RX_DELAY_CUSTOM;
		break;
	case PHY_INTERFACE_MODE_SGMII:
		break;
	default:
		ret = -EOPNOTSUPP;
		goto err_restore_page;
	}

	/* configure rgmii delay mode */
	if (phydev->interface != PHY_INTERFACE_MODE_SGMII) {
		ret = mxlphy_modify_extended_reg(phydev, MXL8611X_EXT_RGMII_CFG1_REG,
				       MXL8611X_EXT_RGMII_CFG1_FULL_MASK, val);
		if (ret < 0)
			goto err_restore_page;
	}

	if (MXL8611x_UTP_DISABLE_AUTO_SLEEP_FEATURE_CUSTOM == 1) {
		/* disable auto sleep */
		ret = mxlphy_modify_extended_reg(phydev, MXL8611x_UTP_EXT_SLEEP_CTRL_REG,
					MXL8611x_UTP_EXT_SLEEP_CTRL_EN_SLEEP_SW_MASK,
					MXL8611x_UTP_EXT_SLEEP_CTRL_EN_SLEEP_SW_OFF);
		if (ret < 0)
			goto err_restore_page;
	}

	ret = mxl8611x_led_cfg(phydev);
	if (ret < 0)
		goto err_restore_page;

err_restore_page:
	return phy_restore_page(phydev, page_to_restore, ret);
}


/**
 * mxlphy_read_lpa() - read LPA and setup lp_advertising for UTP mode
 * @phydev: pointer to the phy_device
 *
 * NOTE: The caller must have taken the MDIO bus lock.
 *
 * identical to genphy_check_read_lpa, but phy_read without mdio lock
 *
 * returns 0 or negative errno code
 */
static int mxlphy_read_lpa(struct phy_device *phydev)
{
	int lpa, lpagb;

	if (phydev->autoneg == AUTONEG_ENABLE) {
		if (!phydev->autoneg_complete) {
			mii_stat1000_mod_linkmode_lpa_t(phydev->lp_advertising,
							0);
			mii_lpa_mod_linkmode_lpa_t(phydev->lp_advertising, 0);
			return 0;
		}

		if (phydev->is_gigabit_capable) {
			lpagb = __phy_read(phydev, MII_STAT1000);
			if (lpagb < 0)
				return lpagb;

			if (lpagb & LPA_1000MSFAIL) {
				int adv = __phy_read(phydev, MII_CTRL1000);

				if (adv < 0)
					return adv;

				if (adv & CTL1000_ENABLE_MASTER)
					phydev_err(phydev, "Master/Slave resolution failed, maybe conflicting manual settings?\n");
				else
					phydev_err(phydev, "Master/Slave resolution failed\n");
				return -ENOLINK;
			}

			mii_stat1000_mod_linkmode_lpa_t(phydev->lp_advertising,
							lpagb);
		}

		lpa = __phy_read(phydev, MII_LPA);
		if (lpa < 0)
			return lpa;

		mii_lpa_mod_linkmode_lpa_t(phydev->lp_advertising, lpa);
	} else {
		linkmode_zero(phydev->lp_advertising);
	}

	return 0;
}


/**
 * mxl86111_adjust_status() - update speed and duplex to phydev. when in fiber
 * mode, adjust speed and duplex.
 * @phydev: pointer to the phy_device
 * @status: mxl86111 status read from MXL86111_PHY_STAT_REG
 * @is_utp: false(mxl86111 work in fiber mode) or true(mxl86111 work in utp mode)
 *
 * NOTE: The caller must have taken the MDIO bus lock.
 *
 * returns 0
 */
static int mxl86111_adjust_status(struct phy_device *phydev, int status, bool is_utp)
{
	int speed_mode, duplex;
	int speed;
	int err;
	int lpa;

	if (is_utp)
		duplex = (status & MXL86111_PHY_STAT_DPX) >> MXL86111_PHY_STAT_DPX_OFFSET;
	else
		duplex = DUPLEX_FULL;	/* FIBER is always DUPLEX_FULL */

	speed_mode = (status & MXL86111_PHY_STAT_SPEED_MASK) >>
		     MXL86111_PHY_STAT_SPEED_OFFSET;

	switch (speed_mode) {
	case MXL86111_PHY_STAT_SPEED_10M:
		if (is_utp)
			speed = SPEED_10;
		else
			/* not supported for FIBER mode */
			speed = SPEED_UNKNOWN;
		break;
	case MXL86111_PHY_STAT_SPEED_100M:
		speed = SPEED_100;
		break;
	case MXL86111_PHY_STAT_SPEED_1000M:
		speed = SPEED_1000;
		break;
	default:
		speed = SPEED_UNKNOWN;
		break;
	}

	phydev->speed = speed;
	phydev->duplex = duplex;

	if (is_utp) {
		err = mxlphy_read_lpa(phydev);
		if (err < 0)
			return err;

		phy_resolve_aneg_pause(phydev);
	} else {
		lpa = __phy_read(phydev, MII_LPA);
		if (lpa < 0)
			return lpa;
		/* only support 1000baseX Full Duplex in Fiber mode */
		linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseX_Full_BIT,
				 phydev->lp_advertising, lpa & LPA_1000XFULL);
		if (!(lpa & MXL86111_SDS_AN_LPA_PAUSE)) {
			/* disable pause */
			phydev->pause = 0;
			phydev->asym_pause = 0;
		} else if ((lpa & MXL86111_SDS_AN_LPA_ASYM_PAUSE)) {
			/* asymmetric pause */
			phydev->pause = 1;
			phydev->asym_pause = 1;
		} else {
			/* symmetric pause */
			phydev->pause = 1;
			phydev->asym_pause = 0;
		}
	}

	return 0;
}

/**
 * mxl86111_read_status_paged() -  determines the speed and duplex of one page
 * @phydev: pointer to the phy_device
 * @page: The reg page to operate (MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE/MXL86111_EXT_SMI_SDS_PHYUTP_SPACE)
 *
 * returns 1 (utp or fiber link), 0 (no link) or negative errno code
 */
static int mxl86111_read_status_paged(struct phy_device *phydev, int page)
{
	int sds_stat_stored, sds_stat_curr;
	int page_to_restore;
	int ret = 0;
	int status;
	int link;

	linkmode_zero(phydev->lp_advertising);
	phydev->duplex = DUPLEX_UNKNOWN;
	phydev->speed = SPEED_UNKNOWN;
	phydev->asym_pause = 0;
	phydev->pause = 0;

	/* MxL86111 uses two reg spaces (utp/fiber). In Dual Media mode,
	 * the PHY HW will do the switching autonomously upon link status.
	 * By default, utp has priority. In this mode the HW controlled
	 * reg page selection bit is read-only and can only be read.
	 * The reg space should be properly set before reading
	 * MXL86111_PHY_STAT_REG.
	 */

	page &= MXL86111_EXT_SMI_SDS_PHYSPACE_MASK;
	page_to_restore = phy_select_page(phydev, page);
	if (page_to_restore < 0)
		goto err_restore_page;

	/* Read MXL86111_PHY_STAT_REG, which reports the actual speed
	 * and duplex.
	 */
	ret = __phy_read(phydev, MXL86111_PHY_STAT_REG);
	if (ret < 0)
		goto err_restore_page;

	status = ret;
	link = !!(status & MXL86111_PHY_STAT_LSRT);

	/* In fiber mode there is no link down from MXL86111_PHY_STAT_REG
	 * when speed switches from 1000 Mbps to 100Mbps.
	 * It is required to check MII_BMSR to identify such a situation.
	 */
	if (page == MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE) {
		ret = __phy_read(phydev, MII_BMSR);
		if (ret < 0)
			goto err_restore_page;
		sds_stat_stored = ret;

		ret = __phy_read(phydev, MII_BMSR);
		if (ret < 0)
			goto err_restore_page;
		sds_stat_curr = ret;

		if (link && sds_stat_stored != sds_stat_curr) {
			link = 0;
			phydev_info(phydev,
				    "%s, fiber link down detected by SDS_STAT change from %04x to %04x\n",
				    __func__, sds_stat_stored, sds_stat_curr);
		}
	} else {
		/* Read autonegotiation status */
		ret = __phy_read(phydev, MII_BMSR);
		if (ret < 0)
			goto err_restore_page;

		phydev->autoneg_complete = ret & BMSR_ANEGCOMPLETE ? 1 : 0;
	}

	if (link) {
		if (page == MXL86111_EXT_SMI_SDS_PHYUTP_SPACE)
			mxl86111_adjust_status(phydev, status, true);
		else
			mxl86111_adjust_status(phydev, status, false);
	}
	return phy_restore_page(phydev, page_to_restore, link);

err_restore_page:
	return phy_restore_page(phydev, page_to_restore, ret);
}

/**
 * mxl86111_read_status() -  determines the negotiated speed and duplex
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl86111_read_status(struct phy_device *phydev)
{
	struct mxl86111_priv *priv = phydev->priv;
	int link;
	int page;

	page = mxl86111_locked_read_page(phydev);
	if (page < 0)
		return page;

	if (priv->reg_page != MXL86111_EXT_SMI_SDS_PHY_AUTO) {
		link = mxl86111_read_status_paged(phydev, priv->reg_page);
		if (link < 0)
			return link;
	} else {
		link = mxl86111_read_status_paged(phydev, page);
	}

	if (link) {
		if (phydev->link == 0) {
			/* set port type in Dual Media mode based on active page select (controlled by PHY HW). */
			if (priv->reg_page_mode == MXL86111_MODE_AUTO &&
			    priv->reg_page == MXL86111_EXT_SMI_SDS_PHY_AUTO) {
				priv->reg_page = page;

				phydev->port = (page == MXL86111_EXT_SMI_SDS_PHYFIBER_SPACE) ? PORT_FIBRE : PORT_TP;

				phydev_info(phydev, "%s, link up, media: %s\n", __func__,
					    (phydev->port == PORT_TP) ? "UTP" : "Fiber");
			}
			phydev->link = 1;
		} else {
			if (priv->reg_page != page) {
				/* PHY HW detected UTP port active in parallel to FIBER port.
				 * Since UTP has priority, the page will be hard switched from FIBER
				 * to UTP. This needs to be detected and FIBER link reported as down.
				 */
				phydev_info(phydev, "%s, link down, media: %s\n",
					    __func__, (phydev->port == PORT_TP) ? "UTP" : "Fiber");

				/* When in MXL86111_MODE_AUTO mode, prepare to detect the next activation mode
				 * to support Dual Media mode
				 */
				if (priv->reg_page_mode == MXL86111_MODE_AUTO) {
					priv->reg_page = MXL86111_EXT_SMI_SDS_PHY_AUTO;
					phydev->port = PORT_NONE;
				}
				phydev->link = 0;
			}
		}
	} else {
		if (phydev->link == 1) {
			phydev_info(phydev, "%s, link down, media: %s\n",
				    __func__, (phydev->port == PORT_TP) ? "UTP" : "Fiber");

			/* When in MXL86111_MODE_AUTO mode, arbitration will be prepare
			 * to support Dual Media mode
			 */
			if (priv->reg_page_mode == MXL86111_MODE_AUTO) {
				priv->reg_page = MXL86111_EXT_SMI_SDS_PHY_AUTO;
				phydev->port = PORT_NONE;
			}
		}

		phydev->link = 0;
	}

	return 0;
}

/**
 * mxl86111_soft_reset() - issue a PHY software reset
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl86111_soft_reset(struct phy_device *phydev)
{
	return mxl86111_modify_bmcr(phydev, 0, BMCR_RESET);
}

/**
 * mxl86111_suspend() - suspend the PHY hardware
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl86111_suspend(struct phy_device *phydev)
{
	struct mxl86111_priv *priv = phydev->priv;
	int wol_config;

	wol_config = mxlphy_locked_read_extended_reg(phydev, MXL8611X_EXT_WOL_CFG_REG);
	if (wol_config < 0)
		return wol_config;

	/* if wake-on-lan is enabled, do nothing */
	if (wol_config & MXL8611X_EXT_WOL_CFG_WOLE_MASK)
		return 0;

	/* do not power down in Dual Media mode, since page is controlled by HW
	 * and the mode might not 'wake up' anymore later on when the other mode is active
	 */
	if (priv->reg_page_mode == MXL86111_MODE_AUTO)
		return 0;

	return mxl86111_modify_bmcr(phydev, 0, BMCR_PDOWN);
}

/**
 * mxl86111_resume() - resume the PHY hardware
 * @phydev: pointer to the phy_device
 *
 * returns 0 or negative errno code
 */
static int mxl86111_resume(struct phy_device *phydev)
{
	int wol_config;

	wol_config = mxlphy_locked_read_extended_reg(phydev, MXL8611X_EXT_WOL_CFG_REG);
	if (wol_config < 0)
		return wol_config;

	/* if wake-on-lan is enabled, do nothing */
	if (wol_config & MXL8611X_EXT_WOL_CFG_WOLE_MASK)
		return 0;

	return mxl86111_modify_bmcr(phydev, BMCR_PDOWN, 0);
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
	},
	{
		PHY_ID_MATCH_EXACT(PHY_ID_MXL86111),
		.name			= "MXL86111 Gigabit Ethernet",
		.get_features	= mxl86111_get_features,
		.probe			= mxl86111_probe,
		.config_init	= mxl86111_config_init,
		.config_aneg	= mxl86111_config_aneg,
		.aneg_done		= mxl86111_aneg_done,
		.read_status	= mxl86111_read_status,
		.read_page		= mxl86111_read_page,
		.write_page		= mxl86111_write_page,
		.get_wol		= mxlphy_get_wol,
		.set_wol		= mxlphy_set_wol,
		.suspend		= mxl86111_suspend,
		.resume			= mxl86111_resume,
		.soft_reset		= mxl86111_soft_reset,
	},
};

module_phy_driver(mxl_phy_drvs);

MODULE_DESCRIPTION(MXL8611x_DRIVER_DESC);
MODULE_VERSION(MXL8611x_DRIVER_DESC);
MODULE_LICENSE("GPL");

static const struct mdio_device_id __maybe_unused mxl_tbl[] = {
	{ PHY_ID_MATCH_EXACT(PHY_ID_MXL86110) },
	{ PHY_ID_MATCH_EXACT(PHY_ID_MXL86111) },
	{  }
};

MODULE_DEVICE_TABLE(mdio, mxl_tbl);
