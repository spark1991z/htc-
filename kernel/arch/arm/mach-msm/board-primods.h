/* linux/arch/arm/mach-msm/board-pico.h
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef __ARCH_ARM_MACH_MSM_BOARD_PRIMODS_H
#define __ARCH_ARM_MACH_MSM_BOARD_PRIMODS_H

#include <mach/board.h>

#define MSM_MEM_BASE		0x10000000
#define MSM_MEM_SIZE		0x20000000

#define MSM_LINUX_BASE_OFFSET	0x03200000

#define MSM_FB_BASE             0x2FB00000
#define MSM_FB_SIZE             0x00500000

#define MSM_LINUX_BASE         (MSM_MEM_BASE + MSM_LINUX_BASE_OFFSET) /* 2MB alignment */
#define MSM_LINUX_SIZE          (MSM_MEM_SIZE - MSM_LINUX_BASE_OFFSET - MSM_FB_SIZE)

#define MSM_PMEM_MDP_SIZE       0x01800000

#define MSM_PMEM_ADSP_SIZE      0x00D00000
#define MSM_PMEM_ADSP2_SIZE     0x00234000

#define MSM_RAM_CONSOLE_BASE    0x13100000 /* MSM_HTC_RAM_CONSOLE_PHYS must be the same */
#define MSM_RAM_CONSOLE_SIZE    MSM_HTC_RAM_CONSOLE_SIZE

#define PRIMODS_GPIO_TO_INT(x)           (x+64) /* from gpio_to_irq */

#define PRIMODS_GPIO_USB_ID         (42)

/*flashlight */
#define PRIMODS_GPIO_FLASH_ENABLE	(32)
#define PRIMODS_GPIO_FLASH_SWITCH	(115)

#define PRIMODS_POWER_KEY                (37)
#define PRIMODS_GPIO_PS_HOLD         (25)
#define PRIMODS_GPIO_WIFI_IRQ            (29)
/*#define PRIMODS_GPIO_RESET_BTN_N         (36)*/
#define PRIMODS_GPIO_SDMC_CD_N           (38)
/*#define PRIMODS_GPIO_UP_INT_N            (39)*/
#define PRIMODS_GPIO_CHG_INT		(40)
#define PRIMODS_GPIO_CHG_STAT		(41)
#define PRIMODS_GPIO_VOL_DOWN_XB         (48)
#define PRIMODS_GPIO_VOL_DOWN            (48)

/* Camera */
#define PRIMODS_GPIO_CAMERA_SCL		  (60)
#define PRIMODS_GPIO_CAMERA_SDA		  (61)
#define PRIMODS_GPIO_CAM_VCM_PD     (129)
#define PRIMODS_GPIO_CAM_RST        (128)
#define PRIMODS_CAMERA_MCLK 			  (15)

/* WLAN SD data */
#define PRIMODS_GPIO_SD_D3               (64)
#define PRIMODS_GPIO_SD_D2               (65)
#define PRIMODS_GPIO_SD_D1               (66)
#define PRIMODS_GPIO_SD_D0               (67)
#define PRIMODS_GPIO_SD_CMD              (63)
#define PRIMODS_GPIO_SD_CLK_0            (62)

/* I2C */
#define PRIMODS_GPIO_I2C_SCL             (131)
#define PRIMODS_GPIO_I2C_SDA             (132)

/* MicroSD */
#define PRIMODS_GPIO_SDMC_CLK_1          (56)
#define PRIMODS_GPIO_SDMC_CMD            (55)
#define PRIMODS_GPIO_SDMC_D3             (51)
#define PRIMODS_GPIO_SDMC_D2             (52)
#define PRIMODS_GPIO_SDMC_D1             (53)
#define PRIMODS_GPIO_SDMC_D0             (54)

/* BT PCM */
#define PRIMODS_GPIO_AUD_PCM_DO          (68)
#define PRIMODS_GPIO_AUD_PCM_DI          (69)
#define PRIMODS_GPIO_AUD_PCM_SYNC        (70)
#define PRIMODS_GPIO_AUD_PCM_CLK         (71)

/*#define PRIMODS_GPIO_UP_RESET_N          (76)*/
/*#define PRIMODS_GPIO_FLASHLIGHT          (85)*/
#define PRIMODS_GPIO_UART3_RX            (122)
#define PRIMODS_GPIO_UART3_TX            (123)
#define PRIMODS_GPIO_VOL_UP              (86)
/*#define PRIMODS_GPIO_LS_EN               (93)*/
#define PRIMODS_GPIO_WIFI_BT_FAST_CLK_EN   (11)
#define PRIMODS_GPIO_WIFI_SHUTDOWN_N       (13)
/*#define PRIMODS_GPIO_V_USBPHY_3V3_EN     (109)*/

/* Audio External Micbias */
#define PRIMODS_GPIO_AUDIO_2V85_EN       (116)

/* Compass  */
#define PRIMODS_GPIO_GSENSORS_INT         (76)
#define PRIMODS_LAYOUTS			{ \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0, -1} }, \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0,  1} }, \
		{ { -1,  0, 0}, {  0,  1, 0}, {0, 0, -1} }, \
		{ {  1,  0, 0}, {  0,  0, 1}, {0, 1,  0} }  \
					}

/* Proximity Sensor */
#define PRIMODS_GPIO_PROXIMITY_INT       (36)

/* BT */
#define PRIMODS_GPIO_BT_UART1_RTS        (43)
#define PRIMODS_GPIO_BT_UART1_CTS        (44)
#define PRIMODS_GPIO_BT_UART1_RX         (45)
#define PRIMODS_GPIO_BT_UART1_TX         (46)
/*not defined in Primo#DS*/
/*#define PRIMODS_GPIO_BT_RESET_N          (90)*/
/*#define PRIMODS_GPIO_BT_HOST_WAKE        (112)*/
/*#define PRIMODS_GPIO_BT_CHIP_WAKE        (122)*/
/*#define PRIMODS_GPIO_BT_SD_N             (123)*/
#define PRIMODS_GPIO_BT_SDz				   (112)

/* Touch Panel */
#define PRIMODS_GPIO_TP_ATT_N            (18)
#define PRIMODS_V_TP_3V3_EN              (31)
#define PRIMODS_LCD_RSTz		        (118)
#define PRIMODS_GPIO_TP_RST_N            (120)

/* Accessory */
#define PRIMODS_AUD_UART_OEz		(7)
#define PRIMODS_GPIO_AUD_HP_DET		(27)
#define PRIMODS_GPIO_AUD_REMO_PRES		(114)
#define PRIMODS_AUD_2V85_EN				(116)
#define PRIMODS_AUD_UART_SEL			(119)
#define PRIMODS_AUD_UART_RX				(122)
#define PRIMODS_AUD_UART_TX				(123)

/* Button backlight */
#define PRIMODS_GPIO_BUTTON_BACKLIGHT	(83)

/*Display*/
#define PRIMODS_GPIO_LCD_ID0             (34)
#define PRIMODS_GPIO_LCD_ID1             (35)
#define PRIMODS_GPIO_MDDI_TE             (97)
#define PRIMODS_GPIO_LCD_RST_N           (118)
#define PRIMODS_GPIO_LCM_1v8_EN             (5)
#define PRIMODS_GPIO_LCM_3v_EN             (6)

int __init primods_init_keypad(void);
int primods_init_mmc(unsigned int sys_rev);
int __init primods_init_panel(void);
int __init primods_wifi_init(void);
void __init primods_audio_init(void);

extern int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat);
extern int panel_type;
#endif /* GUARD */

