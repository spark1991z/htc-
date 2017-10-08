/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio_event.h>
#include <linux/himax8526a.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/system.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>
#endif
#include <mach/htc_usb.h>
#include <mach/rpc_pmapp.h>
#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android_composite.h>
#include <mach/usbdiag.h>
#endif
#include <mach/msm_memtypes.h>
#include <mach/msm_serial_hs.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/mmc.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/mfd/marimba.h>
#include <mach/vreg.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_battery.h>
#include <linux/ds2746_battery.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_pmic.h>
#include <mach/htc_headset_one_wire.h>

#include <linux/smsc911x.h>
#include "devices.h"
#include "timer.h"
#include "board-msm7x27a-regulator.h"
#include "devices-msm7x2xa.h"
#include "pm.h"
#include "pm-boot.h"
#include <mach/rpc_server_handset.h>
#include <mach/socinfo.h>
#include <linux/bma250.h>
#include <mach/htc_battery.h>
#include <linux/tps65200.h>
#include <linux/leds-pm8029.h>
#include "board-primods.h"
#include <mach/board_htc.h>
#include <linux/htc_flashlight.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_CODEC_AIC3254
#include <linux/i2c/aic3254.h>
#endif
#include <linux/ti_wilink_st.h>
#include <linux/cm3629.h>
#include <mach/htc_util.h>

extern int primods_bluetooth_set_power(int on);
#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif
#include <mach/htc_fast_clk.h>

#include <mach/cable_detect.h>
int htc_get_usb_accessory_adc_level(uint32_t *buffer);

#define PMEM_KERNEL_EBI1_SIZE	0x3A000
#define MSM_PMEM_AUDIO_SIZE	0x5B000
#define BAHAMA_SLAVE_ID_FM_ADDR         0x2A
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B
#define BAHAMA_SLAVE_ID_FM_REG 0x02
#define FM_GPIO	83
#define XA_board 0
#define XB_board 1

extern int emmc_partition_read_proc(char *page, char **start, off_t off,
					int count, int *eof, void *data);

enum {
	GPIO_EXPANDER_IRQ_BASE	= NR_MSM_IRQS + NR_GPIO_IRQS,
	GPIO_EXPANDER_GPIO_BASE	= NR_MSM_GPIOS,
	/* SURF expander */
	GPIO_CORE_EXPANDER_BASE	= GPIO_EXPANDER_GPIO_BASE,
	GPIO_BT_SYS_REST_EN	= GPIO_CORE_EXPANDER_BASE,
	GPIO_WLAN_EXT_POR_N,
	GPIO_DISPLAY_PWR_EN,
	GPIO_BACKLIGHT_EN,
	GPIO_PRESSURE_XCLR,
	GPIO_VREG_S3_EXP,
	GPIO_UBM2M_PWRDWN,
	GPIO_ETM_MODE_CS_N,
	GPIO_HOST_VBUS_EN,
	GPIO_SPI_MOSI,
	GPIO_SPI_MISO,
	GPIO_SPI_CLK,
	GPIO_SPI_CS0_N,
	GPIO_CORE_EXPANDER_IO13,
	GPIO_CORE_EXPANDER_IO14,
	GPIO_CORE_EXPANDER_IO15,
	/* Camera expander */
	GPIO_CAM_EXPANDER_BASE	= GPIO_CORE_EXPANDER_BASE + 16,
	GPIO_CAM_GP_STROBE_READY	= GPIO_CAM_EXPANDER_BASE,
	GPIO_CAM_GP_AFBUSY,
	GPIO_CAM_GP_CAM_PWDN,
	GPIO_CAM_GP_CAM1MP_XCLR,
	GPIO_CAM_GP_CAMIF_RESET_N,
	GPIO_CAM_GP_STROBE_CE,
	GPIO_CAM_GP_LED_EN1,
	GPIO_CAM_GP_LED_EN2,
};

/*static uint opt_disable_uart3;*/

#if defined(CONFIG_GPIO_SX150X)
enum {
	SX150X_CORE,
	SX150X_CAM,
};

static struct sx150x_platform_data sx150x_data[] __initdata = {
	[SX150X_CORE]	= {
		.gpio_base		= GPIO_CORE_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0x02,
		.io_open_drain_ena	= 0xfef8,
		.irq_summary		= -1,
	},
	[SX150X_CAM]	= {
		.gpio_base		= GPIO_CAM_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0,
		.io_open_drain_ena	= 0x23,
		.irq_summary		= -1,
	},
};
#endif

static void config_usb_id_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[CAM] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}
int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

void wl_chip_awake(struct uart_port *uport)
{
	ti_dc_msm_hs_request_clock_on(uport);
}

void wl_chip_asleep(struct uart_port *uport)
{
	ti_dc_msm_hs_request_clock_off(uport);
}

/* wl128x BT, FM, GPS connectivity chip */
struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 112,
	.dev_name = "/dev/ttyHS0",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
#if 1
	.chip_asleep = wl_chip_asleep,
	.chip_awake = wl_chip_awake,
#endif
	.bluetooth_set_power = primods_bluetooth_set_power,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device wl128x_device = {
	.name           = "kim",
	.id             = -1,
	.dev.platform_data = &wilink_pdata,
};



#ifdef CONFIG_PERFLOCK
static unsigned primods_perf_acpu_table[] = {
	245760000,
	480000000,
	1008000000,
};

static struct perflock_platform_data primods_perflock_data = {
	.perf_acpu_table = primods_perf_acpu_table,
	.table_size = ARRAY_SIZE(primods_perf_acpu_table),
};
#endif

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)

	/* FM Platform power and shutdown routines */
#define FPGA_MSM_CNTRL_REG2 0x90008010

static void config_pcm_i2s_mode(int mode)
{
	void __iomem *cfg_ptr;
	u8 reg2;

	cfg_ptr = ioremap_nocache(FPGA_MSM_CNTRL_REG2, sizeof(char));

	if (!cfg_ptr)
		return;
	if (mode) {
		/*enable the pcm mode in FPGA*/
		reg2 = readb_relaxed(cfg_ptr);
		if (reg2 == 0) {
			reg2 = 1;
			writeb_relaxed(reg2, cfg_ptr);
		}
	} else {
		/*enable i2s mode in FPGA*/
		reg2 = readb_relaxed(cfg_ptr);
		if (reg2 == 1) {
			reg2 = 0;
			writeb_relaxed(reg2, cfg_ptr);
		}
	}
	iounmap(cfg_ptr);
}

static unsigned fm_i2s_config_power_on[] = {
	/*FM_I2S_SD*/
	GPIO_CFG(68, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*FM_I2S_WS*/
	GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*FM_I2S_SCK*/
	GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static unsigned fm_i2s_config_power_off[] = {
	/*FM_I2S_SD*/
	GPIO_CFG(68, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*FM_I2S_WS*/
	GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*FM_I2S_SCK*/
	GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static unsigned bt_config_power_on[] = {
	/*RFR*/
	GPIO_CFG(43, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*CTS*/
	GPIO_CFG(44, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*RX*/
	GPIO_CFG(45, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*TX*/
	GPIO_CFG(46, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static unsigned bt_config_pcm_on[] = {
	/*PCM_DOUT*/
	GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_DIN*/
	GPIO_CFG(69, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_SYNC*/
	GPIO_CFG(70, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_CLK*/
	GPIO_CFG(71, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static unsigned bt_config_power_off[] = {
	/*RFR*/
	GPIO_CFG(43, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*CTS*/
	GPIO_CFG(44, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*RX*/
	GPIO_CFG(45, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*TX*/
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};
static unsigned bt_config_pcm_off[] = {
	/*PCM_DOUT*/
	GPIO_CFG(68, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_DIN*/
	GPIO_CFG(69, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_SYNC*/
	GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_CLK*/
	GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static int config_i2s(int mode)
{
	int pin, rc = 0;

	if (mode == FM_I2S_ON) {
		if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf())
			config_pcm_i2s_mode(0);
		pr_err("%s mode = FM_I2S_ON", __func__);
		for (pin = 0; pin < ARRAY_SIZE(fm_i2s_config_power_on);
			pin++) {
				rc = gpio_tlmm_config(
					fm_i2s_config_power_on[pin],
					GPIO_CFG_ENABLE
					);
				if (rc < 0)
					return rc;
			}
	} else if (mode == FM_I2S_OFF) {
		pr_err("%s mode = FM_I2S_OFF", __func__);
		for (pin = 0; pin < ARRAY_SIZE(fm_i2s_config_power_off);
			pin++) {
				rc = gpio_tlmm_config(
					fm_i2s_config_power_off[pin],
					GPIO_CFG_ENABLE
					);
				if (rc < 0)
					return rc;
			}
	}
	return rc;
}
static int config_pcm(int mode)
{
	int pin, rc = 0;

	if (mode == BT_PCM_ON) {
		if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf())
			config_pcm_i2s_mode(1);
		pr_err("%s mode =BT_PCM_ON", __func__);
		for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_on);
			pin++) {
				rc = gpio_tlmm_config(bt_config_pcm_on[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0)
					return rc;
			}
	} else if (mode == BT_PCM_OFF) {
		pr_err("%s mode =BT_PCM_OFF", __func__);
		for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_off);
			pin++) {
				rc = gpio_tlmm_config(bt_config_pcm_off[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0)
					return rc;
			}

	}

	return rc;
}

static int msm_bahama_setup_pcm_i2s(int mode)
{
	int fm_state = 0, bt_state = 0;
	int rc = 0;
	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	fm_state = marimba_get_fm_status(&config);
	bt_state = marimba_get_bt_status(&config);

	switch (mode) {
	case BT_PCM_ON:
	case BT_PCM_OFF:
		if (!fm_state)
			rc = config_pcm(mode);
		break;
	case FM_I2S_ON:
		rc = config_i2s(mode);
		break;
	case FM_I2S_OFF:
		if (bt_state)
			rc = config_pcm(BT_PCM_ON);
		else
			rc = config_i2s(mode);
		break;
	default:
		rc = -EIO;
		pr_err("%s:Unsupported mode", __func__);
	}
	return rc;
}

static int bt_set_gpio(int on)
{
	int rc = 0;
	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	if (on) {
		rc = gpio_direction_output(GPIO_BT_SYS_REST_EN, 1);
		msleep(100);
	} else {
		if (!marimba_get_fm_status(&config) &&
				!marimba_get_bt_status(&config)) {
			gpio_set_value_cansleep(GPIO_BT_SYS_REST_EN, 0);
			rc = gpio_direction_input(GPIO_BT_SYS_REST_EN);
			msleep(100);
		}
	}
	if (rc)
		pr_err("%s: BT sys_reset_en GPIO : Error", __func__);

	return rc;
}
static struct regulator *fm_regulator;
static int fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc = 0;
	const char *id = "FMPW";
	uint32_t irqcfg;
	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};
	u8 value;

	/* Voting for 1.8V Regulator */
	fm_regulator = regulator_get(NULL, "msme1");
	if (IS_ERR(fm_regulator)) {
		rc = PTR_ERR(fm_regulator);
		pr_err("%s: could not get regulator: %d\n", __func__, rc);
		goto out;
	}

	/* Set the voltage level to 1.8V */
	rc = regulator_set_voltage(fm_regulator, 1800000, 1800000);
	if (rc < 0) {
		pr_err("%s: could not set voltage: %d\n", __func__, rc);
		goto reg_free;
	}

	/* Enabling the 1.8V regulator */
	rc = regulator_enable(fm_regulator);
	if (rc) {
		pr_err("%s: could not enable regulator: %d\n", __func__, rc);
		goto reg_free;
	}

	/* Voting for 19.2MHz clock */
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
			PMAPP_CLOCK_VOTE_ON);
	if (rc < 0) {
		pr_err("%s: clock vote failed with :(%d)\n",
			 __func__, rc);
		goto reg_disable;
	}

	rc = bt_set_gpio(1);
	if (rc) {
		pr_err("%s: bt_set_gpio = %d", __func__, rc);
		goto gpio_deconfig;
	}
	/*re-write FM Slave Id, after reset*/
	value = BAHAMA_SLAVE_ID_FM_ADDR;
	rc = marimba_write_bit_mask(&config,
			BAHAMA_SLAVE_ID_FM_REG, &value, 1, 0xFF);
	if (rc < 0) {
		pr_err("%s: FM Slave ID rewrite Failed = %d", __func__, rc);
		goto gpio_deconfig;
	}
	/* Configuring the FM GPIO */
	irqcfg = GPIO_CFG(FM_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA);

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			 __func__, irqcfg, rc);
		goto gpio_deconfig;
	}

	return 0;

gpio_deconfig:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
		PMAPP_CLOCK_VOTE_OFF);
	bt_set_gpio(0);
reg_disable:
	regulator_disable(fm_regulator);
reg_free:
	regulator_put(fm_regulator);
	fm_regulator = NULL;
out:
	return rc;
};

static void fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	const char *id = "FMPW";

	/* Releasing the GPIO line used by FM */
	uint32_t irqcfg = GPIO_CFG(FM_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA);

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			 __func__, irqcfg, rc);

	/* Releasing the 1.8V Regulator */
	if (!IS_ERR_OR_NULL(fm_regulator)) {
		rc = regulator_disable(fm_regulator);
		if (rc)
			pr_err("%s: could not disable regulator: %d\n",
					__func__, rc);
		regulator_put(fm_regulator);
		fm_regulator = NULL;
	}

	/* Voting off the clock */
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
		PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0)
		pr_err("%s: voting off failed with :(%d)\n",
			__func__, rc);
	rc = bt_set_gpio(0);
	if (rc)
		pr_err("%s: bt_set_gpio = %d", __func__, rc);
}

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.fm_setup = fm_radio_setup,
	.fm_shutdown = fm_radio_shutdown,
	.irq = MSM_GPIO_TO_INT(FM_GPIO),
	.vreg_s2 = NULL,
	.vreg_xo_out = NULL,
	/* Configuring the FM SoC as I2S Master */
	.is_fm_soc_i2s_master = true,
	.config_i2s_gpio = msm_bahama_setup_pcm_i2s,
};

static struct platform_device msm_wlan_ar6000_pm_device = {
	.name           = "wlan_ar6000_pm_dev",
	.id             = -1,
};

static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};
struct bahama_config_register {
		u8 reg;
		u8 value;
		u8 mask;
};
struct bt_vreg_info {
	const char *name;
	unsigned int pmapp_id;
	unsigned int min_level;
	unsigned int max_level;
	unsigned int is_pin_controlled;
	struct regulator *reg;
};
static struct bt_vreg_info bt_vregs[] = {
	{"msme1", 2, 1800000, 1800000, 0, NULL},
	{"bt", 21, 2900000, 3050000, 1, NULL}
};

static int bahama_bt(int on)
{
	int rc = 0;
	int i;

	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	struct bahama_variant_register {
		const size_t size;
		const struct bahama_config_register *set;
	};

	const struct bahama_config_register *p;

	u8 version;

	const struct bahama_config_register v10_bt_on[] = {
		{ 0xE9, 0x00, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xE4, 0x00, 0xFF },
		{ 0xE5, 0x00, 0x0F },
#ifdef CONFIG_WLAN
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
		{ 0x01, 0x0C, 0x1F },
		{ 0x01, 0x08, 0x1F },
	};

	const struct bahama_config_register v20_bt_on_fm_off[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xF0, 0x00, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0x8E, 0x15, 0xFF },
		{ 0x8F, 0x15, 0xFF },
		{ 0x90, 0x15, 0xFF },

		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v20_bt_on_fm_on[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v10_bt_off[] = {
		{ 0xE9, 0x00, 0xFF },
	};

	const struct bahama_config_register v20_bt_off_fm_off[] = {
		{ 0xF4, 0x84, 0xFF },
		{ 0xF0, 0x04, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};

	const struct bahama_config_register v20_bt_off_fm_on[] = {
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};
	const struct bahama_variant_register bt_bahama[2][3] = {
	{
		{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
		{ ARRAY_SIZE(v20_bt_off_fm_off), v20_bt_off_fm_off },
		{ ARRAY_SIZE(v20_bt_off_fm_on), v20_bt_off_fm_on }
	},
	{
		{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
		{ ARRAY_SIZE(v20_bt_on_fm_off), v20_bt_on_fm_off },
		{ ARRAY_SIZE(v20_bt_on_fm_on), v20_bt_on_fm_on }
	}
	};

	u8 offset = 0; /* index into bahama configs */
	on = on ? 1 : 0;
	version = marimba_read_bahama_ver(&config);
	if ((int)version < 0 || version == BAHAMA_VER_UNSUPPORTED) {
		dev_err(&msm_bt_power_device.dev, "%s: Bahama \
				version read Error, version = %d \n",
				__func__, version);
		return -EIO;
	}

	if (version == BAHAMA_VER_2_0) {
		if (marimba_get_fm_status(&config))
			offset = 0x01;
	}

	p = bt_bahama[on][version + offset].set;

	dev_info(&msm_bt_power_device.dev,
		"%s: found version %d\n", __func__, version);

	for (i = 0; i < bt_bahama[on][version + offset].size; i++) {
		u8 value = (p+i)->value;
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			dev_err(&msm_bt_power_device.dev,
				"%s: reg %x write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		dev_dbg(&msm_bt_power_device.dev,
			"%s: reg 0x%02x write value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
		value = 0;
		rc = marimba_read_bit_mask(&config,
				(p+i)->reg, &value,
				sizeof((p+i)->value), (p+i)->mask);
		if (rc < 0)
			dev_err(&msm_bt_power_device.dev, "%s marimba_read_bit_mask- error",
					__func__);
		dev_dbg(&msm_bt_power_device.dev,
			"%s: reg 0x%02x read value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	/* Update BT Status */
	if (on)
		marimba_set_bt_status(&config, true);
	else
		marimba_set_bt_status(&config, false);
	return rc;
}
static int bluetooth_switch_regulators(int on)
{
	int i, rc = 0;
	const char *id = "BTPW";

	for (i = 0; i < ARRAY_SIZE(bt_vregs); i++) {
		if (IS_ERR_OR_NULL(bt_vregs[i].reg)) {
			rc = bt_vregs[i].reg ?
				PTR_ERR(bt_vregs[i].reg) :
				-ENODEV;
			dev_err(&msm_bt_power_device.dev,
				"%s: invalid regulator handle for %s: %d\n",
					__func__, bt_vregs[i].name, rc);
			goto reg_disable;
		}

		rc = on ? regulator_set_voltage(bt_vregs[i].reg,
				bt_vregs[i].min_level,
					bt_vregs[i].max_level) : 0;
		if (rc) {
			dev_err(&msm_bt_power_device.dev,
				"%s: could not set voltage for %s: %d\n",
					__func__, bt_vregs[i].name, rc);
			goto reg_disable;
		}

		rc = on ? regulator_enable(bt_vregs[i].reg) : 0;
		if (rc) {
			dev_err(&msm_bt_power_device.dev,
				"%s: could not %sable regulator %s: %d\n",
					__func__, "en", bt_vregs[i].name, rc);
			goto reg_disable;
		}

		if (bt_vregs[i].is_pin_controlled) {
			rc = pmapp_vreg_lpm_pincntrl_vote(id,
					bt_vregs[i].pmapp_id,
					PMAPP_CLOCK_ID_D1,
					on ? PMAPP_CLOCK_VOTE_ON :
						PMAPP_CLOCK_VOTE_OFF);
			if (rc) {
				dev_err(&msm_bt_power_device.dev,
					"%s: pin control failed for %s: %d\n",
					__func__, bt_vregs[i].name, rc);
				goto pin_cnt_fail;
			}
		}
		rc = on ? 0 : regulator_disable(bt_vregs[i].reg);

		if (rc) {
			dev_err(&msm_bt_power_device.dev,
				"%s: could not %sable regulator %s: %d\n",
				__func__, "dis", bt_vregs[i].name, rc);
			goto reg_disable;
		}
	}

	return rc;
pin_cnt_fail:
	if (on)
		regulator_disable(bt_vregs[i].reg);
reg_disable:
	while (i) {
		if (on) {
			i--;
			regulator_disable(bt_vregs[i].reg);
			regulator_put(bt_vregs[i].reg);
		}
	}
	return rc;
}

static struct regulator *reg_s3;
static unsigned int msm_bahama_setup_power(void)
{
	int rc = 0;

	reg_s3 = regulator_get(NULL, "msme1");
	if (IS_ERR(reg_s3)) {
		rc = PTR_ERR(reg_s3);
		pr_err("%s: could not get regulator: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_set_voltage(reg_s3, 1800000, 1800000);
	if (rc) {
		pr_err("%s: could not set voltage: %d\n", __func__, rc);
		goto reg_fail;
	}

	rc = regulator_enable(reg_s3);
	if (rc < 0) {
		pr_err("%s: could not enable regulator: %d\n", __func__, rc);
		goto reg_fail;
	}

	/*setup Bahama_sys_reset_n*/
	rc = gpio_request(GPIO_BT_SYS_REST_EN, "bahama sys_rst_n");
	if (rc) {
		pr_err("%s: gpio_request %d = %d\n", __func__,
			GPIO_BT_SYS_REST_EN, rc);
		goto reg_disable;
	}

	rc = bt_set_gpio(1);
	if (rc) {
		pr_err("%s: bt_set_gpio %d = %d\n", __func__,
			GPIO_BT_SYS_REST_EN, rc);
		goto gpio_fail;
	}

	return rc;

gpio_fail:
	gpio_free(GPIO_BT_SYS_REST_EN);
reg_disable:
	regulator_disable(reg_s3);
reg_fail:
	regulator_put(reg_s3);
out:
	reg_s3 = NULL;
	return rc;
}

static unsigned int msm_bahama_shutdown_power(int value)
{
	int rc = 0;

	if (IS_ERR_OR_NULL(reg_s3)) {
		rc = reg_s3 ? PTR_ERR(reg_s3) : -ENODEV;
		goto out;
	}

	rc = regulator_disable(reg_s3);
	if (rc) {
		pr_err("%s: could not disable regulator: %d\n", __func__, rc);
		goto out;
	}

	if (value == BAHAMA_ID) {
		rc = bt_set_gpio(0);
		if (rc) {
			pr_err("%s: bt_set_gpio = %d\n",
					__func__, rc);
		}
		gpio_free(GPIO_BT_SYS_REST_EN);
	}

	regulator_put(reg_s3);
	reg_s3 = NULL;

	return 0;

out:
	return rc;
}

static unsigned int msm_bahama_core_config(int type)
{
	int rc = 0;

	if (type == BAHAMA_ID) {
		int i;
		struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};
		const struct bahama_config_register v20_init[] = {
			/* reg, value, mask */
			{ 0xF4, 0x84, 0xFF }, /* AREG */
			{ 0xF0, 0x04, 0xFF } /* DREG */
		};
		if (marimba_read_bahama_ver(&config) == BAHAMA_VER_2_0) {
			for (i = 0; i < ARRAY_SIZE(v20_init); i++) {
				u8 value = v20_init[i].value;
				rc = marimba_write_bit_mask(&config,
					v20_init[i].reg,
					&value,
					sizeof(v20_init[i].value),
					v20_init[i].mask);
				if (rc < 0) {
					pr_err("%s: reg %d write failed: %d\n",
						__func__, v20_init[i].reg, rc);
					return rc;
				}
				pr_debug("%s: reg 0x%02x value 0x%02x"
					" mask 0x%02x\n",
					__func__, v20_init[i].reg,
					v20_init[i].value, v20_init[i].mask);
			}
		}
	}
	rc = bt_set_gpio(0);
	if (rc) {
		pr_err("%s: bt_set_gpio = %d\n",
		       __func__, rc);
	}
	pr_debug("core type: %d\n", type);
	return rc;
}

static int bluetooth_power(int on)
{
	int pin, rc = 0;
	const char *id = "BTPW";
	int cid = 0;

	cid = adie_get_detected_connectivity_type();
	if (cid != BAHAMA_ID) {
		pr_err("%s: unexpected adie connectivity type: %d\n",
					__func__, cid);
		return -ENODEV;
	}
	if (on) {
		/*setup power for BT SOC*/
		rc = bt_set_gpio(on);
		if (rc) {
			pr_err("%s: bt_set_gpio = %d\n",
					__func__, rc);
			goto exit;
		}
		rc = bluetooth_switch_regulators(on);
		if (rc < 0) {
			pr_err("%s: bluetooth_switch_regulators rc = %d",
					__func__, rc);
			goto exit;
		}
		/*setup BT GPIO lines*/
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on);
			pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],
					GPIO_CFG_ENABLE);
			if (rc < 0) {
				pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
						__func__,
						bt_config_power_on[pin],
						rc);
				goto fail_power;
			}
		}
		/*Setup BT clocks*/
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
			PMAPP_CLOCK_VOTE_ON);
		if (rc < 0) {
			pr_err("Failed to vote for TCXO_D1 ON\n");
			goto fail_clock;
		}
		msleep(20);

		/*I2C config for Bahama*/
		rc = bahama_bt(1);
		if (rc < 0) {
			pr_err("%s: bahama_bt rc = %d", __func__, rc);
			goto fail_i2c;
		}
		msleep(20);

		/*setup BT PCM lines*/
		rc = msm_bahama_setup_pcm_i2s(BT_PCM_ON);
		if (rc < 0) {
			pr_err("%s: msm_bahama_setup_pcm_i2s , rc =%d\n",
				__func__, rc);
				goto fail_power;
			}
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
				  PMAPP_CLOCK_VOTE_PIN_CTRL);
		if (rc < 0)
			pr_err("%s:Pin Control Failed, rc = %d",
					__func__, rc);

	} else {
		rc = bahama_bt(0);
		if (rc < 0)
			pr_err("%s: bahama_bt rc = %d", __func__, rc);

		rc = bt_set_gpio(on);
		if (rc) {
			pr_err("%s: bt_set_gpio = %d\n",
					__func__, rc);
		}
fail_i2c:
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
				  PMAPP_CLOCK_VOTE_OFF);
		if (rc < 0)
			pr_err("%s: Failed to vote Off D1\n", __func__);
fail_clock:
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off);
			pin++) {
				rc = gpio_tlmm_config(bt_config_power_off[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0) {
					pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_off[pin], rc);
				}
			}
		rc = msm_bahama_setup_pcm_i2s(BT_PCM_OFF);
		if (rc < 0) {
			pr_err("%s: msm_bahama_setup_pcm_i2s, rc =%d\n",
					__func__, rc);
				}
fail_power:
		rc = bluetooth_switch_regulators(0);
		if (rc < 0) {
			pr_err("%s: switch_regulators : rc = %d",\
					__func__, rc);
			goto exit;
		}
	}
	return rc;
exit:
	pr_err("%s: failed with rc = %d", __func__, rc);
	return rc;
}

static int __init bt_power_init(void)
{
	int i, rc = 0;
	struct device *dev = &msm_bt_power_device.dev;

	for (i = 0; i < ARRAY_SIZE(bt_vregs); i++) {
		bt_vregs[i].reg = regulator_get(dev, bt_vregs[i].name);
		if (IS_ERR(bt_vregs[i].reg)) {
			rc = PTR_ERR(bt_vregs[i].reg);
			dev_err(dev, "%s: could not get regulator %s: %d\n",
					__func__, bt_vregs[i].name, rc);
			goto reg_get_fail;
		}
	}

	dev->platform_data = &bluetooth_power;

	return rc;

reg_get_fail:
	while (i--) {
		regulator_put(bt_vregs[i].reg);
		bt_vregs[i].reg = NULL;
	}
	return rc;
}

static struct marimba_platform_data marimba_pdata = {
	.slave_id[SLAVE_ID_BAHAMA_FM]        = BAHAMA_SLAVE_ID_FM_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST]  = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
	.bahama_setup                        = msm_bahama_setup_power,
	.bahama_shutdown                     = msm_bahama_shutdown_power,
	.bahama_core_config                  = msm_bahama_core_config,
	.fm				     = &marimba_fm_pdata,
};

#endif

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
static struct i2c_board_info core_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
	},
};
static struct i2c_board_info cam_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data	= &sx150x_data[SX150X_CAM],
	},
};
#endif
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static struct i2c_board_info bahama_devices[] = {
{
	I2C_BOARD_INFO("marimba", 0xc),
	.platform_data = &marimba_pdata,
},
};
#endif

#ifdef CONFIG_FLASHLIGHT_TPS61310
static void config_flashlight_gpios(void)
{
	static uint32_t flashlight_gpio_table[] = {
		GPIO_CFG(PRIMODS_GPIO_FLASH_ENABLE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		GPIO_CFG(PRIMODS_GPIO_FLASH_SWITCH, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	};

	gpio_tlmm_config(flashlight_gpio_table[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(flashlight_gpio_table[1], GPIO_CFG_ENABLE);

	gpio_direction_output(PRIMODS_GPIO_FLASH_ENABLE, 0);
	gpio_direction_output(PRIMODS_GPIO_FLASH_SWITCH, 0);

}

static struct TPS61310_flashlight_platform_data tps61310_pdata = {
	.gpio_init = config_flashlight_gpios,
	.tps61310_strb0 = PRIMODS_GPIO_FLASH_ENABLE,
	.tps61310_strb1 = PRIMODS_GPIO_FLASH_SWITCH,
	.led_count = 1,
	.flash_duration_ms = 600,
};

static struct i2c_board_info tps61310_i2c_info[] = {
	{
		I2C_BOARD_INFO("TPS61310_FLASHLIGHT", 0x66 >> 1),
		.platform_data = &tps61310_pdata,
	},
};
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
static void __init register_i2c_devices(void)
{

	i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID,
				cam_exp_i2c_info,
				ARRAY_SIZE(cam_exp_i2c_info));

	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf())
		sx150x_data[SX150X_CORE].io_open_drain_ena = 0xe0f0;

	core_exp_i2c_info[0].platform_data =
			&sx150x_data[SX150X_CORE];

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				core_exp_i2c_info,
				ARRAY_SIZE(core_exp_i2c_info));
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				bahama_devices,
				ARRAY_SIZE(bahama_devices));
#endif
}
#endif

static uint32_t qup_i2c_gpio_table_io[] = {
	GPIO_CFG(60, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(61, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(131, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(132, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	};


static uint32_t qup_i2c_gpio_table_hw[] = {
	GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(131, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(132, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	if (adap_id < 0 || adap_id > 1)
		return;

	pr_info("%s. adap_id:%d\n", __func__, adap_id);

	if (config_type){
		gpio_tlmm_config(qup_i2c_gpio_table_hw[adap_id*2], GPIO_CFG_ENABLE);
		gpio_tlmm_config(qup_i2c_gpio_table_hw[adap_id*2 + 1], GPIO_CFG_ENABLE);
	} else {
		gpio_tlmm_config(qup_i2c_gpio_table_io[adap_id*2], GPIO_CFG_ENABLE);
		gpio_tlmm_config(qup_i2c_gpio_table_io[adap_id*2 + 1], GPIO_CFG_ENABLE);
	}
}

/* HTC_START Jacky 20111221 */
#ifdef CONFIG_I2C_HTCTOOLS
#include <linux/i2c/i2c_htc_tools.h>

static uint32_t i2c_recover_gpio_table[] = {
	GPIO_CFG(PICO_GPIO_CAMERA_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(PICO_GPIO_CAMERA_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
	GPIO_CFG(PICO_GPIO_I2C_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(PICO_GPIO_I2C_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static uint32_t i2c_normal_gpio_table[] = {
	GPIO_CFG(PICO_GPIO_CAMERA_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(PICO_GPIO_CAMERA_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
	GPIO_CFG(PICO_GPIO_I2C_SCL, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(PICO_GPIO_I2C_SDA, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static void setupHtcPlatformData(int adap_id, struct msm_i2c_platform_data *pdata)
{
	struct htc_i2c_platform_data *htc_data = 0;
	printk(KERN_INFO "[QUP]: setting up hTC I2C tools....\n");

	htc_data = kzalloc(sizeof(struct htc_i2c_platform_data), GFP_KERNEL);
	if (!htc_data) {
		goto err_alloc_htc_data_failed;
	}
	pdata->htc_pdata = htc_data;

	htc_data->recoverSetting = &i2c_recover_gpio_table[adap_id*2];

	htc_data->operatingSetting = &i2c_normal_gpio_table[adap_id*2];

	switch (adap_id) {
	case 0:
		htc_data->gpio_clk = PICO_GPIO_CAMERA_SCL;
		htc_data->gpio_data = PICO_GPIO_CAMERA_SDA;
		break;
	case 1:
		htc_data->gpio_clk = PICO_GPIO_I2C_SCL;
		htc_data->gpio_data = PICO_GPIO_I2C_SDA;
		break;
	}

	htc_data->allocDone = 1;

	return;

err_alloc_htc_data_failed:
	return;
}

void freeHtcPlatformData(struct msm_i2c_platform_data *pdata)
{
	if (pdata)
		if (pdata->htc_pdata)
			kfree(pdata->htc_pdata);
}

#endif
/*HTC_END */

static struct msm_i2c_platform_data msm_gsbi0_qup_i2c_pdata = {

/* HTC_START
// from 100k to 400k
	.clk_freq 	  = 100000,
*/
	.clk_freq		= 400000,
/* HTC_END */
	.clk			= "gsbi_qup_clk",
	.pclk			= "gsbi_qup_pclk",
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
#ifdef CONFIG_I2C_HTCTOOLS
	.setupHtcPlatformData	= setupHtcPlatformData,
	.freeHtcPlatformData	= freeHtcPlatformData,
#endif
};

static struct msm_i2c_platform_data msm_gsbi1_qup_i2c_pdata = {

/* HTC_START
 //from 100k to 384k
	.clk_freq 	  = 100000,
*/

	.clk_freq		= 384000,
/* HTC_END */
	.clk			= "gsbi_qup_clk",
	.pclk			= "gsbi_qup_pclk",
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
#ifdef CONFIG_I2C_HTCTOOLS
	.setupHtcPlatformData	= setupHtcPlatformData,
	.freeHtcPlatformData	= freeHtcPlatformData,
#endif
};
/*
#ifdef CONFIG_ARCH_MSM7X27A
#define MSM_PMEM_MDP_SIZE       0x1900000
#define MSM_PMEM_ADSP_SIZE      0x1000000

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_SIZE		0x260000
#else
#define MSM_FB_SIZE		0x195000
#endif

#endif
*/

#define PM8058ADC_16BIT(adc) ((adc * 1800) / 65535) /* vref=1.8v, 16-bits resolution */
int64_t primods_get_usbid_adc(void)
{
	uint32_t adc_value = 0xffffffff;
	/*TODO: build break*/
	htc_get_usb_accessory_adc_level(&adc_value);
	adc_value = PM8058ADC_16BIT(adc_value);
	return adc_value;
}

static uint32_t usb_ID_PIN_input_table[] = {
	GPIO_CFG(PRIMODS_GPIO_USB_ID, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
	GPIO_CFG(PRIMODS_GPIO_USB_ID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

void config_primods_usb_id_gpios(bool output)
{
	if (output) {
		config_usb_id_table(usb_ID_PIN_ouput_table, ARRAY_SIZE(usb_ID_PIN_ouput_table));
		gpio_set_value(PRIMODS_GPIO_USB_ID, 1);
		printk(KERN_INFO "%s %d output high\n",  __func__, PRIMODS_GPIO_USB_ID);
	} else {
		config_usb_id_table(usb_ID_PIN_input_table, ARRAY_SIZE(usb_ID_PIN_input_table));
		printk(KERN_INFO "%s %d input none pull\n",  __func__, PRIMODS_GPIO_USB_ID);
	}
}

static struct cable_detect_platform_data cable_detect_pdata = {
	.detect_type 		= CABLE_TYPE_PMIC_ADC,
	.usb_id_pin_gpio 	= PRIMODS_GPIO_USB_ID,
	.config_usb_id_gpios 	= config_primods_usb_id_gpios,
	.get_adc_cb		= primods_get_usbid_adc,
};

static struct platform_device cable_detect_device = {
	.name	= "cable_detect",
	.id	= -1,
	.dev	= {
		.platform_data = &cable_detect_pdata,
	},
};

struct platform_device htc_drm = {
	.name = "htcdrm",
	.id = 0,
};

#ifdef CONFIG_USB_G_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id		= 0x0bb4,
	.product_id		= 0x0cf0,
	.version		= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products		= ARRAY_SIZE(usb_products),
	.products		= usb_products,
	.num_functions		= ARRAY_SIZE(usb_functions_all),
	.functions		= usb_functions_all,
	.fserial_init_string = "tty:modem,tty:autobot,tty:serial,tty:autobot",
	.nluns			= 2,
	.usb_id_pin_gpio = PRIMODS_GPIO_USB_ID,
	.req_reset_during_switch_func = 1,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc = 0;
	unsigned gpio;

	gpio = GPIO_HOST_VBUS_EN;

	rc = gpio_request(gpio, "i2c_host_vbus_en");
	if (rc < 0) {
		pr_err("failed to request %d GPIO\n", gpio);
		return;
	}
	gpio_direction_output(gpio, !!on);
	gpio_set_value_cansleep(gpio, !!on);
	gpio_free(gpio);
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
};

static void __init msm7x2x_init_host(void)
{
	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct regulator *reg_hsusb;
static int msm_hsusb_ldo_init(int init)
{
	int rc = 0;

	if (init) {
		reg_hsusb = regulator_get(NULL, "usb");
		if (IS_ERR(reg_hsusb)) {
			rc = PTR_ERR(reg_hsusb);
			pr_err("%s: could not get regulator: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_set_voltage(reg_hsusb, 3300000, 3300000);
		if (rc) {
			pr_err("%s: could not set voltage: %d\n",
					__func__, rc);
			goto reg_free;
		}

		return 0;
	}
	/* else fall through */
reg_free:
	regulator_put(reg_hsusb);
out:
	reg_hsusb = NULL;
	return rc;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (IS_ERR_OR_NULL(reg_hsusb))
		return reg_hsusb ? PTR_ERR(reg_hsusb) : -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	return enable ?
		regulator_enable(reg_hsusb) :
		regulator_disable(reg_hsusb);
}
#endif

static int primods_phy_init_seq[] =
{
	0x0C, 0x31,
	0x28, 0x32,
	0x06, 0x36,
	0x1D, 0x0D,
	0x1D, 0x10,
	-1
};

static struct msm_otg_platform_data msm_otg_pdata = {
	.phy_init_seq		 = primods_phy_init_seq,
	.mode			 = USB_PERIPHERAL,
	.otg_control		 = OTG_PMIC_CONTROL,
	.power_budget		 = 750,
	.phy_type		 = CI_45NM_INTEGRATED_PHY,
	.reset_phy_before_lpm	 = 1,
};

#ifdef CONFIG_USB_GADGET_MSM_72K
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};
#endif

#if 0
static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x90000300,
		.end   = 0x900003ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(4),
		.end   = MSM_GPIO_TO_INT(4),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};
#endif

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

static unsigned long vreg_sts, gpio_sts;

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

/**
 * Due to insufficient drive strengths for SDC GPIO lines some old versioned
 * SD/MMC cards may cause data CRC errors. Hence, set optimal values
 * for SDC slots based on timing closure and marginality. SDC1 slot
 * require higher value since it should handle bad signal quality due
 * to size of T-flash adapters.
 */
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
								"sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_0"},
};
#endif

static int get_thermal_id(void)
{
	return THERMAL_470_100_4360;
}
static int get_battery_id(void)
{
	return BATTERY_ID_FORMOSA_SANYO;
}
static void primods_poweralg_config_init(struct poweralg_config_type *config)
{
	pr_info("[BATT] %s() is used\n",__func__);
	config->full_charging_mv = 4250;
	config->full_charging_ma = 50;
	config->full_pending_ma = 0;		/* disabled*/
	config->full_charging_timeout_sec = 60 * 60;
	config->voltage_recharge_mv = 4250;
	config->voltage_exit_full_mv = 4200;
	config->min_taper_current_mv = 0;	/* disabled */
	config->min_taper_current_ma = 0;	/* disabled */
	config->wait_votlage_statble_sec = 1 * 60;
	config->predict_timeout_sec = 10;
	config->polling_time_in_charging_sec = 30;

	config->enable_full_calibration = TRUE;
	config->enable_weight_percentage = TRUE;
	config->software_charger_timeout_sec = 57600; /* 16hr */
	config->superchg_software_charger_timeout_sec = 0;	/* disabled */
	config->charger_hw_safety_timer_watchdog_sec =  0;	/* disabled */

	config->debug_disable_shutdown = FALSE;
	if(system_rev == XA_board || system_rev == XB_board) // XA,XB board: set fake temp
		config->debug_fake_room_temp = TRUE;
	else
		config->debug_fake_room_temp = FALSE;

	config->debug_disable_hw_timer = FALSE;
	config->debug_always_predict = FALSE;
	config->por_reset_fail= TRUE;
	config->full_level = 0;
}

static int primods_update_charging_protect_flag(int ibat_ma, int vbat_mv, int temp_01c, BOOL* chg_allowed, BOOL* hchg_allowed, BOOL* temp_fault)
{
	static int pState = 0;
	int old_pState = pState;
	/* pStates:
		0: initial (temp detection)
		1: temp < 0 degree c
		2: 0 <= temp <= 45 degree c
		3: 45 < temp <= 48 degree c
		4: 48 < temp <= 60 degree c
		5: 60 < temp
	*/
	enum {
		PSTAT_DETECT = 0,
		PSTAT_LOW_STOP,
		PSTAT_NORMAL,
		PSTAT_SLOW,
		PSTAT_LIMITED,
		PSTAT_HIGH_STOP
	};
	/* generally we assumed that pState implies last temp.
		it won't hold if temp changes faster than sample rate */

	// step 1. check if change state condition is hit
	pr_debug("[BATT] %s(i=%d, v=%d, t=%d, %d, %d)\n",__func__, ibat_ma, vbat_mv, temp_01c, *chg_allowed, *hchg_allowed);
	switch(pState) {
		default:
			pr_info("[BATT] error: unexpected pState\n");
		case PSTAT_DETECT:
			if (temp_01c < 0)
				pState = PSTAT_LOW_STOP;
			if ((0 <= temp_01c) && (temp_01c <= 450))
				pState = PSTAT_NORMAL;
			if ((450 < temp_01c) && (temp_01c <= 480))
				pState = PSTAT_SLOW;
			if ((480 < temp_01c) && (temp_01c <= 600))
				pState = PSTAT_LIMITED;
			if (600 < temp_01c)
				pState = PSTAT_HIGH_STOP;
			break;
		case PSTAT_LOW_STOP:
			if (30 <= temp_01c)
				pState = PSTAT_NORMAL;
			/* suppose never jump to LIMITED/HIGH_STOP from here */
			break;
		case PSTAT_NORMAL:
			if (temp_01c < 0)
				pState = PSTAT_LOW_STOP;
			else if (600 < temp_01c)
				pState = PSTAT_HIGH_STOP;
			else if (480 < temp_01c) /* also implies t <= 550 */
				pState = PSTAT_LIMITED;
			else if (450 < temp_01c)
				pState = PSTAT_SLOW;
			break;
		case PSTAT_SLOW:
			if (temp_01c < 0)
				pState = PSTAT_LOW_STOP;
			else if (600 < temp_01c)
				pState = PSTAT_HIGH_STOP;
			else if (480 < temp_01c)
				pState = PSTAT_LIMITED;
			else if (temp_01c < 420)
				pState = PSTAT_NORMAL;
			break;
		case PSTAT_LIMITED:
			if (temp_01c < 0)
				pState = PSTAT_LOW_STOP;
			else if (600 < temp_01c)
				pState = PSTAT_HIGH_STOP;
			else if (temp_01c < 420)
				pState = PSTAT_NORMAL;
			else if (temp_01c < 470)
				pState = PSTAT_SLOW;
			break;
		case PSTAT_HIGH_STOP:
			if (temp_01c < 420)
				pState = PSTAT_NORMAL;
			else if (temp_01c < 470)
				pState = PSTAT_SLOW;
			else if ((temp_01c <= 570) && (vbat_mv <= 3800))
				pState = PSTAT_LIMITED;
			/* suppose never jump to LOW_STOP from here */
			break;
	}
	if (old_pState != pState)
		pr_info("[BATT] Protect pState changed from %d to %d\n", old_pState, pState);

	/* step 2. check state protect condition */
	/* chg_allowed = TRUE:only means it's allowed no matter it has charger.
		same as hchg_allowed. */
	switch(pState) {
		default:
		case PSTAT_DETECT:
			pr_info("[BATT] error: unexpected pState\n");
			break;
		case PSTAT_LOW_STOP:
			*chg_allowed = FALSE;
			*hchg_allowed = FALSE;
			break;
		case PSTAT_NORMAL:
			*chg_allowed = TRUE;
			*hchg_allowed = TRUE;
			break;
		case PSTAT_SLOW:	/* 4.2V Charge Full, 4.15V recharge */
			if (4200 < vbat_mv)
				*chg_allowed = FALSE;
			else if (vbat_mv <= 4150)
				*chg_allowed = TRUE;
			*hchg_allowed = FALSE;
			break;
		case PSTAT_LIMITED:	/* 4.1V Charge Full, 3.8V recharge */
			if (PSTAT_LIMITED != old_pState)
				*chg_allowed = TRUE;
			if (4100 < vbat_mv)
				*chg_allowed = FALSE;
			else if (vbat_mv <= 3800)
				*chg_allowed = TRUE;
			*hchg_allowed = FALSE;
			break;
		case PSTAT_HIGH_STOP:
			*chg_allowed = FALSE;
			*hchg_allowed = FALSE;
			break;
	}

	/* update temp_fault */
	if (PSTAT_NORMAL == pState)
		*temp_fault = FALSE;
	else
		*temp_fault = TRUE;

	return pState;
}


static struct htc_battery_platform_data htc_battery_pdev_data = {
	.func_show_batt_attr = htc_battery_show_attr,
	.suspend_highfreq_check_reason = SUSPEND_HIGHFREQ_CHECK_BIT_TALK,
	.guage_driver = GUAGE_DS2746,
	.charger = SWITCH_CHARGER_TPS65200,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};

/* battery parameters */
UINT32 m_parameter_TWS_SDI_1650mah[] = {
	/* capacity (in 0.01%) -> voltage (in mV)*/
	10000, 4250, 8100, 4101, 5100, 3846,
	2000, 3700, 900, 3641, 0, 3411,
};
UINT32 m_parameter_Formosa_Sanyo_1650mah[] = {
	/* capacity (in 0.01%) -> voltage (in mV)*/
	10000, 4250, 8200, 4145, 5000, 3845,
	1800, 3735, 500, 3625, 0, 3405,
};
UINT32 m_parameter_PydTD_1520mah[] = {
	10000, 4100, 5500, 3826, 2000, 3739,
	500, 3665, 0, 3397,
};
UINT32 m_parameter_unknown_1650mah[] = {
	10000, 4250, 8100, 4145, 5100, 3845,
	2000, 3735, 900, 3625, 0, 3405,
};


static UINT32* m_param_tbl[] = {
	m_parameter_unknown_1650mah,
	m_parameter_TWS_SDI_1650mah,
	m_parameter_Formosa_Sanyo_1650mah,
	m_parameter_PydTD_1520mah
};

static UINT32 fl_25[] = {1650, 1620, 1650, 1520};
static UINT32 pd_m_coef[] = {26, 22, 26, 26};
static UINT32 pd_m_resl[] = {100, 100, 100, 100};
static UINT32 pd_t_coef[] = {220, 100, 220, 220};
static INT32 padc[] = {100, 100, 100, 100};
static INT32 pw[] = {5, 5, 5, 5};

static UINT32* pd_m_coef_tbl[] = {pd_m_coef,};
static UINT32* pd_m_resl_tbl[] = {pd_m_resl,};
static UINT32 capacity_deduction_tbl_01p[] = {0,};

static struct battery_parameter primods_battery_parameter = {
	.fl_25 = fl_25,
	.pd_m_coef_tbl = pd_m_coef_tbl,
	.pd_m_coef_tbl_boot = pd_m_coef_tbl,
	.pd_m_resl_tbl = pd_m_resl_tbl,
	.pd_m_resl_tbl_boot = pd_m_resl_tbl,
	.pd_t_coef = pd_t_coef,
	.padc = padc,
	.pw = pw,
	.capacity_deduction_tbl_01p = capacity_deduction_tbl_01p,
	.id_tbl = NULL,
	.temp_index_tbl = NULL,
	.m_param_tbl = m_param_tbl,
	.m_param_tbl_size = sizeof(m_param_tbl)/sizeof(UINT32*),
};

static ds2746_platform_data ds2746_pdev_data = {
	.func_get_thermal_id = get_thermal_id,
	.func_get_battery_id = get_battery_id,
	.func_poweralg_config_init = primods_poweralg_config_init,
	.func_update_charging_protect_flag = primods_update_charging_protect_flag,
	.r2_kohm = 0,	/* use get_battery_id, doesn't need this */
	.batt_param = &primods_battery_parameter,
#ifdef CONFIG_TPS65200
	.func_kick_charger_ic = tps65200_kick_charger_ic,
#endif
};

static struct platform_device ds2746_battery_pdev = {
	.name = "ds2746-battery",
	.id = -1,
	.dev = {
		.platform_data = &ds2746_pdev_data,
	},
};

static struct tps65200_platform_data tps65200_data = {
	.gpio_chg_int = MSM_GPIO_TO_INT(PRIMODS_GPIO_CHG_INT),
	.gpio_chg_stat = MSM_GPIO_TO_INT(PRIMODS_GPIO_CHG_STAT),
};

static struct i2c_board_info i2c_tps65200_devices[] = {
	{
		I2C_BOARD_INFO("tps65200", 0xD4 >> 1),
		.platform_data = &tps65200_data,
	},
};

#ifdef CONFIG_SUPPORT_DQ_BATTERY
static int __init check_dq_setup(char *str)
{
	if (!strcmp(str, "PASS"))
		tps65200_data.dq_result = 1;
	else
		tps65200_data.dq_result = 0;

	return 1;
}
__setup("androidboot.dq=", check_dq_setup);
#endif


static struct msm_gpio sdc1_sleep_cfg_data[] = {
	{GPIO_CFG(51, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc1_dat_3"},
	{GPIO_CFG(52, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc1_dat_2"},
	{GPIO_CFG(53, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc1_dat_1"},
	{GPIO_CFG(54, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc1_dat_0"},
	{GPIO_CFG(55, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc1_cmd"},
	{GPIO_CFG(56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc1_clk"},
};

static struct msm_gpio sdc2_sleep_cfg_data[] = {
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_0"},
};
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
								"sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc3_dat_0"},
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	{GPIO_CFG(19, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc3_dat_7"},
	{GPIO_CFG(20, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc3_dat_6"},
	{GPIO_CFG(21, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc3_dat_5"},
	{GPIO_CFG(108, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_12MA),
								"sdc3_dat_4"},
#endif
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(19, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_3"},
	{GPIO_CFG(20, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_2"},
	{GPIO_CFG(21, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_1"},
	{GPIO_CFG(107, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_cmd"},
	{GPIO_CFG(108, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_0"},
	{GPIO_CFG(109, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_clk"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = sdc1_sleep_cfg_data,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = sdc2_sleep_cfg_data,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
	},
};

static struct regulator *sdcc_vreg_data[ARRAY_SIZE(sdcc_cfg_data)];

static int msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			pr_err("%s: Failed to turn on GPIOs for slot %d\n",
					__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			rc = msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
			return rc;
		}
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
	return rc;
}

static int msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct regulator *curr = sdcc_vreg_data[dev_id - 1];

	if (test_bit(dev_id, &vreg_sts) == enable)
		return 0;

	if (!curr)
		return -ENODEV;

	if (IS_ERR(curr))
		return PTR_ERR(curr);
	mdelay(5);
	if (enable) {
		if (dev_id == 1)
			printk(KERN_INFO "%s: Enabling SD slot power\n", __func__);
		set_bit(dev_id, &vreg_sts);

		rc = regulator_enable(curr);
		if (rc)
			pr_err("%s: could not enable regulator: %d\n",
					__func__, rc);
	} else {
		if (dev_id == 1)
			printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
		clear_bit(dev_id, &vreg_sts);

		rc = regulator_disable(curr);
		if (rc)
			pr_err("%s: could not disable regulator: %d\n",
					__func__, rc);
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	rc = msm_sdcc_setup_gpio(pdev->id, !!vdd);
	if (rc)
		goto out;

	rc = msm_sdcc_setup_vreg(pdev->id, !!vdd);
out:
	return rc;
}

#define GPIO_SDC1_HW_DET 38

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT) \
	&& defined(CONFIG_MMC_MSM_CARD_HW_DETECTION)
static unsigned int msm7x2xa_sdcc_slot_status(struct device *dev)
{
	int status;

	status = gpio_tlmm_config(GPIO_CFG(GPIO_SDC1_HW_DET, 2, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (status)
		pr_err("%s:Failed to configure tlmm for GPIO %d\n", __func__,
				GPIO_SDC1_HW_DET);

	status = gpio_request(GPIO_SDC1_HW_DET, "SD_HW_Detect");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n", __func__,
				GPIO_SDC1_HW_DET);
	} else {
		status = gpio_direction_input(GPIO_SDC1_HW_DET);
		if (!status)
			status = !gpio_get_value(GPIO_SDC1_HW_DET);
		gpio_free(GPIO_SDC1_HW_DET);
	}
	return status;
}
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static unsigned int primods_sdc1_slot_type = MMC_TYPE_SD;
static struct mmc_platform_data sdc1_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 50000000,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x2xa_sdcc_slot_status,
	.status_irq  = MSM_GPIO_TO_INT(GPIO_SDC1_HW_DET),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	.slot_type	= &primods_sdc1_slot_type,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static unsigned int primods_sdc2_slot_type = MMC_TYPE_SDIO_WIFI;
static struct mmc_platform_data sdc2_plat_data = {
	/*
	 * SDC2 supports only 1.8V, claim for 2.85V range is just
	 * for allowing buggy cards who advertise 2.8V even though
	 * they can operate at 1.8V supply.
	 */
	.ocr_mask	= MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(66),
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 49152000,
	.slot_type	= &primods_sdc2_slot_type,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static unsigned int primods_sdc3_slot_type = MMC_TYPE_MMC;
static struct mmc_platform_data sdc3_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 50000000,
	.slot_type	= &primods_sdc3_slot_type,
	.nonremovable	= 1,
	.emmc_dma_ch    = 7,
};
#endif

#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
static struct mmc_platform_data sdc4_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
};
#endif

static int __init mmc_regulator_init(int sdcc_no, const char *supply, int uV)
{
	int rc;

	BUG_ON(sdcc_no < 1 || sdcc_no > 4);

	sdcc_no--;

	sdcc_vreg_data[sdcc_no] = regulator_get(NULL, supply);

	if (IS_ERR(sdcc_vreg_data[sdcc_no])) {
		rc = PTR_ERR(sdcc_vreg_data[sdcc_no]);
		pr_err("%s: could not get regulator \"%s\": %d\n",
				__func__, supply, rc);
		goto out;
	}

	rc = regulator_set_voltage(sdcc_vreg_data[sdcc_no], uV, uV);

	if (rc) {
		pr_err("%s: could not set voltage for \"%s\" to %d uV: %d\n",
				__func__, supply, uV, rc);
		goto reg_free;
	}

	return rc;

reg_free:
	regulator_put(sdcc_vreg_data[sdcc_no]);
out:
	sdcc_vreg_data[sdcc_no] = NULL;
	return rc;
}

static void __init msm7x27a_init_mmc(void)
{
	/* eMMC slot */
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	if (mmc_regulator_init(3, "emmc", 3000000))
		return;
	msm_add_sdcc(3, &sdc3_plat_data);
#endif
	/* Micro-SD slot */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (mmc_regulator_init(1, "mmc", 2850000))
		return;
	msm_add_sdcc(1, &sdc1_plat_data);
#endif
	/* SDIO WLAN slot */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (mmc_regulator_init(2, "mmc", 2850000))
		return;
	msm_add_sdcc(2, &sdc2_plat_data);
#endif
	/* Not Used */
#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
	if (mmc_regulator_init(4, "mmc", 2850000))
		return;
	msm_add_sdcc(4, &sdc4_plat_data);
#endif
}

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(PRIMODS_GPIO_BT_UART1_RX),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
	.cpu_lock_supported = 1,
};
#endif
static struct platform_device primods_rfkill = {
	.name = "primods_rfkill",
	.id = -1,
};




static struct pm8029_led_config pm_led_config[] = {
	{
		.name = "green",
		.bank = 0,
		.init_pwm_brightness = 200,
		.flag = FIX_BRIGHTNESS,
	},
	{
		.name = "amber",
		.bank = 4,
		.init_pwm_brightness = 200,
		.flag = FIX_BRIGHTNESS,
	},

};

static struct pm8029_led_platform_data pm8029_leds_data = {
	.led_config = pm_led_config,
	.num_leds = ARRAY_SIZE(pm_led_config),
};

static struct platform_device pm8029_leds = {
	.name   = "leds-pm8029",
	.id     = -1,
	.dev    = {
		.platform_data  = &pm8029_leds_data,
	},
};

static struct msm_pm_platform_data msm7x27a_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 16000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 12000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 1,
					.latency = 2000,
					.residency = 0,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 2,
					.residency = 0,
	},
};

static struct bma250_platform_data gsensor_bma250_platform_data = {
	.intr = PRIMODS_GPIO_GSENSORS_INT,
	.chip_layout = 0,
	.layouts = PRIMODS_LAYOUTS,
};

static struct i2c_board_info i2c_bma250_devices[] = {
	{
		I2C_BOARD_INFO(BMA250_I2C_NAME_REMOVE_ECOMPASS, \
				0x30 >> 1),
		.platform_data = &gsensor_bma250_platform_data,
		.irq = MSM_GPIO_TO_INT(PRIMODS_GPIO_GSENSORS_INT),
	},
};

#ifdef CONFIG_CODEC_AIC3254
static int aic3254_lowlevel_init(void)
{
	return 0;
}

static struct i2c_board_info i2c_aic3254_devices[] = {
	{
		I2C_BOARD_INFO(AIC3254_I2C_NAME, \
				AIC3254_I2C_ADDR),
	},
};
#endif
static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct android_pmem_platform_data android_pmem_adsp2_pdata = {
        .name = "pmem_adsp2",
        .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
        .cached = 1,
        .memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp2_device = {
        .name = "android_pmem",
        .id = 3,
        .dev = { .platform_data = &android_pmem_adsp2_pdata },
};

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_adsp2_size = MSM_PMEM_ADSP2_SIZE;
static int __init pmem_adsp2_size_setup(char *p)
{
        pmem_adsp2_size = memparse(p, NULL);
        return 0;
}

early_param("pmem_adsp2_size", pmem_adsp2_size_setup);

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
	SND(FM_DIGITAL_STEREO_HEADSET, 26),
	SND(FM_DIGITAL_SPEAKER_PHONE, 27),
	SND(FM_DIGITAL_BT_A2DP_HEADSET, 28),
	SND(CURRENT, 0x7FFFFFFE),
	SND(FM_ANALOG_STEREO_HEADSET, 35),
	SND(FM_ANALOG_STEREO_HEADSET_CODEC, 36),
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	0, 0, 0, 0,

	/* Concurrency 7 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design     = 2800,
	.voltage_max_design     = 4300,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage	 = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}

static struct platform_device msm_batt_device = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};
#if 0
static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_16BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start	= 0x90000000,
		.end	= 0x90007fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(48),
		.end	= MSM_GPIO_TO_INT(48),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data	= &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "smsc911x_irq"  },
	{ GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "eth_fifo_sel" },
};

#define ETH_FIFO_SEL_GPIO	49
static void msm7x27a_cfg_smsc911x(void)
{
	int res;

	res = msm_gpios_request_enable(smsc911x_gpios,
				 ARRAY_SIZE(smsc911x_gpios));
	if (res) {
		pr_err("%s: unable to enable gpios for SMSC911x\n", __func__);
		return;
	}

	/* ETH_FIFO_SEL */
	res = gpio_direction_output(ETH_FIFO_SEL_GPIO, 0);
	if (res) {
		pr_err("%s: unable to get direction for gpio %d\n", __func__,
							 ETH_FIFO_SEL_GPIO);
		msm_gpios_disable_free(smsc911x_gpios,
						 ARRAY_SIZE(smsc911x_gpios));
		return;
	}
	gpio_set_value(ETH_FIFO_SEL_GPIO, 0);
}
#endif
#ifdef CONFIG_MSM_CAMERA
#if 0
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_EXT,
	._fsrc.ext_driver_src.led_en = GPIO_CAM_GP_LED_EN1,
	._fsrc.ext_driver_src.led_flash_en = GPIO_CAM_GP_LED_EN2,
};
#endif
static struct regulator_bulk_data regs_camera[] = {
	{ .supply = "msme1", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "gp2",   .min_uV = 2850000, .max_uV = 2850000 },
	{ .supply = "usb2",  .min_uV = 1800000, .max_uV = 1800000 },
};

#if 0 /* mark QCT code by camera*/
static void __init msm_camera_vreg_init(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_camera), regs_camera);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		return;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_camera), regs_camera);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		return;
	}
}
#endif

static void msm_camera_vreg_config(int vreg_en)
{
	int rc = vreg_en ?
		regulator_bulk_enable(ARRAY_SIZE(regs_camera), regs_camera) :
		regulator_bulk_disable(ARRAY_SIZE(regs_camera), regs_camera);

	if (rc)
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, vreg_en ? "en" : "dis", rc);
}

static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(PRIMODS_GPIO_CAMERA_SDA,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_GPIO_CAMERA_SCL,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_GPIO_CAM_RST,     0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_GPIO_CAM_VCM_PD,   0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_CAMERA_MCLK,      0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(PRIMODS_GPIO_CAMERA_SDA,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_GPIO_CAMERA_SCL,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_GPIO_CAM_RST,     0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_GPIO_CAM_VCM_PD,   0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_CAMERA_MCLK,      1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
};

#ifdef CONFIG_MSM_CAMERA_FLASH
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
#if 0 /* HTC enable flashlight */
	._fsrc.current_driver_src.led1 = GPIO_SURF_CAM_GP_LED_EN1,
	._fsrc.current_driver_src.led2 = GPIO_SURF_CAM_GP_LED_EN2,
#else
	.camera_flash = tps61310_flashlight_control,
#endif
};
#endif

static struct vreg *vreg_wlan1p2c150; //L19
static struct vreg *vreg_bt;
static void primods_camera_vreg_config(int vreg_en)
{
	int rc;

	pr_info("[CAM]primods_camera_vreg_config %d\n", vreg_en);
	if (vreg_wlan1p2c150 == NULL) { /* VCM 2V85 */
		vreg_wlan1p2c150 = vreg_get(NULL, "wlan4");
		if (IS_ERR(vreg_wlan1p2c150)) {
			pr_err("[CAM]%s: vreg_get(%s) VCM 2V85 failed (%ld)\n",
				__func__, "wlan1p2c150", PTR_ERR(vreg_wlan1p2c150));
			return;
		}

		rc = vreg_set_level(vreg_wlan1p2c150, 2850);
		if (rc) {
			pr_err("[CAM]%s: VCM 2V85 set_level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_bt == NULL) { /* A2.85v */
		vreg_bt = vreg_get(NULL, "bt");
		if (IS_ERR(vreg_bt)) {
			pr_err("[CAM]%s: vreg_get(%s) failed (%ld)\n",
				__func__, "bt", PTR_ERR(vreg_bt));
			return;
		}
		printk("[CAM]primods_camera_vreg_config: A2.85v\n");
		rc = vreg_set_level(vreg_bt, 2850);
		if (rc) {
			pr_err("[CAM]%s: A2.85v  set level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_en) {
			rc = vreg_enable(vreg_wlan1p2c150);// VCM 2V85
			printk("[CAM]primods_camera_vreg_config: vreg_enable(vreg_wlan1p2c150) VCM 2V85\n");

			if (rc) {
				pr_err("[CAM]%s: VCM 2V85 enable failed (%d)\n",
				__func__, rc);
			}

			udelay(1);

#if 0 /* due to IO and Digital pin connect internal by HW design for Raw Chip */
			/* D1V8 */
			printk("primods_camera_vreg_config: set D1V8 enable\n");
			gpio_set_value(81, 1);
#endif

			/* IO 1V8 */
			printk("[CAM]primods_camera_vreg_config: set IO 1V8 enable\n");
			gpio_set_value(12, 1);

			udelay(1);

			rc = vreg_enable(vreg_bt);/* A2V85 */
			printk("[CAM]primods_camera_vreg_config: vreg_enable(vreg_bt) A2V85\n");

			if (rc) {
				pr_err("[CAM]%s: A2V85 enable failed (%d)\n",
				__func__, rc);
			}

	} else {
		/* A2V85 */
		rc = vreg_disable(vreg_bt);
		printk("[CAM]primods_camera_vreg_config: vreg_disable(vreg_bt) A2V85\n");
		if (rc) {
			pr_err("[CAM]%s: A2V85 disable failed (%d)\n",
				__func__, rc);
		}

		hr_msleep(1);


		/* IO 1V8 */
		printk("[CAM]primods_camera_vreg_config: set IO 1V8 disable\n");
		gpio_set_value(12, 0);

#if 0 /* due to IO and Digital pin connect internal by HW design for Raw Chip*/
		/* D1V8 */
		printk("primods_camera_vreg_config: set D1V8 disable\n");
		gpio_set_value(81, 0);
#endif

		hr_msleep(7);

		/* VCM 2V85 */
		rc = vreg_disable(vreg_wlan1p2c150);
		printk("[CAM]primods_camera_vreg_config: vreg_disable(vreg_wlan1p2c150) VCM 2V85\n");
		if (rc) {
			pr_err("[CAM]%s: VCM 2V85 disable failed (%d)\n",
				__func__, rc);
		}
	}
}
/* HTC_END 20111221 */

static int config_gpio_table(uint32_t *table, int len)
{
	int rc = 0, i = 0;

	for (i = 0; i < len; i++) {
		rc = gpio_tlmm_config(table[i], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s not able to get gpio\n", __func__);
			for (i--; i >= 0; i--)
				gpio_tlmm_config(camera_off_gpio_table[i],
					GPIO_CFG_ENABLE);
		break;
		}
	}
	return rc;
}

static int config_camera_on_gpios_rear(void)
{
	int rc = 0;

	if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa())
		msm_camera_vreg_config(1);
/* HTC_START */
#ifdef CONFIG_ARCH_MSM7X27A
	primods_camera_vreg_config(1);
#endif
/* HTC_END */

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("[CAM]%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_rear(void)
{
	if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa())
		msm_camera_vreg_config(0);
/* HTC_START */
/* Follow HW's IO control designed sequence. (RST-> A2.85v -> D1.8v -> IO1.8v -> MCLK & SCL/SDA)*/
#ifdef CONFIG_ARCH_MSM7X27A
	primods_camera_vreg_config(0);
#endif
/* HTC_END */

	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

static int config_camera_on_gpios_front(void)
{
	int rc = 0;

	if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa())
		msm_camera_vreg_config(1);

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
			"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_front(void)
{
	if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa())
		msm_camera_vreg_config(0);

	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

struct msm_camera_device_platform_data primods_camera_device_data_rear = {
	.camera_gpio_on  = config_camera_on_gpios_rear,
	.camera_gpio_off = config_camera_off_gpios_rear,
	.ioext.csiphy = 0xA1000000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

struct msm_camera_device_platform_data primods_camera_device_data_front = {
	.camera_gpio_on  = config_camera_on_gpios_front,
	.camera_gpio_off = config_camera_off_gpios_front,
	.ioext.csiphy = 0xA0F00000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_0,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_S5K4E1
static struct msm_camera_sensor_platform_info s5k4e1_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_s5k4e1 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src              = &msm_flash_src
#endif
};

#ifdef CONFIG_MSM_CAMERA_FLASH
static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.num_flash_levels = FLASHLIGHT_NUM,
	.low_temp_limit = 5,
	.low_cap_limit = 15,
};
#endif

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1_data;
static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1_data = {
	.sensor_name    = "s5k4e1",
/*	.sensor_reset_enable = 1, */
	.sensor_reset   = PRIMODS_GPIO_CAM_RST,
/*	.sensor_pwd             = 85, */
	.vcm_pwd                = PRIMODS_GPIO_CAM_VCM_PD,
	.vcm_enable             = 1,
	.pdata                  = &primods_camera_device_data_rear,
	.flash_data             = &flash_s5k4e1,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_cfg		= &msm_camera_sensor_flash_cfg,
#endif
	.sensor_platform_info   = &s5k4e1_sensor_7627a_info,
	.full_size_preview		= true,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_s5k4e1 = {
	.name   = "msm_camera_s5k4e1",
	.dev    = {
		.platform_data = &msm_camera_sensor_s5k4e1_data,
	},
};
#endif

#ifdef CONFIG_IMX072
static struct msm_camera_sensor_platform_info imx072_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_imx072 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_imx072_data = {
	.sensor_name    = "imx072",
	.sensor_reset_enable = 1,
	.sensor_reset   = GPIO_CAM_GP_CAMIF_RESET_N, /* TODO 106,*/
	.sensor_pwd             = 85,
	.vcm_pwd                = GPIO_CAM_GP_CAM_PWDN,
	.vcm_enable             = 1,
	.pdata                  = &primods_camera_device_data_rear,
	.flash_data             = &flash_imx072,
	.sensor_platform_info = &imx072_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_imx072 = {
	.name   = "msm_camera_imx072",
	.dev    = {
		.platform_data = &msm_camera_sensor_imx072_data,
	},
};
#endif

#ifdef CONFIG_WEBCAM_OV9726
static struct msm_camera_sensor_platform_info ov9726_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_ov9726 = {
	.flash_type             = MSM_CAMERA_FLASH_NONE,
	.flash_src              = &msm_flash_src
};
static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data;
static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data = {
	.sensor_name    = "ov9726",
	.sensor_reset_enable = 0,
	.sensor_reset   = GPIO_CAM_GP_CAM1MP_XCLR,
	.sensor_pwd             = 85,
	.vcm_pwd                = 1,
	.vcm_enable             = 0,
	.pdata                  = &primods_camera_device_data_front,
	.flash_data             = &flash_ov9726,
	.sensor_platform_info   = &ov9726_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov9726 = {
	.name   = "msm_camera_ov9726",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov9726_data,
	},
};
#else
/*static inline void msm_camera_vreg_init(void) { }*/
#endif

#ifdef CONFIG_MT9E013
static struct msm_camera_sensor_platform_info mt9e013_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9e013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data = {
	.sensor_name    = "mt9e013",
	.sensor_reset   = 0,
	.sensor_reset_enable = 1,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &primods_camera_device_data_rear,
	.flash_data     = &flash_mt9e013,
	.sensor_platform_info   = &mt9e013_sensor_7627a_info,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data,
	},
};
#endif

static struct i2c_board_info i2c_camera_devices[] = {
	#ifdef CONFIG_S5K4E1
	{
		I2C_BOARD_INFO("s5k4e1", 0x20 >>1),
	},
	{
		I2C_BOARD_INFO("s5k4e1_af", 0x18 >>1),
	},
	#endif
	#ifdef CONFIG_WEBCAM_OV9726
	{
		I2C_BOARD_INFO("ov9726", 0x10),
	},
	#endif
	#ifdef CONFIG_IMX072
	{
		I2C_BOARD_INFO("imx072", 0x34),
	},
	#endif
	#ifdef CONFIG_MT9E013
	{
		I2C_BOARD_INFO("mt9e013", 0x6C >> 2),
	},
	#endif
	{
		I2C_BOARD_INFO("sc628a", 0x6E),
	},
};
#endif
#if defined(CONFIG_SERIAL_MSM_HSL_CONSOLE) \
		&& defined(CONFIG_MSM_SHARED_GPIO_FOR_UART2DM)
static struct msm_gpio uart2dm_gpios[] = {
	{GPIO_CFG(19, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rfr_n" },
	{GPIO_CFG(20, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_cts_n" },
	{GPIO_CFG(21, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rx"    },
	{GPIO_CFG(108, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_tx"    },
};

static void msm7x27a_cfg_uart2dm_serial(void)
{
	int ret;
	ret = msm_gpios_request_enable(uart2dm_gpios,
					ARRAY_SIZE(uart2dm_gpios));
	if (ret)
		pr_err("%s: unable to enable gpios for uart2dm\n", __func__);
}
#else
static void msm7x27a_cfg_uart2dm_serial(void) { }
#endif

static struct resource ram_console_resources[] = {
	{
		.start  = MSM_RAM_CONSOLE_BASE,
		.end    = MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resources),
	.resource       = ram_console_resources,
};

static struct htc_fast_clk_platform_data htc_fast_clk_data = {
	.mode = 2,
	.id = "wlan",
	.fast_clk_pin = PRIMODS_GPIO_WIFI_BT_FAST_CLK_EN,
};

static struct platform_device wifi_bt_fast_clk = {
	.name = "htc_fast_clk",
	.id = -1,
	.dev = {
		.platform_data = &htc_fast_clk_data,
	},
};
static struct cm3629_platform_data cm36282_pdata = {
        .model = CAPELLA_CM36282,
        .ps_select = CM3629_PS1_ONLY,
	.intr = PRIMODS_GPIO_PROXIMITY_INT,
	.levels = {5, 36, 71, 237, 1603, 2137, 2493, 2849, 3205, 65535},
        .golden_adc = 0x410,
        .power = NULL,
        .cm3629_slave_address = 0xC0>>1,
	.ps_calibration_rule = 1,
        .ps1_thd_set = 0x08,
	.ps1_thh_diff = 3,
        .ps1_thd_no_cal = 0xF1,
        .ps1_thd_with_cal = 0x08,
        .ps_conf1_val = CM3629_PS_DR_1_80 | CM3629_PS_IT_1_6T |
                        CM3629_PS1_PERS_1,
        .ps_conf2_val = CM3629_PS_ITB_1 | CM3629_PS_ITR_1 |
                        CM3629_PS2_INT_DIS | CM3629_PS1_INT_DIS,
        .ps_conf3_val = CM3629_PS2_PROL_32,

};

static struct i2c_board_info i2c_cm36282_devices[] = {
	{
		I2C_BOARD_INFO(CM3629_I2C_NAME,0xC0 >> 1),
		.platform_data = &cm36282_pdata,
		.irq = PRIMODS_GPIO_TO_INT(PRIMODS_GPIO_PROXIMITY_INT),
	},
};

static struct cm3629_platform_data cm36282_XC_pdata = {
        .model = CAPELLA_CM36282,
        .ps_select = CM3629_PS1_ONLY,
	.intr = PRIMODS_GPIO_PROXIMITY_INT,
	.levels = {5, 36, 71, 237, 1603, 2137, 2493, 2849, 3205, 65535},
        .golden_adc = 0x410,
        .power = NULL,
        .cm3629_slave_address = 0xC0>>1,
	.ps_calibration_rule = 5,
        .ps1_thd_set = 0x08,
	.ps1_thh_diff = 2,
        .ps1_thd_no_cal = 0xF1,
        .ps1_thd_with_cal = 0x08,
        .ps_conf1_val = CM3629_PS_DR_1_80 | CM3629_PS_IT_1_6T |
                        CM3629_PS1_PERS_1,
        .ps_conf2_val = CM3629_PS_ITB_1 | CM3629_PS_ITR_1 |
                        CM3629_PS2_INT_DIS | CM3629_PS1_INT_DIS,
        .ps_conf3_val = CM3629_PS2_PROL_32,

};

static struct i2c_board_info i2c_cm36282_XC_devices[] = {
	{
		I2C_BOARD_INFO(CM3629_I2C_NAME,0xC0 >> 1),
		.platform_data = &cm36282_XC_pdata,
		.irq = PRIMODS_GPIO_TO_INT(PRIMODS_GPIO_PROXIMITY_INT),
	},
};
/* HEADSET DRIVER BEGIN */

#define HEADSET_DETECT		PRIMODS_GPIO_AUD_HP_DET
#define HEADSET_BUTTON		PRIMODS_GPIO_AUD_REMO_PRES

/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= HEADSET_DETECT,
	.key_enable_gpio	= 0,
	.mic_select_gpio	= 0,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

/* HTC_HEADSET_PMIC Driver */
static struct htc_headset_pmic_platform_data htc_headset_pmic_data = {
	.driver_flag		= DRIVER_HS_PMIC_RPC_KEY,
	.hpin_gpio		= 0,
	.hpin_irq		= 0,
	.key_gpio		= HEADSET_BUTTON,
	.key_irq		= 0,
	.key_enable_gpio	= 0,
	.adc_mic		= 14894,
	.adc_remote		= {0, 1714, 1715, 5630, 5631, 12048},
	.hs_controller		= 0,
	.hs_switch		= 0,
};

static struct platform_device htc_headset_pmic = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data,
	},
};

static struct htc_headset_1wire_platform_data htc_headset_1wire_data = {
	.tx_level_shift_en      = PRIMODS_AUD_UART_OEz,
	.uart_sw                = PRIMODS_AUD_UART_SEL,
	.remote_press	       = 0,
	.one_wire_remote        ={0x7E, 0x7F, 0x7D, 0x7F, 0x7B, 0x7F},
	.onewire_tty_dev	= "/dev/ttyMSM0",
};

static struct platform_device htc_headset_one_wire = {
       .name   = "HTC_HEADSET_1WIRE",
       .id     = -1,
       .dev    = {
               .platform_data  = &htc_headset_1wire_data,
       },
};


static struct gpio_led gpio_exp_leds_config[] = {
	{
		.name = "button-backlight",
		.gpio = PRIMODS_GPIO_BUTTON_BACKLIGHT,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data gpio_leds_pdata = {
	.num_leds = ARRAY_SIZE(gpio_exp_leds_config),
	.leds = gpio_exp_leds_config,
};

static struct platform_device gpio_leds = {
	.name		  = "leds-gpio",
	.id			= -1,
	.dev		   = {
		.platform_data = &gpio_leds_pdata,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices[] = {
	&htc_headset_pmic,
	&htc_headset_gpio,
	&htc_headset_one_wire,
	/* Please put the headset detection driver on the last */
};

static struct headset_adc_config htc_headset_mgr_config[] = {
	{
		.type = HEADSET_MIC,
		.adc_max = 59508,
		.adc_min = 47455,
	},
	{
		.type = HEADSET_BEATS,
		.adc_max = 47434,
		.adc_min = 34950,
	},
	{
		.type = HEADSET_BEATS_SOLO,
		.adc_max = 34928,
		.adc_min = 21582,
	},
	{
		.type = HEADSET_NO_MIC, /* HEADSET_INDICATOR */
		.adc_max = 21581,
		.adc_min = 9045,
	},
	{
		.type = HEADSET_NO_MIC,
		.adc_max = 9044,
		.adc_min = 0,
	},
};

static uint32_t headset_cpu_gpio[] = {
	GPIO_CFG(PRIMODS_AUD_UART_OEz, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_GPIO_AUD_HP_DET, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_GPIO_AUD_REMO_PRES, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_AUD_2V85_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_AUD_UART_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t headset_1wire_gpio[] = {
	GPIO_CFG(PRIMODS_AUD_UART_RX, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_AUD_UART_TX, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_AUD_UART_RX, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(PRIMODS_AUD_UART_TX, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
 };

static void uart_tx_gpo(int mode)
{
	switch (mode) {
		case 0:
			gpio_tlmm_config(headset_1wire_gpio[1], GPIO_CFG_ENABLE);
			gpio_set_value_cansleep(PRIMODS_AUD_UART_TX, 0);
			pr_info("[HS_BOARD]UART TX GPO 0\n");
			break;
		case 1:
			gpio_tlmm_config(headset_1wire_gpio[1], GPIO_CFG_ENABLE);
			gpio_set_value_cansleep(PRIMODS_AUD_UART_TX, 1);
			pr_info("[HS_BOARD]UART TX GPO 1\n");
			break;
		case 2:
			gpio_tlmm_config(headset_1wire_gpio[3], GPIO_CFG_ENABLE);
			pr_info("[HS_BOARD]UART TX alternative\n");
			break;
	}
}

static void uart_lv_shift_en(int enable)
{
	gpio_set_value_cansleep(PRIMODS_AUD_UART_OEz, enable);
	pr_info("[HS_BOARD]level shift %d\n", enable);
}

static void headset_init(void)
{
	int i,ret;
	for (i = 0; i < ARRAY_SIZE(headset_cpu_gpio); i++) {
		ret = gpio_tlmm_config(headset_cpu_gpio[i], GPIO_CFG_ENABLE);
		pr_info("[HS_BOARD]Config gpio[%d], ret = %d\n", (headset_cpu_gpio[i] & 0x3FF0) >> 4, ret);
	}
	gpio_set_value(PRIMODS_AUD_UART_OEz, 1); /*Disable 1-wire level shift by default*/
	gpio_set_value(PRIMODS_AUD_2V85_EN, 0);
	gpio_set_value(PRIMODS_AUD_UART_SEL, 0);
}

static void headset_power(int hs_enable)
{
	gpio_set_value(PRIMODS_AUD_UART_OEz, 1);
	gpio_set_value(PRIMODS_AUD_2V85_EN, hs_enable);
}

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.driver_flag		= DRIVER_HS_MGR_FLOAT_DET,
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
	.headset_config_num	= ARRAY_SIZE(htc_headset_mgr_config),
	.headset_config		= htc_headset_mgr_config,
	.headset_power		= headset_power,
	.headset_init		= headset_init,
	.uart_tx_gpo		= uart_tx_gpo,
	.uart_lv_shift_en	= uart_lv_shift_en,
};

static struct platform_device htc_headset_mgr = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data,
	},
};

/* HEADSET DRIVER END */

#if 0
static struct platform_device *rumi_sim_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&smc91x_device,
	&msm_device_uart1,
	&msm_device_nand,
	&msm_device_uart_dm1,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
};

static struct platform_device *primods_devices[] __initdata = {
	&ram_console_device,
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_uart1,
	&msm_device_uart3,
	/*&msm_device_uart_dm1,*/
	&msm_device_nand,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&htc_battery_pdev,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_adsp2_device,
/*	&usb_gadget_fserial_device,*/
/*	&msm_device_adspdec,*/
#ifdef CONFIG_BATTERY_MSM
	&msm_batt_device,
#endif
/*	&htc_headset_mgr,*/
#ifdef CONFIG_S5K4E1
/*	&msm_camera_sensor_s5k4e1,*/
#endif
#ifdef CONFIG_IMX072
/*	&msm_camera_sensor_imx072,*/
#endif
#ifdef CONFIG_WEBCAM_OV9726
/*	&msm_camera_sensor_ov9726,*/
#endif
#ifdef CONFIG_MT9E013
/*	&msm_camera_sensor_mt9e013,*/
#endif
	&msm_kgsl_3d0,
#ifdef CONFIG_BT
	/*&msm_bt_power_device,*/
#endif
#ifdef CONFIG_MT9T013
/*	&msm_camera_sensor_mt9t013,*/
#endif
#ifdef CONFIG_BT
/*	&wifi_bt_slp_clk,*/
/*	&primods_rfkill,*/
/*	&msm_device_uart_dm1,*/
#endif
/*	&pm8029_leds,*/
};

#else
static struct platform_device simhotswap_slot1 = {
	.name	= "htc_simhotswap_slot1",
	.id = -1,
};
static struct platform_device simhotswap_slot2 = {
	.name	= "htc_simhotswap_slot2",
	.id = -1,
};
static struct platform_device *primominiu_devices[] __initdata = {
	&ram_console_device,
	&msm_device_dmov,
	&msm_device_smd,
#if 0
	&msm_device_uart1,
#endif
	/* &msm_device_uart3, */
	&msm_device_uart_dm1,
	&msm_device_uart_dm2,
	&msm_device_nand,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&htc_battery_pdev,
	&ds2746_battery_pdev,
	&msm_device_otg,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_adsp2_device,
	&android_pmem_audio_device,
	&msm_device_snd,
	&msm_device_adspdec,
/*	&lcdc_toshiba_panel_device,*/
	&msm_batt_device,
/*	&smsc911x_device,*/
	&htc_headset_mgr,
#ifdef CONFIG_S5K4E1
	&msm_camera_sensor_s5k4e1,
#endif
#ifdef CONFIG_IMX072
/*	&msm_camera_sensor_imx072,*/
#endif
#ifdef CONFIG_WEBCAM_OV9726
/*	&msm_camera_sensor_ov9726,*/
#endif
#ifdef CONFIG_MT9E013
/*	&msm_camera_sensor_mt9e013,*/
#endif
#ifdef CONFIG_FB_MSM_MIPI_DSI
/*	&mipi_dsi_renesas_panel_device,*/
#endif
	&msm_kgsl_3d0,
#ifdef CONFIG_BT
/*	&msm_bt_power_device,*/
#endif
	&primods_rfkill,
/*	&asoc_msm_pcm,*/
/*	&asoc_msm_dai0,*/
/*	&asoc_msm_dai1,*/
	&wifi_bt_fast_clk,
	&pm8029_leds,
	&cable_detect_device,
	&htc_drm,
	&simhotswap_slot1,
};
static struct platform_device *primods_devices[] __initdata = {
	&ram_console_device,
	&msm_device_dmov,
	&msm_device_smd,
#if 0
	&msm_device_uart1,
#endif
	/* &msm_device_uart3, */
	&msm_device_uart_dm1,
	&msm_device_uart_dm2,
	&msm_device_nand,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&htc_battery_pdev,
	&ds2746_battery_pdev,
	&msm_device_otg,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_adsp2_device,
	&android_pmem_audio_device,
	&msm_device_snd,
	&msm_device_adspdec,
/*	&lcdc_toshiba_panel_device,*/
	&msm_batt_device,
/*	&smsc911x_device,*/
	&htc_headset_mgr,
#ifdef CONFIG_S5K4E1
	&msm_camera_sensor_s5k4e1,
#endif
#ifdef CONFIG_IMX072
/*	&msm_camera_sensor_imx072,*/
#endif
#ifdef CONFIG_WEBCAM_OV9726
/*	&msm_camera_sensor_ov9726,*/
#endif
#ifdef CONFIG_MT9E013
/*	&msm_camera_sensor_mt9e013,*/
#endif
#ifdef CONFIG_FB_MSM_MIPI_DSI
/*	&mipi_dsi_renesas_panel_device,*/
#endif
	&msm_kgsl_3d0,
#ifdef CONFIG_BT
/*	&msm_bt_power_device,*/
#endif
	&primods_rfkill,
/*	&asoc_msm_pcm,*/
/*	&asoc_msm_dai0,*/
/*	&asoc_msm_dai1,*/
	&wifi_bt_fast_clk,
	&pm8029_leds,
	&cable_detect_device,
	&htc_drm,
	&simhotswap_slot1,
	&simhotswap_slot2,
};
#endif
static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static struct memtype_reserve msm7x27a_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
        android_pmem_adsp2_pdata.size = pmem_adsp2_size;
	android_pmem_pdata.size = pmem_mdp_size;
	android_pmem_audio_pdata.size = pmem_audio_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x27a_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_adsp2_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
}

static void __init msm7x27a_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x27a_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27a_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27a_reserve_table,
	.calculate_reserve_sizes = msm7x27a_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27a_paddr_to_memtype,
};

static void __init msm7x27a_reserve(void)
{
	reserve_info = &msm7x27a_reserve_info;
	msm_reserve();
}

static void __init msm_device_i2c_init(void)
{
	msm_gsbi0_qup_i2c_device.dev.platform_data = &msm_gsbi0_qup_i2c_pdata;
	msm_gsbi1_qup_i2c_device.dev.platform_data = &msm_gsbi1_qup_i2c_pdata;
}

static int msm7x27a_ts_himax_power(int on)
{
	printk(KERN_INFO "%s():\n", __func__);
	gpio_set_value(PRIMODS_V_TP_3V3_EN, on);

	return 0;
}

static void msm7x27a_ts_himax_reset(void)
{
	printk(KERN_INFO "%s():\n", __func__);
	gpio_direction_output(PRIMODS_GPIO_TP_RST_N, 0);
	mdelay(20);
	gpio_direction_output(PRIMODS_GPIO_TP_RST_N, 1);
	mdelay(50);
}

struct himax_i2c_platform_data_config_type_3 config_type3[] = {
	{
		.version = 0x0D,
		.common = 1,
		.tw_id = 0,
		.x_fuzz = 4,
		.y_fuzz = 4,
		.z_fuzz = 6,

		.c1 = { 0x62, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c2 = { 0x63, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c3 = { 0x64, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c4 = { 0x65, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c5 = { 0x66, 0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c6 = { 0x67, 0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c7 = { 0x68, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c8 = { 0x69, 0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c9 = { 0x6A, 0x32, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c10 = { 0x6B, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c11 = { 0x6C, 0x12, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c12 = { 0x6D, 0x41, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 },
		.c13 = { 0xC9, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0A, 0x0B,
			  0x0D, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x20, 0x1E,
			  0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c14 = { 0x8A, 0x00, 0x04, 0x00, 0x04, 0x00, 0x08, 0xCC, 0x10, 0x7E, 0x72,
			  0x25, 0x0A, 0x00, 0xB0, 0x02, 0x51, 0x3B, 0x41, 0x0A, 0x00, 0x0B,
			  0x07, 0xFF, 0xFF, 0x0C, 0x06, 0xFF, 0xFF, 0x0D, 0xFF, 0x08, 0xFF,
			  0x0E, 0x05, 0xFF, 0xFF, 0x0F, 0xFF, 0x09, 0x1A, 0xFF, 0x04, 0x1C,
			  0x19, 0xFF, 0xFF, 0x0A, 0x18, 0xFF, 0x03, 0x1B, 0x17, 0xFF, 0x02,
			  0x1D, 0x16, 0x10, 0xFF, 0xFF, 0x15, 0x11, 0x01, 0xFF, 0x14, 0x00,
			  0x12, 0xFF, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c15 = { 0xC5, 0x0D, 0x1F, 0x00, 0x10, 0x1B, 0x1F, 0x0B },
		.c16 = { 0xC6, 0x11, 0x10, 0x17 },
		.c17 = { 0x7D, 0x00, 0x04, 0x0A, 0x0A, 0x02 },
		.c18 = { 0x7F, 0x0A, 0x01, 0x01, 0x01, 0x01, 0x06, 0x04, 0x07, 0x0D, 0x07,
			  0x0D, 0x07, 0x0D, 0x00 },
		.c19 = { 0xD5, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c20 = { 0xE9, 0x00, 0x00 },
		.c21 = { 0xEA, 0x0B, 0x13, 0x00, 0x13 },
		.c22 = { 0xEB, 0x32, 0x32, 0xFA, 0x83 },
		.c23 = { 0xEC, 0x04, 0x0F, 0x0A, 0x2D, 0x2D, 0x00, 0x00, 0x00, 0x00 },
		.c24 = { 0xED, 0x0F, 0x06, 0x00, 0x00 },
		.c25 = { 0xEE, 0x00 },
		.c26 = { 0xEF, 0x11, 0x00 },
		.c27 = { 0xF0, 0x40 },
		.c28 = { 0xF1, 0x06, 0x04, 0x06, 0x03 },
		.c29 = { 0xF2, 0x0A, 0x06, 0x14, 0x3C },
		.c30 = { 0xF3, 0x5F },
		.c31 = { 0xF4, 0x7D, 0xB4, 0x2D, 0x3A },
		.c32 = { 0xF6, 0x00, 0x00, 0x1B, 0x76, 0x0A },
		.c33 = { 0xF7, 0x20, 0x64, 0xFF, 0x0F, 0x40 },
		.c34 = { 0x35, 0x02, 0x01 },
		.c35 = { 0x36, 0x0F, 0x53, 0x01 },
		.c36 = { 0x37, 0xFF, 0x08, 0xFF, 0x08 },
		.c37 = { 0x39, 0x03 },
		.c38 = { 0x3A, 0x00 },
		.c39 = { 0x50, 0xAB },
		.c40 = { 0x6E, 0x04 },
		.c41 = { 0x76, 0x01, 0x2D },
		.c42 = { 0x78, 0x03 },
		.c43 = { 0x7A, 0x00, 0x18, 0x0D },
		.c44 = { 0x8B, 0x00, 0x00 },
		.c45 = { 0x8C, 0x30, 0x0C, 0x0C, 0x0C, 0x0C, 0x08, 0x0C, 0x32, 0x24, 0x80,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c46 = { 0x8D, 0xA0, 0x5A, 0x14, 0x0A, 0x32, 0x0A, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c47 = { 0xC2, 0x11, 0x00, 0x00, 0x00 },
		.c48 = { 0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x00, 0x05, 0x00, 0x05, 0x00 },
		.c49 = { 0xD4, 0x01, 0x04, 0x07 },
		.c50 = { 0xDD, 0x05, 0x02 },
		.checksum = { 0xAC, 0x93, 0x4C },
	},
	{
		.version = 0x13,
		.common = 1,
		.tw_id = 0,
		.x_fuzz = 4,
		.y_fuzz = 4,
		.z_fuzz = 6,

		.c1 = { 0x62, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c2 = { 0x63, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c3 = { 0x64, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c4 = { 0x65, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c5 = { 0x66, 0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c6 = { 0x67, 0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c7 = { 0x68, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00 },
		.c8 = { 0x69, 0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c9 = { 0x6A, 0x32, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c10 = { 0x6B, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c11 = { 0x6C, 0x12, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 },
		.c12 = { 0x6D, 0x41, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 },
		.c13 = { 0xC9, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0A, 0x0B,
			 0x0D, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x20, 0x1E,
			 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c14 = { 0x8A, 0x00, 0x04, 0x00, 0x04, 0x00, 0x08, 0xCC, 0x10, 0x7E, 0x72,
			 0x25, 0x0A, 0x00, 0xB0, 0x02, 0x51, 0x3B, 0x41, 0x0A, 0x00, 0x0B,
			 0x07, 0xFF, 0xFF, 0x0C, 0x06, 0xFF, 0xFF, 0x0D, 0xFF, 0x08, 0xFF,
			 0x0E, 0x05, 0xFF, 0xFF, 0x0F, 0xFF, 0x09, 0x1A, 0xFF, 0x04, 0x1C,
			 0x19, 0xFF, 0xFF, 0x0A, 0x18, 0xFF, 0x03, 0x1B, 0x17, 0xFF, 0x02,
			 0x1D, 0x16, 0x10, 0xFF, 0xFF, 0x15, 0x11, 0x01, 0xFF, 0x14, 0x00,
			 0x12, 0xFF, 0x13, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0A, 0x0B, 0x0D,
			 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x20, 0x1E, 0x21,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c15 = { 0xC5, 0x0D, 0x1F, 0x00, 0x10, 0x1B, 0x1F, 0x0B },
		.c16 = { 0xC6, 0x11, 0x10, 0x17 },
		.c17 = { 0x7D, 0x00, 0x04, 0x0A, 0x0A, 0x02 },
		.c18 = { 0x7F, 0x09, 0x01, 0x01, 0x01, 0x01, 0x06, 0x08, 0x06, 0x0F, 0x06,
			 0x0F, 0x06, 0x0F, 0x00 },
		.c19 = { 0xD5, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c20 = { 0xE9, 0x00, 0x00 },
		.c21 = { 0xEA, 0x0B, 0x13, 0x00, 0x13 },
		.c22 = { 0xEB, 0x32, 0x29, 0xFA, 0x83 },
		.c23 = { 0xEC, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c24 = { 0xED, 0x08, 0xFF, 0x00, 0x00 },
		.c25 = { 0xEE, 0x00 },
		.c26 = { 0xEF, 0x11, 0x00 },
		.c27 = { 0xF0, 0x40 },
		.c28 = { 0xF1, 0x06, 0x04, 0x06, 0x03 },
		.c29 = { 0xF2, 0x0A, 0x06, 0x14, 0x3C },
		.c30 = { 0xF3, 0x5F },
		.c31 = { 0xF4, 0x7D, 0xB4, 0x2D, 0x3A },
		.c32 = { 0xF6, 0x00, 0x00, 0x1B, 0x76, 0x09 },
		.c33 = { 0xF7, 0x20, 0x68, 0x7F, 0x0F, 0x40 },
		.c34 = { 0x35, 0x02, 0x01 },
		.c35 = { 0x36, 0x0F, 0x53, 0x01 },
		.c36 = { 0x37, 0xFF, 0x08, 0xFF, 0x08 },
		.c37 = { 0x39, 0x03 },
		.c38 = { 0x3A, 0x00 },
		.c39 = { 0x50, 0xAB },
		.c40 = { 0x6E, 0x04 },
		.c41 = { 0x76, 0x01, 0x2D },
		.c42 = { 0x78, 0x03 },
		.c43 = { 0x7A, 0x00, 0x18, 0x0D },
		.c44 = { 0x8B, 0x00, 0x00 },
		.c45 = { 0x8C, 0x30, 0x0C, 0x08, 0x0C, 0x0C, 0x04, 0x0C, 0x28, 0x24, 0x28,
			 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00, 0x00 },
		.c46 = { 0x8D, 0xA0, 0x5A, 0x14, 0x0A, 0x32, 0x0A, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.c47 = { 0xC2, 0x11, 0x00, 0x00, 0x00 },
		.c48 = { 0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x00, 0x05, 0x00, 0x05, 0x00 },
		.c49 = { 0xD4, 0x01, 0x04, 0x07 },
		.c50 = { 0xDD, 0x05, 0x02 },
		.checksum = { 0xAC, 0x47, 0x4E },
	},
};

struct himax_i2c_platform_data_config_type_2 config_type2[] = {
	{
		.version = 0x0B,
		.tw_id = 1,

		.c1 = { 0x36, 0x0F, 0x53 },
		.c2 = { 0xDD, 0x04, 0x02 },
		.c3 = { 0x37, 0xFF, 0x08, 0xFF, 0x08 },
		.c4 = { 0x39, 0x03 },
		.c5 = { 0x3A, 0x00 },
		.c6 = { 0x6E, 0x04 },
		.c7 = { 0x76, 0x01, 0x3F },
		.c8 = { 0x78, 0x03 },
		.c9 = { 0x7A, 0x00, 0x18, 0x0D },
		.c10 = { 0x7D, 0x00, 0x04, 0x0A, 0x0A, 0x04 },
		.c11 = { 0x7F, 0x05, 0x01, 0x01, 0x01, 0x01, 0x07, 0x0D, 0x0B, 0x0D, 0x0B,
			  0x0D, 0x02, 0x0B, 0x00 },
		.c12 = { 0xC2, 0x11, 0x00, 0x00, 0x00 },
		.c13 = { 0xC5, 0x0A, 0x1E, 0x00, 0x10, 0x1A, 0x1F, 0x0B },
		.c14 = { 0xC6, 0x11, 0x10, 0x16 },
		.c15 = { 0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x00, 0x05, 0x00, 0x05, 0x00 },
		.c16 = { 0xD4, 0x01, 0x04, 0x07 },
		.c17 = { 0xD5, 0xA5 },

		.c18 = { 0x62, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 },
		.c19 = { 0x63, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00 },
		.c20 = { 0x64, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 },
		.c21 = { 0x65, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00 },
		.c22 = { 0x66, 0x41, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x00 },
		.c23 = { 0x67, 0x34, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x00 },
		.c24 = { 0x68, 0x40, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00 },
		.c25 = { 0x69, 0x34, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x00 },
		.c26 = { 0x6A, 0x43, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x00 },
		.c27 = { 0x6B, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00 },
		.c28 = { 0x6C, 0x41, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x00 },
		.c29 = { 0x6D, 0x24, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00 },

		.c30 = { 0xC9, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x0E, 0x0E, 0x10, 0x10,
			  0x11, 0x11, 0x13, 0x13, 0x15, 0x15, 0x17, 0x17, 0x18, 0x18, 0x1B,
			  0x1B, 0x1D, 0x1D, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },

		.c31 = { 0x8A, 0x04, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38,
			  0xE4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x66, 0x00, 0x00, 0x00,
			  0x1A, 0xFF, 0xFF, 0x01, 0x19, 0xFF, 0xFF, 0x02, 0xFF, 0x1B, 0xFF,
			  0x03, 0x18, 0xFF, 0xFF, 0x04, 0xFF, 0x1C, 0x0F, 0xFF, 0x17, 0x11,
			  0x0E, 0xFF, 0xFF, 0x1D, 0x0D, 0xFF, 0x16, 0x10, 0x0C, 0xFF, 0x15,
			  0x12, 0x0B, 0x05, 0xFF, 0xFF, 0x0A, 0x06, 0x14, 0xFF, 0x09, 0x13,
			  0x07, 0xFF, 0x08},

		.c32 = { 0x8C, 0x30, 0x0C, 0x0A, 0x0C, 0x08, 0x08, 0x08, 0x32, 0x24, 0x40 },
		.c33 = { 0xE9, 0x00 },
		.c34 = { 0xEA, 0x13, 0x0B, 0x00, 0x24 },

		.c35 = { 0xEB, 0x28, 0x32, 0x8A, 0x83 },
		.c36 = { 0xEC, 0x00, 0x0F, 0x0A, 0x2D, 0x2D, 0x00, 0x00 },
		.c37 = { 0xEF, 0x11, 0x00 },
		.c38 = { 0xF0, 0x40 },
		.c39 = { 0xF1, 0x06, 0x04, 0x06, 0x03 },
		.c40 = { 0xF2, 0x0A, 0x06, 0x14, 0x3C },
		.c41 = { 0xF3, 0x07 },
		.c42 = { 0xF4, 0x7D, 0x96, 0x1E, 0xC8 },
		.c43 = { 0xF6, 0x00, 0x00, 0x14, 0x2A, 0x05 },
		.c44 = { 0xF7, 0x20, 0x4E, 0x00, 0x00, 0x00 },
		.c45 = { 0xED, 0x03, 0x06},
	},
};

struct himax_i2c_platform_data msm7x27a_ts_himax_data = {
	.slave_addr = 0x90,
	.abs_x_min = 0,
	.abs_x_max = 1024,
	.abs_y_min = 0,
	.abs_y_max = 947,
	.abs_pressure_min = 0,
	.abs_pressure_max = 200,
	.abs_width_min = 0,
	.abs_width_max = 200,
	.gpio_irq = PRIMODS_GPIO_TP_ATT_N,
	.gpio_reset = PRIMODS_GPIO_TP_RST_N,
	.version = 0x00,
	.tw_id = 0,
	.event_htc_enable = 0,
	.cable_config = { 0x90, 0x00},
	.power = msm7x27a_ts_himax_power,
	.powerOff3V3 = 0,
	.reset = msm7x27a_ts_himax_reset,
	.protocol_type = PROTOCOL_TYPE_B,
	.screenWidth = 480,
	.screenHeight = 800,
	.ID0 = "ALPS",
	.ID1 = "J-Touch",
	.ID2 = "N/A",
	.ID3 = "N/A",
	.type1 = 0,
	.type1_size = 0,
	.type2 = config_type2,
	.type2_size = sizeof(config_type2),
	.type3 = config_type3,
	.type3_size = sizeof(config_type3),
};

static struct i2c_board_info i2c_touch_device[] = {
	{
		I2C_BOARD_INFO(HIMAX8526A_NAME, 0x90>>1),
		.platform_data  = &msm7x27a_ts_himax_data,
		.irq = MSM_GPIO_TO_INT(PRIMODS_GPIO_TP_ATT_N),
	},
};

static ssize_t msm7x27a_virtual_keys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_BACK)	    ":54:853:117:90"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":245:857:97:90"
		":" __stringify(EV_KEY) ":" __stringify(KEY_APP_SWITCH) ":411:856:101:90"
		"\n");
}
static struct kobj_attribute msm7x27a_himax_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.himax-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &msm7x27a_virtual_keys_show,
};

static struct attribute *msm7x27a_properties_attrs[] = {
	&msm7x27a_himax_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group msm7x27a_properties_attr_group = {
	.attrs = msm7x27a_properties_attrs,
};

#define MSM_EBI2_PHYS			0xa0d00000
#define MSM_EBI2_XMEM_CS2_CFG1		0xa0d10030

static void __init msm7x27a_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void __iomem *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_PHYS, sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_rumi3() || machine_is_msm7x27a_surf() ||
			machine_is_msm7625a_surf())
		ebi2_cfg |= (1 << 4); /* CS2 */

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);

	/* Enable A/D MUX[bit 31] from EBI2_XMEM_CS2_CFG1 */
	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_XMEM_CS2_CFG1,
							 sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf())
		ebi2_cfg |= (1 << 31);

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);
}




#if 0
#define KP_INDEX(row, col) ((row)*ARRAY_SIZE(kp_col_gpios) + (col))

static unsigned int kp_row_gpios[] = {31, 32, 33, 34, 35};
static unsigned int kp_col_gpios[] = {36, 37, 38, 39, 40};

static const unsigned short keymap[ARRAY_SIZE(kp_col_gpios) *
					  ARRAY_SIZE(kp_row_gpios)] = {
	[KP_INDEX(0, 0)] = KEY_7,
	[KP_INDEX(0, 1)] = KEY_DOWN,
	[KP_INDEX(0, 2)] = KEY_UP,
	[KP_INDEX(0, 3)] = KEY_RIGHT,
	[KP_INDEX(0, 4)] = KEY_ENTER,

	[KP_INDEX(1, 0)] = KEY_LEFT,
	[KP_INDEX(1, 1)] = KEY_SEND,
	[KP_INDEX(1, 2)] = KEY_1,
	[KP_INDEX(1, 3)] = KEY_4,
	[KP_INDEX(1, 4)] = KEY_CLEAR,

	[KP_INDEX(2, 0)] = KEY_6,
	[KP_INDEX(2, 1)] = KEY_5,
	[KP_INDEX(2, 2)] = KEY_8,
	[KP_INDEX(2, 3)] = KEY_3,
	[KP_INDEX(2, 4)] = KEY_NUMERIC_STAR,

	[KP_INDEX(3, 0)] = KEY_9,
	[KP_INDEX(3, 1)] = KEY_NUMERIC_POUND,
	[KP_INDEX(3, 2)] = KEY_0,
	[KP_INDEX(3, 3)] = KEY_2,
	[KP_INDEX(3, 4)] = KEY_SLEEP,

	[KP_INDEX(4, 0)] = KEY_BACK,
	[KP_INDEX(4, 1)] = KEY_HOME,
	[KP_INDEX(4, 2)] = KEY_MENU,
	[KP_INDEX(4, 3)] = KEY_VOLUMEUP,
	[KP_INDEX(4, 4)] = KEY_VOLUMEDOWN,
};

/* SURF keypad platform device information */
static struct gpio_event_matrix_info kp_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keymap,
	.output_gpios	= kp_row_gpios,
	.input_gpios	= kp_col_gpios,
	.noutputs	= ARRAY_SIZE(kp_row_gpios),
	.ninputs	= ARRAY_SIZE(kp_col_gpios),
	.settle_time.tv_nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv_nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS,
};

static struct gpio_event_info *kp_info[] = {
	&kp_matrix_info.info
};

static struct gpio_event_platform_data kp_pdata = {
	.name		= "7x27a_kp",
	.info		= kp_info,
	.info_count	= ARRAY_SIZE(kp_info)
};

static struct platform_device kp_pdev = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &kp_pdata,
	},
};
#endif
static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_pdev = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct platform_device msm_proccomm_regulator_dev = {
	.name   = PROCCOMM_REGULATOR_DEV_NAME,
	.id     = -1,
	.dev    = {
		.platform_data = &msm7x27a_proccomm_regulator_data
	}
};

#if 0
static void __init msm7627a_rumi3_init(void)
{
	msm7x27a_init_ebi2();
	platform_add_devices(rumi_sim_devices,
			ARRAY_SIZE(rumi_sim_devices));
}
#endif
#define LED_GPIO_PDM		17
#define UART1DM_RX_GPIO		45

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static int __init msm7x27a_init_ar6000pm(void)
{
	return platform_device_register(&msm_wlan_ar6000_pm_device);
}
#else
static int __init msm7x27a_init_ar6000pm(void) { return 0; }
#endif

static void __init msm7x27a_init_regulators(void)
{
	int rc = platform_device_register(&msm_proccomm_regulator_dev);
	if (rc)
		pr_err("%s: could not register regulator device: %d\n",
				__func__, rc);
}

void primods_add_usb_devices(void)
{
	printk(KERN_INFO "%s rev: %d\n", __func__, system_rev);
	android_usb_pdata.products[0].product_id =
			android_usb_pdata.product_id;

	/* diag bit set */
	if (get_radio_flag() & 0x20000)
		android_usb_pdata.diag_init = 1;

	/* add cdrom support in normal mode */
	if (board_mfg_mode() == 0) {
		android_usb_pdata.nluns = 3;
		android_usb_pdata.cdrom_lun = 0x4;
	}

	config_primods_usb_id_gpios(0);
	msm_device_gadget_peripheral.dev.parent = &msm_device_otg.dev;
	platform_device_register(&msm_device_gadget_peripheral);
	platform_device_register(&android_usb_device);
}

static int __init board_serialno_setup(char *serialno)
{
	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);


static void primods_reset(void)
{
	gpio_set_value(PRIMODS_GPIO_PS_HOLD, 0);
}

static void primods_add_bt_devices(void)
{
	platform_device_register(&wl128x_device);
	platform_device_register(&btwilink_device);
}

#if 0
static void __init primods_init(void)
{
	msm7x2x_misc_init();

	printk(KERN_INFO "primods_init() revision = 0x%x\n", system_rev);
	printk(KERN_INFO "MSM_PMEM_MDP_BASE=0x%x MSM_PMEM_ADSP_BASE=0x%x MSM_RAM_CONSOLE_BASE=0x%x MSM_FB_BASE=0x%x\n",
		MSM_PMEM_MDP_BASE, MSM_PMEM_ADSP_BASE, MSM_RAM_CONSOLE_BASE, MSM_FB_BASE);
	/* Must set msm_hw_reset_hook before first proc comm */
	msm_hw_reset_hook = primods_reset;

#ifdef CONFIG_PERFLOCK_BOOT_LOCK
	perflock_init(&holiday_perflock_data);
#endif

	/* Common functions for SURF/FFA/RUMI3 */
	msm_device_i2c_init();

	rc = primods_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

#ifdef CONFIG_BT
	bt_export_bd_address();
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.rx_wakeup_irq = gpio_to_irq(PRIMODS_GPIO_BT_HOST_WAKE);
	msm_device_uart_dm1.name = "msm_serial_hs_brcm"; /* for brcm */
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_otg_pdata.swfi_latency =
		msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
#endif
/*	headset_init();*/
	platform_add_devices(msm_footswitch_devices,
		msm_num_footswitch_devices);
	platform_add_devices(primods_devices,
			ARRAY_SIZE(primods_devices));

	msm_pm_set_platform_data(msm7x27a_pm_data,
				ARRAY_SIZE(msm7x27a_pm_data));

	primods_init_panel();
#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
	register_i2c_devices();
#endif
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	bt_power_init();
#endif

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			i2c_tps65200_devices, ARRAY_SIZE(i2c_tps65200_devices));
#ifdef CONFIG_MSM_CAMERA
	i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID,
			i2c_camera_devices,
			ARRAY_SIZE(i2c_camera_devices));
#endif
#if 0
#if 0
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			i2c_bma250_devices, ARRAY_SIZE(i2c_bma250_devices));
#endif
	/* Disable loading because of no Cypress chip consider by pcbid */
	if (system_rev >= 0x80) {
		printk(KERN_INFO "No Cypress chip!\n");
		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			i2c_touch_pvt_device, ARRAY_SIZE(i2c_touch_pvt_device));
	} else
		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			i2c_touch_device, ARRAY_SIZE(i2c_touch_device));

	pl_sensor_init();
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID, i2c_CM3628_devices,
				ARRAY_SIZE(i2c_CM3628_devices));
#endif
	primods_init_keypad();
	primods_wifi_init();

#ifdef CONFIG_MSM_RPC_VIBRATOR
/*	msm_init_pmic_vibrator();*/
#endif
#ifdef CONFIG_USB_ANDROID
	primods_add_usb_devices();
#endif
#ifdef CONFIG_MSM_HTC_DEBUG_INFO
	htc_debug_info_init();
#endif
#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
				&msm_device_uart3.dev, 1,
				MSM_GPIO_TO_INT(PRIMODS_GPIO_UART3_RX));
#endif
	/*7x25a kgsl initializations*/
	msm7x25a_kgsl_3d0_init();
}

#else
static void __init primods_init(void)
{
	int rc = 0;
	struct kobject *properties_kobj;
	struct proc_dir_entry *entry = NULL;

	msm7x2x_misc_init();

	printk(KERN_INFO "primods_init() revision = 0x%x\n", system_rev);

#ifdef CONFIG_PERFLOCK
	perflock_init(&primods_perflock_data);
#endif

	/* Must set msm_hw_reset_hook before first proc comm */
	msm_hw_reset_hook = primods_reset;

	/* Initialize regulators first so that other devices can use them */
	msm7x27a_init_regulators();

	/* Common functions for SURF/FFA/RUMI3 */
	msm_device_i2c_init();
	msm7x27a_init_ebi2();
	msm7x27a_cfg_uart2dm_serial();
#ifdef CONFIG_SERIAL_MSM_HS
	/*not defined in Primo#DS*/
	/*msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(PRIMODS_GPIO_BT_HOST_WAKE);*/
	msm_device_uart_dm1.name = "msm_serial_hs_ti_dc";
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

#if defined(CONFIG_USB_MSM_OTG_72K) && defined(CONFIG_USB_GADGET)
	msm_otg_pdata.swfi_latency =
		msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
	msm_device_otg.dev.platform_data = &msm_otg_pdata;

	/*msm7x27a_cfg_smsc911x();*/
	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);
	if (get_kernel_flag() & 0x02) {
		htc_headset_mgr_data.headset_devices_num--;
		htc_headset_mgr_data.headset_devices[2] = NULL;
	}
	else {
		platform_device_register(&msm_device_uart1);
	}
	if (machine_is_primods()) {
		platform_add_devices(primods_devices,
			ARRAY_SIZE(primods_devices));
	} else {
		platform_add_devices(primominiu_devices,
			ARRAY_SIZE(primominiu_devices));
	}
	/*usb driver won't be loaded in MFG 58 station and gift mode*/
	if (!(board_mfg_mode() == 6 || board_mfg_mode() == 7))
		primods_add_usb_devices();
#ifdef CONFIG_USB_EHCI_MSM_72K
	msm_add_host(0, &msm_usb_host_pdata);
#endif

	/* Ensure ar6000pm device is registered before MMC/SDC */
	msm7x27a_init_ar6000pm();

	primods_wifi_init();

	primods_audio_init();

#ifdef CONFIG_MMC_MSM
	msm7x27a_init_mmc();
	entry = create_proc_read_entry("emmc", 0, NULL, emmc_partition_read_proc, NULL);
	if (!entry)
		printk(KERN_ERR"Create /proc/emmc failed!\n");
#endif

	entry = create_proc_read_entry("dying_processes", 0, NULL, dying_processors_read_proc, NULL);
	if (!entry)
		printk(KERN_ERR "Create /proc/dying_processes FAILED!\n");

#ifdef CONFIG_USB_EHCI_MSM_72K
	msm7x2x_init_host();
#endif

	msm_pm_set_platform_data(msm7x27a_pm_data,
				ARRAY_SIZE(msm7x27a_pm_data));
	BUG_ON(msm_pm_boot_init(MSM_PM_BOOT_CONFIG_RESET_VECTOR, ioremap(0x0, PAGE_SIZE)));
	primods_init_panel();
#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
	register_i2c_devices();
#endif
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	bt_power_init();
#endif

	primods_add_bt_devices();


	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			i2c_tps65200_devices, ARRAY_SIZE(i2c_tps65200_devices));

#ifdef CONFIG_FLASHLIGHT_TPS61310
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				tps61310_i2c_info,
				ARRAY_SIZE(tps61310_i2c_info));
#endif


	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			i2c_bma250_devices, ARRAY_SIZE(i2c_bma250_devices));



#ifdef CONFIG_CODEC_AIC3254
	aic3254_lowlevel_init();
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			i2c_aic3254_devices, ARRAY_SIZE(i2c_aic3254_devices));
#endif

#if defined(CONFIG_MSM_CAMERA)
/* HTC_START mark 20111221*/
	/* msm_camera_vreg_init(); */
/* HTC_END */
	i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID,
			i2c_camera_devices,
			ARRAY_SIZE(i2c_camera_devices));
#endif
#if 0
	platform_device_register(&kp_pdev);
#endif
	platform_device_register(&hs_pdev);

#if 0
	/* configure it as a pdm function*/
	if (gpio_tlmm_config(GPIO_CFG(LED_GPIO_PDM, 3,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, LED_GPIO_PDM);
	else
		platform_device_register(&led_pdev);
#endif

		msm_init_pmic_vibrator(3000);
		platform_device_register(&gpio_leds);

#if 0
#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
				&msm_device_uart3.dev, 1,
				MSM_GPIO_TO_INT(PRIMODS_GPIO_UART3_RX));
#endif
#endif
	/*7x25a kgsl initializations*/
	msm7x25a_kgsl_3d0_init();
	if (system_rev < 2) {
		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			i2c_cm36282_devices, ARRAY_SIZE(i2c_cm36282_devices));
		pr_info("%s: cm36282 PL-sensor for XA,XB system_rev %d ",
				 __func__, system_rev);
	} else {
		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			i2c_cm36282_XC_devices, ARRAY_SIZE(i2c_cm36282_XC_devices));
		pr_info("%s: cm36282 PL-sensor for XC and newer HW version, system_rev %d ",
				__func__, system_rev);
	}

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				i2c_touch_device,
				ARRAY_SIZE(i2c_touch_device));

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
						&msm7x27a_properties_attr_group);
	if (!properties_kobj || rc)
		printk(KERN_ERR "[TP]failed to create board_properties\n");
	else {
		msm7x27a_ts_himax_data.vk_obj = properties_kobj;
		msm7x27a_ts_himax_data.vk2Use = &msm7x27a_himax_virtual_keys_attr;
	}

	primods_init_keypad();

	if (get_kernel_flag() & KERNEL_FLAG_PM_MONITOR) {
		htc_monitor_init();
		htc_PM_monitor_init();
	}
}
#endif

static void __init primods_fixup(struct machine_desc *desc, struct tag *tags,
							char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = MSM_LINUX_BASE;
	mi->bank[0].size = MSM_LINUX_SIZE;
}

MACHINE_START(PRIMODS, "primods")
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup = primods_fixup,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= primods_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(PRIMOMINIU, "primods")
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup = primods_fixup,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= primods_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(PRIMODSU, "primods")
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup = primods_fixup,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= primods_init,
	.timer		= &msm_timer,
MACHINE_END
