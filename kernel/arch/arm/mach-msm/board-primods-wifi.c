/* linux/arch/arm/mach-msm/board-primods-wifi.c
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/skbuff.h>
#include <linux/wifi_tiwlan.h>
#include <mach/htc_fast_clk.h>

#include "board-primods.h"
#include "devices.h"
#include "proc_comm.h"
#include <mach/vreg.h>

#include <linux/wl12xx.h>

#define ID_WIFI	0
#define CLK_OFF	0
#define CLK_ON	1

static struct wl12xx_platform_data primods_wlan_data __initdata = {
       .irq = MSM_GPIO_TO_INT(PRIMODS_GPIO_WIFI_IRQ),
       .board_ref_clock = WL12XX_REFCLOCK_26,
       .board_tcxo_clock = 1,
};

static uint32_t wifi_on_gpio_table[] = {
	GPIO_CFG(PRIMODS_GPIO_SD_D3, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA), /* DAT3 */
	GPIO_CFG(PRIMODS_GPIO_SD_D2, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA), /* DAT2 */
	GPIO_CFG(PRIMODS_GPIO_SD_D1, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA), /* DAT1 */
	GPIO_CFG(PRIMODS_GPIO_SD_D0, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA), /* DAT0 */
	GPIO_CFG(PRIMODS_GPIO_SD_CMD, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), /* CMD */
	GPIO_CFG(PRIMODS_GPIO_SD_CLK_0, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* CLK */
	GPIO_CFG(PRIMODS_GPIO_WIFI_IRQ, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), /* WLAN IRQ */
	GPIO_CFG(PRIMODS_GPIO_WIFI_SHUTDOWN_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* WLAN EN */
};

static uint32_t wifi_off_gpio_table[] = {
	GPIO_CFG(PRIMODS_GPIO_SD_D3, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), /* DAT3 */
	GPIO_CFG(PRIMODS_GPIO_SD_D2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), /* DAT2 */
	GPIO_CFG(PRIMODS_GPIO_SD_D1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), /* DAT1 */
	GPIO_CFG(PRIMODS_GPIO_SD_D0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), /* DAT0 */
	GPIO_CFG(PRIMODS_GPIO_SD_CMD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), /* CMD */
	GPIO_CFG(PRIMODS_GPIO_SD_CLK_0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), /* CLK */
	GPIO_CFG(PRIMODS_GPIO_WIFI_IRQ, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA), /* WLAN IRQ */
	GPIO_CFG(PRIMODS_GPIO_WIFI_SHUTDOWN_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* WLAN EN */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[WLAN] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

int ti_wifi_power(int on)
{
	printk(KERN_INFO "[WLAN] %s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table, ARRAY_SIZE(wifi_on_gpio_table));
		/* control osc */
		htc_wifi_bt_fast_clk_ctl(CLK_ON, ID_WIFI);
		msleep(200);
		gpio_set_value(PRIMODS_GPIO_WIFI_SHUTDOWN_N, 1); /* WIFI_SHUTDOWN */
		msleep(15);
		gpio_set_value(PRIMODS_GPIO_WIFI_SHUTDOWN_N, 0);
		msleep(1);
		gpio_set_value(PRIMODS_GPIO_WIFI_SHUTDOWN_N, 1);
		msleep(70);
	} else {
		gpio_set_value(PRIMODS_GPIO_WIFI_SHUTDOWN_N,0);
		msleep(1);
		config_gpio_table(wifi_off_gpio_table, ARRAY_SIZE(wifi_off_gpio_table));
		msleep(10);
		/* control osc */
		htc_wifi_bt_fast_clk_ctl(CLK_OFF, ID_WIFI);
	}
	msleep(250);
	printk(KERN_INFO "[WLAN] %s: ---\n", __func__);
	return 0;
}
EXPORT_SYMBOL(ti_wifi_power);

int __init primods_wifi_init(void)
{
	int ret = 0, id = 0, rc = 0;

	printk(KERN_INFO "[WLAN] %s: start\n", __func__);

	id = GPIO_CFG(PRIMODS_GPIO_WIFI_BT_FAST_CLK_EN, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
	rc= gpio_tlmm_config(id, 0);
	if (rc) {
		pr_err("[WLAN] %s: gpio_tlmm_config(%#x)=%d\n",
			__func__, id, rc);
		return rc;
	}

	if(wl12xx_set_platform_data(&primods_wlan_data))
		pr_err("[WLAN] Error setting wl12xx_data\n");

	return ret;
}
