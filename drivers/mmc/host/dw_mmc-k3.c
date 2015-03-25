/*
 * Copyright (c) 2013 Linaro Ltd.
 * Copyright (c) 2013 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/mmc/dw_mmc.h>
#include <linux/mfd/syscon.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

#define AO_SCTRL_SEL18		BIT(10)
#define AO_SCTRL_CTRL3		0x40C

#define SDMMC_CCLK_MAX_24M     24000000
#define SDMMC_CCLK_MAX_25M     25000000
#define SDMMC_CCLK_MAX_48M     48000000
#define SDMMC_CCLK_MAX_50M     50000000
#define SDMMC_CCLK_MAX_80M     80000000
#define SDMMC_CCLK_MAX_96M     96000000
#define SDMMC_CCLK_MAX_100M    100000000
#define SDMMC_CCLK_MAX_150M    150000000
#define SDMMC_CCLK_MAX_180M    180000000
#define SDMMC_CCLK_MAX_200M    200000000

#define GET_FREQ(id, f1, f2) (id == MMC_SD_ID ? f1 : f2)

static int dw_mci_k3_early_ctrl_reset(struct dw_mci *host)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(2000);
	const u32 reset = BIT(31);
	u32 ctrl;

	mci_writel(host, CTRL, SDMMC_CTRL_FIFO_RESET);
	mci_writel(host, CMD, SDMMC_CMD_START | SDMMC_CMD_DISABLE_BOOT);

	do {
		ctrl = mci_readl(host, CMD);
		if (!(ctrl & reset))
			return 0;
	} while (time_before(jiffies, timeout));

	dev_err(host->dev,
		"Timeout resetting MMC block (ctrl reset %#x)\n", ctrl & reset);

	return -ENODEV;
}

static void dw_mci_k3_set_ios(struct dw_mci *host, struct mmc_ios *ios)
{
	struct dw_mci_priv_data *priv;
	u32 freq;
	int id;

	priv = host->priv;
	if (!priv)
		return;

	id = priv->id;

	switch (ios->timing) {
	case MMC_TIMING_LEGACY:
		freq = GET_FREQ(id, SDMMC_CCLK_MAX_24M, SDMMC_CCLK_MAX_25M);
		break;
	case MMC_TIMING_MMC_HS:
	case MMC_TIMING_UHS_SDR25:
		freq = GET_FREQ(id, SDMMC_CCLK_MAX_48M, SDMMC_CCLK_MAX_50M);
		break;
	case MMC_TIMING_UHS_DDR50:
		freq = GET_FREQ(id, SDMMC_CCLK_MAX_100M, SDMMC_CCLK_MAX_50M);
		break;
	case MMC_TIMING_UHS_SDR50:
		freq = GET_FREQ(id, SDMMC_CCLK_MAX_96M, SDMMC_CCLK_MAX_100M);
		break;
	default:
		dev_err(host->dev, "ios timing %d not supported\n",
			ios->timing);
		return;
	}

	if (clk_set_rate(host->biu_clk, freq))
		dev_err(host->dev, "failed to set rate %uHz\n", freq);
	else
		host->bus_hz = clk_get_rate(host->biu_clk);
}

static int dw_mci_k3_switch_voltage(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci_priv_data *priv;
	struct dw_mci *host;
	int min_uv, max_uv;
	int ret = 0;

	host = slot->host;
	priv = host->priv;

	if (!priv || !priv->reg)
		return ret;

	if (priv->id != MMC_SD_ID)
		return ret;

	min_uv = 1800000;
	max_uv = 1800000;

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		ret = regmap_update_bits(priv->reg, AO_SCTRL_CTRL3,
					AO_SCTRL_SEL18, 0);
		if (ret) {
			dev_dbg(host->dev, "switch voltage failed\n");
			return ret;
		}
		min_uv = 3000000;
		max_uv = 3000000;
	}

	if (IS_ERR_OR_NULL(mmc->supply.vqmmc))
		return 0;

	ret = regulator_set_voltage(mmc->supply.vqmmc, min_uv, max_uv);
	if (ret) {
		dev_dbg(host->dev, "Regulator set error %d: %d - %d\n",
				 ret, min_uv, max_uv);
		return ret;
	}

	ret = regulator_enable(mmc->supply.vqmmc);
	if (ret) {
		dev_dbg(host->dev, "Regulator enable error %d\n", ret);
		return ret;
	}

	usleep_range(5000, 5500);

	return 0;
}

static int dw_mci_k3_setup_clock(struct dw_mci *host)
{
	int bus_hz = 0;
	int ret;

	host->biu_clk = devm_clk_get(host->dev, "biu");
	if (IS_ERR(host->biu_clk)) {
		dev_dbg(host->dev, "biu clock not available\n");
		goto set_hz;
	}

	clk_set_rate(host->biu_clk, SDMMC_CCLK_MAX_25M);
	ret = clk_prepare_enable(host->biu_clk);
	if (ret) {
		dev_err(host->dev, "failed to enable biu clock\n");
		return ret;
	}
	bus_hz = clk_get_rate(host->biu_clk);

set_hz:

	host->ciu_clk = devm_clk_get(host->dev, "ciu");
	if (IS_ERR(host->ciu_clk)) {
		dev_dbg(host->dev, "ciu clock not available\n");
		host->bus_hz = bus_hz ? bus_hz : host->pdata->bus_hz;
		return 0;
	}

	if (host->pdata->bus_hz) {
		ret = clk_set_rate(host->biu_clk, host->pdata->bus_hz);
		if (ret)
			dev_warn(host->dev, "Unable to set bus rate to %uHz\n",
				 host->pdata->bus_hz);
	}

	host->bus_hz = clk_get_rate(host->biu_clk);

	return 0;
}

static int dw_mci_k3_parse_dt(struct dw_mci *host)
{
	struct device_node *np = host->dev->of_node;

	if (of_find_property(np, "hisilicon,disable-boot", NULL))
		return dw_mci_k3_early_ctrl_reset(host);

	return 0;
}

static int dw_mci_k3_init(struct dw_mci *host)
{
	struct dw_mci_priv_data *priv;

	priv = devm_kzalloc(host->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->reg = syscon_regmap_lookup_by_phandle(host->dev->of_node,
					 "hisilicon,peripheral-syscon");
	if (IS_ERR(priv->reg))
		priv->reg = NULL;

	priv->id = of_alias_get_id(host->dev->of_node, "mshc");
	host->priv = priv;

	return 0;
}

static const struct dw_mci_drv_data k3_drv_data = {
	.switch_voltage		= dw_mci_k3_switch_voltage,
	.setup_clock		= dw_mci_k3_setup_clock,
	.parse_dt		= dw_mci_k3_parse_dt,
	.set_ios		= dw_mci_k3_set_ios,
	.init			= dw_mci_k3_init,
};

static const struct of_device_id dw_mci_k3_match[] = {
	{ .compatible = "hisilicon,hisi-dw-mshc", .data = &k3_drv_data, },
	{},
};
MODULE_DEVICE_TABLE(of, dw_mci_k3_match);

static int dw_mci_k3_probe(struct platform_device *pdev)
{
	const struct dw_mci_drv_data *drv_data;
	const struct of_device_id *match;

	match = of_match_node(dw_mci_k3_match, pdev->dev.of_node);
	if (!match)
		return -EINVAL;

	drv_data = match->data;
	return dw_mci_pltfm_register(pdev, drv_data);
}

#ifdef CONFIG_PM_SLEEP
static int dw_mci_k3_suspend(struct device *dev)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	int ret;

	ret = dw_mci_suspend(host);
	if (!ret)
		clk_disable_unprepare(host->ciu_clk);

	return ret;
}

static int dw_mci_k3_resume(struct device *dev)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(host->ciu_clk);
	if (ret) {
		dev_err(host->dev, "failed to enable ciu clock\n");
		return ret;
	}

	return dw_mci_resume(host);
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(dw_mci_k3_pmops, dw_mci_k3_suspend, dw_mci_k3_resume);

static struct platform_driver dw_mci_k3_pltfm_driver = {
	.probe		= dw_mci_k3_probe,
	.remove		= dw_mci_pltfm_remove,
	.driver		= {
		.name		= "dwmmc_k3",
		.of_match_table	= dw_mci_k3_match,
		.pm		= &dw_mci_k3_pmops,
	},
};

module_platform_driver(dw_mci_k3_pltfm_driver);

MODULE_DESCRIPTION("K3 Specific DW-MSHC Driver Extension");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:dwmmc-k3");
