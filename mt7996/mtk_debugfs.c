// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2023 MediaTek Inc.
 */
#include "mt7996.h"
#include "../mt76.h"
#include "mcu.h"
#include "mac.h"
#include "eeprom.h"
#include "mtk_debug.h"
#include "mtk_mcu.h"
#include "coredump.h"

#ifdef CONFIG_MTK_DEBUG

#ifndef PKG_YEAR
#define PKG_YEAR 0
#endif

#ifndef PKG_MONTH
#define PKG_MONTH 0
#endif

/* AGG INFO */
static int mt7996_agginfo_show(struct seq_file *s, void *data)
{
	struct mt7996_phy *phy = s->private;
	struct mt7996_dev *dev = phy->dev;
	u64 total_burst, total_ampdu, ampdu_cnt[16];
	u32 value, idx, row_idx, col_idx, start_range, agg_rang_sel[16], burst_cnt[16], band_offset = 0;
	u8 partial_str[16] = {}, full_str[64] = {};
	u8 band_idx = phy->mt76->band_idx;

	switch (band_idx) {
	case 0:
		band_offset = 0;
		break;
	case 1:
		band_offset = BN1_WF_AGG_TOP_BASE - BN0_WF_AGG_TOP_BASE;
		break;
	case 2:
		band_offset = IP1_BN0_WF_AGG_TOP_BASE - BN0_WF_AGG_TOP_BASE;
		break;
	default:
		return 0;
	}

	seq_printf(s, "Band %d AGG Status\n", band_idx);
	seq_printf(s, "===============================\n");
	value = mt76_rr(dev, BN0_WF_AGG_TOP_AALCR0_ADDR + band_offset);
	seq_printf(s, "AC00 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR0_AC00_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR0_AC00_AGG_LIMIT_SHFT);
	seq_printf(s, "AC01 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR0_AC01_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR0_AC01_AGG_LIMIT_SHFT);
	value = mt76_rr(dev, BN0_WF_AGG_TOP_AALCR1_ADDR + band_offset);
	seq_printf(s, "AC02 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR1_AC02_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR1_AC02_AGG_LIMIT_SHFT);
	seq_printf(s, "AC03 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR1_AC03_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR1_AC03_AGG_LIMIT_SHFT);
	value = mt76_rr(dev, BN0_WF_AGG_TOP_AALCR2_ADDR + band_offset);
	seq_printf(s, "AC10 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR2_AC10_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR2_AC10_AGG_LIMIT_SHFT);
	seq_printf(s, "AC11 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR2_AC11_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR2_AC11_AGG_LIMIT_SHFT);
	value = mt76_rr(dev, BN0_WF_AGG_TOP_AALCR3_ADDR + band_offset);
	seq_printf(s, "AC12 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR3_AC12_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR3_AC12_AGG_LIMIT_SHFT);
	seq_printf(s, "AC13 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR3_AC13_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR3_AC13_AGG_LIMIT_SHFT);
	value = mt76_rr(dev, BN0_WF_AGG_TOP_AALCR4_ADDR + band_offset);
	seq_printf(s, "AC20 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR4_AC20_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR4_AC20_AGG_LIMIT_SHFT);
	seq_printf(s, "AC21 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR4_AC21_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR4_AC21_AGG_LIMIT_SHFT);
	value = mt76_rr(dev, BN0_WF_AGG_TOP_AALCR5_ADDR + band_offset);
	seq_printf(s, "AC22 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR5_AC22_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR5_AC22_AGG_LIMIT_SHFT);
	seq_printf(s, "AC23 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR5_AC23_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR5_AC23_AGG_LIMIT_SHFT);
	value = mt76_rr(dev, BN0_WF_AGG_TOP_AALCR6_ADDR + band_offset);
	seq_printf(s, "AC30 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR6_AC30_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR6_AC30_AGG_LIMIT_SHFT);
	seq_printf(s, "AC31 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR6_AC31_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR6_AC31_AGG_LIMIT_SHFT);
	value = mt76_rr(dev, BN0_WF_AGG_TOP_AALCR7_ADDR + band_offset);
	seq_printf(s, "AC32 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR7_AC32_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR7_AC32_AGG_LIMIT_SHFT);
	seq_printf(s, "AC33 Agg limit = %d\t", (value & BN0_WF_AGG_TOP_AALCR7_AC33_AGG_LIMIT_MASK) >>  BN0_WF_AGG_TOP_AALCR7_AC33_AGG_LIMIT_SHFT);

	switch (band_idx) {
	case 0:
		band_offset = 0;
		break;
	case 1:
		band_offset = BN1_WF_MIB_TOP_BASE - BN0_WF_MIB_TOP_BASE;
		break;
	case 2:
		band_offset = IP1_BN0_WF_MIB_TOP_BASE - BN0_WF_MIB_TOP_BASE;
		break;
	default:
		return 0;
	}

	seq_printf(s, "===AMPDU Related Counters===\n");

	value = mt76_rr(dev, BN0_WF_MIB_TOP_TRARC0_ADDR + band_offset);
	agg_rang_sel[0] = (value & BN0_WF_MIB_TOP_TRARC0_AGG_RANG_SEL_0_MASK) >> BN0_WF_MIB_TOP_TRARC0_AGG_RANG_SEL_0_SHFT;
	agg_rang_sel[1] = (value & BN0_WF_MIB_TOP_TRARC0_AGG_RANG_SEL_1_MASK) >> BN0_WF_MIB_TOP_TRARC0_AGG_RANG_SEL_1_SHFT;
	value = mt76_rr(dev, BN0_WF_MIB_TOP_TRARC1_ADDR + band_offset);
	agg_rang_sel[2] = (value & BN0_WF_MIB_TOP_TRARC1_AGG_RANG_SEL_2_MASK) >> BN0_WF_MIB_TOP_TRARC1_AGG_RANG_SEL_2_SHFT;
	agg_rang_sel[3] = (value & BN0_WF_MIB_TOP_TRARC1_AGG_RANG_SEL_3_MASK) >> BN0_WF_MIB_TOP_TRARC1_AGG_RANG_SEL_3_SHFT;
	value = mt76_rr(dev, BN0_WF_MIB_TOP_TRARC2_ADDR + band_offset);
	agg_rang_sel[4] = (value & BN0_WF_MIB_TOP_TRARC2_AGG_RANG_SEL_4_MASK) >> BN0_WF_MIB_TOP_TRARC2_AGG_RANG_SEL_4_SHFT;
	agg_rang_sel[5] = (value & BN0_WF_MIB_TOP_TRARC2_AGG_RANG_SEL_5_MASK) >> BN0_WF_MIB_TOP_TRARC2_AGG_RANG_SEL_5_SHFT;
	value = mt76_rr(dev, BN0_WF_MIB_TOP_TRARC3_ADDR + band_offset);
	agg_rang_sel[6] = (value & BN0_WF_MIB_TOP_TRARC3_AGG_RANG_SEL_6_MASK) >> BN0_WF_MIB_TOP_TRARC3_AGG_RANG_SEL_6_SHFT;
	agg_rang_sel[7] = (value & BN0_WF_MIB_TOP_TRARC3_AGG_RANG_SEL_7_MASK) >> BN0_WF_MIB_TOP_TRARC3_AGG_RANG_SEL_7_SHFT;
	value = mt76_rr(dev, BN0_WF_MIB_TOP_TRARC4_ADDR + band_offset);
	agg_rang_sel[8] = (value & BN0_WF_MIB_TOP_TRARC4_AGG_RANG_SEL_8_MASK) >> BN0_WF_MIB_TOP_TRARC4_AGG_RANG_SEL_8_SHFT;
	agg_rang_sel[9] = (value & BN0_WF_MIB_TOP_TRARC4_AGG_RANG_SEL_9_MASK) >> BN0_WF_MIB_TOP_TRARC4_AGG_RANG_SEL_9_SHFT;
	value = mt76_rr(dev, BN0_WF_MIB_TOP_TRARC5_ADDR + band_offset);
	agg_rang_sel[10] = (value & BN0_WF_MIB_TOP_TRARC5_AGG_RANG_SEL_10_MASK) >> BN0_WF_MIB_TOP_TRARC5_AGG_RANG_SEL_10_SHFT;
	agg_rang_sel[11] = (value & BN0_WF_MIB_TOP_TRARC5_AGG_RANG_SEL_11_MASK) >> BN0_WF_MIB_TOP_TRARC5_AGG_RANG_SEL_11_SHFT;
	value = mt76_rr(dev, BN0_WF_MIB_TOP_TRARC6_ADDR + band_offset);
	agg_rang_sel[12] = (value & BN0_WF_MIB_TOP_TRARC6_AGG_RANG_SEL_12_MASK) >> BN0_WF_MIB_TOP_TRARC6_AGG_RANG_SEL_12_SHFT;
	agg_rang_sel[13] = (value & BN0_WF_MIB_TOP_TRARC6_AGG_RANG_SEL_13_MASK) >> BN0_WF_MIB_TOP_TRARC6_AGG_RANG_SEL_13_SHFT;
	value = mt76_rr(dev, BN0_WF_MIB_TOP_TRARC7_ADDR + band_offset);
	agg_rang_sel[14] = (value & BN0_WF_MIB_TOP_TRARC7_AGG_RANG_SEL_14_MASK) >> BN0_WF_MIB_TOP_TRARC7_AGG_RANG_SEL_14_SHFT;

	burst_cnt[0] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR0_ADDR + band_offset);
	burst_cnt[1] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR1_ADDR + band_offset);
	burst_cnt[2] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR2_ADDR + band_offset);
	burst_cnt[3] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR3_ADDR + band_offset);
	burst_cnt[4] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR4_ADDR + band_offset);
	burst_cnt[5] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR5_ADDR + band_offset);
	burst_cnt[6] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR6_ADDR + band_offset);
	burst_cnt[7] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR7_ADDR + band_offset);
	burst_cnt[8] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR8_ADDR + band_offset);
	burst_cnt[9] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR9_ADDR + band_offset);
	burst_cnt[10] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR10_ADDR + band_offset);
	burst_cnt[11] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR11_ADDR + band_offset);
	burst_cnt[12] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR12_ADDR + band_offset);
	burst_cnt[13] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR13_ADDR + band_offset);
	burst_cnt[14] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR14_ADDR + band_offset);
	burst_cnt[15] = mt76_rr(dev, BN0_WF_MIB_TOP_TRDR15_ADDR + band_offset);

	start_range = 1;
	total_burst = 0;
	total_ampdu = 0;
	agg_rang_sel[15] = 1023;

	/* Need to add 1 after read from AGG_RANG_SEL CR */
	for (idx = 0; idx < 16; idx++) {
		agg_rang_sel[idx]++;
		total_burst += burst_cnt[idx];

		if (start_range == agg_rang_sel[idx])
			ampdu_cnt[idx] = (u64) start_range * burst_cnt[idx];
		else
			ampdu_cnt[idx] = (u64) ((start_range + agg_rang_sel[idx]) >> 1) * burst_cnt[idx];

		start_range = agg_rang_sel[idx] + 1;
		total_ampdu += ampdu_cnt[idx];
	}

	start_range = 1;
	sprintf(full_str, "%13s ", "Tx Agg Range:");

	for (row_idx = 0; row_idx < 4; row_idx++) {
		for (col_idx = 0; col_idx < 4; col_idx++, idx++) {
			idx = 4 * row_idx + col_idx;

			if (start_range == agg_rang_sel[idx])
				sprintf(partial_str, "%d", agg_rang_sel[idx]);
			else
				sprintf(partial_str, "%d~%d", start_range, agg_rang_sel[idx]);

			start_range = agg_rang_sel[idx] + 1;
			sprintf(full_str + strlen(full_str), "%-11s ", partial_str);
		}

		idx = 4 * row_idx;

		seq_printf(s, "%s\n", full_str);
		seq_printf(s, "%13s 0x%-9x 0x%-9x 0x%-9x 0x%-9x\n",
			row_idx ? "" : "Burst count:",
			burst_cnt[idx], burst_cnt[idx + 1],
			burst_cnt[idx + 2], burst_cnt[idx + 3]);

		if (total_burst != 0) {
			if (row_idx == 0)
				sprintf(full_str, "%13s ",
					"Burst ratio:");
			else
				sprintf(full_str, "%13s ", "");

			for (col_idx = 0; col_idx < 4; col_idx++) {
				u64 count = (u64) burst_cnt[idx + col_idx] * 100;

				sprintf(partial_str, "(%llu%%)",
					div64_u64(count, total_burst));
				sprintf(full_str + strlen(full_str),
					"%-11s ", partial_str);
			}

			seq_printf(s, "%s\n", full_str);

			if (row_idx == 0)
				sprintf(full_str, "%13s ",
					"MDPU ratio:");
			else
				sprintf(full_str, "%13s ", "");

			for (col_idx = 0; col_idx < 4; col_idx++) {
				u64 count = ampdu_cnt[idx + col_idx] * 100;

				sprintf(partial_str, "(%llu%%)",
					div64_u64(count, total_ampdu));
				sprintf(full_str + strlen(full_str),
					"%-11s ", partial_str);
			}

			seq_printf(s, "%s\n", full_str);
		}

		sprintf(full_str, "%13s ", "");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt7996_agginfo);

/* AMSDU INFO */
static int mt7996_amsdu_result_read(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	struct mt7996_phy *phy = &dev->phy;
	struct mt76_mib_stats *mib = &phy->mib;
	static u32 tx_amsdu_last[8] = {0};
	static u32 tx_amsdu_cnt_last = 0;
	u32 tx_amsdu, tx_amsdu_cnt, ratio;
	int i;

	mutex_lock(&dev->mt76.mutex);

	mt7996_mac_update_stats(phy);

	tx_amsdu_cnt = mib->tx_amsdu_cnt - tx_amsdu_cnt_last;

	seq_puts(s, "Tx MSDU statistics:\n");
	for (i = 0; i < ARRAY_SIZE(mib->tx_amsdu); i++) {
		tx_amsdu = mib->tx_amsdu[i] - tx_amsdu_last[i];
		ratio = tx_amsdu_cnt ? tx_amsdu * 100 / tx_amsdu_cnt : 0;

		seq_printf(s, "AMSDU pack count of %d MSDU in TXD: %8d (%3d%%)\n",
			   i + 1, tx_amsdu, ratio);

		tx_amsdu_last[i] = mib->tx_amsdu[i];
	}

	tx_amsdu_cnt_last = mib->tx_amsdu_cnt;

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

/* DBG MODLE */
static int
mt7996_fw_debug_module_set(void *data, u64 module)
{
	struct mt7996_dev *dev = data;

	dev->dbg.fw_dbg_module = module;
	return 0;
}

static int
mt7996_fw_debug_module_get(void *data, u64 *module)
{
	struct mt7996_dev *dev = data;

	*module = dev->dbg.fw_dbg_module;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_fw_debug_module, mt7996_fw_debug_module_get,
			 mt7996_fw_debug_module_set, "%lld\n");

static int
mt7996_fw_debug_level_set(void *data, u64 level)
{
	struct mt7996_dev *dev = data;

	dev->dbg.fw_dbg_lv = level;
	mt7996_mcu_fw_dbg_ctrl(dev, dev->dbg.fw_dbg_module, dev->dbg.fw_dbg_lv);
	return 0;
}

static int
mt7996_fw_debug_level_get(void *data, u64 *level)
{
	struct mt7996_dev *dev = data;

	*level = dev->dbg.fw_dbg_lv;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_fw_debug_level, mt7996_fw_debug_level_get,
			 mt7996_fw_debug_level_set, "%lld\n");

/* usage: echo 0x[arg3][arg2][arg1] > fw_wa_set */
static int
mt7996_wa_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;
	u32 arg1, arg2, arg3;

	arg1 = FIELD_GET(GENMASK_ULL(7, 0), val);
	arg2 = FIELD_GET(GENMASK_ULL(15, 8), val);
	arg3 = FIELD_GET(GENMASK_ULL(23, 16), val);

	return mt7996_mcu_wa_cmd(dev, MCU_WA_PARAM_CMD(SET),
				arg1, arg2, arg3);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_wa_set, NULL, mt7996_wa_set,
			 "0x%llx\n");

/* usage: echo 0x[arg3][arg2][arg1] > fw_wa_query */
static int
mt7996_wa_query(void *data, u64 val)
{
	struct mt7996_dev *dev = data;
	u32 arg1, arg2, arg3;

	arg1 = FIELD_GET(GENMASK_ULL(7, 0), val);
	arg2 = FIELD_GET(GENMASK_ULL(15, 8), val);
	arg3 = FIELD_GET(GENMASK_ULL(23, 16), val);

	return mt7996_mcu_wa_cmd(dev, MCU_WA_PARAM_CMD(QUERY),
				arg1, arg2, arg3);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_wa_query, NULL, mt7996_wa_query,
			 "0x%llx\n");

static int mt7996_dump_version(struct seq_file *s, void *data)
{
#define MAX_ADIE_NUM	3
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	u32 regval;
	u16 adie_chip_id, adie_chip_ver;
	int adie_idx;
	static const char * const fem_type[] = {
		[MT7996_FEM_EXT] = "eFEM",
		[MT7996_FEM_INT] = "iFEM",
		[MT7996_FEM_MIX] = "mixed FEM",
	};

	seq_printf(s, "Version: 4.4.%02d.%02d\n", PKG_YEAR, PKG_MONTH);

	if (!test_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state))
		return 0;

	seq_printf(s, "Rom Patch Build Time: %.16s\n", dev->patch_build_date);
	seq_printf(s, "WM Patch Build Time: %.15s, Mode: %s\n",
		   dev->ram_build_date[MT7996_RAM_TYPE_WM],
		   dev->testmode_enable ? "Testmode" : "Normal mode");
	seq_printf(s, "WA Patch Build Time: %.15s\n",
		   dev->ram_build_date[MT7996_RAM_TYPE_WA]);
	seq_printf(s, "DSP Patch Build Time: %.15s\n",
		   dev->ram_build_date[MT7996_RAM_TYPE_DSP]);
	for (adie_idx = 0; adie_idx < MAX_ADIE_NUM; adie_idx++) {
		mt7996_mcu_rf_regval(dev, MT_ADIE_CHIP_ID(adie_idx), &regval, false);
		adie_chip_id = FIELD_GET(MT_ADIE_CHIP_ID_MASK, regval);
		adie_chip_ver = FIELD_GET(MT_ADIE_VERSION_MASK, regval);
		if (adie_chip_id)
			seq_printf(s, "Adie %d: ID = 0x%04x, Ver = 0x%04x\n",
				   adie_idx, adie_chip_id, adie_chip_ver);
		else
			seq_printf(s, "Adie %d: ID = N/A, Ver = N/A\n", adie_idx);
	}
	seq_printf(s, "FEM type: %s\n", fem_type[dev->var.fem]);

	return 0;
}

/* fw wm call trace info dump */
void mt7996_show_lp_history(struct seq_file *s, u32 type)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	struct mt7996_crash_data *crash_data;
	struct mt7996_coredump *dump;
	u64 now = 0;
	int i = 0;
	u8 fw_type = !!type;

	mutex_lock(&dev->dump_mutex);

	crash_data = mt7996_coredump_new(dev, fw_type);
	if (!crash_data) {
		mutex_unlock(&dev->dump_mutex);
		seq_printf(s, "the coredump is disable!\n");
		return;
	}
	mutex_unlock(&dev->dump_mutex);

	dump = mt7996_coredump_build(dev, fw_type, false);
	if (!dump) {
		seq_printf(s, "no call stack data found!\n");
		return;
	}

	seq_printf(s, "\x1b[32m%s log output\x1b[0m\n", dump->fw_type);
	seq_printf(s, "\x1b[32mfw status: %s\n", dump->fw_state);
	/* PC log */
	now = jiffies;
	for (i = 0; i < 10; i++)
		seq_printf(s, "\tCurrent PC=%x\n", dump->pc_cur[i]);

	seq_printf(s, "PC log contorl=0x%x(T=%llu)(latest PC index = 0x%x)\n",
		dump->pc_dbg_ctrl, now, dump->pc_cur_idx);
	for (i = 0; i < 32; i++)
		seq_printf(s, "\tPC log(%d)=0x%08x\n", i, dump->pc_stack[i]);

	/* LR log */
	now = jiffies;
	seq_printf(s, "\nLR log contorl=0x%x(T=%llu)(latest LR index = 0x%x)\n",
		dump->lr_dbg_ctrl, now, dump->lr_cur_idx);
	for (i = 0; i < 32; i++)
		seq_printf(s, "\tLR log(%d)=0x%08x\n", i, dump->lr_stack[i]);

	vfree(dump);
}

static int mt7996_fw_wa_info_read(struct seq_file *s, void *data)
{
	seq_printf(s, "======[ShowPcLpHistory]======\n");
	mt7996_show_lp_history(s, MT7996_RAM_TYPE_WA);
	seq_printf(s, "======[End ShowPcLpHistory]==\n");

	return 0;
}

static int mt7996_fw_wm_info_read(struct seq_file *s, void *data)
{
	seq_printf(s, "======[ShowPcLpHistory]======\n");
	mt7996_show_lp_history(s, MT7996_RAM_TYPE_WM);
	seq_printf(s, "======[End ShowPcLpHistory]==\n");

	return 0;
}

/* dma info dump */
static void
dump_dma_tx_ring_info(struct seq_file *s, struct mt7996_dev *dev,  char *str1, char *str2, u32 ring_base)
{
	u32 base, cnt, cidx, didx, queue_cnt;

	base= mt76_rr(dev, ring_base);
	cnt = mt76_rr(dev, ring_base + 4);
	cidx = mt76_rr(dev, ring_base + 8);
	didx = mt76_rr(dev, ring_base + 12);
	queue_cnt = (cidx >= didx) ? (cidx - didx) : (cidx - didx + cnt);

	seq_printf(s, "%20s %6s %10x %15x %10x %10x %10x\n", str1, str2, base, cnt, cidx, didx, queue_cnt);
}

static void
dump_dma_rx_ring_info(struct seq_file *s, struct mt7996_dev *dev,  char *str1, char *str2, u32 ring_base)
{
	u32 base, ctrl1, cnt, cidx, didx, queue_cnt;

	base= mt76_rr(dev, ring_base);
	ctrl1 = mt76_rr(dev, ring_base + 4);
	cidx = mt76_rr(dev, ring_base + 8) & 0xfff;
	didx = mt76_rr(dev, ring_base + 12) & 0xfff;
	cnt = ctrl1 & 0xfff;
	queue_cnt = (didx > cidx) ? (didx - cidx - 1) : (didx - cidx + cnt - 1);

	seq_printf(s, "%20s %6s %10x %10x(%3x) %10x %10x %10x\n",
		   str1, str2, base, ctrl1, cnt, cidx, didx, queue_cnt);
}

static void
mt7996_show_dma_info(struct seq_file *s, struct mt7996_dev *dev)
{
	u32 sys_ctrl[10];

	/* HOST DMA0 information */
	sys_ctrl[0] = mt76_rr(dev, WF_WFDMA_HOST_DMA0_HOST_INT_STA_ADDR);
	sys_ctrl[1] = mt76_rr(dev, WF_WFDMA_HOST_DMA0_HOST_INT_ENA_ADDR);
	sys_ctrl[2] = mt76_rr(dev, WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_ADDR);

	seq_printf(s, "HOST_DMA Configuration\n");
	seq_printf(s, "%10s %10s %10s %10s %10s %10s\n",
		"DMA", "IntCSR", "IntMask", "Glocfg", "Tx/RxEn", "Tx/RxBusy");
	seq_printf(s, "%10s %10x %10x %10x %4x/%5x %4x/%5x\n",
		"DMA0", sys_ctrl[0], sys_ctrl[1], sys_ctrl[2],
		(sys_ctrl[2] & WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_TX_DMA_EN_MASK)
			>> WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_TX_DMA_EN_SHFT,
		(sys_ctrl[2] & WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_RX_DMA_EN_MASK)
			>> WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_RX_DMA_EN_SHFT,
		(sys_ctrl[2] & WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_TX_DMA_BUSY_MASK)
			>> WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_TX_DMA_BUSY_SHFT,
		(sys_ctrl[2] & WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_RX_DMA_BUSY_MASK)
			>> WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_RX_DMA_BUSY_SHFT);

	if (dev->hif2) {
		/* HOST DMA1 information */
		sys_ctrl[0] = mt76_rr(dev, WF_WFDMA_HOST_DMA0_PCIE1_HOST_INT_STA_ADDR);
		sys_ctrl[1] = mt76_rr(dev, WF_WFDMA_HOST_DMA0_PCIE1_HOST_INT_ENA_ADDR);
		sys_ctrl[2] = mt76_rr(dev, WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_GLO_CFG_ADDR);

		seq_printf(s, "%10s %10x %10x %10x %4x/%5x %4x/%5x\n",
			"DMA0P1", sys_ctrl[0], sys_ctrl[1], sys_ctrl[2],
			(sys_ctrl[2] & WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_GLO_CFG_TX_DMA_EN_MASK)
				>> WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_GLO_CFG_TX_DMA_EN_SHFT,
			(sys_ctrl[2] & WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_GLO_CFG_RX_DMA_EN_MASK)
				>> WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_GLO_CFG_RX_DMA_EN_SHFT,
			(sys_ctrl[2] & WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_GLO_CFG_TX_DMA_BUSY_MASK)
				>> WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_GLO_CFG_TX_DMA_BUSY_SHFT,
			(sys_ctrl[2] & WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_GLO_CFG_RX_DMA_BUSY_MASK)
				>> WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_GLO_CFG_RX_DMA_BUSY_SHFT);
	}

	seq_printf(s, "HOST_DMA0 Ring Configuration\n");
	seq_printf(s, "%20s %6s %10s %15s %10s %10s %10s\n",
		"Name", "Used", "Base", "Ctrl1(Cnt)", "CIDX", "DIDX", "QCnt");
	dump_dma_tx_ring_info(s, dev, "T0:TXD0(H2MAC)", "STA",
		WF_WFDMA_HOST_DMA0_WPDMA_TX_RING0_CTRL0_ADDR);
	dump_dma_tx_ring_info(s, dev, "T1:TXD1(H2MAC)", "STA",
		WF_WFDMA_HOST_DMA0_WPDMA_TX_RING1_CTRL0_ADDR);
	dump_dma_tx_ring_info(s, dev, "T2:TXD2(H2MAC)", "STA",
		WF_WFDMA_HOST_DMA0_WPDMA_TX_RING2_CTRL0_ADDR);
	dump_dma_tx_ring_info(s, dev, "T3:", "STA",
		WF_WFDMA_HOST_DMA0_WPDMA_TX_RING3_CTRL0_ADDR);
	dump_dma_tx_ring_info(s, dev, "T4:", "STA",
		WF_WFDMA_HOST_DMA0_WPDMA_TX_RING4_CTRL0_ADDR);
	dump_dma_tx_ring_info(s, dev, "T5:", "STA",
		WF_WFDMA_HOST_DMA0_WPDMA_TX_RING5_CTRL0_ADDR);
	dump_dma_tx_ring_info(s, dev, "T6:", "STA",
		WF_WFDMA_HOST_DMA0_WPDMA_TX_RING6_CTRL0_ADDR);
	dump_dma_tx_ring_info(s, dev, "T16:FWDL", "Both",
		WF_WFDMA_HOST_DMA0_WPDMA_TX_RING16_CTRL0_ADDR);
	dump_dma_tx_ring_info(s, dev, "T17:Cmd(H2WM)", "Both",
		WF_WFDMA_HOST_DMA0_WPDMA_TX_RING17_CTRL0_ADDR);
	if (mt7996_has_wa(dev)) {
		dump_dma_tx_ring_info(s, dev, "T18:TXD0(H2WA)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_TX_RING18_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T19:TXD1(H2WA)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_TX_RING19_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T20:Cmd(H2WA)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_TX_RING20_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T21:TXD2(H2WA)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_TX_RING21_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T22:TXD3(H2WA)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_TX_RING22_CTRL0_ADDR);
	} else {
		dump_dma_tx_ring_info(s, dev, "T18:TXD0(H2SDO)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_TX_RING18_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T19:TXD1(H2SDO)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_TX_RING19_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T20:Reserved", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_TX_RING20_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T21:TXD2(H2SDO)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_TX_RING21_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T22:TXD3(H2SDO)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_TX_RING22_CTRL0_ADDR);
	}


	dump_dma_rx_ring_info(s, dev, "R0:Event(WM2H)", "Both",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_CTRL0_ADDR);
	if (mt7996_has_wa(dev)) {
		dump_dma_rx_ring_info(s, dev, "R1:Event(WA2H)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING1_CTRL0_ADDR);
		dump_dma_rx_ring_info(s, dev, "R2:TxDone0(WA2H)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING2_CTRL0_ADDR);
		dump_dma_rx_ring_info(s, dev, "R3:TxDone1(WA2H)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING3_CTRL0_ADDR);
	} else {
		dump_dma_rx_ring_info(s, dev, "R1:Event(SDO2H)", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING1_CTRL0_ADDR);
		dump_dma_rx_ring_info(s, dev, "R2:Reserved", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING2_CTRL0_ADDR);
		dump_dma_rx_ring_info(s, dev, "R3:Reserved", "AP",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING3_CTRL0_ADDR);
	}
	dump_dma_rx_ring_info(s, dev, "R4:Data0(MAC2H)", "Both",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING4_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R5:Data1(MAC2H)", "Both",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING5_CTRL0_ADDR);
	if (is_mt7996(&dev->mt76))
		dump_dma_rx_ring_info(s, dev, "R6:BUF1(MAC2H)", "Both",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING6_CTRL0_ADDR);
	else
		dump_dma_rx_ring_info(s, dev, "R6:TxDone0(MAC2H)", "Both",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING6_CTRL0_ADDR);
	if (is_mt7990(&dev->mt76))
		dump_dma_rx_ring_info(s, dev, "R7:Reserved)", "Both",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING7_CTRL0_ADDR);
	else
		dump_dma_rx_ring_info(s, dev, "R7:TxDone1(MAC2H)", "Both",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING7_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R8:BUF0(MAC2H)", "Both",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING8_CTRL0_ADDR);
	if (is_mt7996(&dev->mt76))
		dump_dma_rx_ring_info(s, dev, "R9:TxDone0(MAC2H)", "Both",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING9_CTRL0_ADDR);
	else
		dump_dma_rx_ring_info(s, dev, "R9:BUF0(MAC2H)", "Both",
			WF_WFDMA_HOST_DMA0_WPDMA_RX_RING9_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R10:MSDU_PG0(MAC2H)", "Both",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING10_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R11:MSDU_PG1(MAC2H)", "Both",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING11_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R12:MSDU_PG2(MAC2H)", "Both",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING12_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "IND:IND_CMD(MAC2H)", "Both",
		WF_RRO_TOP_IND_CMD_0_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "RRO:Data0(MAC2H)", "Both",
		WF_RRO_TOP_RX_RING_AP_0_CTRL0_ADDR);

	if (dev->hif2) {
		seq_printf(s, "HOST_DMA0 PCIe1 Ring Configuration\n");
		seq_printf(s, "%20s %6s %10s %15s %10s %10s %10s\n",
			"Name", "Used", "Base", "Ctrl1(Cnt)", "CIDX", "DIDX", "QCnt");
		if (mt7996_has_wa(dev)) {
			dump_dma_tx_ring_info(s, dev, "T21:TXD2(H2WA)", "AP",
				WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_TX_RING21_CTRL0_ADDR);
			dump_dma_tx_ring_info(s, dev, "T22:TXD?(H2WA)", "AP",
				WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_TX_RING22_CTRL0_ADDR);
			dump_dma_rx_ring_info(s, dev, "R3:TxDone1(WA2H)", "AP",
				WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_RX_RING3_CTRL0_ADDR);
		} else {
			dump_dma_tx_ring_info(s, dev, "T21:TXD2(H2SDO)", "AP",
				WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_TX_RING21_CTRL0_ADDR);
			dump_dma_tx_ring_info(s, dev, "T22:TXD?(H2SDO)", "AP",
				WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_TX_RING22_CTRL0_ADDR);
			dump_dma_rx_ring_info(s, dev, "R3:Reserved", "AP",
				WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_RX_RING3_CTRL0_ADDR);
		}
		dump_dma_rx_ring_info(s, dev, "R5:Data1(MAC2H)", "Both",
			WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_RX_RING5_CTRL0_ADDR);
		if (is_mt7996(&dev->mt76))
			dump_dma_rx_ring_info(s, dev, "R6:BUF1(MAC2H)", "Both",
				WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_RX_RING6_CTRL0_ADDR);
		dump_dma_rx_ring_info(s, dev, "R7:TxDone1(MAC2H)", "Both",
			WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_RX_RING7_CTRL0_ADDR);
		if (is_mt7992(&dev->mt76) || is_mt7990(&dev->mt76))
			dump_dma_rx_ring_info(s, dev, "R9:BUF1(MAC2H)", "Both",
				WF_WFDMA_HOST_DMA0_PCIE1_WPDMA_RX_RING9_CTRL0_ADDR);
	}

	/* MCU DMA information */
	sys_ctrl[0] = mt76_rr(dev, WF_WFDMA_MCU_DMA0_WPDMA_GLO_CFG_ADDR);
	sys_ctrl[1] = mt76_rr(dev, WF_WFDMA_MCU_DMA0_HOST_INT_STA_ADDR);
	sys_ctrl[2] = mt76_rr(dev, WF_WFDMA_MCU_DMA0_HOST_INT_ENA_ADDR);

	seq_printf(s, "MCU_DMA Configuration\n");
	seq_printf(s, "%10s %10s %10s %10s %10s %10s\n",
		"DMA", "IntCSR", "IntMask", "Glocfg", "Tx/RxEn", "Tx/RxBusy");
	seq_printf(s, "%10s %10x %10x %10x %4x/%5x %4x/%5x\n",
		"DMA0", sys_ctrl[1], sys_ctrl[2], sys_ctrl[0],
		(sys_ctrl[0] & WF_WFDMA_MCU_DMA0_WPDMA_GLO_CFG_TX_DMA_EN_MASK)
			>> WF_WFDMA_MCU_DMA0_WPDMA_GLO_CFG_TX_DMA_EN_SHFT,
		(sys_ctrl[0] & WF_WFDMA_MCU_DMA0_WPDMA_GLO_CFG_RX_DMA_EN_MASK)
			>> WF_WFDMA_MCU_DMA0_WPDMA_GLO_CFG_RX_DMA_EN_SHFT,
		(sys_ctrl[0] & WF_WFDMA_MCU_DMA0_WPDMA_GLO_CFG_TX_DMA_BUSY_MASK)
			>> WF_WFDMA_MCU_DMA0_WPDMA_GLO_CFG_TX_DMA_BUSY_SHFT,
		(sys_ctrl[0] & WF_WFDMA_MCU_DMA0_WPDMA_GLO_CFG_RX_DMA_BUSY_MASK)
			>> WF_WFDMA_MCU_DMA0_WPDMA_GLO_CFG_RX_DMA_BUSY_SHFT);

	seq_printf(s, "MCU_DMA0 Ring Configuration\n");
	seq_printf(s, "%20s %6s %10s %15s %10s %10s %10s\n",
		"Name", "Used", "Base", "Cnt", "CIDX", "DIDX", "QCnt");
	dump_dma_tx_ring_info(s, dev, "T0:Event(WM2H)", "Both",
		WF_WFDMA_MCU_DMA0_WPDMA_TX_RING0_CTRL0_ADDR);
	if (mt7996_has_wa(dev)) {
		dump_dma_tx_ring_info(s, dev, "T1:Event(WA2H)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_TX_RING1_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T2:TxDone0(WA2H)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_TX_RING2_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T3:TxDone1(WA2H)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_TX_RING3_CTRL0_ADDR);
	} else {
		dump_dma_tx_ring_info(s, dev, "T1:Event(SDO2H)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_TX_RING1_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T2:Reserved", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_TX_RING2_CTRL0_ADDR);
		dump_dma_tx_ring_info(s, dev, "T3:Reserved", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_TX_RING3_CTRL0_ADDR);
	}
	dump_dma_tx_ring_info(s, dev, "T4:TXD(WM2MAC)", "Both",
		WF_WFDMA_MCU_DMA0_WPDMA_TX_RING4_CTRL0_ADDR);
	dump_dma_tx_ring_info(s, dev, "T5:TXCMD(WM2MAC)", "Both",
		WF_WFDMA_MCU_DMA0_WPDMA_TX_RING5_CTRL0_ADDR);
	if (mt7996_has_wa(dev))
		dump_dma_tx_ring_info(s, dev, "T6:TXD(WA2MAC)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_TX_RING6_CTRL0_ADDR);
	else
		dump_dma_tx_ring_info(s, dev, "T6:TXD(SDO2MAC)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_TX_RING6_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R0:FWDL", "Both",
		WF_WFDMA_MCU_DMA0_WPDMA_RX_RING0_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R1:Cmd(H2WM)", "Both",
		WF_WFDMA_MCU_DMA0_WPDMA_RX_RING1_CTRL0_ADDR);
	if (mt7996_has_wa(dev)) {
		dump_dma_rx_ring_info(s, dev, "R2:TXD0(H2WA)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_RX_RING2_CTRL0_ADDR);
		dump_dma_rx_ring_info(s, dev, "R3:TXD1(H2WA)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_RX_RING3_CTRL0_ADDR);
		dump_dma_rx_ring_info(s, dev, "R4:Cmd(H2WA)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_RX_RING4_CTRL0_ADDR);
	} else {
		dump_dma_rx_ring_info(s, dev, "R2:TXD0(H2SDO)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_RX_RING2_CTRL0_ADDR);
		dump_dma_rx_ring_info(s, dev, "R3:TXD1(H2SDO)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_RX_RING3_CTRL0_ADDR);
		dump_dma_rx_ring_info(s, dev, "R4:Reserved", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_RX_RING4_CTRL0_ADDR);
	}
	dump_dma_rx_ring_info(s, dev, "R5:Data0(MAC2WM)", "Both",
		WF_WFDMA_MCU_DMA0_WPDMA_RX_RING5_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R6:TxDone(MAC2WM)", "Both",
		WF_WFDMA_MCU_DMA0_WPDMA_RX_RING6_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R7:SPL/RPT(MAC2WM)", "Both",
		WF_WFDMA_MCU_DMA0_WPDMA_RX_RING7_CTRL0_ADDR);
	if (mt7996_has_wa(dev))
		dump_dma_rx_ring_info(s, dev, "R8:TxDone(MAC2WA)", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_RX_RING8_CTRL0_ADDR);
	else
		dump_dma_rx_ring_info(s, dev, "R8:Reserved", "AP",
			WF_WFDMA_MCU_DMA0_WPDMA_RX_RING8_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R9:Data1(MAC2WM)", "Both",
		WF_WFDMA_MCU_DMA0_WPDMA_RX_RING9_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R10:TXD2(H2WA)", "AP",
		WF_WFDMA_MCU_DMA0_WPDMA_RX_RING10_CTRL0_ADDR);

	/* MEM DMA information */
	sys_ctrl[0] = mt76_rr(dev, WF_WFDMA_MEM_DMA_WPDMA_GLO_CFG_ADDR);
	sys_ctrl[1] = mt76_rr(dev, WF_WFDMA_MEM_DMA_HOST_INT_STA_ADDR);
	sys_ctrl[2] = mt76_rr(dev, WF_WFDMA_MEM_DMA_HOST_INT_ENA_ADDR);

	seq_printf(s, "MEM_DMA Configuration\n");
	seq_printf(s, "%10s %10s %10s %10s %10s %10s\n",
		"DMA", "IntCSR", "IntMask", "Glocfg", "Tx/RxEn", "Tx/RxBusy");
	seq_printf(s, "%10s %10x %10x %10x %4x/%5x %4x/%5x\n",
		"MEM", sys_ctrl[1], sys_ctrl[2], sys_ctrl[0],
		(sys_ctrl[0] & WF_WFDMA_MEM_DMA_WPDMA_GLO_CFG_TX_DMA_EN_MASK)
			>> WF_WFDMA_MEM_DMA_WPDMA_GLO_CFG_TX_DMA_EN_SHFT,
		(sys_ctrl[0] & WF_WFDMA_MEM_DMA_WPDMA_GLO_CFG_RX_DMA_EN_MASK)
			>> WF_WFDMA_MEM_DMA_WPDMA_GLO_CFG_RX_DMA_EN_SHFT,
		(sys_ctrl[0] & WF_WFDMA_MEM_DMA_WPDMA_GLO_CFG_TX_DMA_BUSY_MASK)
			>> WF_WFDMA_MEM_DMA_WPDMA_GLO_CFG_TX_DMA_BUSY_SHFT,
		(sys_ctrl[0] & WF_WFDMA_MEM_DMA_WPDMA_GLO_CFG_RX_DMA_BUSY_MASK)
			>> WF_WFDMA_MEM_DMA_WPDMA_GLO_CFG_RX_DMA_BUSY_SHFT);

	seq_printf(s, "MEM_DMA Ring Configuration\n");
	seq_printf(s, "%20s %6s %10s %10s %10s %10s %10s\n",
		"Name", "Used", "Base", "Cnt", "CIDX", "DIDX", "QCnt");
	dump_dma_tx_ring_info(s, dev, "T0:CmdEvent(WM2WA)", "AP",
		WF_WFDMA_MEM_DMA_WPDMA_TX_RING0_CTRL0_ADDR);
	dump_dma_tx_ring_info(s, dev, "T1:CmdEvent(WA2WM)", "AP",
		WF_WFDMA_MEM_DMA_WPDMA_TX_RING1_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R0:CmdEvent(WM2WA)", "AP",
		WF_WFDMA_MEM_DMA_WPDMA_RX_RING0_CTRL0_ADDR);
	dump_dma_rx_ring_info(s, dev, "R1:CmdEvent(WA2WM)", "AP",
		WF_WFDMA_MEM_DMA_WPDMA_RX_RING1_CTRL0_ADDR);
}

static int mt7996_trinfo_read(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	mt7996_show_dma_info(s, dev);
	return 0;
}

/* MIB INFO */
static int mt7996_mibinfo_show(struct seq_file *s, void *data)
{
#define BSS_NUM	4
	struct mt7996_phy *phy = s->private;
	struct mt7996_dev *dev = phy->dev;
	u8 band_idx = phy->mt76->band_idx;
	u8 bss_nums = BSS_NUM;
	u32 idx;
	u32 mac_val, band_offset = 0, band_offset_umib = 0;
	u32 msdr6, msdr9, msdr18;
	u32 rvsr0, rscr26, rscr35, mctr5, mctr6, msr0, msr1, msr2;
	u32 tbcr0, tbcr1, tbcr2, tbcr3, tbcr4;
	u32 btscr[7];
	u32 tdrcr[5];
	u32 mbtocr[16], mbtbcr[16], mbrocr[16], mbrbcr[16];
	u32 btcr, btbcr, brocr, brbcr, btdcr, brdcr;
	u32 mu_cnt[5];
	u32 ampdu_cnt[3];
	u64 per = 0;

	switch (band_idx) {
	case 0:
		band_offset = 0;
		band_offset_umib = 0;
		break;
	case 1:
		band_offset = BN1_WF_MIB_TOP_BASE - BN0_WF_MIB_TOP_BASE;
		band_offset_umib = WF_UMIB_TOP_B1BROCR_ADDR - WF_UMIB_TOP_B0BROCR_ADDR;
		break;
	case 2:
		band_offset = IP1_BN0_WF_MIB_TOP_BASE - BN0_WF_MIB_TOP_BASE;
		band_offset_umib = WF_UMIB_TOP_B2BROCR_ADDR - WF_UMIB_TOP_B0BROCR_ADDR;
		break;
	default:
		return true;
	}

	seq_printf(s, "Band %d MIB Status\n", band_idx);
	seq_printf(s, "===============================\n");
	mac_val = mt76_rr(dev, BN0_WF_MIB_TOP_M0SCR0_ADDR + band_offset);
	seq_printf(s, "MIB Status Control=0x%x\n", mac_val);

	msdr6 = mt76_rr(dev, BN0_WF_MIB_TOP_M0SDR6_ADDR + band_offset);
	rvsr0 = mt76_rr(dev, BN0_WF_MIB_TOP_RVSR0_ADDR + band_offset);
	rscr35 = mt76_rr(dev, BN0_WF_MIB_TOP_RSCR35_ADDR + band_offset);
	msdr9 = mt76_rr(dev, BN0_WF_MIB_TOP_M0SDR9_ADDR + band_offset);
	rscr26 = mt76_rr(dev, BN0_WF_MIB_TOP_RSCR26_ADDR + band_offset);
	mctr5 = mt76_rr(dev, BN0_WF_MIB_TOP_MCTR5_ADDR + band_offset);
	mctr6 = mt76_rr(dev, BN0_WF_MIB_TOP_MCTR6_ADDR + band_offset);
	msdr18 = mt76_rr(dev, BN0_WF_MIB_TOP_M0SDR18_ADDR + band_offset);
	msr0 = mt76_rr(dev, BN0_WF_MIB_TOP_MSR0_ADDR + band_offset);
	msr1 = mt76_rr(dev, BN0_WF_MIB_TOP_MSR1_ADDR + band_offset);
	msr2 = mt76_rr(dev, BN0_WF_MIB_TOP_MSR2_ADDR + band_offset);
	ampdu_cnt[0] = mt76_rr(dev, MT_MIB_TSCR0(band_idx));
	ampdu_cnt[1] = mt76_rr(dev, MT_MIB_TSCR3(band_idx));
	ampdu_cnt[2] = mt76_rr(dev, MT_MIB_TSCR4(band_idx));
	ampdu_cnt[1] &= BN0_WF_MIB_TOP_TSCR3_AMPDU_MPDU_COUNT_MASK;
	ampdu_cnt[2] &= BN0_WF_MIB_TOP_TSCR4_AMPDU_ACKED_COUNT_MASK;

	seq_printf(s, "===Phy/Timing Related Counters===\n");
	seq_printf(s, "\tChannelIdleCnt=0x%x\n",
		msdr6 & BN0_WF_MIB_TOP_M0SDR6_CHANNEL_IDLE_COUNT_MASK);
	seq_printf(s, "\tCCA_NAV_Tx_Time=0x%x\n",
		msdr9 & BN0_WF_MIB_TOP_M0SDR9_CCA_NAV_TX_TIME_MASK);
	seq_printf(s, "\tRx_MDRDY_CNT=0x%x\n",
		rscr26 & BN0_WF_MIB_TOP_RSCR26_RX_MDRDY_COUNT_MASK);
	seq_printf(s, "\tCCK_MDRDY_TIME=0x%x, OFDM_MDRDY_TIME=0x%x",
		msr0 & BN0_WF_MIB_TOP_MSR0_CCK_MDRDY_TIME_MASK,
		msr1 & BN0_WF_MIB_TOP_MSR1_OFDM_LG_MIXED_VHT_MDRDY_TIME_MASK);
	seq_printf(s, ", OFDM_GREEN_MDRDY_TIME=0x%x\n",
		msr2 & BN0_WF_MIB_TOP_MSR2_OFDM_GREEN_MDRDY_TIME_MASK);
	seq_printf(s, "\tPrim CCA Time=0x%x\n",
		mctr5 & BN0_WF_MIB_TOP_MCTR5_P_CCA_TIME_MASK);
	seq_printf(s, "\tSec CCA Time=0x%x\n",
		mctr6 & BN0_WF_MIB_TOP_MCTR6_S_CCA_TIME_MASK);
	seq_printf(s, "\tPrim ED Time=0x%x\n",
		msdr18 & BN0_WF_MIB_TOP_M0SDR18_P_ED_TIME_MASK);

	seq_printf(s, "===Tx Related Counters(Generic)===\n");
	mac_val = mt76_rr(dev, BN0_WF_MIB_TOP_TSCR18_ADDR + band_offset);
	dev->dbg.bcn_total_cnt[band_idx] +=
		(mac_val & BN0_WF_MIB_TOP_TSCR18_BEACONTXCOUNT_MASK);
	seq_printf(s, "\tBeaconTxCnt=0x%x\n", dev->dbg.bcn_total_cnt[band_idx]);
	dev->dbg.bcn_total_cnt[band_idx] = 0;

	tbcr0 = mt76_rr(dev, BN0_WF_MIB_TOP_TBCR0_ADDR + band_offset);
	seq_printf(s, "\tTx 20MHz Cnt=0x%x\n",
		tbcr0 & BN0_WF_MIB_TOP_TBCR0_TX_20MHZ_CNT_MASK);
	tbcr1 = mt76_rr(dev, BN0_WF_MIB_TOP_TBCR1_ADDR + band_offset);
	seq_printf(s, "\tTx 40MHz Cnt=0x%x\n",
		tbcr1 & BN0_WF_MIB_TOP_TBCR1_TX_40MHZ_CNT_MASK);
	tbcr2 = mt76_rr(dev, BN0_WF_MIB_TOP_TBCR2_ADDR + band_offset);
	seq_printf(s, "\tTx 80MHz Cnt=0x%x\n",
		tbcr2 & BN0_WF_MIB_TOP_TBCR2_TX_80MHZ_CNT_MASK);
	tbcr3 = mt76_rr(dev, BN0_WF_MIB_TOP_TBCR3_ADDR + band_offset);
	seq_printf(s, "\tTx 160MHz Cnt=0x%x\n",
		tbcr3 & BN0_WF_MIB_TOP_TBCR3_TX_160MHZ_CNT_MASK);
	tbcr4 = is_mt7996(&dev->mt76) ? mt76_rr(dev, BN0_WF_MIB_TOP_TBCR4_ADDR + band_offset) : 0;
	seq_printf(s, "\tTx 320MHz Cnt=0x%x\n",
		tbcr4 & BN0_WF_MIB_TOP_TBCR4_TX_320MHZ_CNT_MASK);
	seq_printf(s, "\tAMPDU Cnt=0x%x\n", ampdu_cnt[0]);
	seq_printf(s, "\tAMPDU MPDU Cnt=0x%x\n", ampdu_cnt[1]);
	seq_printf(s, "\tAMPDU MPDU Ack Cnt=0x%x\n", ampdu_cnt[2]);
	if (ampdu_cnt[1])
		per = (u64)1000 * (ampdu_cnt[1] - ampdu_cnt[2]) / ampdu_cnt[1];
	seq_printf(s, "\tAMPDU MPDU PER=%llu.%1llu%%\n", per / 10, per % 10);

	seq_printf(s, "===MU Related Counters===\n");
	mu_cnt[0] = mt76_rr(dev, BN0_WF_MIB_TOP_BSCR2_ADDR + band_offset);
	mu_cnt[1] = mt76_rr(dev, MT_MIB_TSCR5(band_idx));
	mu_cnt[2] = mt76_rr(dev, MT_MIB_TSCR6(band_idx));
	mu_cnt[3] = mt76_rr(dev, BN0_WF_MIB_TOP_TSCR8_ADDR + band_offset);
	mu_cnt[4] = mt76_rr(dev, MT_MIB_TSCR7(band_idx));

	seq_printf(s, "\tMUBF_TX_COUNT=0x%x\n",
		mu_cnt[0] & BN0_WF_MIB_TOP_BSCR2_MUBF_TX_COUNT_MASK);
	seq_printf(s, "\tMU_TX_MPDU_COUNT(Ok+Fail)=0x%x\n", mu_cnt[1]);
	seq_printf(s, "\tMU_TX_OK_MPDU_COUNT=0x%x\n", mu_cnt[2]);
	seq_printf(s, "\tSU_TX_MPDU_COUNT(Ok+Fail)=0x%x\n", mu_cnt[3]);
	seq_printf(s, "\tSU_TX_OK_MPDU_COUNT=0x%x\n", mu_cnt[4]);

	seq_printf(s, "===Rx Related Counters(Generic)===\n");
	seq_printf(s, "\tVector Mismacth Cnt=0x%x\n",
		rvsr0 & BN0_WF_MIB_TOP_RVSR0_VEC_MISS_COUNT_MASK);
	seq_printf(s, "\tDelimiter Fail Cnt=0x%x\n",
		rscr35 & BN0_WF_MIB_TOP_RSCR35_DELIMITER_FAIL_COUNT_MASK);

	mac_val = mt76_rr(dev, BN0_WF_MIB_TOP_RSCR1_ADDR + band_offset);
	seq_printf(s, "\tRxFCSErrCnt=0x%x\n",
		(mac_val & BN0_WF_MIB_TOP_RSCR1_RX_FCS_ERROR_COUNT_MASK));
	mac_val = mt76_rr(dev, BN0_WF_MIB_TOP_RSCR33_ADDR + band_offset);
	seq_printf(s, "\tRxFifoFullCnt=0x%x\n",
		(mac_val & BN0_WF_MIB_TOP_RSCR33_RX_FIFO_FULL_COUNT_MASK));
	mac_val = mt76_rr(dev, BN0_WF_MIB_TOP_RSCR36_ADDR + band_offset);
	seq_printf(s, "\tRxLenMismatch=0x%x\n",
		(mac_val & BN0_WF_MIB_TOP_RSCR36_RX_LEN_MISMATCH_MASK));
	mac_val = mt76_rr(dev, BN0_WF_MIB_TOP_RSCR31_ADDR + band_offset);
	seq_printf(s, "\tRxMPDUCnt=0x%x\n",
		(mac_val & BN0_WF_MIB_TOP_RSCR31_RX_MPDU_COUNT_MASK));
	mac_val = mt76_rr(dev, BN0_WF_MIB_TOP_RSCR27_ADDR + band_offset);
	seq_printf(s, "\tRx AMPDU Cnt=0x%x\n", mac_val);
	mac_val = mt76_rr(dev, BN0_WF_MIB_TOP_RSCR28_ADDR + band_offset);
	seq_printf(s, "\tRx Total ByteCnt=0x%x\n", mac_val);


	/* Per-BSS T/RX Counters */
	seq_printf(s, "===Per-BSS Related Tx/Rx Counters===\n");
	seq_printf(s, "BSS Idx TxCnt/DataCnt TxByteCnt RxOkCnt/DataCnt RxByteCnt\n");
	for (idx = 0; idx < bss_nums; idx++) {
		btcr = mt76_rr(dev, BN0_WF_MIB_TOP_BTCR_ADDR + band_offset + idx * 4);
		btdcr = mt76_rr(dev, BN0_WF_MIB_TOP_BTDCR_ADDR + band_offset + idx * 4);
		btbcr = mt76_rr(dev, BN0_WF_MIB_TOP_BTBCR_ADDR + band_offset + idx * 4);

		brocr = mt76_rr(dev, WF_UMIB_TOP_B0BROCR_ADDR + band_offset_umib + idx * 4);
		brdcr = mt76_rr(dev, WF_UMIB_TOP_B0BRDCR_ADDR + band_offset_umib + idx * 4);
		brbcr = mt76_rr(dev, WF_UMIB_TOP_B0BRBCR_ADDR + band_offset_umib + idx * 4);

		seq_printf(s, "%d\t 0x%x/0x%x\t 0x%x \t 0x%x/0x%x \t 0x%x\n",
			idx, btcr, btdcr, btbcr, brocr, brdcr, brbcr);
	}

	seq_printf(s, "===Per-BSS Related MIB Counters===\n");
	seq_printf(s, "BSS Idx RTSTx/RetryCnt BAMissCnt AckFailCnt FrmRetry1/2/3Cnt\n");

	/* Per-BSS TX Status */
	for (idx = 0; idx < bss_nums; idx++) {
		btscr[0] = mt76_rr(dev, BN0_WF_MIB_TOP_BTSCR5_ADDR + band_offset + idx * 4);
		btscr[1] = mt76_rr(dev, BN0_WF_MIB_TOP_BTSCR6_ADDR + band_offset + idx * 4);
		btscr[2] = mt76_rr(dev, BN0_WF_MIB_TOP_BTSCR0_ADDR + band_offset + idx * 4);
		btscr[3] = mt76_rr(dev, BN0_WF_MIB_TOP_BTSCR1_ADDR + band_offset + idx * 4);
		btscr[4] = mt76_rr(dev, BN0_WF_MIB_TOP_BTSCR2_ADDR + band_offset + idx * 4);
		btscr[5] = mt76_rr(dev, BN0_WF_MIB_TOP_BTSCR3_ADDR + band_offset + idx * 4);
		btscr[6] = mt76_rr(dev, BN0_WF_MIB_TOP_BTSCR4_ADDR + band_offset + idx * 4);

		seq_printf(s, "%d:\t0x%x/0x%x  0x%x \t 0x%x \t  0x%x/0x%x/0x%x\n",
			idx, (btscr[0] & BN0_WF_MIB_TOP_BTSCR5_RTSTXCOUNTn_MASK),
			(btscr[1] & BN0_WF_MIB_TOP_BTSCR6_RTSRETRYCOUNTn_MASK),
			(btscr[2] & BN0_WF_MIB_TOP_BTSCR0_BAMISSCOUNTn_MASK),
			(btscr[3] & BN0_WF_MIB_TOP_BTSCR1_ACKFAILCOUNTn_MASK),
			(btscr[4] & BN0_WF_MIB_TOP_BTSCR2_FRAMERETRYCOUNTn_MASK),
			(btscr[5] & BN0_WF_MIB_TOP_BTSCR3_FRAMERETRY2COUNTn_MASK),
			(btscr[6] & BN0_WF_MIB_TOP_BTSCR4_FRAMERETRY3COUNTn_MASK));
	}

	/* Dummy delimiter insertion result */
	seq_printf(s, "===Dummy delimiter insertion result===\n");
	tdrcr[0] = mt76_rr(dev, BN0_WF_MIB_TOP_TDRCR0_ADDR + band_offset);
	tdrcr[1] = mt76_rr(dev, BN0_WF_MIB_TOP_TDRCR1_ADDR + band_offset);
	tdrcr[2] = mt76_rr(dev, BN0_WF_MIB_TOP_TDRCR2_ADDR + band_offset);
	tdrcr[3] = mt76_rr(dev, BN0_WF_MIB_TOP_TDRCR3_ADDR + band_offset);
	tdrcr[4] = mt76_rr(dev, BN0_WF_MIB_TOP_TDRCR4_ADDR + band_offset);

	seq_printf(s, "Range0 = %d\t Range1 = %d\t Range2 = %d\t Range3 = %d\t Range4 = %d\n",
		tdrcr[0],
		tdrcr[1],
		tdrcr[2],
		tdrcr[3],
		tdrcr[4]);

	/* Per-MBSS T/RX Counters */
	seq_printf(s, "===Per-MBSS Related Tx/Rx Counters===\n");
	seq_printf(s, "MBSSIdx   TxOkCnt  TxByteCnt  RxOkCnt  RxByteCnt\n");

	for (idx = 0; idx < 16; idx++) {
		mbtocr[idx] = mt76_rr(dev, BN0_WF_MIB_TOP_BTOCR_ADDR + band_offset + (bss_nums + idx) * 4);
		mbtbcr[idx] = mt76_rr(dev, BN0_WF_MIB_TOP_BTBCR_ADDR + band_offset + (bss_nums + idx) * 4);

		mbrocr[idx] = mt76_rr(dev, WF_UMIB_TOP_B0BROCR_ADDR + band_offset_umib + (bss_nums + idx) * 4);
		mbrbcr[idx] = mt76_rr(dev, WF_UMIB_TOP_B0BRBCR_ADDR + band_offset_umib + (bss_nums + idx) * 4);
	}

	for (idx = 0; idx < 16; idx++) {
		seq_printf(s, "%d\t 0x%x\t 0x%x \t 0x%x \t 0x%x\n",
			idx, mbtocr[idx], mbtbcr[idx], mbrocr[idx], mbrbcr[idx]);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt7996_mibinfo);

/* WTBL INFO */
static int
mt7996_wtbl_read_raw(struct mt7996_dev *dev, u16 idx,
		     enum mt7996_wtbl_type type, u16 start_dw,
		     u16 len, void *buf)
{
	u32 *dest_cpy = (u32 *)buf;
	u32 size_dw = len;
	u32 src = 0;

	if (!buf)
		return 0xFF;

	if (type == WTBL_TYPE_LMAC) {
		mt76_wr(dev, MT_WTBLON_TOP_WDUCR,
			FIELD_PREP(MT_WTBLON_TOP_WDUCR_GROUP, (idx >> 7)));
		src = LWTBL_IDX2BASE(idx, start_dw);
	} else if (type == WTBL_TYPE_UMAC) {
		mt76_wr(dev,  MT_DBG_UWTBL_TOP_WDUCR_ADDR,
			FIELD_PREP(MT_DBG_UWTBL_TOP_WDUCR_GROUP, (idx >> 7)));
		src = UWTBL_IDX2BASE(idx, start_dw);
	} else if (type == WTBL_TYPE_KEY) {
		if (is_mt7990(&dev->mt76)) {
			mt76_wr(dev,  MT_DBG_UWTBL_TOP_KDUCR_ADDR,
				FIELD_PREP(MT_DBG_UWTBL_TOP_KDUCR_GROUP, (idx >> 6)));
			src = KEYTBL_IDX2BASE_7990(idx, start_dw);
		} else {
			mt76_wr(dev,  MT_DBG_UWTBL_TOP_WDUCR_ADDR,
				MT_DBG_UWTBL_TOP_WDUCR_TARGET |
				FIELD_PREP(MT_DBG_UWTBL_TOP_WDUCR_GROUP, (idx >> 7)));
			src = KEYTBL_IDX2BASE(idx, start_dw);
		}

	}

	while (size_dw--) {
		*dest_cpy++ = mt76_rr(dev, src);
		src += 4;
	};

	return 0;
}

#if 0
static int
mt7996_wtbl_write_raw(struct mt7996_dev *dev, u16 idx,
			  enum mt7996_wtbl_type type, u16 start_dw,
			  u32 val)
{
	u32 addr = 0;

	if (type == WTBL_TYPE_LMAC) {
		mt76_wr(dev, MT_WTBLON_TOP_WDUCR,
			FIELD_PREP(MT_WTBLON_TOP_WDUCR_GROUP, (idx >> 7)));
		addr = LWTBL_IDX2BASE(idx, start_dw);
	} else if (type == WTBL_TYPE_UMAC) {
		mt76_wr(dev, MT_DBG_UWTBL_TOP_WDUCR_ADDR,
			FIELD_PREP(MT_DBG_UWTBL_TOP_WDUCR_GROUP, (idx >> 7)));
		addr = UWTBL_IDX2BASE(idx, start_dw);
	} else if (type == WTBL_TYPE_KEY) {
		mt76_wr(dev, MT_DBG_UWTBL_TOP_WDUCR_ADDR,
			MT_DBG_UWTBL_TOP_WDUCR_TARGET |
			FIELD_PREP(MT_DBG_UWTBL_TOP_WDUCR_GROUP, (idx >> 7)));
		addr = KEYTBL_IDX2BASE(idx, start_dw);
	}

	mt76_wr(dev, addr, val);

	return 0;
}
#endif

static const struct berse_wtbl_parse WTBL_LMAC_DW0[] = {
	{"MUAR_IDX",	WF_LWTBL_MUAR_MASK,	WF_LWTBL_MUAR_SHIFT,	false},
	{"RCA1",	WF_LWTBL_RCA1_MASK,	NO_SHIFT_DEFINE,	false},
	{"KID",		WF_LWTBL_KID_MASK,	WF_LWTBL_KID_SHIFT,	false},
	{"RCID",	WF_LWTBL_RCID_MASK,	NO_SHIFT_DEFINE,	false},
	{"BAND",	WF_LWTBL_BAND_MASK,	WF_LWTBL_BAND_SHIFT,	false},
	{"RV",		WF_LWTBL_RV_MASK,	NO_SHIFT_DEFINE,	false},
	{"RCA2",	WF_LWTBL_RCA2_MASK,	NO_SHIFT_DEFINE,	false},
	{"WPI_FLAG",	WF_LWTBL_WPI_FLAG_MASK,	NO_SHIFT_DEFINE,	true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw0_1(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	seq_printf(s, "\t\n");
	seq_printf(s, "LinkAddr: %02x:%02x:%02x:%02x:%02x:%02x(D0[B0~15], D1[B0~31])\n",
		lwtbl[4], lwtbl[5], lwtbl[6], lwtbl[7], lwtbl[0], lwtbl[1]);

	/* LMAC WTBL DW 0 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 0/1\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_PEER_INFO_DW_0*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW0[i].name) {

		if (WTBL_LMAC_DW0[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW0[i].name,
					 (dw_value & WTBL_LMAC_DW0[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW0[i].name,
					  (dw_value & WTBL_LMAC_DW0[i].mask) >> WTBL_LMAC_DW0[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse *WTBL_LMAC_DW2;
static const struct berse_wtbl_parse WTBL_LMAC_DW2_7996[] = {
	{"AID",			WF_LWTBL_AID_MASK,		WF_LWTBL_AID_SHIFT,			false},
	{"GID_SU",		WF_LWTBL_GID_SU_MASK,		NO_SHIFT_DEFINE,			false},
	{"SPP_EN",		WF_LWTBL_SPP_EN_MASK,		NO_SHIFT_DEFINE,			false},
	{"WPI_EVEN",		WF_LWTBL_WPI_EVEN_MASK,		NO_SHIFT_DEFINE,			false},
	{"AAD_OM",		WF_LWTBL_AAD_OM_MASK,		NO_SHIFT_DEFINE,			false},
	{"CIPHER_PGTK",		WF_LWTBL_CIPHER_SUIT_PGTK_MASK,	WF_LWTBL_CIPHER_SUIT_PGTK_SHIFT,	true},
	{"FROM_DS",		WF_LWTBL_FD_MASK,		NO_SHIFT_DEFINE,			false},
	{"TO_DS",		WF_LWTBL_TD_MASK,		NO_SHIFT_DEFINE,			false},
	{"SW",			WF_LWTBL_SW_MASK,		NO_SHIFT_DEFINE,			false},
	{"UL",			WF_LWTBL_UL_MASK,		NO_SHIFT_DEFINE,			false},
	{"TX_POWER_SAVE",	WF_LWTBL_TX_PS_MASK,		NO_SHIFT_DEFINE,			true},
	{"QOS",			WF_LWTBL_QOS_MASK,		NO_SHIFT_DEFINE,			false},
	{"HT",			WF_LWTBL_HT_MASK,		NO_SHIFT_DEFINE,			false},
	{"VHT",			WF_LWTBL_VHT_MASK,		NO_SHIFT_DEFINE,			false},
	{"HE",			WF_LWTBL_HE_MASK,		NO_SHIFT_DEFINE,			false},
	{"EHT",			WF_LWTBL_EHT_MASK,		NO_SHIFT_DEFINE,			false},
	{"MESH",		WF_LWTBL_MESH_MASK,		NO_SHIFT_DEFINE,			true},
	{NULL,}
};

static const struct berse_wtbl_parse WTBL_LMAC_DW2_7992[] = {
	{"AID",			WF_LWTBL_AID_MASK,		WF_LWTBL_AID_SHIFT,			false},
	{"GID_SU",		WF_LWTBL_GID_SU_MASK,		NO_SHIFT_DEFINE,			false},
	{"DUAL_PTEC_EN",	WF_LWTBL_DUAL_PTEC_EN_MASK,	NO_SHIFT_DEFINE,			false},
	{"DUAL_CTS_CAP",	WF_LWTBL_DUAL_CTS_CAP_MASK,	NO_SHIFT_DEFINE,			false},
	{"CIPHER_PGTK",		WF_LWTBL_CIPHER_SUIT_PGTK_MASK,	WF_LWTBL_CIPHER_SUIT_PGTK_SHIFT,	true},
	{"FROM_DS",		WF_LWTBL_FD_MASK,		NO_SHIFT_DEFINE,			false},
	{"TO_DS",		WF_LWTBL_TD_MASK,		NO_SHIFT_DEFINE,			false},
	{"SW",			WF_LWTBL_SW_MASK,		NO_SHIFT_DEFINE,			false},
	{"UL",			WF_LWTBL_UL_MASK,		NO_SHIFT_DEFINE,			false},
	{"TX_POWER_SAVE",	WF_LWTBL_TX_PS_MASK,		NO_SHIFT_DEFINE,			true},
	{"QOS",			WF_LWTBL_QOS_MASK,		NO_SHIFT_DEFINE,			false},
	{"HT",			WF_LWTBL_HT_MASK,		NO_SHIFT_DEFINE,			false},
	{"VHT",			WF_LWTBL_VHT_MASK,		NO_SHIFT_DEFINE,			false},
	{"HE",			WF_LWTBL_HE_MASK,		NO_SHIFT_DEFINE,			false},
	{"EHT",			WF_LWTBL_EHT_MASK,		NO_SHIFT_DEFINE,			false},
	{"MESH",		WF_LWTBL_MESH_MASK,		NO_SHIFT_DEFINE,			true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw2(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 2 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 2\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_TRX_CAP_DW_2*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW2[i].name) {

		if (WTBL_LMAC_DW2[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW2[i].name,
					 (dw_value & WTBL_LMAC_DW2[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW2[i].name,
					  (dw_value & WTBL_LMAC_DW2[i].mask) >> WTBL_LMAC_DW2[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW3[] = {
	{"WMM_Q",		WF_LWTBL_WMM_Q_MASK,			WF_LWTBL_WMM_Q_SHIFT,			false},
	{"EHT_SIG_MCS",		WF_LWTBL_EHT_SIG_MCS_MASK,		WF_LWTBL_EHT_SIG_MCS_SHIFT,		false},
	{"HDRT_MODE",		WF_LWTBL_HDRT_MODE_MASK,		NO_SHIFT_DEFINE,			false},
	{"BEAM_CHG",		WF_LWTBL_BEAM_CHG_MASK,			NO_SHIFT_DEFINE,			false},
	{"EHT_LTF_SYM_NUM",	WF_LWTBL_EHT_LTF_SYM_NUM_OPT_MASK,	WF_LWTBL_EHT_LTF_SYM_NUM_OPT_SHIFT,	true},
	{"PFMU_IDX",		WF_LWTBL_PFMU_IDX_MASK,			WF_LWTBL_PFMU_IDX_SHIFT,		false},
	{"ULPF_IDX",		WF_LWTBL_ULPF_IDX_MASK,			WF_LWTBL_ULPF_IDX_SHIFT,		false},
	{"RIBF",		WF_LWTBL_RIBF_MASK,			NO_SHIFT_DEFINE,			false},
	{"ULPF",		WF_LWTBL_ULPF_MASK,			NO_SHIFT_DEFINE,			false},
	{"BYPASS_TXSMM",	WF_LWTBL_BYPASS_TXSMM_MASK,		NO_SHIFT_DEFINE,			true},
	{"TBF_HT",		WF_LWTBL_TBF_HT_MASK,			NO_SHIFT_DEFINE,			false},
	{"TBF_VHT",		WF_LWTBL_TBF_VHT_MASK,			NO_SHIFT_DEFINE,			false},
	{"TBF_HE",		WF_LWTBL_TBF_HE_MASK,			NO_SHIFT_DEFINE,			false},
	{"TBF_EHT",		WF_LWTBL_TBF_EHT_MASK,			NO_SHIFT_DEFINE,			false},
	{"IGN_FBK",		WF_LWTBL_IGN_FBK_MASK,			NO_SHIFT_DEFINE,			true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw3(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 3 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 3\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_TRX_CAP_DW_3*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW3[i].name) {

		if (WTBL_LMAC_DW3[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW3[i].name,
					 (dw_value & WTBL_LMAC_DW3[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW3[i].name,
					  (dw_value & WTBL_LMAC_DW3[i].mask) >> WTBL_LMAC_DW3[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW4[] = {
	{"NEGOTIATED_WINSIZE0",	WF_LWTBL_NEGOTIATED_WINSIZE0_MASK,	WF_LWTBL_NEGOTIATED_WINSIZE0_SHIFT,	false},
	{"WINSIZE1",		WF_LWTBL_NEGOTIATED_WINSIZE1_MASK,	WF_LWTBL_NEGOTIATED_WINSIZE1_SHIFT,	false},
	{"WINSIZE2",		WF_LWTBL_NEGOTIATED_WINSIZE2_MASK,	WF_LWTBL_NEGOTIATED_WINSIZE2_SHIFT,	false},
	{"WINSIZE3",		WF_LWTBL_NEGOTIATED_WINSIZE3_MASK,	WF_LWTBL_NEGOTIATED_WINSIZE3_SHIFT,	true},
	{"WINSIZE4",		WF_LWTBL_NEGOTIATED_WINSIZE4_MASK,	WF_LWTBL_NEGOTIATED_WINSIZE4_SHIFT,	false},
	{"WINSIZE5",		WF_LWTBL_NEGOTIATED_WINSIZE5_MASK,	WF_LWTBL_NEGOTIATED_WINSIZE5_SHIFT,	false},
	{"WINSIZE6",		WF_LWTBL_NEGOTIATED_WINSIZE6_MASK,	WF_LWTBL_NEGOTIATED_WINSIZE6_SHIFT,	false},
	{"WINSIZE7",		WF_LWTBL_NEGOTIATED_WINSIZE7_MASK,	WF_LWTBL_NEGOTIATED_WINSIZE7_SHIFT,	true},
	{"PE",			WF_LWTBL_PE_MASK,			WF_LWTBL_PE_SHIFT,			false},
	{"DIS_RHTR",		WF_LWTBL_DIS_RHTR_MASK,			NO_SHIFT_DEFINE,			false},
	{"LDPC_HT",		WF_LWTBL_LDPC_HT_MASK,			NO_SHIFT_DEFINE,			false},
	{"LDPC_VHT",		WF_LWTBL_LDPC_VHT_MASK,			NO_SHIFT_DEFINE,			false},
	{"LDPC_HE",		WF_LWTBL_LDPC_HE_MASK,			NO_SHIFT_DEFINE,			false},
	{"LDPC_EHT",		WF_LWTBL_LDPC_EHT_MASK,			NO_SHIFT_DEFINE,			true},
	{"BA_MODE",		WF_LWTBL_BA_MODE_MASK,			NO_SHIFT_DEFINE,			true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw4(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 4 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 4\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_TRX_CAP_DW_4*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW4[i].name) {
		if (WTBL_LMAC_DW4[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW4[i].name,
					 (dw_value & WTBL_LMAC_DW4[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW4[i].name,
					  (dw_value & WTBL_LMAC_DW4[i].mask) >> WTBL_LMAC_DW4[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse *WTBL_LMAC_DW5;
static const struct berse_wtbl_parse WTBL_LMAC_DW5_7996[] = {
	{"AF",			WF_LWTBL_AF_MASK,		WF_LWTBL_AF_SHIFT,		false},
	{"AF_HE",		WF_LWTBL_AF_HE_MASK,		WF_LWTBL_AF_HE_SHIFT,		false},
	{"RTS",			WF_LWTBL_RTS_MASK,		NO_SHIFT_DEFINE,		false},
	{"SMPS",		WF_LWTBL_SMPS_MASK,		NO_SHIFT_DEFINE,		false},
	{"DYN_BW",		WF_LWTBL_DYN_BW_MASK,		NO_SHIFT_DEFINE,		true},
	{"MMSS",		WF_LWTBL_MMSS_MASK,		WF_LWTBL_MMSS_SHIFT,		false},
	{"USR",			WF_LWTBL_USR_MASK,		NO_SHIFT_DEFINE,		false},
	{"SR_RATE",		WF_LWTBL_SR_R_MASK,		WF_LWTBL_SR_R_SHIFT,		false},
	{"SR_ABORT",		WF_LWTBL_SR_ABORT_MASK,		NO_SHIFT_DEFINE,		true},
	{"TX_POWER_OFFSET",	WF_LWTBL_TX_POWER_OFFSET_MASK,  WF_LWTBL_TX_POWER_OFFSET_SHIFT,	false},
	{"LTF_EHT",		WF_LWTBL_LTF_EHT_MASK,		WF_LWTBL_LTF_EHT_SHIFT, 	false},
	{"GI_EHT",		WF_LWTBL_GI_EHT_MASK,		WF_LWTBL_GI_EHT_SHIFT,		false},
	{"DOPPL",		WF_LWTBL_DOPPL_MASK,		NO_SHIFT_DEFINE,		false},
	{"TXOP_PS_CAP",		WF_LWTBL_TXOP_PS_CAP_MASK,	NO_SHIFT_DEFINE,		false},
	{"DONOT_UPDATE_I_PSM",	WF_LWTBL_DU_I_PSM_MASK,		NO_SHIFT_DEFINE,		true},
	{"I_PSM",		WF_LWTBL_I_PSM_MASK,		NO_SHIFT_DEFINE,		false},
	{"PSM",			WF_LWTBL_PSM_MASK,		NO_SHIFT_DEFINE,		false},
	{"SKIP_TX",		WF_LWTBL_SKIP_TX_MASK,		NO_SHIFT_DEFINE,		true},
	{NULL,}
};

static const struct berse_wtbl_parse WTBL_LMAC_DW5_7992[] = {
	{"AF",			WF_LWTBL_AF_MASK_7992,		WF_LWTBL_AF_SHIFT,		false},
	{"RTS",			WF_LWTBL_RTS_MASK,		NO_SHIFT_DEFINE,		false},
	{"SMPS",		WF_LWTBL_SMPS_MASK,		NO_SHIFT_DEFINE,		false},
	{"DYN_BW",		WF_LWTBL_DYN_BW_MASK,		NO_SHIFT_DEFINE,		true},
	{"MMSS",		WF_LWTBL_MMSS_MASK,		WF_LWTBL_MMSS_SHIFT,		false},
	{"USR",			WF_LWTBL_USR_MASK,		NO_SHIFT_DEFINE,		false},
	{"SR_RATE",		WF_LWTBL_SR_R_MASK,		WF_LWTBL_SR_R_SHIFT,		false},
	{"SR_ABORT",		WF_LWTBL_SR_ABORT_MASK,		NO_SHIFT_DEFINE,		true},
	{"TX_POWER_OFFSET",	WF_LWTBL_TX_POWER_OFFSET_MASK,	WF_LWTBL_TX_POWER_OFFSET_SHIFT,	false},
	{"LTF_EHT",		WF_LWTBL_LTF_EHT_MASK,		WF_LWTBL_LTF_EHT_SHIFT,		false},
	{"GI_EHT",		WF_LWTBL_GI_EHT_MASK,		WF_LWTBL_GI_EHT_SHIFT,		false},
	{"DOPPL",		WF_LWTBL_DOPPL_MASK,		NO_SHIFT_DEFINE,		false},
	{"TXOP_PS_CAP",		WF_LWTBL_TXOP_PS_CAP_MASK,	NO_SHIFT_DEFINE,		false},
	{"DONOT_UPDATE_I_PSM",	WF_LWTBL_DU_I_PSM_MASK,		NO_SHIFT_DEFINE,		true},
	{"I_PSM",		WF_LWTBL_I_PSM_MASK,		NO_SHIFT_DEFINE,		false},
	{"PSM",			WF_LWTBL_PSM_MASK,		NO_SHIFT_DEFINE,		false},
	{"SKIP_TX",		WF_LWTBL_SKIP_TX_MASK,		NO_SHIFT_DEFINE,		true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw5(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 5 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 5\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_TRX_CAP_DW_5*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW5[i].name) {
		if (WTBL_LMAC_DW5[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW5[i].name,
					 (dw_value & WTBL_LMAC_DW5[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW5[i].name,
					  (dw_value & WTBL_LMAC_DW5[i].mask) >> WTBL_LMAC_DW5[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW6[] = {
	{"CBRN",	WF_LWTBL_CBRN_MASK,	WF_LWTBL_CBRN_SHIFT,	false},
	{"DBNSS_EN",	WF_LWTBL_DBNSS_EN_MASK,	NO_SHIFT_DEFINE,	false},
	{"BAF_EN",	WF_LWTBL_BAF_EN_MASK,	NO_SHIFT_DEFINE,	false},
	{"RDGBA",	WF_LWTBL_RDGBA_MASK,	NO_SHIFT_DEFINE,	false},
	{"RDG",		WF_LWTBL_R_MASK,	NO_SHIFT_DEFINE,	false},
	{"SPE_IDX",	WF_LWTBL_SPE_IDX_MASK,	WF_LWTBL_SPE_IDX_SHIFT,	true},
	{"G2",		WF_LWTBL_G2_MASK,	NO_SHIFT_DEFINE,	false},
	{"G4",		WF_LWTBL_G4_MASK,	NO_SHIFT_DEFINE,	false},
	{"G8",		WF_LWTBL_G8_MASK,	NO_SHIFT_DEFINE,	false},
	{"G16",		WF_LWTBL_G16_MASK,	NO_SHIFT_DEFINE,	true},
	{"G2_LTF",	WF_LWTBL_G2_LTF_MASK,	WF_LWTBL_G2_LTF_SHIFT,	false},
	{"G4_LTF",	WF_LWTBL_G4_LTF_MASK,	WF_LWTBL_G4_LTF_SHIFT,	false},
	{"G8_LTF",	WF_LWTBL_G8_LTF_MASK,	WF_LWTBL_G8_LTF_SHIFT,	false},
	{"G16_LTF",	WF_LWTBL_G16_LTF_MASK,	WF_LWTBL_G16_LTF_SHIFT,	true},
	{"G2_HE",	WF_LWTBL_G2_HE_MASK,	WF_LWTBL_G2_HE_SHIFT,	false},
	{"G4_HE",	WF_LWTBL_G4_HE_MASK,	WF_LWTBL_G4_HE_SHIFT,	false},
	{"G8_HE",	WF_LWTBL_G8_HE_MASK,	WF_LWTBL_G8_HE_SHIFT,	false},
	{"G16_HE",	WF_LWTBL_G16_HE_MASK,	WF_LWTBL_G16_HE_SHIFT,	true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw6(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 6 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 6\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_TRX_CAP_DW_6*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW6[i].name) {
		if (WTBL_LMAC_DW6[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW6[i].name,
					 (dw_value & WTBL_LMAC_DW6[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW6[i].name,
					  (dw_value & WTBL_LMAC_DW6[i].mask) >> WTBL_LMAC_DW6[i].shift);
		i++;
	}
}

static void parse_fmac_lwtbl_dw7(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	int i = 0;

	/* LMAC WTBL DW 7 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 7\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_TRX_CAP_DW_7*4]);
	dw_value = *addr;

	for (i = 0; i < 8; i++) {
		seq_printf(s, "\tBA_WIN_SIZE%u:%lu\n", i, ((dw_value & BITS(i*4, i*4+3)) >> i*4));
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW8[] = {
	{"RTS_FAIL_CNT_AC0",	WF_LWTBL_AC0_RTS_FAIL_CNT_MASK,	WF_LWTBL_AC0_RTS_FAIL_CNT_SHIFT,	false},
	{"AC1",			WF_LWTBL_AC1_RTS_FAIL_CNT_MASK,	WF_LWTBL_AC1_RTS_FAIL_CNT_SHIFT,	false},
	{"AC2",			WF_LWTBL_AC2_RTS_FAIL_CNT_MASK,	WF_LWTBL_AC2_RTS_FAIL_CNT_SHIFT,	false},
	{"AC3",			WF_LWTBL_AC3_RTS_FAIL_CNT_MASK,	WF_LWTBL_AC3_RTS_FAIL_CNT_SHIFT,	true},
	{"PARTIAL_AID",		WF_LWTBL_PARTIAL_AID_MASK,	WF_LWTBL_PARTIAL_AID_SHIFT,		false},
	{"CHK_PER",		WF_LWTBL_CHK_PER_MASK,		NO_SHIFT_DEFINE,			true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw8(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 8 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 8\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_TRX_CAP_DW_8*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW8[i].name) {
		if (WTBL_LMAC_DW8[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW8[i].name,
					 (dw_value & WTBL_LMAC_DW8[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW8[i].name,
					  (dw_value & WTBL_LMAC_DW8[i].mask) >> WTBL_LMAC_DW8[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse *WTBL_LMAC_DW9;
static const struct berse_wtbl_parse WTBL_LMAC_DW9_7996[] = {
	{"RX_AVG_MPDU_SIZE",	WF_LWTBL_RX_AVG_MPDU_SIZE_MASK,	WF_LWTBL_RX_AVG_MPDU_SIZE_SHIFT,	false},
	{"PRITX_SW_MODE",	WF_LWTBL_PRITX_SW_MODE_MASK,	NO_SHIFT_DEFINE,			false},
	{"PRITX_ERSU",		WF_LWTBL_PRITX_ERSU_MASK,	NO_SHIFT_DEFINE,			false},
	{"PRITX_PLR",		WF_LWTBL_PRITX_PLR_MASK,	NO_SHIFT_DEFINE,			true},
	{"PRITX_DCM",		WF_LWTBL_PRITX_DCM_MASK,	NO_SHIFT_DEFINE,			false},
	{"PRITX_ER106T",	WF_LWTBL_PRITX_ER106T_MASK,	NO_SHIFT_DEFINE,			true},
	/* {"FCAP(0:20 1:~40)",	WTBL_FCAP_20_TO_160_MHZ,	WTBL_FCAP_20_TO_160_MHZ_OFFSET}, */
	{"MPDU_FAIL_CNT",	WF_LWTBL_MPDU_FAIL_CNT_MASK,	WF_LWTBL_MPDU_FAIL_CNT_SHIFT,		false},
	{"MPDU_OK_CNT",		WF_LWTBL_MPDU_OK_CNT_MASK,	WF_LWTBL_MPDU_OK_CNT_SHIFT,		false},
	{"RATE_IDX",		WF_LWTBL_RATE_IDX_MASK,		WF_LWTBL_RATE_IDX_SHIFT,		true},
	{NULL,}
};

static const struct berse_wtbl_parse WTBL_LMAC_DW9_7992[] = {
	{"RX_AVG_MPDU_SIZE",	WF_LWTBL_RX_AVG_MPDU_SIZE_MASK,		WF_LWTBL_RX_AVG_MPDU_SIZE_SHIFT,	false},
	{"PRITX_SW_MODE",	WF_LWTBL_PRITX_SW_MODE_MASK_7992,	NO_SHIFT_DEFINE,			false},
	{"PRITX_ERSU",		WF_LWTBL_PRITX_ERSU_MASK_7992,		NO_SHIFT_DEFINE,			false},
	{"PRITX_PLR",		WF_LWTBL_PRITX_PLR_MASK_7992,		NO_SHIFT_DEFINE,			true},
	{"PRITX_DCM",		WF_LWTBL_PRITX_DCM_MASK,		NO_SHIFT_DEFINE,			false},
	{"PRITX_ER106T",	WF_LWTBL_PRITX_ER106T_MASK,		NO_SHIFT_DEFINE,			true},
	/* {"FCAP(0:20 1:~40)",	WTBL_FCAP_20_TO_160_MHZ,		WTBL_FCAP_20_TO_160_MHZ_OFFSET}, */
	{"MPDU_FAIL_CNT",	WF_LWTBL_MPDU_FAIL_CNT_MASK,		WF_LWTBL_MPDU_FAIL_CNT_SHIFT,		false},
	{"MPDU_OK_CNT",		WF_LWTBL_MPDU_OK_CNT_MASK,		WF_LWTBL_MPDU_OK_CNT_SHIFT,		false},
	{"RATE_IDX",		WF_LWTBL_RATE_IDX_MASK,			WF_LWTBL_RATE_IDX_SHIFT,		true},
	{NULL,}
};

char *fcap_name[] = {"20MHz", "20/40MHz", "20/40/80MHz", "20/40/80/160/80+80MHz", "20/40/80/160/80+80/320MHz"};

static void parse_fmac_lwtbl_dw9(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 9 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 9\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_TRX_CAP_DW_9*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW9[i].name) {
		if (WTBL_LMAC_DW9[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW9[i].name,
					 (dw_value & WTBL_LMAC_DW9[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW9[i].name,
					  (dw_value & WTBL_LMAC_DW9[i].mask) >> WTBL_LMAC_DW9[i].shift);
		i++;
	}

	/* FCAP parser */
	seq_printf(s, "\t\n");
	seq_printf(s, "FCAP:%s\n", fcap_name[(dw_value & WF_LWTBL_FCAP_MASK) >> WF_LWTBL_FCAP_SHIFT]);
}

#define HW_TX_RATE_TO_MODE(_x)			(((_x) & WTBL_RATE_TX_MODE_MASK) >> WTBL_RATE_TX_MODE_OFFSET)
#define HW_TX_RATE_TO_MCS(_x, _mode)		((_x) & WTBL_RATE_TX_RATE_MASK >> WTBL_RATE_TX_RATE_OFFSET)
#define HW_TX_RATE_TO_NSS(_x)			(((_x) & WTBL_RATE_NSTS_MASK) >> WTBL_RATE_NSTS_OFFSET)
#define HW_TX_RATE_TO_STBC(_x)			(((_x) & WTBL_RATE_STBC_MASK) >> WTBL_RATE_STBC_OFFSET)

static char *HW_TX_MODE_STR[] = {"CCK", "OFDM", "HT-Mix", "HT-GF", "VHT",
				 "N/A", "N/A", "N/A",
				 "HE_SU", "HE_EXT_SU", "HE_TRIG", "HE_MU",
				 "N/A",
				 "EHT_EXT_SU", "EHT_TRIG", "EHT_MU"};
static char *HW_TX_RATE_CCK_STR[] = {"1M", "2Mlong", "5.5Mlong", "11Mlong", "N/A", "2Mshort", "5.5Mshort", "11Mshort", "N/A"};
static char *HW_TX_RATE_OFDM_STR[] = {"6M", "9M", "12M", "18M", "24M", "36M", "48M", "54M", "N/A"};

static char *hw_rate_ofdm_str(uint16_t ofdm_idx)
{
	switch (ofdm_idx) {
	case 11: /* 6M */
		return HW_TX_RATE_OFDM_STR[0];

	case 15: /* 9M */
		return HW_TX_RATE_OFDM_STR[1];

	case 10: /* 12M */
		return HW_TX_RATE_OFDM_STR[2];

	case 14: /* 18M */
		return HW_TX_RATE_OFDM_STR[3];

	case 9: /* 24M */
		return HW_TX_RATE_OFDM_STR[4];

	case 13: /* 36M */
		return HW_TX_RATE_OFDM_STR[5];

	case 8: /* 48M */
		return HW_TX_RATE_OFDM_STR[6];

	case 12: /* 54M */
		return HW_TX_RATE_OFDM_STR[7];

	default:
		return HW_TX_RATE_OFDM_STR[8];
	}
}

static char *hw_rate_str(u8 mode, uint16_t rate_idx)
{
	if (mode == 0)
		return rate_idx < 8 ? HW_TX_RATE_CCK_STR[rate_idx] : HW_TX_RATE_CCK_STR[8];
	else if (mode == 1)
		return hw_rate_ofdm_str(rate_idx);
	else
		return "MCS";
}

static void
parse_rate(struct seq_file *s, uint16_t rate_idx, uint16_t txrate)
{
	uint16_t txmode, mcs, nss, stbc;

	txmode = HW_TX_RATE_TO_MODE(txrate);
	mcs = HW_TX_RATE_TO_MCS(txrate, txmode);
	nss = HW_TX_RATE_TO_NSS(txrate);
	stbc = HW_TX_RATE_TO_STBC(txrate);

	seq_printf(s, "\tRate%d(0x%x):TxMode=%d(%s), TxRate=%d(%s), Nsts=%d, STBC=%d\n",
			  rate_idx + 1, txrate,
			  txmode, HW_TX_MODE_STR[txmode],
			  mcs, hw_rate_str(txmode, mcs), nss, stbc);
}


static const struct berse_wtbl_parse WTBL_LMAC_DW10[] = {
	{"RATE1",	WF_LWTBL_RATE1_MASK,	WF_LWTBL_RATE1_SHIFT},
	{"RATE2",	WF_LWTBL_RATE2_MASK,	WF_LWTBL_RATE2_SHIFT},
	{NULL,}
};

static void parse_fmac_lwtbl_dw10(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 10 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 10\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_AUTO_RATE_1_2*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW10[i].name) {
		parse_rate(s, i, (dw_value & WTBL_LMAC_DW10[i].mask) >> WTBL_LMAC_DW10[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW11[] = {
	{"RATE3",	WF_LWTBL_RATE3_MASK,	WF_LWTBL_RATE3_SHIFT},
	{"RATE4",	WF_LWTBL_RATE4_MASK,	WF_LWTBL_RATE4_SHIFT},
	{NULL,}
};

static void parse_fmac_lwtbl_dw11(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 11 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 11\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_AUTO_RATE_3_4*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW11[i].name) {
		parse_rate(s, i+2, (dw_value & WTBL_LMAC_DW11[i].mask) >> WTBL_LMAC_DW11[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW12[] = {
	{"RATE5",	WF_LWTBL_RATE5_MASK,	WF_LWTBL_RATE5_SHIFT},
	{"RATE6",	WF_LWTBL_RATE6_MASK,	WF_LWTBL_RATE6_SHIFT},
	{NULL,}
};

static void parse_fmac_lwtbl_dw12(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 12 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 12\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_AUTO_RATE_5_6*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW12[i].name) {
		parse_rate(s, i+4, (dw_value & WTBL_LMAC_DW12[i].mask) >> WTBL_LMAC_DW12[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW13[] = {
	{"RATE7",	WF_LWTBL_RATE7_MASK,	WF_LWTBL_RATE7_SHIFT},
	{"RATE8",	WF_LWTBL_RATE8_MASK,	WF_LWTBL_RATE8_SHIFT},
	{NULL,}
};

static void parse_fmac_lwtbl_dw13(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 13 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 13\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_AUTO_RATE_7_8*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW13[i].name) {
		parse_rate(s, i+6, (dw_value & WTBL_LMAC_DW13[i].mask) >> WTBL_LMAC_DW13[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW14_BMC[] = {
	{"CIPHER_IGTK",		WF_LWTBL_CIPHER_SUIT_IGTK_MASK,		WF_LWTBL_CIPHER_SUIT_IGTK_SHIFT,	false},
	{"CIPHER_BIGTK",	WF_LWTBL_CIPHER_SUIT_BIGTK_MASK,	WF_LWTBL_CIPHER_SUIT_BIGTK_SHIFT,	true},
	{NULL,}
};

static const struct berse_wtbl_parse WTBL_LMAC_DW14[] = {
	{"RATE1_TX_CNT",	WF_LWTBL_RATE1_TX_CNT_MASK,	WF_LWTBL_RATE1_TX_CNT_SHIFT,	false},
	{"RATE1_FAIL_CNT",	WF_LWTBL_RATE1_FAIL_CNT_MASK,	WF_LWTBL_RATE1_FAIL_CNT_SHIFT,	true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw14(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr, *muar_addr = 0;
	u32 dw_value, muar_dw_value = 0;
	u16 i = 0;

	/* DUMP DW14 for BMC entry only */
	muar_addr = (u32 *)&(lwtbl[WF_LWTBL_MUAR_DW*4]);
	muar_dw_value = *muar_addr;
	if (((muar_dw_value & WF_LWTBL_MUAR_MASK) >> WF_LWTBL_MUAR_SHIFT)
		== MUAR_INDEX_OWN_MAC_ADDR_BC_MC) {
		/* LMAC WTBL DW 14 */
		seq_printf(s, "\t\n");
		seq_printf(s, "LWTBL DW 14_BMC\n");
		addr = (u32 *)&(lwtbl[WF_LWTBL_CIPHER_SUIT_IGTK_DW*4]);
		dw_value = *addr;

		while (WTBL_LMAC_DW14_BMC[i].name) {
			if (WTBL_LMAC_DW14_BMC[i].shift == NO_SHIFT_DEFINE)
				seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW14_BMC[i].name,
					(dw_value & WTBL_LMAC_DW14_BMC[i].mask) ? 1 : 0);
			else
				seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW14_BMC[i].name,
					(dw_value & WTBL_LMAC_DW14_BMC[i].mask) >> WTBL_LMAC_DW14_BMC[i].shift);
			i++;
		}
	} else {
		seq_printf(s, "\t\n");
		seq_printf(s, "LWTBL DW 14\n");
		addr = (u32 *)&(lwtbl[WF_LWTBL_CIPHER_SUIT_IGTK_DW*4]);
		dw_value = *addr;

		while (WTBL_LMAC_DW14[i].name) {
			if (WTBL_LMAC_DW14[i].shift == NO_SHIFT_DEFINE)
				seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW14[i].name,
					(dw_value & WTBL_LMAC_DW14[i].mask) ? 1 : 0);
			else
				seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW14[i].name,
					(dw_value & WTBL_LMAC_DW14[i].mask) >> WTBL_LMAC_DW14[i].shift);
			i++;
		}
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW28[] = {
	{"RELATED_IDX0",	WF_LWTBL_RELATED_IDX0_MASK,		WF_LWTBL_RELATED_IDX0_SHIFT,		false},
	{"RELATED_BAND0",	WF_LWTBL_RELATED_BAND0_MASK,		WF_LWTBL_RELATED_BAND0_SHIFT,		false},
	{"PRI_MLD_BAND",	WF_LWTBL_PRIMARY_MLD_BAND_MASK,		WF_LWTBL_PRIMARY_MLD_BAND_SHIFT,	true},
	{"RELATED_IDX1",	WF_LWTBL_RELATED_IDX1_MASK,		WF_LWTBL_RELATED_IDX1_SHIFT,		false},
	{"RELATED_BAND1",	WF_LWTBL_RELATED_BAND1_MASK,		WF_LWTBL_RELATED_BAND1_SHIFT,		false},
	{"SEC_MLD_BAND",	WF_LWTBL_SECONDARY_MLD_BAND_MASK,	WF_LWTBL_SECONDARY_MLD_BAND_SHIFT,	true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw28(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 28 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 28\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_MLO_INFO_LINE_1*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW28[i].name) {
		if (WTBL_LMAC_DW28[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW28[i].name,
				(dw_value & WTBL_LMAC_DW28[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW28[i].name,
				(dw_value & WTBL_LMAC_DW28[i].mask) >>
					WTBL_LMAC_DW28[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW29[] = {
	{"DISPATCH_POLICY_MLD_TID0",	WF_LWTBL_DISPATCH_POLICY0_MASK,		WF_LWTBL_DISPATCH_POLICY0_SHIFT,	false},
	{"MLD_TID1",			WF_LWTBL_DISPATCH_POLICY1_MASK,		WF_LWTBL_DISPATCH_POLICY1_SHIFT,	false},
	{"MLD_TID2",			WF_LWTBL_DISPATCH_POLICY2_MASK,		WF_LWTBL_DISPATCH_POLICY2_SHIFT,	false},
	{"MLD_TID3",			WF_LWTBL_DISPATCH_POLICY3_MASK,		WF_LWTBL_DISPATCH_POLICY3_SHIFT,	true},
	{"MLD_TID4",			WF_LWTBL_DISPATCH_POLICY4_MASK,		WF_LWTBL_DISPATCH_POLICY4_SHIFT,	false},
	{"MLD_TID5",			WF_LWTBL_DISPATCH_POLICY5_MASK,		WF_LWTBL_DISPATCH_POLICY5_SHIFT,	false},
	{"MLD_TID6",			WF_LWTBL_DISPATCH_POLICY6_MASK,		WF_LWTBL_DISPATCH_POLICY6_SHIFT,	false},
	{"MLD_TID7",			WF_LWTBL_DISPATCH_POLICY7_MASK,		WF_LWTBL_DISPATCH_POLICY7_SHIFT,	true},
	{"OMLD_ID",			WF_LWTBL_OWN_MLD_ID_MASK,		WF_LWTBL_OWN_MLD_ID_SHIFT,		false},
	{"EMLSR0",			WF_LWTBL_EMLSR0_MASK,			NO_SHIFT_DEFINE,			false},
	{"EMLMR0",			WF_LWTBL_EMLMR0_MASK,			NO_SHIFT_DEFINE,			false},
	{"EMLSR1",			WF_LWTBL_EMLSR1_MASK,			NO_SHIFT_DEFINE,			false},
	{"EMLMR1",			WF_LWTBL_EMLMR1_MASK,			NO_SHIFT_DEFINE,			true},
	{"EMLSR2",			WF_LWTBL_EMLSR2_MASK,			NO_SHIFT_DEFINE,			false},
	{"EMLMR2",			WF_LWTBL_EMLMR2_MASK,			NO_SHIFT_DEFINE,			false},
	{"STR_BITMAP",			WF_LWTBL_STR_BITMAP_MASK,		WF_LWTBL_STR_BITMAP_SHIFT,		true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw29(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 29 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 29\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_MLO_INFO_LINE_2*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW29[i].name) {
		if (WTBL_LMAC_DW29[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW29[i].name,
				(dw_value & WTBL_LMAC_DW29[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW29[i].name,
				(dw_value & WTBL_LMAC_DW29[i].mask) >>
					WTBL_LMAC_DW29[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW30[] = {
	{"DISPATCH_ORDER",	WF_LWTBL_DISPATCH_ORDER_MASK,	WF_LWTBL_DISPATCH_ORDER_SHIFT,	false},
	{"DISPATCH_RATIO",	WF_LWTBL_DISPATCH_RATIO_MASK,	WF_LWTBL_DISPATCH_RATIO_SHIFT,	false},
	{"LINK_MGF",		WF_LWTBL_LINK_MGF_MASK,		WF_LWTBL_LINK_MGF_SHIFT,	true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw30(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 30 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 30\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_MLO_INFO_LINE_3*4]);
	dw_value = *addr;


	while (WTBL_LMAC_DW30[i].name) {
		if (WTBL_LMAC_DW30[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW30[i].name,
				(dw_value & WTBL_LMAC_DW30[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW30[i].name,
				(dw_value & WTBL_LMAC_DW30[i].mask) >> WTBL_LMAC_DW30[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW31[] = {
	{"BFTX_TB",		WF_LWTBL_BFTX_TB_MASK,		NO_SHIFT_DEFINE,		false},
	{"DROP",		WF_LWTBL_DROP_MASK,		NO_SHIFT_DEFINE,		false},
	{"CASCAD",		WF_LWTBL_CASCAD_MASK,		NO_SHIFT_DEFINE,		false},
	{"ALL_ACK",		WF_LWTBL_ALL_ACK_MASK,		NO_SHIFT_DEFINE,		false},
	{"MPDU_SIZE",		WF_LWTBL_MPDU_SIZE_MASK,	WF_LWTBL_MPDU_SIZE_SHIFT,	false},
	{"RXD_DUP_MODE",	WF_LWTBL_RXD_DUP_MODE_MASK,	WF_LWTBL_RXD_DUP_MODE_SHIFT,	true},
	{"ACK_EN",		WF_LWTBL_ACK_EN_MASK,		NO_SHIFT_DEFINE,		true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw31(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 31 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 31\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_RESP_INFO_DW_31*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW31[i].name) {
		if (WTBL_LMAC_DW31[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW31[i].name,
				(dw_value & WTBL_LMAC_DW31[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW31[i].name,
				(dw_value & WTBL_LMAC_DW31[i].mask) >>
					WTBL_LMAC_DW31[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW32[] = {
	{"OM_INFO",		WF_LWTBL_OM_INFO_MASK,			WF_LWTBL_OM_INFO_SHIFT,			false},
	{"OM_INFO_EHT",		WF_LWTBL_OM_INFO_EHT_MASK,		WF_LWTBL_OM_INFO_EHT_SHIFT,		false},
	{"RXD_DUP_FOR_OM_CHG",	WF_LWTBL_RXD_DUP_FOR_OM_CHG_MASK,	NO_SHIFT_DEFINE,			false},
	{"RXD_DUP_WHITE_LIST",	WF_LWTBL_RXD_DUP_WHITE_LIST_MASK,	WF_LWTBL_RXD_DUP_WHITE_LIST_SHIFT,	false},
	{"RXD_DUP_MODE",	WF_LWTBL_RXD_DUP_MODE_MASK,		WF_LWTBL_RXD_DUP_MODE_SHIFT,		false},
	{"ACK_EN",		WF_LWTBL_ACK_EN_MASK,			NO_SHIFT_DEFINE,			true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw32(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 32 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 32\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_RX_DUP_INFO_DW_32*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW32[i].name) {
		if (WTBL_LMAC_DW32[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW32[i].name,
				(dw_value & WTBL_LMAC_DW32[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW32[i].name,
				(dw_value & WTBL_LMAC_DW32[i].mask) >>
					WTBL_LMAC_DW32[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW33[] = {
	{"USER_RSSI",			WF_LWTBL_USER_RSSI_MASK,		WF_LWTBL_USER_RSSI_SHIFT,		false},
	{"USER_SNR",			WF_LWTBL_USER_SNR_MASK,			WF_LWTBL_USER_SNR_SHIFT,		false},
	{"RAPID_REACTION_RATE",		WF_LWTBL_RAPID_REACTION_RATE_MASK,	WF_LWTBL_RAPID_REACTION_RATE_SHIFT,	true},
	{"HT_AMSDU(Read Only)",		WF_LWTBL_HT_AMSDU_MASK,			NO_SHIFT_DEFINE,			false},
	{"AMSDU_CROSS_LG(Read Only)",	WF_LWTBL_AMSDU_CROSS_LG_MASK,		NO_SHIFT_DEFINE,			true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw33(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 33 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 33\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_RX_STAT_CNT_LINE_1*4]);
	dw_value = *addr;

	while (WTBL_LMAC_DW33[i].name) {
		if (WTBL_LMAC_DW33[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW33[i].name,
				(dw_value & WTBL_LMAC_DW33[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW33[i].name,
				(dw_value & WTBL_LMAC_DW33[i].mask) >>
					WTBL_LMAC_DW33[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse WTBL_LMAC_DW34[] = {
	{"RESP_RCPI0",	WF_LWTBL_RESP_RCPI0_MASK,	WF_LWTBL_RESP_RCPI0_SHIFT,	false},
	{"RCPI1",	WF_LWTBL_RESP_RCPI1_MASK,	WF_LWTBL_RESP_RCPI1_SHIFT,	false},
	{"RCPI2",	WF_LWTBL_RESP_RCPI2_MASK,	WF_LWTBL_RESP_RCPI2_SHIFT,	false},
	{"RCPI3",	WF_LWTBL_RESP_RCPI3_MASK,	WF_LWTBL_RESP_RCPI3_SHIFT,	true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw34(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 34 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 34\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_RX_STAT_CNT_LINE_2*4]);
	dw_value = *addr;


	while (WTBL_LMAC_DW34[i].name) {
		if (WTBL_LMAC_DW34[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW34[i].name,
				(dw_value & WTBL_LMAC_DW34[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW34[i].name,
				(dw_value & WTBL_LMAC_DW34[i].mask) >>
					WTBL_LMAC_DW34[i].shift);
		i++;
	}
}

static const struct berse_wtbl_parse *WTBL_LMAC_DW35;
static const struct berse_wtbl_parse WTBL_LMAC_DW35_7996[] = {
	{"SNR 0",	WF_LWTBL_SNR_RX0_MASK,		WF_LWTBL_SNR_RX0_SHIFT,		false},
	{"SNR 1",	WF_LWTBL_SNR_RX1_MASK,		WF_LWTBL_SNR_RX1_SHIFT,		false},
	{"SNR 2",	WF_LWTBL_SNR_RX2_MASK,		WF_LWTBL_SNR_RX2_SHIFT,		false},
	{"SNR 3",	WF_LWTBL_SNR_RX3_MASK,		WF_LWTBL_SNR_RX3_SHIFT,		true},
	{NULL,}
};

static const struct berse_wtbl_parse WTBL_LMAC_DW35_7992[] = {
	{"SNR 0",	WF_LWTBL_SNR_RX0_MASK_7992,	WF_LWTBL_SNR_RX0_SHIFT_7992,	false},
	{"SNR 1",	WF_LWTBL_SNR_RX1_MASK_7992,	WF_LWTBL_SNR_RX1_SHIFT_7992,	false},
	{"SNR 2",	WF_LWTBL_SNR_RX2_MASK_7992,	WF_LWTBL_SNR_RX2_SHIFT_7992,	false},
	{"SNR 3",	WF_LWTBL_SNR_RX3_MASK_7992,	WF_LWTBL_SNR_RX3_SHIFT_7992,	true},
	{NULL,}
};

static void parse_fmac_lwtbl_dw35(struct seq_file *s, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	/* LMAC WTBL DW 35 */
	seq_printf(s, "\t\n");
	seq_printf(s, "LWTBL DW 35\n");
	addr = (u32 *)&(lwtbl[WTBL_GROUP_RX_STAT_CNT_LINE_3*4]);
	dw_value = *addr;


	while (WTBL_LMAC_DW35[i].name) {
		if (WTBL_LMAC_DW35[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_LMAC_DW35[i].name,
				(dw_value & WTBL_LMAC_DW35[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_LMAC_DW35[i].name,
				(dw_value & WTBL_LMAC_DW35[i].mask) >>
					WTBL_LMAC_DW35[i].shift);
		i++;
	}
}

static void parse_fmac_lwtbl_rx_stats(struct seq_file *s, u8 *lwtbl)
{
	parse_fmac_lwtbl_dw33(s, lwtbl);
	parse_fmac_lwtbl_dw34(s, lwtbl);
	parse_fmac_lwtbl_dw35(s, lwtbl);
}

static void parse_fmac_lwtbl_mlo_info(struct seq_file *s, u8 *lwtbl)
{
	parse_fmac_lwtbl_dw28(s, lwtbl);
	parse_fmac_lwtbl_dw29(s, lwtbl);
	parse_fmac_lwtbl_dw30(s, lwtbl);
}

static const struct berse_wtbl_parse WTBL_UMAC_DW9[] = {
	{"RELATED_IDX0",	WF_UWTBL_RELATED_IDX0_MASK,		WF_UWTBL_RELATED_IDX0_SHIFT,		false},
	{"RELATED_BAND0",	WF_UWTBL_RELATED_BAND0_MASK,		WF_UWTBL_RELATED_BAND0_SHIFT,		false},
	{"PRI_MLD_BAND",	WF_UWTBL_PRIMARY_MLD_BAND_MASK,		WF_UWTBL_PRIMARY_MLD_BAND_SHIFT,	true},
	{"RELATED_IDX1",	WF_UWTBL_RELATED_IDX1_MASK,		WF_UWTBL_RELATED_IDX1_SHIFT,		false},
	{"RELATED_BAND1",	WF_UWTBL_RELATED_BAND1_MASK,		WF_UWTBL_RELATED_BAND1_SHIFT,		false},
	{"SEC_MLD_BAND",	WF_UWTBL_SECONDARY_MLD_BAND_MASK,	WF_UWTBL_SECONDARY_MLD_BAND_SHIFT,	true},
	{NULL,}
};

static void parse_fmac_uwtbl_mlo_info(struct seq_file *s, u8 *uwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	seq_printf(s, "\t\n");
	seq_printf(s, "MldAddr: %02x:%02x:%02x:%02x:%02x:%02x(D0[B0~15], D1[B0~31])\n",
		uwtbl[4], uwtbl[5], uwtbl[6], uwtbl[7], uwtbl[0], uwtbl[1]);

	/* UMAC WTBL DW 0 */
	seq_printf(s, "\t\n");
	seq_printf(s, "UWTBL DW 0\n");
	addr = (u32 *)&(uwtbl[WF_UWTBL_OWN_MLD_ID_DW*4]);
	dw_value = *addr;

	seq_printf(s, "\t%s:%u\n", "OMLD_ID",
		(dw_value & WF_UWTBL_OWN_MLD_ID_MASK) >> WF_UWTBL_OWN_MLD_ID_SHIFT);

	/* UMAC WTBL DW 9 */
	seq_printf(s, "\t\n");
	seq_printf(s, "UWTBL DW 9\n");
	addr = (u32 *)&(uwtbl[WF_UWTBL_RELATED_IDX0_DW*4]);
	dw_value = *addr;

	while (WTBL_UMAC_DW9[i].name) {

		if (WTBL_UMAC_DW9[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_UMAC_DW9[i].name,
				(dw_value & WTBL_UMAC_DW9[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_UMAC_DW9[i].name,
				 (dw_value & WTBL_UMAC_DW9[i].mask) >>
					WTBL_UMAC_DW9[i].shift);
		i++;
	}
}

static bool
is_wtbl_bigtk_exist(u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;

	addr = (u32 *)&(lwtbl[WF_LWTBL_MUAR_DW*4]);
	dw_value = *addr;
	if (((dw_value & WF_LWTBL_MUAR_MASK) >> WF_LWTBL_MUAR_SHIFT) ==
					MUAR_INDEX_OWN_MAC_ADDR_BC_MC) {
		addr = (u32 *)&(lwtbl[WF_LWTBL_CIPHER_SUIT_BIGTK_DW*4]);
		dw_value = *addr;
		if (((dw_value & WF_LWTBL_CIPHER_SUIT_BIGTK_MASK) >>
			WF_LWTBL_CIPHER_SUIT_BIGTK_SHIFT) != IGTK_CIPHER_SUIT_NONE)
			return true;
	}

	return false;
}

static const struct berse_wtbl_parse WTBL_UMAC_DW2[] = {
	{"PN0",		WTBL_PN0_MASK,		WTBL_PN0_OFFSET,	false},
	{"PN1",		WTBL_PN1_MASK,		WTBL_PN1_OFFSET,	false},
	{"PN2",		WTBL_PN2_MASK,		WTBL_PN2_OFFSET,	true},
	{"PN3",		WTBL_PN3_MASK,		WTBL_PN3_OFFSET,	false},
	{NULL,}
};

static const struct berse_wtbl_parse WTBL_UMAC_DW3[] = {
	{"PN4",		WTBL_PN4_MASK,		WTBL_PN4_OFFSET,	false},
	{"PN5",		WTBL_PN5_MASK,		WTBL_PN5_OFFSET,	true},
	{"COM_SN",	WF_UWTBL_COM_SN_MASK,	WF_UWTBL_COM_SN_SHIFT,	true},
	{NULL,}
};

static const struct berse_wtbl_parse WTBL_UMAC_DW4_BIPN[] = {
	{"BIPN0",	WTBL_BIPN0_MASK,	WTBL_BIPN0_OFFSET,	false},
	{"BIPN1",	WTBL_BIPN1_MASK,	WTBL_BIPN1_OFFSET,	false},
	{"BIPN2",	WTBL_BIPN2_MASK,	WTBL_BIPN2_OFFSET,	true},
	{"BIPN3",	WTBL_BIPN3_MASK,	WTBL_BIPN3_OFFSET,	false},
	{NULL,}
};

static const struct berse_wtbl_parse WTBL_UMAC_DW5_BIPN[] = {
	{"BIPN4",	WTBL_BIPN4_MASK,	WTBL_BIPN4_OFFSET,	false},
	{"BIPN5",	WTBL_BIPN5_MASK,	WTBL_BIPN5_OFFSET,	true},
	{NULL,}
};

static void parse_fmac_uwtbl_pn(struct seq_file *s, u8 *uwtbl, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u16 i = 0;

	seq_printf(s, "\t\n");
	seq_printf(s, "UWTBL PN\n");

	/* UMAC WTBL DW 2/3 */
	addr = (u32 *)&(uwtbl[WF_UWTBL_PN_31_0__DW*4]);
	dw_value = *addr;

	while (WTBL_UMAC_DW2[i].name) {
		seq_printf(s, "\t%s:%u\n", WTBL_UMAC_DW2[i].name,
			(dw_value & WTBL_UMAC_DW2[i].mask) >>
				WTBL_UMAC_DW2[i].shift);
		i++;
	}

	i = 0;
	addr = (u32 *)&(uwtbl[WF_UWTBL_PN_47_32__DW*4]);
	dw_value = *addr;

	while (WTBL_UMAC_DW3[i].name) {
		seq_printf(s, "\t%s:%u\n", WTBL_UMAC_DW3[i].name,
			 (dw_value & WTBL_UMAC_DW3[i].mask) >>
			WTBL_UMAC_DW3[i].shift);
		i++;
	}


	/* UMAC WTBL DW 4/5 for BIGTK */
	if (is_wtbl_bigtk_exist(lwtbl) == true) {
		i = 0;
		addr = (u32 *)&(uwtbl[WF_UWTBL_RX_BIPN_31_0__DW*4]);
		dw_value = *addr;

		while (WTBL_UMAC_DW4_BIPN[i].name) {
			seq_printf(s, "\t%s:%u\n", WTBL_UMAC_DW4_BIPN[i].name,
				(dw_value & WTBL_UMAC_DW4_BIPN[i].mask) >>
					WTBL_UMAC_DW4_BIPN[i].shift);
			i++;
		}

		i = 0;
		addr = (u32 *)&(uwtbl[WF_UWTBL_RX_BIPN_47_32__DW*4]);
		dw_value = *addr;

		while (WTBL_UMAC_DW5_BIPN[i].name) {
			seq_printf(s, "\t%s:%u\n", WTBL_UMAC_DW5_BIPN[i].name,
				(dw_value & WTBL_UMAC_DW5_BIPN[i].mask) >>
				WTBL_UMAC_DW5_BIPN[i].shift);
			i++;
		}
	}
}

static void parse_fmac_uwtbl_sn(struct seq_file *s, u8 *uwtbl)
{
	u32 *addr = 0;
	u32 u2SN = 0;

	/* UMAC WTBL DW SN part */
	seq_printf(s, "\t\n");
	seq_printf(s, "UWTBL SN\n");

	addr = (u32 *)&(uwtbl[WF_UWTBL_TID0_SN_DW*4]);
	u2SN = ((*addr) & WF_UWTBL_TID0_SN_MASK) >> WF_UWTBL_TID0_SN_SHIFT;
	seq_printf(s, "\t%s:%u\n", "TID0_AC0_SN", u2SN);

	addr = (u32 *)&(uwtbl[WF_UWTBL_TID1_SN_DW*4]);
	u2SN = ((*addr) & WF_UWTBL_TID1_SN_MASK) >> WF_UWTBL_TID1_SN_SHIFT;
	seq_printf(s, "\t%s:%u\n", "TID1_AC1_SN", u2SN);

	addr = (u32 *)&(uwtbl[WF_UWTBL_TID2_SN_7_0__DW*4]);
	u2SN = ((*addr) & WF_UWTBL_TID2_SN_7_0__MASK) >>
				WF_UWTBL_TID2_SN_7_0__SHIFT;
	addr = (u32 *)&(uwtbl[WF_UWTBL_TID2_SN_11_8__DW*4]);
	u2SN |= (((*addr) & WF_UWTBL_TID2_SN_11_8__MASK) >>
			WF_UWTBL_TID2_SN_11_8__SHIFT) << 8;
	seq_printf(s, "\t%s:%u\n", "TID2_AC2_SN", u2SN);

	addr = (u32 *)&(uwtbl[WF_UWTBL_TID3_SN_DW*4]);
	u2SN = ((*addr) & WF_UWTBL_TID3_SN_MASK) >> WF_UWTBL_TID3_SN_SHIFT;
	seq_printf(s, "\t%s:%u\n", "TID3_AC3_SN", u2SN);

	addr = (u32 *)&(uwtbl[WF_UWTBL_TID4_SN_DW*4]);
	u2SN = ((*addr) & WF_UWTBL_TID4_SN_MASK) >> WF_UWTBL_TID4_SN_SHIFT;
	seq_printf(s, "\t%s:%u\n", "TID4_SN", u2SN);

	addr = (u32 *)&(uwtbl[WF_UWTBL_TID5_SN_3_0__DW*4]);
	u2SN = ((*addr) & WF_UWTBL_TID5_SN_3_0__MASK) >>
				WF_UWTBL_TID5_SN_3_0__SHIFT;
	addr = (u32 *)&(uwtbl[WF_UWTBL_TID5_SN_11_4__DW*4]);
	u2SN |= (((*addr) & WF_UWTBL_TID5_SN_11_4__MASK) >>
				WF_UWTBL_TID5_SN_11_4__SHIFT) << 4;
	seq_printf(s, "\t%s:%u\n", "TID5_SN", u2SN);

	addr = (u32 *)&(uwtbl[WF_UWTBL_TID6_SN_DW*4]);
	u2SN = ((*addr) & WF_UWTBL_TID6_SN_MASK) >> WF_UWTBL_TID6_SN_SHIFT;
	seq_printf(s, "\t%s:%u\n", "TID6_SN", u2SN);

	addr = (u32 *)&(uwtbl[WF_UWTBL_TID7_SN_DW*4]);
	u2SN = ((*addr) & WF_UWTBL_TID7_SN_MASK) >> WF_UWTBL_TID7_SN_SHIFT;
	seq_printf(s, "\t%s:%u\n", "TID7_SN", u2SN);

	addr = (u32 *)&(uwtbl[WF_UWTBL_COM_SN_DW*4]);
	u2SN = ((*addr) & WF_UWTBL_COM_SN_MASK) >> WF_UWTBL_COM_SN_SHIFT;
	seq_printf(s, "\t%s:%u\n", "COM_SN", u2SN);
}

static void dump_key_table(
	struct seq_file *s,
	uint16_t keyloc0,
	uint16_t keyloc1,
	uint16_t keyloc2
)
{
#define ONE_KEY_ENTRY_LEN_IN_DW                8
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	u8 keytbl[ONE_KEY_ENTRY_LEN_IN_DW*4] = {0};
	uint16_t x;

	seq_printf(s, "\t\n");
	seq_printf(s, "\t%s:%d\n", "keyloc0", keyloc0);
	if (keyloc0 != INVALID_KEY_ENTRY) {
		/* Don't swap below two lines, halWtblReadRaw will
		* write new value WF_WTBLON_TOP_WDUCR_ADDR
		*/
		mt7996_wtbl_read_raw(dev, keyloc0,
			WTBL_TYPE_KEY, 0, ONE_KEY_ENTRY_LEN_IN_DW, keytbl);
		if (is_mt7990(&dev->mt76))
			seq_printf(s, "\t\tKEY WTBL Addr: group:0x%x=0x%x addr: 0x%lx\n",
				MT_DBG_UWTBL_TOP_KDUCR_ADDR,
				mt76_rr(dev, MT_DBG_UWTBL_TOP_KDUCR_ADDR),
				KEYTBL_IDX2BASE_7990(keyloc0, 0));
		else
			seq_printf(s, "\t\tKEY WTBL Addr: group:0x%x=0x%x addr: 0x%lx\n",
				MT_DBG_UWTBL_TOP_WDUCR_ADDR,
				mt76_rr(dev, MT_DBG_UWTBL_TOP_WDUCR_ADDR),
				KEYTBL_IDX2BASE(keyloc0, 0));
		for (x = 0; x < ONE_KEY_ENTRY_LEN_IN_DW; x++) {
			seq_printf(s, "\t\tDW%02d: %02x %02x %02x %02x\n",
				x,
				keytbl[x * 4 + 3],
				keytbl[x * 4 + 2],
				keytbl[x * 4 + 1],
				keytbl[x * 4]);
		}
	}

	seq_printf(s, "\t%s:%d\n", "keyloc1", keyloc1);
	if (keyloc1 != INVALID_KEY_ENTRY) {
		/* Don't swap below two lines, halWtblReadRaw will
		* write new value WF_WTBLON_TOP_WDUCR_ADDR
		*/
		mt7996_wtbl_read_raw(dev, keyloc1,
			WTBL_TYPE_KEY, 0, ONE_KEY_ENTRY_LEN_IN_DW, keytbl);
		if (is_mt7990(&dev->mt76))
			seq_printf(s, "\t\tKEY WTBL Addr: group:0x%x=0x%x addr: 0x%lx\n",
				MT_DBG_UWTBL_TOP_KDUCR_ADDR,
				mt76_rr(dev, MT_DBG_UWTBL_TOP_KDUCR_ADDR),
				KEYTBL_IDX2BASE_7990(keyloc1, 0));
		else
			seq_printf(s, "\t\tKEY WTBL Addr: group:0x%x=0x%x addr: 0x%lx\n",
				MT_DBG_UWTBL_TOP_WDUCR_ADDR,
				mt76_rr(dev, MT_DBG_UWTBL_TOP_WDUCR_ADDR),
				KEYTBL_IDX2BASE(keyloc1, 0));
		for (x = 0; x < ONE_KEY_ENTRY_LEN_IN_DW; x++) {
			seq_printf(s, "\t\tDW%02d: %02x %02x %02x %02x\n",
				x,
				keytbl[x * 4 + 3],
				keytbl[x * 4 + 2],
				keytbl[x * 4 + 1],
				keytbl[x * 4]);
		}
	}

	seq_printf(s, "\t%s:%d\n", "keyloc2", keyloc2);
	if (keyloc2 != INVALID_KEY_ENTRY) {
		/* Don't swap below two lines, halWtblReadRaw will
		* write new value WF_WTBLON_TOP_WDUCR_ADDR
		*/
		mt7996_wtbl_read_raw(dev, keyloc2,
			WTBL_TYPE_KEY, 0, ONE_KEY_ENTRY_LEN_IN_DW, keytbl);
		if (is_mt7990(&dev->mt76))
			seq_printf(s, "\t\tKEY WTBL Addr: group:0x%x=0x%x addr: 0x%lx\n",
				MT_DBG_UWTBL_TOP_KDUCR_ADDR,
				mt76_rr(dev, MT_DBG_UWTBL_TOP_KDUCR_ADDR),
				KEYTBL_IDX2BASE_7990(keyloc2, 0));
		else
			seq_printf(s, "\t\tKEY WTBL Addr: group:0x%x=0x%x addr: 0x%lx\n",
				MT_DBG_UWTBL_TOP_WDUCR_ADDR,
				mt76_rr(dev, MT_DBG_UWTBL_TOP_WDUCR_ADDR),
				KEYTBL_IDX2BASE(keyloc2, 0));
		for (x = 0; x < ONE_KEY_ENTRY_LEN_IN_DW; x++) {
			seq_printf(s, "\t\tDW%02d: %02x %02x %02x %02x\n",
				x,
				keytbl[x * 4 + 3],
				keytbl[x * 4 + 2],
				keytbl[x * 4 + 1],
				keytbl[x * 4]);
		}
	}
}

static void parse_fmac_uwtbl_key_info(struct seq_file *s, u8 *uwtbl, u8 *lwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	uint16_t keyloc0 = INVALID_KEY_ENTRY;
	uint16_t keyloc1 = INVALID_KEY_ENTRY;
	uint16_t keyloc2 = INVALID_KEY_ENTRY;

	/* UMAC WTBL DW 7 */
	seq_printf(s, "\t\n");
	seq_printf(s, "UWTBL key info\n");

	addr = (u32 *)&(uwtbl[WF_UWTBL_KEY_LOC0_DW*4]);
	dw_value = *addr;
	keyloc0 = (dw_value & WF_UWTBL_KEY_LOC0_MASK) >> WF_UWTBL_KEY_LOC0_SHIFT;
	keyloc1 = (dw_value & WF_UWTBL_KEY_LOC1_MASK) >> WF_UWTBL_KEY_LOC1_SHIFT;

	seq_printf(s, "\t%s:%u/%u\n", "Key Loc 0/1", keyloc0, keyloc1);

	/* UMAC WTBL DW 6 for BIGTK */
	if (is_wtbl_bigtk_exist(lwtbl) == true) {
		addr = (u32 *)&(uwtbl[WF_UWTBL_KEY_LOC2_DW*4]);
		dw_value = *addr;
		keyloc2 = (dw_value & WF_UWTBL_KEY_LOC2_MASK) >>
			WF_UWTBL_KEY_LOC2_SHIFT;
		seq_printf(s, "\t%s:%u\n", "Key Loc 2", keyloc2);
	}

	/* Parse KEY link */
	dump_key_table(s, keyloc0, keyloc1, keyloc2);
}

static const struct berse_wtbl_parse WTBL_UMAC_DW8[] = {
	{"UWTBL_WMM_Q",		WF_UWTBL_WMM_Q_MASK,		WF_UWTBL_WMM_Q_SHIFT,	false},
	{"UWTBL_QOS",		WF_UWTBL_QOS_MASK,		NO_SHIFT_DEFINE,	false},
	{"UWTBL_HT_VHT_HE",	WF_UWTBL_HT_MASK,		NO_SHIFT_DEFINE,	false},
	{"UWTBL_HDRT_MODE",	WF_UWTBL_HDRT_MODE_MASK,	NO_SHIFT_DEFINE,	true},
	{"UWTBL_AAD_OM",	WF_UWTBL_AAD_OM_MASK,		WF_UWTBL_AAD_OM_SHIFT,	true},
	{NULL,}
};

static void parse_fmac_uwtbl_msdu_info(struct seq_file *s, u8 *uwtbl)
{
	u32 *addr = 0;
	u32 dw_value = 0;
	u32 amsdu_len = 0;
	u16 i = 0;

	/* UMAC WTBL DW 8 */
	seq_printf(s, "\t\n");
	seq_printf(s, "UWTBL DW8\n");

	addr = (u32 *)&(uwtbl[WF_UWTBL_AMSDU_CFG_DW*4]);
	dw_value = *addr;

	while (WTBL_UMAC_DW8[i].name) {

		if (WTBL_UMAC_DW8[i].shift == NO_SHIFT_DEFINE)
			seq_printf(s, "\t%s:%d\n", WTBL_UMAC_DW8[i].name,
				(dw_value & WTBL_UMAC_DW8[i].mask) ? 1 : 0);
		else
			seq_printf(s, "\t%s:%u\n", WTBL_UMAC_DW8[i].name,
				(dw_value & WTBL_UMAC_DW8[i].mask) >>
					WTBL_UMAC_DW8[i].shift);
		i++;
	}

	/* UMAC WTBL DW 8 - SEC_ADDR_MODE */
	addr = (u32 *)&(uwtbl[WF_UWTBL_SEC_ADDR_MODE_DW*4]);
	dw_value = *addr;
	seq_printf(s, "\t%s:%lu\n", "SEC_ADDR_MODE",
		(dw_value & WTBL_SEC_ADDR_MODE_MASK) >> WTBL_SEC_ADDR_MODE_OFFSET);

	/* UMAC WTBL DW 8 - AMSDU_CFG */
	seq_printf(s, "\t%s:%d\n", "HW AMSDU Enable",
				(dw_value & WTBL_AMSDU_EN_MASK) ? 1 : 0);

	amsdu_len = (dw_value & WTBL_AMSDU_LEN_MASK) >> WTBL_AMSDU_LEN_OFFSET;
	if (amsdu_len == 0)
		seq_printf(s, "\t%s:invalid (WTBL value=0x%x)\n", "HW AMSDU Len",
			amsdu_len);
	else if (amsdu_len == 1)
		seq_printf(s, "\t%s:%d~%d (WTBL value=0x%x)\n", "HW AMSDU Len",
			1,
			255,
			amsdu_len);
	else if (amsdu_len == 2)
		seq_printf(s, "\t%s:%d~%d (WTBL value=0x%x)\n", "HW AMSDU Len",
			256,
			511,
			amsdu_len);
	else if (amsdu_len == 3)
		seq_printf(s, "\t%s:%d~%d (WTBL value=0x%x)\n", "HW AMSDU Len",
			512,
			767,
			amsdu_len);
	else
		seq_printf(s, "\t%s:%d~%d (WTBL value=0x%x)\n", "HW AMSDU Len",
			256 * (amsdu_len - 1),
			256 * (amsdu_len - 1) + 255,
			amsdu_len);

	seq_printf(s, "\t%s:%lu (WTBL value=0x%lx)\n", "HW AMSDU Num",
		((dw_value & WTBL_AMSDU_NUM_MASK) >> WTBL_AMSDU_NUM_OFFSET) + 1,
		(dw_value & WTBL_AMSDU_NUM_MASK) >> WTBL_AMSDU_NUM_OFFSET);
}

static int mt7996_wtbl_read(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	u8 lwtbl[LWTBL_LEN_IN_DW * 4] = {0};
	u8 uwtbl[UWTBL_LEN_IN_DW * 4] = {0};
	int x;

	mt7996_wtbl_read_raw(dev, dev->wlan_idx, WTBL_TYPE_LMAC, 0,
				 LWTBL_LEN_IN_DW, lwtbl);
	seq_printf(s, "Dump WTBL info of WLAN_IDX:%d\n", dev->wlan_idx);
	seq_printf(s, "LMAC WTBL Addr: group:0x%x=0x%x addr: 0x%lx\n",
		   MT_WTBLON_TOP_WDUCR,
		   mt76_rr(dev, MT_WTBLON_TOP_WDUCR),
		   LWTBL_IDX2BASE(dev->wlan_idx, 0));
	for (x = 0; x < LWTBL_LEN_IN_DW; x++) {
		seq_printf(s, "DW%02d: %02x %02x %02x %02x\n",
			   x,
			   lwtbl[x * 4 + 3],
			   lwtbl[x * 4 + 2],
			   lwtbl[x * 4 + 1],
			   lwtbl[x * 4]);
	}

	/* Parse LWTBL */
	parse_fmac_lwtbl_dw0_1(s, lwtbl);
	parse_fmac_lwtbl_dw2(s, lwtbl);
	parse_fmac_lwtbl_dw3(s, lwtbl);
	parse_fmac_lwtbl_dw4(s, lwtbl);
	parse_fmac_lwtbl_dw5(s, lwtbl);
	parse_fmac_lwtbl_dw6(s, lwtbl);
	parse_fmac_lwtbl_dw7(s, lwtbl);
	parse_fmac_lwtbl_dw8(s, lwtbl);
	parse_fmac_lwtbl_dw9(s, lwtbl);
	parse_fmac_lwtbl_dw10(s, lwtbl);
	parse_fmac_lwtbl_dw11(s, lwtbl);
	parse_fmac_lwtbl_dw12(s, lwtbl);
	parse_fmac_lwtbl_dw13(s, lwtbl);
	parse_fmac_lwtbl_dw14(s, lwtbl);
	parse_fmac_lwtbl_mlo_info(s, lwtbl);
	parse_fmac_lwtbl_dw31(s, lwtbl);
	parse_fmac_lwtbl_dw32(s, lwtbl);
	parse_fmac_lwtbl_rx_stats(s, lwtbl);

	mt7996_wtbl_read_raw(dev, dev->wlan_idx, WTBL_TYPE_UMAC, 0,
				 UWTBL_LEN_IN_DW, uwtbl);
	seq_printf(s, "Dump WTBL info of WLAN_IDX:%d\n", dev->wlan_idx);
	seq_printf(s, "UMAC WTBL Addr: group:0x%x=0x%x addr: 0x%lx\n",
		   MT_DBG_UWTBL_TOP_WDUCR_ADDR,
		   mt76_rr(dev, MT_DBG_UWTBL_TOP_WDUCR_ADDR),
		   UWTBL_IDX2BASE(dev->wlan_idx, 0));
	for (x = 0; x < UWTBL_LEN_IN_DW; x++) {
		seq_printf(s, "DW%02d: %02x %02x %02x %02x\n",
			   x,
			   uwtbl[x * 4 + 3],
			   uwtbl[x * 4 + 2],
			   uwtbl[x * 4 + 1],
			   uwtbl[x * 4]);
	}

	/* Parse UWTBL */
	parse_fmac_uwtbl_mlo_info(s, uwtbl);
	parse_fmac_uwtbl_pn(s, uwtbl, lwtbl);
	parse_fmac_uwtbl_sn(s, uwtbl);
	parse_fmac_uwtbl_key_info(s, uwtbl, lwtbl);
	parse_fmac_uwtbl_msdu_info(s, uwtbl);

	return 0;
}

static int mt7996_sta_info(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	u8 lwtbl[LWTBL_LEN_IN_DW*4] = {0};
	u16 i = 0;

	for (i=0; i < mt7996_wtbl_size(dev); i++) {
		mt7996_wtbl_read_raw(dev, i, WTBL_TYPE_LMAC, 0,
				     LWTBL_LEN_IN_DW, lwtbl);

		if (lwtbl[4] || lwtbl[5] || lwtbl[6] || lwtbl[7] || lwtbl[0] || lwtbl[1]) {
			u32 *addr, dw_value;

			seq_printf(s, "wcid:%d\tAddr: %02x:%02x:%02x:%02x:%02x:%02x",
					i, lwtbl[4], lwtbl[5], lwtbl[6], lwtbl[7], lwtbl[0], lwtbl[1]);

			addr = (u32 *)&(lwtbl[WTBL_GROUP_TRX_CAP_DW_2*4]);
			dw_value = *addr;
			seq_printf(s, "\t%s:%u", WTBL_LMAC_DW2[0].name,
					(dw_value & WTBL_LMAC_DW2[0].mask) >> WTBL_LMAC_DW2[0].shift);

			addr = (u32 *)&(lwtbl[WTBL_GROUP_TRX_CAP_DW_5*4]);
			dw_value = *addr;
			seq_printf(s, "\tPSM:%u\n", !!(dw_value & WF_LWTBL_PSM_MASK));
		}
	}

	return 0;
}

static int mt7996_token_read(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	int msdu_id, i;
	struct mt76_txwi_cache *txwi;

	seq_printf(s, "Token from host:\n");
	spin_lock_bh(&dev->mt76.token_lock);
	idr_for_each_entry(&dev->mt76.token, txwi, msdu_id) {
		seq_printf(s, "%4d (wcid = %4d, pending time %u ms)\n",
			   msdu_id, txwi->wcid,
			   jiffies_to_msecs(jiffies - txwi->jiffies));
	}
	seq_printf(s, "\n");
	for (i = 0; i < __MT_MAX_BAND; i++) {
		struct mt76_phy *phy = mt76_dev_phy(&dev->mt76, i);

		if (!mt7996_band_valid(dev, i))
			continue;

		seq_printf(s, "Band%u consume: %d, free: %d total: %d\n",
			   i, phy->tokens, dev->mt76.token_threshold - phy->tokens,
			   dev->mt76.token_threshold);
	}
	spin_unlock_bh(&dev->mt76.token_lock);

	return 0;
}

static int
mt7996_scs_enable_set(void *data, u64 val)
{
	struct mt7996_phy *phy = data;

	return mt7996_mcu_set_scs(phy, (u8) val);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_scs_enable, NULL,
			 mt7996_scs_enable_set, "%lld\n");

static int
mt7996_txpower_level_set(void *data, u64 val)
{
	struct mt7996_phy *phy = data;
	int ret;

	if (val > 100)
		return -EINVAL;

	ret = mt7996_mcu_set_tx_power_ctrl(phy, UNI_TXPOWER_PERCENTAGE_CTRL, !!val);
	if (ret)
		return ret;

	return mt7996_mcu_set_tx_power_ctrl(phy, UNI_TXPOWER_PERCENTAGE_DROP_CTRL, val);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_txpower_level, NULL,
			 mt7996_txpower_level_set, "%lld\n");

static ssize_t
mt7996_get_txpower_info(struct file *file, char __user *user_buf,
			size_t count, loff_t *ppos)
{
	struct mt7996_phy *phy = file->private_data;
	struct mt7996_mcu_txpower_event *event;
	struct txpower_basic_info *basic_info;
	struct device_node *np;
	static const size_t size = 2048;
	int len = 0;
	ssize_t ret;
	char *buf;

	buf = kzalloc(size, GFP_KERNEL);
	event = kzalloc(sizeof(*event), GFP_KERNEL);
	if (!buf || !event) {
		ret = -ENOMEM;
		goto out;
	}

	ret = mt7996_mcu_get_tx_power_info(phy, BASIC_INFO, event);
	if (ret ||
	    le32_to_cpu(event->basic_info.category) != UNI_TXPOWER_BASIC_INFO)
		goto out;

	basic_info = &event->basic_info;

	len += scnprintf(buf + len, size - len,
			 "======================== BASIC INFO ========================\n");
	len += scnprintf(buf + len, size - len, "    Band Index: %d, Channel Band: %d\n",
			 basic_info->band_idx, basic_info->band);
	len += scnprintf(buf + len, size - len, "    PA Type: %s\n",
			 basic_info->is_epa ? "ePA" : "iPA");
	len += scnprintf(buf + len, size - len, "    LNA Type: %s\n",
			 basic_info->is_elna ? "eLNA" : "iLNA");

	len += scnprintf(buf + len, size - len,
			 "------------------------------------------------------------\n");
	len += scnprintf(buf + len, size - len, "    SKU: %s\n",
			 basic_info->sku_enable ? "enable" : "disable");
	len += scnprintf(buf + len, size - len, "    Percentage Control: %s\n",
			 basic_info->percentage_ctrl_enable ? "enable" : "disable");
	len += scnprintf(buf + len, size - len, "    Power Drop: %d [dBm]\n",
			 basic_info->power_drop_level >> 1);
	len += scnprintf(buf + len, size - len, "    Backoff: %s\n",
			 basic_info->bf_backoff_enable ? "enable" : "disable");
	len += scnprintf(buf + len, size - len, "    TX Front-end Loss:  %d, %d, %d, %d\n",
			 basic_info->front_end_loss_tx[0], basic_info->front_end_loss_tx[1],
			 basic_info->front_end_loss_tx[2], basic_info->front_end_loss_tx[3]);
	len += scnprintf(buf + len, size - len, "    RX Front-end Loss:  %d, %d, %d, %d\n",
			 basic_info->front_end_loss_rx[0], basic_info->front_end_loss_rx[1],
			 basic_info->front_end_loss_rx[2], basic_info->front_end_loss_rx[3]);
	len += scnprintf(buf + len, size - len,
			 "    MU TX Power Mode:  %s\n",
			 basic_info->mu_tx_power_manual_enable ? "manual" : "auto");
	len += scnprintf(buf + len, size - len,
			 "    MU TX Power (Auto / Manual): %d / %d [0.5 dBm]\n",
			 basic_info->mu_tx_power_auto, basic_info->mu_tx_power_manual);
	len += scnprintf(buf + len, size - len,
			 "    Thermal Compensation:  %s\n",
			 basic_info->thermal_compensate_enable ? "enable" : "disable");
	len += scnprintf(buf + len, size - len,
			 "    Theraml Compensation Value: %d\n",
			 basic_info->thermal_compensate_value);
	np = mt76_find_power_limits_node(phy->mt76);
	len += scnprintf(buf + len, size - len,
			 "    RegDB:  %s\n",
			 !np ? "enable" : "disable");
	len += scnprintf(buf + len, size - len,
			 "    sku_index:  %d\n", phy->mt76->sku_idx);
	len += scnprintf(buf + len, size - len,
			 "    lpi:  %s\n",
			 phy->mt76->dev->lpi_mode ? "enable" : "disable");
	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

out:
	kfree(buf);
	kfree(event);
	return ret;
}

static const struct file_operations mt7996_txpower_info_fops = {
	.read = mt7996_get_txpower_info,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

#define mt7996_txpower_puts(rate, _len)							\
({											\
	len += scnprintf(buf + len, size - len, "%-*s:", _len, #rate " (TMAC)");	\
	for (i = 0; i < mt7996_sku_group_len[SKU_##rate]; i++, offs++)			\
		len += scnprintf(buf + len, size - len, " %6d",				\
				 event->phy_rate_info.frame_power[offs][band_idx]);	\
	len += scnprintf(buf + len, size - len, "\n");					\
})

static ssize_t
mt7996_get_txpower_sku(struct file *file, char __user *user_buf,
		       size_t count, loff_t *ppos)
{
	struct mt7996_phy *phy = file->private_data;
	struct mt7996_dev *dev = phy->dev;
	struct mt7996_mcu_txpower_event *event;
	struct ieee80211_channel *chan = phy->mt76->chandef.chan;
	struct ieee80211_supported_band sband;
	u8 band_idx = phy->mt76->band_idx;
	static const size_t size = 5120;
	int i, offs = 0, len = 0;
	u32 target_power = 0;
	int n_chains = hweight16(phy->mt76->chainmask);
	int path_delta = mt76_tx_power_path_delta(n_chains);
	int pwr_delta;
	ssize_t ret;
	char *buf;
	u32 reg;

	buf = kzalloc(size, GFP_KERNEL);
	event = kzalloc(sizeof(*event), GFP_KERNEL);
	if (!buf || !event) {
		ret = -ENOMEM;
		goto out;
	}

	ret = mt7996_mcu_get_tx_power_info(phy, PHY_RATE_INFO, event);
	if (ret ||
	    le32_to_cpu(event->phy_rate_info.category) != UNI_TXPOWER_PHY_RATE_INFO)
		goto out;

	len += scnprintf(buf + len, size - len,
			 "\nPhy %d TX Power Table (Channel %d)\n",
			 band_idx, phy->mt76->chandef.chan->hw_value);
	len += scnprintf(buf + len, size - len, "%-21s  %6s %6s %6s %6s\n",
			 " ", "1m", "2m", "5m", "11m");
	mt7996_txpower_puts(CCK, 21);

	len += scnprintf(buf + len, size - len,
			 "%-21s  %6s %6s %6s %6s %6s %6s %6s %6s\n",
			 " ", "6m", "9m", "12m", "18m", "24m", "36m", "48m",
			 "54m");
	mt7996_txpower_puts(OFDM, 21);

	len += scnprintf(buf + len, size - len,
			 "%-21s  %6s %6s %6s %6s %6s %6s %6s %6s\n",
			 " ", "mcs0", "mcs1", "mcs2", "mcs3", "mcs4",
			 "mcs5", "mcs6", "mcs7");
	mt7996_txpower_puts(HT20, 21);

	len += scnprintf(buf + len, size - len,
			 "%-21s  %6s %6s %6s %6s %6s %6s %6s %6s %6s\n",
			 " ", "mcs0", "mcs1", "mcs2", "mcs3", "mcs4", "mcs5",
			 "mcs6", "mcs7", "mcs32");
	mt7996_txpower_puts(HT40, 21);

	len += scnprintf(buf + len, size - len,
			 "%-21s  %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s %6s\n",
			 " ", "mcs0", "mcs1", "mcs2", "mcs3", "mcs4", "mcs5",
			 "mcs6", "mcs7", "mcs8", "mcs9", "mcs10", "mcs11");
	mt7996_txpower_puts(VHT20, 21);
	mt7996_txpower_puts(VHT40, 21);
	mt7996_txpower_puts(VHT80, 21);
	mt7996_txpower_puts(VHT160, 21);
	mt7996_txpower_puts(HE26, 21);
	mt7996_txpower_puts(HE52, 21);
	mt7996_txpower_puts(HE106, 21);
	len += scnprintf(buf + len, size - len, "BW20/");
	mt7996_txpower_puts(HE242, 16);
	len += scnprintf(buf + len, size - len, "BW40/");
	mt7996_txpower_puts(HE484, 16);
	len += scnprintf(buf + len, size - len, "BW80/");
	mt7996_txpower_puts(HE996, 16);
	len += scnprintf(buf + len, size - len, "BW160/");
	mt7996_txpower_puts(HE2x996, 15);

	len += scnprintf(buf + len, size - len,
			 "%-21s  %6s %6s %6s %6s %6s %6s %6s %6s ",
			 " ", "mcs0", "mcs1", "mcs2", "mcs3", "mcs4", "mcs5", "mcs6", "mcs7");
	len += scnprintf(buf + len, size - len,
			 "%6s %6s %6s %6s %6s %6s %6s %6s\n",
			 "mcs8", "mcs9", "mcs10", "mcs11", "mcs12", "mcs13", "mcs14", "mcs15");
	mt7996_txpower_puts(EHT26, 21);
	mt7996_txpower_puts(EHT52, 21);
	mt7996_txpower_puts(EHT106, 21);
	len += scnprintf(buf + len, size - len, "BW20/");
	mt7996_txpower_puts(EHT242, 16);
	len += scnprintf(buf + len, size - len, "BW40/");
	mt7996_txpower_puts(EHT484, 16);
	len += scnprintf(buf + len, size - len, "BW80/");
	mt7996_txpower_puts(EHT996, 16);
	len += scnprintf(buf + len, size - len, "BW160/");
	mt7996_txpower_puts(EHT2x996, 15);
	len += scnprintf(buf + len, size - len, "BW320/");
	mt7996_txpower_puts(EHT4x996, 15);
	mt7996_txpower_puts(EHT26_52, 21);
	mt7996_txpower_puts(EHT26_106, 21);
	mt7996_txpower_puts(EHT484_242, 21);
	mt7996_txpower_puts(EHT996_484, 21);
	mt7996_txpower_puts(EHT996_484_242, 21);
	mt7996_txpower_puts(EHT2x996_484, 21);
	mt7996_txpower_puts(EHT3x996, 21);
	mt7996_txpower_puts(EHT3x996_484, 21);

	len += scnprintf(buf + len, size - len, "\nePA Gain: %d\n",
			 event->phy_rate_info.epa_gain);
	len += scnprintf(buf + len, size - len, "Max Power Bound: %d\n",
			 event->phy_rate_info.max_power_bound);
	len += scnprintf(buf + len, size - len, "Min Power Bound: %d\n",
			 event->phy_rate_info.min_power_bound);

	reg = MT_WF_PHYDFE_TSSI_TXCTRL01(band_idx);
	len += scnprintf(buf + len, size - len,
			 "\nBBP TX Power (target power from TMAC)  : %6ld [0.5 dBm]\n",
			 mt76_get_field(dev, reg, MT_WF_PHYDFE_TSSI_TXCTRL_POWER_TMAC));
	len += scnprintf(buf + len, size - len,
			 "RegDB maximum power:\t%d [dBm]\n",
			 chan->max_reg_power);

	if (chan->band == NL80211_BAND_2GHZ)
		sband = phy->mt76->sband_2g.sband;
	else if (chan->band == NL80211_BAND_5GHZ)
		sband = phy->mt76->sband_5g.sband;
	else if (chan->band == NL80211_BAND_6GHZ)
		sband = phy->mt76->sband_6g.sband;

	pwr_delta = mt7996_eeprom_get_power_delta(dev, sband.band);

	target_power = max_t(u32, target_power, mt7996_eeprom_get_target_power(dev, chan));
	target_power += pwr_delta + path_delta;
	target_power = DIV_ROUND_UP(target_power, 2);
	len += scnprintf(buf + len, size - len,
			 "eeprom maximum power:\t%d [dBm]\n",
			 target_power);

	len += scnprintf(buf + len, size - len,
			 "path_delta:\t%d [0.5 dBm]\n",
			 path_delta);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

out:
	kfree(buf);
	kfree(event);
	return ret;
}

static const struct file_operations mt7996_txpower_sku_fops = {
	.read = mt7996_get_txpower_sku,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

#define mt7996_txpower_path_puts(rate, arr_length)					\
({											\
	len += scnprintf(buf + len, size - len, "%23s:", #rate " (TMAC)");		\
	for (i = 0; i < arr_length; i++, offs++)					\
		len += scnprintf(buf + len, size - len, " %4d",				\
				 event->backoff_table_info.frame_power[offs]);		\
	len += scnprintf(buf + len, size - len, "\n");					\
})

static ssize_t
mt7996_get_txpower_path(struct file *file, char __user *user_buf,
		       size_t count, loff_t *ppos)
{
	struct mt7996_phy *phy = file->private_data;
	struct mt7996_mcu_txpower_event *event;
	static const size_t size = 5120;
	int i, offs = 0, len = 0;
	ssize_t ret;
	char *buf;

	buf = kzalloc(size, GFP_KERNEL);
	event = kzalloc(sizeof(*event), GFP_KERNEL);
	if (!buf || !event) {
		ret = -ENOMEM;
		goto out;
	}

	ret = mt7996_mcu_get_tx_power_info(phy, BACKOFF_TABLE_INFO, event);
	if (ret ||
	    le32_to_cpu(event->phy_rate_info.category) != UNI_TXPOWER_BACKOFF_TABLE_SHOW_INFO)
		goto out;

	len += scnprintf(buf + len, size - len, "\n%*c", 25, ' ');
	len += scnprintf(buf + len, size - len, "1T1S/2T1S/3T1S/4T1S/5T1S/2T2S/3T2S/4T2S/5T2S/"
			 "3T3S/4T3S/5T3S/4T4S/5T4S/5T5S\n");

	mt7996_txpower_path_puts(CCK, 5);
	mt7996_txpower_path_puts(OFDM, 5);
	mt7996_txpower_path_puts(BF-OFDM, 4);

	mt7996_txpower_path_puts(RU26, 15);
	mt7996_txpower_path_puts(BF-RU26, 15);
	mt7996_txpower_path_puts(RU52, 15);
	mt7996_txpower_path_puts(BF-RU52, 15);
	mt7996_txpower_path_puts(RU26_52, 15);
	mt7996_txpower_path_puts(BF-RU26_52, 15);
	mt7996_txpower_path_puts(RU106, 15);
	mt7996_txpower_path_puts(BF-RU106, 15);
	mt7996_txpower_path_puts(RU106_52, 15);
	mt7996_txpower_path_puts(BF-RU106_52, 15);

	mt7996_txpower_path_puts(BW20/RU242, 15);
	mt7996_txpower_path_puts(BF-BW20/RU242, 15);
	mt7996_txpower_path_puts(BW40/RU484, 15);
	mt7996_txpower_path_puts(BF-BW40/RU484, 15);
	mt7996_txpower_path_puts(RU242_484, 15);
	mt7996_txpower_path_puts(BF-RU242_484, 15);
	mt7996_txpower_path_puts(BW80/RU996, 15);
	mt7996_txpower_path_puts(BF-BW80/RU996, 15);
	mt7996_txpower_path_puts(RU484_996, 15);
	mt7996_txpower_path_puts(BF-RU484_996, 15);
	mt7996_txpower_path_puts(RU242_484_996, 15);
	mt7996_txpower_path_puts(BF-RU242_484_996, 15);
	mt7996_txpower_path_puts(BW160/RU996x2, 15);
	mt7996_txpower_path_puts(BF-BW160/RU996x2, 15);
	mt7996_txpower_path_puts(RU484_996x2, 15);
	mt7996_txpower_path_puts(BF-RU484_996x2, 15);
	mt7996_txpower_path_puts(RU996x3, 15);
	mt7996_txpower_path_puts(BF-RU996x3, 15);
	mt7996_txpower_path_puts(RU484_996x3, 15);
	mt7996_txpower_path_puts(BF-RU484_996x3, 15);
	mt7996_txpower_path_puts(BW320/RU996x4, 15);
	mt7996_txpower_path_puts(BF-BW320/RU996x4, 15);

	len += scnprintf(buf + len, size - len, "\nBackoff table: %s\n",
			 event->backoff_table_info.backoff_en ? "enable" : "disable");

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

out:
	kfree(buf);
	kfree(event);
	return ret;
}

static const struct file_operations mt7996_txpower_path_fops = {
	.read = mt7996_get_txpower_path,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static int mt7996_show_eeprom_mode(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
#ifdef CONFIG_NL80211_TESTMODE
	struct mt76_dev *mdev = &dev->mt76;
	const char *mtd_name = mdev->test_mtd.name;
	u32 mtd_offset = mdev->test_mtd.offset;
#else
	const char *mtd_name = NULL;
	u32 mtd_offset;
#endif

	seq_printf(s, "Current eeprom mode:\n");

	switch (dev->eeprom_mode) {
	case DEFAULT_BIN_MODE:
		seq_printf(s, "   default bin mode\n   filename = %s\n", mt7996_eeprom_name(dev));
		break;
	case EFUSE_MODE:
		seq_printf(s, "   efuse mode\n");
		break;
	case FLASH_MODE:
		if (mtd_name)
			seq_printf(s, "   flash mode\n   mtd name = %s\n   flash offset = 0x%x\n",
				   mtd_name, mtd_offset);
		else
			seq_printf(s, "   flash mode\n");
		break;
	case BIN_FILE_MODE:
		seq_printf(s, "   bin file mode\n   filename = %s\n", dev->mt76.bin_file_name);
		break;
	case EXT_EEPROM_MODE:
		seq_printf(s, "   external eeprom mode\n");
		break;
	default:
		break;
	}

	return 0;
}

static int
mt7996_sr_enable_get(void *data, u64 *val)
{
	struct mt7996_phy *phy = data;

	*val = phy->sr_enable;

	return 0;
}

static int
mt7996_sr_enable_set(void *data, u64 val)
{
	struct mt7996_phy *phy = data;
	int ret;

	if (!!val == phy->sr_enable)
		return 0;

	ret = mt7996_mcu_set_sr_enable(phy, UNI_CMD_SR_CFG_SR_ENABLE, val, true);
	if (ret)
		return ret;

	return mt7996_mcu_set_sr_enable(phy, UNI_CMD_SR_CFG_SR_ENABLE, 0, false);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_sr_enable, mt7996_sr_enable_get,
			 mt7996_sr_enable_set, "%lld\n");
static int
mt7996_sr_enhanced_enable_get(void *data, u64 *val)
{
	struct mt7996_phy *phy = data;

	*val = phy->enhanced_sr_enable;

	return 0;
}

static int
mt7996_sr_enhanced_enable_set(void *data, u64 val)
{
	struct mt7996_phy *phy = data;
	int ret;

	if (!!val == phy->enhanced_sr_enable)
		return 0;

	ret = mt7996_mcu_set_sr_enable(phy, UNI_CMD_SR_HW_ENHANCE_SR_ENABLE, val, true);
	if (ret)
		return ret;

	return mt7996_mcu_set_sr_enable(phy, UNI_CMD_SR_HW_ENHANCE_SR_ENABLE, 0, false);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_sr_enhanced_enable, mt7996_sr_enhanced_enable_get,
			 mt7996_sr_enhanced_enable_set, "%lld\n");

static int
mt7996_sr_stats_show(struct seq_file *file, void *data)
{
	struct mt7996_phy *phy = file->private;

	mt7996_mcu_set_sr_enable(phy, UNI_CMD_SR_HW_IND, 0, false);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt7996_sr_stats);

static int
mt7996_sr_scene_cond_show(struct seq_file *file, void *data)
{
	struct mt7996_phy *phy = file->private;

	mt7996_mcu_set_sr_enable(phy, UNI_CMD_SR_SW_SD, 0, false);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt7996_sr_scene_cond);

static int
mt7996_starec_bf_read_set(void *data, u64 wlan_idx)
{
	struct mt7996_phy *phy = data;

	return mt7996_mcu_set_txbf_internal(phy, BF_STA_REC_READ, wlan_idx, 0);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_starec_bf_read, NULL,
			 mt7996_starec_bf_read_set, "%lld\n");

static ssize_t
mt7996_bf_txsnd_info_set(struct file *file,
			 const char __user *user_buf,
			 size_t count, loff_t *ppos)
{
	struct mt7996_phy *phy = file->private_data;
	char buf[40];
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	if (count && buf[count - 1] == '\n')
		buf[count - 1] = '\0';
	else
		buf[count] = '\0';

	ret = mt7996_mcu_set_txbf_snd_info(phy->dev, buf);

	if (ret) return -EFAULT;

	return count;
}

static const struct file_operations fops_bf_txsnd_info = {
	.write = mt7996_bf_txsnd_info_set,
	.read = NULL,
	.open = simple_open,
	.llseek = default_llseek,
};

static int
mt7996_bf_fbk_rpt_set(void *data, u64 wlan_idx)
{
	struct mt7996_phy *phy = data;

	return mt7996_mcu_set_txbf_internal(phy, BF_FBRPT_DBG_INFO_READ, wlan_idx, 0);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_bf_fbk_rpt, NULL,
			 mt7996_bf_fbk_rpt_set, "%lld\n");

static int
mt7996_bf_pfmu_tag_read_set(void *data, u64 wlan_idx)
{
	struct mt7996_phy *phy = data;

	return mt7996_mcu_set_txbf_internal(phy, BF_PFMU_TAG_READ, wlan_idx, 1);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_bf_pfmu_tag_read, NULL,
			 mt7996_bf_pfmu_tag_read_set, "%lld\n");

static int
mt7996_muru_fixed_rate_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;

	return mt7996_mcu_set_muru_fixed_rate_enable(dev, UNI_CMD_MURU_FIXED_RATE_CTRL,
						     val);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_muru_fixed_rate_enable, NULL,
			 mt7996_muru_fixed_rate_set, "%lld\n");

static ssize_t
mt7996_muru_fixed_rate_parameter_set(struct file *file,
				     const char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	struct mt7996_dev *dev = file->private_data;
	char buf[40];
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	if (count && buf[count - 1] == '\n')
		buf[count - 1] = '\0';
	else
		buf[count] = '\0';


	ret = mt7996_mcu_set_muru_fixed_rate_parameter(dev, UNI_CMD_MURU_FIXED_GROUP_RATE_CTRL,
						       buf);

	if (ret) return -EFAULT;

	return count;
}

static const struct file_operations fops_muru_fixed_group_rate = {
	.write = mt7996_muru_fixed_rate_parameter_set,
	.read = NULL,
	.open = simple_open,
	.llseek = default_llseek,
};

static int mt7996_muru_prot_thr_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;

	return mt7996_mcu_muru_set_prot_frame_thr(dev, (u32)val);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_muru_prot_thr, NULL,
			 mt7996_muru_prot_thr_set, "%lld\n");

static int
mt7996_red_config_get(void *data, u64 *val)
{
	struct mt7996_dev *dev = data;

	*val = dev->red_enable;

	return 0;
}

static int
mt7996_red_config_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;

	return mt7996_mcu_red_config(dev, !!val);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_red_config, mt7996_red_config_get,
			 mt7996_red_config_set, "%lld\n");

static int
mt7996_vow_drr_dbg(void *data, u64 val)
{
	struct mt7996_dev *dev = data;

	return mt7996_mcu_set_vow_drr_dbg(dev, (u32)val);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_vow_drr_dbg, NULL,
			 mt7996_vow_drr_dbg, "%lld\n");

static int
mt7996_rro_session_read(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	struct mt7996_rro_ba_session *tbl;
	u32 value[2];

	mt76_wr(dev, MT_RRO_DBG_RD_CTRL, MT_RRO_DBG_RD_EXEC +
		(dev->dbg.sid >> 1) + 0x200);

	if (dev->dbg.sid & 0x1) {
		value[0] = mt76_rr(dev, MT_RRO_DBG_RDAT_DW(2));
		value[1] = mt76_rr(dev, MT_RRO_DBG_RDAT_DW(3));
	} else {
		value[0] = mt76_rr(dev, MT_RRO_DBG_RDAT_DW(0));
		value[1] = mt76_rr(dev, MT_RRO_DBG_RDAT_DW(1));
	}

	tbl = (struct mt7996_rro_ba_session *)&value[0];

	seq_printf(s, " seid %d:\nba session table DW0:%08x DW2:%08x\n",
		   dev->dbg.sid, value[0], value[1]);

	seq_printf(s, "ack_sn = 0x%x, last_in_sn = 0x%x, sat/bn/bc/bd/cn = %d/%d/%d/%d/%d\n",
		   tbl->ack_sn, tbl->last_in_sn, tbl->sat, tbl->bn, tbl->bc, tbl->bd, tbl->cn);

	seq_printf(s, "within_cnt = %d, to_sel = %d, last_in_rxtime = %d\n",
		   tbl->within_cnt, tbl->to_sel, tbl->last_in_rxtime);

	return 0;
}

static int
mt7996_show_rro_mib(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	u32 reg[12];

	seq_printf(s, "RRO mib Info:\n");

	reg[0] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(0));
	reg[1] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(1));
	reg[2] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(2));
	reg[3] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(3));
	reg[4] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(4));
	reg[5] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(5));
	reg[6] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(6));
	reg[7] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(7));
	reg[8] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(8));
	reg[9] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(9));
	reg[10] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(10));
	reg[11] = mt76_rr(dev, WF_RRO_TOP_STATISTIC(11));

	seq_printf(s, "STEP_ONE/WITHIN/SURPASS = %x/%x/%x\n", reg[0], reg[3], reg[4]);
	seq_printf(s, "REPEAT/OLDPKT/BAR = %x/%x/%x\n", reg[1], reg[2], reg[5]);
	seq_printf(s, "SURPASS with big gap = %x\n", reg[6]);
	seq_printf(s, "DISCONNECT/INVALID = %x/%x\n", reg[7], reg[8]);
	seq_printf(s, "TO(Step one)/TO(flush all) = %x/%x\n", reg[9], reg[10]);
	seq_printf(s, "buf ran out = %x\n", reg[11]);

	return 0;
}

static int
mt7996_thermal_enable_get(void *data, u64 *enable)
{
	struct mt7996_phy *phy = data;

	*enable = phy->thermal_protection_enable;

	return 0;
}

static int
mt7996_thermal_enable_set(void *data, u64 action)
{
	struct mt7996_phy *phy = data;
	int ret;
	u8 throttling;

	if (action > 1)
		return -EINVAL;

	if (!!action == phy->thermal_protection_enable)
		return 0;

	ret = mt7996_mcu_set_thermal_protect(phy, !!action);
	if (ret)
		return ret;

	if (!!!action)
		goto out;

	throttling = MT7996_THERMAL_THROTTLE_MAX - phy->cdev_state;
	ret = mt7996_mcu_set_thermal_throttling(phy, throttling);
	if (ret)
		return ret;

out:
	phy->thermal_protection_enable = !!action;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_thermal_enable, mt7996_thermal_enable_get,
			 mt7996_thermal_enable_set, "%lld\n");

static int
mt7996_thermal_recal_set(void *data, u64 val)
{
#define THERMAL_DEBUG_OPERATION_MANUAL_TRIGGER 2
#define THERMAL_DEBUG_MODE_RECAL 1
	struct mt7996_dev *dev = data;

	if (val > THERMAL_DEBUG_OPERATION_MANUAL_TRIGGER)
		return -EINVAL;

	return mt7996_mcu_thermal_debug(dev, THERMAL_DEBUG_MODE_RECAL, val);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_thermal_recal, NULL,
			 mt7996_thermal_recal_set, "%llu\n");

static int
mt7996_reset_counter(void *data, u64 val)
{
	struct mt7996_dev *dev = data;
	struct mt76_dev *mdev = &dev->mt76;
	struct mt76_wcid *wcid;
	int ret;

	/* Reset read-clear counters in FW and WTBL. */
	ret = mt7996_mcu_get_all_sta_info(mdev, UNI_ALL_STA_TXRX_ADM_STAT);
	if (ret)
		return ret;

	ret = mt7996_mcu_get_all_sta_info(mdev, UNI_ALL_STA_TXRX_MSDU_COUNT);
	if (ret)
		return ret;

	ret = mt7996_mcu_get_all_sta_info(mdev, UNI_ALL_STA_TXRX_AIR_TIME);
	if (ret)
		return ret;

	ret = mt7996_mcu_get_all_sta_info(mdev, UNI_ALL_STA_RX_MPDU_COUNT);
	if (ret)
		return ret;

	/* Reset counters in MT76. */
	rcu_read_lock();
	wcid = rcu_dereference(dev->mt76.wcid[dev->wlan_idx]);
	if (wcid)
		memset(&wcid->stats, 0, sizeof(struct mt76_sta_stats));
	else
		ret = -EINVAL;
	rcu_read_unlock();

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_reset_counter, NULL, mt7996_reset_counter, "%llu\n");

void mt7996_packet_log_to_host(struct mt7996_dev *dev, const void *data, int len, int type, int des_len)
{
	struct bin_debug_hdr *hdr;
	char *buf;

	if (len > 1500 - sizeof(*hdr))
	len = 1500 - sizeof(*hdr);

	buf = kzalloc(sizeof(*hdr) + len, GFP_KERNEL);
	if (!buf)
		return;

	hdr = (struct bin_debug_hdr *)buf;
	hdr->magic_num = cpu_to_le32(PKT_BIN_DEBUG_MAGIC);
	hdr->serial_id = cpu_to_le16(dev->fw_debug_seq++);
	hdr->msg_type = cpu_to_le16(type);
	hdr->len = cpu_to_le16(len);
	hdr->des_len = cpu_to_le16(des_len);

	memcpy(buf + sizeof(*hdr), data, len);

	mt7996_debugfs_rx_log(dev, buf, sizeof(*hdr) + len);
	kfree(buf);
}

static int mt7996_rx_token_read(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	int id, count = 0;
	struct mt76_txwi_cache *t;

	seq_printf(s, "Rx cut through token:\n");
	spin_lock_bh(&dev->mt76.rx_token_lock);
	idr_for_each_entry(&dev->mt76.rx_token, t, id) {
		count++;
	}
	seq_printf(s, "\ttotal:%8d used:%8d\n",
		   dev->mt76.rx_token_size, count);
	spin_unlock_bh(&dev->mt76.rx_token_lock);

	return 0;
}

/* AMSDU SETTING */
static ssize_t mt7996_amsdu_algo_write(struct file *file,
				   const char __user *user_buf,
				   size_t count,
				   loff_t *ppos)
{
	struct mt7996_dev *dev = file->private_data;
	char buf[100];
	int ret;
	struct {
		u8 _rsv[4];

		u16 tag;
		u16 len;

		u16 wlan_idx;
		u8 algo_en;
		u8 rsv[1];
	} __packed data = {
		.tag = cpu_to_le16(UNI_MEC_AMSDU_ALGO_EN_STA),
		.len = cpu_to_le16(sizeof(data) - 4),
	};

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	if (count && buf[count - 1] == '\n')
		buf[count - 1] = '\0';
	else
		buf[count] = '\0';

	if (sscanf(buf, "%hu %hhu", &data.wlan_idx, &data.algo_en) != 2)
		return -EINVAL;

	if (data.wlan_idx >= mt7996_wtbl_size(dev))
		return -EINVAL;

	ret = mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(MEC), &data,
				sizeof(data), true);
	if (ret)
		return -EINVAL;

	return count;
}
static const struct file_operations fops_amsdu_algo = {
	.write = mt7996_amsdu_algo_write,
	.read = NULL,
	.open = simple_open,
	.llseek = default_llseek,
};

static ssize_t mt7996_amsdu_para_write(struct file *file,
				   const char __user *user_buf,
				   size_t count,
				   loff_t *ppos)
{
	struct mt7996_dev *dev = file->private_data;
	char buf[100];
	int ret;
	struct {
		u8 _rsv[4];

		u16 tag;
		u16 len;

		u16 wlan_idx;
		u8  amsdu_en;
		u8  num;
		u16 lenth;
		u8  rsv[2];
	} __packed data = {
		.tag = cpu_to_le16(UNI_MEC_AMSDU_PARA_STA),
		.len = cpu_to_le16(sizeof(data) - 4),
	};

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	if (count && buf[count - 1] == '\n')
		buf[count - 1] = '\0';
	else
		buf[count] = '\0';

	if (sscanf(buf, "%hu %hhu %hhu %hu", &data.wlan_idx, &data.amsdu_en, &data.num, &data.lenth) != 4)
		return -EINVAL;

	if (data.wlan_idx >= mt7996_wtbl_size(dev))
		return -EINVAL;

	ret = mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(MEC), &data,
			  sizeof(data), true);
	if (ret)
		return -EINVAL;

	return count;
}
static const struct file_operations fops_amsdu_para = {
	.write = mt7996_amsdu_para_write,
	.read = NULL,
	.open = simple_open,
	.llseek = default_llseek,
};

/* PSE INFO */
static struct bmac_queue_info_t pse_queue_empty_info[] = {
	{"CPU Q0",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_0},
	{"CPU Q1",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_1},
	{"CPU Q2",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_2},
	{"CPU Q3",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_3},
	{NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, /* 4~7 not defined */
	{NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	{NULL, 0, 0}, {NULL, 0, 0},  /* 14~15 not defined */
	{"LMAC Q",  ENUM_UMAC_LMAC_PORT_2,    0},
	{"MDP TX Q0", ENUM_UMAC_LMAC_PORT_2, 1},
	{"MDP RX Q", ENUM_UMAC_LMAC_PORT_2, 2},
	{"SEC TX Q0", ENUM_UMAC_LMAC_PORT_2, 3},
	{"SEC RX Q", ENUM_UMAC_LMAC_PORT_2, 4},
	{"SFD_PARK Q", ENUM_UMAC_LMAC_PORT_2, 5},
	{"MDP_TXIOC Q0", ENUM_UMAC_LMAC_PORT_2, 6},
	{"MDP_RXIOC Q0", ENUM_UMAC_LMAC_PORT_2, 7},
	{"MDP TX Q1", ENUM_UMAC_LMAC_PORT_2, 0x11},
	{"SEC TX Q1", ENUM_UMAC_LMAC_PORT_2, 0x13},
	{"MDP_TXIOC Q1", ENUM_UMAC_LMAC_PORT_2, 0x16},
	{"MDP_RXIOC Q1", ENUM_UMAC_LMAC_PORT_2, 0x17},
	{"CPU Q3",  ENUM_UMAC_CPU_PORT_1,     4},
	{NULL, 0, 0}, {NULL, 0, 0},
	{"RLS Q",  ENUM_PLE_CTRL_PSE_PORT_3, ENUM_UMAC_PLE_CTRL_P3_Q_0X1F}
};

static struct bmac_queue_info_t pse_queue_empty2_info[] = {
	{"MDP_TDPIOC Q0", ENUM_UMAC_LMAC_PORT_2, 0x8},
	{"MDP_RDPIOC Q0", ENUM_UMAC_LMAC_PORT_2, 0x9},
	{"MDP_TDPIOC Q1", ENUM_UMAC_LMAC_PORT_2, 0x18},
	{"MDP_RDPIOC Q1", ENUM_UMAC_LMAC_PORT_2, 0x19},
	{"MDP_TDPIOC Q2", ENUM_UMAC_LMAC_PORT_2, 0x28},
	{"MDP_RDPIOC Q2", ENUM_UMAC_LMAC_PORT_2, 0x29},
	{NULL, 0, 0},
	{"MDP_RDPIOC Q3", ENUM_UMAC_LMAC_PORT_2, 0x39},
	{"MDP TX Q2", ENUM_UMAC_LMAC_PORT_2, 0x21},
	{"SEC TX Q2", ENUM_UMAC_LMAC_PORT_2, 0x23},
	{"MDP_TXIOC Q2", ENUM_UMAC_LMAC_PORT_2, 0x26},
	{"MDP_RXIOC Q2", ENUM_UMAC_LMAC_PORT_2, 0x27},
	{NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	{"MDP_RXIOC Q3", ENUM_UMAC_LMAC_PORT_2, 0x37},
	{"HIF Q0", ENUM_UMAC_HIF_PORT_0,    0},
	{"HIF Q1", ENUM_UMAC_HIF_PORT_0,    1},
	{"HIF Q2", ENUM_UMAC_HIF_PORT_0,    2},
	{"HIF Q3", ENUM_UMAC_HIF_PORT_0,    3},
	{"HIF Q4", ENUM_UMAC_HIF_PORT_0,    4},
	{"HIF Q5", ENUM_UMAC_HIF_PORT_0,    5},
	{"HIF Q6", ENUM_UMAC_HIF_PORT_0,    6},
	{"HIF Q7", ENUM_UMAC_HIF_PORT_0,    7},
	{"HIF Q8", ENUM_UMAC_HIF_PORT_0,    8},
	{"HIF Q9", ENUM_UMAC_HIF_PORT_0,    9},
	{"HIF Q10", ENUM_UMAC_HIF_PORT_0,    10},
	{"HIF Q11", ENUM_UMAC_HIF_PORT_0,    11},
	{"HIF Q12", ENUM_UMAC_HIF_PORT_0,    12},
	{"HIF Q13", ENUM_UMAC_HIF_PORT_0,    13},
	{NULL, 0, 0}, {NULL, 0, 0}
};

static int
mt7996_pseinfo_read(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	u32 pse_buf_ctrl, pg_sz, pg_num;
	u32 pse_stat[2], pg_flow_ctrl[28] = {0};
	u32 fpg_cnt, ffa_cnt, fpg_head, fpg_tail;
	u32 max_q, min_q, rsv_pg, used_pg;
	int i;

	pse_buf_ctrl = mt76_rr(dev, WF_PSE_TOP_PBUF_CTRL_ADDR);
	pse_stat[0] = mt76_rr(dev, WF_PSE_TOP_QUEUE_EMPTY_ADDR);
	pse_stat[1] = mt76_rr(dev, WF_PSE_TOP_QUEUE_EMPTY_1_ADDR);
	pg_flow_ctrl[0] = mt76_rr(dev, WF_PSE_TOP_FREEPG_CNT_ADDR);
	pg_flow_ctrl[1] = mt76_rr(dev, WF_PSE_TOP_FREEPG_HEAD_TAIL_ADDR);
	pg_flow_ctrl[2] = mt76_rr(dev, WF_PSE_TOP_PG_HIF0_GROUP_ADDR);
	pg_flow_ctrl[3] = mt76_rr(dev, WF_PSE_TOP_HIF0_PG_INFO_ADDR);
	pg_flow_ctrl[4] = mt76_rr(dev, WF_PSE_TOP_PG_HIF1_GROUP_ADDR);
	pg_flow_ctrl[5] = mt76_rr(dev, WF_PSE_TOP_HIF1_PG_INFO_ADDR);
	pg_flow_ctrl[6] = mt76_rr(dev, WF_PSE_TOP_PG_CPU_GROUP_ADDR);
	pg_flow_ctrl[7] = mt76_rr(dev, WF_PSE_TOP_CPU_PG_INFO_ADDR);
	pg_flow_ctrl[8] = mt76_rr(dev, WF_PSE_TOP_PG_LMAC0_GROUP_ADDR);
	pg_flow_ctrl[9] = mt76_rr(dev, WF_PSE_TOP_LMAC0_PG_INFO_ADDR);
	pg_flow_ctrl[10] = mt76_rr(dev, WF_PSE_TOP_PG_LMAC1_GROUP_ADDR);
	pg_flow_ctrl[11] = mt76_rr(dev, WF_PSE_TOP_LMAC1_PG_INFO_ADDR);
	pg_flow_ctrl[12] = mt76_rr(dev, WF_PSE_TOP_PG_LMAC2_GROUP_ADDR);
	pg_flow_ctrl[13] = mt76_rr(dev, WF_PSE_TOP_LMAC2_PG_INFO_ADDR);
	pg_flow_ctrl[14] = mt76_rr(dev, WF_PSE_TOP_PG_PLE_GROUP_ADDR);
	pg_flow_ctrl[15] = mt76_rr(dev, WF_PSE_TOP_PLE_PG_INFO_ADDR);
	pg_flow_ctrl[16] = mt76_rr(dev, WF_PSE_TOP_PG_LMAC3_GROUP_ADDR);
	pg_flow_ctrl[17] = mt76_rr(dev, WF_PSE_TOP_LMAC3_PG_INFO_ADDR);
	pg_flow_ctrl[18] = mt76_rr(dev, WF_PSE_TOP_PG_MDP_GROUP_ADDR);
	pg_flow_ctrl[19] = mt76_rr(dev, WF_PSE_TOP_MDP_PG_INFO_ADDR);
	pg_flow_ctrl[20] = mt76_rr(dev, WF_PSE_TOP_PG_PLE1_GROUP_ADDR);
	pg_flow_ctrl[21] = mt76_rr(dev, WF_PSE_TOP_PLE1_PG_INFO_ADDR);
	pg_flow_ctrl[22] = mt76_rr(dev, WF_PSE_TOP_PG_MDP2_GROUP_ADDR);
	pg_flow_ctrl[23] = mt76_rr(dev, WF_PSE_TOP_MDP2_PG_INFO_ADDR);
	if (mt7996_band_valid(dev, MT_BAND2)) {
		pg_flow_ctrl[24] = mt76_rr(dev, WF_PSE_TOP_PG_MDP3_GROUP_ADDR);
		pg_flow_ctrl[25] = mt76_rr(dev, WF_PSE_TOP_MDP3_PG_INFO_ADDR);
	}
	pg_flow_ctrl[26] = mt76_rr(dev, WF_PSE_TOP_PG_HIF2_GROUP_ADDR);
	pg_flow_ctrl[27] = mt76_rr(dev, WF_PSE_TOP_HIF2_PG_INFO_ADDR);
	/* Configuration Info */
	seq_printf(s, "PSE Configuration Info:\n");
	seq_printf(s, "\tPacket Buffer Control: 0x%08x\n", pse_buf_ctrl);
	pg_sz = (pse_buf_ctrl & WF_PSE_TOP_PBUF_CTRL_PAGE_SIZE_CFG_MASK) >> WF_PSE_TOP_PBUF_CTRL_PAGE_SIZE_CFG_SHFT;
	seq_printf(s, "\t\tPage Size=%d(%d bytes per page)\n", pg_sz, (pg_sz == 1 ? 256 : 128));
	seq_printf(s, "\t\tPage Offset=%d(in unit of 64KB)\n",
			 (pse_buf_ctrl & WF_PSE_TOP_PBUF_CTRL_PBUF_OFFSET_MASK) >> WF_PSE_TOP_PBUF_CTRL_PBUF_OFFSET_SHFT);
	pg_num = (pse_buf_ctrl & WF_PSE_TOP_PBUF_CTRL_TOTAL_PAGE_NUM_MASK) >> WF_PSE_TOP_PBUF_CTRL_TOTAL_PAGE_NUM_SHFT;
	seq_printf(s, "\t\tTotal page numbers=%d pages\n", pg_num);
	/* Page Flow Control */
	seq_printf(s, "PSE Page Flow Control:\n");
	seq_printf(s, "\tFree page counter: 0x%08x\n", pg_flow_ctrl[0]);
	fpg_cnt = (pg_flow_ctrl[0] & WF_PSE_TOP_FREEPG_CNT_FREEPG_CNT_MASK) >> WF_PSE_TOP_FREEPG_CNT_FREEPG_CNT_SHFT;
	seq_printf(s, "\t\tThe toal page number of free=0x%03x\n", fpg_cnt);
	ffa_cnt = (pg_flow_ctrl[0] & WF_PSE_TOP_FREEPG_CNT_FFA_CNT_MASK) >> WF_PSE_TOP_FREEPG_CNT_FFA_CNT_SHFT;
	seq_printf(s, "\t\tThe free page numbers of free for all=0x%03x\n", ffa_cnt);
	seq_printf(s, "\tFree page head and tail: 0x%08x\n", pg_flow_ctrl[1]);
	fpg_head = (pg_flow_ctrl[1] & WF_PSE_TOP_FREEPG_HEAD_TAIL_FREEPG_HEAD_MASK) >> WF_PSE_TOP_FREEPG_HEAD_TAIL_FREEPG_HEAD_SHFT;
	fpg_tail = (pg_flow_ctrl[1] & WF_PSE_TOP_FREEPG_HEAD_TAIL_FREEPG_TAIL_MASK) >> WF_PSE_TOP_FREEPG_HEAD_TAIL_FREEPG_TAIL_SHFT;
	seq_printf(s, "\t\tThe tail/head page of free page list=0x%03x/0x%03x\n", fpg_tail, fpg_head);
	seq_printf(s, "\tReserved page counter of HIF0 group: 0x%08x\n", pg_flow_ctrl[2]);
	seq_printf(s, "\tHIF0 group page status: 0x%08x\n", pg_flow_ctrl[3]);
	min_q = (pg_flow_ctrl[2] & WF_PSE_TOP_PG_HIF0_GROUP_HIF0_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_HIF0_GROUP_HIF0_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[2] & WF_PSE_TOP_PG_HIF0_GROUP_HIF0_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_HIF0_GROUP_HIF0_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of HIF0 group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[3] & WF_PSE_TOP_HIF0_PG_INFO_HIF0_RSV_CNT_MASK) >> WF_PSE_TOP_HIF0_PG_INFO_HIF0_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[3] & WF_PSE_TOP_HIF0_PG_INFO_HIF0_SRC_CNT_MASK) >> WF_PSE_TOP_HIF0_PG_INFO_HIF0_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of HIF0 group=0x%03x/0x%03x\n", used_pg, rsv_pg);
	seq_printf(s, "\tReserved page counter of HIF1 group: 0x%08x\n", pg_flow_ctrl[4]);
	seq_printf(s, "\tHIF1 group page status: 0x%08x\n", pg_flow_ctrl[5]);
	min_q = (pg_flow_ctrl[4] & WF_PSE_TOP_PG_HIF1_GROUP_HIF1_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_HIF1_GROUP_HIF1_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[4] & WF_PSE_TOP_PG_HIF1_GROUP_HIF1_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_HIF1_GROUP_HIF1_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of HIF1 group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[5] & WF_PSE_TOP_HIF1_PG_INFO_HIF1_RSV_CNT_MASK) >> WF_PSE_TOP_HIF1_PG_INFO_HIF1_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[5] & WF_PSE_TOP_HIF1_PG_INFO_HIF1_SRC_CNT_MASK) >> WF_PSE_TOP_HIF1_PG_INFO_HIF1_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of HIF1 group=0x%03x/0x%03x\n", used_pg, rsv_pg);
	seq_printf(s, "\tReserved page counter of HIF2 group: 0x%08x\n", pg_flow_ctrl[26]);
	seq_printf(s, "\tHIF2 group page status: 0x%08x\n", pg_flow_ctrl[27]);
	min_q = (pg_flow_ctrl[26] & WF_PSE_TOP_PG_HIF2_GROUP_HIF2_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_HIF2_GROUP_HIF2_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[26] & WF_PSE_TOP_PG_HIF2_GROUP_HIF2_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_HIF2_GROUP_HIF2_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of HIF2 group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[27] & WF_PSE_TOP_HIF2_PG_INFO_HIF2_RSV_CNT_MASK) >> WF_PSE_TOP_HIF2_PG_INFO_HIF2_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[27] & WF_PSE_TOP_HIF2_PG_INFO_HIF2_SRC_CNT_MASK) >> WF_PSE_TOP_HIF2_PG_INFO_HIF2_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of HIF2 group=0x%03x/0x%03x\n", used_pg, rsv_pg);
	seq_printf(s, "\tReserved page counter of CPU group: 0x%08x\n", pg_flow_ctrl[6]);
	seq_printf(s, "\tCPU group page status: 0x%08x\n", pg_flow_ctrl[7]);
	min_q = (pg_flow_ctrl[6] & WF_PSE_TOP_PG_CPU_GROUP_CPU_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_CPU_GROUP_CPU_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[6] & WF_PSE_TOP_PG_CPU_GROUP_CPU_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_CPU_GROUP_CPU_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of CPU group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[7] & WF_PSE_TOP_CPU_PG_INFO_CPU_RSV_CNT_MASK) >> WF_PSE_TOP_CPU_PG_INFO_CPU_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[7] & WF_PSE_TOP_CPU_PG_INFO_CPU_SRC_CNT_MASK) >> WF_PSE_TOP_CPU_PG_INFO_CPU_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of CPU group=0x%03x/0x%03x\n", used_pg, rsv_pg);
	seq_printf(s, "\tReserved page counter of LMAC0 group: 0x%08x\n", pg_flow_ctrl[8]);
	seq_printf(s, "\tLMAC0 group page status: 0x%08x\n", pg_flow_ctrl[9]);
	min_q = (pg_flow_ctrl[8] & WF_PSE_TOP_PG_LMAC0_GROUP_LMAC0_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_LMAC0_GROUP_LMAC0_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[8] & WF_PSE_TOP_PG_LMAC0_GROUP_LMAC0_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_LMAC0_GROUP_LMAC0_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of LMAC0 group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[9] & WF_PSE_TOP_LMAC0_PG_INFO_LMAC0_RSV_CNT_MASK) >> WF_PSE_TOP_LMAC0_PG_INFO_LMAC0_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[9] & WF_PSE_TOP_LMAC0_PG_INFO_LMAC0_SRC_CNT_MASK) >> WF_PSE_TOP_LMAC0_PG_INFO_LMAC0_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of LMAC0 group=0x%03x/0x%03x\n", used_pg, rsv_pg);
	seq_printf(s, "\tReserved page counter of LMAC1 group: 0x%08x\n", pg_flow_ctrl[10]);
	seq_printf(s, "\tLMAC1 group page status: 0x%08x\n", pg_flow_ctrl[11]);
	min_q = (pg_flow_ctrl[10] & WF_PSE_TOP_PG_LMAC1_GROUP_LMAC1_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_LMAC1_GROUP_LMAC1_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[10] & WF_PSE_TOP_PG_LMAC1_GROUP_LMAC1_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_LMAC1_GROUP_LMAC1_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of LMAC1 group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[11] & WF_PSE_TOP_LMAC1_PG_INFO_LMAC1_RSV_CNT_MASK) >> WF_PSE_TOP_LMAC1_PG_INFO_LMAC1_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[11] & WF_PSE_TOP_LMAC1_PG_INFO_LMAC1_SRC_CNT_MASK) >> WF_PSE_TOP_LMAC1_PG_INFO_LMAC1_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of LMAC1 group=0x%03x/0x%03x\n", used_pg, rsv_pg);
	seq_printf(s, "\tReserved page counter of LMAC2 group: 0x%08x\n", pg_flow_ctrl[11]);
	seq_printf(s, "\tLMAC2 group page status: 0x%08x\n", pg_flow_ctrl[12]);
	min_q = (pg_flow_ctrl[12] & WF_PSE_TOP_PG_LMAC2_GROUP_LMAC2_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_LMAC2_GROUP_LMAC2_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[12] & WF_PSE_TOP_PG_LMAC2_GROUP_LMAC2_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_LMAC2_GROUP_LMAC2_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of LMAC2 group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[13] & WF_PSE_TOP_LMAC2_PG_INFO_LMAC2_RSV_CNT_MASK) >> WF_PSE_TOP_LMAC2_PG_INFO_LMAC2_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[13] & WF_PSE_TOP_LMAC2_PG_INFO_LMAC2_SRC_CNT_MASK) >> WF_PSE_TOP_LMAC2_PG_INFO_LMAC2_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of LMAC2 group=0x%03x/0x%03x\n", used_pg, rsv_pg);

	seq_printf(s, "\tReserved page counter of LMAC3 group: 0x%08x\n", pg_flow_ctrl[16]);
	seq_printf(s, "\tLMAC3 group page status: 0x%08x\n", pg_flow_ctrl[17]);
	min_q = (pg_flow_ctrl[16] & WF_PSE_TOP_PG_LMAC3_GROUP_LMAC3_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_LMAC3_GROUP_LMAC3_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[16] & WF_PSE_TOP_PG_LMAC3_GROUP_LMAC3_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_LMAC3_GROUP_LMAC3_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of LMAC3 group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[17] & WF_PSE_TOP_LMAC3_PG_INFO_LMAC3_RSV_CNT_MASK) >> WF_PSE_TOP_LMAC3_PG_INFO_LMAC3_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[17] & WF_PSE_TOP_LMAC3_PG_INFO_LMAC3_SRC_CNT_MASK) >> WF_PSE_TOP_LMAC3_PG_INFO_LMAC3_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of LMAC3 group=0x%03x/0x%03x\n", used_pg, rsv_pg);

	seq_printf(s, "\tReserved page counter of PLE group: 0x%08x\n", pg_flow_ctrl[14]);
	seq_printf(s, "\tPLE group page status: 0x%08x\n", pg_flow_ctrl[15]);
	min_q = (pg_flow_ctrl[14] & WF_PSE_TOP_PG_PLE_GROUP_PLE_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_PLE_GROUP_PLE_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[14] & WF_PSE_TOP_PG_PLE_GROUP_PLE_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_PLE_GROUP_PLE_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of PLE group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[15] & WF_PSE_TOP_PLE_PG_INFO_PLE_RSV_CNT_MASK) >> WF_PSE_TOP_PLE_PG_INFO_PLE_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[15] & WF_PSE_TOP_PLE_PG_INFO_PLE_SRC_CNT_MASK) >> WF_PSE_TOP_PLE_PG_INFO_PLE_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of PLE group=0x%03x/0x%03x\n", used_pg, rsv_pg);

	seq_printf(s, "\tReserved page counter of PLE1 group: 0x%08x\n", pg_flow_ctrl[14]);
	seq_printf(s, "\tPLE1 group page status: 0x%08x\n", pg_flow_ctrl[15]);
	min_q = (pg_flow_ctrl[20] & WF_PSE_TOP_PG_PLE_GROUP_PLE_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_PLE_GROUP_PLE_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[20] & WF_PSE_TOP_PG_PLE_GROUP_PLE_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_PLE_GROUP_PLE_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of PLE1 group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[21] & WF_PSE_TOP_PLE_PG_INFO_PLE_RSV_CNT_MASK) >> WF_PSE_TOP_PLE_PG_INFO_PLE_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[21] & WF_PSE_TOP_PLE_PG_INFO_PLE_SRC_CNT_MASK) >> WF_PSE_TOP_PLE_PG_INFO_PLE_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of PLE1 group=0x%03x/0x%03x\n", used_pg, rsv_pg);

	seq_printf(s, "\tReserved page counter of MDP group: 0x%08x\n", pg_flow_ctrl[18]);
	seq_printf(s, "\tMDP group page status: 0x%08x\n", pg_flow_ctrl[19]);
	min_q = (pg_flow_ctrl[18] & WF_PSE_TOP_PG_MDP_GROUP_MDP_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_MDP_GROUP_MDP_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[18] & WF_PSE_TOP_PG_MDP_GROUP_MDP_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_MDP_GROUP_MDP_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of MDP group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[19] & WF_PSE_TOP_MDP_PG_INFO_MDP_RSV_CNT_MASK) >> WF_PSE_TOP_MDP_PG_INFO_MDP_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[19] & WF_PSE_TOP_MDP_PG_INFO_MDP_SRC_CNT_MASK) >> WF_PSE_TOP_MDP_PG_INFO_MDP_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of MDP group=0x%03x/0x%03x\n", used_pg, rsv_pg);
	seq_printf(s, "\tReserved page counter of MDP2 group: 0x%08x\n", pg_flow_ctrl[22]);
	seq_printf(s, "\tMDP2 group page status: 0x%08x\n", pg_flow_ctrl[23]);
	min_q = (pg_flow_ctrl[22] & WF_PSE_TOP_PG_MDP2_GROUP_MDP2_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_MDP2_GROUP_MDP2_MIN_QUOTA_SHFT;
	max_q = (pg_flow_ctrl[22] & WF_PSE_TOP_PG_MDP2_GROUP_MDP2_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_MDP2_GROUP_MDP2_MAX_QUOTA_SHFT;
	seq_printf(s, "\t\tThe max/min quota pages of MDP2 group=0x%03x/0x%03x\n", max_q, min_q);
	rsv_pg = (pg_flow_ctrl[23] & WF_PSE_TOP_MDP2_PG_INFO_MDP2_RSV_CNT_MASK) >> WF_PSE_TOP_MDP2_PG_INFO_MDP2_RSV_CNT_SHFT;
	used_pg = (pg_flow_ctrl[23] & WF_PSE_TOP_MDP2_PG_INFO_MDP2_SRC_CNT_MASK) >> WF_PSE_TOP_MDP2_PG_INFO_MDP2_SRC_CNT_SHFT;
	seq_printf(s, "\t\tThe used/reserved pages of MDP2 group=0x%03x/0x%03x\n", used_pg, rsv_pg);
	if (mt7996_band_valid(dev, MT_BAND2)) {
		seq_printf(s, "\tReserved page counter of MDP3 group: 0x%08x\n", pg_flow_ctrl[24]);
		seq_printf(s, "\tMDP3 group page status: 0x%08x\n", pg_flow_ctrl[25]);
		min_q = (pg_flow_ctrl[24] & WF_PSE_TOP_PG_MDP3_GROUP_MDP3_MIN_QUOTA_MASK) >> WF_PSE_TOP_PG_MDP3_GROUP_MDP3_MIN_QUOTA_SHFT;
		max_q = (pg_flow_ctrl[24] & WF_PSE_TOP_PG_MDP3_GROUP_MDP3_MAX_QUOTA_MASK) >> WF_PSE_TOP_PG_MDP3_GROUP_MDP3_MAX_QUOTA_SHFT;
		seq_printf(s, "\t\tThe max/min quota pages of MDP3 group=0x%03x/0x%03x\n", max_q, min_q);
		rsv_pg = (pg_flow_ctrl[25] & WF_PSE_TOP_MDP3_PG_INFO_MDP3_RSV_CNT_MASK) >> WF_PSE_TOP_MDP3_PG_INFO_MDP3_RSV_CNT_SHFT;
		used_pg = (pg_flow_ctrl[25] & WF_PSE_TOP_MDP3_PG_INFO_MDP3_SRC_CNT_MASK) >> WF_PSE_TOP_MDP3_PG_INFO_MDP3_SRC_CNT_SHFT;
		seq_printf(s, "\t\tThe used/reserved pages of MDP3 group=0x%03x/0x%03x\n", used_pg, rsv_pg);
	}
	/* Queue Empty Status */
	seq_printf(s, "PSE Queue Empty Status:\n");
	seq_printf(s, "\tQUEUE_EMPTY: 0x%08x, QUEUE_EMPTY2: 0x%08x\n", pse_stat[0], pse_stat[1]);
	seq_printf(s, "\t\tCPU Q0/1/2/3/4 empty=%d/%d/%d/%d/%d\n",
			  (pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_CPU_Q0_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_CPU_Q0_EMPTY_SHFT,
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_CPU_Q1_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_CPU_Q1_EMPTY_SHFT),
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_CPU_Q2_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_CPU_Q2_EMPTY_SHFT),
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_CPU_Q3_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_CPU_Q3_EMPTY_SHFT),
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_CPU_Q4_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_CPU_Q4_EMPTY_SHFT));
	seq_printf(s, "\t\tHIF Q0/1/2/3/4/5/6/7/8 empty=%d/%d/%d/%d/%d/%d/%d/%d/%d\n",
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_0_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_0_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_1_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_1_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_2_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_2_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_3_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_3_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_4_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_4_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_5_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_5_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_6_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_6_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_7_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_7_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_8_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_8_EMPTY_SHFT));
	seq_printf(s, "\t\tHIF Q9/10/11/12/13/14/15/16 empty=%d/%d/%d/%d/%d/%d/%d/%d\n",
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_9_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_9_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_10_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_10_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_11_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_11_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_12_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_12_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_13_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_13_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_14_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_14_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_15_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_15_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_HIF_16_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_HIF_16_EMPTY_SHFT));
	seq_printf(s, "\t\tLMAC TX Q empty=%d\n",
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_LMAC_TX_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_LMAC_TX_QUEUE_EMPTY_SHFT));
	seq_printf(s, "\t\tMDP TX Q0/Q1/Q2/RX Q empty=%d/%d/%d/%d\n",
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_MDP_TX_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_MDP_TX_QUEUE_EMPTY_SHFT),
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_MDP_TX1_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_MDP_TX1_QUEUE_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_MDP_TX2_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_MDP_TX2_QUEUE_EMPTY_SHFT),
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_MDP_RX_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_MDP_RX_QUEUE_EMPTY_SHFT));
	seq_printf(s, "\t\tSEC TX Q0/Q1/Q2/RX Q empty=%d/%d/%d/%d\n",
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_SEC_TX_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_SEC_TX_QUEUE_EMPTY_SHFT),
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_SEC_TX1_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_SEC_TX1_QUEUE_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_SEC_TX2_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_SEC_TX2_QUEUE_EMPTY_SHFT),
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_SEC_RX_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_SEC_RX_QUEUE_EMPTY_SHFT));
	seq_printf(s, "\t\tSFD PARK Q empty=%d\n",
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_SFD_PARK_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_SFD_PARK_QUEUE_EMPTY_SHFT));
	seq_printf(s, "\t\tMDP TXIOC Q0/Q1/Q2 empty=%d/%d/%d\n",
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_MDP_TXIOC_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_MDP_TXIOC_QUEUE_EMPTY_SHFT),
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_MDP_TXIOC1_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_MDP_TXIOC1_QUEUE_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_MDP_TXIOC2_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_MDP_TXIOC2_QUEUE_EMPTY_SHFT));
	seq_printf(s, "\t\tMDP RXIOC Q0/Q1/Q2/Q3 empty=%d/%d/%d/%d\n",
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_MDP_RXIOC_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_MDP_RXIOC_QUEUE_EMPTY_SHFT),
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_MDP_RXIOC1_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_MDP_RXIOC1_QUEUE_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_MDP_RXIOC2_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_MDP_RXIOC2_QUEUE_EMPTY_SHFT),
			  ((pse_stat[1] & WF_PSE_TOP_QUEUE_EMPTY_1_MDP_RXIOC3_QUEUE_EMPTY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_1_MDP_RXIOC3_QUEUE_EMPTY_SHFT));
	seq_printf(s, "\t\tRLS Q empty=%d\n",
			  ((pse_stat[0] & WF_PSE_TOP_QUEUE_EMPTY_RLS_Q_EMTPY_MASK) >> WF_PSE_TOP_QUEUE_EMPTY_RLS_Q_EMTPY_SHFT));
	seq_printf(s, "Nonempty Q info:\n");

	for (i = 0; i < 31; i++) {
		if (((pse_stat[0] & (0x1 << i)) >> i) == 0) {
			u32 hfid, tfid, pktcnt, fl_que_ctrl[3] = {0};

			if (pse_queue_empty_info[i].QueueName != NULL) {
				seq_printf(s, "\t%s: ", pse_queue_empty_info[i].QueueName);
				fl_que_ctrl[0] |= WF_PSE_TOP_FL_QUE_CTRL_0_EXECUTE_MASK;
				fl_que_ctrl[0] |= (pse_queue_empty_info[i].Portid << WF_PSE_TOP_FL_QUE_CTRL_0_Q_BUF_PID_SHFT);
				fl_que_ctrl[0] |= (pse_queue_empty_info[i].Queueid << WF_PSE_TOP_FL_QUE_CTRL_0_Q_BUF_QID_SHFT);
			} else
				continue;

			fl_que_ctrl[0] |= (0x1 << 31);
			mt76_wr(dev, WF_PSE_TOP_FL_QUE_CTRL_0_ADDR, fl_que_ctrl[0]);
			fl_que_ctrl[1] = mt76_rr(dev, WF_PSE_TOP_FL_QUE_CTRL_2_ADDR);
			fl_que_ctrl[2] = mt76_rr(dev, WF_PSE_TOP_FL_QUE_CTRL_3_ADDR);
			hfid = (fl_que_ctrl[1] & WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_HEAD_FID_MASK) >> WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_HEAD_FID_SHFT;
			tfid = (fl_que_ctrl[1] & WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_TAIL_FID_MASK) >> WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_TAIL_FID_SHFT;
			pktcnt = (fl_que_ctrl[2] & WF_PSE_TOP_FL_QUE_CTRL_3_QUEUE_PKT_NUM_MASK) >> WF_PSE_TOP_FL_QUE_CTRL_3_QUEUE_PKT_NUM_SHFT;
			seq_printf(s, "tail/head fid = 0x%03x/0x%03x, pkt cnt = 0x%03x\n",
					  tfid, hfid, pktcnt);
		}
	}

	for (i = 0; i < 31; i++) {
		if (((pse_stat[1] & (0x1 << i)) >> i) == 0) {
			u32 hfid, tfid, pktcnt, fl_que_ctrl[3] = {0};

			if (pse_queue_empty2_info[i].QueueName != NULL) {
				seq_printf(s, "\t%s: ", pse_queue_empty2_info[i].QueueName);
				fl_que_ctrl[0] |= WF_PSE_TOP_FL_QUE_CTRL_0_EXECUTE_MASK;
				fl_que_ctrl[0] |= (pse_queue_empty2_info[i].Portid << WF_PSE_TOP_FL_QUE_CTRL_0_Q_BUF_PID_SHFT);
				fl_que_ctrl[0] |= (pse_queue_empty2_info[i].Queueid << WF_PSE_TOP_FL_QUE_CTRL_0_Q_BUF_QID_SHFT);
			} else
				continue;

			fl_que_ctrl[0] |= (0x1 << 31);
			mt76_wr(dev, WF_PSE_TOP_FL_QUE_CTRL_0_ADDR, fl_que_ctrl[0]);
			fl_que_ctrl[1] = mt76_rr(dev, WF_PSE_TOP_FL_QUE_CTRL_2_ADDR);
			fl_que_ctrl[2] = mt76_rr(dev, WF_PSE_TOP_FL_QUE_CTRL_3_ADDR);
			hfid = (fl_que_ctrl[1] & WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_HEAD_FID_MASK) >> WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_HEAD_FID_SHFT;
			tfid = (fl_que_ctrl[1] & WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_TAIL_FID_MASK) >> WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_TAIL_FID_SHFT;
			pktcnt = (fl_que_ctrl[2] & WF_PSE_TOP_FL_QUE_CTRL_3_QUEUE_PKT_NUM_MASK) >> WF_PSE_TOP_FL_QUE_CTRL_3_QUEUE_PKT_NUM_SHFT;
			seq_printf(s, "tail/head fid = 0x%03x/0x%03x, pkt cnt = 0x%03x\n",
					  tfid, hfid, pktcnt);
		}
	}

	return 0;
}

/* PLE INFO */
static char *sta_ctrl_reg[] = {"ENABLE", "DISABLE", "PAUSE", "TWT_PAUSE"};
static struct bmac_queue_info ple_queue_empty_info[] = {
	{"CPU Q0",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_0, 0},
	{"CPU Q1",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_1, 0},
	{"CPU Q2",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_2, 0},
	{"CPU Q3",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_3, 0},
	{"ALTX Q0", ENUM_UMAC_LMAC_PORT_2,    0x10, 0},
	{"BMC Q0",  ENUM_UMAC_LMAC_PORT_2,    0x11, 0},
	{"BCN Q0",  ENUM_UMAC_LMAC_PORT_2,    0x12, 0},
	{"PSMP Q0", ENUM_UMAC_LMAC_PORT_2,    0x13, 0},
	{"ALTX Q1", ENUM_UMAC_LMAC_PORT_2,    0x10, 1},
	{"BMC Q1",  ENUM_UMAC_LMAC_PORT_2,    0x11, 1},
	{"BCN Q1",  ENUM_UMAC_LMAC_PORT_2,    0x12, 1},
	{"PSMP Q1", ENUM_UMAC_LMAC_PORT_2,    0x13, 1},
	{"ALTX Q2", ENUM_UMAC_LMAC_PORT_2,    0x10, 2},
	{"BMC Q2",  ENUM_UMAC_LMAC_PORT_2,    0x11, 2},
	{"BCN Q2",  ENUM_UMAC_LMAC_PORT_2,    0x12, 2},
	{"PSMP Q2", ENUM_UMAC_LMAC_PORT_2,    0x13, 2},
	{"NAF Q",   ENUM_UMAC_LMAC_PORT_2,    0x18, 0},
	{"NBCN Q",  ENUM_UMAC_LMAC_PORT_2,    0x19, 0},
	{NULL, 0, 0, 0}, {NULL, 0, 0, 0}, /* 18, 19 not defined */
	{"FIXFID Q", ENUM_UMAC_LMAC_PORT_2, 0x1a, 0},
	{NULL, 0, 0, 0}, {NULL, 0, 0, 0}, {NULL, 0, 0, 0}, {NULL, 0, 0, 0}, {NULL, 0, 0, 0},
	{NULL, 0, 0, 0}, {NULL, 0, 0, 0},
	{"RLS4 Q",   ENUM_PLE_CTRL_PSE_PORT_3, 0x7c, 0},
	{"RLS3 Q",   ENUM_PLE_CTRL_PSE_PORT_3, 0x7d, 0},
	{"RLS2 Q",   ENUM_PLE_CTRL_PSE_PORT_3, 0x7e, 0},
	{"RLS Q",  ENUM_PLE_CTRL_PSE_PORT_3, 0x7f, 0}
};

static struct bmac_queue_info_t ple_txcmd_queue_empty_info[__MT_MAX_BAND][32] = {
	{{"AC00Q", ENUM_UMAC_LMAC_PORT_2, 0x40},
	 {"AC01Q", ENUM_UMAC_LMAC_PORT_2, 0x41},
	 {"AC02Q", ENUM_UMAC_LMAC_PORT_2, 0x42},
	 {"AC03Q", ENUM_UMAC_LMAC_PORT_2, 0x43},
	 {"AC10Q", ENUM_UMAC_LMAC_PORT_2, 0x44},
	 {"AC11Q", ENUM_UMAC_LMAC_PORT_2, 0x45},
	 {"AC12Q", ENUM_UMAC_LMAC_PORT_2, 0x46},
	 {"AC13Q", ENUM_UMAC_LMAC_PORT_2, 0x47},
	 {"AC20Q", ENUM_UMAC_LMAC_PORT_2, 0x48},
	 {"AC21Q", ENUM_UMAC_LMAC_PORT_2, 0x49},
	 {"AC22Q", ENUM_UMAC_LMAC_PORT_2, 0x4a},
	 {"AC23Q", ENUM_UMAC_LMAC_PORT_2, 0x4b},
	 {"AC30Q", ENUM_UMAC_LMAC_PORT_2, 0x4c},
	 {"AC31Q", ENUM_UMAC_LMAC_PORT_2, 0x4d},
	 {"AC32Q", ENUM_UMAC_LMAC_PORT_2, 0x4e},
	 {"AC33Q", ENUM_UMAC_LMAC_PORT_2, 0x4f},
	 {"ALTX Q0", ENUM_UMAC_LMAC_PORT_2, 0x70},
	 {"TF Q0", ENUM_UMAC_LMAC_PORT_2, 0x71},
	 {"TWT TSF-TF Q0", ENUM_UMAC_LMAC_PORT_2, 0x72},
	 {"TWT DL Q0", ENUM_UMAC_LMAC_PORT_2, 0x73},
	 {"TWT UL Q0", ENUM_UMAC_LMAC_PORT_2, 0x74},
	 {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	 {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	 {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}},

	{{"AC00Q", ENUM_UMAC_LMAC_PORT_2, 0x50},
	 {"AC01Q", ENUM_UMAC_LMAC_PORT_2, 0x51},
	 {"AC02Q", ENUM_UMAC_LMAC_PORT_2, 0x52},
	 {"AC03Q", ENUM_UMAC_LMAC_PORT_2, 0x53},
	 {"AC10Q", ENUM_UMAC_LMAC_PORT_2, 0x54},
	 {"AC11Q", ENUM_UMAC_LMAC_PORT_2, 0x55},
	 {"AC12Q", ENUM_UMAC_LMAC_PORT_2, 0x56},
	 {"AC13Q", ENUM_UMAC_LMAC_PORT_2, 0x57},
	 {"AC20Q", ENUM_UMAC_LMAC_PORT_2, 0x58},
	 {"AC21Q", ENUM_UMAC_LMAC_PORT_2, 0x59},
	 {"AC22Q", ENUM_UMAC_LMAC_PORT_2, 0x5a},
	 {"AC23Q", ENUM_UMAC_LMAC_PORT_2, 0x5b},
	 {"AC30Q", ENUM_UMAC_LMAC_PORT_2, 0x5c},
	 {"AC31Q", ENUM_UMAC_LMAC_PORT_2, 0x5d},
	 {"AC32Q", ENUM_UMAC_LMAC_PORT_2, 0x5e},
	 {"AC33Q", ENUM_UMAC_LMAC_PORT_2, 0x5f},
	 {"ALTX Q0", ENUM_UMAC_LMAC_PORT_2, 0x75},
	 {"TF Q0", ENUM_UMAC_LMAC_PORT_2, 0x76},
	 {"TWT TSF-TF Q0", ENUM_UMAC_LMAC_PORT_2, 0x77},
	 {"TWT DL Q0", ENUM_UMAC_LMAC_PORT_2, 0x78},
	 {"TWT UL Q0", ENUM_UMAC_LMAC_PORT_2, 0x79},
	 {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	 {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	 {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}},

	{{"AC00Q", ENUM_UMAC_LMAC_PORT_2, 0x60},
	 {"AC01Q", ENUM_UMAC_LMAC_PORT_2, 0x61},
	 {"AC02Q", ENUM_UMAC_LMAC_PORT_2, 0x62},
	 {"AC03Q", ENUM_UMAC_LMAC_PORT_2, 0x63},
	 {"AC10Q", ENUM_UMAC_LMAC_PORT_2, 0x64},
	 {"AC11Q", ENUM_UMAC_LMAC_PORT_2, 0x65},
	 {"AC12Q", ENUM_UMAC_LMAC_PORT_2, 0x66},
	 {"AC13Q", ENUM_UMAC_LMAC_PORT_2, 0x67},
	 {"AC20Q", ENUM_UMAC_LMAC_PORT_2, 0x68},
	 {"AC21Q", ENUM_UMAC_LMAC_PORT_2, 0x69},
	 {"AC22Q", ENUM_UMAC_LMAC_PORT_2, 0x6a},
	 {"AC23Q", ENUM_UMAC_LMAC_PORT_2, 0x6b},
	 {"AC30Q", ENUM_UMAC_LMAC_PORT_2, 0x6c},
	 {"AC31Q", ENUM_UMAC_LMAC_PORT_2, 0x6d},
	 {"AC32Q", ENUM_UMAC_LMAC_PORT_2, 0x6e},
	 {"AC33Q", ENUM_UMAC_LMAC_PORT_2, 0x6f},
	 {"ALTX Q0", ENUM_UMAC_LMAC_PORT_2, 0x7a},
	 {"TF Q0", ENUM_UMAC_LMAC_PORT_2, 0x7b},
	 {"TWT TSF-TF Q0", ENUM_UMAC_LMAC_PORT_2, 0x7c},
	 {"TWT DL Q0", ENUM_UMAC_LMAC_PORT_2, 0x7d},
	 {"TWT UL Q0", ENUM_UMAC_LMAC_PORT_2, 0x7e},
	 {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	 {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	 {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}}
};

static size_t
ple_cr_num_of_ac(struct mt76_dev *dev)
{
	switch (mt76_chip(dev)) {
	case MT7996_DEVICE_ID:
		return CR_NUM_OF_AC_MT7996;
	case MT7992_DEVICE_ID:
	case MT7990_DEVICE_ID:
	default:
		return CR_NUM_OF_AC_MT7992;
	}
}

static size_t
ple_cr_num_of_twt(struct mt76_dev *dev)
{
	switch (mt76_chip(dev)) {
	case MT7996_DEVICE_ID:
		return CR_NUM_OF_TWT_MT7996;
	case MT7990_DEVICE_ID:
		return CR_NUM_OF_TWT_MT7990;
	case MT7992_DEVICE_ID:
	default:
		return CR_NUM_OF_TWT_MT7992;
	}
}

static void
mt7996_show_ple_pg_info(struct mt7996_dev *dev, struct seq_file *s)
{
	u32 val[2];

	seq_printf(s, "PLE Configuration Info:\n");

	val[0] = mt76_rr(dev, WF_PLE_TOP_PBUF_CTRL_ADDR);
	seq_printf(s, "\tPacket Buffer Control: 0x%08x\n", val[0]);
	seq_printf(s, "\t\tPage size: %u bytes\n",
		   u32_get_bits(val[0], WF_PLE_TOP_PBUF_CTRL_PAGE_SIZE_CFG_MASK) ? 128 : 64);
	seq_printf(s, "\t\tPacket buffer offset: %u (unit: 2KB)\n",
		   u32_get_bits(val[0], WF_PLE_TOP_PBUF_CTRL_PBUF_OFFSET_MASK));
	seq_printf(s, "\t\tTotal number of pages: %u pages\n",
		   u32_get_bits(val[0], WF_PLE_TOP_PBUF_CTRL_TOTAL_PAGE_NUM_MASK));

	seq_printf(s, "PLE Page Flow Control:\n");

	val[0] = mt76_rr(dev, WF_PLE_TOP_FREEPG_CNT_ADDR);
	val[1] = mt76_rr(dev, WF_PLE_TOP_FREEPG_HEAD_TAIL_ADDR);
	seq_printf(s, "\tFree Page Counter: 0x%08x\n", val[0]);
	seq_printf(s, "\tFree Page Head and Tail: 0x%08x\n", val[1]);
	seq_printf(s, "\t\tNumber of free pages: 0x%04x\n",
		   u32_get_bits(val[0], WF_PLE_TOP_FREEPG_CNT_FREEPG_CNT_MASK));
	seq_printf(s, "\t\tNumber of unassigned pages: 0x%04x\n",
		   u32_get_bits(val[0], WF_PLE_TOP_FREEPG_CNT_FFA_CNT_MASK));
	seq_printf(s, "\t\tFID of tail/head free page: 0x%04x/0x%04x\n",
		   u32_get_bits(val[1], WF_PLE_TOP_FREEPG_HEAD_TAIL_FREEPG_TAIL_MASK),
		   u32_get_bits(val[1], WF_PLE_TOP_FREEPG_HEAD_TAIL_FREEPG_HEAD_MASK));

	val[0] = mt76_rr(dev, WF_PLE_TOP_PG_HIF_GROUP_ADDR);
	val[1] = mt76_rr(dev, WF_PLE_TOP_HIF_PG_INFO_ADDR);
	seq_printf(s, "\tReserved Page Counter of HIF Group: 0x%08x\n", val[0]);
	seq_printf(s, "\tHIF Group Page Status: 0x%08x\n", val[1]);
	seq_printf(s, "\t\tMax/min page quota for HIF group: 0x%04x/0x%04x\n",
		   u32_get_bits(val[0], WF_PLE_TOP_PG_HIF_GROUP_HIF_MAX_QUOTA_MASK),
		   u32_get_bits(val[0], WF_PLE_TOP_PG_HIF_GROUP_HIF_MIN_QUOTA_MASK));
	seq_printf(s, "\t\tUsed/free page count for HIF group: 0x%04x/0x%04x\n",
		   u32_get_bits(val[1], WF_PLE_TOP_HIF_PG_INFO_HIF_SRC_CNT_MASK),
		   u32_get_bits(val[1], WF_PLE_TOP_HIF_PG_INFO_HIF_RSV_CNT_MASK));

	val[0] = mt76_rr(dev, WF_PLE_TOP_PG_HIF_WMTXD_GROUP_ADDR);
	val[1] = mt76_rr(dev, WF_PLE_TOP_HIF_WMTXD_PG_INFO_ADDR);
	seq_printf(s, "\tReserved Page Counter of HIF WMCPU TXD Group: 0x%08x\n", val[0]);
	seq_printf(s, "\tHIF WMCPU TXD Group Page Status: 0x%08x\n", val[1]);
	seq_printf(s, "\t\tMax/min page quota for HIF WMCPU TXD group: 0x%04x/0x%04x\n",
		   u32_get_bits(val[0], WF_PLE_TOP_PG_HIF_WMTXD_GROUP_HIF_WMTXD_MAX_QUOTA_MASK),
		   u32_get_bits(val[0], WF_PLE_TOP_PG_HIF_WMTXD_GROUP_HIF_WMTXD_MIN_QUOTA_MASK));
	seq_printf(s, "\t\tUsed/free page count for HIF WMCPU TXD group: 0x%04x/0x%04x\n",
		   u32_get_bits(val[1], WF_PLE_TOP_HIF_WMTXD_PG_INFO_HIF_WMTXD_SRC_CNT_MASK),
		   u32_get_bits(val[1], WF_PLE_TOP_HIF_WMTXD_PG_INFO_HIF_WMTXD_RSV_CNT_MASK));

	val[0] = mt76_rr(dev, WF_PLE_TOP_PG_HIF_TXCMD_GROUP_ADDR);
	val[1] = mt76_rr(dev, WF_PLE_TOP_HIF_TXCMD_PG_INFO_ADDR);
	seq_printf(s, "\tReserved Page Counter of HIF TXCMD Group: 0x%08x\n", val[0]);
	seq_printf(s, "\tHIF TXCMD Group Page Status: 0x%08x\n", val[1]);
	seq_printf(s, "\t\tMax/min page quota for HIF TXCMD group: 0x%04x/0x%04x\n",
		   u32_get_bits(val[0], WF_PLE_TOP_PG_HIF_TXCMD_GROUP_HIF_TXCMD_MAX_QUOTA_MASK),
		   u32_get_bits(val[0], WF_PLE_TOP_PG_HIF_TXCMD_GROUP_HIF_TXCMD_MIN_QUOTA_MASK));
	seq_printf(s, "\t\tUsed/free page count for HIF TXCMD group: 0x%04x/0x%04x\n",
		   u32_get_bits(val[1], WF_PLE_TOP_HIF_TXCMD_PG_INFO_HIF_TXCMD_SRC_CNT_MASK),
		   u32_get_bits(val[1], WF_PLE_TOP_HIF_TXCMD_PG_INFO_HIF_TXCMD_RSV_CNT_MASK));

	val[0] = mt76_rr(dev, WF_PLE_TOP_PG_CPU_GROUP_ADDR);
	val[1] = mt76_rr(dev, WF_PLE_TOP_CPU_PG_INFO_ADDR);
	seq_printf(s, "\tReserved Page Counter of CPU Group: 0x%08x\n", val[0]);
	seq_printf(s, "\tCPU Group Page Status: 0x%08x\n", val[1]);
	seq_printf(s, "\t\tMax/min page quota for CPU group: 0x%04x/0x%04x\n",
		   u32_get_bits(val[0], WF_PLE_TOP_PG_CPU_GROUP_CPU_MAX_QUOTA_MASK),
		   u32_get_bits(val[0], WF_PLE_TOP_PG_CPU_GROUP_CPU_MIN_QUOTA_MASK));
	seq_printf(s, "\t\tUsed/free page count for CPU group: 0x%04x/0x%04x\n",
		   u32_get_bits(val[1], WF_PLE_TOP_CPU_PG_INFO_CPU_SRC_CNT_MASK),
		   u32_get_bits(val[1], WF_PLE_TOP_CPU_PG_INFO_CPU_RSV_CNT_MASK));
}

static void
mt7996_get_ple_acq_stat(struct mt7996_dev *dev, unsigned long *ple_stat)
{
	u32 i, addr;
	size_t cr_num_of_ac = ple_cr_num_of_ac(&dev->mt76);
	bool has_ext = is_mt7992(&dev->mt76) || is_mt7990(&dev->mt76);

	ple_stat[0] = mt76_rr(dev, WF_PLE_TOP_QUEUE_EMPTY_ADDR);

	/* Legacy */
	addr = WF_PLE_TOP_AC0_QUEUE_EMPTY0_ADDR;
	for (i = 1; i <= cr_num_of_ac; i++, addr += 4) {
		if (i == cr_num_of_ac && has_ext)
			ple_stat[i] = mt76_rr(dev, WF_PLE_TOP_AC0_QUEUE_EMPTY_EXT0_ADDR);
		else
			ple_stat[i] = mt76_rr(dev, addr);
	}

	addr = WF_PLE_TOP_AC1_QUEUE_EMPTY0_ADDR;
	for (; i <= cr_num_of_ac * 2; i++, addr += 4) {
		if (i == cr_num_of_ac * 2 && has_ext)
			ple_stat[i] = mt76_rr(dev, WF_PLE_TOP_AC1_QUEUE_EMPTY_EXT0_ADDR);
		else
			ple_stat[i] = mt76_rr(dev, addr);
	}

	addr = WF_PLE_TOP_AC2_QUEUE_EMPTY0_ADDR;
	for (; i <= cr_num_of_ac * 3; i++, addr += 4) {
		if (i == cr_num_of_ac * 3 && has_ext)
			ple_stat[i] = mt76_rr(dev, WF_PLE_TOP_AC2_QUEUE_EMPTY_EXT0_ADDR);
		else
			ple_stat[i] = mt76_rr(dev, addr);
	}

	addr = WF_PLE_TOP_AC3_QUEUE_EMPTY0_ADDR;
	for (; i <= cr_num_of_ac * 4; i++, addr += 4) {
		if (i == cr_num_of_ac * 4 && has_ext)
			ple_stat[i] = mt76_rr(dev, WF_PLE_TOP_AC3_QUEUE_EMPTY_EXT0_ADDR);
		else
			ple_stat[i] = mt76_rr(dev, addr);
	}
}

static void
mt7996_get_sta_pause(struct mt7996_dev *dev, u8 band, u32 *sta_pause, u32 *twt_pause)
{
	u32 i, addr;
	size_t cr_num_of_ac = ple_cr_num_of_ac(&dev->mt76);
	size_t cr_num_of_twt = ple_cr_num_of_twt(&dev->mt76);
	bool has_ext = is_mt7992(&dev->mt76) || is_mt7990(&dev->mt76);

	/* switch to target band */
	mt76_wr(dev, WF_DRR_TOP_SBRR_ADDR, u32_encode_bits(band, WF_DRR_TOP_SBRR_TARGET_BAND_MASK));

	/* Legacy */
	addr = WF_DRR_TOP_AC0_STATION_PAUSE00_ADDR;
	for (i = 0; i < cr_num_of_ac; i++, addr += 4) {
		if (i == cr_num_of_ac - 1 && has_ext)
			sta_pause[i] = mt76_rr(dev, WF_DRR_TOP_AC0_STATION_PAUSE_EXT_00_ADDR);
		else
			sta_pause[i] = mt76_rr(dev, addr);
	}

	addr = WF_DRR_TOP_AC1_STATION_PAUSE00_ADDR;
	for (; i < cr_num_of_ac * 2; i++, addr += 4) {
		if (i == cr_num_of_ac * 2 - 1 && has_ext)
			sta_pause[i] = mt76_rr(dev, WF_DRR_TOP_AC1_STATION_PAUSE_EXT_00_ADDR);
		else
			sta_pause[i] = mt76_rr(dev, addr);
	}

	addr = WF_DRR_TOP_AC2_STATION_PAUSE00_ADDR;
	for (; i < cr_num_of_ac * 3; i++, addr += 4) {
		if (i == cr_num_of_ac * 3 - 1 && has_ext)
			sta_pause[i] = mt76_rr(dev, WF_DRR_TOP_AC2_STATION_PAUSE_EXT_00_ADDR);
		else
			sta_pause[i] = mt76_rr(dev, addr);
	}

	addr = WF_DRR_TOP_AC3_STATION_PAUSE00_ADDR;
	for (; i < cr_num_of_ac * 4; i++, addr += 4) {
		if (i == cr_num_of_ac * 4 - 1 && has_ext)
			sta_pause[i] = mt76_rr(dev, WF_DRR_TOP_AC3_STATION_PAUSE_EXT_00_ADDR);
		else
			sta_pause[i] = mt76_rr(dev, addr);
	}

	/* TWT */
	addr = WF_DRR_TOP_TWT_STA_MAP00_ADDR;
	for (i = 0; i < cr_num_of_twt; i++, addr += 4) {
		if (i == cr_num_of_twt - 1 && has_ext)
			twt_pause[i] = mt76_rr(dev, WF_DRR_TOP_TWT_STA_MAP_EXT_00_ADDR);
		else
			twt_pause[i] = mt76_rr(dev, addr);
	}
}

static void
mt7996_get_ple_queue_info(struct mt7996_dev *dev, u32 pid, u32 qid, u32 tgid,
			  u16 wlan_idx, u16 *hfid, u16 *tfid, u16 *pktcnt)
{
	u32 val = WF_PLE_TOP_FL_QUE_CTRL_0_EXECUTE_MASK |
		  u32_encode_bits(pid, WF_PLE_TOP_FL_QUE_CTRL_0_Q_BUF_PID_MASK) |
		  u32_encode_bits(tgid, WF_PLE_TOP_FL_QUE_CTRL_0_Q_BUF_TGID_MASK) |
		  u32_encode_bits(qid, WF_PLE_TOP_FL_QUE_CTRL_0_Q_BUF_QID_MASK) |
		  u32_encode_bits(wlan_idx, WF_PLE_TOP_FL_QUE_CTRL_0_Q_BUF_WLANID_MASK);
	mt76_wr(dev, WF_PLE_TOP_FL_QUE_CTRL_0_ADDR, val);

	val = mt76_rr(dev, WF_PLE_TOP_FL_QUE_CTRL_2_ADDR);
	*hfid = u32_get_bits(val, WF_PLE_TOP_FL_QUE_CTRL_2_QUEUE_HEAD_FID_MASK);
	*tfid = u32_get_bits(val, WF_PLE_TOP_FL_QUE_CTRL_2_QUEUE_TAIL_FID_MASK);

	val = mt76_rr(dev, WF_PLE_TOP_FL_QUE_CTRL_3_ADDR);
	*pktcnt = u32_get_bits(val, WF_PLE_TOP_FL_QUE_CTRL_3_QUEUE_PKT_NUM_MASK);
}

static void
mt7996_show_sta_acq_info(struct seq_file *s, unsigned long *ple_stat,
			 u32 *sta_pause, u32 *twt_sta_pause)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	size_t cr_num_of_ac = ple_cr_num_of_ac(&dev->mt76);
	size_t cr_num_of_all_ac = cr_num_of_ac * IEEE80211_NUM_ACS;
	size_t cr_num_of_twt = ple_cr_num_of_twt(&dev->mt76);
	int i, j;

	for (j = 0; j < cr_num_of_all_ac; j++) { /* show AC Q info */
		for (i = 0; i < 32; i++) {
			if (!test_bit(i, &ple_stat[j + 1])) {
				u16 hfid, tfid, pktcnt, wlan_idx = i + (j % cr_num_of_ac) * 32;
				u8 wmmidx, ctrl = 0, acq_idx = j / cr_num_of_ac;
				struct mt7996_sta_link *msta_link;
				struct mt76_wcid *wcid;
				size_t idx;

				if (wlan_idx >= MT76_N_WCIDS) {
					seq_printf(s, "Error: WCID %hu exceeded threshold.\n", wlan_idx);
					continue;
				}
				wcid = rcu_dereference(dev->mt76.wcid[wlan_idx]);
				if (!wcid) {
					seq_printf(s, "Error: STA %hu does not exist.\n", wlan_idx);
					continue;
				}
				msta_link = container_of(wcid, struct mt7996_sta_link, wcid);
				wmmidx = msta_link->sta->vif->deflink.mt76.wmm_idx;

				seq_printf(s, "\tSTA%hu AC%hhu: ", wlan_idx, acq_idx);
				mt7996_get_ple_queue_info(dev, ENUM_UMAC_LMAC_PORT_2, acq_idx,
							  0, wlan_idx, &hfid, &tfid, &pktcnt);
				seq_printf(s, "tail/head fid = 0x%04x/0x%04x, pkt cnt = 0x%04x",
					   tfid, hfid, pktcnt);

				idx = wcid->phy_idx * cr_num_of_all_ac + j;
				if (sta_pause[idx] & BIT(i))
					ctrl = 2;

				idx = wcid->phy_idx * cr_num_of_twt + j % cr_num_of_twt;
				if (twt_sta_pause[idx] & BIT(i))
					ctrl = 3;

				seq_printf(s, ", ctrl = %s (wmmidx=%hhu, band=%hhu)\n",
					   sta_ctrl_reg[ctrl], wmmidx, wcid->phy_idx);
			}
		}
	}
}

static void
mt7996_show_txcmdq_info(struct seq_file *s)
{
	const u32 txcmd_queue_empty_addr[__MT_MAX_BAND][2] = {
		[MT_BAND0] = {WF_PLE_TOP_TXCMD_QUEUE_EMPTY_ADDR,
			      WF_PLE_TOP_NATIVE_TXCMD_QUEUE_EMPTY_ADDR},
		[MT_BAND1] = {WF_PLE_TOP_BN1_TXCMD_QUEUE_EMPTY_ADDR,
			      WF_PLE_TOP_BN1_NATIVE_TXCMD_QUEUE_EMPTY_ADDR},
		[MT_BAND2] = {WF_PLE_TOP_BN2_TXCMD_QUEUE_EMPTY_ADDR,
			      WF_PLE_TOP_BN2_NATIVE_TXCMD_QUEUE_EMPTY_ADDR}
	};
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	u8 band;

	for (band = MT_BAND0; band < __MT_MAX_BAND; ++band) {
		unsigned long txcmdq_stat, native_txcmdq_stat;
		int i;

		if (!dev->mt76.phys[band])
			continue;

		txcmdq_stat = mt76_rr(dev, txcmd_queue_empty_addr[band][0]);
		native_txcmdq_stat = mt76_rr(dev, txcmd_queue_empty_addr[band][1]);

		seq_printf(s, "Band%hhu Non-native/native TXCMD Queue Empty: 0x%08lx/0x%08lx\n",
			   band, txcmdq_stat, native_txcmdq_stat);

		for (i = 0; i < 32 ; i++) {
			if (!test_bit(i, &native_txcmdq_stat)) {
				struct bmac_queue_info_t *queue = &ple_txcmd_queue_empty_info[band][i];
				u16 hfid, tfid, pktcnt;

				if (!queue->QueueName)
					continue;

				seq_printf(s, "\t%s: ", queue->QueueName);
				mt7996_get_ple_queue_info(dev, queue->Portid, queue->Queueid,
							  0, 0, &hfid, &tfid, &pktcnt);
				seq_printf(s, "tail/head fid = 0x%04x/0x%04x, pkt cnt = 0x%04x\n",
					   tfid, hfid, pktcnt);
			}
		}
	}
}

static int
mt7996_pleinfo_read(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	size_t cr_num_of_ac = ple_cr_num_of_ac(&dev->mt76);
	size_t cr_num_of_all_ac = cr_num_of_ac * IEEE80211_NUM_ACS;
	size_t cr_num_of_twt = ple_cr_num_of_twt(&dev->mt76);
	u32 *sta_pause, *twt_sta_pause;
	unsigned long *ple_stat;
	int i, j, ret = 0;

	ple_stat = kzalloc((cr_num_of_all_ac + 1) * sizeof(unsigned long), GFP_KERNEL);
	if (!ple_stat)
		return -ENOMEM;

	sta_pause = kzalloc(__MT_MAX_BAND * cr_num_of_all_ac * sizeof(u32), GFP_KERNEL);
	if (!sta_pause) {
		ret = -ENOMEM;
		goto out;
	}

	twt_sta_pause = kzalloc(__MT_MAX_BAND * cr_num_of_twt * sizeof(u32), GFP_KERNEL);
	if (!twt_sta_pause) {
		ret = -ENOMEM;
		goto out;
	}

	mt7996_show_ple_pg_info(dev, s);
	mt7996_get_ple_acq_stat(dev, ple_stat);

	for (i = MT_BAND0; i < __MT_MAX_BAND; i++) {
		if (dev->mt76.phys[i])
			mt7996_get_sta_pause(dev, i,
					     sta_pause + i * cr_num_of_all_ac,
					     twt_sta_pause + i * cr_num_of_twt);
	}

	if ((ple_stat[0] & WF_PLE_TOP_QUEUE_EMPTY_ALL_AC_EMPTY_MASK) == 0) {
		for (j = 0; j < cr_num_of_all_ac; j++) {
			if (j % cr_num_of_ac == 0)
				seq_printf(s, "\n\tSTA in nonempty AC%ld TXD queue: ", j / cr_num_of_ac);

			for (i = 0; i < 32; i++) {
				if (!test_bit(i, &ple_stat[j + 1]))
					seq_printf(s, "%lu ", i + (j % cr_num_of_ac) * 32);
			}
		}
		seq_printf(s, "\n");
	}

	seq_printf(s, "Nonempty TXD Queue Info:\n");

	for (i = 0; i < 32; i++) {
		if (!test_bit(i, &ple_stat[0])) {
			struct bmac_queue_info *queue = &ple_queue_empty_info[i];
			u16 hfid, tfid, pktcnt;

			if (!queue->QueueName)
				continue;

			seq_printf(s, "\t%s: ", queue->QueueName);
			mt7996_get_ple_queue_info(dev, queue->Portid, queue->Queueid,
						  queue->tgid, 0, &hfid, &tfid, &pktcnt);
			seq_printf(s, "tail/head fid = 0x%04x/0x%04x, pkt cnt = 0x%04x\n",
				   tfid, hfid, pktcnt);
		}
	}

	mt7996_show_sta_acq_info(s, ple_stat, sta_pause, twt_sta_pause);
	mt7996_show_txcmdq_info(s);

	kfree(twt_sta_pause);
out:
	kfree(sta_pause);
	kfree(ple_stat);
	return ret;
}

static int
mt7996_tx_drop_show(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = s->private;
	struct mt76_dev *mdev = &dev->mt76;
	struct mt76_tx_debug *dev_stats = &mdev->tx_dbg_stats;
	struct mt76_tx_debug *phy_stats[__MT_MAX_BAND];
	int i = 0;

	seq_printf(s, "\t\t\t\t       dev");
	for (i = 0; i < __MT_MAX_BAND; i++) {
		seq_printf(s, "       Band%d", i);
		if (mdev->phys[i]) {
			phy_stats[i] = &mdev->phys[i]->tx_dbg_stats;
		} else {
			phy_stats[i] = kzalloc(sizeof(struct mt76_tx_debug),
					       GFP_KERNEL);
			if (!phy_stats[i])
				goto out;
		}

	}
	seq_printf(s, "       total\n");

	seq_printf(s, "%-30s%12d%12d%12d%12d%12d\n", "Receive from mac80211",
		       dev_stats->tx_from_mac80211,
		       phy_stats[0]->tx_from_mac80211,
		       phy_stats[1]->tx_from_mac80211,
		       phy_stats[2]->tx_from_mac80211,
		       dev_stats->tx_from_mac80211 +
		       phy_stats[0]->tx_from_mac80211 +
		       phy_stats[1]->tx_from_mac80211 +
		       phy_stats[2]->tx_from_mac80211);
	seq_printf(s, "%-30s%12d%12d%12d%12d%12d\n\n", "Send to hw",
		       dev_stats->tx_to_hw,
		       phy_stats[0]->tx_to_hw,
		       phy_stats[1]->tx_to_hw,
		       phy_stats[2]->tx_to_hw,
		       dev_stats->tx_to_hw +
		       phy_stats[0]->tx_to_hw +
		       phy_stats[1]->tx_to_hw +
		       phy_stats[2]->tx_to_hw);
#define __pr(t) seq_printf(s, "Drop due to %-18s%12d%12d%12d%12d%12d\n",\
			   #t, dev_stats->tx_drop[MT_TX_DROP_##t],	\
			   phy_stats[0]->tx_drop[MT_TX_DROP_##t],	\
			   phy_stats[1]->tx_drop[MT_TX_DROP_##t],	\
			   phy_stats[2]->tx_drop[MT_TX_DROP_##t],	\
			   dev_stats->tx_drop[MT_TX_DROP_##t] +		\
			   phy_stats[0]->tx_drop[MT_TX_DROP_##t] + 	\
			   phy_stats[1]->tx_drop[MT_TX_DROP_##t] +	\
			   phy_stats[2]->tx_drop[MT_TX_DROP_##t])

	__pr(IN_TESTMODE);
	__pr(WCID_NOT_INIT);
	__pr(STOPPED_QUEUE);
	__pr(RESET_STATE);
	__pr(GET_TXWI_FAIL);
	__pr(DMA_FAIL);
	__pr(AGG_EXCEEDED);
	__pr(RING_FULL);
	__pr(INVALID_SKB);
	__pr(GET_TOKEN_FAIL);
	__pr(ADDR_TRANS_FAIL);
	__pr(INVALID_WCID);
	__pr(INVALID_LINK);

#undef __pr
out:
	for (i = 0; i < __MT_MAX_BAND; i++) {
		if (!mdev->phys[i] && phy_stats[i])
			kfree(phy_stats[i]);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt7996_tx_drop);

static int
mt7996_rx_drop_show(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = s->private;
	struct mt76_dev *mdev = &dev->mt76;
	struct mt76_rx_debug *stats[__MT_MAX_BAND];
	struct mt76_queue *q[2];
	int i = 0;

	if (mtk_wed_device_active(&mdev->mmio.wed) &&
	    mdev->mmio.wed.version == MTK_WED_HW_V3_1)
		q[0] = &mdev->q_rx[MT_RXQ_WED_RX_DATA];
	else
		q[0] = &mdev->q_rx[MT_RXQ_MAIN];

	q[1] = is_mt7996(mdev) ? &mdev->q_rx[MT_RXQ_BAND2] :
				 &mdev->q_rx[MT_RXQ_BAND1];

	seq_printf(s, "\t\t\t\t   ");
	for (i = 0; i < 2; i++) {
		seq_printf(s, "        RXQ%d", q[i]->hw_idx);
	}
	seq_printf(s, "\n");

#define __pr(t) seq_printf(s, "Drop due to %-22s%12d%12d\n", #t, \
			   q[0]->rx_drop[MT_RX_DROP_##t],	\
			   q[1]->rx_drop[MT_RX_DROP_##t]);
	__pr(DMAD_RRO_REPEAT);
	__pr(DMAD_RRO_OLDPKT);
	__pr(DMAD_RRO_PN_CHK_FAIL);
	__pr(DMAD_WO_FRAG);
	__pr(DMAD_WO_DROP);
	__pr(DMAD_ADDR_NOT_FOUND);
	__pr(DMAD_TOKEN_NOT_FOUND);
	__pr(DMAD_GET_TOKEN_FAIL);
	__pr(DMAD_GET_RXWI_FAIL);
	__pr(DMAD_NOMEM);
	__pr(DMAD_DMA_MAPPING_FAIL);
	__pr(FRAG);
	__pr(BUILD_SKB_FAIL);
#undef __pr

	seq_printf(s, "\n\t\t\t\t   ");
	for (i = 0; i < __MT_MAX_BAND; i++) {
		seq_printf(s, "       Band%d", i);
		if (mdev->phys[i]) {
			stats[i] = &mdev->phys[i]->rx_dbg_stats;
		} else {
			stats[i] = kzalloc(sizeof(struct mt76_rx_debug),
					       GFP_KERNEL);
			if (!stats[i])
				goto out;
		}
	}
	seq_printf(s, "\n");
	seq_printf(s, "%-35s%12d%12d%12d\n", "Receive from hw",
		       stats[MT_BAND0]->rx_from_hw,
		       stats[MT_BAND1]->rx_from_hw,
		       stats[MT_BAND2]->rx_from_hw);
	seq_printf(s, "%-35s%12d%12d%12d\n\n", "Send to mac80211",
		       stats[MT_BAND0]->rx_to_mac80211,
		       stats[MT_BAND1]->rx_to_mac80211,
		       stats[MT_BAND2]->rx_to_mac80211);
#define __pr(t) seq_printf(s, "Drop due to %-22s%12d%12d%12d\n", #t, \
			   stats[MT_BAND0]->rx_drop[MT_RX_DROP_##t],	\
			   stats[MT_BAND1]->rx_drop[MT_RX_DROP_##t],	\
			   stats[MT_BAND2]->rx_drop[MT_RX_DROP_##t])
	__pr(RXD_ERR);
	__pr(STATE_ERR);
	__pr(RFC_PKT);
	__pr(AGG_SN_LESS);
	__pr(AGG_DUP);
#undef __pr

out:
	for (i = 0; i < __MT_MAX_BAND; i++) {
		if (!mdev->phys[i] && stats[i])
			kfree(stats[i]);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt7996_rx_drop);
/* DRR */
static int
mt7996_drr_info(struct seq_file *s, void *data)
{
	/* TODO: Wait MIB counter API implement complete */
	return 0;
}

static ssize_t mt7996_muru_dbg_info_set(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct mt7996_dev *dev = file->private_data;
	char buf[10];
	u16 item;
	u8 val;
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	if (count && buf[count - 1] == '\n')
		buf[count - 1] = '\0';
	else
		buf[count] = '\0';

	if (sscanf(buf, "%hu-%hhu", &item, &val) != 2) {
		dev_warn(dev->mt76.dev,"format: item-value\n");
		return -EINVAL;
	}

	ret = mt7996_mcu_muru_dbg_info(dev, item, val);
	if (ret) {
		dev_warn(dev->mt76.dev, "Fail to send mcu cmd.\n");
		return -EFAULT;
	}

	return count;
}

static const struct file_operations fops_muru_dbg_info = {
	.write = mt7996_muru_dbg_info_set,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static int mt7996_pp_alg_show(struct seq_file *s, void *data)
{
	struct mt7996_phy *phy = s->private;
	struct mt7996_dev *dev = phy->dev;

	dev_info(dev->mt76.dev, "pp_mode = %d\n", phy->pp_mode);
	mt7996_mcu_set_pp_alg_ctrl(phy, PP_ALG_GET_STATISTICS);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt7996_pp_alg);

static int mt7996_afc_table_show(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = s->private;

	char str[200] = {0}, *pos;
	char *end = str + sizeof(str);
	int i, j;

	if (!dev->mt76.afc_power_table || !dev->mt76.afc_power_table[0]) {
		seq_printf(s, "afc table doesn't exist.\n");
		return 0;
	}

	seq_printf(s, "bw/ru :    20    40    80   160  320-1 320-2   26    52    78   "
		   "106   132   726  1480  1772  2476  2988  3472\n");
	for(i = 0; i < MAX_CHANNEL_NUM_6G; i ++) {
		pos = str;
		for (j = 0; j < afc_power_table_num; j ++) {
			pos += snprintf(pos, end - pos, "%5d ",
					dev->mt76.afc_power_table[i][j]);
		}
		seq_printf(s, "ch %3d: %s\n", i * 4 + 1, str);
		memset(str, 0, sizeof(str));
	}
	seq_printf(s, "Unit : 0.5 dBm\n");
	seq_printf(s, "NOTE : power of the table is translated to single path.\n");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt7996_afc_table);

void mt7996_mtk_init_band_debugfs(struct mt7996_phy *phy, struct dentry *dir)
{
	/* agg */
	debugfs_create_file("agginfo", 0400, dir, phy, &mt7996_agginfo_fops);
	debugfs_create_file("mibinfo", 0400, dir, phy, &mt7996_mibinfo_fops);
	debugfs_create_file("txpower_level", 0600, dir, phy, &fops_txpower_level);
	debugfs_create_file("txpower_info", 0600, dir, phy, &mt7996_txpower_info_fops);
	debugfs_create_file("txpower_sku", 0600, dir, phy, &mt7996_txpower_sku_fops);
	debugfs_create_file("txpower_path", 0600, dir, phy, &mt7996_txpower_path_fops);

	debugfs_create_file("sr_enable", 0600, dir, phy, &fops_sr_enable);
	debugfs_create_file("sr_enhanced_enable", 0600, dir, phy, &fops_sr_enhanced_enable);
	debugfs_create_file("sr_stats", 0400, dir, phy, &mt7996_sr_stats_fops);
	debugfs_create_file("sr_scene_cond", 0400, dir, phy, &mt7996_sr_scene_cond_fops);

	debugfs_create_file("bf_txsnd_info", 0600, dir, phy, &fops_bf_txsnd_info);
	debugfs_create_file("bf_starec_read", 0600, dir, phy, &fops_starec_bf_read);
	debugfs_create_file("bf_fbk_rpt", 0600, dir, phy, &fops_bf_fbk_rpt);
	debugfs_create_file("pfmu_tag_read", 0600, dir, phy, &fops_bf_pfmu_tag_read);

	debugfs_create_file("thermal_enable", 0600, dir, phy, &fops_thermal_enable);
	debugfs_create_file("scs_enable", 0200, dir, phy, &fops_scs_enable);

	debugfs_create_file("pp_alg", 0200, dir, phy, &mt7996_pp_alg_fops);
}

void mt7996_mtk_init_dev_debugfs(struct mt7996_dev *dev, struct dentry *dir)
{
	u32 device_id = (dev->mt76.rev) >> 16;
	int i = 0;
	static const struct mt7996_dbg_reg_desc dbg_reg_s[] = {
		{ MT7996_DEVICE_ID, mt7996_dbg_offs },
		{ MT7992_DEVICE_ID, mt7992_dbg_offs },
		{ MT7990_DEVICE_ID, mt7990_dbg_offs },
	};

	for (i = 0; i < ARRAY_SIZE(dbg_reg_s); i++) {
		if (device_id == dbg_reg_s[i].id) {
			dev->dbg_reg = &dbg_reg_s[i];
			break;
		}
	}

	if (is_mt7996(&dev->mt76)) {
		WTBL_LMAC_DW2 = WTBL_LMAC_DW2_7996;
		WTBL_LMAC_DW5 = WTBL_LMAC_DW5_7996;
		WTBL_LMAC_DW9 = WTBL_LMAC_DW9_7996;
		WTBL_LMAC_DW35 = WTBL_LMAC_DW35_7996;
	} else {
		WTBL_LMAC_DW2 = WTBL_LMAC_DW2_7992;
		WTBL_LMAC_DW5 = WTBL_LMAC_DW5_7992;
		WTBL_LMAC_DW9 = WTBL_LMAC_DW9_7992;
		WTBL_LMAC_DW35 = WTBL_LMAC_DW35_7992;
	}

	/* amsdu */
	debugfs_create_devm_seqfile(dev->mt76.dev, "amsdu_info", dir,
				    mt7996_amsdu_result_read);

	debugfs_create_file("fw_debug_module", 0600, dir, dev,
			    &fops_fw_debug_module);
	debugfs_create_file("fw_debug_level", 0600, dir, dev,
			    &fops_fw_debug_level);
	debugfs_create_file("fw_wa_query", 0600, dir, dev, &fops_wa_query);
	debugfs_create_file("fw_wa_set", 0600, dir, dev, &fops_wa_set);
	debugfs_create_devm_seqfile(dev->mt76.dev, "fw_version", dir,
				    mt7996_dump_version);
	debugfs_create_devm_seqfile(dev->mt76.dev, "fw_wa_info", dir,
				    mt7996_fw_wa_info_read);
	debugfs_create_devm_seqfile(dev->mt76.dev, "fw_wm_info", dir,
				    mt7996_fw_wm_info_read);

	debugfs_create_devm_seqfile(dev->mt76.dev, "sta_info", dir,
				    mt7996_sta_info);

	debugfs_create_devm_seqfile(dev->mt76.dev, "tr_info", dir,
				    mt7996_trinfo_read);

	debugfs_create_devm_seqfile(dev->mt76.dev, "eeprom_mode", dir,
				    mt7996_show_eeprom_mode);

	debugfs_create_devm_seqfile(dev->mt76.dev, "wtbl_info", dir,
				    mt7996_wtbl_read);

	debugfs_create_devm_seqfile(dev->mt76.dev, "token", dir, mt7996_token_read);
	debugfs_create_file("red", 0200, dir, dev, &fops_red_config);
	debugfs_create_file("vow_drr_dbg", 0200, dir, dev, &fops_vow_drr_dbg);

#ifdef CONFIG_MTK_DEBUG
	dev->dbg.sku_disable = true; /* For SQC */
	debugfs_create_u8("sku_disable", 0600, dir, &dev->dbg.sku_disable);
#endif

	debugfs_create_file("muru_prot_thr", 0200, dir, dev, &fops_muru_prot_thr);
	debugfs_create_file("muru_fixed_rate_enable", 0600, dir, dev,
			    &fops_muru_fixed_rate_enable);
	debugfs_create_file("muru_fixed_group_rate", 0600, dir, dev,
			    &fops_muru_fixed_group_rate);

	if (mt7996_has_hwrro(dev)) {
		debugfs_create_u32("rro_sid", 0600, dir, &dev->dbg.sid);
		debugfs_create_devm_seqfile(dev->mt76.dev, "rro_sid_info", dir,
					    mt7996_rro_session_read);
		debugfs_create_devm_seqfile(dev->mt76.dev, "rro_mib", dir,
					    mt7996_show_rro_mib);
	}

	debugfs_create_file("thermal_recal", 0200, dir, dev, &fops_thermal_recal);
	debugfs_create_file("reset_counter", 0200, dir, dev, &fops_reset_counter);

	debugfs_create_devm_seqfile(dev->mt76.dev, "drr_info", dir,
				    mt7996_drr_info);

	debugfs_create_u32("token_idx", 0600, dir, &dev->dbg.token_idx);
	debugfs_create_devm_seqfile(dev->mt76.dev, "rx_token", dir,
				    mt7996_rx_token_read);

	debugfs_create_devm_seqfile(dev->mt76.dev, "ple_info", dir,
				    mt7996_pleinfo_read);
	debugfs_create_devm_seqfile(dev->mt76.dev, "pse_info", dir,
				    mt7996_pseinfo_read);
	/* amsdu */
	debugfs_create_file("amsdu_algo", 0600, dir, dev, &fops_amsdu_algo);
	debugfs_create_file("amsdu_para", 0600, dir, dev, &fops_amsdu_para);

	/* Drop counters */
	debugfs_create_file("tx_drop_stats", 0400, dir, dev, &mt7996_tx_drop_fops);
	debugfs_create_file("rx_drop_stats", 0400, dir, dev, &mt7996_rx_drop_fops);

	debugfs_create_file("muru_dbg", 0200, dir, dev, &fops_muru_dbg_info);
	debugfs_create_bool("mgmt_pwr_enhance", 0600, dir, &dev->mt76.mgmt_pwr_enhance);
	debugfs_create_file("afc_table", 0200, dir, dev, &mt7996_afc_table_fops);
}

#endif
