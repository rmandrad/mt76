/* SPDX-License-Identifier: ISC */
/*
 * Copyright (C) 2022 MediaTek Inc.
 */

#ifndef __MT7996_MCU_H
#define __MT7996_MCU_H

#include "../mt76_connac_mcu.h"

struct mt7996_mcu_rxd {
	__le32 rxd[8];

	__le16 len;
	__le16 pkt_type_id;

	u8 eid;
	u8 seq;
	u8 option;
	u8 __rsv;

	u8 ext_eid;
	u8 __rsv1[2];
	u8 s2d_index;
};

struct mt7996_mcu_uni_event {
	u8 cid;
	u8 __rsv[3];
	__le32 status; /* 0: success, others: fail */
} __packed;

struct mt7996_mcu_thermal_ctrl {
	u8 ctrl_id;
	u8 band_idx;
	union {
		struct {
			u8 protect_type; /* 1: duty admit, 2: radio off */
			u8 trigger_type; /* 0: low, 1: high */
		} __packed type;
		struct {
			u8 duty_level;	/* level 0~3 */
			u8 duty_cycle;
		} __packed duty;
	};
} __packed;

struct mt7996_mcu_thermal_enable {
	__le32 trigger_temp;
	__le32 restore_temp;
	__le16 sustain_time;
	u8 rsv[2];
} __packed;

struct mt7996_mcu_csa_notify {
	struct mt7996_mcu_rxd rxd;

	u8 omac_idx;
	u8 csa_count;
	u8 band_idx;
	u8 rsv;
} __packed;

struct mt7996_mcu_rdd_report {
	struct mt7996_mcu_rxd rxd;

	u8 __rsv1[4];

	__le16 tag;
	__le16 len;

	u8 rdd_idx;
	u8 long_detected;
	u8 constant_prf_detected;
	u8 staggered_prf_detected;
	u8 radar_type_idx;
	u8 periodic_pulse_num;
	u8 long_pulse_num;
	u8 hw_pulse_num;

	u8 out_lpn;
	u8 out_spn;
	u8 out_crpn;
	u8 out_crpw;
	u8 out_crbn;
	u8 out_stgpn;
	u8 out_stgpw;

	u8 __rsv2;

	__le32 out_pri_const;
	__le32 out_pri_stg[3];
	__le32 out_pri_stg_dmin;

	struct {
		__le32 start;
		__le16 pulse_width;
		__le16 pulse_power;
		u8 mdrdy_flag;
		u8 rsv[3];
	} long_pulse[32];

	struct {
		__le32 start;
		__le16 pulse_width;
		__le16 pulse_power;
		u8 mdrdy_flag;
		u8 rsv[3];
	} periodic_pulse[32];

	struct {
		__le32 start;
		__le16 pulse_width;
		__le16 pulse_power;
		u8 sc_pass;
		u8 sw_reset;
		u8 mdrdy_flag;
		u8 tx_active;
	} hw_pulse[32];
} __packed;

struct mt7996_rdd_ctrl {
	u8 _rsv[4];

	__le16 tag;
	__le16 len;

	u8 ctrl;
	u8 rdd_idx;
	u8 rdd_rx_sel;
	u8 val;
	u8 disable_timer;
	u8 rsv[3];
} __packed;

struct mt7996_mcu_background_chain_ctrl {
	u8 _rsv[4];

	__le16 tag;
	__le16 len;

	u8 chan;		/* primary channel */
	u8 central_chan;	/* central channel */
	u8 bw;
	u8 tx_stream;
	u8 rx_stream;

	u8 monitor_chan;	/* monitor channel */
	u8 monitor_central_chan;/* monitor central channel */
	u8 monitor_bw;
	u8 monitor_tx_stream;
	u8 monitor_rx_stream;

	u8 scan_mode;		/* 0: ScanStop
				 * 1: ScanStart
				 * 2: ScanRunning
				 */
	u8 band_idx;		/* DBDC */
	u8 monitor_scan_type;
	u8 band;		/* 0: 2.4GHz, 1: 5GHz */
	u8 rsv[2];
} __packed;

struct mt7996_mcu_eeprom_update {
	u8 _rsv[4];

	__le16 tag;
	__le16 len;
	u8 buffer_mode;		/* bit 0: efuse or buffer mode, bit 1: has patch back */
	u8 format;
	__le16 buf_len;
} __packed;

union eeprom_data {
	struct {
		__le32 data_len;
		DECLARE_FLEX_ARRAY(u8, data);
	} ext_eeprom;
	DECLARE_FLEX_ARRAY(u8, efuse);
} __packed;

struct mt7996_mcu_eeprom_info {
	u8 _rsv[4];

	__le16 tag;
	__le16 len;
	__le32 addr;
	__le32 valid;
} __packed;

struct mt7996_mcu_eeprom_access {
	struct mt7996_mcu_eeprom_info info;
	union eeprom_data eeprom;
} __packed;

struct mt7996_mcu_eeprom_access_event {
	u8 _rsv[4];

	__le16 tag;
	__le16 len;
	__le32 version;
	__le32 addr;
	__le32 valid;
	__le32 size;
	__le32 magic_no;
	__le32 type;
	__le32 rsv[4];
	union eeprom_data eeprom;
} __packed;

struct mt7996_mcu_eeprom_patch {
	__le16 tag;
	__le16 len;
	__le16 adie_offset;
	__le16 eep_offset;
	__le16 count;
	bool complete;
	u8 rsv;
} __packed;

struct mt7996_mcu_phy_rx_info {
	u8 category;
	u8 rate;
	u8 mode;
	u8 nsts;
	u8 gi;
	u8 coding;
	u8 stbc;
	u8 bw;
};

struct mt7996_mcu_mib {
	__le16 tag;
	__le16 len;
	__le32 offs;
	__le64 data;
} __packed;

struct per_sta_rssi {
	__le16 wlan_idx;
	u8 __rsv[2];
	u8 rcpi[IEEE80211_MAX_CHAINS];
} __packed;

struct per_sta_snr {
	__le16 wlan_idx;
	u8 __rsv[2];
	s8 val[IEEE80211_MAX_CHAINS];
} __packed;

struct per_sta_msdu_cnt {
	__le16 wlan_idx;
	u8 __rsv[2];
	__le32 tx_drops;
	__le32 tx_retries;
} __packed;

struct mt7996_mcu_per_sta_info_event {
	u8 __rsv[4];

	__le16 tag;
	__le16 len;

	union {
		struct per_sta_rssi rssi[0];
		struct per_sta_snr snr[0];
		struct per_sta_msdu_cnt msdu_cnt[0];
	};
} __packed;

struct all_sta_trx_rate {
	__le16 wlan_idx;
	u8 __rsv1[2];
	u8 tx_mode;
	u8 flags;
	u8 tx_stbc;
	u8 tx_gi;
	u8 tx_bw;
	u8 tx_ldpc;
	u8 tx_mcs;
	u8 tx_nss;
	u8 rx_rate;
	u8 rx_mode;
	u8 rx_nsts;
	u8 rx_gi;
	u8 rx_coding;
	u8 rx_stbc;
	u8 rx_bw;
	u8 __rsv2;
} __packed;

#define UNI_EVENT_SIZE_ADM_STAT_V1	1452

struct mt7996_mcu_all_sta_info_event {
	u8 rsv[4];
	__le16 tag;
	__le16 len;
	u8 more;
	u8 rsv2;
	__le16 sta_num;
	u8 rsv3[4];

	union {
		DECLARE_FLEX_ARRAY(struct all_sta_trx_rate, rate);
		DECLARE_FLEX_ARRAY(struct {
			__le16 wlan_idx;
			u8 rsv[2];
			__le32 tx_bytes[IEEE80211_NUM_ACS];
			__le32 rx_bytes[IEEE80211_NUM_ACS];
		} __packed, adm_stat_v1);

		DECLARE_FLEX_ARRAY(struct {
			__le16 wlan_idx;
			u8 rsv[2];
			__le32 tx_bytes[IEEE80211_NUM_ACS];
			__le32 rx_bytes[IEEE80211_NUM_ACS];
			__le32 tx_bytes_failed[IEEE80211_NUM_ACS];
		} __packed, adm_stat_v2);

		DECLARE_FLEX_ARRAY(struct {
			__le16 wlan_idx;
			u8 rsv[2];
			__le32 tx_msdu_cnt;
			__le32 rx_msdu_cnt;
		} __packed, msdu_cnt);

		DECLARE_FLEX_ARRAY(struct {
			__le16 wlan_idx;
			u8 rsv[2];
			__le32 tx[IEEE80211_NUM_ACS];
			__le32 rx[IEEE80211_NUM_ACS];
		} __packed, airtime);

		DECLARE_FLEX_ARRAY(struct {
			__le16 wlan_idx;
			u8 rsv[2];
			__le32 total;
			__le32 success;
		} __packed, rx_mpdu_cnt);
	} __packed;
} __packed;

struct mt7996_mcu_wed_rro_event {
	struct mt7996_mcu_rxd rxd;

	u8 __rsv1[4];

	__le16 tag;
	__le16 len;
} __packed;

struct mt7996_mcu_wed_rro_ba_event {
	__le16 tag;
	__le16 len;

	__le16 wlan_id;
	u8 tid;
	u8 __rsv1;
	__le32 status;
	__le16 id;
	u8 __rsv2[2];
} __packed;

struct mt7996_mcu_wed_rro_ba_delete_event {
	__le16 tag;
	__le16 len;

	__le16 session_id;
	__le16 mld_id;
	u8 tid;
	u8 __rsv[3];
} __packed;

enum  {
	UNI_WED_RRO_BA_SESSION_STATUS,
	UNI_WED_RRO_BA_SESSION_TBL,
	UNI_WED_RRO_BA_SESSION_DELETE,
};

struct mt7996_mcu_thermal_notify {
	struct mt7996_mcu_rxd rxd;

	u8 __rsv1[4];

	__le16 tag;
	__le16 len;

	u8 event_id;
	u8 band_idx;
	u8 level_idx;
	u8 duty_percent;
	__le32 restore_temp;
	u8 __rsv2[4];
} __packed;

enum mt7996_chan_mib_offs {
	UNI_MIB_OBSS_AIRTIME = 26,
	UNI_MIB_NON_WIFI_TIME = 27,
	UNI_MIB_TX_TIME = 28,
	UNI_MIB_RX_TIME = 29
};

struct edca {
	__le16 tag;
	__le16 len;

	u8 queue;
	u8 set;
	u8 cw_min;
	u8 cw_max;
	__le16 txop;
	u8 aifs;
	u8 __rsv;
};

#define MCU_PQ_ID(p, q)			(((p) << 15) | ((q) << 10))
#define MCU_PKT_ID			0xa0

enum {
	MCU_FW_LOG_WM,
	MCU_FW_LOG_WA,
	MCU_FW_LOG_TO_HOST,
	MCU_FW_LOG_RELAY = 16,
	MCU_FW_LOG_RELAY_IDX = 40
};

enum {
	MCU_TWT_AGRT_ADD,
	MCU_TWT_AGRT_MODIFY,
	MCU_TWT_AGRT_DELETE,
	MCU_TWT_AGRT_TEARDOWN,
	MCU_TWT_AGRT_GET_TSF,
};

enum {
	MCU_WA_PARAM_CMD_QUERY,
	MCU_WA_PARAM_CMD_SET,
	MCU_WA_PARAM_CMD_CAPABILITY,
	MCU_WA_PARAM_CMD_DEBUG,
};

#define BSS_ACQ_PKT_CNT_BSS_NUM		24
#define BSS_ACQ_PKT_CNT_BSS_BITMAP_ALL	0x00ffffff
#define BSS_ACQ_PKT_CNT_READ_CLR	BIT(31)
#define WMM_PKT_THRESHOLD		100

struct mt7996_mcu_bss_acq_pkt_cnt_event {
	struct mt7996_mcu_rxd rxd;

	__le32 bss_bitmap;
	struct {
		__le32 cnt[IEEE80211_NUM_ACS];
	} __packed bss[BSS_ACQ_PKT_CNT_BSS_NUM];
} __packed;

enum {
	MCU_WA_PARAM_PDMA_RX = 0x04,
	MCU_WA_PARAM_CPU_UTIL = 0x0b,
	MCU_WA_PARAM_RED_EN = 0x0e,
	MCU_WA_PARAM_BSS_ACQ_PKT_CNT = 0x12,
	MCU_WA_PARAM_HW_PATH_HIF_VER = 0x2f,
	MCU_WA_PARAM_RED_CONFIG = 0x40,
};

enum mcu_mmps_mode {
	MCU_MMPS_STATIC,
	MCU_MMPS_DYNAMIC,
	MCU_MMPS_RSV,
	MCU_MMPS_DISABLE,
};

struct bss_rate_tlv {
	__le16 tag;
	__le16 len;
	u8 __rsv1[4];
	__le16 bc_trans;
	__le16 mc_trans;
	u8 short_preamble;
	u8 bc_fixed_rate;
	u8 mc_fixed_rate;
	u8 __rsv2[9];
} __packed;

enum {
	BP_DISABLE,
	BP_SW_MODE,
	BP_HW_MODE,
};

struct bss_ra_tlv {
	__le16 tag;
	__le16 len;
	u8 short_preamble;
	u8 force_sgi;
	u8 force_gf;
	u8 ht_mode;
	u8 se_off;
	u8 antenna_idx;
	__le16 max_phyrate;
	u8 force_tx_streams;
	u8 __rsv[3];
} __packed;

struct bss_rlm_tlv {
	__le16 tag;
	__le16 len;
	u8 control_channel;
	u8 center_chan;
	u8 center_chan2;
	u8 bw;
	u8 tx_streams;
	u8 rx_streams;
	u8 ht_op_info;
	u8 sco;
	u8 band;
	u8 __rsv[3];
} __packed;

struct bss_color_tlv {
	__le16 tag;
	__le16 len;
	u8 enable;
	u8 color;
	u8 rsv[2];
} __packed;

struct bss_inband_discovery_tlv {
	__le16 tag;
	__le16 len;
	u8 tx_type;
	u8 tx_mode;
	u8 tx_interval;
	u8 enable;
	__le16 wcid;
	__le16 prob_rsp_len;
} __packed;

struct bss_bcn_content_tlv {
	__le16 tag;
	__le16 len;
	__le16 tim_ie_pos;
	__le16 csa_ie_pos;
	__le16 bcc_ie_pos;
	u8 enable;
	u8 type;
	__le16 pkt_len;
} __packed;

struct bss_bcn_cntdwn_tlv {
	__le16 tag;
	__le16 len;
	u8 cnt;
	u8 rsv[3];
} __packed;

struct bss_bcn_mbss_tlv {
	__le16 tag;
	__le16 len;
	__le32 bitmap;
#define MAX_BEACON_NUM	32
	__le16 offset[MAX_BEACON_NUM];
} __packed __aligned(4);

struct bss_bcn_crit_update_tlv {
	__le16 tag;
	__le16 len;
	__le32 bss_bitmap;
	/* Bypass the beacon sequence handling in firmware for the
	 * BSSes in the bitmap. If the flag is set for a BSS, then the
	 * firmware will not set the beacon of the BSS in sequence.
	 */
	__le32 bypass_seq_bitmap;
	__le16 tim_ie_pos[32];
	__le16 cap_info_ie_pos[32];
	bool require_event;
	u8 rsv[3];
} __packed;

struct bss_bcn_sta_prof_cntdwn_tlv {
	__le16 tag;
	__le16 len;
	__le16 sta_prof_csa_offs[__MT_MAX_BAND];
	u8 cs_bss_idx[__MT_MAX_BAND];
	u8 pkt_content[3];
} __packed;

struct bss_bcn_ml_reconf_tlv {
	__le16 tag;
	__le16 len;
	u8 reconf_count;
	u8 rsv[3];
	u8 offset[];
} __packed;

struct bss_bcn_ml_reconf_offset {
	__le16 ap_removal_timer_offs;
	u8 bss_idx;
	u8 rsv;
} __packed;

struct bss_bcn_attlm_offset_tlv {
	__le16 tag;
	__le16 len;
	u8 valid_id_bitmap;
	u8 rsv;
	__le16 offset;
} __packed;

struct bss_txcmd_tlv {
	__le16 tag;
	__le16 len;
	u8 txcmd_mode;
	u8 __rsv[3];
} __packed;

struct bss_sec_tlv {
	__le16 tag;
	__le16 len;
	u8 __rsv1[2];
	u8 cipher;
	u8 __rsv2[1];
} __packed;

struct bss_ifs_time_tlv {
	__le16 tag;
	__le16 len;
	u8 slot_valid;
	u8 sifs_valid;
	u8 rifs_valid;
	u8 eifs_valid;
	__le16 slot_time;
	__le16 sifs_time;
	__le16 rifs_time;
	__le16 eifs_time;
	u8 eifs_cck_valid;
	u8 rsv;
	__le16 eifs_cck_time;
} __packed;

struct bss_power_save {
	__le16 tag;
	__le16 len;
	u8 profile;
	u8 _rsv[3];
} __packed;

struct bss_mld_tlv {
	__le16 tag;
	__le16 len;
	u8 group_mld_id;
	u8 own_mld_id;
	u8 mac_addr[ETH_ALEN];
	u8 remap_idx;
	u8 link_id;
	u8 __rsv[2];
} __packed;

struct bss_mld_link_op_tlv {
	__le16 tag;
	__le16 len;
	u8 group_mld_id;
	u8 own_mld_id;
	u8 mac_addr[ETH_ALEN];
	u8 remap_idx;
	u8 link_operation;
	u8 link_id;
	u8 rsv[2];
} __packed;

struct sta_rec_ht_uni {
	__le16 tag;
	__le16 len;
	__le16 ht_cap;
	__le16 ht_cap_ext;
	u8 ampdu_param;
	u8 _rsv[3];
} __packed;

struct sta_rec_ba_uni {
	__le16 tag;
	__le16 len;
	u8 tid;
	u8 ba_type;
	u8 amsdu;
	u8 ba_en;
	__le16 ssn;
	__le16 winsize;
	u8 ba_rdd_rro;
	u8 __rsv[3];
} __packed;

struct sta_rec_tx_cap {
	__le16 tag;
	__le16 len;
	u8 ampdu_limit_en;
	u8 rsv[3];
} __packed;

struct sta_rec_eht {
	__le16 tag;
	__le16 len;
	u8 tid_bitmap;
	u8 _rsv;
	__le16 mac_cap;
	__le64 phy_cap;
	__le64 phy_cap_ext;
	u8 mcs_map_bw20[4];
	u8 mcs_map_bw80[3];
	u8 mcs_map_bw160[3];
	u8 mcs_map_bw320[3];
	u8 _rsv2[3];
} __packed;

struct sec_key_uni {
	__le16 wlan_idx;
	u8 mgmt_prot;
	u8 cipher_id;
	u8 cipher_len;
	u8 key_id;
	u8 key_len;
	u8 need_resp;
	u8 key[32];
	u8 pn[6];
	u8 bcn_mode;
	u8 _rsv;
} __packed;

struct sta_rec_sec_uni {
	__le16 tag;
	__le16 len;
	u8 add;
	u8 n_cipher;
	u8 rsv[2];

	struct sec_key_uni key[2];
} __packed;

struct sta_phy_uni {
	u8 type;
	u8 flag;
	u8 stbc;
	u8 sgi;
	u8 bw;
	u8 ldpc;
	u8 mcs;
	u8 nss;
	u8 he_ltf;
	u8 rsv[3];
};

struct sta_rec_ra_uni {
	__le16 tag;
	__le16 len;

	u8 valid;
	u8 auto_rate;
	u8 phy_mode;
	u8 channel;
	u8 bw;
	u8 disable_cck;
	u8 ht_mcs32;
	u8 ht_gf;
	u8 ht_mcs[4];
	u8 mmps_mode;
	u8 gband_256;
	u8 af;
	u8 auth_wapi_mode;
	u8 rate_len;

	u8 supp_mode;
	u8 supp_cck_rate;
	u8 supp_ofdm_rate;
	__le32 supp_ht_mcs;
	__le16 supp_vht_mcs[4];

	u8 op_mode;
	u8 op_vht_chan_width;
	u8 op_vht_rx_nss;
	u8 op_vht_rx_nss_type;

	__le32 sta_cap;

	struct sta_phy_uni phy;
	u8 rx_rcpi[4];
} __packed;

struct sta_rec_ra_fixed_uni {
	__le16 tag;
	__le16 len;

	__le32 field;
	u8 op_mode;
	u8 op_vht_chan_width;
	u8 op_vht_rx_nss;
	u8 op_vht_rx_nss_type;

	struct sta_phy_uni phy;

	u8 spe_idx;
	u8 short_preamble;
	u8 is_5g;
	u8 mmps_mode;
} __packed;

struct sta_rec_hdrt {
	__le16 tag;
	__le16 len;
	u8 hdrt_mode;
	u8 rsv[3];
} __packed;

struct sta_rec_hdr_trans {
	__le16 tag;
	__le16 len;
	u8 from_ds;
	u8 to_ds;
	u8 dis_rx_hdr_tran;
	u8 mesh;
} __packed;

struct sta_rec_ps_leave {
	__le16 tag;
	__le16 len;
	u8 __rsv[4];
} __packed;

struct sta_rec_mld_setup {
	__le16 tag;
	__le16 len;
	u8 mld_addr[ETH_ALEN];
	__le16 primary_id;
	__le16 seconed_id;
	__le16 setup_wcid;
	u8 link_num;
	u8 info;
	u8 __rsv[2];
	u8 link_info[];
} __packed;

struct sta_rec_eht_mld {
	__le16 tag;
	__le16 len;
	u8 nsep;
	u8 __rsv1[2];
	u8 str_cap[__MT_MAX_BAND];
	__le16 eml_cap;
	u8 __rsv2[4];
} __packed;

struct mld_setup_link {
	__le16 wcid;
	u8 bss_idx;
	u8 __rsv;
} __packed;

struct hdr_trans_en {
	__le16 tag;
	__le16 len;
	u8 enable;
	u8 check_bssid;
	u8 mode;
	u8 __rsv;
} __packed;

struct hdr_trans_vlan {
	__le16 tag;
	__le16 len;
	u8 insert_vlan;
	u8 remove_vlan;
	u8 tid;
	u8 __rsv;
} __packed;

struct hdr_trans_blacklist {
	__le16 tag;
	__le16 len;
	u8 idx;
	u8 enable;
	__le16 type;
} __packed;

struct uni_header {
	u8 __rsv[4];
} __packed;

struct vow_rx_airtime {
	__le16 tag;
	__le16 len;

	u8 enable;
	u8 band;
	u8 __rsv[2];
} __packed;

struct bf_sounding_on {
	__le16 tag;
	__le16 len;

	u8 snd_mode;
	u8 sta_num;
	u8 __rsv[2];
	__le16 wlan_id[4];
	__le32 snd_period;
} __packed;

enum sounding_mode {
	SU_SOUNDING,
	MU_SOUNDING,
	SU_PERIODIC_SOUNDING,
	MU_PERIODIC_SOUNDING,
	BF_PROCESSING,
	TXCMD_NONTB_SU_SOUNDING,
	TXCMD_VHT_MU_SOUNDING,
	TXCMD_TB_PER_BRP_SOUNDING,
	TXCMD_TB_SOUNDING,

	/* keep last */
	NUM_SOUNDING_MODE,
	SOUNDING_MODE_MAX = NUM_SOUNDING_MODE - 1,
};

struct bf_hw_en_status_update {
	__le16 tag;
	__le16 len;

	bool ebf;
	bool ibf;
	u8 __rsv[2];
} __packed;

struct bf_mod_en_ctrl {
	__le16 tag;
	__le16 len;

	u8 bf_num;
	u8 bf_bitmap;
	u8 bf_sel[8];
	u8 __rsv[2];
} __packed;

union bf_tag_tlv {
	struct bf_sounding_on bf_snd;
	struct bf_hw_en_status_update bf_hw_en;
	struct bf_mod_en_ctrl bf_mod_en;
};

enum {
	BF_SOUNDING_OFF = 0,
	BF_SOUNDING_ON = 1,
	BF_DATA_PACKET_APPLY = 2,
	BF_PFMU_TAG_READ = 5,
	BF_PFMU_TAG_WRITE = 6,
	BF_STA_REC_READ = 11,
	BF_PHASE_CALIBRATION = 12,
	BF_IBF_PHASE_COMP = 13,
	BF_PROFILE_WRITE_20M_ALL = 15,
	BF_HW_EN_UPDATE = 17,
	BF_MOD_EN_CTRL = 20,
	BF_FBRPT_DBG_INFO_READ = 23,
	BF_TXSND_INFO = 24,
	BF_CMD_TXCMD = 27,
	BF_CFG_PHY = 28,
	BF_PROFILE_WRITE_20M_ALL_5X5 = 30,
};

struct ra_rate {
	__le16 wlan_idx;
	u8 mode;
	u8 stbc;
	__le16 gi;
	u8 bw;
	u8 ldpc;
	u8 mcs;
	u8 nss;
	__le16 ltf;
	u8 spe;
	u8 preamble;
	u8 __rsv[2];
} __packed;

struct ra_fixed_rate {
	__le16 tag;
	__le16 len;

	__le16 version;
	struct ra_rate rate;
} __packed;

enum {
	UNI_RA_FIXED_RATE = 0xf,
};

enum {
	UNI_MURU_PST_LOW_POWER = 0x6e,
};

#define MT7996_HDR_TRANS_MAX_SIZE	(sizeof(struct hdr_trans_en) +	 \
					 sizeof(struct hdr_trans_vlan) + \
					 sizeof(struct hdr_trans_blacklist))

enum {
	UNI_HDR_TRANS_EN,
	UNI_HDR_TRANS_VLAN,
	UNI_HDR_TRANS_BLACKLIST,
};

enum {
	RATE_PARAM_VHT_OMN_UPDATE = 1,
	RATE_PARAM_FIXED = 3,
	RATE_PARAM_MMPS_UPDATE = 5,
	RATE_PARAM_FIXED_HE_LTF = 7,
	RATE_PARAM_FIXED_MCS,
	RATE_PARAM_FIXED_GI = 11,
	RATE_PARAM_FIXED_ENCODING,
	RATE_PARAM_AUTO = 20,
#ifdef CONFIG_MTK_VENDOR
	RATE_PARAM_FIXED_MIMO = 30,
	RATE_PARAM_FIXED_OFDMA = 31,
	RATE_PARAM_AUTO_MU = 32,
#endif
};

#define RATE_CFG_BAND_IDX	GENMASK(17, 16)
#define RATE_CFG_MODE	GENMASK(15, 8)
#define RATE_CFG_VAL	GENMASK(7, 0)

/* MURU */
#define OFDMA_DL                       BIT(0)
#define OFDMA_UL                       BIT(1)
#define MUMIMO_DL                      BIT(2)
#define MUMIMO_UL                      BIT(3)
#define MUMIMO_DL_CERT                 BIT(4)

enum {
	CMD_BAND_NONE,
	CMD_BAND_24G,
	CMD_BAND_5G,
	CMD_BAND_6G,
};

struct bss_req_hdr {
	u8 bss_idx;
	u8 __rsv[3];
} __packed;

enum {
	UNI_CHANNEL_SWITCH,
	UNI_CHANNEL_RX_PATH,
};

#define MT7996_BSS_UPDATE_MAX_SIZE	(sizeof(struct bss_req_hdr) +		\
					 sizeof(struct mt76_connac_bss_basic_tlv) +	\
					 sizeof(struct bss_rlm_tlv) +		\
					 sizeof(struct bss_ra_tlv) +		\
					 sizeof(struct bss_info_uni_he) +	\
					 sizeof(struct bss_rate_tlv) +		\
					 sizeof(struct bss_txcmd_tlv) +		\
					 sizeof(struct bss_power_save) +	\
					 sizeof(struct bss_sec_tlv) +		\
					 sizeof(struct bss_ifs_time_tlv) +	\
					 sizeof(struct bss_mld_tlv))

#define MT7996_STA_UPDATE_MAX_SIZE	(sizeof(struct sta_req_hdr) +		\
					 sizeof(struct sta_rec_basic) +		\
					 sizeof(struct sta_rec_bf) +		\
					 sizeof(struct sta_rec_ht_uni) +	\
					 sizeof(struct sta_rec_he_v2) +		\
					 sizeof(struct sta_rec_ba_uni) +	\
					 sizeof(struct sta_rec_vht) +		\
					 sizeof(struct sta_rec_uapsd) + 	\
					 sizeof(struct sta_rec_amsdu) +		\
					 sizeof(struct sta_rec_bfee) +		\
					 sizeof(struct sta_rec_ra_uni) +	\
					 sizeof(struct sta_rec_sec) +		\
					 sizeof(struct sta_rec_ra_fixed_uni) +	\
					 sizeof(struct sta_rec_he_6g_capa) +	\
					 sizeof(struct sta_rec_eht) +		\
					 sizeof(struct sta_rec_hdrt) +		\
					 sizeof(struct sta_rec_hdr_trans) +	\
					 sizeof(struct sta_rec_mld_setup) +	\
					 sizeof(struct mld_setup_link) * 3 +	\
					 sizeof(struct sta_rec_eht_mld) +	\
					 sizeof(struct sta_rec_tx_cap) +	\
					 sizeof(struct tlv))

#define MT7996_BEACON_UPDATE_SIZE	(sizeof(struct bss_req_hdr) +		\
					 sizeof(struct bss_bcn_content_tlv) +	\
					 4 + MT_TXD_SIZE +			\
					 sizeof(struct bss_bcn_cntdwn_tlv) +	\
					 sizeof(struct bss_bcn_mbss_tlv) +	\
					 sizeof(struct bss_bcn_crit_update_tlv) +	\
					 sizeof(struct bss_bcn_sta_prof_cntdwn_tlv) +	\
					 sizeof(struct bss_bcn_ml_reconf_tlv) +	\
					 3 * sizeof(struct bss_bcn_ml_reconf_offset))
#define MT7996_MAX_BSS_OFFLOAD_SIZE	2048
#define MT7996_MAX_BEACON_SIZE		(MT7996_MAX_BSS_OFFLOAD_SIZE - \
					 MT7996_BEACON_UPDATE_SIZE)

enum {
	UNI_BAND_CONFIG_RADIO_ENABLE,
	UNI_BAND_CONFIG_EDCCA_ENABLE = 0x05,
	UNI_BAND_CONFIG_EDCCA_THRESHOLD = 0x06,
	UNI_BAND_CONFIG_RTS_THRESHOLD = 0x08,
	UNI_BAND_CONFIG_RTS_SIGTA_EN = 0x09,
	UNI_BAND_CONFIG_DIS_SECCH_CCA_DET = 0x0a,
	UNI_BAND_CONFIG_LPI_CTRL = 0x0d,
	UNI_BAND_CONFIG_BSSID_MAPPING_ADDR = 0x12,
};

enum {
	UNI_WSYS_CONFIG_FW_LOG_CTRL,
	UNI_WSYS_CONFIG_FW_DBG_CTRL,
	UNI_CMD_CERT_CFG = 6,
	UNI_WSYS_CONFIG_FW_TIME_SYNC, /* UNI_CMD_FW_TIME_SYNC in FW */
};

enum {
	UNI_RDD_CTRL_PARM,
	UNI_RDD_CTRL_SET_TH = 0x3,
};

enum {
	UNI_EFUSE_ACCESS = 1,
	UNI_EFUSE_BUFFER_MODE,
	UNI_EFUSE_FREE_BLOCK,
	UNI_EFUSE_BUFFER_RD,
	UNI_EFUSE_PATCH,
};

enum {
	UNI_EXT_EEPROM_ACCESS = 1,
};

enum {
	UNI_VOW_DRR_CTRL,
	UNI_VOW_FEATURE_CTRL,
	UNI_VOW_RX_AT_AIRTIME_EN = 0x0b,
	UNI_VOW_RX_AT_AIRTIME_CLR_EN = 0x0e,
	UNI_VOW_RED_ENABLE = 0x18,
};

enum {
	UNI_CMD_MIB_DATA,
};

enum {
	UNI_POWER_OFF,
	UNI_POWER_LOW_POWER = 0x6,
};

enum {
	UNI_CMD_TWT_ARGT_UPDATE = 0x0,
	UNI_CMD_TWT_MGMT_OFFLOAD,
};

enum {
	UNI_RRO_DEL_ENTRY = 0x1,
	UNI_RRO_SET_PLATFORM_TYPE,
	UNI_RRO_GET_BA_SESSION_TABLE,
	UNI_RRO_SET_BYPASS_MODE,
	UNI_RRO_SET_TXFREE_PATH,
	UNI_RRO_DEL_BA_SESSION,
	UNI_RRO_SET_FLUSH_TIMEOUT
};

enum {
	UNI_MEC_READ_INFO = 0,
	UNI_MEC_AMSDU_ALGO_EN_STA,
	UNI_MEC_AMSDU_PARA_STA,
	UNI_MEC_AMSDU_ALGO_THRESHOLD,
	UNI_MEC_IFAC_SPEED,
};

enum{
	UNI_CMD_SR_ENABLE = 0x1,
	UNI_CMD_SR_ENABLE_SD,
	UNI_CMD_SR_ENABLE_MODE,
	UNI_CMD_SR_ENABLE_DPD = 0x12,
	UNI_CMD_SR_ENABLE_TX,
	UNI_CMD_SR_SET_SRG_BITMAP = 0x80,
	UNI_CMD_SR_SET_PARAM = 0xc1,
	UNI_CMD_SR_SET_SIGA = 0xd0,
};

enum {
	UNI_CMD_THERMAL_PROTECT_ENABLE = 0x6,
	UNI_CMD_THERMAL_PROTECT_DISABLE,
	UNI_CMD_THERMAL_PROTECT_DUTY_CONFIG,
};

enum {
	UNI_CMD_MAC_INFO_TSF_DIFF = 2,
};

enum {
	UNI_EVENT_MAC_INFO_TSF_DIFF = 2,
};

struct mt7996_mcu_mac_info_tsf_diff {
	__le16 tag;
	__le16 len;
	u8 bss_idx0;
	u8 bss_idx1;
	u8 rsv[2];
} __packed;

struct mt7996_mcu_mac_info_event {
	u8 rsv[4];
	u8 buf[];
} __packed;

struct mt7996_mcu_mac_info_tsf_diff_resp {
	__le16 tag;
	__le16 len;
	__le32 tsf0_bit0_31;
	__le32 tsf0_bit32_63;
	__le32 tsf1_bit0_31;
	__le32 tsf1_bit32_63;
} __packed;

struct mld_req_hdr {
	u8 ver;
	u8 mld_addr[ETH_ALEN];
	u8 mld_idx;
	u8 flag;
	u8 rsv[3];
	u8 buf[];
} __packed;

struct mld_attlm_req {
	__le16 tag;
	__le16 len;
	u8 attlm_idx;
	u8 bss_idx;
	u8 mst_timer;
	u8 e_timer;
	__le16 mst_timer_adv_time;
	__le16 e_timer_adv_time;
	__le32 mst_duration;
	__le32 e_duration;
	__le16 disabled_link_bitmap;
	u8 disabled_bss_idx[16];
	u8 rsv[2];
} __packed;

struct mld_reconf_timer {
	__le16 tag;
	__le16 len;
	__le16 link_bitmap;
	__le16 to_sec[__MT_MAX_BAND]; /* timeout of reconf (second) */
	u8 bss_idx[__MT_MAX_BAND];
	u8 rsv;
} __packed;

struct mld_reconf_stop_link {
	__le16 tag;
	__le16 len;
	__le16 link_bitmap;
	u8 rsv[2];
	u8 bss_idx[16];
} __packed;

enum {
	UNI_CMD_MLD_ATTLM_RES_REQ = 0x02,
	UNI_CMD_MLD_RECONF_AP_REM_TIMER = 0x03,
	UNI_CMD_MLD_RECONF_STOP_LINK = 0x04,
};

struct mt7996_mcu_mld_event {
	struct mt7996_mcu_rxd rxd;

	/* fixed field */
	u8 ver;
	u8 mld_addr[ETH_ALEN];
	u8 mld_idx;
	u8 rsv[4];
	/* tlv */
	u8 buf[];
} __packed;

struct mt7996_mld_event_data {
	u8 mld_addr[ETH_ALEN];
	u8 *data;
};

struct mt7996_mcu_sdo_event {
	struct mt7996_mcu_rxd rxd;

	/* fixed field */
	u8 rsv[4];
	/* tlv */
	u8 buf[];
} __packed;

#define UNI_CMD_SDO_CFG_BSS_NUM 96
#define UNI_CMD_SDO_CFG_BSS_MAP_WORDLEN ((UNI_CMD_SDO_CFG_BSS_NUM) / 32)

struct mt7996_mld_sdo_bss_acq_pkt_cnt {
	__le16 tag;
	__le16 len;

	__le32 bitmap[UNI_CMD_SDO_CFG_BSS_MAP_WORDLEN];
	__le32 pkt_cnt[UNI_CMD_SDO_CFG_BSS_NUM][IEEE80211_NUM_ACS];
};

enum {
	UNI_EVENT_SDO_BSS_ACQ_PKT_CNT,
};

struct mt7996_mcu_mld_attlm_resp_event {
	__le16 tag;
	__le16 len;
	u8 status;
	u8 attlm_idx;
	u8 bss_idx;
	u8 rsv;
	__le32 switch_time_tsf[2];
	__le32 end_tsf[2];
} __packed;

struct mt7996_mcu_mld_attlm_timeout_event {
	__le16 tag;
	__le16 len;
	u8 attlm_idx;
	u8 event_type;
	u8 rsv[2];
} __packed;

struct mt7996_mcu_mld_ap_reconf_event {
	__le16 tag;
	__le16 len;
	__le16 link_bitmap;
	u8 bss_idx[3];
	u8 rsv[3];
} __packed;

enum {
	UNI_EVENT_MLD_ATTLM_RES_RSP = 0x02,
	UNI_EVENT_MLD_ATTLM_TIMEOUT = 0x03,
	UNI_EVENT_MLD_RECONF_AP_REM_TIMER = 0x04,
};

struct peer_mld_req_hdr {
	u8 ver;
	u8 peer_mld_addr[ETH_ALEN];
	u8 mld_idx;
	u8 rsv[4];
	u8 buf[];
} __packed;

struct peer_mld_ttlm_req {
	__le16 tag;
	__le16 len;
	u8 mld_addr[ETH_ALEN];
	__le16 enabled_link_bitmap;
	__le16 link_to_wcid[IEEE80211_MLD_MAX_NUM_LINKS + 1];
	u8 dl_tid_map[IEEE80211_MLD_MAX_NUM_LINKS + 1];
	u8 ul_tid_map[IEEE80211_MLD_MAX_NUM_LINKS + 1];
} __packed;

enum {
	UNI_CMD_PEER_MLD_TTLM_REQ = 0x0,
};

struct mt7996_mcu_bss_event {
	struct mt7996_mcu_rxd rxd;

	/* fixed field */
	u8 bss_idx;
	u8 __rsv[3];
	/* tlv */
	u8 buf[];
} __packed;

struct mt7996_mcu_bss_bcn_crit_update_event {
	__le16 tag;
	__le16 len;
	u8 rsv[4];
} __packed;

enum {
	UNI_EVENT_BSS_BCN_CRIT_UPDATE = 0x01,
};

struct tx_power_ctrl {
	u8 _rsv[4];

	__le16 tag;
	__le16 len;

	u8 power_ctrl_id;
	union {
		bool sku_enable;
		bool ate_mode_enable;
		bool percentage_ctrl_enable;
		bool bf_backoff_enable;
		u8 show_info_category;
		u8 power_drop_level;
	};
	u8 band_idx;
	u8 rsv[1];
} __packed;

enum {
	UNI_TXPOWER_SKU_POWER_LIMIT_CTRL = 0,
	UNI_TXPOWER_PERCENTAGE_CTRL = 1,
	UNI_TXPOWER_PERCENTAGE_DROP_CTRL = 2,
	UNI_TXPOWER_BACKOFF_POWER_LIMIT_CTRL = 3,
	UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL = 4,
	UNI_TXPOWER_ATE_MODE_CTRL = 6,
	UNI_TXPOWER_SHOW_INFO = 7,
};

#define MAX_CHANNEL_NUM_6G 59
#define AFC_INVALID_POWER 127
enum afc_table_info {
	afc_power_bw20,
	afc_power_bw40,
	afc_power_bw80,
	afc_power_bw160,
	afc_power_bw320_1,
	afc_power_bw320_2,
	afc_power_ru26,
	afc_power_ru52,
	afc_power_ru78,
	afc_power_ru106,
	afc_power_ru132,
	afc_power_ru726,
	afc_power_ru1480,
	afc_power_ru1772,
	afc_power_ru2476,
	afc_power_ru2988,
	afc_power_ru3472,
	afc_power_table_num,
};

static inline int mt7996_get_bw_power_table_idx(int bw, int *mcs, int *ru, int *eht,
						int *path)
{
	switch (bw) {
	case afc_power_bw20:
		*mcs = 0;
		*ru = 3;
		*eht = 3;
		*path = 5;
		break;
	case afc_power_bw40:
		*mcs = 1;
		*ru = 4;
		*eht = 4;
		*path = 6;
		break;
	case afc_power_bw80:
		*mcs = 2;
		*ru = 5;
		*eht = 5;
		*path = 8;
		break;
	case afc_power_bw160:
		*mcs = 3;
		*ru = 6;
		*eht = 6;
		*path = 11;
		break;
	case afc_power_bw320_1:
		*mcs = -1;
		*ru = -1;
		*eht = 7;
		*path = 15;
		break;
	case afc_power_bw320_2:
		*mcs = -1;
		*ru = -1;
		*eht = 7;
		*path = 15;
		break;
	case afc_power_ru26:
		*mcs = -1;
		*ru = 0;
		*eht = 0;
		*path = 0;
		break;
	case afc_power_ru52:
		*mcs = -1;
		*ru = 1;
		*eht = 1;
		*path = 1;
		break;
	case afc_power_ru78:
		*mcs = -1;
		*ru = -1;
		*eht = 8;
		*path = 2;
		break;
	case afc_power_ru106:
		*mcs = -1;
		*ru = 2;
		*eht = 2;
		*path = 3;
		break;
	case afc_power_ru132:
		*mcs = -1;
		*ru = -1;
		*eht = 9;
		*path = 4;
		break;
	case afc_power_ru726:
		*mcs = -1;
		*ru = -1;
		*eht = 10;
		*path = 7;
		break;
	case afc_power_ru1480:
		*mcs = -1;
		*ru = -1;
		*eht = 11;
		*path = 9;
		break;
	case afc_power_ru1772:
		*mcs = -1;
		*ru = -1;
		*eht = 12;
		*path = 10;
		break;
	case afc_power_ru2476:
		*mcs = -1;
		*ru = -1;
		*eht = 13;
		*path = 12;
		break;
	case afc_power_ru2988:
		*mcs = -1;
		*ru = -1;
		*eht = 14;
		*path = 13;
		break;
	case afc_power_ru3472:
		*mcs = -1;
		*ru = -1;
		*eht = 15;
		*path = 14;
		break;
	default:
		*mcs = -1;
		*ru = -1;
		*eht = -1;
		*path = -1;
		return -EINVAL;
	}
	return 0;
}

enum {
	UNI_CMD_ACCESS_REG_BASIC = 0x0,
	UNI_CMD_ACCESS_RF_REG_BASIC,
};

enum {
	UNI_CMD_SER_QUERY,
	/* recovery */
	UNI_CMD_SER_SET_RECOVER_L1,
	UNI_CMD_SER_SET_RECOVER_L2,
	UNI_CMD_SER_SET_RECOVER_L3_RX_ABORT,
	UNI_CMD_SER_SET_RECOVER_L3_TX_ABORT,
	UNI_CMD_SER_SET_RECOVER_L3_TX_DISABLE,
	UNI_CMD_SER_SET_RECOVER_L3_BF,
	UNI_CMD_SER_SET_RECOVER_L4_MDP,
	UNI_CMD_SER_SET_RECOVER_FROM_ETH,
	UNI_CMD_SER_SET_RECOVER_FULL = 8,
	/* fw assert */
	UNI_CMD_SER_SET_SYSTEM_ASSERT,
	/* coredump */
	UNI_CMD_SER_FW_COREDUMP_WA,
	UNI_CMD_SER_FW_COREDUMP_WM,
	/*hw bit detect only*/
	UNI_CMD_SER_SET_HW_BIT_DETECT_ONLY,
	/* action */
	UNI_CMD_SER_ENABLE = 1,
	UNI_CMD_SER_SET,
	UNI_CMD_SER_TRIGGER
};

enum {
	UNI_CMD_SDO_SET = 1,
	UNI_CMD_SDO_QUERY,
	UNI_CMD_SDO_AUTO_BA,
	UNI_CMD_SDO_SET_QOS_MAP,
	UNI_CMD_SDO_HOTSPOT,
	UNI_CMD_SDO_CP_MODE,
	UNI_CMD_SDO_RED_SETTING,
	UNI_CMD_SDO_PKT_BUDGET_CTRL_CFG,
	UNI_CMD_SDO_GET_BSS_ACQ_PKT_NUM,
	UNI_CMD_SDO_OVERRIDE_CTRL
};

enum {
	MT7996_SEC_MODE_PLAIN,
	MT7996_SEC_MODE_AES,
	MT7996_SEC_MODE_SCRAMBLE,
	MT7996_SEC_MODE_MAX,
};

enum {
	UNI_CMD_PP_EN_CTRL,
	UNI_CMD_PP_ALG_CTRL,
	UNI_CMD_PP_DSCB_CTRL,
	UNI_CMD_PP_CHANGE_CAP_CTRL = 4,
};

enum pp_mode {
	PP_DISABLE = 0,
	PP_FW_MODE,
	PP_USR_MODE,
};

enum pp_alg_action {
	PP_ALG_SET_TIMER,
	PP_ALG_SET_THR,
	PP_ALG_GET_STATISTICS,
};

enum {
	UNI_EVENT_PP_TAG_ALG_CTRL = 1,
	UNI_EVENT_STATIC_PP_TAG_DSCB_IE,
	UNI_EVENT_STATIC_PP_TAG_CSA_DSCB_IE,
	UNI_EVENT_PP_SHOW_INFO,
};

struct mt7996_mcu_pp_basic_event {
	struct mt7996_mcu_rxd rxd;

	u8 __rsv1[4];

	__le16 tag;
	__le16 len;
	u8 band_idx;
	u8 __rsv2[3];
} __packed;

struct mt7996_mcu_pp_dscb_event {
	struct mt7996_mcu_rxd rxd;

	u8 __rsv1[4];

	__le16 tag;
	__le16 len;
	u8 band_idx;
	u8 omac_idx;
	u8 new_dscb;
	u8 __rsv2;
	__le16 punct_bitmap;
	u8 __rsv3[2];
} __packed;

struct mt7996_mcu_pp_alg_ctrl_event {
	struct mt7996_mcu_rxd rxd;

	u8 __rsv1[4];

	__le16 tag;
	__le16 len;

	__le32 pp_timer_intv;
	__le32 thr_x2_value;
	__le32 thr_x2_shift;
	__le32 thr_x3_value;
	__le32 thr_x3_shift;
	__le32 thr_x4_value;
	__le32 thr_x4_shift;
	__le32 thr_x5_value;
	__le32 thr_x5_shift;
	__le32 thr_x6_value;
	__le32 thr_x6_shift;
	__le32 thr_x7_value;
	__le32 thr_x7_shift;
	__le32 thr_x8_value;
	__le32 thr_x8_shift;
	__le32 sw_pp_time;
	__le32 hw_pp_time;
	__le32 no_pp_time;
	__le32 auto_bw_time;
	u8 band_idx;
	u8 __rsv2;
	__le16 punct_bitmap;
} __packed;

enum {
	UNI_CMD_SCS_SEND_DATA,
	UNI_CMD_SCS_SET_PD_THR_RANGE = 2,
	UNI_CMD_SCS_ENABLE,
};

enum {
	UNI_CMD_MLO_AGC_TX = 4,
	UNI_CMD_MLO_AGC_TRIG = 5,
};

struct mt7996_mlo_agc_set {
	u8 rsv[4];

	__le16 tag;
	__le16 len;

	u8 mld_id;
	u8 link_id;
	u8 ac;
	u8 disp_pol;
	u8 ratio;
	u8 order;
	__le16 mgf;
} __packed;

enum {
	UNI_CMD_TPO_CTRL,
};

enum {
	MT7996_LP_TPO_ALL,
	MT7996_LP_TPO_PB,
	MT7996_LP_TPO_LP,
	MT7996_LP_TPO_MIN_TX,
};

#define MT7996_PATCH_SEC		GENMASK(31, 24)
#define MT7996_PATCH_SCRAMBLE_KEY	GENMASK(15, 8)
#define MT7996_PATCH_AES_KEY		GENMASK(7, 0)

#define MT7996_SEC_ENCRYPT		BIT(0)
#define MT7996_SEC_KEY_IDX		GENMASK(2, 1)
#define MT7996_SEC_IV			BIT(3)

struct fixed_rate_table_ctrl {
	u8 _rsv[4];

	__le16 tag;
	__le16 len;

	u8 table_idx;
	u8 antenna_idx;
	__le16 rate_idx;
	u8 spe_idx_sel;
	u8 spe_idx;
	u8 gi;
	u8 he_ltf;
	bool ldpc;
	bool txbf;
	bool dynamic_bw;

	u8 _rsv2;
} __packed;

#define EPCS_MAX_WMM_PARAMS	16
#define EPCS_CTRL_WMM_FLAG_VALID	BIT(0)

enum {
	UNI_CMD_EPCS_CTRL = 1,
};

enum {
	UNI_EPCS_CTRL_ENABLE = 1,
	UNI_EPCS_CTRL_SET_WMM_PARAMS,
	UNI_EPCS_CTRL_SET_WMM_IDX,
	UNI_EPCS_CTRL_GET_WMM_PARAMS,
	UNI_EPCS_CTRL_GET_STA,
	UNI_EPCS_CTRL_RESET_STA
};

#ifdef CONFIG_MTK_VENDOR
struct mt7996_mcu_csi_event {
	struct mt7996_mcu_rxd rxd;

	u8 band_idx;
	u8 _rsv[3];

	__le16 tag;
	__le16 len;
	u8 tlv_buf[0];
};

enum UNI_EVENT_CSI_TAG_T {
	UNI_EVENT_CSI_DATA = 0,
	UNI_EVENT_CSI_MAX_NUM
};

struct csi_tlv {
	struct {
		__le32 tag;
		__le32 len;
	} basic;
	union {
		u8 mac[ETH_ALEN];
		__le32 info;
		s16 data[0];
	};
} __packed;

struct csi_bitmap_info_update {
	u8 action;
	u8 addr[ETH_ALEN];
};

#define CSI_MAX_BUF_NUM	3000

enum CSI_EVENT_TLV_TAG {
	CSI_EVENT_FW_VER,
	CSI_EVENT_CBW,
	CSI_EVENT_RSSI,
	CSI_EVENT_SNR,
	CSI_EVENT_BAND,
	CSI_EVENT_CSI_NUM,
	CSI_EVENT_CSI_I_DATA,
	CSI_EVENT_CSI_Q_DATA,
	CSI_EVENT_DBW,
	CSI_EVENT_CH_IDX,
	CSI_EVENT_TA,
	CSI_EVENT_EXTRA_INFO,
	CSI_EVENT_RX_MODE,
	CSI_EVENT_RSVD1,
	CSI_EVENT_RSVD2,
	CSI_EVENT_RSVD3,
	CSI_EVENT_RSVD4,
	CSI_EVENT_H_IDX,
	CSI_EVENT_TX_RX_IDX,
	CSI_EVENT_TS,
	CSI_EVENT_PKT_SN,
	CSI_EVENT_BW_SEG,
	CSI_EVENT_REMAIN_LAST,
	CSI_EVENT_TR_STREAM,
	CSI_EVENT_TLV_TAG_NUM,
};

enum CSI_CHAIN_TYPE {
	CSI_CHAIN_ERR,
	CSI_CHAIN_COMPLETE,
	CSI_CHAIN_SEGMENT_FIRST,
	CSI_CHAIN_SEGMENT_MIDDLE,
	CSI_CHAIN_SEGMENT_LAST,
	CSI_CHAIN_SEGMENT_ERR,
};

enum CSI_CONTROL_MODE_T {
	CSI_CONTROL_MODE_STOP,
	CSI_CONTROL_MODE_START,
	CSI_CONTROL_MODE_SET,
	CSI_CONTROL_MODE_NUM
};

enum CSI_CONFIG_ITEM_T {
	CSI_CONFIG_RSVD1,
	CSI_CONFIG_WF,
	CSI_CONFIG_RSVD2,
	CSI_CONFIG_FRAME_TYPE,
	CSI_CONFIG_TX_PATH,
	CSI_CONFIG_OUTPUT_FORMAT,
	CSI_CONFIG_INFO,
	CSI_CONFIG_CHAIN_FILTER,
	CSI_CONFIG_STA_FILTER,
	CSI_CONFIG_ACTIVE_MODE,
	CSI_CONFIG_ITEM_NUM
};

/* CSI config Tag */
enum UNI_CMD_CSI_TAG_T {
	UNI_CMD_CSI_STOP = 0,
	UNI_CMD_CSI_START = 1,
	UNI_CMD_CSI_SET_FRAME_TYPE = 2,
	UNI_CMD_CSI_SET_CHAIN_FILTER = 3,
	UNI_CMD_CSI_SET_STA_FILTER = 4,
	UNI_CMD_CSI_SET_ACTIVE_MODE = 5,
};
#endif

#endif
