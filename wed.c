// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2023 Lorenzo Bianconi <lorenzo@kernel.org>
 */

#include "mt76.h"
#include "dma.h"

void mt76_wed_release_rx_buf(struct mtk_wed_device *wed)
{
	struct mt76_dev *dev = container_of(wed, struct mt76_dev, mmio.wed);
	int i;

	pr_debug("Releasing RX buffers for WED\n");

	for (i = 0; i < dev->rx_token_size; i++) {
		struct mt76_txwi_cache *t;

		t = mt76_rx_token_release(dev, i);
		if (!t || !t->ptr) {
			pr_debug("Skipping release for RX token %d (NULL or no pointer)\n", i);
			continue;
		}

		mt76_put_page_pool_buf(t->ptr, false);
		pr_debug("Released buffer for RX token %d\n", i);
		t->ptr = NULL;

		mt76_put_rxwi(dev, t);
	}

	mt76_free_pending_rxwi(dev);
	pr_debug("Completed releasing RX buffers\n");
}
EXPORT_SYMBOL_GPL(mt76_wed_release_rx_buf);

#ifdef CONFIG_NET_MEDIATEK_SOC_WED
u32 mt76_wed_init_rx_buf(struct mtk_wed_device *wed, int size)
{
	struct mt76_dev *dev = container_of(wed, struct mt76_dev, mmio.wed);
	struct mtk_wed_bm_desc *desc = wed->rx_buf_ring.desc;
	struct mt76_queue *q = &dev->q_rx[MT_RXQ_MAIN];
	int i, len = SKB_WITH_OVERHEAD(q->buf_size);
	struct mt76_txwi_cache *t = NULL;

	pr_debug("Initializing RX buffers, requested size: %d\n", size);

	for (i = 0; i < size; i++) {
		enum dma_data_direction dir;
		dma_addr_t addr;
		u32 offset;
		int token;
		void *buf;

		t = mt76_get_rxwi(dev);
		if (!t) {
			pr_debug("Failed to allocate RX work item at index %d\n", i);
			goto unmap;
		}

		buf = mt76_get_page_pool_buf(q, &offset, q->buf_size);
		if (!buf) {
			pr_debug("Failed to allocate page pool buffer at index %d\n", i);
			goto unmap;
		}

		addr = page_pool_get_dma_addr(virt_to_head_page(buf)) + offset;
		dir = page_pool_get_dma_dir(q->page_pool);
		dma_sync_single_for_device(dev->dma_dev, addr, len, dir);

		desc->buf0 = cpu_to_le32(addr);
		token = mt76_rx_token_consume(dev, buf, t, addr);
		if (token < 0) {
			mt76_put_page_pool_buf(buf, false);
			pr_debug("Token consumption failed at index %d\n", i);
			goto unmap;
		}

		token = FIELD_PREP(MT_DMA_CTL_TOKEN, token);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
		pr_debug("CONFIG_ARCH_DMA_ADDR_T_64BIT\n");
		token |= FIELD_PREP(MT_DMA_CTL_SDP0_H, addr >> 32);
#endif
		desc->token |= cpu_to_le32(token);

		pr_debug("Initialized RX buffer at index %d with token %d\n", i, token);

		desc++;
	}

	pr_debug("Completed RX buffer initialization\n");

	return 0;

unmap:
	if (t)
		mt76_put_rxwi(dev, t);
	mt76_wed_release_rx_buf(wed);
	pr_debug("RX buffer initialization failed, cleaned up resources\n");
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(mt76_wed_init_rx_buf);

int mt76_wed_offload_enable(struct mtk_wed_device *wed)
{
	struct mt76_dev *dev = container_of(wed, struct mt76_dev, mmio.wed);

	pr_debug("Enabling WED offload\n");

	spin_lock_bh(&dev->token_lock);
	dev->token_size = wed->wlan.token_start;
	spin_unlock_bh(&dev->token_lock);

	return !wait_event_timeout(dev->tx_wait, !dev->wed_token_count, HZ);
}
EXPORT_SYMBOL_GPL(mt76_wed_offload_enable);

int mt76_wed_dma_setup(struct mt76_dev *dev, struct mt76_queue *q, bool reset)
{
	int ret = 0, type, ring;
	u16 flags;

	pr_debug("Setting up DMA for WED\n");

	if (!q || !q->ndesc)
		return -EINVAL;

	flags = q->flags;
	if (!q->wed || !mtk_wed_device_active(q->wed)) {
		pr_debug("WED device not active, disabling WED flag for queue\n");
		q->flags &= ~MT_QFLAG_WED;
	}

	if (!(q->flags & MT_QFLAG_WED)) {
		pr_debug("WED flag not set for queue\n");
		return 0;
	}

	type = FIELD_GET(MT_QFLAG_WED_TYPE, q->flags);
	ring = FIELD_GET(MT_QFLAG_WED_RING, q->flags);

	pr_debug("Queue type: %d, ring: %d\n", type, ring);

	switch (type) {
	case MT76_WED_Q_TX:
		ret = mtk_wed_device_tx_ring_setup(q->wed, ring, q->regs,
						   reset);
		if (!ret)
			q->wed_regs = q->wed->tx_ring[ring].reg_base;
		break;
	case MT76_WED_Q_TXFREE:
		/* WED txfree queue needs ring to be initialized before setup */
		q->flags = 0;
		mt76_dma_queue_reset(dev, q);
		mt76_dma_rx_fill(dev, q, false);

		ret = mtk_wed_device_txfree_ring_setup(q->wed, q->regs);
		if (!ret)
			q->wed_regs = q->wed->txfree_ring.reg_base;
		break;
	case MT76_WED_Q_RX:
		ret = mtk_wed_device_rx_ring_setup(q->wed, ring, q->regs,
						   reset);
		if (!ret)
			q->wed_regs = q->wed->rx_ring[ring].reg_base;
		break;
	case MT76_WED_RRO_Q_DATA:
		q->flags &= ~MT_QFLAG_WED;
		__mt76_dma_queue_reset(dev, q, false);
		mtk_wed_device_rro_rx_ring_setup(q->wed, ring, q->regs);
		q->head = q->ndesc - 1;
		q->queued = q->head;
		break;
	case MT76_WED_RRO_Q_MSDU_PG:
		q->flags &= ~MT_QFLAG_WED;
		__mt76_dma_queue_reset(dev, q, false);
		mtk_wed_device_msdu_pg_rx_ring_setup(q->wed, ring, q->regs);
		q->head = q->ndesc - 1;
		q->queued = q->head;
		break;
	case MT76_WED_RRO_Q_IND:
		q->flags &= ~MT_QFLAG_WED;
		mt76_dma_queue_reset(dev, q);
		mt76_dma_rx_fill(dev, q, false);
		mtk_wed_device_ind_rx_ring_setup(q->wed, q->regs);
		break;
	default:
		pr_debug("Unsupported queue type: %d\n", type);
		ret = -EINVAL;
		break;
	}
	q->flags = flags;

	pr_debug("DMA setup completed with result: %d\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(mt76_wed_dma_setup);
#endif /*CONFIG_NET_MEDIATEK_SOC_WED */

void mt76_wed_offload_disable(struct mtk_wed_device *wed)
{
	struct mt76_dev *dev = container_of(wed, struct mt76_dev, mmio.wed);

	pr_debug("Disabling WED offload\n");

	spin_lock_bh(&dev->token_lock);
	dev->token_size = dev->drv->token_size;
	spin_unlock_bh(&dev->token_lock);

	pr_debug("WED offload disabled\n");
}
EXPORT_SYMBOL_GPL(mt76_wed_offload_disable);

void mt76_wed_reset_complete(struct mtk_wed_device *wed)
{
	struct mt76_dev *dev = container_of(wed, struct mt76_dev, mmio.wed);

	pr_debug("WED reset complete\n");
	complete(&dev->mmio.wed_reset_complete);
}
EXPORT_SYMBOL_GPL(mt76_wed_reset_complete);

int mt76_wed_net_setup_tc(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			  struct net_device *netdev, enum tc_setup_type type,
			  void *type_data)
{
	struct mt76_phy *phy = hw->priv;
	struct mtk_wed_device *wed = &phy->dev->mmio.wed;

	pr_debug("Setting up traffic control for netdev: %s, type: %d\n", netdev->name, type);

	if (!mtk_wed_device_active(wed)) {
		pr_debug("WED device not active, TC setup not supported\n");
		return -EOPNOTSUPP;
	}

	return mtk_wed_device_setup_tc(wed, netdev, type, type_data);
}
EXPORT_SYMBOL_GPL(mt76_wed_net_setup_tc);

void mt76_wed_dma_reset(struct mt76_dev *dev)
{
	struct mt76_mmio *mmio = &dev->mmio;

	pr_debug("Starting WED DMA reset\n");
	
	if (!test_bit(MT76_STATE_WED_RESET, &dev->phy.state))
		return;

	complete(&mmio->wed_reset);

	if (!wait_for_completion_timeout(&mmio->wed_reset_complete, 3 * HZ)) {
		dev_err(dev->dev, "wed reset complete timeout\n");
		pr_debug("WED reset timed out\n");
	} else {
		pr_debug("WED reset completed successfully\n");
	}

}
EXPORT_SYMBOL_GPL(mt76_wed_dma_reset);
