// SPDX-License-Identifier: GPL-2.0+
/* drivers/net/phy/realtek.c
 *
 * Driver for Realtek PHYs
 *
 */
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mii_timestamper.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/phy.h>
#include <linux/ptp_classify.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/skbuff.h>
#include <linux/udp.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define RTL821x_PHYSR				0x11
#define RTL821x_PHYSR_DUPLEX			BIT(13)
#define RTL821x_PHYSR_SPEED			GENMASK(15, 14)

#define RTL821x_INER				0x12
#define RTL8211B_INER_INIT			0x6400
#define RTL8211E_INER_LINK_STATUS		BIT(10)
#define RTL8211F_INER_LINK_STATUS		BIT(4)

#define RTL821x_INSR				0x13

#define RTL821x_EXT_PAGE_SELECT			0x1e
#define RTL821x_PAGE_SELECT			0x1f

#define RTL8211F_PHYCR1				0x18
#define RTL8211F_INSR				0x1d

#define RTL8211F_TX_DELAY			BIT(8)
#define RTL8211F_RX_DELAY			BIT(3)

#define RTL8211F_ALDPS_PLL_OFF			BIT(1)
#define RTL8211F_ALDPS_ENABLE			BIT(2)
#define RTL8211F_ALDPS_XTAL_OFF			BIT(12)

#define RTL8211E_CTRL_DELAY			BIT(13)
#define RTL8211E_TX_DELAY			BIT(12)
#define RTL8211E_RX_DELAY			BIT(11)

#define RTL8201F_ISR				0x1e
#define RTL8201F_IER				0x13

#define RTL8366RB_POWER_SAVE			0x15
#define RTL8366RB_POWER_SAVE_ON			BIT(12)

#define RTL_SUPPORTS_5000FULL			BIT(14)
#define RTL_SUPPORTS_2500FULL			BIT(13)
#define RTL_SUPPORTS_10000FULL			BIT(0)
#define RTL_ADV_2500FULL			BIT(7)
#define RTL_LPADV_10000FULL			BIT(11)
#define RTL_LPADV_5000FULL			BIT(6)
#define RTL_LPADV_2500FULL			BIT(5)

#define RTLGEN_SPEED_MASK			0x0630

#define RTL_GENERIC_PHYID			0x001cc800

/* RTL8211FSI/FSVG PTP 寄存器页 */
#define RTL8211F_E40_PAGE			0xe40
#define RTL8211F_E41_PAGE			0xe41
#define RTL8211F_E43_PAGE			0xe43
#define RTL8211F_E44_PAGE			0xe44

#define RTL8211F_PTP_CTL			0x10
#define RTL8211F_PTP_INER			0x11
#define RTL8211F_PTP_INSR			0x12
#define RTL8211F_SYNCE_CTL			0x13

#define RTL8211F_PTP_CLK_CFG			0x10
#define RTL8211F_PTP_CFG_NS_LO			0x11
#define RTL8211F_PTP_CFG_NS_HI			0x12
#define RTL8211F_PTP_CFG_S_LO			0x13
#define RTL8211F_PTP_CFG_S_MI			0x14
#define RTL8211F_PTP_CFG_S_HI			0x15

#define RTL8211F_PTP_TRX_TS_STA			0x10
#define RTL8211F_PTP_TRX_TS_INFO		0x11
#define RTL8211F_PTP_TRX_TS_SH			0x12
#define RTL8211F_PTP_TRX_TS_NS_LO		0x13
#define RTL8211F_PTP_TRX_TS_NS_HI		0x14
#define RTL8211F_PTP_TRX_TS_S_LO		0x15
#define RTL8211F_PTP_TRX_TS_S_MI		0x16
#define RTL8211F_PTP_TRX_TS_S_HI		0x17
#define RTL8211F_PTP_TRX_TS_SID			0x18

#define RTL8211F_DR_2STEP_INS			BIT(9)
#define RTL8211F_FU_2STEP_INS			BIT(8)
#define RTL8211F_SYNC_1STEP			BIT(6)
#define RTL8211F_PTPV2_LAYER2			BIT(4)
#define RTL8211F_PTPV2_UDPIPV4			BIT(3)
#define RTL8211F_PTP_ENABLE			BIT(0)

#define RTL8211F_TX_TIMESTAMP			BIT(3)
#define RTL8211F_RX_TIMESTAMP			BIT(2)
#define RTL8211F_SYNC_E_ENABLE			BIT(0)

#define RTL8211F_TXTS_SYNC_RDY			BIT(15)
#define RTL8211F_TXTS_DELAY_REQ_RDY		BIT(14)
#define RTL8211F_RXTS_SYNC_RDY			BIT(11)
#define RTL8211F_RXTS_DELAY_REQ_RDY		BIT(10)
#define RTL8211F_TRXTS_OVERWR_EN		BIT(4)
#define RTL8211F_TRXTS_TX			0
#define RTL8211F_TRXTS_RX			1
#define RTL8211F_TRXTS_RD			BIT(0)

#define RTL8211F_MAX_RXTS			32
#define RTL8211F_SKB_TIMESTAMP_TIMEOUT		2 /* jiffies */

MODULE_DESCRIPTION("Realtek PHY driver with RTL8211FSI/FSVG PTP draft");
MODULE_AUTHOR("people");
MODULE_LICENSE("GPL");

enum rtl8211f_ptp_clock_adj_mode {
	RTL8211F_NO_FUNCTION = 0,
	RTL8211F_RESERVED,
	RTL8211F_DIRECT_READ,
	RTL8211F_DIRECT_WRITE,
	RTL8211F_INCREMENT_STEP,
	RTL8211F_DECREMENT_STEP,
	RTL8211F_RATE_READ,
	RTL8211F_RATE_WRITE,
};

enum rtl8211f_msg_type {
	RTL8211F_MSG_SYNC = 0,
	RTL8211F_MSG_DELAY_REQ = 1,
};

struct rtl8211f_ptphdr {
	u8 tsmt;
	u8 ver;
	__be16 msglen;
	u8 domain;
	u8 reserved1;
	__be16 flags;
	__be64 correction;
	__be32 reserved2;
	__be64 clk_identity;
	__be16 src_port_id;
	__be16 seq_id;
	u8 ctrl;
	u8 log_interval;
} __packed;

struct rtl8211f_ptp {
	struct phy_device *phydev;
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info caps;
	struct gpio_desc *ptp_int_gpiod;
	struct delayed_work ts_work;
	struct sk_buff_head tx_queue;
	struct sk_buff_head rx_queue;
	struct list_head rxts;
	struct list_head rxpool;
	spinlock_t rx_ts_lock;
	spinlock_t tx_queue_lock;
	spinlock_t rx_queue_lock;
	struct mutex ts_mutex;
	struct completion complete;
	struct task_struct *irq_thread;
	struct rtl8211f_rxts *rx_pool_data;
	enum hwtstamp_tx_types tx_type;
	enum hwtstamp_rx_filters rx_filter;
	int irq;
	bool configured;
	bool irq_enabled;
};

struct rtl8211f_private {
	struct mii_timestamper mii_ts;
	struct rtl8211f_ptp *ptp;
};

struct rtl8211f_trxstamp_meta {
	u16 info;
	__be16 sh;
	__be16 sid;
	u8 domain;
	u8 tsmt;
};

struct rtl8211f_skb_info {
	unsigned long tmo;
};

struct rtl8211f_rxts {
	struct list_head list;
	unsigned long tmo;
	struct timespec64 ts;
	struct rtl8211f_trxstamp_meta meta;
	u8 msg_type;
};

static int rtl821x_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, RTL821x_PAGE_SELECT);
}

static int rtl821x_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, RTL821x_PAGE_SELECT, page);
}

static int rtl8201_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, RTL8201F_ISR);

	return (err < 0) ? err : 0;
}

static int rtl821x_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, RTL821x_INSR);

	return (err < 0) ? err : 0;
}

static int rtl8211f_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read_paged(phydev, 0xa43, RTL8211F_INSR);

	return (err < 0) ? err : 0;
}

static int rtl8201_config_intr(struct phy_device *phydev)
{
	u16 val;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		val = BIT(13) | BIT(12) | BIT(11);
	else
		val = 0;

	return phy_write_paged(phydev, 0x7, RTL8201F_IER, val);
}

static int rtl8211b_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER, RTL8211B_INER_INIT);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211e_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER, RTL8211E_INER_LINK_STATUS);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211f_config_intr(struct phy_device *phydev)
{
	u16 val;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		val = RTL8211F_INER_LINK_STATUS;
	else
		val = 0;

	return phy_write_paged(phydev, 0xa42, RTL821x_INER, val);
}

/*
 * ---------------------------
 * RTL8211FSI/FSVG PHY PTP 区域
 * ---------------------------
 *
 * 这部分逻辑直接并进单文件，目的是给你一个更接近“直接替换”的版本。
 * 如果后面正式并回内核，建议你仍然保留这些中文注释，后续维护会轻松很多。
 */
#if IS_ENABLED(CONFIG_NETWORK_PHY_TIMESTAMPING)
static struct rtl8211f_ptphdr *rtl8211f_get_ptp_header_l4(struct sk_buff *skb,
							  struct iphdr *iphdr,
							  struct udphdr *udphdr)
{
	if (iphdr->version != 4 || iphdr->protocol != IPPROTO_UDP)
		return NULL;

	return (struct rtl8211f_ptphdr *)(((u8 *)udphdr) + UDP_HLEN);
}

static struct rtl8211f_ptphdr *rtl8211f_get_ptp_header_tx(struct sk_buff *skb)
{
	struct ethhdr *ethhdr = eth_hdr(skb);

	if (ethhdr->h_proto == htons(ETH_P_1588))
		return (struct rtl8211f_ptphdr *)(((u8 *)ethhdr) +
						  skb_mac_header_len(skb));

	if (ethhdr->h_proto != htons(ETH_P_IP))
		return NULL;

	return rtl8211f_get_ptp_header_l4(skb, ip_hdr(skb), udp_hdr(skb));
}

static struct rtl8211f_ptphdr *rtl8211f_get_ptp_header_rx(struct sk_buff *skb,
						enum hwtstamp_rx_filters filter)
{
	struct iphdr *iphdr;
	struct udphdr *udphdr;

	if (filter == HWTSTAMP_FILTER_PTP_V2_L2_EVENT)
		return (struct rtl8211f_ptphdr *)skb->data;

	iphdr = (struct iphdr *)skb->data;
	udphdr = (struct udphdr *)(skb->data + iphdr->ihl * 4);

	return rtl8211f_get_ptp_header_l4(skb, iphdr, udphdr);
}

static struct rtl8211f_skb_info *rtl8211f_skb_info(struct sk_buff *skb)
{
	return (struct rtl8211f_skb_info *)skb->cb;
}

static int rtl8211f_get_trxstamp(struct ptp_clock_info *info, u8 msg_type,
				 bool rx, struct rtl8211f_trxstamp_meta *meta,
				 struct timespec64 *ts)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	int val;

	val = RTL8211F_TRXTS_OVERWR_EN | (msg_type << 2) |
	      ((rx ? RTL8211F_TRXTS_RX : RTL8211F_TRXTS_TX) << 1) |
	      RTL8211F_TRXTS_RD;
	phy_write_paged(ptp->phydev, RTL8211F_E43_PAGE, RTL8211F_PTP_TRX_TS_STA,
			val);

	if (meta) {
		meta->info = phy_read_paged(ptp->phydev, RTL8211F_E44_PAGE,
					    RTL8211F_PTP_TRX_TS_INFO);
		meta->sh = cpu_to_be16(phy_read_paged(ptp->phydev,
						      RTL8211F_E44_PAGE,
						      RTL8211F_PTP_TRX_TS_SH));
		meta->sid = cpu_to_be16(phy_read_paged(ptp->phydev,
						       RTL8211F_E44_PAGE,
						       RTL8211F_PTP_TRX_TS_SID));
		meta->domain = meta->info >> 8;
		meta->tsmt = meta->info & GENMASK(3, 0);
	}

	val = phy_read_paged(ptp->phydev, RTL8211F_E44_PAGE,
			     RTL8211F_PTP_TRX_TS_S_HI);
	ts->tv_sec = val;
	ts->tv_sec = (ts->tv_sec << 16) |
		     phy_read_paged(ptp->phydev, RTL8211F_E44_PAGE,
				    RTL8211F_PTP_TRX_TS_S_MI);
	ts->tv_sec = (ts->tv_sec << 16) |
		     phy_read_paged(ptp->phydev, RTL8211F_E44_PAGE,
				    RTL8211F_PTP_TRX_TS_S_LO);

	val = phy_read_paged(ptp->phydev, RTL8211F_E44_PAGE,
			     RTL8211F_PTP_TRX_TS_NS_HI);
	ts->tv_nsec = val & 0x3fff;
	ts->tv_nsec = (ts->tv_nsec << 16) |
		      phy_read_paged(ptp->phydev, RTL8211F_E44_PAGE,
				     RTL8211F_PTP_TRX_TS_NS_LO);

	return 0;
}

static int rtl8211f_match_hwstamp(struct rtl8211f_ptphdr *ptphdr,
				  const struct rtl8211f_trxstamp_meta *meta,
				  u8 msg_type)
{
	int score = 0;

	if (!ptphdr || ((ptphdr->tsmt & 0xf) != msg_type))
		return -1;

	score = 1;

	if (meta->tsmt == msg_type)
		score++;

	if (ptphdr->seq_id == meta->sid)
		score += 4;

	if (ptphdr->src_port_id == meta->sh)
		score += 2;

	if (ptphdr->domain == meta->domain)
		score++;

	return score;
}

static bool rtl8211f_rxts_expired(const struct rtl8211f_rxts *rxts)
{
	return time_after(jiffies, rxts->tmo);
}

static void rtl8211f_prune_rxts(struct rtl8211f_ptp *ptp)
{
	struct rtl8211f_rxts *rxts, *tmp;

	list_for_each_entry_safe(rxts, tmp, &ptp->rxts, list) {
		if (!rtl8211f_rxts_expired(rxts))
			break;

		list_del_init(&rxts->list);
		list_add_tail(&rxts->list, &ptp->rxpool);
	}
}

static bool rtl8211f_match_rxts_skb(struct sk_buff *skb,
				    enum hwtstamp_rx_filters filter,
				    const struct rtl8211f_rxts *rxts)
{
	struct rtl8211f_ptphdr *ptphdr;

	ptphdr = rtl8211f_get_ptp_header_rx(skb, filter);
	if (!ptphdr)
		return false;

	return rtl8211f_match_hwstamp(ptphdr, &rxts->meta, rxts->msg_type) >= 0;
}

static void rtl8211f_complete_rxskb(struct sk_buff *skb,
				    const struct timespec64 *ts)
{
	struct skb_shared_hwtstamps *shhwtstamps;

	shhwtstamps = skb_hwtstamps(skb);
	memset(shhwtstamps, 0, sizeof(*shhwtstamps));
	shhwtstamps->hwtstamp = ktime_set(ts->tv_sec, ts->tv_nsec);
	netif_rx_ni(skb);
}

static void rtl8211f_rx_timestamp_work(struct work_struct *work)
{
	struct rtl8211f_ptp *ptp =
		container_of(work, struct rtl8211f_ptp, ts_work.work);
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&ptp->rx_queue))) {
		struct rtl8211f_skb_info *skb_info = rtl8211f_skb_info(skb);

		if (!time_after(jiffies, skb_info->tmo)) {
			skb_queue_head(&ptp->rx_queue, skb);
			break;
		}

		netif_rx_ni(skb);
	}

	if (!skb_queue_empty(&ptp->rx_queue))
		schedule_delayed_work(&ptp->ts_work,
				      RTL8211F_SKB_TIMESTAMP_TIMEOUT);
}

static void rtl8211f_reset_pending_rxts(struct rtl8211f_ptp *ptp)
{
	struct rtl8211f_rxts *rxts, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&ptp->rx_ts_lock, flags);
	list_for_each_entry_safe(rxts, tmp, &ptp->rxts, list) {
		list_del_init(&rxts->list);
		list_add_tail(&rxts->list, &ptp->rxpool);
	}
	spin_unlock_irqrestore(&ptp->rx_ts_lock, flags);
}

static void rtl8211f_adjust_mode_set(struct ptp_clock_info *info, u8 mode)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);

	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CLK_CFG,
			(mode << 1) | 0x1);
}

static int rtl8211f_gettime(struct ptp_clock_info *info, struct timespec64 *ts)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	int val;

	mutex_lock(&ptp->ts_mutex);
	rtl8211f_adjust_mode_set(info, RTL8211F_DIRECT_READ);

	val = phy_read_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_HI);
	ts->tv_sec = val;
	ts->tv_sec = (ts->tv_sec << 16) |
		     phy_read_paged(ptp->phydev, RTL8211F_E41_PAGE,
				    RTL8211F_PTP_CFG_S_MI);
	ts->tv_sec = (ts->tv_sec << 16) |
		     phy_read_paged(ptp->phydev, RTL8211F_E41_PAGE,
				    RTL8211F_PTP_CFG_S_LO);

	val = phy_read_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_HI);
	ts->tv_nsec = val;
	ts->tv_nsec = (ts->tv_nsec << 16) |
		      phy_read_paged(ptp->phydev, RTL8211F_E41_PAGE,
				     RTL8211F_PTP_CFG_NS_LO);
	mutex_unlock(&ptp->ts_mutex);

	return 0;
}

static int rtl8211f_settime(struct ptp_clock_info *info,
			    const struct timespec64 *ts)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);

	mutex_lock(&ptp->ts_mutex);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_HI,
			(ts->tv_sec >> 32) & 0xffff);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_MI,
			(ts->tv_sec >> 16) & 0xffff);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_LO,
			ts->tv_sec & 0xffff);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_HI,
			(ts->tv_nsec >> 16) & 0xffff);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_LO,
			ts->tv_nsec & 0xffff);
	rtl8211f_adjust_mode_set(info, RTL8211F_DIRECT_WRITE);
	mutex_unlock(&ptp->ts_mutex);

	return 0;
}

static int rtl8211f_adjtime(struct ptp_clock_info *info, s64 delta)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	u64 sec_diff, nsec_diff;
	bool negative = false;

	if (delta < 0) {
		delta = -delta;
		negative = true;
	}

	sec_diff = div_u64(delta, NSEC_PER_SEC);
	nsec_diff = delta - sec_diff * NSEC_PER_SEC;

	mutex_lock(&ptp->ts_mutex);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_LO,
			nsec_diff & 0xffff);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_HI,
			(nsec_diff >> 16) & 0x3fff);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_LO,
			sec_diff & 0xffff);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_MI,
			(sec_diff >> 16) & 0xffff);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_HI,
			(sec_diff >> 32) & 0xffff);
	rtl8211f_adjust_mode_set(info, negative ? RTL8211F_DECREMENT_STEP :
					 RTL8211F_INCREMENT_STEP);
	mutex_unlock(&ptp->ts_mutex);

	return 0;
}

static int rtl8211f_adjfreq(struct ptp_clock_info *info, s32 delta)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);

	mutex_lock(&ptp->ts_mutex);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_HI,
			(delta >> 16) & 0xffff);
	phy_write_paged(ptp->phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_LO,
			delta & 0xffff);
	rtl8211f_adjust_mode_set(info, RTL8211F_RATE_WRITE);
	mutex_unlock(&ptp->ts_mutex);

	return 0;
}

static int rtl8211f_hwtstamp(struct mii_timestamper *mii_ts, struct ifreq *ifr)
{
	struct rtl8211f_private *priv =
		container_of(mii_ts, struct rtl8211f_private, mii_ts);
	struct rtl8211f_ptp *ptp = priv->ptp;
	struct hwtstamp_config cfg;
	u16 mode = 0;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;
	if (cfg.flags)
		return -EINVAL;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_OFF:
	case HWTSTAMP_TX_ON:
	case HWTSTAMP_TX_ONESTEP_SYNC:
		break;
	default:
		return -ERANGE;
	}

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		cfg.rx_filter = HWTSTAMP_FILTER_NONE;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
		mode = RTL8211F_PTPV2_UDPIPV4 | RTL8211F_PTP_ENABLE;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_L2_EVENT;
		mode = RTL8211F_PTPV2_LAYER2 | RTL8211F_PTP_ENABLE;
		break;
	default:
		return -ERANGE;
	}

	if (cfg.tx_type == HWTSTAMP_TX_ON)
		mode |= RTL8211F_DR_2STEP_INS | RTL8211F_FU_2STEP_INS |
			RTL8211F_PTP_ENABLE;
	else if (cfg.tx_type == HWTSTAMP_TX_ONESTEP_SYNC)
		mode |= RTL8211F_SYNC_1STEP | RTL8211F_PTP_ENABLE;

	phy_write_paged(ptp->phydev, RTL8211F_E40_PAGE, RTL8211F_PTP_CTL, mode);
	cancel_delayed_work_sync(&ptp->ts_work);

	spin_lock_irq(&ptp->tx_queue_lock);
	__skb_queue_purge(&ptp->tx_queue);
	__skb_queue_head_init(&ptp->tx_queue);
	spin_unlock_irq(&ptp->tx_queue_lock);

	spin_lock_irq(&ptp->rx_queue_lock);
	__skb_queue_purge(&ptp->rx_queue);
	__skb_queue_head_init(&ptp->rx_queue);
	spin_unlock_irq(&ptp->rx_queue_lock);
	rtl8211f_reset_pending_rxts(ptp);

	ptp->tx_type = cfg.tx_type;
	ptp->rx_filter = cfg.rx_filter;
	ptp->configured = mode != 0;

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static int rtl8211f_ts_info(struct mii_timestamper *mii_ts,
			    struct ethtool_ts_info *info)
{
	struct rtl8211f_private *priv =
		container_of(mii_ts, struct rtl8211f_private, mii_ts);

	info->phc_index = ptp_clock_index(priv->ptp->ptp_clock);
	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE |
				SOF_TIMESTAMPING_TX_SOFTWARE |
				SOF_TIMESTAMPING_RX_SOFTWARE |
				SOF_TIMESTAMPING_SOFTWARE;
	info->tx_types = BIT(HWTSTAMP_TX_OFF) |
			 BIT(HWTSTAMP_TX_ON) |
			 BIT(HWTSTAMP_TX_ONESTEP_SYNC);
	info->rx_filters = BIT(HWTSTAMP_FILTER_NONE) |
			   BIT(HWTSTAMP_FILTER_PTP_V2_L4_EVENT) |
			   BIT(HWTSTAMP_FILTER_PTP_V2_L2_EVENT);

	return 0;
}

static void rtl8211f_txtstamp(struct mii_timestamper *mii_ts,
			      struct sk_buff *skb, int type)
{
	struct rtl8211f_private *priv =
		container_of(mii_ts, struct rtl8211f_private, mii_ts);
	struct rtl8211f_skb_info *skb_info = rtl8211f_skb_info(skb);

	if (!priv->ptp->configured || priv->ptp->tx_type == HWTSTAMP_TX_OFF) {
		kfree_skb(skb);
		return;
	}

	skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
	skb_info->tmo = jiffies + RTL8211F_SKB_TIMESTAMP_TIMEOUT;
	spin_lock_irq(&priv->ptp->tx_queue_lock);
	__skb_queue_tail(&priv->ptp->tx_queue, skb);
	spin_unlock_irq(&priv->ptp->tx_queue_lock);
}

static bool rtl8211f_rxtstamp(struct mii_timestamper *mii_ts,
			      struct sk_buff *skb, int type)
{
	struct rtl8211f_private *priv =
		container_of(mii_ts, struct rtl8211f_private, mii_ts);
	struct rtl8211f_skb_info *skb_info = rtl8211f_skb_info(skb);
	struct rtl8211f_rxts *rxts, *tmp;
	struct rtl8211f_ptphdr *ptphdr;
	unsigned long flags;

	switch (skb->protocol) {
	case htons(ETH_P_IP):
	case htons(ETH_P_1588):
		break;
	default:
		return false;
	}

	if (!priv->ptp->configured || priv->ptp->rx_filter == HWTSTAMP_FILTER_NONE)
		return false;
	if (type == PTP_CLASS_NONE)
		return false;

	ptphdr = rtl8211f_get_ptp_header_rx(skb, priv->ptp->rx_filter);
	if (!ptphdr)
		return false;

	spin_lock_irqsave(&priv->ptp->rx_ts_lock, flags);
	rtl8211f_prune_rxts(priv->ptp);
	list_for_each_entry_safe(rxts, tmp, &priv->ptp->rxts, list) {
		if (!rtl8211f_match_rxts_skb(skb, priv->ptp->rx_filter, rxts))
			continue;

		list_del_init(&rxts->list);
		spin_unlock_irqrestore(&priv->ptp->rx_ts_lock, flags);
		rtl8211f_complete_rxskb(skb, &rxts->ts);
		spin_lock_irqsave(&priv->ptp->rx_ts_lock, flags);
		list_add_tail(&rxts->list, &priv->ptp->rxpool);
		spin_unlock_irqrestore(&priv->ptp->rx_ts_lock, flags);
		return true;
	}
	spin_unlock_irqrestore(&priv->ptp->rx_ts_lock, flags);

	skb_info->tmo = jiffies + RTL8211F_SKB_TIMESTAMP_TIMEOUT;
	spin_lock_irq(&priv->ptp->rx_queue_lock);
	__skb_queue_tail(&priv->ptp->rx_queue, skb);
	spin_unlock_irq(&priv->ptp->rx_queue_lock);
	schedule_delayed_work(&priv->ptp->ts_work, RTL8211F_SKB_TIMESTAMP_TIMEOUT);

	return true;
}

static int rtl8211f_tx_queue_handle(struct rtl8211f_ptp *ptp, u8 message_type)
{
	struct rtl8211f_trxstamp_meta meta;
	struct sk_buff *skb, *tmp, *best = NULL;
	struct rtl8211f_ptphdr *ptphdr;
	struct skb_shared_hwtstamps shhwtstamps;
	struct timespec64 ts;
	int best_score = -1;
	int score;

	rtl8211f_get_trxstamp(&ptp->caps, message_type, false, &meta, &ts);

	spin_lock_irq(&ptp->tx_queue_lock);
	if (skb_queue_empty(&ptp->tx_queue)) {
		spin_unlock_irq(&ptp->tx_queue_lock);
		return 0;
	}

	skb_queue_walk_safe(&ptp->tx_queue, skb, tmp) {
		struct rtl8211f_skb_info *skb_info = rtl8211f_skb_info(skb);

		if (time_after(jiffies, skb_info->tmo)) {
			__skb_unlink(skb, &ptp->tx_queue);
			kfree_skb(skb);
			continue;
		}

		ptphdr = rtl8211f_get_ptp_header_tx(skb);
		score = rtl8211f_match_hwstamp(ptphdr, &meta, message_type);
		if (score > best_score) {
			best = skb;
			best_score = score;
		}
	}

	if (best)
		__skb_unlink(best, &ptp->tx_queue);
	spin_unlock_irq(&ptp->tx_queue_lock);

	if (!best)
		return 0;

	memset(&shhwtstamps, 0, sizeof(shhwtstamps));
	shhwtstamps.hwtstamp = ktime_set(ts.tv_sec, ts.tv_nsec);
	skb_complete_tx_timestamp(best, &shhwtstamps);

	return 0;
}

static int rtl8211f_rx_ts_handle(struct rtl8211f_ptp *ptp, u8 message_type)
{
	struct rtl8211f_trxstamp_meta meta;
	struct rtl8211f_rxts *rxts;
	struct sk_buff *skb, *tmp, *best = NULL;
	struct timespec64 ts;
	unsigned long flags;

	rtl8211f_get_trxstamp(&ptp->caps, message_type, true, &meta, &ts);

	spin_lock_irq(&ptp->rx_queue_lock);
	skb_queue_walk_safe(&ptp->rx_queue, skb, tmp) {
		struct rtl8211f_ptphdr *ptphdr;
		int score;

		ptphdr = rtl8211f_get_ptp_header_rx(skb, ptp->rx_filter);
		score = rtl8211f_match_hwstamp(ptphdr, &meta, message_type);
		if (score >= 0) {
			__skb_unlink(skb, &ptp->rx_queue);
			best = skb;
			break;
		}
	}
	spin_unlock_irq(&ptp->rx_queue_lock);

	if (best) {
		rtl8211f_complete_rxskb(best, &ts);
		return 0;
	}

	spin_lock_irqsave(&ptp->rx_ts_lock, flags);
	rtl8211f_prune_rxts(ptp);
	if (list_empty(&ptp->rxpool)) {
		spin_unlock_irqrestore(&ptp->rx_ts_lock, flags);
		return 0;
	}

	rxts = list_first_entry(&ptp->rxpool, struct rtl8211f_rxts, list);
	list_del_init(&rxts->list);
	rxts->tmo = jiffies + RTL8211F_SKB_TIMESTAMP_TIMEOUT;
	rxts->ts = ts;
	rxts->meta = meta;
	rxts->msg_type = message_type;
	list_add_tail(&rxts->list, &ptp->rxts);
	spin_unlock_irqrestore(&ptp->rx_ts_lock, flags);

	return 0;
}

static irqreturn_t rtl8211f_ptp_irq_handler(int irq, void *data)
{
	struct rtl8211f_ptp *ptp = data;

	complete(&ptp->complete);
	return IRQ_HANDLED;
}

static int rtl8211f_irq_thread(void *arg)
{
	struct rtl8211f_ptp *ptp = arg;
	u32 val;

	while (!kthread_should_stop()) {
		if (!wait_for_completion_interruptible_timeout(&ptp->complete,
				msecs_to_jiffies(5000)))
			continue;

		phy_read_paged(ptp->phydev, RTL8211F_E40_PAGE, RTL8211F_PTP_INSR);
		val = phy_read_paged(ptp->phydev, RTL8211F_E43_PAGE,
				     RTL8211F_PTP_TRX_TS_STA);

		if (val & RTL8211F_RXTS_SYNC_RDY)
			rtl8211f_rx_ts_handle(ptp, RTL8211F_MSG_SYNC);
		if (val & RTL8211F_RXTS_DELAY_REQ_RDY)
			rtl8211f_rx_ts_handle(ptp, RTL8211F_MSG_DELAY_REQ);
		if (val & RTL8211F_TXTS_SYNC_RDY)
			rtl8211f_tx_queue_handle(ptp, RTL8211F_MSG_SYNC);
		if (val & RTL8211F_TXTS_DELAY_REQ_RDY)
			rtl8211f_tx_queue_handle(ptp, RTL8211F_MSG_DELAY_REQ);
	}

	return 0;
}

static int rtl8211f_ptp_irq_setup(struct rtl8211f_ptp *ptp)
{
	struct device *dev = &ptp->phydev->mdio.dev;
	int ret;

	ptp->ptp_int_gpiod = devm_gpiod_get_optional(dev, "ptp-int", GPIOD_IN);
	if (IS_ERR(ptp->ptp_int_gpiod))
		return PTR_ERR(ptp->ptp_int_gpiod);
	if (!ptp->ptp_int_gpiod)
		return -ENODEV;

	ptp->irq = gpiod_to_irq(ptp->ptp_int_gpiod);
	if (ptp->irq < 0)
		return ptp->irq;

	init_completion(&ptp->complete);
	ptp->irq_thread = kthread_run(rtl8211f_irq_thread, ptp, "rtl8211f-ptp");
	if (IS_ERR(ptp->irq_thread)) {
		ret = PTR_ERR(ptp->irq_thread);
		ptp->irq_thread = NULL;
		return ret;
	}

	ret = request_irq(ptp->irq, rtl8211f_ptp_irq_handler,
			  IRQF_SHARED | IRQF_TRIGGER_FALLING,
			  "rtl8211f-ptp", ptp);
	if (ret) {
		kthread_stop(ptp->irq_thread);
		ptp->irq_thread = NULL;
		return ret;
	}

	ptp->irq_enabled = true;
	return 0;
}

static const struct ptp_clock_info rtl8211f_caps = {
	.owner		= THIS_MODULE,
	.name		= "RTL8211F timer",
	.max_adj	= S32_MAX,
	.n_alarm	= 0,
	.n_pins		= 0,
	.n_ext_ts	= 0,
	.n_per_out	= 0,
	.pps		= 0,
	.adjfreq	= rtl8211f_adjfreq,
	.adjtime	= rtl8211f_adjtime,
	.gettime64	= rtl8211f_gettime,
	.settime64	= rtl8211f_settime,
};

static int rtl8211f_ptp_probe(struct phy_device *phydev)
{
	struct rtl8211f_private *priv;
	struct rtl8211f_ptp *ptp;

	priv = devm_kzalloc(&phydev->mdio.dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ptp = devm_kzalloc(&phydev->mdio.dev, sizeof(*ptp), GFP_KERNEL);
	if (!ptp)
		return -ENOMEM;

	ptp->rx_pool_data = devm_kcalloc(&phydev->mdio.dev, RTL8211F_MAX_RXTS,
					 sizeof(*ptp->rx_pool_data), GFP_KERNEL);
	if (!ptp->rx_pool_data)
		return -ENOMEM;

	ptp->phydev = phydev;
	INIT_DELAYED_WORK(&ptp->ts_work, rtl8211f_rx_timestamp_work);
	INIT_LIST_HEAD(&ptp->rxts);
	INIT_LIST_HEAD(&ptp->rxpool);
	mutex_init(&ptp->ts_mutex);
	skb_queue_head_init(&ptp->tx_queue);
	skb_queue_head_init(&ptp->rx_queue);
	spin_lock_init(&ptp->rx_ts_lock);
	spin_lock_init(&ptp->tx_queue_lock);
	spin_lock_init(&ptp->rx_queue_lock);
	memcpy(&ptp->caps, &rtl8211f_caps, sizeof(ptp->caps));
	{
		int i;

		for (i = 0; i < RTL8211F_MAX_RXTS; i++)
			list_add_tail(&ptp->rx_pool_data[i].list, &ptp->rxpool);
	}

	priv->ptp = ptp;
	priv->mii_ts.rxtstamp = rtl8211f_rxtstamp;
	priv->mii_ts.txtstamp = rtl8211f_txtstamp;
	priv->mii_ts.hwtstamp = rtl8211f_hwtstamp;
	priv->mii_ts.ts_info = rtl8211f_ts_info;

	phydev->priv = priv;
	phydev->mii_ts = &priv->mii_ts;

	phy_write_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_SYNCE_CTL,
			RTL8211F_SYNC_E_ENABLE);
	phy_write_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_PTP_INER,
			RTL8211F_TX_TIMESTAMP | RTL8211F_RX_TIMESTAMP);
	phy_write_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_PTP_CTL, 0);

	ptp->ptp_clock = ptp_clock_register(&ptp->caps, &phydev->mdio.dev);
	if (IS_ERR(ptp->ptp_clock))
		return PTR_ERR(ptp->ptp_clock);

	rtl8211f_ptp_irq_setup(ptp);
	return 0;
}

static void rtl8211f_ptp_remove(struct phy_device *phydev)
{
	struct rtl8211f_private *priv = phydev->priv;

	if (!priv || !priv->ptp)
		return;

	if (priv->ptp->irq_enabled)
		free_irq(priv->ptp->irq, priv->ptp);
	if (priv->ptp->irq_thread)
		kthread_stop(priv->ptp->irq_thread);
	cancel_delayed_work_sync(&priv->ptp->ts_work);
	if (priv->ptp->ptp_clock && !IS_ERR(priv->ptp->ptp_clock))
		ptp_clock_unregister(priv->ptp->ptp_clock);

	phydev->mii_ts = NULL;
}
#else
static inline int rtl8211f_ptp_probe(struct phy_device *phydev)
{
	return 0;
}

static inline void rtl8211f_ptp_remove(struct phy_device *phydev)
{
}
#endif

static int rtl8211_config_aneg(struct phy_device *phydev)
{
	int ret;

	ret = genphy_config_aneg(phydev);
	if (ret < 0)
		return ret;

	if (phydev->speed == SPEED_100 && phydev->autoneg == AUTONEG_DISABLE) {
		phy_write(phydev, 0x17, 0x2138);
		phy_write(phydev, 0x0e, 0x0260);
	} else {
		phy_write(phydev, 0x17, 0x2108);
		phy_write(phydev, 0x0e, 0x0000);
	}

	return 0;
}

static int rtl8211c_config_init(struct phy_device *phydev)
{
	return phy_set_bits(phydev, MII_CTRL1000,
			    CTL1000_ENABLE_MASTER | CTL1000_AS_MASTER);
}

static int rtl8211f_config_init(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	u16 val_txdly, val_rxdly;
	u16 val;
	int ret;

	phy_write(phydev, 0x1f, 0x0d04);
	phy_write(phydev, 0x11, 0x0000);
	phy_write(phydev, 0x1f, 0x0000);

	phy_write(phydev, 0x1f, 0x0d04);
	phy_write(phydev, 0x10, 0xc160);
	phy_write(phydev, 0x1f, 0x0000);

	val = RTL8211F_ALDPS_ENABLE | RTL8211F_ALDPS_PLL_OFF |
	      RTL8211F_ALDPS_XTAL_OFF;
	phy_modify_paged_changed(phydev, 0xa43, RTL8211F_PHYCR1, val, val);

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII:
		val_txdly = 0;
		val_rxdly = 0;
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		val_txdly = 0;
		val_rxdly = RTL8211F_RX_DELAY;
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val_txdly = RTL8211F_TX_DELAY;
		val_rxdly = 0;
		break;
	case PHY_INTERFACE_MODE_RGMII_ID:
		val_txdly = RTL8211F_TX_DELAY;
		val_rxdly = RTL8211F_RX_DELAY;
		break;
	default:
		return 0;
	}

	ret = phy_modify_paged_changed(phydev, 0xd08, 0x11, RTL8211F_TX_DELAY,
				       val_txdly);
	if (ret < 0) {
		dev_err(dev, "Failed to update the TX delay register\n");
		return ret;
	}

	ret = phy_modify_paged_changed(phydev, 0xd08, 0x15, RTL8211F_RX_DELAY,
				       val_rxdly);
	if (ret < 0) {
		dev_err(dev, "Failed to update the RX delay register\n");
		return ret;
	}

	return 0;
}

static int rtl821x_resume(struct phy_device *phydev)
{
	int ret;

	ret = genphy_resume(phydev);
	if (ret < 0)
		return ret;

	msleep(20);

	return 0;
}

static int rtl8211e_config_init(struct phy_device *phydev)
{
	int ret = 0, oldpage;
	u16 val;

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII:
		val = RTL8211E_CTRL_DELAY | 0;
		break;
	case PHY_INTERFACE_MODE_RGMII_ID:
		val = RTL8211E_CTRL_DELAY | RTL8211E_TX_DELAY | RTL8211E_RX_DELAY;
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		val = RTL8211E_CTRL_DELAY | RTL8211E_RX_DELAY;
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val = RTL8211E_CTRL_DELAY | RTL8211E_TX_DELAY;
		break;
	default:
		return 0;
	}

	oldpage = phy_select_page(phydev, 0x7);
	if (oldpage < 0)
		goto err_restore_page;

	ret = __phy_write(phydev, RTL821x_EXT_PAGE_SELECT, 0xa4);
	if (ret)
		goto err_restore_page;

	ret = __phy_modify(phydev, 0x1c,
			   RTL8211E_CTRL_DELAY | RTL8211E_TX_DELAY |
			   RTL8211E_RX_DELAY, val);
err_restore_page:
	return phy_restore_page(phydev, oldpage, ret);
}

static int rtl8211b_suspend(struct phy_device *phydev)
{
	phy_write(phydev, MII_MMD_DATA, BIT(9));
	return genphy_suspend(phydev);
}

static int rtl8211b_resume(struct phy_device *phydev)
{
	phy_write(phydev, MII_MMD_DATA, 0);
	return genphy_resume(phydev);
}

static int rtl8366rb_config_init(struct phy_device *phydev)
{
	int ret;

	ret = phy_set_bits(phydev, RTL8366RB_POWER_SAVE,
			   RTL8366RB_POWER_SAVE_ON);
	if (ret)
		dev_err(&phydev->mdio.dev, "error enabling power management\n");

	return ret;
}

static int rtlgen_get_speed(struct phy_device *phydev)
{
	int val;

	if (!phydev->link)
		return 0;

	val = phy_read_paged(phydev, 0xa43, 0x12);
	if (val < 0)
		return val;

	switch (val & RTLGEN_SPEED_MASK) {
	case 0x0000:
		phydev->speed = SPEED_10;
		break;
	case 0x0010:
		phydev->speed = SPEED_100;
		break;
	case 0x0020:
		phydev->speed = SPEED_1000;
		break;
	case 0x0200:
		phydev->speed = SPEED_10000;
		break;
	case 0x0210:
		phydev->speed = SPEED_2500;
		break;
	case 0x0220:
		phydev->speed = SPEED_5000;
		break;
	default:
		break;
	}

	return 0;
}

static int rtlgen_read_status(struct phy_device *phydev)
{
	int ret;

	ret = genphy_read_status(phydev);
	if (ret < 0)
		return ret;

	return rtlgen_get_speed(phydev);
}

static int rtlgen_read_mmd(struct phy_device *phydev, int devnum, u16 regnum)
{
	int ret;

	if (devnum == MDIO_MMD_PCS && regnum == MDIO_PCS_EEE_ABLE) {
		rtl821x_write_page(phydev, 0xa5c);
		ret = __phy_read(phydev, 0x12);
		rtl821x_write_page(phydev, 0);
	} else if (devnum == MDIO_MMD_AN && regnum == MDIO_AN_EEE_ADV) {
		rtl821x_write_page(phydev, 0xa5d);
		ret = __phy_read(phydev, 0x10);
		rtl821x_write_page(phydev, 0);
	} else if (devnum == MDIO_MMD_AN && regnum == MDIO_AN_EEE_LPABLE) {
		rtl821x_write_page(phydev, 0xa5d);
		ret = __phy_read(phydev, 0x11);
		rtl821x_write_page(phydev, 0);
	} else {
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static int rtlgen_write_mmd(struct phy_device *phydev, int devnum, u16 regnum,
			    u16 val)
{
	int ret;

	if (devnum == MDIO_MMD_AN && regnum == MDIO_AN_EEE_ADV) {
		rtl821x_write_page(phydev, 0xa5d);
		ret = __phy_write(phydev, 0x10, val);
		rtl821x_write_page(phydev, 0);
	} else {
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static int rtl822x_read_mmd(struct phy_device *phydev, int devnum, u16 regnum)
{
	int ret = rtlgen_read_mmd(phydev, devnum, regnum);

	if (ret != -EOPNOTSUPP)
		return ret;

	if (devnum == MDIO_MMD_PCS && regnum == MDIO_PCS_EEE_ABLE2) {
		rtl821x_write_page(phydev, 0xa6e);
		ret = __phy_read(phydev, 0x16);
		rtl821x_write_page(phydev, 0);
	} else if (devnum == MDIO_MMD_AN && regnum == MDIO_AN_EEE_ADV2) {
		rtl821x_write_page(phydev, 0xa6d);
		ret = __phy_read(phydev, 0x12);
		rtl821x_write_page(phydev, 0);
	} else if (devnum == MDIO_MMD_AN && regnum == MDIO_AN_EEE_LPABLE2) {
		rtl821x_write_page(phydev, 0xa6d);
		ret = __phy_read(phydev, 0x10);
		rtl821x_write_page(phydev, 0);
	}

	return ret;
}

static int rtl822x_write_mmd(struct phy_device *phydev, int devnum, u16 regnum,
			     u16 val)
{
	int ret = rtlgen_write_mmd(phydev, devnum, regnum, val);

	if (ret != -EOPNOTSUPP)
		return ret;

	if (devnum == MDIO_MMD_AN && regnum == MDIO_AN_EEE_ADV2) {
		rtl821x_write_page(phydev, 0xa6d);
		ret = __phy_write(phydev, 0x12, val);
		rtl821x_write_page(phydev, 0);
	}

	return ret;
}

static int rtl822x_get_features(struct phy_device *phydev)
{
	int val;

	val = phy_read_paged(phydev, 0xa61, 0x13);
	if (val < 0)
		return val;

	linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
			 phydev->supported, val & RTL_SUPPORTS_2500FULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_5000baseT_Full_BIT,
			 phydev->supported, val & RTL_SUPPORTS_5000FULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
			 phydev->supported, val & RTL_SUPPORTS_10000FULL);

	return genphy_read_abilities(phydev);
}

static int rtl822x_config_aneg(struct phy_device *phydev)
{
	int ret = 0;

	if (phydev->autoneg == AUTONEG_ENABLE) {
		u16 adv2500 = 0;

		if (linkmode_test_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
				      phydev->advertising))
			adv2500 = RTL_ADV_2500FULL;

		ret = phy_modify_paged_changed(phydev, 0xa5d, 0x12,
					       RTL_ADV_2500FULL, adv2500);
		if (ret < 0)
			return ret;
	}

	return __genphy_config_aneg(phydev, ret);
}

static int rtl822x_read_status(struct phy_device *phydev)
{
	int ret;

	if (phydev->autoneg == AUTONEG_ENABLE) {
		int lpadv = phy_read_paged(phydev, 0xa5d, 0x13);

		if (lpadv < 0)
			return lpadv;

		linkmode_mod_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
				 phydev->lp_advertising,
				 lpadv & RTL_LPADV_10000FULL);
		linkmode_mod_bit(ETHTOOL_LINK_MODE_5000baseT_Full_BIT,
				 phydev->lp_advertising,
				 lpadv & RTL_LPADV_5000FULL);
		linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
				 phydev->lp_advertising,
				 lpadv & RTL_LPADV_2500FULL);
	}

	ret = genphy_read_status(phydev);
	if (ret < 0)
		return ret;

	return rtlgen_get_speed(phydev);
}

static bool rtlgen_supports_2_5gbps(struct phy_device *phydev)
{
	int val;

	phy_write(phydev, RTL821x_PAGE_SELECT, 0xa61);
	val = phy_read(phydev, 0x13);
	phy_write(phydev, RTL821x_PAGE_SELECT, 0);

	return val >= 0 && val & RTL_SUPPORTS_2500FULL;
}

static int rtlgen_match_phy_device(struct phy_device *phydev)
{
	return phydev->phy_id == RTL_GENERIC_PHYID &&
	       !rtlgen_supports_2_5gbps(phydev);
}

static int rtl8226_match_phy_device(struct phy_device *phydev)
{
	return phydev->phy_id == RTL_GENERIC_PHYID &&
	       rtlgen_supports_2_5gbps(phydev);
}

static int rtlgen_resume(struct phy_device *phydev)
{
	int ret = genphy_resume(phydev);

	msleep(20);

	return ret;
}

static int rtl9010a_config_init(struct phy_device *phydev)
{
	phydev->autoneg = AUTONEG_DISABLE;
	phydev->speed = SPEED_1000;
	phydev->duplex = DUPLEX_FULL;

	return 0;
}

static int rtl9010a_config_aneg(struct phy_device *phydev)
{
	return 0;
}

static int rtl9010a_get_features(struct phy_device *phydev)
{
	linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT1_Full_BIT,
			 phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT1_Full_BIT,
			 phydev->supported);

	return 0;
}

static int rtl9010a_read_status(struct phy_device *phydev)
{
	int ret;
	u16 val;
	int link_status, local_status, remote_status;

	ret = genphy_read_status(phydev);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, 0x01);
	val = phy_read(phydev, 0x01);
	link_status = val & 0x0004 ? 1 : 0;

	if (phydev->speed == SPEED_1000) {
		val = phy_read(phydev, 0x0a);
		local_status = val & 0x2000 ? 1 : 0;
		remote_status = val & 0x1000 ? 1 : 0;
	} else {
		phy_write(phydev, 0x1f, 0xa64);
		val = phy_read(phydev, 0x17);
		local_status = val & 0x0004 ? 1 : 0;
		remote_status = val & 0x0400 ? 1 : 0;
	}

	if (link_status && local_status && remote_status)
		phydev->link = 1;
	else
		phydev->link = 0;

	return 0;
}

static int rtl821x_probe(struct phy_device *phydev)
{
	return rtl8211f_ptp_probe(phydev);
}

static void rtl821x_remove(struct phy_device *phydev)
{
	rtl8211f_ptp_remove(phydev);
}

static struct phy_driver realtek_drvs[] = {
	{
		PHY_ID_MATCH_EXACT(0x00008201),
		.name		= "RTL8201CP Ethernet",
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
	}, {
		PHY_ID_MATCH_EXACT(0x001cc816),
		.name		= "RTL8201F Fast Ethernet",
		.ack_interrupt	= &rtl8201_ack_interrupt,
		.config_intr	= &rtl8201_config_intr,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
	}, {
		PHY_ID_MATCH_MODEL(0x001cc880),
		.name		= "RTL8208 Fast Ethernet",
		.read_mmd	= genphy_read_mmd_unsupported,
		.write_mmd	= genphy_write_mmd_unsupported,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
	}, {
		PHY_ID_MATCH_EXACT(0x001cc910),
		.name		= "RTL8211 Gigabit Ethernet",
		.config_aneg	= rtl8211_config_aneg,
		.read_mmd	= &genphy_read_mmd_unsupported,
		.write_mmd	= &genphy_write_mmd_unsupported,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
	}, {
		PHY_ID_MATCH_EXACT(0x001cc912),
		.name		= "RTL8211B Gigabit Ethernet",
		.ack_interrupt	= &rtl821x_ack_interrupt,
		.config_intr	= &rtl8211b_config_intr,
		.read_mmd	= &genphy_read_mmd_unsupported,
		.write_mmd	= &genphy_write_mmd_unsupported,
		.suspend	= rtl8211b_suspend,
		.resume		= rtl8211b_resume,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
	}, {
		PHY_ID_MATCH_EXACT(0x001cc913),
		.name		= "RTL8211C Gigabit Ethernet",
		.config_init	= rtl8211c_config_init,
		.read_mmd	= &genphy_read_mmd_unsupported,
		.write_mmd	= &genphy_write_mmd_unsupported,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
	}, {
		PHY_ID_MATCH_EXACT(0x001cc914),
		.name		= "RTL8211DN Gigabit Ethernet",
		.ack_interrupt	= rtl821x_ack_interrupt,
		.config_intr	= rtl8211e_config_intr,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
	}, {
		PHY_ID_MATCH_EXACT(0x001cc915),
		.name		= "RTL8211E Gigabit Ethernet",
		.config_init	= &rtl8211e_config_init,
		.ack_interrupt	= &rtl821x_ack_interrupt,
		.config_intr	= &rtl8211e_config_intr,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
	}, {
		/*
		 * RTL8211FSI-VS-GC 通常复用 RTL8211F 这条 PHY 驱动路径。
		 * 这里把 PTP probe/remove 只挂在 RTL8211F 这项上。
		 */
		PHY_ID_MATCH_EXACT(0x001cc916),
		.name		= "RTL8211F/FSI Gigabit Ethernet",
		.probe		= rtl821x_probe,
		.remove		= rtl821x_remove,
		.config_init	= &rtl8211f_config_init,
		.ack_interrupt	= &rtl8211f_ack_interrupt,
		.config_intr	= &rtl8211f_config_intr,
		.suspend	= genphy_suspend,
		.resume		= rtl821x_resume,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
	}, {
		.name		= "Generic FE-GE Realtek PHY",
		.match_phy_device = rtlgen_match_phy_device,
		.read_status	= rtlgen_read_status,
		.suspend	= genphy_suspend,
		.resume		= rtlgen_resume,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
		.read_mmd	= rtlgen_read_mmd,
		.write_mmd	= rtlgen_write_mmd,
	}, {
		.name		= "RTL8226 2.5Gbps PHY",
		.match_phy_device = rtl8226_match_phy_device,
		.get_features	= rtl822x_get_features,
		.config_aneg	= rtl822x_config_aneg,
		.read_status	= rtl822x_read_status,
		.suspend	= genphy_suspend,
		.resume		= rtlgen_resume,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
		.read_mmd	= rtl822x_read_mmd,
		.write_mmd	= rtl822x_write_mmd,
	}, {
		PHY_ID_MATCH_EXACT(0x001cc840),
		.name		= "RTL8226B_RTL8221B 2.5Gbps PHY",
		.get_features	= rtl822x_get_features,
		.config_aneg	= rtl822x_config_aneg,
		.read_status	= rtl822x_read_status,
		.suspend	= genphy_suspend,
		.resume		= rtlgen_resume,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
		.read_mmd	= rtl822x_read_mmd,
		.write_mmd	= rtl822x_write_mmd,
	}, {
		PHY_ID_MATCH_EXACT(0x001cc961),
		.name		= "RTL8366RB Gigabit Ethernet",
		.config_init	= &rtl8366rb_config_init,
		.ack_interrupt	= genphy_no_ack_interrupt,
		.config_intr	= genphy_no_config_intr,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}, {
		PHY_ID_MATCH_EXACT(0x001ccb30),
		.name		= "RTL9010AA_RTL9010AR_RTL9010AS Ethernet",
		.config_init	= rtl9010a_config_init,
		.config_aneg	= rtl9010a_config_aneg,
		.read_status	= rtl9010a_read_status,
		.get_features	= rtl9010a_get_features,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.read_page	= rtl821x_read_page,
		.write_page	= rtl821x_write_page,
	},
};

module_phy_driver(realtek_drvs);

static const struct mdio_device_id __maybe_unused realtek_tbl[] = {
	{ PHY_ID_MATCH_VENDOR(0x001cc800) },
	{ PHY_ID_MATCH_VENDOR(0x001ccb30) },
	{ }
};

MODULE_DEVICE_TABLE(mdio, realtek_tbl);
