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
#define RTL8211F_RESET  			BIT(15)
#define RTL8211F_BMCR 				0x00

#define RTL8211F_A43_PAGE			0xa43
#define RTL8211F_PHYCR2				0x19
#define RTL8211F_PAGSR				0x1f

#define RTL8211F_E40_PAGE			0xe40
#define RTL8211F_E41_PAGE			0xe41
#define RTL8211F_E42_PAGE			0xe42
#define RTL8211F_E43_PAGE			0xe43
#define RTL8211F_E44_PAGE			0xe44

#define RTL8211F_PTP_CTL			0x10
#define RTL8211F_PTP_INER			0x11
#define RTL8211F_PTP_INSR			0x12
#define RTL8211F_SYNCE_CTL			0x13

/*
 * 下面这组扩展 PTP 能力位和 PDELAY/TAI 寄存器定义，
 * 是后面对齐 /home/harn/linux-5.10.y/drivers/net/phy/realtek/realtek_ptp.h
 * 之后补进来的，当前主流程未必全部使用，但先与原 PTP 头文件保持一致。
 */
#define RTL8211F_UDP_CHKSUM_UPDATE		BIT(12)
#define RTL8211F_P_DRFU_2STEP_INS		BIT(11)
#define RTL8211F_P_DR_2STEP_INS			BIT(10)
#define RTL8211F_PTP_CLK_CFG			0x10
#define RTL8211F_PTP_CFG_NS_LO			0x11
#define RTL8211F_PTP_CFG_NS_HI			0x12
#define RTL8211F_PTP_CFG_S_LO			0x13
#define RTL8211F_PTP_CFG_S_MI			0x14
#define RTL8211F_PTP_CFG_S_HI			0x15

#define RTL8211F_P_DR_1STEP			BIT(7)
#define RTL8211F_PTP_TRX_TS_STA			0x10
#define RTL8211F_AVB_802_1AS			BIT(5)
#define RTL8211F_PTPV2_UDPIPV6			BIT(2)
#define RTL8211F_PTPV1				BIT(1)

#define RTL8211F_TRIGGER_GEN			BIT(1)
#define RTL8211F_EVENT_CAPTURE			BIT(0)

#define RTL8211F_PTP_TAI_CFG			0x10
#define RTL8211F_PTP_TRIG_CFG			0x11
#define RTL8211F_PTP_TAI_STA			0x12
#define RTL8211F_PTP_TAI_TS_NS_LO		0x13
#define RTL8211F_PTP_TAI_TS_NS_HI		0x14
#define RTL8211F_PTP_TAI_TS_S_LO		0x15
#define RTL8211F_PTP_TAI_TS_S_HI		0x16

#define RTL8211F_TXTS_PDELAY_REQ_RDY		BIT(13)
#define RTL8211F_TXTS_PDELAY_RSP_RDY		BIT(12)
#define RTL8211F_RXTS_PDELAY_REQ_RDY		BIT(9)
#define RTL8211F_RXTS_PDELAY_RSP_RDY		BIT(8)
#define RTL8211F_TRXTS_MSGTYPE_SEL		(BIT(3) | BIT(2))
#define RTL8211F_TRXTS_SYNC			0
#define RTL8211F_TRXTS_DELAY_REQ		1
#define RTL8211F_TRXTS_PDELAY_REQ		2
#define RTL8211F_TRXTS_PDELAY_RSP		3
#define RTL8211F_TRXTS_SEL			BIT(1)

#define RTL8211F_PTP_TRX_TS_INFO		0x10
#define RTL8211F_PTP_TRX_TS_SH			0x11
#define RTL8211F_PTP_TRX_TS_SID			0x12
#define RTL8211F_PTP_TRX_TS_NS_LO		0x13
#define RTL8211F_PTP_TRX_TS_NS_HI		0x14
#define RTL8211F_PTP_TRX_TS_S_LO		0x15
#define RTL8211F_PTP_TRX_TS_S_MI		0x16
#define RTL8211F_PTP_TRX_TS_S_HI		0x17

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
/**
 * @brief 
 * SourcePortIdentity 字段
 * @clk_identity: 8 bytes 设备指示
 * @src_port_id: 2 bytes 源端口ID 即clk_identity指示的设备的网络端口。
 * 
 */
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
/**
 * @brief 
 * @info 这个结构体是为了存储从 RTL8211F 读出的时间戳信息,
 * @sh source port hash value
 * @sid sequence ID of the PTP message
 * @domain Transport specific message |  mesage type  all from info register 
 * @tsmt PTP version filed from info register
 */
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
/**
 * @brief 获取 skb 中的 私有信息结构体指针
 * 
 * @param skb 
 * @return struct rtl8211f_skb_info* 类型的指针
 */
static struct rtl8211f_skb_info *rtl8211f_skb_info(struct sk_buff *skb)
{
	return (struct rtl8211f_skb_info *)skb->cb;
}

static int rtl8211f_write_page(struct phy_device *phydev, int page, int regnum,
			       u16 val)
{
	int ret;

	ret = phy_write_paged(phydev, page, regnum, val);
	if (ret < 0)
		dev_err_ratelimited(&phydev->mdio.dev,
				    "ptp write failed: page=0x%x reg=0x%x ret=%d\n",
				    page, regnum, ret);

	return ret;
}

static int rtl8211f_read_page(struct phy_device *phydev, int page, int regnum)
{
	int ret;

	ret = phy_read_paged(phydev, page, regnum);
	if (ret < 0)
		dev_err_ratelimited(&phydev->mdio.dev,
				    "ptp read failed: page=0x%x reg=0x%x ret=%d\n",
				    page, regnum, ret);

	return ret;
}
/**
 * @brief 根据消息类型从寄存器之中读最新的 stamp的元数据。
 * 
 * @param info 用于获取上游 rtl8211f_ptp 结构体指针
 * @param msg_type 根据消息类型 
 * @param rx 所需要获取的元数据是 最近的tx 或者 rx 类型的消息
 * @param meta 元数据
 * @param ts 最新的时间戳值
 * @return int 
 */

static int rtl8211f_soft_reset(struct phy_device *phydev)
{
	int ret, val;
	int timeout = 100;

	val = phy_read(phydev, MII_BMCR);
	if (val < 0)
		return val;

	ret = phy_write(phydev, MII_BMCR, val | BMCR_RESET);
	if (ret < 0)
		return ret;

	do {
		usleep_range(1000, 2000);
		val = phy_read(phydev, MII_BMCR);
		if (val < 0)
			return val;
		if (!(val & BMCR_RESET))
			return 0;
	} while (--timeout);

	return -ETIMEDOUT;
}
void rtl8211f_ptp_phy_reset(struct phy_device *phydev)
{
    phy_write_paged(phydev, 0x0, 0x0, 0x9040);
}
/**
 * Enable sync ethernet function
 *
 * After phy link-up, the value of (page 0xa43, 0x1a) should be 0x302e,
 * and the value of (page 0xe40, 0x13) should be 0x1.
 */
void rtl8211f_sync_ethernet(struct phy_device *phydev)
{
    int val = 0;

    /* 
     Harn note:
     0x09 GBCR 寄存器 
     bit 12: MASTER/SLAVE Manual Configuration Enable: Manual MASTER/SLAVE configuration
     bit 11: default MASTER/SLAVE Mode: value 0 Slave mode 
     Bit 9: 1000Base-T Full Duplex Capable: 
                                value 1 Advertise 
                                value 0 Do not advertise 
    */

    phy_write(phydev, 0x09, 0x1200);
    /* 使能Sync-E 功能，此功能应用需要重启 */
    
    val = phy_read_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_SYNCE_CTL);
    val |= RTL8211F_SYNC_E_ENABLE;
    phy_write_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_SYNCE_CTL, val);
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
	val = rtl8211f_write_page(ptp->phydev, RTL8211F_E43_PAGE,
				  RTL8211F_PTP_TRX_TS_STA, val);
	if (val < 0)
		return val;

	if (meta) {
		val = rtl8211f_read_page(ptp->phydev, RTL8211F_E44_PAGE,
					 RTL8211F_PTP_TRX_TS_INFO);
		if (val < 0)
			return val;
		meta->info = val;
		val = rtl8211f_read_page(ptp->phydev, RTL8211F_E44_PAGE,
					 RTL8211F_PTP_TRX_TS_SH);
		if (val < 0)
			return val;
		meta->sh = cpu_to_be16(val);
		val = rtl8211f_read_page(ptp->phydev, RTL8211F_E44_PAGE,
					 RTL8211F_PTP_TRX_TS_SID);
		if (val < 0)
			return val;
		meta->sid = cpu_to_be16(val);
		meta->domain = meta->info >> 8;
		meta->tsmt = meta->info & GENMASK(3, 0);
	}

	val = rtl8211f_read_page(ptp->phydev, RTL8211F_E44_PAGE,
				 RTL8211F_PTP_TRX_TS_S_HI);
	if (val < 0)
		return val;
	ts->tv_sec = val;
	val = rtl8211f_read_page(ptp->phydev, RTL8211F_E44_PAGE,
				 RTL8211F_PTP_TRX_TS_S_MI);
	if (val < 0)
		return val;
	ts->tv_sec = (ts->tv_sec << 16) | val;
	val = rtl8211f_read_page(ptp->phydev, RTL8211F_E44_PAGE,
				 RTL8211F_PTP_TRX_TS_S_LO);
	if (val < 0)
		return val;
	ts->tv_sec = (ts->tv_sec << 16) | val;

	val = rtl8211f_read_page(ptp->phydev, RTL8211F_E44_PAGE,
				 RTL8211F_PTP_TRX_TS_NS_HI);
	if (val < 0)
		return val;
	ts->tv_nsec = val & 0x3fff;
	val = rtl8211f_read_page(ptp->phydev, RTL8211F_E44_PAGE,
				 RTL8211F_PTP_TRX_TS_NS_LO);
	if (val < 0)
		return val;
	ts->tv_nsec = (ts->tv_nsec << 16) | val;

	return 0;
}

/*
 * 按 Realtek 应用说明中的规则计算 SourcePortIdentity hash：
 * 1. 将 8-byte clockIdentity 扩展为 10-byte SourcePortIdentity
 *    对 IEEE 1588 v1：在第 4/5 字节插入 0x00 0x00
 *    对 IEEE 1588 v2/802.1AS：直接拼接 8-byte clockIdentity + 2-byte sourcePortNumber
 * 2. 每 2 字节求和
 * 3. 将高 16-bit 与低 16-bit 相加，得到 16-bit hash
 */
static u16 rtl8211f_source_port_hash(struct rtl8211f_ptphdr *ptphdr)
{
	u8 spi[10];
	u32 sum;
	int i;

	if ((ptphdr->ver & 0xf) == 1) {
		memcpy(&spi[0], &ptphdr->clk_identity, 3);
		spi[3] = 0x00;
		spi[4] = 0x00;
		memcpy(&spi[5], ((u8 *)&ptphdr->clk_identity) + 3, 5);
	} else {//这里做的是还原成大端法然后想加
		memcpy(&spi[0], &ptphdr->clk_identity, 8);
		//硬件是按照大端的顺序直接截断相加的，并没有转换位实际值。所以只需要按照大端法的排列组合成数字，加起来就可以
		spi[8] = ((__force u16)ptphdr->src_port_id >> 8) & 0xff;
		spi[9] = (__force u16)ptphdr->src_port_id & 0xff;
	}

	sum = 0;
	// 因为123456 变成小端是654321 ，但是只需要保证每次操作的两个数的顺序正确，相加顺序无所谓
	for (i = 0; i < ARRAY_SIZE(spi); i += 2)
		sum += ((u16)spi[i] << 8) | spi[i + 1];

	return (sum & 0xffff) + (sum >> 16);
}

static bool rtl8211f_match_hwstamp(struct rtl8211f_ptphdr *ptphdr,
				   const struct rtl8211f_trxstamp_meta *meta,
				   u8 msg_type)
{
	u16 src_port_hash;

	if (!ptphdr)
		return false;

	if ((ptphdr->tsmt & 0xf) != msg_type)
		return false;

	if (meta->tsmt != msg_type)
		return false;

	if (ptphdr->seq_id != meta->sid)
		return false;

	src_port_hash = rtl8211f_source_port_hash(ptphdr);
	if (src_port_hash != be16_to_cpu(meta->sh))
		return false;

	if (ptphdr->domain != meta->domain)
		return false;

	return true;
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

	return rtl8211f_match_hwstamp(ptphdr, &rxts->meta, rxts->msg_type);
}

static void rtl8211f_prune_tx_queue(struct rtl8211f_ptp *ptp);

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
	bool resched = false;

	while ((skb = skb_dequeue(&ptp->rx_queue))) {
		struct rtl8211f_skb_info *skb_info = rtl8211f_skb_info(skb);

		if (!time_after(jiffies, skb_info->tmo)) {
			skb_queue_head(&ptp->rx_queue, skb);
			resched = true;
			break;
		}

		netif_rx_ni(skb);
	}

	rtl8211f_prune_tx_queue(ptp);

	if (resched || !skb_queue_empty(&ptp->rx_queue) ||
	    !skb_queue_empty(&ptp->tx_queue))
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

	rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
			    RTL8211F_PTP_CLK_CFG, (mode << 1) | 0x1);
}

static int rtl8211f_gettime(struct ptp_clock_info *info, struct timespec64 *ts)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	int val;

	mutex_lock(&ptp->ts_mutex);
	rtl8211f_adjust_mode_set(info, RTL8211F_DIRECT_READ);

	val = rtl8211f_read_page(ptp->phydev, RTL8211F_E41_PAGE,
				 RTL8211F_PTP_CFG_S_HI);
	if (val < 0)
		goto out_unlock_gettime;
	ts->tv_sec = val;
	val = rtl8211f_read_page(ptp->phydev, RTL8211F_E41_PAGE,
				 RTL8211F_PTP_CFG_S_MI);
	if (val < 0)
		goto out_unlock_gettime;
	ts->tv_sec = (ts->tv_sec << 16) | val;
	val = rtl8211f_read_page(ptp->phydev, RTL8211F_E41_PAGE,
				 RTL8211F_PTP_CFG_S_LO);
	if (val < 0)
		goto out_unlock_gettime;
	ts->tv_sec = (ts->tv_sec << 16) | val;

	val = rtl8211f_read_page(ptp->phydev, RTL8211F_E41_PAGE,
				 RTL8211F_PTP_CFG_NS_HI);
	if (val < 0)
		goto out_unlock_gettime;
	ts->tv_nsec = val;
	val = rtl8211f_read_page(ptp->phydev, RTL8211F_E41_PAGE,
				 RTL8211F_PTP_CFG_NS_LO);
	if (val < 0)
		goto out_unlock_gettime;
	ts->tv_nsec = (ts->tv_nsec << 16) | val;
	val = 0;
out_unlock_gettime:
	mutex_unlock(&ptp->ts_mutex);

	return val;
}

static int rtl8211f_settime(struct ptp_clock_info *info,
			    const struct timespec64 *ts)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	int ret;

	mutex_lock(&ptp->ts_mutex);
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_S_HI,
				  (ts->tv_sec >> 32) & 0xffff);
	if (ret < 0)
		goto out_unlock_settime;
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_S_MI,
				  (ts->tv_sec >> 16) & 0xffff);
	if (ret < 0)
		goto out_unlock_settime;
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_S_LO, ts->tv_sec & 0xffff);
	if (ret < 0)
		goto out_unlock_settime;
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_NS_HI,
				  (ts->tv_nsec >> 16) & 0xffff);
	if (ret < 0)
		goto out_unlock_settime;
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_NS_LO, ts->tv_nsec & 0xffff);
	if (ret < 0)
		goto out_unlock_settime;
	rtl8211f_adjust_mode_set(info, RTL8211F_DIRECT_WRITE);
	ret = 0;
out_unlock_settime:
	mutex_unlock(&ptp->ts_mutex);

	return ret;
}

static int rtl8211f_adjtime(struct ptp_clock_info *info, s64 delta)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	u64 sec_diff, nsec_diff;
	bool negative = false;
	int ret;

	if (delta < 0) {
		delta = -delta;
		negative = true;
	}

	sec_diff = div_u64(delta, NSEC_PER_SEC);
	nsec_diff = delta - sec_diff * NSEC_PER_SEC;

	mutex_lock(&ptp->ts_mutex);
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_NS_LO, nsec_diff & 0xffff);
	if (ret < 0)
		goto out_unlock_adjtime;
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_NS_HI,
				  (nsec_diff >> 16) & 0x3fff);
	if (ret < 0)
		goto out_unlock_adjtime;
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_S_LO, sec_diff & 0xffff);
	if (ret < 0)
		goto out_unlock_adjtime;
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_S_MI,
				  (sec_diff >> 16) & 0xffff);
	if (ret < 0)
		goto out_unlock_adjtime;
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_S_HI,
				  (sec_diff >> 32) & 0xffff);
	if (ret < 0)
		goto out_unlock_adjtime;
	rtl8211f_adjust_mode_set(info, negative ? RTL8211F_DECREMENT_STEP :
					 RTL8211F_INCREMENT_STEP);
	ret = 0;
out_unlock_adjtime:
	mutex_unlock(&ptp->ts_mutex);

	return ret;
}

static int rtl8211f_adjfreq(struct ptp_clock_info *info, s32 delta)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	int ret;

	mutex_lock(&ptp->ts_mutex);
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_NS_HI,
				  (delta >> 16) & 0xffff);
	if (ret < 0)
		goto out_unlock_adjfreq;
	ret = rtl8211f_write_page(ptp->phydev, RTL8211F_E41_PAGE,
				  RTL8211F_PTP_CFG_NS_LO, delta & 0xffff);
	if (ret < 0)
		goto out_unlock_adjfreq;
	rtl8211f_adjust_mode_set(info, RTL8211F_RATE_WRITE);
	ret = 0;
out_unlock_adjfreq:
	mutex_unlock(&ptp->ts_mutex);

	return ret;
}

static void rtl8211f_prune_tx_queue(struct rtl8211f_ptp *ptp)
{
	struct sk_buff *skb, *tmp;

	spin_lock_irq(&ptp->tx_queue_lock);
	skb_queue_walk_safe(&ptp->tx_queue, skb, tmp) {
		struct rtl8211f_skb_info *skb_info = rtl8211f_skb_info(skb);

		if (!time_after(jiffies, skb_info->tmo))
			continue;

		__skb_unlink(skb, &ptp->tx_queue);
		kfree_skb(skb);
	}
	spin_unlock_irq(&ptp->tx_queue_lock);
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

	if (rtl8211f_write_page(ptp->phydev, RTL8211F_E40_PAGE,
				RTL8211F_PTP_CTL, mode) < 0)
		return -EIO;
	
	// rtl8211f_soft_reset(ptp->phydev);//重启

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
	schedule_delayed_work(&priv->ptp->ts_work, RTL8211F_SKB_TIMESTAMP_TIMEOUT);
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
	struct skb_shared_hwtstamps shhwtstamps;
	struct timespec64 ts;
	int ret;

	ret = rtl8211f_get_trxstamp(&ptp->caps, message_type, false, &meta, &ts);
	if (ret < 0)
		return ret;

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

		if (rtl8211f_match_hwstamp(rtl8211f_get_ptp_header_tx(skb), &meta,
					   message_type)) {
			best = skb;
			break;
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
	int ret;

	ret = rtl8211f_get_trxstamp(&ptp->caps, message_type, true, &meta, &ts);
	if (ret < 0)
		return ret;

	spin_lock_irq(&ptp->rx_queue_lock);
	skb_queue_walk_safe(&ptp->rx_queue, skb, tmp) {
		if (rtl8211f_match_hwstamp(rtl8211f_get_ptp_header_rx(skb,
					      ptp->rx_filter),
					  &meta, message_type)) {
			best = skb;
			break;
		}
	}
	if (best)
		__skb_unlink(best, &ptp->rx_queue);
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
	int val;

	while (!kthread_should_stop()) {
		if (!wait_for_completion_interruptible_timeout(&ptp->complete,
				msecs_to_jiffies(5000)))
			continue;

		val = rtl8211f_read_page(ptp->phydev, RTL8211F_E40_PAGE,
					 RTL8211F_PTP_INSR);
		if (val < 0)
			continue;
		val = rtl8211f_read_page(ptp->phydev, RTL8211F_E43_PAGE,
					 RTL8211F_PTP_TRX_TS_STA);
		if (val < 0)
			continue;

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
	int ret;

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

	rtl8211f_sync_ethernet(phydev);
	
	//清除ptp功能，只保留使能后续进行重启
	ret = rtl8211f_write_page(phydev, RTL8211F_E40_PAGE, RTL8211F_PTP_CTL, RTL8211F_PTP_ENABLE);
	if (ret < 0)
		return ret;
	//重启
	rtl8211f_ptp_phy_reset(phydev);
	//重启后再配置寄存器
	ret = rtl8211f_write_page(phydev, RTL8211F_E40_PAGE, RTL8211F_PTP_INER,
				  RTL8211F_TX_TIMESTAMP | RTL8211F_RX_TIMESTAMP);
	if (ret < 0)
		return ret;
	

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
