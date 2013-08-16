#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>

#include <asm/delay.h>
#include <asm/irq.h>
#include <asm/io.h>

#undef LYDM9000_DEBUG
// #define LYDM9000_DEBUG
#ifdef LYDM9000_DEBUG
#define printdbg(fmt, args...) \
    printk("lydm9k[%s]:\t" fmt, __func__, ##args)
#else
#define printdbg(fmt, args...) \
    do {} while(0)
#endif /* LYDM9000_DEBUG */

#define dm9kmsg(fmt, args...) \
    printk("lydm9k[%s]:\t" fmt, __func__, ##args)

// **************** register ********************
#define DM9000_NCR             0x00
#define DM9000_NSR             0x01
#define DM9000_TCR             0x02
#define DM9000_TSR1            0x03
#define DM9000_TSR2            0x04
#define DM9000_RCR             0x05
#define DM9000_RSR             0x06
#define DM9000_ROCR            0x07
#define DM9000_BPTR            0x08
#define DM9000_FCTR            0x09
#define DM9000_FCR             0x0A
#define DM9000_EPCR            0x0B
#define DM9000_EPAR            0x0C
#define DM9000_EPDRL           0x0D
#define DM9000_EPDRH           0x0E
#define DM9000_WCR             0x0F

#define DM9000_PAR             0x10
#define DM9000_PAB0            0x10
#define DM9000_PAB1            0x11
#define DM9000_PAB2            0x12
#define DM9000_PAB3            0x13
#define DM9000_PAB4            0x14
#define DM9000_PAB5            0x15
#define DM9000_MAR             0x16

#define DM9000_GPCR	       0x1e
#define DM9000_GPR             0x1f
#define DM9000_TRPAL           0x22
#define DM9000_TRPAH           0x23
#define DM9000_RWPAL           0x24
#define DM9000_RWPAH           0x25

#define DM9000_VIDL            0x28
#define DM9000_VIDH            0x29
#define DM9000_PIDL            0x2A
#define DM9000_PIDH            0x2B

#define DM9000_CHIPR           0x2C
#define DM9000_SMCR            0x2F

#define DM9000_ETXCSR          0x30
#define DM9000_TCCR	       0x31
#define DM9000_RCSR	       0x32

#define CHIPR_DM9000A	       0x19
#define CHIPR_DM9000B	       0x1A

#define DM9000_MRCMDX          0xF0
#define DM9000_MRCMD           0xF2
#define DM9000_MRRL            0xF4
#define DM9000_MRRH            0xF5
#define DM9000_MWCMDX          0xF6
#define DM9000_MWCMD           0xF8
#define DM9000_MWRL            0xFA
#define DM9000_MWRH            0xFB
#define DM9000_TXPLL           0xFC
#define DM9000_TXPLH           0xFD
#define DM9000_ISR             0xFE
#define DM9000_IMR             0xFF

#define NCR_EXT_PHY         (1<<7)
#define NCR_WAKEEN          (1<<6)
#define NCR_FCOL            (1<<4)
#define NCR_FDX             (1<<3)
#define NCR_LBK             (3<<1)
#define NCR_RST	            (1<<0)

#define NSR_SPEED           (1<<7)
#define NSR_LINKST          (1<<6)
#define NSR_WAKEST          (1<<5)
#define NSR_TX2END          (1<<3)
#define NSR_TX1END          (1<<2)
#define NSR_RXOV            (1<<1)

#define TCR_TJDIS           (1<<6)
#define TCR_EXCECM          (1<<5)
#define TCR_PAD_DIS2        (1<<4)
#define TCR_CRC_DIS2        (1<<3)
#define TCR_PAD_DIS1        (1<<2)
#define TCR_CRC_DIS1        (1<<1)
#define TCR_TXREQ           (1<<0)

#define TSR_TJTO            (1<<7)
#define TSR_LC              (1<<6)
#define TSR_NC              (1<<5)
#define TSR_LCOL            (1<<4)
#define TSR_COL             (1<<3)
#define TSR_EC              (1<<2)

#define RCR_WTDIS           (1<<6)
#define RCR_DIS_LONG        (1<<5)
#define RCR_DIS_CRC         (1<<4)
#define RCR_ALL	            (1<<3)
#define RCR_RUNT            (1<<2)
#define RCR_PRMSC           (1<<1)
#define RCR_RXEN            (1<<0)

#define RSR_RF              (1<<7)
#define RSR_MF              (1<<6)
#define RSR_LCS             (1<<5)
#define RSR_RWTO            (1<<4)
#define RSR_PLE             (1<<3)
#define RSR_AE              (1<<2)
#define RSR_CE              (1<<1)
#define RSR_FOE             (1<<0)

#define WCR_LINKEN		(1 << 5)
#define WCR_SAMPLEEN		(1 << 4)
#define WCR_MAGICEN		(1 << 3)
#define WCR_LINKST		(1 << 2)
#define WCR_SAMPLEST		(1 << 1)
#define WCR_MAGICST		(1 << 0)

#define FCTR_HWOT(ot)	(( ot & 0xf ) << 4 )
#define FCTR_LWOT(ot)	( ot & 0xf )

#define IMR_PAR             (1<<7)
#define IMR_ROOM            (1<<3)
#define IMR_ROM             (1<<2)
#define IMR_PTM             (1<<1)
#define IMR_PRM             (1<<0)

#define ISR_ROOS            (1<<3)
#define ISR_ROS             (1<<2)
#define ISR_PTS             (1<<1)
#define ISR_PRS             (1<<0)
#define ISR_CLR_STATUS      (ISR_ROOS | ISR_ROS | ISR_PTS | ISR_PRS)

#define EPCR_REEP           (1<<5)
#define EPCR_WEP            (1<<4)
#define EPCR_EPOS           (1<<3)
#define EPCR_ERPRR          (1<<2)
#define EPCR_ERPRW          (1<<1)
#define EPCR_ERRE           (1<<0)

#define GPCR_GEP_CNTL       (1<<0)

#define TCCR_IP		    (1<<0)
#define TCCR_TCP	    (1<<1)
#define TCCR_UDP	    (1<<2)

#define RCSR_UDP_BAD	    (1<<7)
#define RCSR_TCP_BAD	    (1<<6)
#define RCSR_IP_BAD	    (1<<5)
#define RCSR_UDP	    (1<<4)
#define RCSR_TCP	    (1<<3)
#define RCSR_IP		    (1<<2)
#define RCSR_CSUM	    (1<<1)
#define RCSR_DISCARD	    (1<<0)

#define DM9000_PKT_RDY		0x01	/* Packet ready to receive */
#define DM9000_PKT_ERR		0x02
#define DM9000_PKT_MAX		1536	/* Received packet max size */

/* DM9000A / DM9000B definitions */

#define IMR_LNKCHNG		(1<<5)
#define IMR_UNDERRUN		(1<<4)

#define ISR_LNKCHNG		(1<<5)
#define ISR_UNDERRUN		(1<<4)
// ************** end register ******************

struct lydm9k_plat_data {
    unsigned char mac[6];
    unsigned int watchdog_timeo_msecs;
};

struct lydm9k_priv {
    struct platform_device *pdev;

    struct resource *addr_res;
    struct resource *data_res;
    struct resource *irq_res;
    void __iomem *io_reg;
    void __iomem *io_data;

    unsigned char	mac[6];

    spinlock_t spin_reg;
};

// 写register
static void inline regw(struct lydm9k_priv *priv,
        unsigned char reg,
        unsigned char data)
{
    unsigned long irq_tmp;

    spin_lock_irqsave(&priv->spin_reg, irq_tmp);

    writeb(reg, priv->io_reg);
    writeb(data, priv->io_data);

    spin_unlock_irqrestore(&priv->spin_reg, irq_tmp);
}

// 读register
static unsigned char inline regr(struct lydm9k_priv *priv,
        unsigned char reg)
{
    unsigned char val;
    unsigned long irq_tmp;

    spin_lock_irqsave(&priv->spin_reg, irq_tmp);

    writeb(reg, priv->io_reg);
    val = readb(priv->io_data);

    spin_unlock_irqrestore(&priv->spin_reg, irq_tmp);

    return val;
}

static void outblk(struct lydm9k_priv *priv, 
        unsigned char reg, void *data, int count)
{
    unsigned long irq_tmp;

    spin_lock_irqsave(&priv->spin_reg, irq_tmp);
    writeb(reg, priv->io_reg);
    writesw(priv->io_data, data, (count+1) >> 1);
    spin_unlock_irqrestore(&priv->spin_reg, irq_tmp);
}

static void inblk(struct lydm9k_priv *priv, 
        unsigned char reg, void *data, int count)
{
    unsigned long irq_tmp;

    spin_lock_irqsave(&priv->spin_reg, irq_tmp);
    writeb(reg, priv->io_reg);
    readsw(priv->io_data, data, (count+1) >> 1);
    spin_unlock_irqrestore(&priv->spin_reg, irq_tmp);
}

static void dumpblk(struct lydm9k_priv *priv, 
        unsigned char reg, int count)
{
    unsigned long irq_tmp;

    spin_lock_irqsave(&priv->spin_reg, irq_tmp);
    writeb(reg, priv->io_reg);
    while(count > 0)
    {
        readw(priv->io_data);
        count -= 2;
    }
    spin_unlock_irqrestore(&priv->spin_reg, irq_tmp);
}

static void inline dm9k_reset(struct lydm9k_priv *priv)
{
    regw(priv, DM9000_NCR, NCR_RST | (1<<1));
    udelay(200); // at least 20 usecs
}

static void inline dm9k_init(struct lydm9k_priv *priv)
{
    int i;
    int oft;

    regw(priv, DM9000_GPCR, GPCR_GEP_CNTL);
    regw(priv, DM9000_GPR, 0);

    regw(priv, DM9000_RCR, RCR_RXEN |  RCR_DIS_CRC | RCR_DIS_LONG);
    regw(priv, DM9000_TCR, 0x00);
    regw(priv, DM9000_BPTR, 0x3f);
    regw(priv, DM9000_FCTR, 0x38);
    regw(priv, DM9000_FCR, 0xff);
    regw(priv, DM9000_SMCR, 0x00);
    regw(priv, DM9000_NSR, NSR_TX1END | NSR_TX2END | NSR_WAKEST);
    regw(priv, DM9000_ISR, 0x0f); // clear interrupt status

    regw(priv, DM9000_PAB0, priv->mac[0]);
    regw(priv, DM9000_PAB1, priv->mac[1]);
    regw(priv, DM9000_PAB2, priv->mac[2]);
    regw(priv, DM9000_PAB3, priv->mac[3]);
    regw(priv, DM9000_PAB4, priv->mac[4]);
    regw(priv, DM9000_PAB5, priv->mac[5]);

    for(i = 0, oft = DM9000_MAR; i<8; i++, oft++)
    {
        regw(priv, oft, 0xff);
    }

    regw(priv, DM9000_IMR, IMR_PRM | IMR_PTM | IMR_PAR);
}

static void dm9k_send_pkt(struct lydm9k_priv *priv,
        unsigned char *data,
        size_t len)
{
    regw(priv, DM9000_TXPLH, (len >> 8) & 0xff);
    regw(priv, DM9000_TXPLL, len & 0xff);

    outblk(priv, DM9000_MWCMD, data, len);

    regw(priv, DM9000_TCR, TCR_TXREQ);
}

struct dm9000_rxhdr {
    u8	RxPktReady;
    u8	RxStatus;
    __le16	RxLen;
} __attribute__((__packed__));

static int dm9k_recv_pkt(struct net_device *ndev)
{
    unsigned char rx_status;
    unsigned short rx_len;
    struct dm9000_rxhdr rxhdr;
    unsigned char *data;
    struct sk_buff *skb;
    struct lydm9k_priv *priv = netdev_priv(ndev);

    printdbg("invoked\n");

    /* Get most updated data */
    rx_status = regr(priv, DM9000_MRCMDX);
    printdbg("rx_status = 0x%x\n", rx_status);

    /* Status check: this byte must be 0 or 1 */
    if (rx_status & DM9000_PKT_ERR) {
        printdbg("recv pkt err\n");
        regw(priv, DM9000_RCR, 0x00);	/* Stop Device */
        regw(priv, DM9000_ISR, IMR_PAR);	/* Stop INT request */
        return -1;
    }

    if (!(rx_status & DM9000_PKT_RDY))
    {
        printdbg("there is no pkt\n");
        return -1;
    }

    inblk(priv, DM9000_MRCMD, &rxhdr, sizeof(rxhdr));
    rx_len = le16_to_cpu(rxhdr.RxLen);
    printdbg("recv 0x%x bytes\n", rx_len);

    skb = dev_alloc_skb(rx_len + 4);
    if(NULL == skb)
    {
        dumpblk(priv, DM9000_MRCMD, rx_len);
        return -ENOMEM;
    }

    skb_reserve(skb, 2);
    data = skb_put(skb, rx_len - 4);
    inblk(priv, DM9000_MRCMD, data, rx_len);

    skb->protocol = eth_type_trans(skb, ndev);
    skb->ip_summed = CHECKSUM_NONE;

#ifdef LYDM9000_DEBUG
    for(i = 0; i<skb->len; i++)
    {
        printk("0x%02x\t", *(skb->data + i));
    }
    printk("\n");
#endif

    netif_rx(skb);

    return 0;

    return -1;
}

static void dm9k_tx_done(struct net_device *ndev)
{
    struct lydm9k_priv *priv = netdev_priv(ndev);
    unsigned char tx_status = regr(priv, DM9000_NSR);

    if(tx_status & (NSR_TX1END | NSR_TX2END))
    {
        netif_wake_queue(ndev);
    }
}

static irqreturn_t dm9k_interrupt(int irq, void *dev_id)
{
    struct net_device *ndev = dev_id;
    struct lydm9k_priv *priv = netdev_priv(ndev);
    unsigned char int_status;
    unsigned char reg_save;

    /* Save previous register address */
    reg_save = readb(priv->io_reg);

    /* Disable all interrupts */
    regw(priv, DM9000_IMR, IMR_PAR);

    int_status = regr(priv, DM9000_ISR);
    regw(priv, DM9000_ISR, int_status);
    printdbg("int_status = %02x\n", int_status);

    // recv irq
    if(int_status & ISR_PRS)
    {
        while(0 == dm9k_recv_pkt(ndev));
    }

    // trans irq
    if(int_status & ISR_PTS)
    {
        dm9k_tx_done(ndev);
    }

    /* Re-enable interrupt mask */
    regw(priv, DM9000_IMR, IMR_PRM | IMR_PTM | IMR_PAR);

    /* Restore previous register address */
    writeb(reg_save, priv->io_reg);

    return IRQ_HANDLED;
}

static int lydm9k_open(struct net_device *ndev)
{
    struct lydm9k_priv *priv = netdev_priv(ndev);
    unsigned long irqflags = priv->irq_res->flags & IRQF_TRIGGER_MASK;

    printdbg("invoked\n");

    irqflags |= IRQF_SHARED;
    if(request_irq(ndev->irq, dm9k_interrupt, irqflags, ndev->name, ndev))
    {
        dm9kmsg("request irq fail\n");
        return -EAGAIN;
    }

    dm9k_reset(priv);
    dm9k_init(priv);

    ndev->trans_start = 0;

    netif_start_queue(ndev);

    return 0;
}

static int lydm9k_stop(struct net_device *ndev)
{
    // struct lydm9k_priv *priv = netdev_priv(ndev);

    printdbg("invoked\n");

    netif_stop_queue(ndev);

    free_irq(ndev->irq, ndev);

    return 0;
}

static int lydm9k_start_xmit(struct sk_buff *skb, 
        struct net_device *ndev)
{
    struct lydm9k_priv *priv = netdev_priv(ndev);

    printdbg("invoked\n");

    dm9k_send_pkt(priv, skb->data, skb->len);

    netif_stop_queue(ndev);

    dev_kfree_skb(skb);

    return NETDEV_TX_OK;
}

static void lydm9k_timeout(struct net_device *ndev)
{
    struct lydm9k_priv *priv = netdev_priv(ndev);

    printdbg("invoked\n");

    netif_stop_queue(ndev);

    dm9k_reset(priv);
    dm9k_init(priv);

    ndev->trans_start = jiffies;

    netif_start_queue(ndev);
}

static struct net_device_ops lydm9k_ops = {
    .ndo_open			= lydm9k_open,
    .ndo_stop			= lydm9k_stop,
    .ndo_start_xmit		= lydm9k_start_xmit,
    .ndo_tx_timeout		= lydm9k_timeout,
};

static int __devinit lydm9k_probe(struct platform_device *pdev)
{
    int ret;
    struct net_device *ndev;
    struct lydm9k_priv *priv;
    struct lydm9k_plat_data *pdata = pdev->dev.platform_data;
    size_t io_size;
    struct resource *addr_res;
    struct resource *data_res;

    printdbg("detect a dm9k device\n");

    // 申请ndev空间
    ndev = alloc_etherdev(sizeof(struct lydm9k_priv));
    if (NULL == ndev)
    {
        dm9kmsg("fail to alloc_etherdev\n");
        return -ENOMEM;
    }

    // 设置ndev的父设备为pdev
    SET_NETDEV_DEV(ndev, &pdev->dev);

    // 设置ndev和pdev直接的关联
    platform_set_drvdata(pdev, ndev);
    priv = netdev_priv(ndev);
    priv->pdev = pdev;

    // remap
    addr_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    data_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    priv->irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if(	(NULL == addr_res) 
            || (NULL == data_res)
            || (NULL == priv->irq_res))
    {
        dm9kmsg("cannot get platform resource\n");
        ret = -ENOENT;
        goto failure_platform_get_resource;
    }

    io_size = resource_size(addr_res);
    if(NULL == request_mem_region(	addr_res->start, 
                io_size, 
                pdev->name))
    {
        dm9kmsg("cannot claim addr reg area");
        ret = -EIO;
        goto failure_request_mem_region_addr;
    }
    priv->io_reg = ioremap(addr_res->start, io_size);
    if(NULL == priv->io_reg)
    {
        dm9kmsg("fail to remap io addr\n");
        ret = -EINVAL;
        goto failure_ioremap_addr;
    }

    io_size = resource_size(data_res);
    if(NULL == request_mem_region(	data_res->start, 
                io_size, 
                pdev->name))
    {
        dm9kmsg("cannot claim data reg area");
        ret = -EIO;
        goto failure_request_mem_region_data;
    }
    priv->io_data = ioremap(data_res->start, io_size);
    if(NULL == priv->io_data)
    {
        dm9kmsg("fail to remap io data\n");
        ret = -EINVAL;
        goto failure_ioremap_data;
    }

    ndev->irq = priv->irq_res->start;
    priv->addr_res = addr_res;
    priv->data_res = data_res;

    // 初始化自旋锁
    spin_lock_init(&priv->spin_reg);

    // 设置操作函数集/mac/watchdog_timeo
    ndev->netdev_ops = &lydm9k_ops;
    ndev->watchdog_timeo = 
        msecs_to_jiffies(pdata->watchdog_timeo_msecs);
    memcpy(ndev->dev_addr, pdata->mac, 6);
    memcpy(priv->mac, pdata->mac, 6);

    // 注册网卡
    ret = register_netdev(ndev);
    if(0 > ret)
    {
        dm9kmsg("fail to register netdev\n");
        goto failure_register_netdev;
    }

    return 0;

failure_register_netdev:
    iounmap(priv->io_data);
failure_ioremap_data:
    io_size = resource_size(data_res);
    release_mem_region(data_res->start, io_size);
failure_request_mem_region_data:
    iounmap(priv->io_reg);
failure_ioremap_addr:
    io_size = resource_size(addr_res);
    release_mem_region(addr_res->start, io_size);
failure_request_mem_region_addr:
    do {} while(0);
failure_platform_get_resource:
    platform_set_drvdata(pdev, NULL);
    free_netdev(ndev);

    return ret;
}

static int __devexit lydm9k_remove(struct platform_device *pdev)
{
    struct net_device *ndev = platform_get_drvdata(pdev);
    struct lydm9k_priv *priv = netdev_priv(ndev);
    size_t io_size;

    unregister_netdev(ndev);

    iounmap(priv->io_data);
    io_size = resource_size(priv->data_res);
    release_mem_region(priv->data_res->start, io_size);

    iounmap(priv->io_reg);
    io_size = resource_size(priv->addr_res);
    release_mem_region(priv->addr_res->start, io_size);

    platform_set_drvdata(pdev, NULL);
    free_netdev(ndev);

    printdbg("remove ok\n");

    return 0;
}

static struct platform_driver lydm9k_driver = {
    .driver			= {
        .name		= "lydm9k",
        .owner		= THIS_MODULE,
    },
    .probe			= lydm9k_probe,
    .remove			= __devexit_p(lydm9k_remove),
};

static struct lydm9k_plat_data lydm9k_plat_data = {
    .mac			= {0x00, 0x01, 0x02, 0x03, 0x04, 0x05},
    .watchdog_timeo_msecs = 5000,
};

static struct resource lydm9k_resource[] = {
    [0]			= {
        .start	= 0x20000300,
        .end	= 0x20000303,
        .flags	= IORESOURCE_MEM
    },
    [1]			= {
        .start	= 0x20000204,
        .end	= 0x20000207,
        .flags	= IORESOURCE_MEM
    },
    [2]			= {
        .start	= IRQ_EINT7,
        .end	= IRQ_EINT7,
        .flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE
    }
};

static void lydm9k_device_release(struct device *dev)
{
    printdbg("invoked\n");
}

static struct platform_device lydm9k_device = {
    .name			= "lydm9k",
    .id				= -1,
    .num_resources	= ARRAY_SIZE(lydm9k_resource),
    .resource		= lydm9k_resource,
    .dev			= {
        .platform_data	= &lydm9k_plat_data,
        .release	= lydm9k_device_release,
    },
};

static int __init lydm9k_init(void)
{
    dm9kmsg("insert module lydm9k\n");
    platform_device_register(&lydm9k_device);
    return platform_driver_register(&lydm9k_driver);
}

static void __exit lydm9k_exit(void)
{
    platform_device_unregister(&lydm9k_device);
    platform_driver_unregister(&lydm9k_driver);
    dm9kmsg("remove module lydm9k\n");
}

MODULE_LICENSE("GPL");
module_init(lydm9k_init);
module_exit(lydm9k_exit);

