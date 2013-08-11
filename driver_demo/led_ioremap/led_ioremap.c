


//insmod ioremap_driver.ko  亮灯
//rmmod ioremap_driver		灭灯

#if 1
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <linux/ioport.h>
#define DEVICE_NAME    "led_decly"
#define LED1_ON        ~(1 << 5) //低电平点亮LED
#define LED2_ON        ~(1 << 6)
#define LED3_ON        ~(1 << 7)
#define LED4_ON        ~(1 << 8)
#define LED1_OFF       (1 << 5)
#define LED2_OFF       (1 << 6)
#define LED3_OFF       (1 << 7)
#define LED4_OFF       (1 << 8)
#define GPBCON         0x56000010           //寄存器地址（物理地址）
#define GPBDAT         0x56000014
static volatile unsigned long *gpbcon_addr; //经过ioremap映射后的虚拟地址
static volatile unsigned long *gpbdat_addr;
static void Led_port_init(void)
{
    //设置GPB5-GPB8为输出端口
    *gpbcon_addr &= ~((3 << 10) | (3 << 12) | (3 << 14) | (3 << 16));
    *gpbcon_addr |= (1 << 10) | (1 << 12) | (1 << 14) | (1 << 16);

    //全亮
    *gpbdat_addr &= LED1_ON & LED2_ON & LED3_ON & LED4_ON;
}

static void led_turn_on(unsigned int led_nu)
{
    switch (led_nu)
    {
    case 1:
        *gpbdat_addr &= LED1_ON;
        break;

    case 2:
        *gpbdat_addr &= LED2_ON;
        break;

    case 3:
        *gpbdat_addr &= LED3_ON;
        break;

    case 4:
        *gpbdat_addr &= LED4_ON;
        break;

    default:
        break;
    }
}
static void led_turn_off(unsigned int led_nu)
{
    switch (led_nu)
    {
    case 1:
        *gpbdat_addr |= LED1_OFF;
        break;

    case 2:
        *gpbdat_addr |= LED2_OFF;
        break;

    case 3:
        *gpbdat_addr |= LED3_OFF;
        break;

    case 4:
        *gpbdat_addr |= LED4_OFF;
        break;

    default:
        break;
    }
}
static int led_open(struct inode *inode, struct file *filp)
{
    return 0;
}
static int led_release(struct inode *inode, struct file *filp)
{
    return 0;
}
static int led_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    switch (cmd)
    {
    case 0:
        led_turn_off(arg);
        break;

    case 1:
        led_turn_on(arg);
        break;

    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}
static const struct file_operations led_fops =
{
    .owner   = THIS_MODULE,
    .open    = led_open,
    .release = led_release,
    .ioctl   = led_ioctl,
};
static struct miscdevice            led_dev =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name  = DEVICE_NAME,
    .fops  = &led_fops,
};
static int __init led_init(void)
{
    int ret;

    //申请IO内存，不是必须的。
    if (!request_mem_region(GPBCON, 8, "leds"))
    {
        ret = -EBUSY;
        goto request_mem_failed;
    }

    gpbcon_addr = ioremap(GPBCON, 4); //将物理地址映射为虚拟地址
    if (NULL == gpbcon_addr)
    {
        ret = -EIO;
        printk("gpbcon remap failed\n");
        goto con_map_failed;
    }
    gpbdat_addr = ioremap(GPBDAT, 4);
    if (NULL == gpbdat_addr)
    {
        ret = -EIO;
        printk("gpbdat remap failed\n");
        goto dat_map_failed;
    }
    printk("gpbcon_addr remap on %p\n", gpbcon_addr);
    printk("gpbdat_addr remap on %p\n", gpbdat_addr);

    Led_port_init();
    ret = misc_register(&led_dev);
    if (ret)
    {
        printk("misc_register failed\n");
        goto failed;
    }

    printk("leds init\n");
    return 0;

 failed:
    iounmap(gpbdat_addr);
 dat_map_failed:
    iounmap(gpbcon_addr);
 con_map_failed:
    release_mem_region(GPBCON, 8);
 request_mem_failed:
    return ret;
}
static void __exit led_exit(void)
{
    iounmap(gpbdat_addr);          //取消映射
    iounmap(gpbcon_addr);
    release_mem_region(GPBCON, 8); //释放I/O内存
    misc_deregister(&led_dev);

    printk("leds exit\n");
}
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Decly");
module_init(led_init);
module_exit(led_exit);


#else
/***************************************************************/
//file name: ioremap_driver.c
//我们的驱动程序是在虚拟内存上运行的。但是虚拟内存怎么和实际硬件上的寄存器对应起来呢?
/*void * __ioremap(unsigned long phys_addr, unsigned long size, unsigned long flags)
 *
 * 入口： phys_addr：要映射的起始的IO地址，即：物理地址
 *
 * size：要映射的空间的大小；
 *
 * flags：要映射的IO空间的和权限有关的标志；
 *
 * 下面是我用ioremap函数写的第一个LED 的驱动：（硬件是S3C2440的开发板）
 */

#include <linux/module.h>
#include <linux/init.h>
#include <asm/io.h>
volatile unsigned long virt, phys;               //用于存放虚拟地址和物理地址
volatile unsigned long *GPBCON, *GPBDAT, *GPBUP; //用与存放三个寄存器的地址
void led_device_init(void)
{
// 0x56000010 + 0x10 包揽全所有的IO引脚寄存器地址
    phys = 0x56000010; // 0x56000010=GPBCON
//在虚拟地址空间中申请一块长度为0x10的连续空间
//这样，物理地址phys到phys+0x10对应虚拟地址virt到virt+0x10
    virt   = (unsigned long)ioremap(phys, 0x10);
    GPBCON = (unsigned long *)(virt + 0x00); //指定需要操作的三个寄存器的地址
    GPBDAT = (unsigned long *)(virt + 0x04);
    GPBUP  = (unsigned long *)(virt + 0x08);
}
//led配置函数,配置开发板的GPIO的寄存器
void led_configure(void)
{
    *GPBCON &= ~(3 << 10) & ~(3 << 12) & ~(3 << 16) & ~(3 << 20); //GPB12 defaule 清零
    *GPBCON |= (1 << 10) | (1 << 12) | (1 << 16) | (1 << 20);     //output  输出模式
    *GPBUP  |= (1 << 5) | (1 << 6) | (1 << 8) | (1 << 10);        //禁止上拉电阻
}
void led_on(void)                                                 //点亮led
{
    *GPBDAT &= ~(1 << 5) & ~(1 << 6) & ~(1 << 8) & ~(1 << 10);
}
void led_off(void) //灭掉led
{
    *GPBDAT |= (1 << 5) | (1 << 6) | (1 << 8) | (1 << 10);
}
static int __init led_init(void) //模块初始化函数
{
    led_device_init();           //实现IO内存的映射
    led_configure();             //配置GPB5 6 8 10为输出
    led_on();
    printk("hello ON!\n");
    return 0;
}
static void __exit led_exit(void) //模块卸载函数
{
    led_off();
    iounmap((void *)virt); //撤销映射关系
    printk("led OFF!\n");
}
module_init(led_init);
module_exit(led_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("hurryliu<>");
MODULE_VERSION("2012-8-5.1.0");
/*************************************************************************/
#endif
