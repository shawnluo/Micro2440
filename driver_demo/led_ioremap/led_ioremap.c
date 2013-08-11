


//insmod ioremap_driver.ko  ����
//rmmod ioremap_driver		���

#if 1
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <linux/ioport.h>
#define DEVICE_NAME    "led_decly"
#define LED1_ON        ~(1 << 5) //�͵�ƽ����LED
#define LED2_ON        ~(1 << 6)
#define LED3_ON        ~(1 << 7)
#define LED4_ON        ~(1 << 8)
#define LED1_OFF       (1 << 5)
#define LED2_OFF       (1 << 6)
#define LED3_OFF       (1 << 7)
#define LED4_OFF       (1 << 8)
#define GPBCON         0x56000010           //�Ĵ�����ַ�������ַ��
#define GPBDAT         0x56000014
static volatile unsigned long *gpbcon_addr; //����ioremapӳ���������ַ
static volatile unsigned long *gpbdat_addr;
static void Led_port_init(void)
{
    //����GPB5-GPB8Ϊ����˿�
    *gpbcon_addr &= ~((3 << 10) | (3 << 12) | (3 << 14) | (3 << 16));
    *gpbcon_addr |= (1 << 10) | (1 << 12) | (1 << 14) | (1 << 16);

    //ȫ��
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

    //����IO�ڴ棬���Ǳ���ġ�
    if (!request_mem_region(GPBCON, 8, "leds"))
    {
        ret = -EBUSY;
        goto request_mem_failed;
    }

    gpbcon_addr = ioremap(GPBCON, 4); //�������ַӳ��Ϊ�����ַ
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
    iounmap(gpbdat_addr);          //ȡ��ӳ��
    iounmap(gpbcon_addr);
    release_mem_region(GPBCON, 8); //�ͷ�I/O�ڴ�
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
//���ǵ������������������ڴ������еġ����������ڴ���ô��ʵ��Ӳ���ϵļĴ�����Ӧ������?
/*void * __ioremap(unsigned long phys_addr, unsigned long size, unsigned long flags)
 *
 * ��ڣ� phys_addr��Ҫӳ�����ʼ��IO��ַ�����������ַ
 *
 * size��Ҫӳ��Ŀռ�Ĵ�С��
 *
 * flags��Ҫӳ���IO�ռ�ĺ�Ȩ���йصı�־��
 *
 * ����������ioremap����д�ĵ�һ��LED ����������Ӳ����S3C2440�Ŀ����壩
 */

#include <linux/module.h>
#include <linux/init.h>
#include <asm/io.h>
volatile unsigned long virt, phys;               //���ڴ�������ַ�������ַ
volatile unsigned long *GPBCON, *GPBDAT, *GPBUP; //�����������Ĵ����ĵ�ַ
void led_device_init(void)
{
// 0x56000010 + 0x10 ����ȫ���е�IO���żĴ�����ַ
    phys = 0x56000010; // 0x56000010=GPBCON
//�������ַ�ռ�������һ�鳤��Ϊ0x10�������ռ�
//�����������ַphys��phys+0x10��Ӧ�����ַvirt��virt+0x10
    virt   = (unsigned long)ioremap(phys, 0x10);
    GPBCON = (unsigned long *)(virt + 0x00); //ָ����Ҫ�����������Ĵ����ĵ�ַ
    GPBDAT = (unsigned long *)(virt + 0x04);
    GPBUP  = (unsigned long *)(virt + 0x08);
}
//led���ú���,���ÿ������GPIO�ļĴ���
void led_configure(void)
{
    *GPBCON &= ~(3 << 10) & ~(3 << 12) & ~(3 << 16) & ~(3 << 20); //GPB12 defaule ����
    *GPBCON |= (1 << 10) | (1 << 12) | (1 << 16) | (1 << 20);     //output  ���ģʽ
    *GPBUP  |= (1 << 5) | (1 << 6) | (1 << 8) | (1 << 10);        //��ֹ��������
}
void led_on(void)                                                 //����led
{
    *GPBDAT &= ~(1 << 5) & ~(1 << 6) & ~(1 << 8) & ~(1 << 10);
}
void led_off(void) //���led
{
    *GPBDAT |= (1 << 5) | (1 << 6) | (1 << 8) | (1 << 10);
}
static int __init led_init(void) //ģ���ʼ������
{
    led_device_init();           //ʵ��IO�ڴ��ӳ��
    led_configure();             //����GPB5 6 8 10Ϊ���
    led_on();
    printk("hello ON!\n");
    return 0;
}
static void __exit led_exit(void) //ģ��ж�غ���
{
    led_off();
    iounmap((void *)virt); //����ӳ���ϵ
    printk("led OFF!\n");
}
module_init(led_init);
module_exit(led_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("hurryliu<>");
MODULE_VERSION("2012-8-5.1.0");
/*************************************************************************/
#endif
