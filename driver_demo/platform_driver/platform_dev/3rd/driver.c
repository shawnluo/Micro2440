/*通过platform_driver_register和struct platform_driver来注册平台类驱动*/
#include <linux/module.h>
#include <linux/init.h>

#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/sizes.h>

struct _plat_led_t {
	unsigned long phys, virt;
	unsigned long gpecon, gpedat, gpeup;
	unsigned long reg;
};

struct _plat_led_t pled;

int led_driver_probe(struct platform_device *pdev)
{
	pled.phys = pdev->resource[0].start;
	pled.virt = ioremap(pled.phys, SZ_4K);
	pled.gpecon = pled.virt + 0x40;
	pled.gpedat = pled.virt + 0x44;
	pled.gpeup = pled.virt + 0x48;

	//config
	pled.reg = ioread32(pled.gpecon);
	pled.reg &= ~(3 << 24);
	pled.reg |= (1 << 24);
	iowrite32(pled.reg, pled.gpecon);	
	
	//up
	pled.reg = ioread32(pled.gpeup);
	pled.reg |= (1 << 12);
	iowrite32(pled.reg, pled.gpeup);	
	
	//dat
	pled.reg = ioread32(pled.gpedat);
	pled.reg &= ~(1 << 12);
	iowrite32(pled.reg, pled.gpedat);

	printk("led on\n");
    return 0;
}

int led_driver_remove(struct platform_device *pdev)
{
	pled.reg = ioread32(pled.gpedat);
	pled.reg |= (1 << 12);
	iowrite32(pled.reg, pled.gpedat);

	printk("led off\n");
    return 0;
}

struct platform_driver s3c_led_drv = {
    .probe = led_driver_probe,
    .remove = led_driver_remove,
    .driver = {
        .name = "plat_led",	//在/sys/中的驱动目录名字
    },
};

static int __init usb_driver_init(void)
{
    int ret;

    ret = platform_driver_register(&s3c_led_drv);
    if(ret){
        printk("led register failed!\n");
        return ret;	
    }
    printk("led driver init\n");
    return 0;
}

static void __exit usb_driver_exit(void)
{
    platform_driver_unregister(&s3c_led_drv);
    printk("led driver bye!\n");
}

module_init(usb_driver_init);
module_exit(usb_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiao bai");
