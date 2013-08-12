#include <linux/module.h>
#include <linux/init.h>

#include <linux/platform_device.h>

//#include <asm/arch/regs-gpio.h>
//#include <asm/arch/leds-gpio.h>

static void led_dev_release(struct device *dev)
{
	printk("<kernel> release\n");
}

static struct s3c24xx_led_platdata s3c2416_pdata_led4 = {			//LED控制数据
	.gpio		= S3C2410_GPK5,
	.flags		= S3C24XX_LEDF_ACTLOW,
	.name		= "led4"
};

static struct platform_device s3c_led_dev = {			//定义一个LED平台设备
	.name = "s3c2416_led",								//平台设备名字，此名字很重要</span>
	.id   = 0,
	.dev = {
		.platform_data = &s3c2416_pdata_led4,
		.release = led_dev_release,						//平台设备撤销的时候会调用，如果没有则会出错，可以试试
	},
};


static int __init led_device_init(void)					//模块初始化，主要注册该设备
{
	int ret;
	
	ret = platform_device_register(&s3c_led_dev);
	if (ret) {
		printk("device register failed\n");
		return ret;
	}

	printk("led device init\n");
	return 0;
}

static void __exit led_device_exit(void)
{
	platform_device_unregister(&s3c_led_dev);
	printk("led device bye!\n");
}

module_init(led_device_init);
module_exit(led_device_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("laneyu<yulane.lang@gmail.com>");

