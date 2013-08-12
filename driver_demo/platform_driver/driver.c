#include <linux/module.h>
#include <linux/init.h>

#include <linux/platform_device.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/leds-gpio.h>

int led_driver_probe(struct platform_device *pdev)
{
	struct s3c24xx_led_platdata *pdata = pdev->dev.platform_data;	
	
	s3c2410_gpio_cfgpin(pdata->gpio, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(pdata->gpio, 0);

	printk("led on\n");
	return 0;
}

int led_driver_remove(struct platform_device *pdev)
{
	struct s3c24xx_led_platdata *pdata = pdev->dev.platform_data;	

	s3c2410_gpio_setpin(pdata->gpio, 1);

	printk("led off\n");
	return 0;
}

struct platform_driver s3c_led_drv = {
	.probe  = led_driver_probe,
	.remove = led_driver_remove,
	.driver = {
		.name = "s3c2416_led",	//平台驱动名字，注意此名字必须和平台设备名字一致</span>
	},
};

static int __init plat_led_init(void)
{
	int ret;

	ret = platform_driver_register(&s3c_led_drv);
	if (ret) {
		printk("led register failed!\n");
		return ret;
	}

	printk("led dirver init\n");
	return 0;
}

static void __exit plat_led_exit(void)
{
	platform_driver_unregister(&s3c_led_drv);
	printk("led driver exit");
}

module_init(plat_led_init);
module_exit(plat_led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("laneyu<yulane.lang@gmail.com>");

