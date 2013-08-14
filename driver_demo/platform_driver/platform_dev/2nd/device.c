#include <linux/module.h>
#include <linux/init.h>

#include <linux/platform_device.h>

void led_dev_release(struct device *dev)
{
	printk("<kernel> release\n");
}
struct resource s3c_led_res[1] = {
	[0] = {
		.start = 0x56000000,
		.end = 0x560000ff,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device s3c_led_dev = {
    .name = "plat_led",
	.id = -1,
    .dev = {
        .release = led_dev_release,
    },
	.num_resources = ARRAY_SIZE(s3c_led_res),	//platform资源的数量，为1
	.resource = s3c_led_res,
};

static int __init led_device_init(void)
{
	int ret;

	ret = platform_device_register(&s3c_led_dev);
	if(ret){
		printk("device register failed!\n");
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
MODULE_AUTHOR("xiao bai");
