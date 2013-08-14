#include <linux/module.h>
#include <linux/init.h>

#include <linux/platform_device.h>

void usb_dev_release(struct device *dev)
{
	printk("<kernel> release\n");
}
struct platform_device mouse_dev = {
    .name = "plat_usb_mouse",
	.id = -1,
    .dev = {
        .bus_id = "usb_mouse",
        .release = usb_dev_release,
    },
};

static int __init usb_device_init(void)
{
	int ret;

	ret = platform_device_register(&mouse_dev);
	if(ret){
		printk("device register failed!\n");
		return ret;	
	}

	printk("usb device init\n");
	return 0;
}

static void __exit usb_device_exit(void)
{
	platform_device_unregister(&mouse_dev);
	printk("usb device bye!\n");
}

module_init(usb_device_init);
module_exit(usb_device_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiao bai");
