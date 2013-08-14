/*通过platform_driver_register和struct platform_driver来注册平台类驱动*/
#include <linux/module.h>
#include <linux/init.h>

#include <linux/platform_device.h>

void init_mouse(void)
{
	printk("init usb mouse\n");
}

int usb_driver_probe(struct platform_device *dev)
{//查询特定设备是否存在，以及是否能够才操作该设备，然后再进行设备操作。
	//check_mouse();	//自己假设一下检查设备
    init_mouse();		//usb鼠标驱动的真正入口
    return 0;
}

int usb_driver_remove(struct platform_device *dev)
{
    printk("remove mouse driver\n");
    return 0;
}
/*结构体中不需要指定总线的成员，交由usb_device_register来完成*/
struct platform_driver mouse_drv = {
    .probe = usb_driver_probe,
    .remove = usb_driver_remove,
    .driver = {
        .name = "plat_usb_mouse",	//在/sys/中的驱动目录名字
    },
};

static int __init usb_driver_init(void)
{
    int ret;
    /*驱动注册，注册成功后在/sys/platform/usb/driver目录下创建目录
	 * plat_usb_mouse*/
    ret = platform_driver_register(&mouse_drv);
    if(ret){
        printk("driver register failed!\n");
        return ret;	
    }
    printk("usb driver init\n");
    return 0;
}

static void __exit usb_driver_exit(void)
{
    platform_driver_unregister(&mouse_drv);
    printk("usb driver bye!\n");
}

module_init(usb_driver_init);
module_exit(usb_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiao bai");
