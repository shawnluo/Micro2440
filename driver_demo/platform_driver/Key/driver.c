#include <linux/device.h>  
#include <linux/string.h>  
#include <linux/platform_device.h>  
#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/fs.h>  
#include <linux/init.h>  
#include <linux/delay.h>  
#include <linux/poll.h>  
#include <linux/irq.h>  
#include <asm/irq.h>  
#include <linux/interrupt.h>  
#include <asm/uaccess.h>  
#include <mach/regs-gpio.h>  
#include <mach/hardware.h>  
#include <linux/cdev.h>  
#include <linux/miscdevice.h>  
#include <linux/sched.h>  
#include <linux/gpio.h>  
  
static struct resource key_resource[]=  
{     
    [0] = {  
        .start = IRQ_EINT8,  
        .end = IRQ_EINT8,  
        .flags = IORESOURCE_IRQ,  
    },  
    [1] = {  
        .start = IRQ_EINT11,  
        .end = IRQ_EINT11,  
        .flags = IORESOURCE_IRQ,  
    },  
    [2]= {  
        .start = IRQ_EINT13,  
        .end = IRQ_EINT13,  
        .flags = IORESOURCE_IRQ,  
    },  
    [3] = {  
        .start = IRQ_EINT14,  
        .end = IRQ_EINT14,  
        .flags = IORESOURCE_IRQ,  
    },  
    [4] = {  
        .start = IRQ_EINT15,  
        .end = IRQ_EINT15,  
        .flags = IORESOURCE_IRQ,  
    },  
    [5] = {  
        .start = IRQ_EINT19,  
        .end = IRQ_EINT19,  
        .flags = IORESOURCE_IRQ,  
    },  
};  
  
struct platform_device *my_buttons_dev;  
  
static int __init platform_dev_init(void)  
{  
    int ret;  
      
    my_buttons_dev = platform_device_alloc("my_buttons", -1);  
      
    platform_device_add_resources(my_buttons_dev,key_resource,6);//添加资源一定要用该函数，不能使用对platform_device->resource幅值  
                                                                //否则会导致platform_device_unregister调用失败，内核异常。  
      
    ret = platform_device_add(my_buttons_dev);  
      
    if(ret)  
        platform_device_put(my_buttons_dev);  
      
    return ret;  
}  
  
static void __exit platform_dev_exit(void)  
{  
    platform_device_unregister(my_buttons_dev);  
}  
  
module_init(platform_dev_init);  
module_exit(platform_dev_exit);  
  
MODULE_AUTHOR("Y-Kee");  
MODULE_LICENSE("GPL");  

