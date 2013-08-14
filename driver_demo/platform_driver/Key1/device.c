//platform�豸ģ�����

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/stat.h>

#define GPGCON 0x56000060 //���ƶ˿ڵ�ַ
#define GPGDAT 0x56000064 //���ݶ˿ڵ�ַ

ssize_t test_show(struct device *dev, struct attribute *attr, char *buf);

ssize_t test_store(struct device *dev, struct attribute *attr, char *buf,size_t count);

 

static DEVICE_ATTR(buttons, S_IRWXUGO, test_show, test_store); //�豸����

 

ssize_t test_show(struct device *dev, struct attribute *attr, char *buf) //���豸����

{

printk("call : test_show . \n");

printk("attrname:%s . \n",attr->name);

sprintf(buf,"%s\n",attr->name);

return strlen(attr->name)+2;

}

 

ssize_t test_store(struct device *dev, struct attribute *attr, char *buf,size_t count) //д�豸����

{

printk("call : test_store . \n");

printk("write : %s . \n",buf);

strcpy(attr->name,buf);

return count;

}

 

static struct resource s3c_buttons_resource[]=

{

[0]={ //�ڴ���Դ

.start = GPGCON,

.end = GPGDAT,

.flags=IORESOURCE_MEM,

},

[1]={ //�жϺ�

//KEY1

.start = IRQ_EINT8,

.end = IRQ_EINT8,

.flags=IORESOURCE_IRQ,

},

[2]={

//KEY2

.start = IRQ_EINT11,

.end = IRQ_EINT11,

.flags=IORESOURCE_IRQ,

},

};

 

MODULE_AUTHOR("WJB");

MODULE_LICENSE("Dual BSD/GPL");

 

static struct platform_device *my_device = NULL;

 

static int __init my_device_init(void)

{

int ret = 0;

my_device = platform_device_alloc("s3c2410-buttons", -1); //����ƽ̨�豸

platform_device_add_resources(my_device, s3c_buttons_resource, 3); //�����Դ

ret = platform_device_add(my_device); //ע��ƽ̨�豸

device_create_file(&my_device->dev,&dev_attr_buttons); //����豸����

if(ret)

platform_device_put(my_device);

return ret;

}

 

static void my_device_exit(void)

{

platform_device_unregister(my_device);

device_remove_file(&my_device->dev,&dev_attr_buttons);

}

 

module_init(my_device_init);

module_exit(my_device_exit);

 


 

 

