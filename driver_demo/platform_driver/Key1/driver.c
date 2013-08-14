
//platform驱动模块代码

#include <linux/device.h>

#include <linux/module.h>

#include <linux/kernel.h>

#include <linux/init.h>

#include <linux/string.h>

#include <linux/platform_device.h>

#include <asm/io.h>

#include <linux/poll.h>

#include <linux/irq.h>

#include <linux/interrupt.h>

#include <linux/miscdevice.h>

#include <linux/sched.h>

MODULE_AUTHOR("WJB");

MODULE_LICENSE("Dual BSD/GPL");

 

#define BUTTONS_12INPUT_MASK 0x41

struct button_irq_desc { //私有数据结构体

int number;

char *name;

};

static struct button_irq_desc buttons_irqs [] = { //私有数据

{ 0, "KEY1"},

{ 1, "KEY2"},

};

 

static volatile char key_values [] = {'0', '0'};

static DECLARE_WAIT_QUEUE_HEAD(button_waitq); //定义等待队列

static volatile int ev_press = 0;

 

static struct resource *buttons_irq1;

static struct resource *buttons_irq2;

static struct resource *buttons_mem;

static void __iomem *buttons_base;

 

static irqreturn_t s3c2410buttons_irq(int irq, void *dev_id)

{

struct button_irq_desc *buttons_irqs = (struct button_irq_desc *)dev_id;

unsigned int tmp;

void __iomem *base = buttons_base;

tmp=readb(base+0x04);

if(buttons_irqs->number==0)

{

tmp &=0x01;

}else{

tmp &=0x08;

}

// process data

if (tmp == (key_values[buttons_irqs->number] & 1)) { // Changed

key_values[buttons_irqs->number] = '1' ;

}

ev_press = 1;

wake_up_interruptible(&button_waitq);

return IRQ_RETVAL(IRQ_HANDLED);

}

 

static int s3c24xx_buttons_open(struct inode *inode, struct file *file)

{ int ret;

unsigned int tmp;

void __iomem *base = buttons_base;

// set key1 and key2 input

tmp=readb(base);

writeb(tmp|BUTTONS_12INPUT_MASK ,base);

ret = request_irq(buttons_irq1->start, s3c2410buttons_irq,IRQ_TYPE_EDGE_FALLING, "KET1", (void *)&buttons_irqs[0]);

if (ret != 0) {

printk( "failed to install irq (%d)\n", ret);

goto err1;

}

ret = request_irq(buttons_irq2->start, s3c2410buttons_irq, IRQ_TYPE_EDGE_FALLING, "KET2", (void *)&buttons_irqs[1]);

if (ret != 0) {

printk( "failed to install irq (%d)\n", ret);

goto err2;

}

ev_press = 1;

 

return 0;

err2: disable_irq(buttons_irq2->start);

free_irq(buttons_irq2->start, (void *)&buttons_irqs[1]);

err1: disable_irq(buttons_irq1->start);

free_irq(buttons_irq1->start, (void *)&buttons_irqs[0]);

 

return -EBUSY;

}

 

static int s3c24xx_buttons_close(struct inode *inode, struct file *file)

{

disable_irq(buttons_irq2->start);

free_irq(buttons_irq2->start, (void *)&buttons_irqs[1]);

disable_irq(buttons_irq1->start);

free_irq(buttons_irq1->start, (void *)&buttons_irqs[0]);

return 0;

}

 

static int s3c24xx_buttons_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)

{

unsigned long err;

int i;

if (!ev_press) {

if (filp->f_flags & O_NONBLOCK)

return -EAGAIN;

else

wait_event_interruptible(button_waitq, ev_press);

}

ev_press = 0;

err = copy_to_user(buff, (const void *)key_values, min(sizeof(key_values), count));

for (i=0;i<2;i++)

{

key_values[i]='0';

}

return err ? -EFAULT : min(sizeof(key_values), count);

}

 

static struct file_operations dev_fops = {

.owner = THIS_MODULE,

.open = s3c24xx_buttons_open,

.release = s3c24xx_buttons_close,

.read = s3c24xx_buttons_read,

};

static struct miscdevice s3c2410buttons_miscdev = {

.minor = MISC_DYNAMIC_MINOR,

.name = "s3c2410-buttons",

.fops = &dev_fops,

};

 

static int my_probe(struct platform_device* pdev)

{

int ret;

struct resource *res;

struct device *dev;

dev = &pdev->dev;

// get resource

res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

if (res == NULL) {

dev_err(&pdev->dev, "failed to get memory region resource\n");

return -ENOENT;

}

buttons_mem = request_mem_region(res->start,

res->end-res->start+1,

pdev->name);

if (buttons_mem == NULL) {

dev_err(&pdev->dev, "failed to reserve memory region\n");

ret = -ENOENT;

goto err_nores;

}

buttons_base = ioremap(res->start, res->end - res->start + 1);

if (buttons_base == NULL) {

dev_err(&pdev->dev, "failed ioremap()\n");

ret = -EINVAL;

goto err_nores;

}

//get key1 interrupt

buttons_irq1 = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

if (buttons_irq1 == NULL) {

dev_err(dev, "no irq resource specified\n");

ret = -ENOENT;

goto err_map;

}

//get key2 interrupt

buttons_irq2 = platform_get_resource(pdev, IORESOURCE_IRQ, 1);

if (buttons_irq2 == NULL) {

dev_err(dev, "no irq resource specified\n");

ret = -ENOENT;

goto err_map;

}

// register misc device

ret = misc_register(&s3c2410buttons_miscdev);

if (ret) {

dev_err(dev, "cannot register miscdev on minor=%d (%d)\n",

WATCHDOG_MINOR, ret);

goto err_map;

}

 

printk("driver found device which my driver can handle!\n");

err_map:

iounmap(buttons_base);

err_nores:

release_resource(buttons_mem);

kfree(buttons_mem);

 

return ret;

}

 

 

static int my_remove(struct platform_device* pdev)

{

release_resource(buttons_mem);

kfree(buttons_mem);

buttons_mem = NULL;

free_irq(buttons_irq1->start, (void *)&buttons_irqs[0]);

buttons_irq1 = NULL;

free_irq(buttons_irq2->start, (void *)&buttons_irqs[1]);

buttons_irq2 = NULL;

iounmap(buttons_base);

misc_deregister(&s3c2410buttons_miscdev);

printk("drvier found device unpluged!/n");

return 0;

}

 

static struct platform_driver my_driver = {

.probe = my_probe,

.remove = my_remove,

.driver = {

.owner = THIS_MODULE,

.name = "s3c2410-buttons",

},

};

 

static int __init my_driver_init(void)

{

return platform_driver_register(&my_driver);

}

 

static void my_driver_exit(void)

{

platform_driver_unregister(&my_driver);

}

 

module_init(my_driver_init);

module_exit(my_driver_exit);