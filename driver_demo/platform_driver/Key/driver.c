//platform driver
#include <linux/module.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <mach/map.h>
#include <mach/regs-gpio.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <asm/unistd.h>
#include <linux/device.h>
#include <linux/sched.h>

static int buttons_irq[6];

struct irq_des
{
    int  *buttons_irq;
    char *name[6];
};


struct irq_des button_irqs =
{
    .buttons_irq = buttons_irq,
    .name        = { "KEY0",   "KEY1","KEY2", "KEY3", "KEY4", "KEY5" },
};

static volatile int key_values;


static DECLARE_WAIT_QUEUE_HEAD(button_waitq);


static volatile int ev_press = 0;


static irqreturn_t buttons_interrupt(int irq, void *dev_id)
{
    int i;

    for (i = 0; i < 6; i++)
    {
        if (irq == buttons_irq[i])
        {
            key_values = i;
            ev_press   = 1;
            wake_up_interruptible(&button_waitq);
        }
    }

    return IRQ_RETVAL(IRQ_HANDLED);
}


static int s3c24xx_buttons_open(struct inode *inode, struct file *file)
{
    int i;
    int err = 0;

    for (i = 0; i < 6; i++)
    {
        err = request_irq(button_irqs.buttons_irq[i], buttons_interrupt, IRQ_TYPE_EDGE_BOTH,
                          button_irqs.name[i], (void *)&button_irqs.buttons_irq[i]);
        if (err)
        {
            break;
        }
    }

    if (err)
    {
        i--;
        for ( ; i >= 0; i--)
        {
            if (button_irqs.buttons_irq[i] < 0)
            {
                continue;
            }
            disable_irq(button_irqs.buttons_irq[i]);
            free_irq(button_irqs.buttons_irq[i], (void *)&button_irqs.buttons_irq[i]);
        }
        return -EBUSY;
    }

    return 0;
}


static int s3c24xx_buttons_close(struct inode *inode, struct file *file)
{
    int i;

    for (i = 0; i < 6; i++)
    {
        free_irq(button_irqs.buttons_irq[i], (void *)&button_irqs.buttons_irq[i]);
    }

    return 0;
}


static int s3c24xx_buttons_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
    unsigned long err;

    if (!ev_press)
    {
        if (filp->f_flags & O_NONBLOCK)
        {
            return -EAGAIN;
        }
        else
        {
            wait_event_interruptible(button_waitq, ev_press);
        }
    }

    ev_press = 0;

    err = copy_to_user(buff, (const void *)&key_values, min(sizeof(key_values), count));

    return err ? -EFAULT : min(sizeof(key_values), count);
}

static unsigned int s3c24xx_buttons_poll(struct file *file, struct poll_table_struct *wait)
{
    unsigned int mask = 0;

    poll_wait(file, &button_waitq, wait);
    if (ev_press)
    {
        mask |= POLLIN | POLLRDNORM;
    }
    return mask;
}


static struct file_operations dev_fops =
{
    .owner   = THIS_MODULE,
    .open    = s3c24xx_buttons_open,
    .release = s3c24xx_buttons_close,
    .read    = s3c24xx_buttons_read,
    .poll    = s3c24xx_buttons_poll,
};

static struct miscdevice misc =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "my_buttons",
    .fops  = &dev_fops,
};


static int my_plat_probe(struct platform_device *dev)
{
    int                    ret, i;
    struct resource        *plat_resource;
    struct platform_device *pdev = dev;

    printk("my platform dirver find my platfrom device.\n");

    for (i = 0; i < 6; i++)
    {
        plat_resource = platform_get_resource(pdev, IORESOURCE_IRQ, i);
        if (plat_resource == NULL)
        {
            return -ENOENT;
        }
        buttons_irq[i] = plat_resource->start;
    }

    ret = misc_register(&misc);
    if (ret)
    {
        return ret;
    }


    return 0;
}

static int my_plat_remove(struct platform_device *dev)
{
    printk("my platfrom device has removed.\n");
    misc_deregister(&misc);
    return 0;
}

struct platform_driver my_buttons_drv =
{
    .probe  = my_plat_probe,
    .remove = my_plat_remove,
    .driver =
    {
        .owner = THIS_MODULE,
        .name  = "my_buttons",
    },
};

static int __init platform_drv_init(void)
{
    int ret;

    ret = platform_driver_register(&my_buttons_drv);

    return ret;
}

static void __exit platform_drv_exit(void)
{
    platform_driver_unregister(&my_buttons_drv);
}

module_init(platform_drv_init);
module_exit(platform_drv_exit);

MODULE_AUTHOR("Y-Kee");
MODULE_LICENSE("GPL");
