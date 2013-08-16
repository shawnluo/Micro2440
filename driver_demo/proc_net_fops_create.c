#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/net.h>
#include <net/sock.h>
#include <linux/in.h>
#include <linux/types.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <asm-generic/unaligned.h>
#include <linux/sysctl.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <asm/checksum.h>
#include <linux/ip.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <net/net_namespace.h>
#include <net/route.h>
#include <linux/route.h>
#include <linux/stddef.h>

static int array[5] = {0, 1, 2, 3, 4};

static int myproc_open(struct inode *inodp, struct file *filp);

static struct file_operations myproc_fops = {
        .owner = THIS_MODULE,
        .open = myproc_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release,
};

static void * myproc_seq_start(struct seq_file *m, loff_t *pos)
{
        return *pos ? NULL : SEQ_START_TOKEN;
}

static void * myproc_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
    if (v == SEQ_START_TOKEN)
        return array;

        (*pos)++;        
        if (*pos >= 5)
                return NULL;

        return array + *pos;
}

static void myproc_seq_stop(struct seq_file *m, void *v)
{
        return;
}

static int myproc_seq_show(struct seq_file *m, void *v)
{
        int *p;

    if (v == SEQ_START_TOKEN)
        seq_puts(m, "This is the list.\n");
    else {
        p = (int *) v;
        seq_printf(m, "%d\n", *p);
    }

        return 0;
}

static struct seq_operations myproc_seq_ops = {
        .start = myproc_seq_start,
        .next = myproc_seq_next,
        .stop = myproc_seq_stop,
        .show = myproc_seq_show,
};

static int myproc_open(struct inode *inodp, struct file *filp)
{
        return seq_open(filp, &myproc_seq_ops);
}

static int __init myproc_init(void)
{
    proc_net_fops_create(&init_net, "myproc", 0, &myproc_fops);

        return 0;
}

static void __exit myproc_exit(void)
{
    proc_net_remove(&init_net, "myproc");
}

MODULE_LICENSE("GPL");

module_init(myproc_init);
module_exit(myproc_exit);

