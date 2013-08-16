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

#define err(msg) printk(KERN_INFO "%s failed.\n", msg)

static int sysctl_sg_vs_amemthresh = 1024;

static int proc_do_defense_mode(ctl_table *table, int write, struct file * filp, void __user *buffer, size_t *lenp, loff_t *ppos)
{
    int rc;
    int *data = table->data;

    printk(KERN_INFO "original value = %d\n", *data);

    rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
    if (write)
        printk(KERN_INFO "this is write operation, current value = %d\n", *data);

    return rc;
}

static struct ctl_table sg_vars[] = {
    {
        .procname    = "amemthresh",
        .data        = &sysctl_sg_vs_amemthresh,
        .maxlen        = sizeof(int),
        .mode        = 0644,
        .proc_handler    = proc_do_defense_mode,
    },
    {
        .ctl_name = 0,
    },
};

static struct ctl_table_header *sysctl_header;

static int __init main_init(void)
{
    sysctl_header = register_sysctl_table(sg_vars);
    if (sysctl_header == NULL) {
        err("register_sysctl_table");
        goto out;
    }

    printk(KERN_INFO "sysctl register success.\n");
    return 0;
out:
    return -1;
}

static void __exit main_exit(void)
{
    unregister_sysctl_table(sysctl_header);
    printk(KERN_INFO "sysctl unregister success.\n");
}

module_init(main_init);
module_exit(main_exit);
MODULE_LICENSE("GPL");

