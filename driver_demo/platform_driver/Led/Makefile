.PHONY: all clean

obj-m += device.o driver.o

KDIR := /home/wallace/workSPACE/SourceLib/linux-2.6.32.2-mini2440-20100106

all: 
	make ARCH=arm CROSS_COMPILE=arm-linux- -C $(KDIR) SUBDIRS=$(PWD) modules -I $(KDIR)/arch/arm/mach-s3c2410/include/mach

clean:
	make ARCH=arm CROSS_COMPILE=arm-linux- -C $(KDIR) SUBDIRS=$(PWD) clean

