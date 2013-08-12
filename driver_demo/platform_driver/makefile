.PHONY: all clean

obj-m += device.o driver.o

KDIR := /home/kernel2416

all: 
	make ARCH=arm CROSS_COMPILE=arm-linux- -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	make ARCH=arm CROSS_COMPILE=arm-linux- -C $(KDIR) SUBDIRS=$(PWD) clean

