obj-m += acb.o amidi.o

KDIR ?= /lib/modules/`uname -r`/build

default: acb.ko amidi.ko

amidi.ko:
	$(MAKE) -C $(KDIR) M=$$PWD amidi.ko
acb.ko:
	$(MAKE) -C $(KDIR) M=$$PWD acb.ko

clean:
	rm -f *.ko *.o
