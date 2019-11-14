ARCH= arm64
#MVTOOL_PREFIX = /opt/FriendlyARM/toolschain/4.5.1/bin/arm-linux-
#MVTOOL_PREFIX = /home/wsg/spi2serial/openwrt/staging_dir/toolchain-arm_gcc-andromeda/bin/arm-oe-linux-gnueabi-
MVTOOL_PREFIX = /home/wsg/spi2serial/i.mx8/openwrt/staging_dir/toolchain-arm64_gcc-imx8mm/bin/aarch64-linux-gnu-
CROSS_COMPILE= $(MVTOOL_PREFIX)
#KDIR := /opt/FriendlyARM/tiny210/linux/linux-3.0.8
#KDIR := /home/wsg/spi2serial/openwrt/build_dir/linux-andromeda_andromeda-iot/linux-3.18.20/
KDIR := /home/wsg/spi2serial/i.mx8/openwrt/build_dir/linux-imx8mm_NewCarGateway/linux-4.14.98
TARGET				=wk2xxx_spi
EXEC = $(TARGET)
obj-m :=$(TARGET).o
PWD :=$(shell pwd)
all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules
clean:
	rm -rf *.o *~core.depend.*.cmd *.ko *.mod.c .tmp_versions $(TARGET)
