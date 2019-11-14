ARCH= arm64

export STAGING_DIR=$STAGING_DIR:/home/wsg/ubox/openwrt/

#MVTOOL_PREFIX = /opt/FriendlyARM/toolschain/4.5.1/bin/arm-linux-
#MVTOOL_PREFIX = /home/wsg/spi2serial/i.mx8/openwrt/staging_dir/toolchain-arm64_gcc-imx8mm/bin/aarch64-linux-gnu-
MVTOOL_PREFIX = /home/wsg/ubox/openwrt/staging_dir/toolchain-aarch64_cortex-a53+neon_gcc-8.3.0_musl/bin/aarch64-openwrt-linux-
CROSS_COMPILE= $(MVTOOL_PREFIX)

#KDIR := /opt/FriendlyARM/tiny210/linux/linux-3.0.8
#KDIR := /home/wsg/spi2serial/i.mx8/openwrt/build_dir/linux-imx8mm_NewCarGateway/linux-4.14.98
KDIR := /home/wsg/ubox/openwrt/build_dir/target-aarch64_cortex-a53+neon_musl/linux-imx8/linux-4.14.98
TARGET				=wk2xxx_uart
EXEC = $(TARGET)
obj-m :=$(TARGET).o
PWD :=$(shell pwd)
all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules
clean:
	rm -rf *.o *~core.depend.*.cmd *.ko *.mod.c .tmp_versions $(TARGET)

