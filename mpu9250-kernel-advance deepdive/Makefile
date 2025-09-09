# Makefile
# For building MPU9250 kernel module and user-space tests on Raspberry Pi 4.
# Supports cross-compilation for arm64 (Pi 4).
# Usage: make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
# Install: make install
# Test: ./user_test, ./mq_test, or ./aio_test
# Author: Nguyen Nhan

obj-m += mpu9250_driver.o mpu9250_iio.o

KDIR ?= /lib/modules/$(shell uname -r)/build
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-
CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall -Wextra -std=c99 -I$(KDIR)/include
EXTRA_CFLAGS += -DCONFIG_MPU9250_DEBUG -DCONFIG_MPU9250_TRACE -DCONFIG_MPU9250_IIO

all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean
	rm -f user_test mq_test aio_test *.o *.ko *.mod.* Module.symvers modules.order mpu9250.log
	rm -f mpu9250_iio.o mpu9250_iio.ko mpu9250_iio.mod.*

user_test:
	$(CC) -o user_test user_space_test.c -lm -pthread -lrt -laio $(CFLAGS)

mq_test:
	$(CC) -o mq_test mq_test.c -lrt -pthread $(CFLAGS)

aio_test:
	$(CC) -o aio_test aio_test.c -laio -pthread $(CFLAGS)

install:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules_install
	sudo insmod mpu9250_driver.ko
	sudo insmod mpu9250_iio.ko
	major=$$(awk '/mpu9250/ {print $$1}' /proc/devices); \
	sudo mknod /dev/mpu9250 c $$major 0; \
	sudo chmod 660 /dev/mpu9250; \
	sudo chgrp iio /dev/mpu9250; \
	sudo mknod /dev/iio:device0 c $$(awk '/iio:device0/ {print $$1}' /proc/devices) 0; \
	sudo chmod 660 /dev/iio:device0; \
	sudo chgrp iio /dev/iio:device0; \
	echo 'SUBSYSTEM=="char", KERNEL=="mpu9250", MODE="0660", GROUP="iio", OWNER="root", SECLABEL="{selinux}{mls}{s0:c0.c1023}"' | sudo tee /etc/udev/rules.d/99-mpu9250.rules; \
	echo 'SUBSYSTEM=="iio", KERNEL=="iio:device0", MODE="0660", GROUP="iio", OWNER="root", SECLABEL="{selinux}{mls}{s0:c0.c1023}"' | sudo tee -a /etc/udev/rules.d/99-mpu9250.rules
	sudo udevadm control --reload-rules && sudo udevadm trigger

uninstall:
	sudo rmmod mpu9250_driver mpu9250_iio
	sudo rm -f /dev/mpu9250 /dev/iio:device0
	sudo rm -f /etc/udev/rules.d/99-mpu9250.rules
	sudo udevadm control --reload-rules && sudo udevadm trigger

log:
	./user_test --log mpu9250.log