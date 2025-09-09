# Makefile
# For building MPU9250 kernel module and user-space tests on Raspberry Pi 4.
# Supports cross-compilation for arm64 (Pi 4).
# Usage: make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
# Install: make install
# Test: ./user_test or ./mq_test

obj-m += mpu9250_driver.o

KDIR ?= /lib/modules/$(shell uname -r)/build
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-
CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall -Wextra -std=c99
EXTRA_CFLAGS += -DCONFIG_MPU9250_DEBUG

all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean
	rm -f user_test mq_test *.o *.ko *.mod.* Module.symvers modules.order

user_test:
	$(CC) -o user_test user_space_test.c -lm -pthread $(CFLAGS)

mq_test:
	$(CC) -o mq_test mq_test.c -lrt -pthread $(CFLAGS)

install:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules_install
	sudo insmod mpu9250_driver.ko
	major=$$(awk '/mpu9250/ {print $$1}' /proc/devices); \
	sudo mknod /dev/mpu9250 c $$major 0; \
	sudo chmod 666 /dev/mpu9250; \
	echo 'SUBSYSTEM=="char", KERNEL=="mpu9250", MODE="0666"' | sudo tee /etc/udev/rules.d/99-mpu9250.rules
	sudo udevadm control --reload-rules && sudo udevadm trigger

uninstall:
	sudo rmmod mpu9250_driver
	sudo rm -f /dev/mpu9250
	sudo rm -f /etc/udev/rules.d/99-mpu9250.rules
	sudo udevadm control --reload-rules && sudo udevadm trigger