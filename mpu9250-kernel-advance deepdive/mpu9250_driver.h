// mpu9250_driver.h
// Header file for MPU9250 kernel driver on Raspberry Pi 4 with IIO and AIO support.
// Author: Nguyen Nhan


#ifndef _MPU9250_DRIVER_H_
#define _MPU9250_DRIVER_H_

#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/semaphore.h>

#define MPU9250_DEFAULT_ADDR    0x68
#define MPU9250_DEVICE_NAME     "mpu9250"
#define MPU9250_CLASS_NAME      "mpu9250_class"
#define MPU9250_MAX_FIFO        4096
#define MPU9250_SHM_NAME        "/mpu9250_shm"
#define MPU9250_MQ_NAME         "/mpu9250_mq"
#define MPU9250_SEM_NAME        "/mpu9250_sem"
#define DRIVER_VERSION          "2.3"
#define MPU9250_WATCHDOG_TIMEOUT (HZ * 5) // 5 seconds

// Registers
#define MPU9250_WHO_AM_I        0x75
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_PWR_MGMT_2      0x6C
#define MPU9250_ACCEL_XOUT_H    0x3B
#define MPU9250_ACCEL_YOUT_H    0x3D
#define MPU9250_ACCEL_ZOUT_H    0x3F
#define MPU9250_TEMP_OUT_H      0x41
#define MPU9250_GYRO_XOUT_H     0x43
#define MPU9250_GYRO_YOUT_H     0x45
#define MPU9250_GYRO_ZOUT_H     0x47
#define MPU9250_MAG_XOUT_L      0x03
#define MPU9250_MAG_YOUT_L      0x05
#define MPU9250_MAG_ZOUT_L      0x07
#define MPU9250_FIFO_EN         0x23
#define MPU9250_FIFO_COUNTH     0x72
#define MPU9250_FIFO_COUNTL     0x73
#define MPU9250_FIFO_R_W        0x74
#define MPU9250_DMP_CFG_1       0x01
#define MPU9250_DMP_CFG_2       0x02
#define MPU9250_USER_CTRL       0x6A
#define MPU9250_INT_ENABLE      0x38
#define MPU9250_INT_STATUS      0x3A
#define MPU9250_SMPLRT_DIV      0x19
#define MPU9250_CONFIG          0x1A
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_BYPASS_EN       0x37

// IRQ types
enum mpu9250_irq_type {
    MPU9250_INTERRUPT_MOTION = 0x00,
    MPU9250_INTERRUPT_FIFO_OVERFLOW = 0x01,
    MPU9250_INTERRUPT_FSYNC_INT = 0x02,
    MPU9250_INTERRUPT_DMP_INT = 0x03,
    MPU9250_INTERRUPT_DATA_READY = 0x04,
};

// DMP orientations
enum mpu9250_dmp_tap {
    MPU9250_DMP_TAP_X_UP = 0x00,
    MPU9250_DMP_TAP_X_DOWN = 0x01,
    MPU9250_DMP_TAP_Y_UP = 0x02,
    MPU9250_DMP_TAP_Y_DOWN = 0x03,
    MPU9250_DMP_TAP_Z_UP = 0x04,
    MPU9250_DMP_TAP_Z_DOWN = 0x05,
    MPU9250_DMP_ORIENT_QUAT = 0x10,
};

// IOCTL structs
struct mpu9250_reg {
    uint8_t reg;
    uint8_t val;
};

struct mpu9250_data {
    float accel[3];
    float gyro[3];
    float mag[3];
    float quat[4];
    uint8_t fifo[MPU9250_MAX_FIFO];
    int fifo_len;
    loff_t fifo_offset;
};

struct mpu9250_aio_data {
    float accel[3];
    float gyro[3];
    float mag[3];
    float quat[4];
};

struct mpu9250_mq_data {
    float accel[3];
    float gyro[3];
    float mag[3];
    float quat[4];
};

struct mpu9250_config {
    uint32_t sample_rate; // Hz
    uint8_t accel_range;  // 2, 4, 8, 16 (g)
    uint16_t gyro_range;  // 250, 500, 1000, 2000 (dps)
    bool enable_batch;    // Enable batch mode
    bool enable_iio;      // Enable IIO driver
};

// IIO channels
#ifdef CONFIG_MPU9250_IIO
#define MPU9250_CHAN(_type, _index) { \
    .type = IIO_##_type, \
    .indexed = 1, \
    .channel = _index, \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), \
    .scan_index = _index, \
    .scan_type = { .sign = 's', .realbits = 16, .storagebits = 16, .shift = 0, .endianness = IIO_BE } \
}
#else
#define MPU9250_CHAN(_type, _index) {}
#endif

// IOCTL commands
#define MPU9250_IOCTL_READ_REG       _IOWR('m', 1, struct mpu9250_reg)
#define MPU9250_IOCTL_WRITE_REG      _IOW('m', 2, struct mpu9250_reg)
#define MPU9250_IOCTL_SET_IRQ_TYPE   _IOW('m', 3, enum mpu9250_irq_type)
#define MPU9250_IOCTL_READ_ACCEL     _IOR('m', 4, struct mpu9250_data)
#define MPU9250_IOCTL_READ_GYRO      _IOR('m', 5, struct mpu9250_data)
#define MPU9250_IOCTL_READ_MAG       _IOR('m', 6, struct mpu9250_data)
#define MPU9250_IOCTL_READ_FIFO      _IOR('m', 7, struct mpu9250_data)
#define MPU9250_IOCTL_INIT_DMP       _IO('m', 8)
#define MPU9250_IOCTL_READ_DMP       _IOR('m', 9, struct mpu9250_data)
#define MPU9250_IOCTL_SET_NONBLOCK   _IOW('m', 10, int)
#define MPU9250_IOCTL_SET_CONFIG     _IOW('m', 11, struct mpu9250_config)
#define MPU9250_IOCTL_GET_CONFIG     _IOR('m', 12, struct mpu9250_config)
#define MPU9250_IOCTL_AIO_READ       _IOR('m', 13, struct mpu9250_aio_data)

// Tracepoints
DECLARE_TRACE(mpu9250_read,
    TP_PROTO(struct mpu9250_data *data, uint8_t reg, int len),
    TP_ARGS(data, reg, len));
DECLARE_TRACE(mpu9250_error,
    TP_PROTO(struct mpu9250_data *data, int error),
    TP_ARGS(data, error));

#endif /* _MPU9250_DRIVER_H_ */