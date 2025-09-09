// mpu9250_iio.c
// IIO driver for MPU9250 on Raspberry Pi 4, supporting buffered I/O and sysfs interface.
// Integrated with mpu9250_driver for shared functionality.
// Author: Nguyen Nhan

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/tracepoint.h>
#include "mpu9250_driver.h"

struct mpu9250_iio_data {
    struct i2c_client *client;
    struct regmap *regmap;
    struct semaphore sem;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    unsigned int error_count;
};

// Scaling factors
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f
#define MAG_SCALE   0.15f

// IIO channels
static struct iio_chan_spec mpu9250_iio_channels[] = {
    MPU9250_CHAN(ACCEL, 0), // X
    MPU9250_CHAN(ACCEL, 1), // Y
    MPU9250_CHAN(ACCEL, 2), // Z
    MPU9250_CHAN(ANG_VEL, 0), // X
    MPU9250_CHAN(ANG_VEL, 1), // Y
    MPU9250_CHAN(ANG_VEL, 2), // Z
    MPU9250_CHAN(MAGN, 0), // X
    MPU9250_CHAN(MAGN, 1), // Y
    MPU9250_CHAN(MAGN, 2), // Z
};

// IIO read raw
static int mpu9250_iio_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
                                int *val, int *val2, long mask)
{
    struct mpu9250_iio_data *data = iio_priv(indio_dev);
    int ret;

    down(&data->sem);
    switch (mask) {
    case IIO_CHAN_INFO_RAW:
        if (chan->type == IIO_ACCEL)
            *val = data->accel_raw[chan->channel];
        else if (chan->type == IIO_ANG_VEL)
            *val = data->gyro_raw[chan->channel];
        else if (chan->type == IIO_MAGN)
            *val = data->mag_raw[chan->channel];
        else
            ret = -EINVAL;
        break;
    case IIO_CHAN_INFO_SCALE:
        if (chan->type == IIO_ACCEL)
            *val2 = ACCEL_SCALE;
        else if (chan->type == IIO_ANG_VEL)
            *val2 = GYRO_SCALE;
        else if (chan->type == IIO_MAGN)
            *val2 = MAG_SCALE;
        else
            ret = -EINVAL;
        *val = 0;
        break;
    default:
        ret = -EINVAL;
    }
    up(&data->sem);
    return ret ? ret : IIO_VAL_INT;
}

// IIO read data
static void mpu9250_iio_read_data(struct mpu9250_iio_data *data)
{
    uint8_t accel_raw[6], gyro_raw[6], mag_raw[7];
    int ret, retries = 3;

    down(&data->sem);
    while (retries--) {
        ret = regmap_bulk_read(data->regmap, MPU9250_ACCEL_XOUT_H, accel_raw, 6);
        if (!ret) break;
        msleep(10);
    }
    if (ret) {
        data->error_count++;
        trace_mpu9250_error((struct mpu9250_data *)data, ret);
        goto out;
    }
    for (int i = 0; i < 3; i++)
        data->accel_raw[i] = (int16_t)((accel_raw[i*2] << 8) | accel_raw[i*2+1]);

    retries = 3;
    while (retries--) {
        ret = regmap_bulk_read(data->regmap, MPU9250_GYRO_XOUT_H, gyro_raw, 6);
        if (!ret) break;
        msleep(10);
    }
    if (ret) {
        data->error_count++;
        trace_mpu9250_error((struct mpu9250_data *)data, ret);
        goto out;
    }
    for (int i = 0; i < 3; i++)
        data->gyro_raw[i] = (int16_t)((gyro_raw[i*2] << 8) | gyro_raw[i*2+1]);

    retries = 3;
    while (retries--) {
        ret = regmap_bulk_read(data->regmap, MPU9250_MAG_XOUT_L, mag_raw, 7);
        if (!ret && !(mag_raw[6] & 0x08)) break;
        msleep(10);
    }
    if (!ret) {
        for (int i = 0; i < 3; i++)
            data->mag_raw[i] = (int16_t)((mag_raw[i*2+1] << 8) | mag_raw[i*2]);
    } else {
        data->error_count++;
        trace_mpu9250_error((struct mpu9250_data *)data, ret);
    }

    iio_push_to_buffers(data->iio_dev, &data->accel_raw);
    iio_push_to_buffers(data->iio_dev, &data->gyro_raw);
    iio_push_to_buffers(data->iio_dev, &data->mag_raw);
out:
    up(&data->sem);
}

// IIO trigger handler
static void mpu9250_iio_trigger_work(struct iio_dev *indio_dev)
{
    struct mpu9250_iio_data *data = iio_priv(indio_dev);
    mpu9250_iio_read_data(data);
}

// I2C probe
static int mpu9250_iio_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct mpu9250_iio_data *data;
    struct iio_dev *indio_dev;
    struct device *dev = &client->dev;
    int ret;

    indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;

    data = iio_priv(indio_dev);
    data->client = client;
    data->iio_dev = indio_dev;
    i2c_set_clientdata(client, data);
    sema_init(&data->sem, 1);

    data->regmap = devm_regmap_init_i2c(client, &mpu9250_regmap_config);
    if (IS_ERR(data->regmap))
        return PTR_ERR(data->regmap);

    indio_dev->name = MPU9250_DEVICE_NAME;
    indio_dev->info = &mpu9250_iio_info;
    indio_dev->channels = mpu9250_iio_channels;
    indio_dev->num_channels = ARRAY_SIZE(mpu9250_iio_channels);

    ret = devm_iio_triggered_buffer_setup(dev, indio_dev, NULL, mpu9250_iio_trigger_work, NULL);
    if (ret)
        return ret;

    data->trig = devm_iio_trigger_alloc(dev, "%s-dev%d", MPU9250_DEVICE_NAME, iio_device_id(indio_dev));
    if (!data->trig)
        return -ENOMEM;

    ret = devm_iio_device_register(dev, indio_dev);
    if (ret)
        return ret;

    dev_info(dev, "MPU9250 IIO probe successful\n");
    return 0;
}

static int mpu9250_iio_remove(struct i2c_client *client)
{
    dev_info(&client->dev, "MPU9250 IIO removed\n");
    return 0;
}

static const struct i2c_device_id mpu9250_iio_id[] = {
    { "mpu9250", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu9250_iio_id);

static const struct of_device_id mpu9250_iio_of_match[] = {
    { .compatible = "invensense,mpu9250" },
    { }
};
MODULE_DEVICE_TABLE(of, mpu9250_iio_of_match);

static struct i2c_driver mpu9250_iio_driver = {
    .driver = {
        .name = "mpu9250_iio",
        .owner = THIS_MODULE,
        .of_match_table = mpu9250_iio_of_match,
    },
    .probe = mpu9250_iio_probe,
    .remove = mpu9250_iio_remove,
    .id_table = mpu9250_iio_id,
};

static int __init mpu9250_iio_init(void)
{
    return i2c_add_driver(&mpu9250_iio_driver);
}

static void __exit mpu9250_iio_exit(void)
{
    i2c_del_driver(&mpu9250_iio_driver);
}

module_init(mpu9250_iio_init);
module_exit(mpu9250_iio_exit);

MODULE_AUTHOR("Nguyen Nhan updated for Pi 4");
MODULE_DESCRIPTION("MPU9250 IIO Driver for Raspberry Pi 4");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.2");