// mpu9250_driver.c
// Core kernel module driver for MPU9250 on Raspberry Pi 4.
// Handles initialization, probe, and removal with power management and sysfs.
// Author: Nguyen Nhan

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/of_device.h>
#include <linux/kthread.h> // New: For kernel thread demo
#include <linux/vmalloc.h> // New: For vmalloc
#include <linux/input.h> // New: For evdev
#include "mpu9250.h"

// Module parameters
static unsigned int i2c_addr = MPU9250_DEFAULT_ADDR;
static unsigned int irq_type = MPU9250_INTERRUPT_DATA_READY;
static int debug = 0;
module_param(i2c_addr, uint, 0644);
module_param(irq_type, uint, 0644);
module_param(debug, int, 0644);

// Sysfs attributes
static ssize_t mpu9250_data_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    unsigned long flags;
    ssize_t len;

    // Race condition note: spin_lock prevents concurrent access to shared data (accel_g, etc.) from IRQ and sysfs.
    spin_lock_irqsave(&data->data_lock, flags);
    len = sprintf(buf, "Accel: %.2f %.2f %.2f\nGyro: %.2f %.2f %.2f\nMag: %.2f %.2f %.2f\nQuat: %.2f %.2f %.2f %.2f\n",
                  data->accel_g[0], data->accel_g[1], data->accel_g[2],
                  data->gyro_dps[0], data->gyro_dps[1], data->gyro_dps[2],
                  data->mag_uT[0], data->mag_uT[1], data->mag_uT[2],
                  data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
    spin_unlock_irqrestore(&data->data_lock, flags);
    return len;
}

static struct kobj_attribute dev_attr_mpu9250_data = __ATTR(mpu9250_data, 0444, mpu9250_data_show, NULL);

static ssize_t accel_scale_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    return sprintf(buf, "%d\n", data->accel_scale);
}

static ssize_t accel_scale_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    int scale;
    if (kstrtoint(buf, 10, &scale))
        return -EINVAL;
    if (mpu9250_set_accel_scale(data, scale))
        return -EINVAL;
    return count;
}

static struct kobj_attribute dev_attr_accel_scale = __ATTR(accel_scale, 0664, accel_scale_show, accel_scale_store);

static ssize_t gyro_scale_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    return sprintf(buf, "%d\n", data->gyro_scale);
}

static ssize_t gyro_scale_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    int scale;
    if (kstrtoint(buf, 10, &scale))
        return -EINVAL;
    if (mpu9250_set_gyro_scale(data, scale))
        return -EINVAL;
    return count;
}

static struct kobj_attribute dev_attr_gyro_scale = __ATTR(gyro_scale, 0664, gyro_scale_show, gyro_scale_store);

static struct attribute *mpu9250_attrs[] = {
    &dev_attr_mpu9250_data.attr,
    &dev_attr_accel_scale.attr,
    &dev_attr_gyro_scale.attr,
    NULL
};

static const struct attribute_group mpu9250_attr_group = {
    .attrs = mpu9250_attrs,
};

// Kernel thread function
static int kthread_func(void *arg)
{
    struct mpu9250_data *data = arg;
    while (!kthread_should_stop()) {
        // Demo process management: Log data periodically
        pr_info("MPU9250 kthread: Accel X: %f\n", data->accel_g[0]);
        msleep(1000);
    }
    return 0;
}

// PM ops
static int mpu9250_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mpu9250_data *data = i2c_get_clientdata(client);
    uint8_t buf;
    // Read current PWR_MGMT_1
    if (mpu9250_read(data, MPU9250_PWR_MGMT_1, &buf, 1)) return -EIO;
    buf |= 0x40; // Set sleep bit
    return mpu9250_write(data, MPU9250_PWR_MGMT_1, &buf, 1);
}

static int mpu9250_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mpu9250_data *data = i2c_get_clientdata(client);
    uint8_t buf;
    // Read current PWR_MGMT_1
    if (mpu9250_read(data, MPU9250_PWR_MGMT_1, &buf, 1)) return -EIO;
    buf &= ~0x40; // Clear sleep bit
    return mpu9250_write(data, MPU9250_PWR_MGMT_1, &buf, 1);
}

static const struct dev_pm_ops mpu9250_pm_ops = {
    .suspend = mpu9250_suspend,
    .resume = mpu9250_resume,
};

// Alloc/free buffers with vmalloc for large
int mpu9250_alloc_buffers(struct mpu9250_data *data)
{
    data->fifo_buf = vmalloc(MPU9250_MAX_FIFO); // New: Use vmalloc
    if (!data->fifo_buf) return -ENOMEM;
    return 0;
}

void mpu9250_free_buffers(struct mpu9250_data *data)
{
    vfree(data->fifo_buf); // New
}

static int mpu9250_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct mpu9250_data *data;
    struct task_struct *kthread = NULL;
    int ret;

    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data) return -ENOMEM;

    i2c_set_clientdata(client, data);
    data->client = client;
    data->regmap = devm_regmap_init_i2c(client, &mpu9250_regmap_config);
    if (IS_ERR(data->regmap)) return PTR_ERR(data->regmap);

    mutex_init(&data->lock);
    spin_lock_init(&data->data_lock);
    INIT_WORK(&data->read_work, mpu9250_read_work);
    init_waitqueue_head(&data->wq);

    ret = mpu9250_alloc_buffers(data);
    if (ret) return ret;

    ret = mpu9250_i2c_init(data);
    if (ret) goto err_free;

    data->mag_client = i2c_new_ancillary_device(client, "ak8963", 0x0C);
    if (!data->mag_client) {
        ret = -ENODEV;
        goto err_free;
    }

    ret = mpu9250_fileops_init(data);
    if (ret) goto err_mag;

    ret = sysfs_create_group(&data->device->kobj, &mpu9250_attr_group);
    if (ret) goto err_device;

    pm_runtime_enable(&client->dev);
    ret = devm_request_threaded_irq(&client->dev, client->irq, mpu9250_irq_handler, NULL,
                                   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "mpu9250", data);
    if (ret) goto err_sysfs;

    // New: Start kernel thread for Process Management
    kthread = kthread_create(kthread_func, data, "mpu9250_kthread");
    if (IS_ERR(kthread)) {
        dev_err(&client->dev, "Failed to create kernel thread\n");
        goto err_sysfs;
    }
    wake_up_process(kthread);

    // New: Calibration in probe
    mpu9250_calibrate(data);

    // New: Input subsystem integration (evdev)
    data->input_dev = devm_input_allocate_device(&client->dev);
    if (!data->input_dev) {
        ret = -ENOMEM;
        goto err_sysfs;
    }
    data->input_dev->name = "MPU9250 IMU";
    input_set_abs_params(data->input_dev, ABS_X, -32768, 32767, 0, 0); // Example for accel X
    input_set_abs_params(data->input_dev, ABS_Y, -32768, 32767, 0, 0);
    input_set_abs_params(data->input_dev, ABS_Z, -32768, 32767, 0, 0);
    input_set_abs_params(data->input_dev, ABS_RX, -32768, 32767, 0, 0); // Gyro X
    input_set_abs_params(data->input_dev, ABS_RY, -32768, 32767, 0, 0);
    input_set_abs_params(data->input_dev, ABS_RZ, -32768, 32767, 0, 0);
    // ... Add for mag/quat if needed
    ret = input_register_device(data->input_dev);
    if (ret) goto err_sysfs;

    // Note on Memory Layout: In probe, devm_kzalloc allocates from heap. Kernel layout: text (code), data (statics), stack (local), heap (dynamic).

    dev_info(&client->dev, "MPU9250 probed successfully\n");
    return 0;

err_sysfs:
    sysfs_remove_group(&data->device->kobj, &mpu9250_attr_group);
err_device:
    mpu9250_fileops_cleanup(data);
err_mag:
    i2c_unregister_device(data->mag_client);
err_free:
    mpu9250_free_buffers(data);
    return ret;
}

static int mpu9250_remove(struct i2c_client *client)
{
    struct mpu9250_data *data = i2c_get_clientdata(client);
    // New: Stop kernel thread
    if (kthread) kthread_stop(kthread);
    // New: Unregister input
    input_unregister_device(data->input_dev);
    pm_runtime_disable(&client->dev);
    cancel_work_sync(&data->read_work);
    sysfs_remove_group(&data->device->kobj, &mpu9250_attr_group);
    mpu9250_fileops_cleanup(data);
    i2c_unregister_device(data->mag_client);
    mpu9250_free_buffers(data);
    dev_info(&client->dev, "MPU9250 removed\n");
    return 0;
}

static const struct i2c_device_id mpu9250_id[] = {
    { "mpu9250", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu9250_id);

static const struct of_device_id mpu9250_of_match[] = {
    { .compatible = "invensense,mpu9250" },
    { }
};
MODULE_DEVICE_TABLE(of, mpu9250_of_match);

static struct i2c_driver mpu9250_driver = {
    .driver = {
        .name = "mpu9250",
        .owner = THIS_MODULE,
        .of_match_table = mpu9250_of_match,
        .pm = &mpu9250_pm_ops,
    },
    .probe = mpu9250_probe,
    .remove = mpu9250_remove,
    .id_table = mpu9250_id,
};

static int __init mpu9250_init(void)
{
    return i2c_add_driver(&mpu9250_driver);
}

static void __exit mpu9250_exit(void)
{
    i2c_del_driver(&mpu9250_driver);
}

module_init(mpu9250_init);
module_exit(mpu9250_exit);

MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("MPU9250 Kernel Driver with modular design and advanced features");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.2");