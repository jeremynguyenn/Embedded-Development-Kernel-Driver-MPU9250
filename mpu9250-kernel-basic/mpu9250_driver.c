// mpu9250_driver.c
// Kernel module driver for MPU9250 on Raspberry Pi 4 with advanced features.
// Supports file ops (lseek, non-blocking), memory management, signals, and POSIX IPC.
// Author: Nguyen Nhan

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/signal.h>
#include <linux/vmalloc.h>

#include "mpu9250_driver.h"

// Module parameters
static unsigned int i2c_addr = MPU9250_DEFAULT_ADDR;
static unsigned int irq_type = MPU9250_INTERRUPT_DATA_READY;
static int debug = 0;
module_param(i2c_addr, uint, 0644);
module_param(irq_type, uint, 0644);
module_param(debug, int, 0644);

// Per-device data
struct mpu9250_data {
    struct i2c_client *client;
    struct i2c_client *mag_client;
    struct regmap *regmap;
    struct gpio_desc *irq_gpio;
    struct mutex lock;
    spinlock_t data_lock;
    struct work_struct read_work;
    wait_queue_head_t wq;
    bool data_ready;
    bool nonblock;
    uint8_t gs_flag;
    int16_t *accel_raw; // Dynamic allocation
    float *accel_g;
    int16_t *gyro_raw;
    float *gyro_dps;
    int16_t *mag_raw;
    float *mag_uT;
    uint8_t *fifo_buf;
    int fifo_len;
    loff_t fifo_offset;
    float *quat;
};

// Scaling factors
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f
#define MAG_SCALE   0.15f

// Regmap configuration
static const struct regmap_config mpu9250_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0xFF,
};

// Memory allocation
static int mpu9250_alloc_buffers(struct mpu9250_data *data)
{
    data->accel_raw = kmalloc_array(3, sizeof(int16_t), GFP_KERNEL);
    data->accel_g = kmalloc_array(3, sizeof(float), GFP_KERNEL);
    data->gyro_raw = kmalloc_array(3, sizeof(int16_t), GFP_KERNEL);
    data->gyro_dps = kmalloc_array(3, sizeof(float), GFP_KERNEL);
    data->mag_raw = kmalloc_array(3, sizeof(int16_t), GFP_KERNEL);
    data->mag_uT = kmalloc_array(3, sizeof(float), GFP_KERNEL);
    data->fifo_buf = vmalloc(MPU9250_MAX_FIFO);
    data->quat = kmalloc_array(4, sizeof(float), GFP_KERNEL);

    if (!data->accel_raw || !data->accel_g || !data->gyro_raw ||
        !data->gyro_dps || !data->mag_raw || !data->mag_uT ||
        !data->fifo_buf || !data->quat)
        return -ENOMEM;
    return 0;
}

static void mpu9250_free_buffers(struct mpu9250_data *data)
{
    kfree(data->accel_raw);
    kfree(data->accel_g);
    kfree(data->gyro_raw);
    kfree(data->gyro_dps);
    kfree(data->mag_raw);
    kfree(data->mag_uT);
    vfree(data->fifo_buf);
    kfree(data->quat);
}

// I2C read/write
static int mpu9250_read(struct mpu9250_data *data, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return regmap_bulk_read(data->regmap, reg, buf, len);
}

static int mpu9250_write(struct mpu9250_data *data, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return regmap_bulk_write(data->regmap, reg, buf, len);
}

// Data conversion
static void convert_accel(uint8_t *raw, float *g)
{
    int16_t x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t y = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t z = (int16_t)((raw[4] << 8) | raw[5]);
    g[0] = x / ACCEL_SCALE;
    g[1] = y / ACCEL_SCALE;
    g[2] = z / ACCEL_SCALE;
}

static void convert_gyro(uint8_t *raw, float *dps)
{
    int16_t x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t y = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t z = (int16_t)((raw[4] << 8) | raw[5]);
    dps[0] = x / GYRO_SCALE;
    dps[1] = y / GYRO_SCALE;
    dps[2] = z / GYRO_SCALE;
}

static void convert_mag(uint8_t *raw, float *uT)
{
    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);
    uT[0] = x * MAG_SCALE;
    uT[1] = y * MAG_SCALE;
    uT[2] = z * MAG_SCALE;
}

// Enable magnetometer bypass
static int enable_mag_bypass(struct mpu9250_data *data)
{
    uint8_t bypass = 0x02;
    return mpu9250_write(data, MPU9250_BYPASS_EN, &bypass, 1);
}

// FIFO init
static int mpu9250_fifo_init(struct mpu9250_data *data)
{
    uint8_t fifo_en = 0x78; // Accel, gyro, mag
    uint8_t sample_rate = 0x07; // 1kHz
    mpu9250_write(data, MPU9250_SMPLRT_DIV, &sample_rate, 1);
    mpu9250_write(data, MPU9250_FIFO_EN, &fifo_en, 1);
    return 0;
}

// DMP init (simplified)
static int mpu9250_dmp_init(struct mpu9250_data *data)
{
    uint8_t user_ctrl = 0x00;
    mpu9250_write(data, MPU9250_USER_CTRL, &user_ctrl, 1);
    msleep(10);
    user_ctrl = 0x10; // Enable DMP
    mpu9250_write(data, MPU9250_USER_CTRL, &user_ctrl, 1);
    uint8_t dmp_cfg1 = 0x01; // Quaternion
    mpu9250_write(data, MPU9250_DMP_CFG_1, &dmp_cfg1, 1);
    uint8_t dmp_cfg2 = 0x00;
    mpu9250_write(data, MPU9250_DMP_CFG_2, &dmp_cfg2, 1);
    uint8_t int_en = 0x02; // DMP interrupt
    mpu9250_write(data, MPU9250_INT_ENABLE, &int_en, 1);
    return 0;
}

// Read FIFO
static int mpu9250_read_fifo(struct mpu9250_data *data, uint8_t *buf, int *len)
{
    uint8_t count_h, count_l;
    mpu9250_read(data, MPU9250_FIFO_COUNTH, &count_h, 1);
    mpu9250_read(data, MPU9250_FIFO_COUNTL, &count_l, 1);
    *len = (count_h << 8) | count_l;
    if (*len > MPU9250_MAX_FIFO) *len = MPU9250_MAX_FIFO;
    return mpu9250_read(data, MPU9250_FIFO_R_W, buf, *len);
}

// Read DMP quaternion
static void mpu9250_read_dmp_quat(struct mpu9250_data *data, float *quat)
{
    uint8_t quat_raw[16];
    mpu9250_read(data, MPU9250_FIFO_R_W, quat_raw, 16);
    for (int i = 0; i < 4; i++) {
        int32_t q = (int32_t)((quat_raw[i*4] << 24) | (quat_raw[i*4+1] << 16) |
                              (quat_raw[i*4+2] << 8) | quat_raw[i*4+3]);
        quat[i] = q / (float)(1 << 30);
    }
}

// Workqueue handler
static void mpu9250_read_work(struct work_struct *work)
{
    struct mpu9250_data *data = container_of(work, struct mpu9250_data, read_work);
    unsigned long flags;
    uint8_t accel_raw[6], gyro_raw[6], mag_raw[7];

    mutex_lock(&data->lock);
    pm_runtime_get_sync(&data->client->dev);

    spin_lock_irqsave(&data->data_lock, flags);
    mpu9250_read(data, MPU9250_ACCEL_XOUT_H, accel_raw, 6);
    convert_accel(accel_raw, data->accel_g);
    mpu9250_read(data, MPU9250_GYRO_XOUT_H, gyro_raw, 6);
    convert_gyro(gyro_raw, data->gyro_dps);
    enable_mag_bypass(data);
    regmap_bulk_read(data->mag_client->dev.driver_data, 0x03, mag_raw, 7);
    if (!(mag_raw[6] & 0x08)) {
        convert_mag(mag_raw, data->mag_uT);
    }
    mpu9250_read_fifo(data, data->fifo_buf, &data->fifo_len);
    data->data_ready = true;
    spin_unlock_irqrestore(&data->data_lock, flags);

    wake_up_interruptible(&data->wq);
    pm_runtime_put(&data->client->dev);
    mutex_unlock(&data->lock);
}

// IRQ handler
static irqreturn_t mpu9250_irq_handler(int irq, void *dev_id)
{
    struct mpu9250_data *data = dev_id;
    unsigned long flags;

    spin_lock_irqsave(&data->data_lock, flags);
    switch (irq_type) {
        case MPU9250_INTERRUPT_MOTION:
            data->gs_flag |= 1 << 0;
            dev_dbg(&data->client->dev, "Motion IRQ\n");
            break;
        case MPU9250_INTERRUPT_FIFO_OVERFLOW:
            data->gs_flag |= 1 << 1;
            dev_dbg(&data->client->dev, "FIFO overflow IRQ\n");
            break;
        case MPU9250_INTERRUPT_FSYNC_INT:
            data->gs_flag |= 1 << 2;
            dev_dbg(&data->client->dev, "FSYNC IRQ\n");
            break;
        case MPU9250_INTERRUPT_DMP_INT:
            data->gs_flag |= 1 << 3;
            dev_dbg(&data->client->dev, "DMP IRQ\n");
            break;
        case MPU9250_INTERRUPT_DATA_READY:
            data->gs_flag |= 1 << 4;
            dev_dbg(&data->client->dev, "Data ready IRQ\n");
            break;
    }
    spin_unlock_irqrestore(&data->data_lock, flags);

    schedule_work(&data->read_work);
    return IRQ_HANDLED;
}

// Signal handler for SIGUSR1
static void mpu9250_signal_handler(struct kernel_siginfo *info, struct task_struct *task)
{
    struct mpu9250_data *data = task->private_data;
    dev_info(&data->client->dev, "SIGUSR1 received, dumping data: Accel X=%.2f g\n", data->accel_g[0]);
}

// File operations
static int mpu9250_open(struct inode *inode, struct file *file)
{
    struct mpu9250_data *data = container_of(inode->i_cdev, struct mpu9250_data, cdev);
    file->private_data = data;
    data->nonblock = file->f_flags & O_NONBLOCK;
    file->private_data = task_set_private(current, data);
    return 0;
}

static ssize_t mpu9250_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    struct mpu9250_data *data = file->private_data;
    unsigned long flags;
    uint8_t read_buf[14];
    ssize_t ret;

    if (data->nonblock && !data->data_ready)
        return -EAGAIN;

    wait_event_interruptible(data->wq, data->data_ready || data->nonblock);
    if (!data->data_ready)
        return -EAGAIN;

    mutex_lock(&data->lock);
    pm_runtime_get_sync(&data->client->dev);
    spin_lock_irqsave(&data->data_lock, flags);
    ret = mpu9250_read(data, MPU9250_ACCEL_XOUT_H, read_buf, 14);
    data->data_ready = false;
    spin_unlock_irqrestore(&data->data_lock, flags);
    pm_runtime_put(&data->client->dev);
    mutex_unlock(&data->lock);

    if (ret)
        return ret;
    if (copy_to_user(buf, read_buf, min_t(size_t, count, 14)))
        return -EFAULT;
    return min_t(size_t, count, 14);
}

static ssize_t mpu9250_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
    struct mpu9250_data *data = file->private_data;
    uint8_t write_buf[2];
    if (count != 2)
        return -EINVAL;
    if (copy_from_user(write_buf, buf, 2))
        return -EFAULT;
    pm_runtime_get_sync(&data->client->dev);
    mutex_lock(&data->lock);
    mpu9250_write(data, write_buf[0], &write_buf[1], 1);
    mutex_unlock(&data->lock);
    pm_runtime_put(&data->client->dev);
    return count;
}

static loff_t mpu9250_lseek(struct file *file, loff_t offset, int whence)
{
    struct mpu9250_data *data = file->private_data;
    unsigned long flags;
    loff_t new_offset;

    spin_lock_irqsave(&data->data_lock, flags);
    switch (whence) {
    case SEEK_SET:
        new_offset = offset;
        break;
    case SEEK_CUR:
        new_offset = data->fifo_offset + offset;
        break;
    case SEEK_END:
        new_offset = data->fifo_len + offset;
        break;
    default:
        spin_unlock_irqrestore(&data->data_lock, flags);
        return -EINVAL;
    }

    if (new_offset < 0 || new_offset > data->fifo_len) {
        spin_unlock_irqrestore(&data->data_lock, flags);
        return -EINVAL;
    }

    data->fifo_offset = new_offset;
    spin_unlock_irqrestore(&data->data_lock, flags);
    return new_offset;
}

static long mpu9250_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct mpu9250_data *data = file->private_data;
    int ret = 0;
    unsigned long flags;

    mutex_lock(&data->lock);
    pm_runtime_get_sync(&data->client->dev);

    switch (cmd) {
    case MPU9250_IOCTL_READ_REG: {
        struct mpu9250_reg reg;
        if (copy_from_user(&reg, (void __user *)arg, sizeof(reg))) {
            ret = -EFAULT;
            goto out;
        }
        ret = mpu9250_read(data, reg.reg, &reg.val, 1);
        if (!ret && copy_to_user((void __user *)arg, &reg, sizeof(reg)))
            ret = -EFAULT;
        break;
    }
    case MPU9250_IOCTL_WRITE_REG: {
        struct mpu9250_reg reg;
        if (copy_from_user(&reg, (void __user *)arg, sizeof(reg))) {
            ret = -EFAULT;
            goto out;
        }
        ret = mpu9250_write(data, reg.reg, &reg.val, 1);
        break;
    }
    case MPU9250_IOCTL_SET_IRQ_TYPE:
        if (copy_from_user(&irq_type, (void __user *)arg, sizeof(irq_type)))
            ret = -EFAULT;
        break;
    case MPU9250_IOCTL_SET_NONBLOCK:
        spin_lock_irqsave(&data->data_lock, flags);
        data->nonblock = arg;
        spin_unlock_irqrestore(&data->data_lock, flags);
        break;
    case MPU9250_IOCTL_READ_ACCEL: {
        struct mpu9250_data user_data;
        spin_lock_irqsave(&data->data_lock, flags);
        user_data.accel[0] = data->accel_g[0];
        user_data.accel[1] = data->accel_g[1];
        user_data.accel[2] = data->accel_g[2];
        spin_unlock_irqrestore(&data->data_lock, flags);
        if (copy_to_user((void __user *)arg, &user_data, sizeof(user_data)))
            ret = -EFAULT;
        break;
    }
    case MPU9250_IOCTL_READ_GYRO: {
        struct mpu9250_data user_data;
        spin_lock_irqsave(&data->data_lock, flags);
        user_data.gyro[0] = data->gyro_dps[0];
        user_data.gyro[1] = data->gyro_dps[1];
        user_data.gyro[2] = data->gyro_dps[2];
        spin_unlock_irqrestore(&data->data_lock, flags);
        if (copy_to_user((void __user *)arg, &user_data, sizeof(user_data)))
            ret = -EFAULT;
        break;
    }
    case MPU9250_IOCTL_READ_MAG: {
        struct mpu9250_data user_data;
        uint8_t mag_raw[7];
        ret = regmap_bulk_read(data->mag_client->dev.driver_data, 0x03, mag_raw, 7);
        if (!ret && !(mag_raw[6] & 0x08)) {
            spin_lock_irqsave(&data->data_lock, flags);
            convert_mag(mag_raw, user_data.mag);
            spin_unlock_irqrestore(&data->data_lock, flags);
            if (copy_to_user((void __user *)arg, &user_data, sizeof(user_data)))
                ret = -EFAULT;
        } else {
            ret = -EIO;
        }
        break;
    }
    case MPU9250_IOCTL_READ_FIFO: {
        struct mpu9250_data user_data;
        spin_lock_irqsave(&data->data_lock, flags);
        ret = mpu9250_read_fifo(data, user_data.fifo, &user_data.fifo_len);
        user_data.fifo_offset = data->fifo_offset;
        spin_unlock_irqrestore(&data->data_lock, flags);
        if (!ret && copy_to_user((void __user *)arg, &user_data, sizeof(user_data)))
            ret = -EFAULT;
        else if (!ret)
            ret = user_data.fifo_len;
        break;
    }
    case MPU9250_IOCTL_INIT_DMP:
        ret = mpu9250_dmp_init(data);
        break;
    case MPU9250_IOCTL_READ_DMP: {
        struct mpu9250_data user_data;
        spin_lock_irqsave(&data->data_lock, flags);
        mpu9250_read_dmp_quat(data, user_data.quat);
        spin_unlock_irqrestore(&data->data_lock, flags);
        if (copy_to_user((void __user *)arg, &user_data, sizeof(user_data)))
            ret = -EFAULT;
        break;
    }
    default:
        ret = -ENOTTY;
        break;
    }
out:
    pm_runtime_put(&data->client->dev);
    mutex_unlock(&data->lock);
    return ret;
}

static int mpu9250_release(struct inode *inode, struct file *file)
{
    return 0;
}

static const struct file_operations mpu9250_fops = {
    .owner = THIS_MODULE,
    .open = mpu9250_open,
    .read = mpu9250_read,
    .write = mpu9250_write,
    .llseek = mpu9250_lseek,
    .unlocked_ioctl = mpu9250_ioctl,
    .release = mpu9250_release,
};

// Sysfs for POSIX shared memory
static ssize_t mpu9250_data_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    unsigned long flags;
    ssize_t len;

    spin_lock_irqsave(&data->data_lock, flags);
    len = sprintf(buf, "Accel: %.2f %.2f %.2f\nGyro: %.2f %.2f %.2f\nMag: %.2f %.2f %.2f\n",
                  data->accel_g[0], data->accel_g[1], data->accel_g[2],
                  data->gyro_dps[0], data->gyro_dps[1], data->gyro_dps[2],
                  data->mag_uT[0], data->mag_uT[1], data->mag_uT[2]);
    spin_unlock_irqrestore(&data->data_lock, flags);
    return len;
}

static struct kobj_attribute mpu9250_data_attr = __ATTR(data, 0444, mpu9250_data_show, NULL);

static struct attribute *mpu9250_attrs[] = {
    &mpu9250_data_attr.attr,
    NULL,
};
static struct attribute_group mpu9250_attr_group = {
    .attrs = mpu9250_attrs,
};

// Runtime PM
static int mpu9250_suspend(struct device *dev)
{
    struct mpu9250_data *data = dev_get_drvdata(dev);
    disable_irq(gpiod_to_irq(data->irq_gpio));
    return 0;
}

static int mpu9250_resume(struct device *dev)
{
    struct mpu9250_data *data = dev_get_drvdata(dev);
    enable_irq(gpiod_to_irq(data->irq_gpio));
    schedule_work(&data->read_work);
    return 0;
}

static const struct dev_pm_ops mpu9250_pm_ops = {
    SET_RUNTIME_PM_OPS(mpu9250_suspend, mpu9250_resume, NULL)
};

// I2C probe
static int mpu9250_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct mpu9250_data *data;
    struct device *dev = &client->dev;
    int ret;

    dev_info(dev, "Probing MPU9250 at addr 0x%02x\n", client->addr);

    data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    ret = mpu9250_alloc_buffers(data);
    if (ret)
        return ret;

    data->client = client;
    i2c_set_clientdata(client, data);
    mutex_init(&data->lock);
    spin_lock_init(&data->data_lock);
    init_waitqueue_head(&data->wq);
    INIT_WORK(&data->read_work, mpu9250_read_work);

    // Regmap
    data->regmap = devm_regmap_init_i2c(client, &mpu9250_regmap_config);
    if (IS_ERR(data->regmap)) {
        ret = PTR_ERR(data->regmap);
        goto err_free;
    }

    // AK8963 client
    data->mag_client = i2c_new_dummy_device(client->adapter, 0x0C);
    if (IS_ERR(data->mag_client)) {
        ret = PTR_ERR(data->mag_client);
        goto err_free;
    }
    data->mag_client->dev.driver_data = devm_regmap_init_i2c(data->mag_client, &mpu9250_regmap_config);

    // GPIO IRQ
    data->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
    if (IS_ERR(data->irq_gpio)) {
        ret = PTR_ERR(data->irq_gpio);
        goto err_mag;
    }
    ret = devm_request_threaded_irq(dev, gpiod_to_irq(data->irq_gpio), NULL,
                                    mpu9250_irq_handler, IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
                                    "mpu9250_irq", data);
    if (ret)
        goto err_mag;

    // Char device
    ret = alloc_chrdev_region(&data->dev_t, 0, 1, MPU9250_DEVICE_NAME);
    if (ret)
        goto err_mag;
    cdev_init(&data->cdev, &mpu9250_fops);
    data->cdev.owner = THIS_MODULE;
    ret = cdev_add(&data->cdev, data->dev_t, 1);
    if (ret)
        goto err_chrdev;

    // Device class
    data->class = class_create(THIS_MODULE, MPU9250_CLASS_NAME);
    if (IS_ERR(data->class)) {
        ret = PTR_ERR(data->class);
        goto err_cdev;
    }
    data->device = device_create(data->class, NULL, data->dev_t, NULL, MPU9250_DEVICE_NAME);
    if (IS_ERR(data->device)) {
        ret = PTR_ERR(data->device);
        goto err_class;
    }

    // Sysfs
    ret = sysfs_create_group(&data->device->kobj, &mpu9250_attr_group);
    if (ret)
        goto err_device;

    // Enable runtime PM
    pm_runtime_enable(dev);
    pm_runtime_set_active(dev);

    // Sensor init
    uint8_t wakeup = 0x00;
    mpu9250_write(data, MPU9250_PWR_MGMT_1, &wakeup, 1);
    msleep(100);
    uint8_t accel_cfg = 0x00; // +/-2g
    mpu9250_write(data, MPU9250_ACCEL_CONFIG, &accel_cfg, 1);
    uint8_t gyro_cfg = 0x00; // +/-250 dps
    mpu9250_write(data, MPU9250_GYRO_CONFIG, &gyro_cfg, 1);
    enable_mag_bypass(data);
    mpu9250_fifo_init(data);
    mpu9250_dmp_init(data);

    dev_info(dev, "MPU9250 probe successful with advanced features\n");
    return 0;

err_device:
    device_destroy(data->class, data->dev_t);
err_class:
    class_destroy(data->class);
err_cdev:
    cdev_del(&data->cdev);
err_chrdev:
    unregister_chrdev_region(data->dev_t, 1);
err_mag:
    i2c_unregister_device(data->mag_client);
err_free:
    mpu9250_free_buffers(data);
    return ret;
}

static int mpu9250_remove(struct i2c_client *client)
{
    struct mpu9250_data *data = i2c_get_clientdata(client);
    pm_runtime_disable(&client->dev);
    cancel_work_sync(&data->read_work);
    sysfs_remove_group(&data->device->kobj, &mpu9250_attr_group);
    device_destroy(data->class, data->dev_t);
    class_destroy(data->class);
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->dev_t, 1);
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

MODULE_AUTHOR("NguyenNhan updated for Pi 4");
MODULE_DESCRIPTION("MPU9250 Kernel Driver with advanced file ops, memory, and IPC");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.1");