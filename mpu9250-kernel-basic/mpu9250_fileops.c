// mpu9250_fileops.c
// File operations for MPU9250 kernel driver on Raspberry Pi 4.
// Supports blocking/non-blocking I/O, atomic operations, and lseek.
// Author: Nguyen Nhan

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h> // New: For poll support
#include <linux/mm.h> // New: For mmap
#include "mpu9250.h"

// Workqueue handler
void mpu9250_read_work(struct work_struct *work)
{
    struct mpu9250_data *data = container_of(work, struct mpu9250_data, read_work);
    unsigned long flags;
    uint8_t accel_raw[6], gyro_raw[6], mag_raw[7];

    // Race condition note: mutex_lock prevents concurrent access to I2C bus from multiple workqueues or threads.
    mutex_lock(&data->lock);
    pm_runtime_get_sync(&data->client->dev);

    // Race condition note: spin_lock protects shared data (accel_g, data_ready) from IRQ concurrent modifications.
    spin_lock_irqsave(&data->data_lock, flags);
    if (mpu9250_read(data, MPU9250_ACCEL_XOUT_H, accel_raw, 6) == 0)
        convert_accel(accel_raw, data->accel_g, data->accel_scale);
    if (mpu9250_read(data, MPU9250_GYRO_XOUT_H, gyro_raw, 6) == 0)
        convert_gyro(gyro_raw, data->gyro_dps, data->gyro_scale);
    if (enable_mag_bypass(data) == 0 && regmap_bulk_read(data->mag_client->dev.driver_data, 0x03, mag_raw, 7) == 0) {
        if (!(mag_raw[6] & 0x08))
            convert_mag(mag_raw, data->mag_uT);
    }
    mpu9250_read_fifo(data, data->fifo_buf, &data->fifo_len);
    mpu9250_read_dmp_quat(data, data->quat);
    data->data_ready = true;
    spin_unlock_irqrestore(&data->data_lock, flags);

    wake_up_interruptible(&data->wq);
    pm_runtime_put(&data->client->dev);
    mutex_unlock(&data->lock);
}

// IRQ handler
irqreturn_t mpu9250_irq_handler(int irq, void *dev_id)
{
    struct mpu9250_data *data = dev_id;
    unsigned long flags;

    // Race condition note: spin_lock ensures gs_flag update is atomic, preventing race with read_work.
    spin_lock_irqsave(&data->data_lock, flags);
    data->gs_flag |= 1 << MPU9250_INTERRUPT_DATA_READY;
    dev_dbg(&data->client->dev, "IRQ triggered\n");
    schedule_work(&data->read_work);
    spin_unlock_irqrestore(&data->data_lock, flags);
    return IRQ_HANDLED;
}

static int mpu9250_open(struct inode *inode, struct file *file)
{
    struct mpu9250_data *data = container_of(inode->i_cdev, struct mpu9250_data, cdev);
    file->private_data = data;
    atomic_inc(&data->client->dev.power.usage_count);
    return 0;
}

static ssize_t mpu9250_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    struct mpu9250_data *data = file->private_data;
    struct mpu9250_sensor_data sensor_data = {0};
    unsigned long flags;
    int ret;

    if (count < sizeof(sensor_data))
        return -EINVAL;

    if (file->f_flags & O_NONBLOCK) {
        spin_lock_irqsave(&data->data_lock, flags);
        if (!data->data_ready) {
            spin_unlock_irqrestore(&data->data_lock, flags);
            return -EAGAIN;
        }
        // Copy data (assume sensor_data filled from shared)
        memcpy(&sensor_data, data->accel_g, sizeof(float)*3); // Example for accel
        data->data_ready = false;
        spin_unlock_irqrestore(&data->data_lock, flags);
    } else {
        ret = wait_event_interruptible(data->wq, data->data_ready);
        if (ret) return ret;
        spin_lock_irqsave(&data->data_lock, flags);
        memcpy(&sensor_data, data->accel_g, sizeof(float)*3); // Example
        data->data_ready = false;
        spin_unlock_irqrestore(&data->data_lock, flags);
    }

    if (copy_to_user(buf, &sensor_data, sizeof(sensor_data)))
        return -EFAULT;
    return sizeof(sensor_data);
}

static ssize_t mpu9250_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
    // Implement full write for reg writes or config
    struct mpu9250_data *data = file->private_data;
    uint8_t write_buf[256];
    if (count > 256) return -EINVAL;
    if (copy_from_user(write_buf, buf, count)) return -EFAULT;
    // Assume first byte reg, rest data
    return mpu9250_write(data, write_buf[0], &write_buf[1], count - 1);
}

static loff_t mpu9250_lseek(struct file *file, loff_t off, int whence)
{
    // Full lseek for FIFO or data buffer (demo seekable)
    struct mpu9250_data *data = file->private_data;
    loff_t newpos;
    switch (whence) {
        case SEEK_SET: newpos = off; break;
        case SEEK_CUR: newpos = file->f_pos + off; break;
        case SEEK_END: newpos = data->fifo_len + off; break;
        default: return -EINVAL;
    }
    if (newpos < 0) return -EINVAL;
    file->f_pos = newpos;
    return newpos;
}

static long mpu9250_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct mpu9250_data *data = file->private_data;
    switch (cmd) {
        case MPU9250_IOCTL_READ_REG: {
            struct mpu9250_reg reg;
            if (copy_from_user(&reg, (void __user *)arg, sizeof(reg))) return -EFAULT;
            if (mpu9250_read(data, reg.reg, &reg.val, 1)) return -EIO;
            if (copy_to_user((void __user *)arg, &reg, sizeof(reg))) return -EFAULT;
            break;
        }
        case MPU9250_IOCTL_WRITE_REG: {
            struct mpu9250_reg reg;
            if (copy_from_user(&reg, (void __user *)arg, sizeof(reg))) return -EFAULT;
            if (mpu9250_write(data, reg.reg, &reg.val, 1)) return -EIO;
            break;
        }
        case MPU9250_IOCTL_READ_ACCEL: {
            struct mpu9250_sensor_data sensor;
            uint8_t raw[6];
            if (mpu9250_read(data, MPU9250_ACCEL_XOUT_H, raw, 6)) return -EIO;
            convert_accel(raw, sensor.values, data->accel_scale);
            sensor.values[0] -= data->accel_offset[0];
            sensor.values[1] -= data->accel_offset[1];
            sensor.values[2] -= data->accel_offset[2];
            if (copy_to_user((void __user *)arg, &sensor, sizeof(sensor))) return -EFAULT;
            break;
        }
        case MPU9250_IOCTL_READ_GYRO: {
            struct mpu9250_sensor_data sensor;
            uint8_t raw[6];
            if (mpu9250_read(data, MPU9250_GYRO_XOUT_H, raw, 6)) return -EIO;
            convert_gyro(raw, sensor.values, data->gyro_scale);
            sensor.values[0] -= data->gyro_offset[0];
            sensor.values[1] -= data->gyro_offset[1];
            sensor.values[2] -= data->gyro_offset[2];
            if (copy_to_user((void __user *)arg, &sensor, sizeof(sensor))) return -EFAULT;
            break;
        }
        case MPU9250_IOCTL_READ_MAG: {
            struct mpu9250_sensor_data sensor;
            uint8_t raw[7];
            if (enable_mag_bypass(data)) return -EIO;
            if (regmap_bulk_read(data->mag_client->dev.driver_data, MPU9250_MAG_XOUT_L, raw, 7)) return -EIO;
            if (raw[6] & 0x08) return -EAGAIN; // Overflow
            convert_mag(raw, sensor.values);
            if (copy_to_user((void __user *)arg, &sensor, sizeof(sensor))) return -EFAULT;
            break;
        }
        case MPU9250_IOCTL_READ_FIFO: {
            struct mpu9250_sensor_data sensor = {0};
            int len;
            if (mpu9250_read_fifo(data, data->fifo_buf, &len)) return -EIO;
            // Process FIFO data (demo: copy length)
            sensor.values[0] = len;
            if (copy_to_user((void __user *)arg, &sensor, sizeof(sensor))) return -EFAULT;
            break;
        }
        case MPU9250_IOCTL_RESET: {
            return mpu9250_i2c_init(data); // Reset via init
        }
        case MPU9250_IOCTL_INIT_DMP: {
            return mpu9250_dmp_init(data);
        }
        case MPU9250_IOCTL_READ_DMP: {
            struct mpu9250_sensor_data sensor;
            mpu9250_read_dmp_quat(data, sensor.values);
            if (copy_to_user((void __user *)arg, &sensor, sizeof(sensor))) return -EFAULT;
            break;
        }
        case MPU9250_IOCTL_SET_NONBLOCK: {
            file->f_flags = (arg) ? (file->f_flags | O_NONBLOCK) : (file->f_flags & ~O_NONBLOCK);
            break;
        }
        case MPU9250_IOCTL_SET_ACCEL_SCALE: {
            return mpu9250_set_accel_scale(data, arg);
        }
        case MPU9250_IOCTL_SET_GYRO_SCALE: {
            return mpu9250_set_gyro_scale(data, arg);
        }
        case MPU9250_IOCTL_SET_SAMPLE_RATE: {
            return mpu9250_set_sample_rate(data, arg);
        }
        case MPU9250_IOCTL_CALIBRATE: {
            return mpu9250_calibrate(data);
        }
        default:
            return -EINVAL;
    }
    return 0;
}

static int mpu9250_release(struct inode *inode, struct file *file)
{
    struct mpu9250_data *data = file->private_data;
    atomic_dec(&data->client->dev.power.usage_count);
    return 0;
}

// New: Poll support
static unsigned int mpu9250_poll(struct file *file, struct poll_table_struct *wait)
{
    struct mpu9250_data *data = file->private_data;
    unsigned int mask = 0;
    poll_wait(file, &data->wq, wait);
    if (data->data_ready) mask |= POLLIN | POLLRDNORM;
    return mask;
}

// New: Mmap support for shared buffer (e.g., fifo_buf)
static int mpu9250_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct mpu9250_data *data = file->private_data;
    unsigned long size = vma->vm_end - vma->vm_start;
    if (size > MPU9250_MAX_FIFO) return -EINVAL;
    return remap_pfn_range(vma, vma->vm_start, virt_to_pfn(data->fifo_buf), size, vma->vm_page_prot);
}

const struct file_operations mpu9250_fops = {
    .owner = THIS_MODULE,
    .open = mpu9250_open,
    .read = mpu9250_read,
    .write = mpu9250_write,
    .llseek = mpu9250_lseek,
    .unlocked_ioctl = mpu9250_ioctl,
    .release = mpu9250_release,
    .poll = mpu9250_poll, // New
    .mmap = mpu9250_mmap, // New
};

int mpu9250_fileops_init(struct mpu9250_data *data)
{
    int ret;

    ret = alloc_chrdev_region(&data->dev_t, 0, 1, MPU9250_DEVICE_NAME);
    if (ret) {
        dev_err(&data->client->dev, "Failed to allocate chrdev: %d\n", ret);
        return ret;
    }
    cdev_init(&data->cdev, &mpu9250_fops);
    data->cdev.owner = THIS_MODULE;
    ret = cdev_add(&data->cdev, data->dev_t, 1);
    if (ret) {
        dev_err(&data->client->dev, "Failed to add cdev: %d\n", ret);
        goto err_chrdev;
    }

    data->class = class_create(THIS_MODULE, MPU9250_CLASS_NAME);
    if (IS_ERR(data->class)) {
        ret = PTR_ERR(data->class);
        dev_err(&data->client->dev, "Failed to create class: %d\n", ret);
        goto err_cdev;
    }
    data->device = device_create(data->class, NULL, data->dev_t, NULL, MPU9250_DEVICE_NAME);
    if (IS_ERR(data->device)) {
        ret = PTR_ERR(data->device);
        dev_err(&data->client->dev, "Failed to create device: %d\n", ret);
        goto err_class;
    }
    return 0;

err_class:
    class_destroy(data->class);
err_cdev:
    cdev_del(&data->cdev);
err_chrdev:
    unregister_chrdev_region(data->dev_t, 1);
    return ret;
}

void mpu9250_fileops_cleanup(struct mpu9250_data *data)
{
    device_destroy(data->class, data->dev_t);
    class_destroy(data->class);
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->dev_t, 1);
}

int enable_mag_bypass(struct mpu9250_data *data)
{
    uint8_t buf = 0x02; // Enable bypass mode
    int ret = mpu9250_write(data, MPU9250_INT_PIN_CFG, &buf, 1);
    if (ret) {
        dev_err(&data->client->dev, "Mag bypass failed, retrying...\n"); // Improved error handling
        msleep(10);
        ret = mpu9250_write(data, MPU9250_INT_PIN_CFG, &buf, 1);
    }
    return ret;
}