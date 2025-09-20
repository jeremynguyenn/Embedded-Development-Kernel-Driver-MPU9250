// mpu9250_ops.c
// Sensor operations for MPU9250 kernel driver on Raspberry Pi 4.
// Handles I2C communication, FIFO, DMP operations, and scale configuration.
// Author: Nguyen Nhan

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include "mpu9250.h"

// Regmap configuration
const struct regmap_config mpu9250_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0xFF,
};

// I2C read/write with enhanced retry and timeout
int mpu9250_read(struct mpu9250_data *data, uint8_t reg, uint8_t *buf, uint16_t len)
{
    int ret = regmap_bulk_read(data->regmap, reg, buf, len);
    if (ret) {
        dev_err(&data->client->dev, "I2C read failed at reg 0x%02x, len %u: %d\n", reg, len, ret);
        // Improved error handling: Add timeout check (simulate with delay for demo)
        msleep(10); // Wait and retry once
        ret = regmap_bulk_read(data->regmap, reg, buf, len);
        if (ret) dev_err(&data->client->dev, "I2C read retry failed\n");
    }
    return ret;
}

int mpu9250_write(struct mpu9250_data *data, uint8_t reg, uint8_t *buf, uint16_t len)
{
    int ret = regmap_bulk_write(data->regmap, reg, buf, len);
    if (ret) {
        dev_err(&data->client->dev, "I2C write failed at reg 0x%02x, len %u: %d\n", reg, len, ret);
        // Improved error handling: Retry on write failure
        msleep(10);
        ret = regmap_bulk_write(data->regmap, reg, buf, len);
        if (ret) dev_err(&data->client->dev, "I2C write retry failed\n");
    }
    return ret;
}

// Data conversion functions (full implementations)
void convert_accel(uint8_t *raw, float *g, enum mpu9250_accel_scale scale)
{
    int16_t x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t y = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t z = (int16_t)((raw[4] << 8) | raw[5]);
    float factor;
    switch (scale) {
        case ACCEL_SCALE_2G: factor = 16384.0f; break;
        case ACCEL_SCALE_4G: factor = 8192.0f; break;
        case ACCEL_SCALE_8G: factor = 4096.0f; break;
        case ACCEL_SCALE_16G: factor = 2048.0f; break;
        default: factor = 16384.0f; break;
    }
    g[0] = x / factor;
    g[1] = y / factor;
    g[2] = z / factor;
}

void convert_gyro(uint8_t *raw, float *dps, enum mpu9250_gyro_scale scale)
{
    int16_t x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t y = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t z = (int16_t)((raw[4] << 8) | raw[5]);
    float factor;
    switch (scale) {
        case GYRO_SCALE_250DPS: factor = 131.0f; break;
        case GYRO_SCALE_500DPS: factor = 65.5f; break;
        case GYRO_SCALE_1000DPS: factor = 32.8f; break;
        case GYRO_SCALE_2000DPS: factor = 16.4f; break;
        default: factor = 131.0f; break;
    }
    dps[0] = x / factor;
    dps[1] = y / factor;
    dps[2] = z / factor;
}

void convert_mag(uint8_t *raw, float *uT)
{
    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);
    uT[0] = x * MAG_SCALE;
    uT[1] = y * MAG_SCALE;
    uT[2] = z * MAG_SCALE;
}

void convert_quat(uint8_t *raw, float *quat, int len)
{
    if (len >= 16) {
        int32_t q0 = (int32_t)((raw[0] << 24) | (raw[1] << 16) | (raw[2] << 8) | raw[3]);
        int32_t q1 = (int32_t)((raw[4] << 24) | (raw[5] << 16) | (raw[6] << 8) | raw[7]);
        int32_t q2 = (int32_t)((raw[8] << 24) | (raw[9] << 16) | (raw[10] << 8) | raw[11]);
        int32_t q3 = (int32_t)((raw[12] << 24) | (raw[13] << 16) | (raw[14] << 8) | raw[15]);
        quat[0] = q0 / (1 << 30);
        quat[1] = q1 / (1 << 30);
        quat[2] = q2 / (1 << 30);
        quat[3] = q3 / (1 << 30);
    }
}

// Device reset and init
int mpu9250_i2c_init(struct mpu9250_data *data)
{
    uint8_t buf = 0x80; // Reset device
    int ret = mpu9250_write(data, MPU9250_PWR_MGMT_1, &buf, 1);
    if (ret)
        return ret;
    msleep(100); // Wait for reset
    buf = 0x01; // Auto-select clock source
    ret = mpu9250_write(data, MPU9250_PWR_MGMT_1, &buf, 1);
    if (ret)
        return ret;
    buf = 0x00; // Disable sleep mode
    ret = mpu9250_write(data, MPU9250_PWR_MGMT_1, &buf, 1);
    if (ret)
        return ret;
    return 0;
}

int mpu9250_fifo_init(struct mpu9250_data *data)
{
    uint8_t buf = 0x00;
    int ret = mpu9250_write(data, MPU9250_FIFO_EN, &buf, 1); // Disable FIFO
    if (ret)
        return ret;
    buf = 0x78; // Enable FIFO for accel, gyro, mag
    ret = mpu9250_write(data, MPU9250_FIFO_EN, &buf, 1);
    if (ret)
        return ret;
    buf = 0x08; // Reset FIFO
    ret = mpu9250_write(data, MPU9250_USER_CTRL, &buf, 1);
    if (ret)
        return ret;
    buf = 0x40; // Enable FIFO
    ret = mpu9250_write(data, MPU9250_USER_CTRL, &buf, 1);
    if (ret)
        return ret;
    return 0;
}

int mpu9250_read_fifo(struct mpu9250_data *data, uint8_t *buf, int *len)
{
    uint8_t count_h, count_l;
    int ret;
    ret = mpu9250_read(data, MPU9250_FIFO_COUNTH, &count_h, 1);
    if (ret)
        return ret;
    ret = mpu9250_read(data, MPU9250_FIFO_COUNTL, &count_l, 1);
    if (ret)
        return ret;
    *len = (count_h << 8) | count_l;
    if (*len > MPU9250_MAX_FIFO)
        *len = MPU9250_MAX_FIFO;
    return mpu9250_read(data, MPU9250_FIFO_R_W, buf, *len);
}

int mpu9250_dmp_init(struct mpu9250_data *data)
{
    uint8_t buf = 0x03;
    int ret = mpu9250_write(data, MPU9250_DMP_CFG_1, &buf, 1);
    if (ret)
        return ret;
    buf = 0x00;
    ret = mpu9250_write(data, MPU9250_DMP_CFG_2, &buf, 1);
    if (ret)
        return ret;
    // Additional DMP setup if needed
    return 0;
}

void mpu9250_read_dmp_quat(struct mpu9250_data *data, float *quat)
{
    uint8_t quat_raw[16];
    int len = 16;
    if (mpu9250_read_fifo(data, quat_raw, &len) == 0)
        convert_quat(quat_raw, quat, len);
}

// New: Calibration function for accel/gyro (basic zero-offset)
int mpu9250_calibrate(struct mpu9250_data *data)
{
    // Read multiple samples and average for offset (demo)
    float accel_sum[3] = {0}, gyro_sum[3] = {0};
    int samples = 100;
    for (int i = 0; i < samples; i++) {
        uint8_t raw[6];
        mpu9250_read(data, MPU9250_ACCEL_XOUT_H, raw, 6);
        float g[3];
        convert_accel(raw, g, data->accel_scale);
        accel_sum[0] += g[0]; accel_sum[1] += g[1]; accel_sum[2] += g[2];

        mpu9250_read(data, MPU9250_GYRO_XOUT_H, raw, 6);
        float dps[3];
        convert_gyro(raw, dps, data->gyro_scale);
        gyro_sum[0] += dps[0]; gyro_sum[1] += dps[1]; gyro_sum[2] += dps[2];
        msleep(10);
    }
    // Store offsets (in data struct, assume added fields in header)
    data->accel_offset[0] = accel_sum[0] / samples;
    data->accel_offset[1] = accel_sum[1] / samples;
    data->accel_offset[2] = accel_sum[2] / samples - 1.0f; // Gravity correction for Z

    data->gyro_offset[0] = gyro_sum[0] / samples;
    data->gyro_offset[1] = gyro_sum[1] / samples;
    data->gyro_offset[2] = gyro_sum[2] / samples;
    return 0;
}

// Scale configuration with validation
int mpu9250_set_accel_scale(struct mpu9250_data *data, enum mpu9250_accel_scale scale)
{
    if (scale < ACCEL_SCALE_2G || scale > ACCEL_SCALE_16G) return -EINVAL;
    uint8_t buf = scale << 3;
    int ret = mpu9250_write(data, MPU9250_ACCEL_CONFIG, &buf, 1);
    if (!ret) data->accel_scale = scale;
    return ret;
}

int mpu9250_set_gyro_scale(struct mpu9250_data *data, enum mpu9250_gyro_scale scale)
{
    if (scale < GYRO_SCALE_250DPS || scale > GYRO_SCALE_2000DPS) return -EINVAL;
    uint8_t buf = scale << 3;
    int ret = mpu9250_write(data, MPU9250_GYRO_CONFIG, &buf, 1);
    if (!ret) data->gyro_scale = scale;
    return ret;
}

int mpu9250_set_sample_rate(struct mpu9250_data *data, unsigned int rate)
{
    if (rate < 4 || rate > 1000) return -EINVAL; // Validate range
    uint8_t div = (1000 / rate) - 1;
    int ret = mpu9250_write(data, MPU9250_SMPLRT_DIV, &div, 1);
    if (!ret) data->sample_rate = rate;
    return ret;
}