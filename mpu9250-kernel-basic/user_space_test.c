// user_space_test.c
// User-space test for MPU9250 kernel driver on Raspberry Pi 4.
// Uses POSIX threads, synchronization, and message queue for parallel testing.
// Compile: gcc -o user_test user_space_test.c -lm -pthread -lrt -Wall -Wextra -std=c99
// Run: sudo ./user_test

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <mqueue.h>
#include "mpu9250_driver.h"

static int fd = -1;
static mqd_t mq = -1;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
static volatile int running = 1;

static void signal_handler(int sig)
{
    running = 0;
    pthread_cond_broadcast(&cond);
    if (fd >= 0) close(fd);
    if (mq != -1) mq_close(mq);
    printf("MPU9250: Closed on signal %d\n", sig);
    exit(0);
}

static void *accel_thread(void *arg)
{
    struct mpu9250_data data = {0};
    while (running) {
        pthread_mutex_lock(&mutex);
        if (ioctl(fd, MPU9250_IOCTL_READ_ACCEL, &data) < 0) {
            perror("Accel thread read failed");
        } else {
            struct mpu9250_mq_data mq_data;
            memcpy(mq_data.accel, data.accel, sizeof(data.accel));
            mq_send(mq, (char *)&mq_data, sizeof(mq_data), 0);
            printf("Accel: X=%.2f g, Y=%.2f g, Z=%.2f g\n",
                   data.accel[0], data.accel[1], data.accel[2]);
        }
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_unlock(&mutex);
        usleep(100000);
    }
    return NULL;
}

static void *gyro_thread(void *arg)
{
    struct mpu9250_data data = {0};
    while (running) {
        pthread_mutex_lock(&mutex);
        if (ioctl(fd, MPU9250_IOCTL_READ_GYRO, &data) < 0) {
            perror("Gyro thread read failed");
        } else {
            struct mpu9250_mq_data mq_data;
            memcpy(mq_data.gyro, data.gyro, sizeof(data.gyro));
            mq_send(mq, (char *)&mq_data, sizeof(mq_data), 0);
            printf("Gyro: X=%.2f dps, Y=%.2f dps, Z=%.2f dps\n",
                   data.gyro[0], data.gyro[1], data.gyro[2]);
        }
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_unlock(&mutex);
        usleep(100000);
    }
    return NULL;
}

static void *mag_thread(void *arg)
{
    struct mpu9250_data data = {0};
    while (running) {
        pthread_mutex_lock(&mutex);
        if (ioctl(fd, MPU9250_IOCTL_READ_MAG, &data) < 0) {
            perror("Mag thread read failed");
        } else {
            struct mpu9250_mq_data mq_data;
            memcpy(mq_data.mag, data.mag, sizeof(data.mag));
            mq_send(mq, (char *)&mq_data, sizeof(mq_data), 0);
            printf("Mag: X=%.2f uT, Y=%.2f uT, Z=%.2f uT\n",
                   data.mag[0], data.mag[1], data.mag[2]);
        }
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_unlock(&mutex);
        usleep(100000);
    }
    return NULL;
}

static void *dmp_thread(void *arg)
{
    struct mpu9250_data data = {0};
    while (running) {
        pthread_mutex_lock(&mutex);
        if (ioctl(fd, MPU9250_IOCTL_INIT_DMP, 0) < 0 ||
            ioctl(fd, MPU9250_IOCTL_READ_DMP, &data) < 0) {
            perror("DMP thread read failed");
        } else {
            struct mpu9250_mq_data mq_data;
            memcpy(mq_data.quat, data.quat, sizeof(data.quat));
            mq_send(mq, (char *)&mq_data, sizeof(mq_data), 0);
            printf("DMP: w=%.2f, x=%.2f, y=%.2f, z=%.2f\n",
                   data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
        }
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_unlock(&mutex);
        usleep(100000);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    struct mpu9250_reg reg;
    struct mpu9250_data data = {0};
    uint8_t write_buf[2] = {MPU9250_PWR_MGMT_1, 0x00};
    pthread_t accel_tid, gyro_tid, mag_tid, dmp_tid;
    pid_t pid;
    int ret;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGUSR1, signal_handler);

    // Open message queue
    struct mq_attr attr = { .mq_maxmsg = 10, .mq_msgsize = sizeof(struct mpu9250_mq_data) };
    mq = mq_open(MPU9250_MQ_NAME, O_CREAT | O_WRONLY, 0666, &attr);
    if (mq == -1) {
        perror("Open message queue failed");
        return 1;
    }

    // Fork child process to run mq_test
    pid = fork();
    if (pid < 0) {
        perror("Fork failed");
        mq_close(mq);
        return 1;
    } else if (pid == 0) {
        execl("./mq_test", "mq_test", NULL);
        perror("Exec mq_test failed");
        exit(1);
    }

    fd = open("/dev/mpu9250", O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        perror("Open /dev/mpu9250 failed");
        mq_close(mq);
        return 1;
    }

    // Set non-blocking
    ret = ioctl(fd, MPU9250_IOCTL_SET_NONBLOCK, 1);
    if (ret < 0) {
        perror("Set non-blocking failed");
        goto err;
    }

    // Write wakeup
    ret = write(fd, write_buf, 2);
    if (ret != 2) {
        perror("Write wakeup failed");
        goto err;
    }
    printf("MPU9250: Wakeup written\n");

    // Read WHO_AM_I
    reg.reg = MPU9250_WHO_AM_I;
    ret = ioctl(fd, MPU9250_IOCTL_READ_REG, &reg);
    if (ret < 0) {
        perror("IOCTL read WHO_AM_I failed");
        goto err;
    }
    printf("MPU9250: WHO_AM_I = 0x%02X\n", reg.val);

    // Create threads
    pthread_create(&accel_tid, NULL, accel_thread, NULL);
    pthread_create(&gyro_tid, NULL, gyro_thread, NULL);
    pthread_create(&mag_tid, NULL, mag_thread, NULL);
    pthread_create(&dmp_tid, NULL, dmp_thread, NULL);

    // Signal threads to start
    pthread_cond_broadcast(&cond);

    // Wait for threads
    pthread_join(accel_tid, NULL);
    pthread_join(gyro_tid, NULL);
    pthread_join(mag_tid, NULL);
    pthread_join(dmp_tid, NULL);

    // Wait for child process
    wait(NULL);

err:
    if (fd >= 0) close(fd);
    if (mq != -1) {
        mq_close(mq);
        mq_unlink(MPU9250_MQ_NAME);
    }
    printf("MPU9250: Test completed\n");
    return 0;
}