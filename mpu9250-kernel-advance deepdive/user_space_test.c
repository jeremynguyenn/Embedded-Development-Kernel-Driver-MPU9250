// user_space_test.c
// User-space test for MPU9250 kernel driver on Raspberry Pi 4.
// Supports IIO, AIO, POSIX threads, semaphores, shared memory, and daemon mode.
// Compile: gcc -o user_test user_space_test.c -lm -pthread -lrt -laio -Wall -Wextra -std=c99
// Run: sudo ./user_test [--log file] [--sample-rate hz] [--accel-range g] [--gyro-range dps] [--use-iio] [--use-aio]
// Author: Nguyen Nhan

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <mqueue.h>
#include <semaphore.h>
#include <libaio.h>
#include <getopt.h>
#include "mpu9250_driver.h"

static int fd = -1;
static int iio_fd = -1;
static mqd_t mq = -1;
static sem_t *sem = SEM_FAILED;
static int shm_fd = -1;
static struct mpu9250_mq_data *shm_data = NULL;
static FILE *log_file = NULL;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
static volatile int running = 1;
static struct mpu9250_config config = {1000, 2, 250, true, true};
static bool use_iio = false;
static bool use_aio = false;

static void signal_handler(int sig)
{
    if (sig == SIGHUP) {
        // Reload configuration from /sys/class/mpu9250_class/config
        FILE *config_file = fopen("/sys/class/mpu9250_class/config", "r");
        if (config_file) {
            fscanf(config_file, "Sample Rate: %u Hz\nAccel Range: %hhu g\nGyro Range: %hu dps\nBatch: %d\nIIO Enabled: %d\n",
                   &config.sample_rate, &config.accel_range, &config.gyro_range,
                   &config.enable_batch, &config.enable_iio);
            fclose(config_file);
            if (fd >= 0)
                ioctl(fd, MPU9250_IOCTL_SET_CONFIG, &config);
            printf("Reloaded config: sample_rate=%u, accel_range=%hhu, gyro_range=%hu\n",
                   config.sample_rate, config.accel_range, config.gyro_range);
        }
        return;
    }
    running = 0;
    pthread_cond_broadcast(&cond);
    if (fd >= 0) close(fd);
    if (iio_fd >= 0) close(iio_fd);
    if (mq != -1) mq_close(mq);
    if (sem != SEM_FAILED) sem_close(sem);
    if (shm_fd >= 0) {
        munmap(shm_data, sizeof(*shm_data));
        close(shm_fd);
    }
    if (log_file) fclose(log_file);
    printf("MPU9250: Closed on signal %d\n", sig);
    exit(0);
}

static void log_data(struct mpu9250_mq_data *data)
{
    if (log_file) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        fprintf(log_file, "%ld.%06ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                tv.tv_sec, tv.tv_usec,
                data->accel[0], data->accel[1], data->accel[2],
                data->gyro[0], data->gyro[1], data->gyro[2],
                data->mag[0], data->mag[1], data->mag[2],
                data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
        fflush(log_file);
    }
}

static void *accel_thread(void *arg)
{
    struct mpu9250_data data = {0};
    while (running) {
        sem_wait(sem);
        pthread_mutex_lock(&mutex);
        if (use_iio && iio_fd >= 0) {
            int16_t iio_data[9];
            if (read(iio_fd, iio_data, sizeof(iio_data)) < 0) {
                perror("IIO accel read failed");
            } else {
                struct mpu9250_mq_data mq_data = {0};
                mq_data.accel[0] = iio_data[0] / ACCEL_SCALE;
                mq_data.accel[1] = iio_data[1] / ACCEL_SCALE;
                mq_data.accel[2] = iio_data[2] / ACCEL_SCALE;
                sem_wait(sem);
                memcpy(shm_data->accel, mq_data.accel, sizeof(mq_data.accel));
                log_data(&mq_data);
                mq_send(mq, (char *)&mq_data, sizeof(mq_data), 0);
                sem_post(sem);
                printf("IIO Accel: X=%.2f g, Y=%.2f g, Z=%.2f g\n",
                       mq_data.accel[0], mq_data.accel[1], mq_data.accel[2]);
            }
        } else if (ioctl(fd, MPU9250_IOCTL_READ_ACCEL, &data) < 0) {
            perror("Accel thread read failed");
        } else {
            struct mpu9250_mq_data mq_data = {0};
            memcpy(mq_data.accel, data.accel, sizeof(data.accel));
            sem_wait(sem);
            memcpy(shm_data->accel, data.accel, sizeof(data.accel));
            log_data(&mq_data);
            mq_send(mq, (char *)&mq_data, sizeof(mq_data), 0);
            sem_post(sem);
            printf("Accel: X=%.2f g, Y=%.2f g, Z=%.2f g\n",
                   data.accel[0], data.accel[1], data.accel[2]);
        }
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_unlock(&mutex);
        sem_post(sem);
        usleep(1000000 / config.sample_rate);
    }
    return NULL;
}

static void *gyro_thread(void *arg)
{
    struct mpu9250_data data = {0};
    while (running) {
        sem_wait(sem);
        pthread_mutex_lock(&mutex);
        if (use_iio && iio_fd >= 0) {
            int16_t iio_data[9];
            if (read(iio_fd, iio_data, sizeof(iio_data)) < 0) {
                perror("IIO gyro read failed");
            } else {
                struct mpu9250_mq_data mq_data = {0};
                mq_data.gyro[0] = iio_data[3] / GYRO_SCALE;
                mq_data.gyro[1] = iio_data[4] / GYRO_SCALE;
                mq_data.gyro[2] = iio_data[5] / GYRO_SCALE;
                sem_wait(sem);
                memcpy(shm_data->gyro, mq_data.gyro, sizeof(mq_data.gyro));
                log_data(&mq_data);
                mq_send(mq, (char *)&mq_data, sizeof(mq_data), 0);
                sem_post(sem);
                printf("IIO Gyro: X=%.2f dps, Y=%.2f dps, Z=%.2f dps\n