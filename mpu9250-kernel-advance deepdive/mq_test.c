// mq_test.c
// User-space test for MPU9250 kernel driver using POSIX message queue and shared memory.
// Compile: gcc -o mq_test mq_test.c -lrt -pthread -Wall -Wextra -std=c99
// Run: sudo ./mq_test [--log file]
// Author: Nguyen Nhan

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <mqueue.h>
#include <semaphore.h>
#include <getopt.h>
#include "mpu9250_driver.h"

static mqd_t mq = -1;
static sem_t *sem = SEM_FAILED;
static int shm_fd = -1;
static struct mpu9250_mq_data *shm_data = NULL;
static FILE *log_file = NULL;
static volatile int running = 1;

static void signal_handler(int sig)
{
    running = 0;
    if (mq != -1) mq_close(mq);
    if (sem != SEM_FAILED) sem_close(sem);
    if (shm_fd >= 0) {
        munmap(shm_data, sizeof(*shm_data));
        close(shm_fd);
    }
    if (log_file) fclose(log_file);
    printf("MPU9250 MQ: Closed on signal %d\n", sig);
    exit(0);
}

static void log_data(struct mpu9250_mq_data *data)
{
    if (log_file) {
        fprintf(log_file, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                data->accel[0], data->accel[1], data->accel[2],
                data->gyro[0], data->gyro[1], data->gyro[2],
                data->mag[0], data->mag[1], data->mag[2],
                data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
        fflush(log_file);
    }
}

int main(int argc, char **argv)
{
    struct mpu9250_mq_data mq_data;
    char *log_path = NULL;
    int ret;

    // Parse command-line arguments
    struct option long_opts[] = {
        {"log", required_argument, NULL, 'l'},
        {0, 0, 0, 0}
    };
    while ((ret = getopt_long(argc, argv, "l:", long_opts, NULL)) != -1) {
        switch (ret) {
        case 'l': log_path = optarg; break;
        default: fprintf(stderr, "Usage: %s [--log file]\n", argv[0]); return 1;
        }
    }

    // Open log file
    if (log_path) {
        log_file = fopen(log_path, "w");
        if (!log_file) {
            perror("Open log file failed");
            return 1;
        }
        fprintf(log_file, "AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,QuatW,QuatX,QuatY,QuatZ\n");
    }

    // Set process group
    if (setpgid(0, 0) < 0) {
        perror("Set process group failed");
        if (log_file) fclose(log_file);
        return 1;
    }

    // Signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGUSR1, signal_handler);
    signal(SIGUSR2, signal_handler);
    signal(SIGHUP, signal_handler);

    // Open message queue
    struct mq_attr attr = { .mq_maxmsg = 10, .mq_msgsize = sizeof(struct mpu9250_mq_data) };
    mq = mq_open(MPU9250_MQ_NAME, O_CREAT | O_RDONLY, 0666, &attr);
    if (mq == -1) {
        perror("Open message queue failed");
        if (log_file) fclose(log_file);
        return 1;
    }

    // Open shared memory
    shm_fd = shm_open(MPU9250_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        perror("Open shared memory failed");
        mq_close(mq);
        if (log_file) fclose(log_file);
        return 1;
    }
    ftruncate(shm_fd, sizeof(*shm_data));
    shm_data = mmap(NULL, sizeof(*shm_data), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_data == MAP_FAILED) {
        perror("Map shared memory failed");
        close(shm_fd);
        mq_close(mq);
        if (log_file) fclose(log_file);
        return 1;
    }

    // Open semaphore
    sem = sem_open(MPU9250_SEM_NAME, O_CREAT, 0666, 1);
    if (sem == SEM_FAILED) {
        perror("Open semaphore failed");
        munmap(shm_data, sizeof(*shm_data));
        close(shm_fd);
        mq_close(mq);
        if (log_file) fclose(log_file);
        return 1;
    }

    // Receive messages
    while (running) {
        ret = mq_receive(mq, (char *)&mq_data, sizeof(mq_data), NULL);
        if (ret < 0) {
            if (errno != EAGAIN) perror("Receive message failed");
            usleep(100000);
            continue;
        }
        sem_wait(sem);
        log_data(&mq_data);
        printf("MQ: Accel=%.2f,%.2f,%.2f Gyro=%.2f,%.2f,%.2f Mag=%.2f,%.2f,%.2f Quat=%.2f,%.2f,%.2f,%.2f\n",
               mq_data.accel[0], mq_data.accel[1], mq_data.accel[2],
               mq_data.gyro[0], mq_data.gyro[1], mq_data.gyro[2],
               mq_data.mag[0], mq_data.mag[1], mq_data.mag[2],
               mq_data.quat[0], mq_data.quat[1], mq_data.quat[2], mq_data.quat[3]);
        sem_wait(sem);
        memcpy(&mq_data, shm_data, sizeof(mq_data));
        log_data(&mq_data);
        printf("SHM: Accel=%.2f,%.2f,%.2f Gyro=%.2f,%.2f,%.2f Mag=%.2f,%.2f,%.2f Quat=%.2f,%.2f,%.2f,%.2f\n",
               mq_data.accel[0], mq_data.accel[1], mq_data.accel[2],
               mq_data.gyro[0], mq_data.gyro[1], mq_data.gyro[2],
               mq_data.mag[0], mq_data.mag[1], mq_data.mag[2],
               mq_data.quat[0], mq_data.quat[1], mq_data.quat[2], mq_data.quat[3]);
        sem_post(sem);
        usleep(100000);
    }

    // Cleanup
    mq_close(mq);
    sem_close(sem);
    munmap(shm_data, sizeof(*shm_data));
    close(shm_fd);
    if (log_file) fclose(log_file);
    printf("MPU9250 MQ: Test completed\n");
    return 0;
}