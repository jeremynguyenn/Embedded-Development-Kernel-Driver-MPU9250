// aio_test.c
// User-space test for MPU9250 kernel driver using asynchronous I/O on Raspberry Pi 4.
// Compile: gcc -o aio_test aio_test.c -laio -pthread -lrt -Wall -Wextra -std=c99
// Run: sudo ./aio_test [--log file] [--sample-rate hz] [--accel-range g] [--gyro-range dps]
// Author: Nguyen Nhan

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <libaio.h>
#include <semaphore.h>
#include <getopt.h>
#include "mpu9250_driver.h"

static int fd = -1;
static sem_t *sem = SEM_FAILED;
static int shm_fd = -1;
static struct mpu9250_mq_data *shm_data = NULL;
static FILE *log_file = NULL;
static volatile int running = 1;
static struct mpu9250_config config = {1000, 2, 250, true};

static void signal_handler(int sig)
{
    running = 0;
    if (fd >= 0) close(fd);
    if (sem != SEM_FAILED) sem_close(sem);
    if (shm_fd >= 0) {
        munmap(shm_data, sizeof(*shm_data));
        close(shm_fd);
    }
    if (log_file) fclose(log_file);
    printf("MPU9250 AIO: Closed on signal %d\n", sig);
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

static void aio_completion_handler(io_context_t ctx, struct iocb *iocb, long res, long res2)
{
    struct mpu9250_data *data = iocb->data;
    if (res < 0) {
        fprintf(stderr, "AIO read failed: %ld\n", res);
    } else {
        sem_wait(sem);
        memcpy(shm_data->accel, data->accel, sizeof(data->accel));
        memcpy(shm_data->gyro, data->gyro, sizeof(data->gyro));
        memcpy(shm_data->mag, data->mag, sizeof(data->mag));
        memcpy(shm_data->quat, data->quat, sizeof(data->quat));
        log_data(shm_data);
        printf("AIO: Accel=%.2f,%.2f,%.2f Gyro=%.2f,%.2f,%.2f Mag=%.2f,%.2f,%.2f Quat=%.2f,%.2f,%.2f,%.2f\n",
               data->accel[0], data->accel[1], data->accel[2],
               data->gyro[0], data->gyro[1], data->gyro[2],
               data->mag[0], data->mag[1], data->mag[2],
               data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
        sem_post(sem);
    }
    free(data);
    free(iocb);
}

int main(int argc, char **argv)
{
    io_context_t ctx = 0;
    struct iocb *iocb;
    struct mpu9250_data *data;
    char *log_path = NULL;
    int ret;

    // Parse command-line arguments
    struct option long_opts[] = {
        {"log", required_argument, NULL, 'l'},
        {"sample-rate", required_argument, NULL, 's'},
        {"accel-range", required_argument, NULL, 'a'},
        {"gyro-range", required_argument, NULL, 'g'},
        {0, 0, 0, 0}
    };
    while ((ret = getopt_long(argc, argv, "l:s:a:g:", long_opts, NULL)) != -1) {
        switch (ret) {
        case 'l': log_path = optarg; break;
        case 's': config.sample_rate = atoi(optarg); break;
        case 'a': config.accel_range = atoi(optarg); break;
        case 'g': config.gyro_range = atoi(optarg); break;
        default: fprintf(stderr, "Usage: %s [--log file] [--sample-rate hz] [--accel-range g] [--gyro-range dps]\n", argv[0]); return 1;
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

    // Signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGUSR1, signal_handler);
    signal(SIGUSR2, signal_handler);
    signal(SIGHUP, signal_handler);

    // Open shared memory
    shm_fd = shm_open(MPU9250_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        perror("Open shared memory failed");
        if (log_file) fclose(log_file);
        return 1;
    }
    ftruncate(shm_fd, sizeof(*shm_data));
    shm_data = mmap(NULL, sizeof(*shm_data), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_data == MAP_FAILED) {
        perror("Map shared memory failed");
        close(shm_fd);
        if (log_file) fclose(log_file);
        return 1;
    }

    // Open semaphore
    sem = sem_open(MPU9250_SEM_NAME, O_CREAT, 0666, 1);
    if (sem == SEM_FAILED) {
        perror("Open semaphore failed");
        munmap(shm_data, sizeof(*shm_data));
        close(shm_fd);
        if (log_file) fclose(log_file);
        return 1;
    }

    // Open device
    fd = open("/dev/mpu9250", O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        perror("Open /dev/mpu9250 failed");
        sem_close(sem);
        munmap(shm_data, sizeof(*shm_data));
        close(shm_fd);
        if (log_file) fclose(log_file);
        return 1;
    }

    // Set configuration
    ret = ioctl(fd, MPU9250_IOCTL_SET_CONFIG, &config);
    if (ret < 0) {
        perror("Set config failed");
        goto err;
    }

    // Initialize AIO
    ret = io_setup(1, &ctx);
    if (ret < 0) {
        perror("AIO setup failed");
        goto err;
    }

    // Submit AIO read
    while (running) {
        data = malloc(sizeof(*data));
        iocb = malloc(sizeof(*iocb));
        if (!data || !iocb) {
            fprintf(stderr, "Memory allocation failed\n");
            if (data) free(data);
            if (iocb) free(iocb);
            break;
        }
        io_prep_pread(iocb, fd, data, sizeof(*data), 0);
        iocb->data = data;
        io_set_callback(iocb, aio_completion_handler);
        ret = io_submit(ctx, 1, &iocb);
        if (ret != 1) {
            fprintf(stderr, "AIO submit failed: %d\n", ret);
            free(data);
            free(iocb);
            break;
        }
        usleep(1000000 / config.sample_rate);
    }

    // Cleanup
    io_destroy(ctx);
err:
    close(fd);
    sem_close(sem);
    munmap(shm_data, sizeof(*shm_data));
    close(shm_fd);
    if (log_file) fclose(log_file);
    printf("MPU9250 AIO: Test completed\n");
    return 0;
}