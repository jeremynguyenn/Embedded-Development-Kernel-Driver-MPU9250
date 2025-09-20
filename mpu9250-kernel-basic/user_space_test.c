// user_space_test.c
// User-space test for MPU9250 kernel driver on Raspberry Pi 4.
// Supports POSIX threads, synchronization, message queues, command-line args, signal sending, and detachable threads.
// Compile: gcc -o user_test user_space_test.c -lm -pthread -lrt -Wall -Wextra -std=c99
// Run: sudo ./user_test [--accel] [--gyro] [--mag] [--dmp] [--detach] [--sample-rate RATE] [--accel-scale SCALE] [--gyro-scale SCALE]

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
#include <getopt.h>
#include "mpu9250.h"

static int fd = -1;
static mqd_t mq = -1;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
static volatile int running = 1;
static int test_accel = 0, test_gyro = 0, test_mag = 0, test_dmp = 0, test_detach = 0;
static unsigned int sample_rate = 100;
static int accel_scale = ACCEL_SCALE_2G;
static int gyro_scale = GYRO_SCALE_250DPS;

static void signal_handler(int sig)
{
    if (sig == SIGUSR1) {
        printf("MPU9250: Received SIGUSR1, continuing\n");
        return;
    }
    running = 0;
    pthread_cond_broadcast(&cond);
    if (fd >= 0) close(fd);
    if (mq != -1) {
        mq_close(mq);
        mq_unlink(MPU9250_MQ_NAME);
    }
    printf("MPU9250: Closed on signal %d\n", sig);
    exit(0);
}

// Demo default signal handler: By default, SIGINT terminates the process.
static void default_signal_demo() {
    // To use default, call signal(SIGINT, SIG_DFL);
    printf("Demo: Default SIGINT would terminate without custom handler.\n");
    signal(SIGINT, SIG_DFL); // New: Set to default
}

static void *accel_thread(void *arg)
{
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data)); // New: malloc
    if (!data) pthread_exit(NULL);
    pthread_t tid = pthread_self(); // New: Get Thread ID
    printf("Accel Thread ID: %lu\n", (unsigned long)tid);
    while (running) {
        pthread_mutex_lock(&mutex);
        pthread_cond_wait(&cond, &mutex); // New: Use cond_wait instead of usleep
        if (ioctl(fd, MPU9250_IOCTL_READ_ACCEL, data) < 0) {
            fprintf(stderr, "Accel thread read failed: %s\n", strerror(errno));
        } else {
            struct mpu9250_mq_data mq_data = {0};
            memcpy(mq_data.accel, data->values, sizeof(data->values));
            printf("Accel: %.2f %.2f %.2f\n", mq_data.accel[0], mq_data.accel[1], mq_data.accel[2]);
        }
        pthread_mutex_unlock(&mutex);
    }
    free(data); // New: free
    pthread_exit(NULL); // New: Explicit exit
    return NULL;
}

static void *gyro_thread(void *arg)
{
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data));
    if (!data) pthread_exit(NULL);
    pthread_t tid = pthread_self();
    printf("Gyro Thread ID: %lu\n", (unsigned long)tid);
    while (running) {
        pthread_mutex_lock(&mutex);
        pthread_cond_wait(&cond, &mutex);
        if (ioctl(fd, MPU9250_IOCTL_READ_GYRO, data) < 0) {
            fprintf(stderr, "Gyro thread read failed: %s\n", strerror(errno));
        } else {
            struct mpu9250_mq_data mq_data = {0};
            memcpy(mq_data.gyro, data->values, sizeof(data->values));
            printf("Gyro: %.2f %.2f %.2f\n", mq_data.gyro[0], mq_data.gyro[1], mq_data.gyro[2]);
        }
        pthread_mutex_unlock(&mutex);
    }
    free(data);
    pthread_exit(NULL);
    return NULL;
}

static void *mag_thread(void *arg)
{
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data));
    if (!data) pthread_exit(NULL);
    pthread_t tid = pthread_self();
    printf("Mag Thread ID: %lu\n", (unsigned long)tid);
    while (running) {
        pthread_mutex_lock(&mutex);
        pthread_cond_wait(&cond, &mutex);
        if (ioctl(fd, MPU9250_IOCTL_READ_MAG, data) < 0) {
            fprintf(stderr, "Mag thread read failed: %s\n", strerror(errno));
        } else {
            struct mpu9250_mq_data mq_data = {0};
            memcpy(mq_data.mag, data->values, sizeof(data->values));
            printf("Mag: %.2f %.2f %.2f\n", mq_data.mag[0], mq_data.mag[1], mq_data.mag[2]);
        }
        pthread_mutex_unlock(&mutex);
    }
    free(data);
    pthread_exit(NULL);
    return NULL;
}

static void *dmp_thread(void *arg)
{
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data));
    if (!data) pthread_exit(NULL);
    pthread_t tid = pthread_self();
    printf("DMP Thread ID: %lu\n", (unsigned long)tid);
    while (running) {
        pthread_mutex_lock(&mutex);
        pthread_cond_wait(&cond, &mutex);
        if (ioctl(fd, MPU9250_IOCTL_READ_DMP, data) < 0) {
            fprintf(stderr, "DMP thread read failed: %s\n", strerror(errno));
        } else {
            struct mpu9250_mq_data mq_data = {0};
            memcpy(mq_data.quat, data->values, sizeof(float)*4);
            printf("Quat: %.2f %.2f %.2f %.2f\n", mq_data.quat[0], mq_data.quat[1], mq_data.quat[2], mq_data.quat[3]);
        }
        pthread_mutex_unlock(&mutex);
    }
    free(data);
    pthread_exit(NULL);
    return NULL;
}

int main(int argc, char **argv)
{
    // Parse args (full from truncated)
    struct option long_options[] = {
        {"accel", no_argument, &test_accel, 1},
        {"gyro", no_argument, &test_gyro, 1},
        {"mag", no_argument, &test_mag, 1},
        {"dmp", no_argument, &test_dmp, 1},
        {"detach", no_argument, &test_detach, 1},
        {"sample-rate", required_argument, NULL, 'r'},
        {"accel-scale", required_argument, NULL, 'a'},
        {"gyro-scale", required_argument, NULL, 'g'},
        {0, 0, 0, 0}
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "", long_options, NULL)) != -1) {
        switch (opt) {
            case 'r': sample_rate = atoi(optarg); break;
            case 'a': accel_scale = atoi(optarg); break;
            case 'g': gyro_scale = atoi(optarg); break;
        }
    }

    default_signal_demo();

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGUSR1, signal_handler);

    fd = open("/dev/mpu9250", O_RDWR);
    if (fd < 0) {
        perror("Open /dev/mpu9250 failed");
        return 1;
    }

    int ret = ioctl(fd, MPU9250_IOCTL_SET_ACCEL_SCALE, accel_scale);
    if (ret < 0) {
        perror("Set accel scale failed");
        goto err;
    }
    ret = ioctl(fd, MPU9250_IOCTL_SET_GYRO_SCALE, gyro_scale);
    if (ret < 0) {
        perror("Set gyro scale failed");
        goto err;
    }
    ret = ioctl(fd, MPU9250_IOCTL_SET_SAMPLE_RATE, sample_rate);
    if (ret < 0) {
        perror("Set sample rate failed");
        goto err;
    }

    uint8_t write_buf[2] = {MPU9250_PWR_MGMT_1, 0x00};
    ret = write(fd, write_buf, 2);
    if (ret != 2) {
        perror("Write wakeup failed");
        goto err;
    }
    printf("MPU9250: Wakeup written\n");

    struct mpu9250_reg reg;
    reg.reg = MPU9250_WHO_AM_I;
    ret = ioctl(fd, MPU9250_IOCTL_READ_REG, &reg);
    if (ret < 0) {
        perror("IOCTL read WHO_AM_I failed");
        goto err;
    }
    printf("MPU9250: WHO_AM_I = 0x%02X\n", reg.val);

    // New: Demo exec (fork and exec child to run another program)
    pid_t exec_pid = fork();
    if (exec_pid == 0) {
        execl("/bin/ls", "ls", "-l", NULL); // Demo exec
        perror("exec failed");
        exit(1);
    } else if (exec_pid > 0) {
        wait(NULL); // Wait for exec child
    }

    pthread_attr_t attr;
    if (test_detach) {
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    }

    pid_t child_pid = fork();
    if (child_pid == -1) {
        perror("Fork failed");
        goto err;
    }
    if (child_pid == 0) {
        sleep(2);
        if (kill(getppid(), SIGUSR1) == -1) {
            perror("Send SIGUSR1 failed");
            exit(1);
        }
        exit(0);
    }

    pthread_t accel_tid, gyro_tid, mag_tid, dmp_tid;
    if (test_accel)
        pthread_create(&accel_tid, test_detach ? &attr : NULL, accel_thread, NULL);
    if (test_gyro)
        pthread_create(&gyro_tid, test_detach ? &attr : NULL, gyro_thread, NULL);
    if (test_mag)
        pthread_create(&mag_tid, test_detach ? &attr : NULL, mag_thread, NULL);
    if (test_dmp)
        pthread_create(&dmp_tid, test_detach ? &attr : NULL, dmp_thread, NULL);

    pthread_cond_broadcast(&cond);

    if (!test_detach) {
        if (test_accel)
            pthread_join(accel_tid, NULL);
        if (test_gyro)
            pthread_join(gyro_tid, NULL);
        if (test_mag)
            pthread_join(mag_tid, NULL);
        if (test_dmp)
            pthread_join(dmp_tid, NULL);
    }

    wait(NULL);

    if (test_detach)
        pthread_attr_destroy(&attr);

err:
    if (fd >= 0) close(fd);
    if (mq != -1) {
        mq_close(mq);
        mq_unlink(MPU9250_MQ_NAME);
    }
    printf("MPU9250: Test completed\n");
    return 0;
}