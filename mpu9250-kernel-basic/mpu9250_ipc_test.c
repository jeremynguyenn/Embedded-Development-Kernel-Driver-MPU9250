// mpu9250_ipc_test.c
// User-space test for MPU9250 kernel driver using POSIX message queues, shared memory, and FIFO.
// Supports command-line args to select test type (--mq, --shm, or --fifo) and queue size.
// Compile: gcc -o mpu9250_ipc_test mpu9250_ipc_test.c -lrt -pthread -Wall -Wextra -std=c99
// Run: sudo ./mpu9250_ipc_test [--mq | --shm | --fifo] [--queue-size SIZE]

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <semaphore.h>
#include <mqueue.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include <time.h>
#include "mpu9250.h"

#define SHM_NAME "/mpu9250_shm"
#define SEM_NAME "/mpu9250_sem"
#define FIFO_NAME "/tmp/mpu9250_fifo"

static volatile int running = 1;
static long queue_size = 10;

static void signal_handler(int sig)
{
    running = 0;
}

// Demo default signal handler: By default, SIGINT terminates the process without custom handler.
// Here, we override it, but to demo default, uncomment signal(SIGINT, SIG_DFL); in main.
static void default_signal_demo() {
    // Default handler for SIGINT would call exit(0) implicitly.
    printf("Demo: Default SIGINT handler would terminate process.\n");
    signal(SIGINT, SIG_DFL); // New: Actually set to default for demo
}

static int run_mq_test(void)
{
    mqd_t mq;
    struct mpu9250_mq_data *data = malloc(sizeof(struct mpu9250_mq_data)); // New: malloc demo
    if (!data) return -ENOMEM;
    struct mq_attr attr;

    attr.mq_maxmsg = queue_size;
    attr.mq_msgsize = sizeof(struct mpu9250_mq_data);
    mq = mq_open(MPU9250_MQ_NAME, O_RDONLY | O_CREAT, 0666, &attr);
    if (mq == -1) {
        fprintf(stderr, "Open message queue failed: %s\n", strerror(errno));
        free(data);
        return 1;
    }

    // Check for queue overflow
    if (mq_getattr(mq, &attr) == 0 && attr.mq_curmsgs >= attr.mq_maxmsg) {
        fprintf(stderr, "Message queue overflow detected\n");
        mq_close(mq);
        mq_unlink(MPU9250_MQ_NAME);
        free(data);
        return 1;
    }

    while (running) {
        if (mq_receive(mq, (char *)data, sizeof(*data), NULL) == -1) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "Message queue receive failed: %s\n", strerror(errno));
            break;
        }
        printf("MQ Data: Accel: %.2f %.2f %.2f, Gyro: %.2f %.2f %.2f, Mag: %.2f %.2f %.2f, Quat: %.2f %.2f %.2f %.2f\n",
               data->accel[0], data->accel[1], data->accel[2],
               data->gyro[0], data->gyro[1], data->gyro[2],
               data->mag[0], data->mag[1], data->mag[2],
               data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
        usleep(100000);
    }
    mq_close(mq);
    mq_unlink(MPU9250_MQ_NAME);
    free(data); // New: free demo
    return 0;
}

static int run_shm_test(void)
{
    int shm_fd = shm_open(SHM_NAME, O_RDONLY | O_CREAT, 0666);
    if (shm_fd == -1) {
        fprintf(stderr, "Open shared memory failed: %s\n", strerror(errno));
        return 1;
    }
    ftruncate(shm_fd, sizeof(struct mpu9250_mq_data)); // Size for data
    struct mpu9250_mq_data *shm_data = mmap(NULL, sizeof(struct mpu9250_mq_data), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm_data == MAP_FAILED) {
        fprintf(stderr, "mmap failed: %s\n", strerror(errno));
        close(shm_fd);
        shm_unlink(SHM_NAME);
        return 1;
    }

    sem_t *sem = sem_open(SEM_NAME, O_CREAT, 0666, 0);
    if (sem == SEM_UNDEFINED) {
        fprintf(stderr, "Open semaphore failed: %s\n", strerror(errno));
        munmap(shm_data, sizeof(struct mpu9250_mq_data));
        close(shm_fd);
        shm_unlink(SHM_NAME);
        return 1;
    }

    struct timespec ts;
    while (running) {
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1; // Timeout 1s
        if (sem_timedwait(sem, &ts) == -1) {
            if (errno == ETIMEDOUT) continue;
            fprintf(stderr, "sem_timedwait failed: %s\n", strerror(errno));
            break;
        }
        printf("SHM Data: Accel: %.2f %.2f %.2f, Gyro: %.2f %.2f %.2f, Mag: %.2f %.2f %.2f, Quat: %.2f %.2f %.2f %.2f\n",
               shm_data->accel[0], shm_data->accel[1], shm_data->accel[2],
               shm_data->gyro[0], shm_data->gyro[1], shm_data->gyro[2],
               shm_data->mag[0], shm_data->mag[1], shm_data->mag[2],
               shm_data->quat[0], shm_data->quat[1], shm_data->quat[2], shm_data->quat[3]);
    }
    sem_close(sem);
    sem_unlink(SEM_NAME);
    munmap(shm_data, sizeof(struct mpu9250_mq_data));
    close(shm_fd);
    shm_unlink(SHM_NAME);
    return 0;
}

static int run_fifo_test(void)
{
    int fd = open(FIFO_NAME, O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        if (mkfifo(FIFO_NAME, 0666) == -1) {
            fprintf(stderr, "mkfifo failed: %s\n", strerror(errno));
            return 1;
        }
        fd = open(FIFO_NAME, O_RDONLY | O_NONBLOCK);
        if (fd == -1) {
            fprintf(stderr, "Open FIFO failed: %s\n", strerror(errno));
            unlink(FIFO_NAME);
            return 1;
        }
    }
    struct mpu9250_mq_data data;
    while (running) {
        ssize_t bytes = read(fd, &data, sizeof(data));
        if (bytes == -1) {
            if (errno == EAGAIN) {
                usleep(100000);
                continue;
            }
            fprintf(stderr, "FIFO read failed: %s\n", strerror(errno));
            break;
        } else if (bytes == 0) continue;
        printf("FIFO Data: Accel: %.2f %.2f %.2f, Gyro: %.2f %.2f %.2f, Mag: %.2f %.2f %.2f, Quat: %.2f %.2f %.2f %.2f\n",
               data.accel[0], data.accel[1], data.accel[2],
               data.gyro[0], data.gyro[1], data.gyro[2],
               data.mag[0], data.mag[1], data.mag[2],
               data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
    }
    close(fd);
    unlink(FIFO_NAME);
    return 0;
}

static int run_pipe_test(void)
{
    int pipe_fd[2];
    if (pipe(pipe_fd) == -1) {
        fprintf(stderr, "Pipe creation failed: %s\n", strerror(errno));
        return 1;
    }
    pid_t pid = fork();
    if (pid == -1) {
        fprintf(stderr, "Fork failed: %s\n", strerror(errno));
        return 1;
    }
    struct mpu9250_mq_data data;
    if (pid == 0) { // Child: Reader
        close(pipe_fd[1]); // Close write end
        while (running) {
            if (read(pipe_fd[0], &data, sizeof(data)) != sizeof(data)) {
                if (errno == EINTR) continue;
                fprintf(stderr, "Pipe read failed: %s\n", strerror(errno));
                break;
            }
            printf("Pipe Data: Accel: %.2f %.2f %.2f, Gyro: %.2f %.2f %.2f, Mag: %.2f %.2f %.2f, Quat: %.2f %.2f %.2f %.2f\n",
                   data.accel[0], data.accel[1], data.accel[2],
                   data.gyro[0], data.gyro[1], data.gyro[2],
                   data.mag[0], data.mag[1], data.mag[2],
                   data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
            usleep(100000);
        }
        close(pipe_fd[0]);
        printf("Pipe Test (Reader): Completed\n");
        exit(0);
    } else { // Parent: Writer (integrate with real driver read)
        close(pipe_fd[0]); // Close read end
        int dev_fd = open("/dev/mpu9250", O_RDONLY); // New: Real integration
        if (dev_fd < 0) {
            perror("Open /dev/mpu9250 failed");
            wait(NULL);
            return 1;
        }
        // Simulate writing data from driver
        while (running) {
            struct mpu9250_sensor_data sensor;
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_ACCEL, &sensor) < 0) { // Example read accel
                perror("IOCTL read failed");
                break;
            }
            // Fill data from sensor
            memcpy(data.accel, sensor.values, sizeof(float)*3);
            // ... Fill gyro/mag/quat similarly
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_GYRO, &sensor) < 0) break;
            memcpy(data.gyro, sensor.values, sizeof(float)*3);
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_MAG, &sensor) < 0) break;
            memcpy(data.mag, sensor.values, sizeof(float)*3);
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_DMP, &sensor) < 0) break;
            memcpy(data.quat, sensor.values, sizeof(float)*4);
            if (write(pipe_fd[1], &data, sizeof(data)) != sizeof(data)) {
                fprintf(stderr, "Pipe write failed: %s\n", strerror(errno));
                break;
            }
            usleep(100000);
        }
        close(pipe_fd[1]);
        close(dev_fd);
        wait(NULL); // Wait for child
        printf("Pipe Test (Writer): Completed\n");
        return 0;
    }
}

int main(int argc, char **argv)
{
    int test_mq = 0, test_shm = 0, test_fifo = 0, test_pipe = 0;
    int opt;

    static struct option long_options[] = {
        {"mq", no_argument, &test_mq, 1},
        {"shm", no_argument, &test_shm, 1},
        {"fifo", no_argument, &test_fifo, 1},
        {"pipe", no_argument, &test_pipe, 1},
        {"queue-size", required_argument, 0, 'q'},
        {0, 0, 0, 0}
    };
    int opt_index;
    while (getopt_long(argc, argv, "", long_options, &opt_index) != -1) {
        switch (opt_index) {
            case 4: // queue-size
                queue_size = atol(optarg);
                if (queue_size < 1 || queue_size > 100) {
                    fprintf(stderr, "Invalid queue size (1-100)\n");
                    return 1;
                }
                break;
            default:
                printf("Usage: %s [--mq | --shm | --fifo | --pipe] [--queue-size SIZE]\n", argv[0]);
                return 1;
        }
    }
    if (!test_mq && !test_shm && !test_fifo && !test_pipe) {
        printf("Please specify --mq, --shm, --fifo, or --pipe\n");
        return 1;
    }

    default_signal_demo();

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGSEGV, signal_handler); // New: Handle more signals

    if (test_mq)
        return run_mq_test();
    else if (test_shm)
        return run_shm_test();
    else if (test_fifo)
        return run_fifo_test();
    else
        return run_pipe_test();
}