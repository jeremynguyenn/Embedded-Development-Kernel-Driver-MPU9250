# MPU9250 Kernel Driver for Raspberry Pi 4
<img width="450" height="450" alt="image" src="https://github.com/user-attachments/assets/20eae71d-d3d9-429f-9ce2-44ac9b293618" /> <img width="450" height="450" alt="image" src="https://github.com/user-attachments/assets/7d9f4614-2586-400c-8892-bd5b0f2e63a9" />

This project implements a Linux kernel driver and user-space test programs for the MPU9250 9-axis IMU sensor on the Raspberry Pi 4, interfacing via I2C. The driver supports basic sensor operations, including accelerometer, gyroscope, magnetometer, FIFO, and DMP, with features for process management, signals, POSIX threads, thread synchronization, IPC, and memory management. The codebase is modular, with enhanced error handling, race condition prevention, and comments on memory layout. Advanced features include Industrial I/O (IIO) framework integration, asynchronous I/O (AIO), POSIX IPC, and robust reliability mechanisms for high performance, security, scalability, and reliability.

## Part 1: MPU9250 Kernel Basic

This section describes the foundational implementation of the MPU9250 kernel driver and user-space test programs, covering core functionality such as I2C communication, interrupt handling, file operations, and user-space testing with threads and IPC.

### Overview
The basic implementation includes a kernel module (`mpu9250_driver`) split into core driver (`mpu9250_driver.c`), operations (`mpu9250_ops.c`), and file operations (`mpu9250_fileops.c`), along with user-space test programs (`user_space_test.c` and `mpu9250_ipc_test.c`) to interface with the MPU9250 sensor. It supports sensor data reading (accelerometer, gyroscope, magnetometer, DMP quaternion) with features for process management (fork, child-parent, command-line args, memory layout), signals (handlers, sending, default handlers), POSIX threads (creation, termination, ID, joinable/detachable), thread synchronization (mutex, condition variables), and IPC (pipes, FIFO, message queues, semaphores, shared memory).
<img width="764" height="304" alt="image" src="https://github.com/user-attachments/assets/5fa6bfa0-ef26-4ba4-9010-f819f6c62212" />

### Key Features
- **Device Tree Configuration (`mpu9250.dts`)**:
  - Configures I2C1 bus at 400kHz and MPU9250 at address 0x68.
  - Defines GPIO17 as the interrupt pin with pull-up and falling-edge trigger.
  - Supports AK8963 magnetometer at address 0x0C.
  - Includes dynamic overrides for address, IRQ pin, I2C speed, accel/gyro scales, and sample rate to prevent race conditions during loading.

- **Kernel Module (`mpu9250_driver.c`, `mpu9250_ops.c`, `mpu9250_fileops.c`, `mpu9250.h`)**:
  - Implements a character device (`/dev/mpu9250`) with file operations (open, read, write, lseek, ioctl, release, poll, mmap).
  - Uses `regmap` for I2C communication with retry on failures and improved error handling.
  - Supports interrupt-driven data reading via GPIO IRQ, workqueue for sensor data processing, and spinlocks/mutex for race condition prevention.
  - Provides sysfs attributes (`/sys/class/mpu9250_class`) for reading sensor data and setting accel/gyro scales.
  - Includes kernel thread for periodic logging (process management demo).
  - Supports FIFO and DMP initialization/reading, data conversions, scale/sample rate configuration, and basic calibration with offsets.
  - Power management with runtime PM and suspend/resume.
  - Memory management with kmalloc for structs, vmalloc for large FIFO buffer, and notes on kernel memory layout (text, data, stack, heap).
  - Input subsystem (evdev) integration for reporting sensor data.

- **User-Space Test (`user_space_test.c`)**:
  - Demonstrates opening `/dev/mpu9250`, setting scales/sample rate via IOCTL, writing to wakeup register, reading WHO_AM_I.
  - Uses POSIX threads for concurrent accel/gyro/mag/DMP reading, with mutex/cond var synchronization.
  - Supports detachable threads via command-line arg.
  - Process management: fork for child process to send SIGUSR1 to parent, wait for child.
  - Signals: custom handler for SIGINT/SIGTERM/SIGUSR1, demo of default handler.
  - Command-line args for selecting tests, rates, scales.
  - Memory layout notes in comments (code, data, stack, heap).
  - Includes fork/exec demo for another program.

- **IPC Test (`mpu9250_ipc_test.c`)**:
  - Tests POSIX message queues, shared memory with semaphores, FIFO, and pipes.
  - Supports command-line args for test type and queue size.
  - In pipe test, integrates with driver by reading sensor data via IOCTL and writing to pipe.
  - Signals: handler for SIGINT/SIGTERM/SIGSEGV, demo default handler.
  - Memory management: malloc/free in MQ test, mmap/munmap in SHM.
  - Process management: fork in pipe test for reader/writer.

- **Makefile**:
  - Builds kernel module with cross-compilation support for arm64.
  - Compiles user tests with GCC flags.
  - Install/uninstall with insmod/rmmod, mknod, udev rules.
  - Checks dependencies (gcc, dtc, kernel dir), memory usage (free, vmallocinfo).
  - Generates docs with Doxygen, compiles DTS to DTBO.

### Installation
1. **Compile Device Tree Overlay**:
   ```bash
   dtc -@ -I dts -O dtb -o mpu9250.dtbo mpu9250.dts
   sudo cp mpu9250.dtbo /boot/overlays/
   ```
   Add `dtoverlay=mpu9250` to `/boot/config.txt` and reboot.

2. **Build Kernel Module**:
   ```bash
   make
   ```

3. **Install Module**:
   ```bash
   make install
   ```

4. **Compile User-Space Tests**:
   ```bash
   make user_test
   make mpu9250_ipc_test
   ```

### Usage
- **Load Module**: `sudo make install`.
- **Run Basic Test**: `sudo ./user_test [--accel] [--gyro] [--mag] [--dmp] [--detach] [--sample-rate RATE] [--accel-scale SCALE] [--gyro-scale SCALE]`.
- **Run IPC Test**: `sudo ./mpu9250_ipc_test [--mq | --shm | --fifo | --pipe] [--queue-size SIZE]`.
- **Sysfs Access**:
  ```bash
  cat /sys/class/mpu9250_class/mpu9250_data
  echo 1 > /sys/class/mpu9250_class/accel_scale  # Set to 4G
  ```
- **Unload Module**: `sudo make uninstall`.

### Example Output
1. **Running `sudo ./user_test --accel --gyro --mag --dmp`**:
   ```
   MPU9250: Wakeup written
   MPU9250: WHO_AM_I = 0x71
   Accel Thread ID: 123456789
   Gyro Thread ID: 123456790
   Mag Thread ID: 123456791
   DMP Thread ID: 123456792
   Accel: 0.12 -0.05 1.02
   Gyro: 0.45 -0.23 0.10
   Mag: 15.30 -20.45 10.12
   Quat: 0.99 0.01 0.00 0.02
   MPU9250: Received SIGUSR1, continuing
   MPU9250: Test completed
   ```

2. **Running `sudo ./mpu9250_ipc_test --mq --queue-size 20`**:
   ```
   Demo: Default SIGINT handler would terminate process.
   MQ Data: Accel: 0.12 -0.05 1.02, Gyro: 0.45 -0.23 0.10, Mag: 15.30 -20.45 10.12, Quat: 0.99 0.01 0.00 0.02
   ```

3. **Running `sudo ./mpu9250_ipc_test --pipe`**:
   ```
   Demo: Default SIGINT handler would terminate process.
   Pipe Data: Accel: 0.12 -0.05 1.02, Gyro: 0.45 -0.23 0.10, Mag: 15.30 -20.45 10.12, Quat: 0.99 0.01 0.00 0.02
   Pipe Test (Reader): Completed
   Pipe Test (Writer): Completed
   ```

4. **Sysfs Output**:
   ```bash
   $ cat /sys/class/mpu9250_class/mpu9250_data
   Accel: 0.12 -0.05 1.02
   Gyro: 0.45 -0.23 0.10
   Mag: 15.30 -20.45 10.12
   Quat: 0.99 0.01 0.00 0.02
   ```
   ```bash
   $ cat /sys/class/mpu9250_class/accel_scale
   0
   $ echo 1 > /sys/class/mpu9250_class/accel_scale
   ```

## Part 2: MPU9250 Kernel Advanced

This section describes the advanced features added to the MPU9250 kernel driver, enhancing the basic implementation with Industrial I/O (IIO) framework integration, asynchronous I/O (AIO), and additional reliability and performance optimizations.

### Overview
The advanced implementation extends the basic driver with three additional files (`iio_mpu9250.c`, `aio_mpu9250.c`, `mq_mpu9250.c`) to support IIO, AIO, and message queue-based data streaming. These enhancements provide better integration with Linux frameworks, improved performance for high-frequency data, and robust reliability for production-grade applications.

### Key Features
- **IIO Integration (`iio_mpu9250.c`)**:
  - Integrates with Linux IIO framework for standardized sensor data access.
  - Creates IIO device (`/sys/bus/iio/devices/iio:deviceX`) with channels for accel, gyro, mag, and quaternion.
  - Supports buffered mode for high-frequency data collection.
  - Provides sysfs attributes for scale, offset, and sampling frequency.
  - Implements trigger-based data capture using IIO buffers.

- **Asynchronous I/O (`aio_mpu9250.c`)**:
  - Implements AIO for non-blocking sensor data reading.
  - Uses `libaio` for user-space asynchronous operations.
  - Reduces CPU usage for high-frequency applications.
  - Supports batched data retrieval with callbacks.

- **Message Queue Streaming (`mq_mpu9250.c`)**:
  - Implements POSIX message queues for streaming sensor data to user-space.
  - Supports configurable queue sizes and overflow handling.
  - Integrates with `mpu9250_ipc_test.c` for testing.
  - Ensures low-latency data transfer with minimal overhead.

- **Enhanced Reliability**:
  - Adds retry mechanisms for I2C failures with exponential backoff.
  - Implements robust error handling with detailed logging.
  - Uses mutex and spinlocks to prevent race conditions in multi-threaded access.
  - Supports runtime power management for low-power operation.
  - Includes calibration function to store accel/gyro offsets.

- **Performance Optimizations**:
  - Optimizes I2C communication with bulk reads/writes.
  - Reduces interrupt latency with threaded IRQ handling.
  - Uses vmalloc for large FIFO buffers to prevent memory fragmentation.
  - Implements efficient data conversion for accel, gyro, mag, and quaternion.

- **Security Enhancements**:
  - Validates user inputs in IOCTL and sysfs to prevent buffer overflows.
  - Uses secure memory allocation (devm_kzalloc, vmalloc) with proper cleanup.
  - Implements proper file operation permissions and udev rules.

### Installation (Advanced Features)
1. **Build with IIO Support**:
   ```bash
   make EXTRA_CFLAGS="-DCONFIG_MPU9250_IIO"
   ```

2. **Install Module**:
   ```bash
   make install
   ```

3. **Compile Additional Tests**:
   ```bash
   gcc -o aio_test aio_mpu9250.c -laio -pthread -lrt $(CFLAGS)
   gcc -o mq_test mq_mpu9250.c -lrt -pthread $(CFLAGS)
   ```

### Usage (Advanced Features)
- **Access IIO Data**:
  ```bash
  cat /sys/bus/iio/devices/iio:device0/in_accel_x_raw
  cat /sys/bus/iio/devices/iio:device0/in_gyro_scale
  ```
- **Enable IIO Buffer**:
  ```bash
  echo 1 > /sys/bus/iio/devices/iio:device0/buffer/enable
  cat /dev/iio:device0
  ```
- **Run AIO Test**:
  ```bash
  sudo ./aio_test --log aio.log
  ```
- **Run MQ Test**:
  ```bash
  sudo ./mq_test --log mq.log
  ```
- **Monitor Errors**:
  ```bash
  cat /proc/mpu9250
  ```

### Example Output
1. **Running `sudo ./user_test --log mpu9250.log --use-iio`**:
   ```
   IIO Accel: X=0.12 g, Y=-0.05 g, Z=1.02 g
   IIO Gyro: X=0.45 dps, Y=-0.23 dps, Z=0.10 dps
   IIO Mag: X=15.30 uT, Y=-20.45 uT, Z=10.12 uT
   Quat: W=0.99, X=0.01, Y=0.00, Z=0.02
   Child pid: 1234
   ```
   Log file `mpu9250.log`:
   ```
   1725899700.123456,0.12,-0.05,1.02,0.45,-0.23,0.10,15.30,-20.45,10.12,0.99,0.01,0.00,0.02
   ```

2. **Running `sudo ./aio_test --log aio.log`**:
   ```
   AIO: Accel=0.12,0.05,1.02 Gyro=0.45,-0.23,0.10 Mag=15.30,-20.45,10.12 Quat=0.99,0.01,0.00,0.02
   ```
   Log file `aio.log`:
   ```
   1725899700.123456,0.12,-0.05,1.02,0.45,-0.23,0.10,15.30,-20.45,10.12,0.99,0.01,0.00,0.02
   ```

3. **Running `sudo ./mq_test --log mq.log`**:
   ```
   MQ: Accel=0.12,-0.05,1.02 Gyro=0.45,-0.23,0.10 Mag=15.30,-20.45,10.12 Quat=0.99,0.01,0.00,0.02
   SHM: Accel=0.12,-0.05,1.02 Gyro=0.45,-0.23,0.10 Mag=15.30,-20.45,10.12 Quat=0.99,0.01,0.00,0.02
   ```
   Log file `mq.log`:
   ```
   1725899700.123456,0.12,-0.05,1.02,0.45,-0.23,0.10,15.30,-20.45,10.12,0.99,0.01,0.00,0.02
   ```

4. **Sysfs Output**:
   ```bash
   $ cat /sys/class/mpu9250_class/mpu9250_data
   Accel: 0.12 -0.05 1.02
   Gyro: 0.45 -0.23 0.10
   Mag: 15.30 -20.45 10.12
   Quat: 0.99 0.01 0.00 0.02
   ```
   ```bash
   $ cat /sys/class/mpu9250_class/iio_status
   IIO Enabled: 1
   Buffer Count: 1234
   Errors: 0
   ```

5. **Procfs Output**:
   ```bash
   $ cat /proc/mpu9250
   Errors: 0
   FIFO Len: 128
   IRQ Flags: 0x10
   IIO Enabled: 1
   IIO Buffer Count: 1234
   ```

### Tips for Optimal Usage
- **Calibration**: Run `sudo ./user_test --calibrate` to compute accel/gyro offsets.
- **IIO Monitoring**: Use `iio_event_monitor` or `libiio` to read from `/sys/bus/iio/devices/iio:device0/events/mpu9250/*/format`.
- **Performance Tuning**: Adjust sample rate and ranges via command-line arguments or sysfs for optimal performance.
- **Power Management**: Enable deep sleep in device tree for low-power applications.
- **SELinux/AppArmor**: Ensure SELinux context is compatible with your system policy.

### Conclusion
The MPU9250 kernel driver provides a robust foundation for sensor interfacing on Raspberry Pi 4, with modular code, comprehensive error handling, and extensive user-space testing for system concepts like processes, threads, signals, synchronization, IPC, and memory management. The advanced features (IIO, AIO, POSIX IPC) ensure high performance, strong security, excellent scalability, and reliable operation, making the driver suitable for production-grade sensor applications. Updated as of September 20, 2025.