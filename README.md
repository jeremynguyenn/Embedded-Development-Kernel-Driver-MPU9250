# MPU9250 Kernel Driver for Raspberry Pi 4

This project implements a Linux kernel driver and user-space test programs for the MPU9250 9-axis IMU sensor on the Raspberry Pi 4, interfacing via I2C. The driver supports both basic and advanced features, including the Industrial I/O (IIO) framework, asynchronous I/O (AIO), POSIX IPC, and enhanced reliability mechanisms. The codebase is designed for high performance, security, scalability, and reliability.

## Part 1: MPU9250 Kernel Basic

This section describes the foundational implementation of the MPU9250 kernel driver and user-space test programs, covering the initial functionality and features.

### Overview
The basic implementation includes a kernel module (`mpu9250_driver`) and a user-space test program (`user_space_test.c`) to interface with the MPU9250 sensor. It supports basic sensor operations (accelerometer, gyroscope, magnetometer, DMP) with features for process management, thread synchronization, and POSIX IPC.

### Key Features
- **Device Tree Configuration (`mpu9250.dts`)**:
  - Configures I2C1 bus and MPU9250 at address 0x68.
  - Defines GPIO17 as the interrupt pin with pull-up and falling-edge trigger.
  - Specifies default sensor settings: 1000 Hz sample rate, ±2g accel range, ±250 dps gyro range.
  - Supports AK8963 magnetometer at address 0x0C.

- **Kernel Module (`mpu9250_driver.c`, `mpu9250_driver.h`)**:
  - Implements a character device (`/dev/mpu9250`) with IOCTLs for reading/writing registers, accel/gyro/mag data, FIFO, and DMP quaternion.
  - Uses `regmap` for I2C communication with a retry mechanism.
  - Supports interrupt-driven data reading via GPIO IRQ.
  - Provides sysfs (`/sys/class/mpu9250_class`) for sensor data and configuration.
  - Implements procfs (`/proc/mpu9250`) for error and FIFO status.

- **User-Space Test (`user_space_test.c`)**:
  - Reads accel, gyro, mag, and DMP data using separate POSIX threads.
  - Logs data to a CSV file with command-line argument `--log`.
  - Supports configuration via `--sample-rate`, `--accel-range`, `--gyro-range`.
  - Forks a child process to execute `mq_test` for message queue testing.

- **Build System (`Makefile`)**:
  - Supports cross-compilation for arm64 (`ARCH=arm64`, `CROSS_COMPILE=aarch64-linux-gnu-`).
  - Builds kernel module (`mpu9250_driver.ko`) and user-space programs (`user_test`, `mq_test`).
  - Installs module and creates device node with udev rules (`/dev/mpu9250`, mode 0660, group `iio`).

### File Operations
- **Implemented in `mpu9250_driver.c`**:
  - Supports `open`, `read`, `write`, `ioctl`, `release`.
  - Added `lseek` for FIFO offset navigation.
  - Supports non-blocking I/O via `O_NONBLOCK` and `MPU9250_IOCTL_SET_NONBLOCK`.

### Memory Management
- **Dynamic Allocation**:
  - Uses `kmalloc` for small buffers (accel, gyro, mag, quaternion).
  - Uses `vmalloc` for large FIFO buffer (4KB).
  - Properly frees memory in `mpu9250_remove` to prevent leaks.

### Process Management
- **In `user_space_test.c`**:
  - Forks a child process to execute `./mq_test` for testing message queue functionality.
  - Uses `execvp` to run `mq_test` and `waitpid` to manage child process.

### Signals
- **Kernel (`mpu9250_driver.c`)**:
  - Handles `SIGUSR1` to log accel X value to kernel log.
- **User-Space (`user_space_test.c`)**:
  - Handles `SIGINT`, `SIGTERM`, `SIGUSR1` for graceful shutdown and status reporting.

### POSIX Threads
- **In `user_space_test.c`**:
  - Creates joinable threads for reading accel, gyro, mag, and DMP data.
  - Each thread operates independently, reading data at the configured sample rate.

### Thread Synchronization
- **In `user_space_test.c`**:
  - Uses POSIX `pthread_mutex_t` and `pthread_cond_t` to synchronize threads.
  - Ensures thread-safe access to shared resources (e.g., file descriptors, shared memory).

### POSIX IPC
- **Message Queue (`/mpu9250_mq`)**:
  - Used in `user_space_test.c` and `mq_test.c` for inter-process communication.
  - Sends/receives `struct mpu9250_mq_data` containing accel, gyro, mag, and quaternion data.

### Installation
1. **Prerequisites**:
   - Raspberry Pi 4 with Raspberry Pi OS (64-bit, kernel version 5.10+).
   - Install cross-compilation toolchain: `sudo apt install gcc-aarch64-linux-gnu`.
   - Ensure I2C is enabled: Add `dtparam=i2c_arm=on` in `/boot/config.txt`.
   - Install dependencies: `sudo apt install libaio-dev libpthread-stubs0-dev librt-dev`.

2. **Compile**:
   ```bash
   make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
   ```
   This builds `mpu9250_driver.ko`, `user_test`, and `mq_test`.

3. **Install**:
   ```bash
   sudo make install
   ```
   - Loads `mpu9250_driver.ko` and creates `/dev/mpu9250` with udev rules.
   - Sets permissions (mode 0660, group `iio`).

4. **Load Device Tree Overlay**:
   - Copy `mpu9250.dtbo` to `/boot/overlays/`.
   - Add `dtoverlay=mpu9250` to `/boot/config.txt`.
   - Reboot: `sudo reboot`.

### Usage
1. **Run User-Space Test**:
   ```bash
   sudo ./user_test --log mpu9250.log --sample-rate 500 --accel-range 4 --gyro-range 500
   ```
   - Logs accel, gyro, mag, and DMP data to `mpu9250.log` in CSV format.
   - Forks `mq_test` to test message queue.

2. **Run Message Queue Test**:
   ```bash
   sudo ./mq_test --log mq.log
   ```
   - Receives data from `/mpu9250_mq` and logs to `mq.log`.

3. **Check Sysfs**:
   ```bash
   cat /sys/class/mpu9250_class/data
   cat /sys/class/mpu9250_class/config
   ```

4. **Check Procfs**:
   ```bash
   cat /proc/mpu9250
   ```

### Notes
- **Hardware Setup**: Ensure MPU9250 is connected to I2C1 (GPIO2: SDA, GPIO3: SCL, GPIO17: IRQ).
- **Permissions**: Run tests as `root` or add user to `iio` group (`sudo usermod -aG iio $USER`).
- **Limitations**: The basic version lacks IIO support and advanced performance features like AIO or DMA.
- **Debugging**: Enable debug output with `echo 1 > /sys/module/mpu9250_driver/parameters/debug`.

### Example Output
Running `sudo ./user_test --log mpu9250.log` produces:
```
Accel: X=0.12 g, Y=-0.05 g, Z=1.02 g
Gyro: X=0.45 dps, Y=-0.23 dps, Z=0.10 dps
Mag: X=15.30 uT, Y=-20.45 uT, Z=10.12 uT
Quat: W=0.99, X=0.01, Y=0.00, Z=0.02
Child pid: 1234
```
Log file `mpu9250.log`:
```
0.12,-0.05,1.02,0.45,-0.23,0.10,15.30,-20.45,10.12,0.99,0.01,0.00,0.02
```

Running `sudo ./mq_test --log mq.log` produces:
```
MQ: Accel=0.12,-0.05,1.02 Gyro=0.45,-0.23,0.10 Mag=15.30,-20.45,10.12 Quat=0.99,0.01,0.00,0.02
```

### Summary of Basic Improvements
The basic implementation provides a robust foundation for MPU9250 sensor interfacing, with a character device driver, interrupt-driven data acquisition, and multi-threaded user-space testing. It includes essential features like non-blocking I/O, dynamic memory management, process forking, signal handling, thread synchronization, and POSIX message queues.

---

## Part 2: MPU9250 Kernel Advanced

This section describes the advanced features and improvements made to the MPU9250 kernel driver and user-space programs to enhance performance, security, scalability, and reliability. The codebase was extended with three additional files (`mpu9250_iio.c`, `aio_test.c`, `mq_test.c`) and updates to the existing five files (`mpu9250.dts`, `Makefile`, `mpu9250_driver.h`, `mpu9250_driver.c`, `user_space_test.c`).

### Overview
The advanced implementation builds on the basic driver by integrating the IIO framework, AIO, POSIX shared memory, semaphores, and advanced reliability mechanisms. The updates ensure seamless integration with the new files and support high-performance, secure, scalable, and reliable sensor data acquisition.

### Key Improvements
- **Device Tree (`mpu9250.dts`)**:
  - Added `enable-iio` property to enable/disable IIO driver.
  - Updated `__overrides__` to allow runtime configuration of `enable-iio`.
  - Enhanced `power-domains` for deep sleep mode compatibility with IIO.

- **Build System (`Makefile`)**:
  - Added support for building and installing `mpu9250_iio.ko`.
  - Updated udev rules for `/dev/iio:device0` (mode 0660, group `iio`, SELinux context).
  - Added `-DCONFIG_MPU9250_IIO` to `EXTRA_CFLAGS` for conditional IIO compilation.
  - Updated `clean` target to remove IIO-related files.

- **Header File (`mpu9250_driver.h`)**:
  - Added `#ifdef CONFIG_MPU9250_IIO` for conditional IIO support.
  - Introduced `struct mpu9250_aio_data` and `MPU9250_IOCTL_AIO_READ` for AIO.
  - Increased `DRIVER_VERSION` to "2.3".
  - Added sysfs attributes for IIO status.

- **Kernel Module (`mpu9250_driver.c`)**:
  - Integrated IIO support with `enable-iio` from device tree.
  - Added `MPU9250_IOCTL_AIO_READ` for asynchronous I/O.
  - Improved DMA to share resources with IIO driver.
  - Added sysfs attribute (`iio_status`) and updated procfs to show IIO buffer count.
  - Synchronized watchdog timer with IIO driver for reset handling.

- **User-Space Test (`user_space_test.c`)**:
  - Added `--use-iio` option to read from `/sys/bus/iio/devices/iio:device0`.
  - Added `--use-aio` option for AIO-based reading using `libaio`.
  - Added timestamp to CSV log for synchronization with `mq_test.c` and `aio_test.c`.
  - Improved signal handling with `SIGHUP` for config reload.
  - Ensured `mq_test` runs in the same process group.

- **New Files**:
  - **`mpu9250_iio.c`**: Implements IIO driver with buffered I/O, supporting `/sys/bus/iio` interface.
  - **`aio_test.c`**: User-space program for testing AIO with `libaio`, using shared memory and semaphores.
  - **`mq_test.c`**: User-space program for testing POSIX message queue and shared memory, synchronized with semaphores.

### Performance Enhancements
- **POSIX Shared Memory**:
  - Used `/mpu9250_shm` in `user_space_test.c`, `aio_test.c`, `mq_test.c` for efficient data sharing.
- **Poll/Select**:
  - Implemented `poll` in `mpu9250_driver.c` for non-blocking I/O, used in `user_space_test.c`.
- **Asynchronous I/O (AIO)**:
  - Supported via `MPU9250_IOCTL_AIO_READ` in `mpu9250_driver.c` and tested in `aio_test.c` with `libaio`.
- **Memory Mapping**:
  - Implemented `mmap` in `mpu9250_driver.c` for FIFO buffer access, used in `user_space_test.c`.
- **DMA**:
  - Enhanced DMA in `mpu9250_driver.c` for efficient I2C transfers, shared with `mpu9250_iio.c`.
- **Batch Processing**:
  - Enabled FIFO batch mode in `mpu9250_driver.c` with `enable-batch` configuration.
- **Dynamic Frequency Scaling**:
  - Configurable sample rate via device tree and IOCTL, supporting runtime adjustments.
- **Deep Sleep Mode**:
  - Integrated runtime PM in `mpu9250_driver.c` with `power-domains` for low-power operation.

### Security Enhancements
- **Capabilities**:
  - Enforced `CAP_SYS_RAWIO` in `mpu9250_open` to restrict device access.
- **SELinux/AppArmor**:
  - Added SELinux context in udev rules (`Makefile`) for `/dev/mpu9250` and `/dev/iio:device0`.
- **Signal Queues**:
  - Used `sigqueue` for reliable signal delivery in `user_space_test.c`, `aio_test.c`, `mq_test.c`.
- **Watchdog Timer**:
  - Enhanced watchdog in `mpu9250_driver.c` to reset sensor on excessive errors, synchronized with IIO.

### Scalability Enhancements
- **POSIX Semaphores**:
  - Used `/mpu9250_sem` in `user_space_test.c`, `aio_test.c`, `mq_test.c` for process synchronization.
- **Shared Memory**:
  - Implemented `/mpu9250_shm` for data sharing across processes.
- **Process Groups**:
  - Set process group in `user_space_test.c` and `mq_test.c` for easier management.
- **Daemon Process**:
  - Supported daemon mode in `user_space_test.c` with `fork` and `setsid`.
- **IIO Framework**:
  - Added `mpu9250_iio.c` for standard IIO interface, supporting `/sys/bus/iio`.
- **Buffered I/O**:
  - Implemented in `mpu9250_iio.c` for efficient data streaming.
- **Dynamic Configuration**:
  - Supported via sysfs, IOCTL, and command-line arguments in all user-space programs.
- **Command-Line Arguments**:
  - Added `--use-iio`, `--use-aio`, `--log`, `--sample-rate`, `--accel-range`, `--gyro-range` in `user_space_test.c`, `aio_test.c`, `mq_test.c`.
- **Logging**:
  - Enhanced CSV logging with timestamps in all user-space programs.

### Reliability Enhancements
- **Watchdog Timer**:
  - Synchronized watchdog in `mpu9250_driver.c` and `mpu9250_iio.c` for automatic resets.
- **Retry Mechanism**:
  - Implemented in `mpu9250_read` and `mpu9250_write` with 3 retries for I2C operations.
- **Tracepoints**:
  - Added `mpu9250_read` and `mpu9250_error` tracepoints for debugging.
- **Procfs**:
  - Updated `/proc/mpu9250` to show error count, FIFO length, IRQ flags, and IIO status.
- **Sysfs**:
  - Added `iio_status` attribute to show IIO buffer count and error count.
- **Process Groups**:
  - Ensured consistent process group management for reliable process termination.
- **Signal Handlers**:
  - Enhanced signal handling (`SIGINT`, `SIGTERM`, `SIGUSR1`, `SIGUSR2`, `SIGHUP`) for graceful shutdown and config reload.
- **Dynamic Configuration**:
  - Supported runtime config updates via sysfs and `SIGHUP`.

### Installation
1. **Prerequisites**:
   - Raspberry Pi 4 with Raspberry Pi OS (64-bit, kernel version 5.10+).
   - Install cross-compilation toolchain: `sudo apt install gcc-aarch64-linux-gnu`.
   - Install dependencies: `sudo apt install libaio-dev libpthread-stubs0-dev librt-dev`.
   - Enable I2C: Add `dtparam=i2c_arm=on` in `/boot/config.txt`.
   - Ensure `iio` group exists: `sudo groupadd iio`.

2. **Compile**:
   ```bash
   make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
   ```
   This builds `mpu9250_driver.ko`, `mpu9250_iio.ko`, `user_test`, `mq_test`, and `aio_test`.

3. **Install**:
   ```bash
   sudo make install
   ```
   - Loads `mpu9250_driver.ko` and `mpu9250_iio.ko`.
   - Creates `/dev/mpu9250` and `/dev/iio:device0` with udev rules (mode 0660, group `iio`, SELinux context).

4. **Load Device Tree Overlay**:
   - Copy `mpu9250.dtbo` to `/boot/overlays/`.
   - Add `dtoverlay=mpu9250` to `/boot/config.txt`.
   - Reboot: `sudo reboot`.

5. **Verify Installation**:
   ```bash
   lsmod | grep mpu9250
   ls -l /dev/mpu9250 /dev/iio:device0
   dmesg | grep mpu9250
   ```
   Expected output in `dmesg`:
   ```
   mpu9250: Probing MPU9250 at addr 0x68
   mpu9250: MPU9250 probe successful with advanced features
   ```

### Usage
1. **Run User-Space Test with IIO**:
   ```bash
   sudo ./user_test --log mpu9250.log --use-iio --sample-rate 500 --accel-range 4 --gyro-range 500
   ```
   - Reads data from `/sys/bus/iio/devices/iio:device0`.
   - Logs to `mpu9250.log` with timestamps.
   - Forks `mq_test` for message queue testing.

2. **Run AIO Test**:
   ```bash
   sudo ./aio_test --log aio.log --sample-rate 1000
   ```
   - Uses `libaio` to read data asynchronously from `/dev/mpu9250`.
   - Logs to `aio.log` with timestamps.

3. **Run Message Queue Test**:
   ```bash
   sudo ./mq_test --log mq.log
   ```
   - Receives data from `/mpu9250_mq` and `/mpu9250_shm`.
   - Logs to `mq.log` with timestamps.

4. **Check Sysfs**:
   ```bash
   cat /sys/class/mpu9250_class/data
   cat /sys/class/mpu9250_class/config
   cat /sys/class/mpu9250_class/iio_status
   ```

5. **Check IIO Interface**:
   ```bash
   cat /sys/bus/iio/devices/iio:device0/in_accel_x_raw
   cat /sys/bus/iio/devices/iio:device0/in_anglvel_x_raw
   ```

6. **Reload Configuration**:
   ```bash
   killall -SIGHUP user_test
   ```
   - Reloads configuration from `/sys/class/mpu9250_class/config`.

7. **Unload**:
   ```bash
   sudo make uninstall
   ```
   - Removes modules and device nodes.

### Notes
- **Hardware Requirements**: MPU9250 connected to I2C1 (GPIO2: SDA, GPIO3: SCL, GPIO17: IRQ, 3.3V power).
- **Permissions**: Run tests as `root` or add user to `iio` group (`sudo usermod -aG iio $USER`).
- **IIO Support**: Enable `enable-iio` in `mpu9250.dts` or via bootargs (`mpu9250.enable_iio=1`).
- **AIO Limitations**: Requires `libaio` and kernel AIO support; non-blocking mode must be enabled.
- **Debugging**:
  - Enable debug: `echo 1 > /sys/module/mpu9250_driver/parameters/debug`.
  - Check tracepoints: `cat /sys/kernel/debug/tracing/events/mpu9250/*/format`.
  - Monitor errors: `cat /proc/mpu9250`.
- **Performance Tuning**: Adjust sample rate and ranges via command-line arguments or sysfs for optimal performance.
- **Power Management**: Enable deep sleep in device tree for low-power applications.
- **SELinux/AppArmor**: Ensure SELinux context is compatible with your system policy.

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
   $ cat /sys/class/mpu9250_class/data
   Accel: 0.12 -0.05 1.02
   Gyro: 0.45 -0.23 0.10
   Mag: 15.30 -20.45 10.12
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

### Conclusion
The advanced MPU9250 driver significantly enhances the basic implementation by integrating IIO, AIO, POSIX IPC, and robust reliability mechanisms. The updates to the five existing files and the addition of three new files ensure high performance, strong security, excellent scalability, and reliable operation, making the driver suitable for production-grade sensor applications on Raspberry Pi 4.