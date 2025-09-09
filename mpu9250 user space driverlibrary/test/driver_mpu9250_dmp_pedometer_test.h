/**
 *
 * @file      driver_mpu9250_dmp_pedometer_test.h
 * @brief     driver mpu9250 dmp pedometer test header file
 * @version   1.0.0
 * @author    Nguyen Nhan
 * @date      2025-08-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/08/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#ifndef DRIVER_MPU9250_DMP_PEDOMETER_TEST_H
#define DRIVER_MPU9250_DMP_PEDOMETER_TEST_H

#include "driver_mpu9250_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @addtogroup mpu9250_test_driver
 * @{
 */

/**
 * @brief  dmp pedometer test irq
 * @return status code
 *         - 0 success
 *         - 1 run failed
 * @note   none
 */
uint8_t mpu9250_dmp_pedometer_test_irq_handler(void);

/**
 * @brief     dmp pedometer test
 * @param[in] interface used interface
 * @param[in] addr iic device address
 * @param[in] times test times
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      don't support spi interface
 */
uint8_t mpu9250_dmp_pedometer_test(mpu9250_interface_t interface, mpu9250_address_t addr, uint32_t times);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
