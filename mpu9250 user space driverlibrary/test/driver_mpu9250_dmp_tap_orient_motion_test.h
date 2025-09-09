/**
 *
 * @file      driver_mpu9250_dmp_tap_orient_motion_test.h
 * @brief     driver mpu9250 dmp tap orient motion test header file
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

#ifndef DRIVER_MPU9250_DMP_TAP_ORIENT_MOTION_TEST_H
#define DRIVER_MPU9250_DMP_TAP_ORIENT_MOTION_TEST_H

#include "driver_mpu9250_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @addtogroup mpu9250_test_driver
 * @{
 */

/**
 * @brief  dmp tap orient motion test irq
 * @return status code
 *         - 0 success
 *         - 1 run failed
 * @note   none
 */
uint8_t mpu9250_dmp_tap_orient_motion_test_irq_handler(void);

/**
 * @brief     dmp test
 * @param[in] interface used interface
 * @param[in] addr iic device address
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      don't support spi interface
 */
uint8_t mpu9250_dmp_tap_orient_motion_test(mpu9250_interface_t interface, mpu9250_address_t addr);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
