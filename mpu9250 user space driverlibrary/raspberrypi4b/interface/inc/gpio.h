/**
 *
 * @file      gpio.h
 * @brief     gpio header file
 * @version   1.0.0
 * @author    Nguyen Nhan
 * @date      2025-08-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/11/11  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#ifndef GPIO_H
#define GPIO_H

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @defgroup gpio gpio function
 * @brief    gpio function modules
 * @{
 */

/**
 * @brief  gpio interrupt init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t gpio_interrupt_init(void);

/**
 * @brief  gpio interrupt deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t gpio_interrupt_deinit(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
