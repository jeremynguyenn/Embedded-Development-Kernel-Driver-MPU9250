/**
 *
 * @file      mutex.h
 * @brief     mutex header file
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

#ifndef MUTEX_H
#define MUTEX_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup mutex mutex function
 * @brief    mutex function modules
 * @{
 */

/**
 * @brief  mutex lock
 * @return status code
 *         - 0 success
 * @note   none
 */
uint8_t mutex_lock(void);

/**
 * @brief  mutex unlock
 * @return status code
 *         - 0 success
 * @note   none
 */
uint8_t mutex_unlock(void);

/**
 * @brief     mutex irq
 * @param[in] *irq pointer to an interrupt funtion
 * @note      none
 */
void mutex_irq(uint8_t (*irq)(void));

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
