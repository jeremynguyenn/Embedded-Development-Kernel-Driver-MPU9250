/**
 * @file      mutex.c
 * @brief     mutex source file
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

#include "mutex.h"

static volatile uint8_t gs_locked = 0;                 /**< mutex locked flag */
static volatile uint32_t gs_int_locked_cnt = 0;        /**< mutex interrupt locked counter */
static uint8_t (*gs_irq)(void) = NULL;                 /**< mutex irq */

/**
 * @brief  mutex lock
 * @return status code
 *         - 0 success
 * @note   none
 */
uint8_t mutex_lock(void)
{
    /* check if find the interrupt mutex */
    if (gs_int_locked_cnt != 0)
    {
        /* if having the irq */
        if (gs_irq != NULL)
        {
            /* run the callback */
            gs_irq();
        }
        
        /* clear the interrupt counter */
        gs_int_locked_cnt = 0;
    }
    
    /* flag locked */
    gs_locked = 1;
    
    return 0;
}

/**
 * @brief  mutex unlock
 * @return status code
 *         - 0 success
 * @note   none
 */
uint8_t mutex_unlock(void)
{
    /* check if find the interrupt mutex */
    if (gs_int_locked_cnt != 0)
    {
        /* if having the irq */
        if (gs_irq != NULL)
        {
            /* run the callback */
            gs_irq();
        }
        
        /* clear the interrupt counter */
        gs_int_locked_cnt = 0;
    }
    
    /* flag unlocked */
    gs_locked = 0;
    
    return 0;
}

/**
 * @brief     mutex irq
 * @param[in] *irq pointer to an interrupt funtion
 * @note      none
 */
void mutex_irq(uint8_t (*irq)(void))
{
    /* if not locked */
    if (gs_locked == 0)
    {
        /* if having the irq */
        if (irq != NULL)
        {
            /* run the callback */
            irq();
        }
    }
    else
    {
        /* set the irq callback */
        gs_irq = irq;
        
        /* interrupt mutex counter increment */
        gs_int_locked_cnt++;
    }
}
