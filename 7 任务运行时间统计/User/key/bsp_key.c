#include "bsp_key.h"

/**
 * @brief ∞¥º¸…®√Ë∫Ø ˝
 */
uint8_t key_scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t key_status;

    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == KEY_ON)
    {
        // À… ÷ºÏ≤‚
        while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == KEY_ON)
            ;
        key_status = KEY_ON;
    }
    else
    {
        key_status = KEY_OFF;
    }
    return key_status;
}
