#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Board key identifiers.
 */
typedef enum
{
    BSP_KEY_TOP_LEFT = 0,
    BSP_KEY_TOP_RIGHT,
    BSP_KEY_BOTTOM_LEFT,
    BSP_KEY_BOTTOM_RIGHT,
    BSP_KEY_COUNT,
} BspKeyId;

#define BSP_KEY_MASK(key)  ((uint8_t)(1U << (uint8_t)(key)))
#define BSP_KEY_MASK_ALL   ((uint8_t)((1U << (uint8_t)BSP_KEY_COUNT) - 1U))

/**
 * @brief Initialize all configured key GPIOs.
 * @warning PA15, PB3, and PB4 require JTAG disable. SWD remains enabled.
 */
void BSP_Key_Init(void);

/**
 * @brief Read one key active state.
 * @param key Key identifier.
 * @return True if the key is currently pressed.
 */
bool BSP_Key_IsPressed(BspKeyId key);

#ifdef __cplusplus
}
#endif

#endif /* BSP_KEY_H */
