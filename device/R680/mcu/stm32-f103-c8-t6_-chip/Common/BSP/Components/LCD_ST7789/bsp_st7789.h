#ifndef BSP_ST7789_H
#define BSP_ST7789_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_ST7789_WIDTH             240U
#define BSP_ST7789_HEIGHT            280U
#define BSP_ST7789_X_OFFSET_DEFAULT  0U
#define BSP_ST7789_Y_OFFSET_DEFAULT  20U

/**
 * @brief ST7789 display rotation mode.
 */
typedef enum
{
    BSP_ST7789_ROTATION_0 = 0,
    BSP_ST7789_ROTATION_180,
    BSP_ST7789_ROTATION_90,
    BSP_ST7789_ROTATION_270,
} BspSt7789Rotation;

/**
 * @brief Millisecond delay callback used by the ST7789 driver.
 * @param ms Delay time in milliseconds.
 */
typedef void (*BspSt7789DelayMsCallback)(uint32_t ms);

/**
 * @brief Register the delay callback used by LCD reset and initialization.
 * @param callback Delay callback. Pass null to use the internal blocking delay.
 */
void BSP_ST7789_SetDelayCallback(BspSt7789DelayMsCallback callback);

/**
 * @brief Initialize GPIO pins and the ST7789 controller.
 */
void BSP_ST7789_Init(void);

/**
 * @brief Turn on the LCD backlight.
 */
void BSP_ST7789_BacklightOn(void);

/**
 * @brief Turn off the LCD backlight.
 */
void BSP_ST7789_BacklightOff(void);

/**
 * @brief Set display rotation.
 * @param rotation Rotation mode.
 */
void BSP_ST7789_SetRotation(BspSt7789Rotation rotation);

/**
 * @brief Get the current logical display width.
 * @return Width in pixels after applying the current rotation.
 */
uint16_t BSP_ST7789_GetWidth(void);

/**
 * @brief Get the current logical display height.
 * @return Height in pixels after applying the current rotation.
 */
uint16_t BSP_ST7789_GetHeight(void);

/**
 * @brief Set the active GRAM write window.
 * @param x0 Start X coordinate.
 * @param y0 Start Y coordinate.
 * @param x1 End X coordinate.
 * @param y1 End Y coordinate.
 */
void BSP_ST7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/**
 * @brief Fill a rectangular area with one RGB565 color.
 * @param x0 Start X coordinate.
 * @param y0 Start Y coordinate.
 * @param x1 End X coordinate.
 * @param y1 End Y coordinate.
 * @param color RGB565 color.
 */
void BSP_ST7789_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Draw one pixel.
 * @param x X coordinate.
 * @param y Y coordinate.
 * @param color RGB565 color.
 */
void BSP_ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Write a continuous RGB565 pixel buffer to the current GRAM window.
 * @param colors Pixel buffer.
 * @param count Number of pixels to write.
 */
void BSP_ST7789_WritePixels(const uint16_t *colors, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ST7789_H */
