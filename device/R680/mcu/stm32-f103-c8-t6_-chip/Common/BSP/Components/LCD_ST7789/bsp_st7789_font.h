#ifndef BSP_ST7789_FONT_H
#define BSP_ST7789_FONT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_ST7789_FONT5X7_WIDTH    5U
#define BSP_ST7789_FONT5X7_HEIGHT   7U
#define BSP_ST7789_FONT5X7_SPACING  1U

/**
 * @brief Calculate the rendered width of a 5x7 text string.
 * @param text Null-terminated ASCII string.
 * @param scale Pixel scale factor.
 * @return Text width in pixels.
 */
uint16_t BSP_ST7789_TextWidth5x7(const char *text, uint8_t scale);

/**
 * @brief Draw one scaled 5x7 ASCII character.
 * @param x Left coordinate.
 * @param y Top coordinate.
 * @param ch ASCII character.
 * @param color RGB565 text color.
 * @param scale Pixel scale factor.
 */
void BSP_ST7789_DrawChar5x7(int16_t x, int16_t y, char ch, uint16_t color, uint8_t scale);

/**
 * @brief Draw a scaled 5x7 ASCII string.
 * @param x Left coordinate.
 * @param y Top coordinate.
 * @param text Null-terminated ASCII string.
 * @param color RGB565 text color.
 * @param scale Pixel scale factor.
 */
void BSP_ST7789_DrawString5x7(int16_t x, int16_t y, const char *text, uint16_t color, uint8_t scale);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ST7789_FONT_H */
