#include "bsp_st7789.h"

#include "bsp_pin.h"

#ifndef BSP_DISPLAY_SCL_PORT
#error "BSP_ST7789 requires project display pin macros in bsp_pin.h"
#endif

#ifndef BSP_ST7789_WIDTH
#define BSP_ST7789_WIDTH 240U
#endif

#ifndef BSP_ST7789_HEIGHT
#define BSP_ST7789_HEIGHT 280U
#endif

#ifndef BSP_ST7789_X_OFFSET
#define BSP_ST7789_X_OFFSET BSP_ST7789_X_OFFSET_DEFAULT
#endif

#ifndef BSP_ST7789_Y_OFFSET
#define BSP_ST7789_Y_OFFSET BSP_ST7789_Y_OFFSET_DEFAULT
#endif

#ifndef BSP_ST7789_SPI_DELAY_CYCLES
#define BSP_ST7789_SPI_DELAY_CYCLES 0U
#endif

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t clock;
} BspSt7789Pin;

static const BspSt7789Pin st7789_scl = {
    .port = BSP_DISPLAY_SCL_PORT,
    .pin = BSP_DISPLAY_SCL_PIN,
    .clock = BSP_DISPLAY_SCL_CLOCK,
};

static const BspSt7789Pin st7789_sda = {
    .port = BSP_DISPLAY_SDA_PORT,
    .pin = BSP_DISPLAY_SDA_PIN,
    .clock = BSP_DISPLAY_SDA_CLOCK,
};

static const BspSt7789Pin st7789_res = {
    .port = BSP_DISPLAY_RES_PORT,
    .pin = BSP_DISPLAY_RES_PIN,
    .clock = BSP_DISPLAY_RES_CLOCK,
};

static const BspSt7789Pin st7789_dc = {
    .port = BSP_DISPLAY_DC_PORT,
    .pin = BSP_DISPLAY_DC_PIN,
    .clock = BSP_DISPLAY_DC_CLOCK,
};

static const BspSt7789Pin st7789_cs = {
    .port = BSP_DISPLAY_CS_PORT,
    .pin = BSP_DISPLAY_CS_PIN,
    .clock = BSP_DISPLAY_CS_CLOCK,
};

static const BspSt7789Pin st7789_blk = {
    .port = BSP_DISPLAY_BLK_PORT,
    .pin = BSP_DISPLAY_BLK_PIN,
    .clock = BSP_DISPLAY_BLK_CLOCK,
};

static BspSt7789DelayMsCallback st7789_delay_ms_callback;
static BspSt7789Rotation st7789_rotation = BSP_ST7789_ROTATION_0;

static void st7789_pin_set(const BspSt7789Pin *pin)
{
    GPIO_SetBits(pin->port, pin->pin);
}

static void st7789_pin_reset(const BspSt7789Pin *pin)
{
    GPIO_ResetBits(pin->port, pin->pin);
}

static void st7789_delay_cycles(void)
{
#if BSP_ST7789_SPI_DELAY_CYCLES > 0
    for (volatile uint32_t i = 0; i < BSP_ST7789_SPI_DELAY_CYCLES; i++)
    {
    }
#endif
}

static void st7789_default_delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        for (volatile uint32_t j = 0; j < 8000U; j++)
        {
        }
    }
}

static void st7789_gpio_output_init(const BspSt7789Pin *pin)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB2PeriphClockCmd(pin->clock, ENABLE);

    gpio_init.GPIO_Pin = pin->pin;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(pin->port, &gpio_init);
}

static void st7789_delay_ms(uint32_t ms)
{
    if (st7789_delay_ms_callback != 0)
    {
        st7789_delay_ms_callback(ms);
    }
    else
    {
        st7789_default_delay_ms(ms);
    }
}

static void st7789_gpio_init(void)
{
    const BspSt7789Pin *pins[] = {
        &st7789_scl,
        &st7789_sda,
        &st7789_res,
        &st7789_dc,
        &st7789_cs,
        &st7789_blk,
    };

    for (uint32_t i = 0; i < (sizeof(pins) / sizeof(pins[0])); i++)
    {
        st7789_gpio_output_init(pins[i]);
    }

    st7789_pin_set(&st7789_scl);
    st7789_pin_set(&st7789_sda);
    st7789_pin_set(&st7789_res);
    st7789_pin_set(&st7789_dc);
    st7789_pin_set(&st7789_cs);
    st7789_pin_reset(&st7789_blk);
}

static void st7789_write_bus(uint8_t data)
{
    st7789_pin_reset(&st7789_cs);

    for (uint8_t i = 0; i < 8U; i++)
    {
        st7789_pin_reset(&st7789_scl);

        if ((data & 0x80U) != 0U)
        {
            st7789_pin_set(&st7789_sda);
        }
        else
        {
            st7789_pin_reset(&st7789_sda);
        }

        st7789_delay_cycles();
        st7789_pin_set(&st7789_scl);
        st7789_delay_cycles();
        data <<= 1U;
    }

    st7789_pin_set(&st7789_cs);
}

static void st7789_write_data8(uint8_t data)
{
    st7789_write_bus(data);
}

static void st7789_write_data16(uint16_t data)
{
    st7789_write_bus((uint8_t)(data >> 8U));
    st7789_write_bus((uint8_t)data);
}

static void st7789_write_cmd(uint8_t cmd)
{
    st7789_pin_reset(&st7789_dc);
    st7789_write_bus(cmd);
    st7789_pin_set(&st7789_dc);
}

static uint8_t st7789_rotation_madctl(BspSt7789Rotation rotation)
{
    switch (rotation)
    {
    case BSP_ST7789_ROTATION_180:
        return 0xC0U;
    case BSP_ST7789_ROTATION_90:
        return 0x70U;
    case BSP_ST7789_ROTATION_270:
        return 0xA0U;
    case BSP_ST7789_ROTATION_0:
    default:
        return 0x00U;
    }
}

static uint16_t st7789_x_offset(void)
{
    return (st7789_rotation == BSP_ST7789_ROTATION_90 ||
            st7789_rotation == BSP_ST7789_ROTATION_270)
               ? BSP_ST7789_Y_OFFSET
               : BSP_ST7789_X_OFFSET;
}

static uint16_t st7789_y_offset(void)
{
    return (st7789_rotation == BSP_ST7789_ROTATION_90 ||
            st7789_rotation == BSP_ST7789_ROTATION_270)
               ? BSP_ST7789_X_OFFSET
               : BSP_ST7789_Y_OFFSET;
}

void BSP_ST7789_SetDelayCallback(BspSt7789DelayMsCallback callback)
{
    st7789_delay_ms_callback = callback;
}

void BSP_ST7789_Init(void)
{
    st7789_gpio_init();

    st7789_pin_reset(&st7789_res);
    st7789_delay_ms(100U);
    st7789_pin_set(&st7789_res);
    st7789_delay_ms(100U);

    BSP_ST7789_BacklightOn();
    st7789_delay_ms(100U);

    st7789_write_cmd(0x11U);
    st7789_delay_ms(120U);

    BSP_ST7789_SetRotation(st7789_rotation);

    st7789_write_cmd(0x3AU);
    st7789_write_data8(0x05U);

    st7789_write_cmd(0xB2U);
    st7789_write_data8(0x0CU);
    st7789_write_data8(0x0CU);
    st7789_write_data8(0x00U);
    st7789_write_data8(0x33U);
    st7789_write_data8(0x33U);

    st7789_write_cmd(0xB7U);
    st7789_write_data8(0x35U);

    st7789_write_cmd(0xBBU);
    st7789_write_data8(0x32U);

    st7789_write_cmd(0xC2U);
    st7789_write_data8(0x01U);

    st7789_write_cmd(0xC3U);
    st7789_write_data8(0x15U);

    st7789_write_cmd(0xC4U);
    st7789_write_data8(0x20U);

    st7789_write_cmd(0xC6U);
    st7789_write_data8(0x0FU);

    st7789_write_cmd(0xD0U);
    st7789_write_data8(0xA4U);
    st7789_write_data8(0xA1U);

    st7789_write_cmd(0xE0U);
    st7789_write_data8(0xD0U);
    st7789_write_data8(0x08U);
    st7789_write_data8(0x0EU);
    st7789_write_data8(0x09U);
    st7789_write_data8(0x09U);
    st7789_write_data8(0x05U);
    st7789_write_data8(0x31U);
    st7789_write_data8(0x33U);
    st7789_write_data8(0x48U);
    st7789_write_data8(0x17U);
    st7789_write_data8(0x14U);
    st7789_write_data8(0x15U);
    st7789_write_data8(0x31U);
    st7789_write_data8(0x34U);

    st7789_write_cmd(0xE1U);
    st7789_write_data8(0xD0U);
    st7789_write_data8(0x08U);
    st7789_write_data8(0x0EU);
    st7789_write_data8(0x09U);
    st7789_write_data8(0x09U);
    st7789_write_data8(0x15U);
    st7789_write_data8(0x31U);
    st7789_write_data8(0x33U);
    st7789_write_data8(0x48U);
    st7789_write_data8(0x17U);
    st7789_write_data8(0x14U);
    st7789_write_data8(0x15U);
    st7789_write_data8(0x31U);
    st7789_write_data8(0x34U);

    st7789_write_cmd(0x21U);
    st7789_write_cmd(0x29U);
}

void BSP_ST7789_BacklightOn(void)
{
    st7789_pin_set(&st7789_blk);
}

void BSP_ST7789_BacklightOff(void)
{
    st7789_pin_reset(&st7789_blk);
}

void BSP_ST7789_SetRotation(BspSt7789Rotation rotation)
{
    st7789_rotation = rotation;
    st7789_write_cmd(0x36U);
    st7789_write_data8(st7789_rotation_madctl(rotation));
}

uint16_t BSP_ST7789_GetWidth(void)
{
    return (st7789_rotation == BSP_ST7789_ROTATION_90 ||
            st7789_rotation == BSP_ST7789_ROTATION_270)
               ? BSP_ST7789_HEIGHT
               : BSP_ST7789_WIDTH;
}

uint16_t BSP_ST7789_GetHeight(void)
{
    return (st7789_rotation == BSP_ST7789_ROTATION_90 ||
            st7789_rotation == BSP_ST7789_ROTATION_270)
               ? BSP_ST7789_WIDTH
               : BSP_ST7789_HEIGHT;
}

void BSP_ST7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    const uint16_t width = BSP_ST7789_GetWidth();
    const uint16_t height = BSP_ST7789_GetHeight();

    if (x0 >= width)
    {
        x0 = width - 1U;
    }
    if (x1 >= width)
    {
        x1 = width - 1U;
    }
    if (y0 >= height)
    {
        y0 = height - 1U;
    }
    if (y1 >= height)
    {
        y1 = height - 1U;
    }

    st7789_write_cmd(0x2AU);
    st7789_write_data16((uint16_t)(x0 + st7789_x_offset()));
    st7789_write_data16((uint16_t)(x1 + st7789_x_offset()));

    st7789_write_cmd(0x2BU);
    st7789_write_data16((uint16_t)(y0 + st7789_y_offset()));
    st7789_write_data16((uint16_t)(y1 + st7789_y_offset()));

    st7789_write_cmd(0x2CU);
}

void BSP_ST7789_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    uint32_t count;

    if (x1 < x0 || y1 < y0)
    {
        return;
    }

    BSP_ST7789_SetWindow(x0, y0, x1, y1);
    count = ((uint32_t)x1 - x0 + 1U) * ((uint32_t)y1 - y0 + 1U);

    for (uint32_t i = 0; i < count; i++)
    {
        st7789_write_data16(color);
    }
}

void BSP_ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= BSP_ST7789_GetWidth() || y >= BSP_ST7789_GetHeight())
    {
        return;
    }

    BSP_ST7789_SetWindow(x, y, x, y);
    st7789_write_data16(color);
}

void BSP_ST7789_WritePixels(const uint16_t *colors, uint32_t count)
{
    if (colors == 0)
    {
        return;
    }

    for (uint32_t i = 0; i < count; i++)
    {
        st7789_write_data16(colors[i]);
    }
}
