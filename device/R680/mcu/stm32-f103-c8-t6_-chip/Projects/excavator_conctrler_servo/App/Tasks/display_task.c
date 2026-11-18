#include "display_task.h"

#include "bsp_st7789.h"
#include "bsp_st7789_color.h"
#include "bsp_st7789_font.h"
#include "excavator_protocol.h"
#include "input_adc_task.h"
#include "key_task.h"
#include "nrf_control_tx_task.h"
#include "wireless_task.h"

#define DISPLAY_TEXT          "LUXI CONFIG2"
#define DISPLAY_TITLE_SCALE   2U
#define DISPLAY_VALUE_SCALE   2U
#define DISPLAY_ROW_HEIGHT    34U
#define DISPLAY_FIRST_ROW_Y   54U
#define DISPLAY_LEFT_COL_X    16U
#define DISPLAY_RIGHT_COL_X   150U
#define DISPLAY_EXCAVATOR_X   90
#define DISPLAY_EXCAVATOR_Y   136
#define DISPLAY_STATUS_Y      172U
#define DISPLAY_KEY_Y         210U
#define DISPLAY_REFRESH_PERIOD_MS 100U

static void Display_DelayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static int16_t Display_MinInt16(int16_t a, int16_t b)
{
    return (a < b) ? a : b;
}

static int16_t Display_MaxInt16(int16_t a, int16_t b)
{
    return (a > b) ? a : b;
}

static void Display_FillClipped(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    const int16_t screen_width = (int16_t)BSP_ST7789_GetWidth();
    const int16_t screen_height = (int16_t)BSP_ST7789_GetHeight();

    if (x1 < 0 || y1 < 0 || x0 >= screen_width || y0 >= screen_height || x1 < x0 || y1 < y0)
    {
        return;
    }

    x0 = Display_MaxInt16(x0, 0);
    y0 = Display_MaxInt16(y0, 0);
    x1 = Display_MinInt16(x1, (int16_t)(screen_width - 1));
    y1 = Display_MinInt16(y1, (int16_t)(screen_height - 1));

    BSP_ST7789_Fill((uint16_t)x0, (uint16_t)y0, (uint16_t)x1, (uint16_t)y1, color);
}

static void Display_DrawColoredText(int16_t x, int16_t y, uint8_t scale)
{
    static const uint16_t colors[] = {
        BSP_ST7789_COLOR_RED,
        BSP_ST7789_COLOR_ORANGE,
        BSP_ST7789_COLOR_YELLOW,
        BSP_ST7789_COLOR_GREEN,
        BSP_ST7789_COLOR_CYAN,
        BSP_ST7789_COLOR_BLUE,
        BSP_ST7789_COLOR_PURPLE,
        BSP_ST7789_COLOR_PINK,
    };
    const char *text = DISPLAY_TEXT;
    uint32_t index = 0U;
    const int16_t step = (int16_t)((BSP_ST7789_FONT5X7_WIDTH + BSP_ST7789_FONT5X7_SPACING) * scale);

    while (*text != '\0')
    {
        BSP_ST7789_DrawChar5x7((int16_t)(x + (int16_t)(index * step)),
                               y,
                               *text,
                               colors[index % (sizeof(colors) / sizeof(colors[0]))],
                               scale);
        text++;
        index++;
    }
}

static void Display_FormatValue(char *buffer, const char *label, uint16_t value)
{
    buffer[0] = label[0];
    buffer[1] = label[1];
    buffer[2] = ':';
    buffer[3] = (char)('0' + ((value / 1000U) % 10U));
    buffer[4] = (char)('0' + ((value / 100U) % 10U));
    buffer[5] = (char)('0' + ((value / 10U) % 10U));
    buffer[6] = (char)('0' + (value % 10U));
    buffer[7] = '\0';
}

static void Display_DrawValue(uint16_t x, uint16_t y, const char *label, uint16_t value)
{
    char text[8];

    Display_FormatValue(text, label, value);
    Display_FillClipped((int16_t)x,
                        (int16_t)y,
                        (int16_t)(x + 118U),
                        (int16_t)(y + (BSP_ST7789_FONT5X7_HEIGHT * DISPLAY_VALUE_SCALE) + 4U),
                        BSP_ST7789_COLOR_BLACK);
    BSP_ST7789_DrawString5x7((int16_t)x, (int16_t)y, text, BSP_ST7789_COLOR_WHITE, DISPLAY_VALUE_SCALE);
}

static void Display_DrawTitle(uint16_t screen_width)
{
    const uint16_t title_width = BSP_ST7789_TextWidth5x7(DISPLAY_TEXT, DISPLAY_TITLE_SCALE);
    const int16_t title_x = (int16_t)((screen_width - title_width) / 2U);

    Display_DrawColoredText(title_x, 12, DISPLAY_TITLE_SCALE);
    BSP_ST7789_Fill(8U, 40U, (uint16_t)(screen_width - 9U), 41U, BSP_ST7789_COLOR_GRAY);
}

static void Display_DrawSample(const InputAdcSample *sample)
{
    Display_DrawValue(DISPLAY_LEFT_COL_X, DISPLAY_FIRST_ROW_Y, "LY", sample->left_y);
    Display_DrawValue(DISPLAY_RIGHT_COL_X, DISPLAY_FIRST_ROW_Y, "LX", sample->left_x);
    Display_DrawValue(DISPLAY_LEFT_COL_X, (uint16_t)(DISPLAY_FIRST_ROW_Y + DISPLAY_ROW_HEIGHT), "RY", sample->right_y);
    Display_DrawValue(DISPLAY_RIGHT_COL_X, (uint16_t)(DISPLAY_FIRST_ROW_Y + DISPLAY_ROW_HEIGHT), "RX", sample->right_x);
    Display_DrawValue(DISPLAY_LEFT_COL_X, (uint16_t)(DISPLAY_FIRST_ROW_Y + (DISPLAY_ROW_HEIGHT * 2U)), "WL", sample->wheel_left);
    Display_DrawValue(DISPLAY_RIGHT_COL_X, (uint16_t)(DISPLAY_FIRST_ROW_Y + (DISPLAY_ROW_HEIGHT * 2U)), "WR", sample->wheel_right);
}

static void Display_DrawBlock(int16_t x, int16_t y, int16_t width, int16_t height, uint16_t color)
{
    Display_FillClipped(x, y, (int16_t)(x + width - 1), (int16_t)(y + height - 1), color);
}

static void Display_DrawExcavatorIcon(void)
{
    const int16_t x = DISPLAY_EXCAVATOR_X;
    const int16_t y = DISPLAY_EXCAVATOR_Y;

    Display_DrawBlock((int16_t)(x + 0), (int16_t)(y + 28), 82, 8, BSP_ST7789_COLOR_GRAY);
    Display_DrawBlock((int16_t)(x + 8), (int16_t)(y + 30), 10, 4, BSP_ST7789_COLOR_BLACK);
    Display_DrawBlock((int16_t)(x + 30), (int16_t)(y + 30), 10, 4, BSP_ST7789_COLOR_BLACK);
    Display_DrawBlock((int16_t)(x + 52), (int16_t)(y + 30), 10, 4, BSP_ST7789_COLOR_BLACK);
    Display_DrawBlock((int16_t)(x + 4), (int16_t)(y + 24), 56, 5, BSP_ST7789_COLOR_ORANGE);

    Display_DrawBlock((int16_t)(x + 22), (int16_t)(y + 10), 30, 18, BSP_ST7789_COLOR_YELLOW);
    Display_DrawBlock((int16_t)(x + 42), (int16_t)(y + 13), 16, 15, BSP_ST7789_COLOR_ORANGE);
    Display_DrawBlock((int16_t)(x + 27), (int16_t)(y + 14), 10, 7, BSP_ST7789_COLOR_CYAN);

    Display_DrawBlock((int16_t)(x + 52), (int16_t)(y + 12), 26, 5, BSP_ST7789_COLOR_YELLOW);
    Display_DrawBlock((int16_t)(x + 73), (int16_t)(y + 17), 5, 15, BSP_ST7789_COLOR_YELLOW);
    Display_DrawBlock((int16_t)(x + 78), (int16_t)(y + 29), 11, 5, BSP_ST7789_COLOR_ORANGE);

    Display_DrawBlock((int16_t)(x + 13), (int16_t)(y + 3), 17, 4, BSP_ST7789_COLOR_ORANGE);
    Display_DrawBlock((int16_t)(x + 9), (int16_t)(y + 7), 4, 10, BSP_ST7789_COLOR_ORANGE);
}

static const char *Display_WirelessStatusText(WirelessStatusCode code)
{
    switch (code)
    {
    case WIRELESS_STATUS_OK:
        return "NRF OK";
    case WIRELESS_STATUS_NOT_FOUND:
        return "NRF MISS";
    case WIRELESS_STATUS_CONFIG_ERROR:
        return "NRF CFG";
    case WIRELESS_STATUS_TX_FAILED:
        return "NRF TXF";
    case WIRELESS_STATUS_CONNECTED:
        return "LINK OK";
    case WIRELESS_STATUS_CONNECTION_ERROR:
        return "LINK ERR";
    case WIRELESS_STATUS_BOOT:
    default:
        return "NRF BOOT";
    }
}

static void Display_FormatHex8(char *buffer, uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    buffer[0] = hex[(value >> 4U) & 0x0FU];
    buffer[1] = hex[value & 0x0FU];
    buffer[2] = '\0';
}

static void Display_DrawWirelessStatus(const WirelessStatus *status)
{
    char status_reg_text[3];
    char channel_text[3];
    uint16_t status_color = BSP_ST7789_COLOR_GREEN;

    Display_FormatHex8(status_reg_text, status->status_reg);
    channel_text[0] = (char)('0' + ((status->channel / 10U) % 10U));
    channel_text[1] = (char)('0' + (status->channel % 10U));
    channel_text[2] = '\0';

    if (status->code == WIRELESS_STATUS_CONNECTION_ERROR ||
        status->code == WIRELESS_STATUS_TX_FAILED ||
        status->code == WIRELESS_STATUS_NOT_FOUND ||
        status->code == WIRELESS_STATUS_CONFIG_ERROR)
    {
        status_color = BSP_ST7789_COLOR_RED;
    }

    Display_FillClipped(0,
                        (int16_t)DISPLAY_STATUS_Y,
                        (int16_t)(BSP_ST7789_GetWidth() - 1U),
                        (int16_t)(DISPLAY_STATUS_Y + 34U),
                        BSP_ST7789_COLOR_BLACK);
    BSP_ST7789_DrawString5x7(16, DISPLAY_STATUS_Y, Display_WirelessStatusText(status->code), status_color, 2U);
    BSP_ST7789_DrawString5x7(124, DISPLAY_STATUS_Y, "CH", BSP_ST7789_COLOR_CYAN, 2U);
    BSP_ST7789_DrawString5x7(152, DISPLAY_STATUS_Y, channel_text, BSP_ST7789_COLOR_CYAN, 2U);
    BSP_ST7789_DrawString5x7(198, DISPLAY_STATUS_Y, "S", BSP_ST7789_COLOR_YELLOW, 2U);
    BSP_ST7789_DrawString5x7(216, DISPLAY_STATUS_Y, status_reg_text, BSP_ST7789_COLOR_YELLOW, 2U);
}

static void Display_DrawKeyItem(uint16_t x, const char *label, uint8_t mask, const KeyState *state)
{
    uint16_t color = BSP_ST7789_COLOR_GRAY;

    if ((state->long_press_mask & mask) != 0U)
    {
        color = BSP_ST7789_COLOR_RED;
    }
    else if ((state->pressed_mask & mask) != 0U)
    {
        color = BSP_ST7789_COLOR_GREEN;
    }

    BSP_ST7789_DrawString5x7((int16_t)x, DISPLAY_KEY_Y, label, color, 2U);
}

static void Display_DrawKeyState(const KeyState *state, uint8_t track_speed_level)
{
    (void)track_speed_level;

    Display_FillClipped(0,
                        (int16_t)DISPLAY_KEY_Y,
                        (int16_t)(BSP_ST7789_GetWidth() - 1U),
                        (int16_t)(DISPLAY_KEY_Y + 22U),
                        BSP_ST7789_COLOR_BLACK);

    Display_DrawKeyItem(26U, "TL", KEY_MASK_TOP_LEFT, state);
    Display_DrawKeyItem(86U, "TR", KEY_MASK_TOP_RIGHT, state);
    Display_DrawKeyItem(146U, "BL", KEY_MASK_BOTTOM_LEFT, state);
    Display_DrawKeyItem(206U, "BR", KEY_MASK_BOTTOM_RIGHT, state);
}

static void Display_Task(void *argument)
{
    (void)argument;
    InputAdcSample sample = {0};
    KeyState key_state = {0};
    WirelessStatus wireless_status = {0};
    QueueHandle_t input_queue;
    QueueHandle_t key_queue;
    QueueHandle_t wireless_queue;
    uint16_t screen_width;
    uint16_t screen_height;

    BSP_ST7789_SetDelayCallback(Display_DelayMs);
    BSP_ST7789_Init();
    BSP_ST7789_SetRotation(BSP_ST7789_ROTATION_90);

    screen_width = BSP_ST7789_GetWidth();
    screen_height = BSP_ST7789_GetHeight();
    input_queue = InputADC_GetQueue();
    key_queue = Key_GetStateQueue();
    wireless_queue = Wireless_GetStatusQueue();

    BSP_ST7789_Fill(0U, 0U, (uint16_t)(screen_width - 1U), (uint16_t)(screen_height - 1U), BSP_ST7789_COLOR_BLACK);
    Display_DrawTitle(screen_width);
    Display_DrawSample(&sample);
    Display_DrawExcavatorIcon();
    Display_DrawWirelessStatus(&wireless_status);
    Display_DrawKeyState(&key_state, NrfControlTx_GetTrackSpeedLevel());

    for (;;)
    {
        if (input_queue != NULL &&
            xQueuePeek(input_queue, &sample, pdMS_TO_TICKS(50U)) == pdPASS)
        {
            Display_DrawSample(&sample);
        }

        if (wireless_queue != NULL &&
            xQueueReceive(wireless_queue, &wireless_status, 0U) == pdPASS)
        {
            Display_DrawWirelessStatus(&wireless_status);
        }

        if (key_queue != NULL &&
            xQueuePeek(key_queue, &key_state, 0U) == pdPASS)
        {
            Display_DrawKeyState(&key_state, NrfControlTx_GetTrackSpeedLevel());
        }

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_PERIOD_MS));
    }
}

void Display_TaskCreate(void)
{
    (void)xTaskCreate(Display_Task,
                      DISPLAY_TASK_NAME,
                      DISPLAY_TASK_STACK_WORDS,
                      NULL,
                      DISPLAY_TASK_PRIORITY,
                      NULL);
}
