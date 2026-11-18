#include "lvgl_port.h"

#include "lvgl.h"

void LVGL_PortInit(void)
{
    lv_init();
}

void LVGL_PortTickInc(uint32_t elapsed_ms)
{
    lv_tick_inc(elapsed_ms);
}

void LVGL_PortHandler(void)
{
    (void)lv_timer_handler();
}
