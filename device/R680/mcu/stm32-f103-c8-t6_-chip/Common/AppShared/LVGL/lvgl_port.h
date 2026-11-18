#ifndef LVGL_PORT_H
#define LVGL_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void LVGL_PortInit(void);
void LVGL_PortTickInc(uint32_t elapsed_ms);
void LVGL_PortHandler(void);

#ifdef __cplusplus
}
#endif

#endif
