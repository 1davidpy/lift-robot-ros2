#include "bsp.h"
#include "freertos_app.h"

int main(void)
{
    BSP_Init();
    FreeRTOS_AppInit();
    FreeRTOS_AppStart();

    for (;;)
    {
    }
}
