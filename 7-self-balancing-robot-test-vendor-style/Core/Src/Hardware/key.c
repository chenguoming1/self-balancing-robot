#include "key.h"

/* KEY and USB_PLUGIN GPIO already initialized by MX_GPIO_Init() in main.c */
void KEY_Init(void)
{
    /* GPIO configured by CubeIDE MX_GPIO_Init - nothing extra needed */
}

u8 KEY_Press(int Ticks)
{
    static int Tick = 0;
    if (!KEY) {
        if (++Tick >= Ticks) {
            Tick = 0;
            return 1;
        }
        return 0;
    }
    Tick = 0;
    return 0;
}
