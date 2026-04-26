#include "led.h"

/* LED already initialized by MX_GPIO_Init() in main.c (PC14 output) */
void LED_Init(void)
{
    /* GPIO configured by CubeIDE MX_GPIO_Init - nothing extra needed */
}

void Led_Flash(u16 time)
{
    static int temp = 0;
    if (time == 0)
        LED = 0;
    else if (++temp == time) {
        LED = ~LED;
        temp = 0;
    }
}
