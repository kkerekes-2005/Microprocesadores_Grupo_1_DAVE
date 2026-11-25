#include <hal/hal_digital_output.h>
#include <drivers/led.h>


void LED_set_rgb(bool r,bool g,bool b)
{
    HAL_DIGOUT_set_R(r);
    HAL_DIGOUT_set_G(g);
    HAL_DIGOUT_set_B(b);
}

uint8_t cycle_status = CYCLE_OFF;
void LED_cycle_rgb()
{
    switch(cycle_status)
    {
    case CYCLE_OFF:
        cycle_status = CYCLE_RED;
        LED_set_rgb(255,0,0);
        break;
    case CYCLE_RED:
        cycle_status = CYCLE_GREEN;
        LED_set_rgb(0,255,0);
        break;
    case CYCLE_GREEN:
        cycle_status = CYCLE_BLUE;
        LED_set_rgb(0,0,255);
        break;
    case CYCLE_BLUE:
        cycle_status = CYCLE_RED;
        LED_set_rgb(255,0,0);
        break;
    }
}
