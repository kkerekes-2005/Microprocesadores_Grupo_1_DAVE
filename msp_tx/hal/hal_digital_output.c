#include <msp430.h>
#include <driverlib.h>
#include <hal/hal_digital_output.h>


void HAL_DIGOUT_init_gpio()
{
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6); // EDUMKII-R
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5); // EDUMKII-G
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN4); // EDUMKII-B
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0); // LAUNCHPAD-LED1
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1); // LAUNCHPAD-LED2
}

void HAL_DIGOUT_config()
{
    HAL_DIGOUT_set_R(false);
    HAL_DIGOUT_set_G(false);
    HAL_DIGOUT_set_B(false);
    HAL_DIGOUT_set_LED1(false);
    HAL_DIGOUT_set_LED2(false);
}

void HAL_DIGOUT_set_R(bool status)
{
    if (status)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
    }else{
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    }
}
void HAL_DIGOUT_set_G(bool status)
{
    if (status)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
    }else{
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    }
}
void HAL_DIGOUT_set_B(bool status)
{
    if (status)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4);
    }else{
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN4);
    }
}

void HAL_DIGOUT_set_LED1(bool status)
{
    if (status)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }else{
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
}

void HAL_DIGOUT_set_LED2(bool status)
{
    if (status)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);
    }else{
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1);
    }
}
