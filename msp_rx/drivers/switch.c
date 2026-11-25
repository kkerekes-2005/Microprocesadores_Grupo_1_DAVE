#include <hal/hal_digital_input.h>
#include <driverlib.h>
#include <stdbool.h>

// Software button state machine (pressed / released) with one INDEPENDENT
// debounce window per edge:
//  - Press edge: latched by the port ISR (flag), so no tap is ever lost no
//    matter how fast the main loop polls. After its debounce window every
//    bounce edge accumulated in the flag is discarded.
//  - Release edge: detected by level inside the state machine, with its own
//    debounce window that also discards the falling edges caused by the
//    release bounce - so releasing can never look like a new press.
// The interrupt is never disabled, and a press is reported exactly once per
// physical press (no auto-repeat while held).

static bool s1_pressed = false;
static bool s2_pressed = false;

bool SWITCH_get_edumkii_S1()
{
    if (!s1_pressed)
    {
        if (flag_edumkii_S1)
        {
            __delay_cycles(160000);         // press debounce: 10 ms at 16 MHz
            flag_edumkii_S1 = false;        // discard press bounce edges
            s1_pressed = true;
            return true;                    // one event per press
        }
    }
    else if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH)
    {
        __delay_cycles(160000);             // release debounce: 10 ms at 16 MHz
        flag_edumkii_S1 = false;            // discard release bounce edges
        if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH)
        {
            s1_pressed = false;             // clean release confirmed
        }
    }
    return false;
}

bool SWITCH_get_edumkii_S2()
{
    if (!s2_pressed)
    {
        if (flag_edumkii_S2)
        {
            __delay_cycles(160000);         // press debounce: 10 ms at 16 MHz
            flag_edumkii_S2 = false;        // discard press bounce edges
            s2_pressed = true;
            return true;                    // one event per press
        }
    }
    else if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN2) == GPIO_INPUT_PIN_HIGH)
    {
        __delay_cycles(160000);             // release debounce: 10 ms at 16 MHz
        flag_edumkii_S2 = false;            // discard release bounce edges
        if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN2) == GPIO_INPUT_PIN_HIGH)
        {
            s2_pressed = false;             // clean release confirmed
        }
    }
    return false;
}
