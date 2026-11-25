#include <driverlib.h>
#include <msp430.h>
#include <hal/hal_pwm.h>

uint16_t current_tb0_period;
uint16_t current_freq;

void HAL_PWM_init_gpio()
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION); // EDUMKII-BUZZER
}

void HAL_PWM_config()
{
    current_freq = 0;
    current_tb0_period = 0;

    HAL_PWM_update_tb0();
}


void HAL_PWM_update_buzzer_freq(int freq)
{
    current_freq = freq;
    // Guard: freq = 0 means silence (the previous code divided by zero here)
    current_tb0_period = 0;
    if (current_freq != 0)
    {
        current_tb0_period = 16000000/current_freq;
    }

    HAL_PWM_update_tb0();
}

void HAL_PWM_update_tb0()
{
	// Use driverlib for buzzer PWM configuration
    Timer_B_outputPWMParam PwmBuzzConfig;
    PwmBuzzConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    PwmBuzzConfig.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    PwmBuzzConfig.timerPeriod = current_tb0_period;
    PwmBuzzConfig.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_6;
    PwmBuzzConfig.compareOutputMode = TIMER_B_OUTPUTMODE_RESET_SET;
    PwmBuzzConfig.dutyCycle = 0;
    if (current_freq != 0){
        PwmBuzzConfig.dutyCycle = current_tb0_period/2;
    }

    Timer_B_outputPWM(TIMER_B0_BASE, &PwmBuzzConfig);

}
