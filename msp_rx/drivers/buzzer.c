#include <hal/hal_pwm.h>

void BUZ_sound_on()
{
    HAL_PWM_update_buzzer_freq(1000);
}

void BUZ_sound_off()
{
    HAL_PWM_update_buzzer_freq(0);
}
