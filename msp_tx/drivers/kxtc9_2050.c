#include <hal/hal_adc.h>


#define SENSITIVITY 660.0
float KXTC9_get_accel_x()
{
    float acc_x_g = (acc_x_raw*3300.0/4095.0 - 1650.0)/SENSITIVITY;
    return acc_x_g;
}

float KXTC9_get_accel_y()
{
    float acc_y_g = (acc_y_raw*3300.0/4095.0 - 1650.0)/SENSITIVITY;
    return acc_y_g;
}
float KXTC9_get_accel_z()
{
    float acc_z_g = (acc_z_raw*3300.0/4095.0 - 1650.0)/SENSITIVITY;
    return acc_z_g;
}

