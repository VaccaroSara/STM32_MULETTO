#include <DC_motor.h>

float DegreeSec2RPM(float speed_degsec){
	float speed_rpm = speed_degsec * 60/360;
	return speed_rpm;
}

float Voltage2Duty(float u){

	float duty = 100*u/V_MAX;

	if(duty>100){
		duty=100;
	} else if(duty<0){
		duty = 0;
	}

	return duty;
}

uint8_t Ref2Direction(float y_ref){
	uint8_t dir;

	if(y_ref>=0){
		dir = 0;
	} else {
		dir = 1;
	}
	return dir;
}

void set_PWM_and_dir(uint32_t duty, uint8_t dir)
{
    if(duty > 100)
        duty = 100;

    TIM3->CCR1 = (duty * (TIM3->ARR + 1)) / 100;

    if(dir == 0)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
    }
}
