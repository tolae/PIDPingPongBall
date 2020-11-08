#include "pwm.h"
#include "tim.h"

pwm_t *configured_pwm;

void _set_tim_pwm(uint32_t value);

void pwm_enable(pwm_t *pwm) {
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_1);

	pwm->enabled = 1;
	configured_pwm = pwm;
}

void pwm_disable() {
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_1);

	configured_pwm->enabled = 0;
	configured_pwm = NULL;
}

void pwm_set_prcnt(float percentage) {
	if (configured_pwm == NULL) return;

	_set_tim_pwm((configured_pwm->max - configured_pwm->min) * percentage);
	configured_pwm->pwm_prcnt = percentage;
}

void pwm_set_mapped_value(int value, int v_max, int v_min)
{
	int mapped_value = (value - v_min) * (configured_pwm->max - configured_pwm->min) / (v_max - v_min) + configured_pwm->min;
	if (mapped_value > configured_pwm->max) mapped_value = configured_pwm->max;
	else if (mapped_value < configured_pwm->min) mapped_value = configured_pwm->min;
	_set_tim_pwm(mapped_value);
	configured_pwm->pwm_value = mapped_value;
}

void pwm_set_value(unsigned int value) {
	if (configured_pwm == NULL) return;

	_set_tim_pwm(value);
	configured_pwm->pwm_value = value;
}

void _set_tim_pwm(uint32_t value) {
	htim5.Instance->CCR1 = value;
}
