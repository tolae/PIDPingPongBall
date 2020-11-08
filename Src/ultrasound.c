#include "ultrasound.h"
#include "tim.h"

ultrasound_read_t *reading;
uint8_t rising_edge_detected;

void _update(uint32_t read_in);

void ultrasound_enable(ultrasound_read_t *ultrasound) {
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);

	ultrasound->enabled = 1;
	reading = ultrasound;
}

void ultrasound_disable(void) {
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2);

	reading->enabled = 0;
	reading = NULL;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// Handle echo feedback
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		uint32_t read_in = htim->Instance->CCR1;
		if (!rising_edge_detected) {
			rising_edge_detected = 1;
			htim->Instance->CNT = 0;
		}
		else {
			rising_edge_detected = 0;

			_update(read_in);

			if (reading->valid) {
				reading->ultrasound_read_callback();
			}
		}
	}
}

void _update(uint32_t read_in) {
	unsigned int dist;

	if (reading == NULL) return;

	dist = (unsigned int) ((float)read_in / 5.8f);
	reading->distance = dist;
	// Between 50mm and 1000mm
	reading->valid = dist <= ULTRASOUND_MAX_RANGE && dist >= ULTRASOUND_MIN_RANGE;
}
