#include <stdio.h>
#include <string.h>
#include <math.h>
#include "my_main.h"
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "post.h"
#include "ultrasound.h"
#include "pwm.h"
#include "pid.h"

#define OUT_BUFFER_MAX (256)

/* PID Parameters */
#define KP 15.0f
#define KI 1.1f
#define KD 15000.0f
#define NUM_ERROR_TERMS 2
#define NUM_SAVED_TERMS 1
#define INTEGRATION_TERMS 2
#define DERIVATION_TERMS 1

#define PERIOD (50) // ms

uint32_t desired_distance_mm = 500; // About 2 feet

const char POST_COMPLETE[] = "Post has finished successfully!\n\r";
const char CHANGE_DESIRED_POS[] = "Enter new desired position (in mm): ";
const char CONFIRM[] = "Press enter again to confirm.";

char OUTPUT[] = "PWM: %d, Dist: %d, PID: %d, P: %d, I: %d, D: %d\n\r";
uint8_t wait_for_steady = 0;

unsigned char hold_char = 0;
uint8_t pwm = 0;
char out_buffer[OUT_BUFFER_MAX] = { 0 };
size_t buffer_size = 0;
uint8_t post_passed = 0;

uint8_t user_key_cnt = 0;

ultrasound_read_t ultrasound_handler;

pwm_t pwm_handler;

float errorTerms[NUM_ERROR_TERMS];
float savedTerms[NUM_SAVED_TERMS];
c_pid_t pid_handler =
{
	.kp = KP,
	.ki = KI,
	.kd = KD,
	.pid_p = 0,
	.pid_i = 0,
	.pid_d = 0,
	.sigma = 0,
	.fErrorTerms = &(errorTerms[0]),
	.fSavedTerms = &(savedTerms[0]),
	.numOfErrors = NUM_ERROR_TERMS,
	.numOfSaved = NUM_SAVED_TERMS,
	.integrationTerms = INTEGRATION_TERMS,
	.derivationTerms = DERIVATION_TERMS
};

void configure_ultrasound();
void configure_pwm();

void setup(void) {
	memset(errorTerms, 0, sizeof(float) * NUM_ERROR_TERMS);
	memset(savedTerms, 0, sizeof(float) * NUM_SAVED_TERMS);

	configure_ultrasound();
	configure_pwm();


	ultrasound_enable(&ultrasound_handler);
	pwm_enable(&pwm_handler);
	pwm_set_value(0);

	while(post_passed == 0) {
		post_passed = run_post();
	}
	HAL_UART_Transmit(&huart2, (uint8_t *)POST_COMPLETE, sizeof(POST_COMPLETE), 0xFFFF);

	HAL_UART_Receive_IT(&huart2, &hold_char, 1);
}

void loop(void) {
}

uint8_t stopped = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint8_t user_key = 0;
	uint8_t usr_ky_cnt_tmp = 0;
	uint32_t new_pos = 0;
	// Press any key to stop
	if (stopped == 0)
	{
		stopped = 1;
		pwm_set_value(0);
		pwm_disable();
		ultrasound_disable();
		HAL_UART_Transmit(&huart2, (uint8_t *)CHANGE_DESIRED_POS, sizeof(CHANGE_DESIRED_POS), 0xFFFF);
		do
		{
			HAL_UART_Receive(&huart2, &user_key, 1, 0xFFFF);
			HAL_UART_Transmit(&huart2, &user_key, 1, 0xFFFF);
			out_buffer[user_key_cnt++] = user_key;
		} while (user_key != '\r');
		HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, 0xFFFF);
		usr_ky_cnt_tmp = --user_key_cnt;
		for (int i = 0; i < usr_ky_cnt_tmp; i++)
		{
			new_pos += (out_buffer[--user_key_cnt] - '0') * pow(10, i);
		}
		desired_distance_mm = new_pos;
		HAL_UART_Transmit(&huart2, (uint8_t *)CONFIRM, sizeof(CONFIRM), 0xFFFF);
	}
	else
	{
		stopped = 0;
		pwm_enable(&pwm_handler);
		pwm_set_value(0);
		ultrasound_enable(&ultrasound_handler);
	}
	// Reinit the call back
	HAL_UART_Receive_IT(&huart2, &hold_char, 1);
}

void ultrasound_callback() {
	uint32_t distance = 0;
	float pid_val = 0;

	if (post_passed == 0) return;

	distance = ultrasound_handler.distance;
	if (distance > 1050) // Hitting the ultrasound
		distance = 0;

	if (wait_for_steady < 10)
	{
		wait_for_steady++;
		return;
	}

	// Use PID to calculate next acceleration
	fcalculatePID(desired_distance_mm, 1050 - distance, &pid_handler);
	pid_val = pid_handler.pid_p + pid_handler.pid_i + pid_handler.pid_d / PERIOD;
	pwm_set_mapped_value(pid_val, 10000, -10000);

	buffer_size = snprintf(out_buffer, OUT_BUFFER_MAX, OUTPUT,
			(int32_t)pwm_handler.pwm_value,
			1050 - distance,
			(int32_t)pid_val,
			(int32_t)pid_handler.pid_p,
			(int32_t)pid_handler.pid_i,
			(int32_t)pid_handler.pid_d);
	HAL_UART_Transmit(&huart2, (uint8_t *)out_buffer, buffer_size, 0xFFFF);
}

void configure_pwm() {
	pwm_handler.max = 1000;
	pwm_handler.min = 0;
	pwm_handler.pwm_prcnt = 0.50f;
}

void configure_ultrasound() {
	ultrasound_handler.ultrasound_read_callback = ultrasound_callback;
}
