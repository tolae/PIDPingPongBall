#include <stdio.h>
#include "post.h"
#include "my_main.h"
#include "usart.h"

#define POST_SUCCESS (1)
#define POST_FAILURE (0)

const uint8_t USER_PROMPT[] =
	"Place a large object about 100 mm from the sensor.\n\r"
	"Press enter to begin.\n\r";

const char TOO_FAR[] = "The object (%ud mm) is too far!\n\r";
const char TOO_CLOSE[] = "The object (%ud mm) is too close!\n\r";

char out_buff[64] = { 0 };
size_t siz = 0;
uint8_t user_key = 0;

unsigned int run_post(void) {
	uint8_t ret_code = POST_SUCCESS;
	HAL_UART_Transmit(&huart2, (uint8_t *)USER_PROMPT, sizeof(USER_PROMPT), 0xFFFF);
	HAL_UART_Receive(&huart2, &user_key, 1, 0xFFFF);
	if (user_key == 0) return POST_FAILURE;

	// Wait until valid signal is read
	while (!ultrasound_handler.valid);
	// Check if within the min and max ranges of the ultrasound
	if (ultrasound_handler.distance < ULTRASOUND_MIN_RANGE) {
		siz = snprintf(out_buff, 64, TOO_CLOSE, ultrasound_handler.distance);
		HAL_UART_Transmit(&huart2, (uint8_t *)out_buff, siz, 0xFFFF);
		ret_code = POST_FAILURE;
	}
	else if (ultrasound_handler.distance > ULTRASOUND_MAX_RANGE) {
		siz = snprintf(out_buff, 64, TOO_FAR, ultrasound_handler.distance);
		HAL_UART_Transmit(&huart2, (uint8_t *)out_buff, siz, 0xFFFF);
		ret_code = POST_FAILURE;
	}
	return ret_code;
}
