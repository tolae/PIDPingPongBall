#ifndef MY_MAIN_H
#define MY_MAIN_H

#include "ultrasound.h"
#include "pwm.h"

extern ultrasound_read_t ultrasound_handler;
extern pwm_t pwm_handler;

void setup(void);
void loop(void);

#endif
