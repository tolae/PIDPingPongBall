#ifndef PWM_H
#define PWM_H

typedef struct pwm {
	int max;
	int min;

	union {
		float pwm_prcnt;
		int pwm_value;
	};

	unsigned int enabled : 1;
} pwm_t;

void pwm_enable(pwm_t *pwm);
void pwm_disable();

void pwm_set_prcnt(float percentage);
void pwm_set_mapped_value(int value, int v_max, int v_min);
void pwm_set_value(unsigned int value);

#endif
