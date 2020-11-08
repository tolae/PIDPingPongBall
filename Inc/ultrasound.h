#ifndef ECHO_CAPTURE_H
#define ECHO_CAPTURE_H

#define ULTRASOUND_MIN_RANGE (0) // mm
#define ULTRASOUND_MAX_RANGE (40000) // mm

typedef void (*void_callback)();

typedef struct ultrasound_read {
	unsigned int distance;

	void_callback ultrasound_read_callback;

	unsigned int valid : 1;

	unsigned int enabled : 1;
} ultrasound_read_t;

void ultrasound_enable(ultrasound_read_t *ultrasound);
void ultrasound_disable();

#endif
