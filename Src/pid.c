#include "pid.h"

/** Internal functions **/
extern inline float _icalculatePorportional(c_pid_t* control, int error);
extern inline float _fcalculatePorportional(c_pid_t* control, float error);

float _icalculateIntegration(c_pid_t* control, int error);
float _fcalculateIntegration(c_pid_t* control, float error);

float _icalculateDerivative(c_pid_t* control, int error);
float _fcalculateDerivative(c_pid_t* control, float error);

void _printPidParams(const float kp, const float ki, const float kd);

int icalculatePID(const int desired, const int actual, c_pid_t* control) {
    // Calculate error
    int error = desired - actual;
    // Calculate PID components
    control->pid_p = _icalculatePorportional(control, error);
	control->pid_i = _icalculateIntegration(control, error);
	control->pid_d = _icalculateDerivative(control, error);
    // Store error terms
    for (unsigned int i = 0; i < control->numOfErrors; i++) {
        control->iErrorTerms[i+1] = control->iErrorTerms[i];
    }
    control->iErrorTerms[0] = error;
    return ((int) (control->pid_p + control->pid_i + control->pid_d));
}

float fcalculatePID(const float desired, const float actual, c_pid_t* control) {
    // Calculate error
    float error = desired - actual;
    // Calculate PID components
    control->pid_p = _fcalculatePorportional(control, error);
    control->pid_i = _fcalculateIntegration(control, error);
    control->pid_d = _fcalculateDerivative(control, error);
    // Store error terms
    for (unsigned int i = 0; i < control->numOfErrors-1; i++) {
        control->fErrorTerms[i+1] = control->fErrorTerms[i];
    }
    control->fErrorTerms[0] = error;
    return (control->pid_p + control->pid_i + control->pid_d);
}

inline float _icalculatePorportional(c_pid_t* control, int error) {
    return control->kp * error;
}
inline float _fcalculatePorportional(c_pid_t* control, float error) {
    return control->kp * error;
}

float _icalculateIntegration(c_pid_t* control, int error) {
    if (control->integrationTerms == 0) { return 0; }
    float sigma = ((((float)error) + ((float)control->iErrorTerms[0])) / 2.0f);
    for (unsigned int i = 1; i < control->integrationTerms; i++) {
        sigma += (((float)control->iErrorTerms[i]) + ((float)control->iErrorTerms[i+1]) / 2.0f);
    }
    return (sigma * control->ki);
}

float _fcalculateIntegration(c_pid_t* control, float error) {
    if (control->integrationTerms == 0) { return 0; }
//    float sigma = ((error + control->fErrorTerms[0]) / 2.0f);
//    for (unsigned int i = 1; i < control->integrationTerms; i++) {
//        sigma += ((control->fErrorTerms[i] + control->fErrorTerms[i+1]) / 2.0f);
//    }
    control->sigma = control->sigma + error;
    return (control->sigma * control->ki);
}

float _icalculateDerivative(c_pid_t* control, int error) {
    if (control->derivationTerms == 0) { return 0; }
    float diffs = error - control->iErrorTerms[0];
    for (unsigned int i = 1; i < control->derivationTerms; i++) {
        diffs -= control->iErrorTerms[i] - control->iErrorTerms[i+1];
    }
    return (diffs * control->kd);
}

float _fcalculateDerivative(c_pid_t* control, float error) {
    if (control->derivationTerms == 0) { return 0; }
    float diffs = error - control->fErrorTerms[0];
    for (unsigned int i = 1; i < control->derivationTerms; i++) {
        diffs -= control->fErrorTerms[i] - control->fErrorTerms[i+1];
    }
    return (diffs * control->kd);
}
