/** \file
 * \authors Ethan Tola, Pamineo Richards
 * \brief Contains general PID implementation for any control system.
 */
#ifndef PID_H
#define PID_H
/** \struct PID
 * \brief A wrapper struct for PID used control variables.
 * \var PID::kp
 * The proportional control variable.
 * \var PID::ki
 * The intergration control variable.
 * \var PID::kd
 * The differentiation control variable.
 * \var PID::iErrorTerms
 * A pointer to a list of integer error terms.
 * \var PID::fErrorTerms
 * A pointer to a list of floating point error terms.
 * \var PID::iSavedValues
 * A pointer to a list of integer saved values.
 * \var PID::fSavedValues
 * A pointer to a list of floating point saved values.
 * \var PID::numOfErrors
 * The number of error terms to expect and store.
 * \var PID::numOfSaved
 * The number of saved values to expect and store.
 * \var PID:integrationTerms;
 * The number of error terms to incorporate when calculating the I term. This
 * should be less than or equal to numOfErrors.
 * \var PID:derivationTerms;
 * The number of error terms to incorporate when calculating the D term. This
 * should be less than or equal to numOfErrors.
 */
typedef struct PID {
    float kp;
    float ki;
    float kd;

    float pid_p;
    float pid_i;
    float pid_d;

    float sigma;

    union {
        int* iErrorTerms;
        float* fErrorTerms;
    };
    union {
        int* iSavedTerms;
        float* fSavedTerms;
    };
    unsigned int numOfErrors;
    unsigned int numOfSaved;

    unsigned int integrationTerms;
    unsigned int derivationTerms;
} c_pid_t;

/** \brief Calculates the next iteration using PID control algorithm.
 * \details PID requires various terms to keep it generic. Tone kp, ki, and kd
 * to get more accurate calculations as time goes on. Calculations are done
 * assuming the input values are integers and expects an integer to return.
 * \param desired The desired value to approach.
 * \param actual The actual value the system is currently at.
 * \param control A pointer to the PID control struct.
 * \return The value of the next iteration.
 */
int icalculatePID(const int desired, const int actual, c_pid_t* control);

/** \brief Calculates the next iteration using PID control algorithm.
 * \details PID requires various terms to keep it generic. Tone kp, ki, and kd
 * to get more accurate calculations as time goes on. Calculations are done
 * assuming the input values are floats and expects a float to return.
 * \param desired The desired value to approach.
 * \param actual The actual value the system is currently at.
 * \param control A pointer to the PID control struct.
 * \return The value of the next iteration.
 */
float fcalculatePID(const float desired, const float actual, c_pid_t* control);

#endif
