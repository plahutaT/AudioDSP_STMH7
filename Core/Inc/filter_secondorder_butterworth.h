/*
 * filter_secondorder_butterworth.h
 *
 *  Created on: Dec 22, 2024
 *      Author: tilen
 */

#ifndef INC_FILTER_SECONDORDER_BUTTERWORTH_H_
#define INC_FILTER_SECONDORDER_BUTTERWORTH_H_

#include <stdint.h>

// Structure for the second-order Butterworth filter
typedef struct {
    float fs_hz;                // Sampling frequency (Hz)
    float filter_coef_b[3];     // Coefficients for the numerator (b0, b1, b2)
    float filter_coef_a[3];     // Coefficients for the denominator (a1, a2)

    // Arrays to hold previous inputs and outputs
    float filter_input[2];      // Previous input values (x[n-1], x[n-2])
    float filter_output[3];     // Previous output values (y[n-1], y[n-2])
} Butterworth_LowPass_t;

// Function to initialize the Butterworth low-pass filter
void Butterworth_LowPass_Init(Butterworth_LowPass_t* filt, float fc_hz, float fs_hz);

// Function to configure the cutoff frequency for the Butterworth filter
void Butterworth_LowPass_SetFc(Butterworth_LowPass_t* filt, float fc_hz);

// Function to update the filter with a new input sample and calculate the output
float Butterworth_LowPass_Update(Butterworth_LowPass_t* filt, float input);

#endif /* INC_FILTER_SECONDORDER_BUTTERWORTH_H_ */
