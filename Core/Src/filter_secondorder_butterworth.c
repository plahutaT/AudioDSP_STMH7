/*
 * filter_secondorder_butterworth.c
 *
 *  Created on: Dec 22, 2024
 *      Author: Tilen Plahuta
 *
 *  Implementation of a second-order low-pass Butterworth filter.
 *  This module provides initialization, configuration, and update functions
 *  for a digital low-pass Butterworth filter with a cutoff frequency of 500 Hz
 *  and a sampling frequency of 48 kHz.
 */

#include "filter_secondorder_butterworth.h"
#include <stdint.h>
#include <math.h>

/*
 * Initializes the second-order Butterworth low-pass filter structure.
 *
 * Parameters:
 *   filt   - Pointer to the filter structure (Butterworth_LowPass_t).
 *   fc_hz  - Cutoff frequency of the filter in Hz (500 Hz).
 *   fs_hz  - Sampling frequency of the system in Hz (48 kHz).
 *
 * This function sets up the filter by assigning the sampling frequency,
 * calculating the initial coefficients based on the cutoff frequency,
 * and initializing the filter output to zero.
 */
void Butterworth_LowPass_Init(Butterworth_LowPass_t* filt, float fc_hz, float fs_hz)
{
    // Set the sampling frequency
    filt->fs_hz = fs_hz;

    // Compute and set the filter coefficients based on the cutoff frequency
    Butterworth_LowPass_SetFc(filt, fc_hz);

    // Initialize the filter outputs and previous inputs to zero
    filt->filter_output[0] = 0.0f;  // y[n]
    filt->filter_output[1] = 0.0f;  // y[n-1]
    filt->filter_output[2] = 0.0f;  // y[n-2]
    filt->filter_input[0] = 0.0f;   // x[n]
    filt->filter_input[1] = 0.0f;   // x[n-1]
}

/*
 * Configures the filter's cutoff frequency.
 *
 * Parameters:
 *   filt   - Pointer to the filter structure (Butterworth_LowPass_t).
 *   fc_hz  - Desired cutoff frequency in Hz (500 Hz).
 *
 * This function calculates the filter coefficients based on the specified cutoff
 * frequency. The cutoff frequency is clamped to the range (0, fs/2) to ensure
 * the filter operates within the valid frequency range (Nyquist criterion).
 */
void Butterworth_LowPass_SetFc(Butterworth_LowPass_t* filt, float fc_hz)
{
    // Clamp the cutoff frequency to the valid range (0 < fc < fs/2)
    if (fc_hz > (0.5f * filt->fs_hz))
    {
        fc_hz = 0.5f * filt->fs_hz;
    }

    // Calculate the normalized cutoff frequency (omega_c / fs)
    float omega_c = 2 * M_PI * fc_hz;
    float T = 1.0f / filt->fs_hz;
    float omega_c_T = omega_c * T;

    // Calculate the filter coefficients (for a second-order Butterworth filter)
    float alpha = sin(omega_c_T) / (2 * cos(omega_c_T));
    float a0 = 1 + alpha;

    // Coefficients for the difference equation
    filt->filter_coef_b[0] = (1 - cos(omega_c_T)) / 2 / a0;
    filt->filter_coef_b[1] = (1 - cos(omega_c_T)) / a0;
    filt->filter_coef_b[2] = filt->filter_coef_b[0];
    filt->filter_coef_a[1] = -2 * cos(omega_c_T) / a0;
    filt->filter_coef_a[2] = (1 - alpha) / a0;
}

/*
 * Updates the filter with a new input sample and calculates the output.
 *
 * Parameters:
 *   filt   - Pointer to the filter structure (Butterworth_LowPass_t).
 *   input  - Current input sample.
 *
 * Returns:
 *   Filtered output value.
 *
 * This function computes the new filter output using the difference equation:
 *
 *   V_OUT[n] = b0 * V_IN[n] + b1 * V_IN[n-1] + b2 * V_IN[n-2]
 *             - a1 * V_OUT[n-1] - a2 * V_OUT[n-2]
 *
 * The output is clamped to the range [-1.0, 1.0] to avoid overflow or instability.
 */
float Butterworth_LowPass_Update(Butterworth_LowPass_t* filt, float input)
{
    // Apply the difference equation to calculate the new output
    float output = filt->filter_coef_b[0] * input + filt->filter_coef_b[1] * filt->filter_input[0]
                   + filt->filter_coef_b[2] * filt->filter_input[1]
                   - filt->filter_coef_a[1] * filt->filter_output[1]
                   - filt->filter_coef_a[2] * filt->filter_output[2];

    // Store the previous input and output for the next calculation
    filt->filter_input[1] = filt->filter_input[0];
    filt->filter_input[0] = input;

    filt->filter_output[2] = filt->filter_output[1];
    filt->filter_output[1] = filt->filter_output[0];
    filt->filter_output[0] = output;

    // Clamp the output to the range [-1.0, 1.0]
    if (output > 1.0f)
    {
        output = 1.0f;
    }
    else if (output < -1.0f)
    {
        output = -1.0f;
    }

    return output;
}
