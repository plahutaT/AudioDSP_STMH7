/*
 * filter_firstorder.c
 *
 *  Created on: Oct 29, 2024
 *      Author: Tilen Plahuta
 *
 *  Implementation of a first-order low-pass filter.
 *  This module provides initialization, configuration, and update functions
 *  for a simple digital low-pass filter. The filter is implemented based on
 *  a discrete-time difference equation.
 */

#include "filter_fisrtorder.h"
#include <stdint.h>
#include <math.h>

/*
 * Initializes the first-order low-pass filter structure.
 *
 * Parameters:
 *   filt   - Pointer to the filter structure (LowPass_FirstOrder_t).
 *   fc_hz  - Cutoff frequency of the filter in Hz.
 *   fs_hz  - Sampling frequency of the system in Hz.
 *
 * This function sets up the filter by assigning the sampling frequency,
 * calculating the initial coefficients based on the cutoff frequency,
 * and initializing the filter output to zero.
 */

void LowPass_FirstOrder_Init(LowPass_FirstOrder_t* filt, float fc_hz, float fs_hz)
{

    // Set the sampling frequency
    filt->fs_hz = fs_hz;

    // Compute and set the filter coefficients based on the cutoff frequency
    LowPass_FirstOrder_SetFc(filt, fc_hz);

    // Initialize the filter output to zero
    filt->filter_output = 0.0f;

}

/*
 * Configures the filter's cutoff frequency.
 *
 * Parameters:
 *   filt   - Pointer to the filter structure (LowPass_FirstOrder_t).
 *   fc_hz  - Desired cutoff frequency in Hz.
 *
 * This function adjusts the filter coefficients based on the specified cutoff
 * frequency. The cutoff frequency is clamped to the range (0, fs/2) to ensure
 * the filter operates within the valid frequency range (Nyquist criterion).
 */


void LowPass_FirstOrder_SetFc(LowPass_FirstOrder_t* filt, float fc_hz)
{

    // Clamp the cutoff frequency to the valid range (0 < fc < fs/2)
    if (fc_hz > (0.5f * filt->fs_hz))
    {
        fc_hz = 0.5f * filt->fs_hz;
    }

    // Calculate filter coefficients based on the cutoff frequency
    float k1 = 2 * M_PI * fc_hz / filt->fs_hz;
    filt->filter_coef[0] = k1 / (1.0f + k1); // Coefficient for input
    filt->filter_coef[1] = 1.0f / (1.0f + k1); // Coefficient for previous output

}



/*
 * Updates the filter with a new input sample and calculates the output.
 *
 * Parameters:
 *   filt   - Pointer to the filter structure (LowPass_FirstOrder_t).
 *   input  - Current input sample.
 *
 * Returns:
 *   Filtered output value.
 *
 * This function computes the new filter output using the difference equation:
 *
 *   V_OUT[n] = (alfa / (1 + alfa)) * V_IN[n] + (1 / (1 + alfa)) * V_OUT[n-1]
 *
 * The output is clamped to the range [-1.0, 1.0] to avoid overflow or instability.
 */


float LowPass_FirstOrder_Update(LowPass_FirstOrder_t* filt, float input)
{

    // Apply the difference equation to calculate the new output
    filt->filter_output = filt->filter_coef[0] * input + filt->filter_coef[1] * filt->filter_output;

    // Clamp the output to the range [-1.0, 1.0]
    if (filt->filter_output > 1.0f)
    {
        filt->filter_output = 1.0f;
    }
    else if (filt->filter_output < -1.0f)
    {
        filt->filter_output = -1.0f;
    }

    return filt->filter_output;


}


