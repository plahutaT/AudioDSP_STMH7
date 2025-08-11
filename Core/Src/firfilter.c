#include "firfilter.h"


/* Create the impulse response coefficients array */
static const float FIR_Impulse_Response[FIR_FILTER_LENGTH] = {
    -0.016123f,
    -0.001675f,
    -0.001294f,
    -0.000556f,
    0.000568f,
    0.002100f,
    0.004052f,
    0.006422f,
    0.009200f,
    0.012372f,
    0.015879f,
    0.019683f,
    0.023724f,
    0.027919f,
    0.032190f,
    0.036450f,
    0.040606f,
    0.044557f,
    0.048195f,
    0.051438f,
    0.054252f,
    0.056484f,
    0.058134f,
    0.059142f,
    0.059481f,
    0.059142f,
    0.058134f,
    0.056484f,
    0.054252f,
    0.051438f,
    0.048195f,
    0.044557f,
    0.040606f,
    0.036450f,
    0.032190f,
    0.027919f,
    0.023724f,
    0.019683f,
    0.015879f,
    0.012372f,
    0.009200f,
    0.006422f,
    0.004052f,
    0.002100f,
    0.000568f,
    -0.000556f,
    -0.001294f,
    -0.001675f,
    -0.016123f
};


/* Initialize the FIR filter structure */
void FirFilter_Init(FirFilter_t* fir) {
    /* Clear the filter buffer */
    for (size_t i = 0; i < FIR_FILTER_LENGTH; i++)
    {
        fir->filter_buff[i] = 0.0f;
    }

    /* Reset the buffer index */
    fir->bufIndex = 0;

    /* Clear the filter output */
    fir->filter_output = 0.0f;
}




/* Update the FIR filter with a new input sample */
float FirFilter_Update(FirFilter_t* fir, float input) {
    /* Store the latest sample in the buffer */
    fir->filter_buff[fir->bufIndex] = input;

    /* Update the buffer index with circular wrapping */
    fir->bufIndex = (fir->bufIndex + 1) % FIR_FILTER_LENGTH;

    /* Compute the filter output using the impulse response */
    fir->filter_output = 0.0f;
    size_t sumIndex = fir->bufIndex;

    for (size_t i = 0; i < FIR_FILTER_LENGTH; i++)
    {
        if (sumIndex > 0)
        {
            sumIndex--;
        }
        else
        {
            sumIndex = FIR_FILTER_LENGTH - 1;
        }

        fir->filter_output += FIR_Impulse_Response[i] * fir->filter_buff[sumIndex];
    }

    return fir->filter_output;
}


