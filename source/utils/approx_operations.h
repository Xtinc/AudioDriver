#ifndef AUDIO_PROCESSING_APPROX_OPERATIONS_H
#define AUDIO_PROCESSING_APPROX_OPERATIONS_H
#include "array_view.h"

// Sqrt approximation.
float SqrtFastApproximation(float f);

// Log base conversion log(x) = log2(x)/log2(e).
float LogApproximation(float x);
void LogApproximation(ArrayView<const float> x, ArrayView<float> y);

// 2^x approximation.
float Pow2Approximation(float p);

// x^p approximation.
float PowApproximation(float x, float p);

// e^x approximation.
float ExpApproximation(float x);
void ExpApproximation(ArrayView<const float> x, ArrayView<float> y);
void ExpApproximationSignFlip(ArrayView<const float> x, ArrayView<float> y);

#endif