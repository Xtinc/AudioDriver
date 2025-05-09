#include "approx_opers.h"
#include <cmath>

static float FastLog2f(float in)
{
    // Read and interpret float as uint32_t and then cast to float.
    // This is done to extract the exponent (bits 30 - 23).
    // "Right shift" of the exponent is then performed by multiplying
    // with the constant (1/2^23). Finally, we subtract a constant to
    // remove the bias (https://en.wikipedia.org/wiki/Exponent_bias).
    union {
        float dummy;
        uint32_t a;
    } x = {in};
    auto out = static_cast<float>(x.a);
    out *= 1.1920929e-7f; // 1/2^23
    out -= 126.942695f;   // Remove bias.
    return out;
}

float SqrtFastApproximation(float f)
{
    // TODO: Add fast approximate implementation.
    return sqrtf(f);
}

float Pow2Approximation(float p)
{
    // TODO: Add fast approximate implementation.
    return powf(2.f, p);
}

float PowApproximation(float x, float p)
{
    return Pow2Approximation(p * FastLog2f(x));
}

float LogApproximation(float x)
{
    constexpr float kLogOf2 = 0.69314718056f;
    return FastLog2f(x) * kLogOf2;
}

void LogApproximation(ArrayView<const float> x, ArrayView<float> y)
{
    for (size_t k = 0; k < x.size(); ++k)
    {
        y[k] = LogApproximation(x[k]);
    }
}

float ExpApproximation(float x)
{
    constexpr float kLog10Ofe = 0.4342944819f;
    return PowApproximation(10.f, x * kLog10Ofe);
}

void ExpApproximation(ArrayView<const float> x, ArrayView<float> y)
{
    for (size_t k = 0; k < x.size(); ++k)
    {
        y[k] = ExpApproximation(x[k]);
    }
}

void ExpApproximationSignFlip(ArrayView<const float> x, ArrayView<float> y)
{
    for (size_t k = 0; k < x.size(); ++k)
    {
        y[k] = ExpApproximation(-x[k]);
    }
}
