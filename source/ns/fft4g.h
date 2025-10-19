#ifndef AUDIO_THIRD_PARTY_OOURA_FFT_SIZE_256_FFT4G_H_
#define AUDIO_THIRD_PARTY_OOURA_FFT_SIZE_256_FFT4G_H_

#include "audio_dataview.h"
#include <vector>

constexpr size_t kFftSize = 256;
constexpr size_t kFftSizeBy2Plus1 = kFftSize / 2 + 1;

// Wrapper class providing 256 point FFT functionality.
class NrFft
{
  public:
    NrFft();
    NrFft(const NrFft &) = delete;
    NrFft &operator=(const NrFft &) = delete;

    // Transforms the signal from time to frequency domain.
    void Fft(ArrayView<float, kFftSize> time_data, ArrayView<float, kFftSize> real, ArrayView<float, kFftSize> imag);

    // Transforms the signal from frequency to time domain.
    void Ifft(ArrayView<const float> real, ArrayView<const float> imag, ArrayView<float> time_data);

  private:
    std::vector<size_t> bit_reversal_state_;
    std::vector<float> tables_;
};

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

#endif // AUDIO_THIRD_PARTY_OOURA_FFT_SIZE_256_FFT4G_H_
