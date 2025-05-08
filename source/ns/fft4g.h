/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the ../../../LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef AUDIO_THIRD_PARTY_OOURA_FFT_SIZE_256_FFT4G_H_
#define AUDIO_THIRD_PARTY_OOURA_FFT_SIZE_256_FFT4G_H_

#include "utils/array_view.h"
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

#endif // AUDIO_THIRD_PARTY_OOURA_FFT_SIZE_256_FFT4G_H_
