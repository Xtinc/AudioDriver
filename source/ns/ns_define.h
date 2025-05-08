#ifndef AUDIO_PROCESSING_NOISE_SUPPRESSION_DEFINE_H
#define AUDIO_PROCESSING_NOISE_SUPPRESSION_DEFINE_H

#include "fft4g.h"

constexpr size_t kNsFrameSize = 160;
constexpr size_t kOverlapSize = kFftSize - kNsFrameSize;

constexpr int kShortStartupPhaseBlocks = 50;
constexpr int kLongStartupPhaseBlocks = 200;
constexpr int kFeatureUpdateWindowSize = 500;

constexpr float kLtrFeatureThr = 0.5f;
constexpr float kBinSizeLrt = 0.1f;
constexpr float kBinSizeSpecFlat = 0.05f;
constexpr float kBinSizeSpecDiff = 0.1f;


struct SuppressionParams
{
    explicit SuppressionParams(int suppression_level)
    {
        switch (suppression_level)
        {
        default:
        case 0:
            over_subtraction_factor = 1.f;
            // 6 dB attenuation.
            minimum_attenuating_gain = 0.5f;
            use_attenuation_adjustment = false;
            break;
        case 1:
            over_subtraction_factor = 1.f;
            // 12 dB attenuation.
            minimum_attenuating_gain = 0.25f;
            use_attenuation_adjustment = true;
            break;
        case 2:
            over_subtraction_factor = 1.1f;
            // 18 dB attenuation.
            minimum_attenuating_gain = 0.125f;
            use_attenuation_adjustment = true;
            break;
        case 3:
            over_subtraction_factor = 1.25f;
            // 20.9 dB attenuation.
            minimum_attenuating_gain = 0.09f;
            use_attenuation_adjustment = true;
            break;
        }
    }
    SuppressionParams(const SuppressionParams &) = delete;
    SuppressionParams &operator=(const SuppressionParams &) = delete;

    float over_subtraction_factor;
    float minimum_attenuating_gain;
    bool use_attenuation_adjustment;
};

#endif