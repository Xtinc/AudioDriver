/**
 * @file filter_banks.cpp
 * @brief An implementation of a 3-band FIR filter-bank with DCT modulation and 2-band
 * QMF filter bank
 *
 * Similar to the proposed in "Multirate Signal Processing for Communication Systems" by
 * Fredric J Harris.
 *
 * The idea is to take a heterodyne system and change the order of the
 * components to get something which is efficient to implement digitally.
 *
 * It is possible to separate the filter using the noble identity as follows:
 *
 * H(z) = H0(z^3) + z^-1 * H1(z^3) + z^-2 * H2(z^3)
 *
 * This is used in the analysis stage to first downsample serial to parallel
 * and then filter each branch with one of these polyphase decompositions of the
 * lowpass prototype. Because each filter is only a modulation of the prototype,
 * it is enough to multiply each coefficient by the respective cosine value to
 * shift it to the desired band. But because the cosine period is 12 samples,
 * it requires separating the prototype even further using the noble identity.
 * After filtering and modulating for each band, the output of all filters is
 * accumulated to get the downsampled bands.
 *
 * A similar logic can be applied to the synthesis stage.
 */
#include "audio_filterbanks.h"
#include <cmath>

namespace
{
/**
* Factors to take into account when choosing `kFilterSize`:
*   1. Higher `kFilterSize`, means faster transition, which ensures less
*      aliasing. This is especially important when there is non-linear
*      processing between the splitting and merging.
*   2. The delay that this filter bank introduces is
*      `kNumBands` * `kSparsity` * `kFilterSize` / 2, so it increases linearly
*      with `kFilterSize`.
*   3. The computation complexity also increases linearly with `kFilterSize`.

* Filter coefficients generated using MATLAB
* The Matlab code to generate these `kFilterCoeffs` is:
*
* N = kNumBands * kSparsity * kFilterSize - 1;
* h = fir1(N, 1 / (2 * kNumBands), kaiser(N + 1, 3.5));
* reshape(h, kNumBands * kSparsity, kFilterSize);
*
* The code below uses the values of kFilterSize, kNumBands and kSparsity
* specified in the header.
*
* Because the total bandwidth of the lower and higher band is double the middle
* one (because of the spectrum parity), the low-pass prototype is half the
* bandwidth of 1 / (2 * `kNumBands`) and is then shifted with cosine modulation
* to the right places.
* A Kaiser window is used because of its flexibility and the alpha is set to
* 3.5, since that sets a stop band attenuation of 40dB ensuring a fast
* transition.
*/

constexpr int kSubSampling = ThreeBandFilterBank::kNumBands;
constexpr int kDctSize = ThreeBandFilterBank::kNumBands;
static_assert(ThreeBandFilterBank::kNumBands * ThreeBandFilterBank::kSplitBandSize ==
                  ThreeBandFilterBank::kFullBandSize,
              "The full band must be split in equally sized subbands");

const float kFilterCoeffs[ThreeBandFilterBank::kNumNonZeroFilters][ThreeBandFilterBank::kFilterSize] = {
    {-0.00047749f, -0.00496888f, +0.16547118f, +0.00425496f}, {-0.00173287f, -0.01585778f, +0.14989004f, +0.00994113f},
    {-0.00304815f, -0.02536082f, +0.12154542f, +0.01157993f}, {-0.00346946f, -0.02587886f, +0.04760441f, +0.00607594f},
    {-0.00154717f, -0.01136076f, +0.01387458f, +0.00186353f}, {+0.00186353f, +0.01387458f, -0.01136076f, -0.00154717f},
    {+0.00607594f, +0.04760441f, -0.02587886f, -0.00346946f}, {+0.00983212f, +0.08543175f, -0.02982767f, -0.00383509f},
    {+0.00994113f, +0.14989004f, -0.01585778f, -0.00173287f}, {+0.00425496f, +0.16547118f, -0.00496888f, -0.00047749f}};

constexpr int kZeroFilterIndex1 = 3;
constexpr int kZeroFilterIndex2 = 9;

const float kDctModulation[ThreeBandFilterBank::kNumNonZeroFilters][kDctSize] = {{2.f, 2.f, 2.f},
                                                                                 {1.73205077f, 0.f, -1.73205077f},
                                                                                 {1.f, -2.f, 1.f},
                                                                                 {-1.f, 2.f, -1.f},
                                                                                 {-1.73205077f, 0.f, 1.73205077f},
                                                                                 {-2.f, -2.f, -2.f},
                                                                                 {-1.73205077f, 0.f, 1.73205077f},
                                                                                 {-1.f, 2.f, -1.f},
                                                                                 {1.f, -2.f, 1.f},
                                                                                 {1.73205077f, 0.f, -1.73205077f}};

/**
 * @brief Filters the input signal with the specified filter
 *
 * @param filter The filter coefficients
 * @param in The input signal
 * @param in_shift The shift value for the input
 * @param out The output signal buffer
 * @param state The previous filter state
 */
void FilterCore(ArrayView<const float, ThreeBandFilterBank::kFilterSize> filter,
                ArrayView<const float, ThreeBandFilterBank::kSplitBandSize> in, const int in_shift,
                ArrayView<float, ThreeBandFilterBank::kSplitBandSize> out,
                ArrayView<float, ThreeBandFilterBank::kMemorySize> state)
{
    std::fill(out.begin(), out.end(), 0.f);

    for (int k = 0; k < in_shift; ++k)
    {
        for (int i = 0, j = ThreeBandFilterBank::kMemorySize + k - in_shift; i < ThreeBandFilterBank::kFilterSize;
             ++i, j -= ThreeBandFilterBank::kStride)
        {
            out[k] += state[j] * filter[i];
        }
    }

    for (int k = in_shift, shift = 0; k < ThreeBandFilterBank::kFilterSize * ThreeBandFilterBank::kStride; ++k, ++shift)
    {
        const int loop_limit =
            std::min(ThreeBandFilterBank::kFilterSize, 1 + (shift >> ThreeBandFilterBank::kStrideLog2));
        for (int i = 0, j = shift; i < loop_limit; ++i, j -= ThreeBandFilterBank::kStride)
        {
            out[k] += in[j] * filter[i];
        }
        for (int i = loop_limit,
                 j = ThreeBandFilterBank::kMemorySize + shift - loop_limit * ThreeBandFilterBank::kStride;
             i < ThreeBandFilterBank::kFilterSize; ++i, j -= ThreeBandFilterBank::kStride)
        {
            out[k] += state[j] * filter[i];
        }
    }

    for (int k = ThreeBandFilterBank::kFilterSize * ThreeBandFilterBank::kStride,
             shift = ThreeBandFilterBank::kFilterSize * ThreeBandFilterBank::kStride - in_shift;
         k < ThreeBandFilterBank::kSplitBandSize; ++k, ++shift)
    {
        for (int i = 0, j = shift; i < ThreeBandFilterBank::kFilterSize; ++i, j -= ThreeBandFilterBank::kStride)
        {
            out[k] += in[j] * filter[i];
        }
    }

    // Update current state.
    std::copy(in.begin() + ThreeBandFilterBank::kSplitBandSize - ThreeBandFilterBank::kMemorySize, in.end(),
              state.begin());
}

constexpr int kAllPassFilterSize = 3;

constexpr float kAllPassFilter1[kAllPassFilterSize] = {6418.0 / 65536.0, 36982.0 / 65536.0, 57261.0 / 65536.0};
constexpr float kAllPassFilter2[kAllPassFilterSize] = {21333.0 / 65536.0, 49062.0 / 65536.0, 63010.0 / 65536.0};

/**
 * @brief Implements All Pass Quadrature Mirror Filter
 *
 * The procedure is to filter the input with three first order all pass
 * filters (cascade operations).
 *
 * @verbatim
 *         a_3 + q^-1    a_2 + q^-1    a_1 + q^-1
 * y[n] =  -----------   -----------   -----------   x[n]
 *         1 + a_3q^-1   1 + a_2q^-1   1 + a_1q^-1
 * @endverbatim
 *
 * The input vector `filter_coefficients` includes these three filter
 * coefficients. The filter state contains the in_data state, in_data[-1],
 * followed by the out_data state, out_data[-1]. This is repeated for each
 * cascade. The first cascade filter will filter the `in_data` and store
 * the output in `out_data`. The second will the take the `out_data` as
 * input and make an intermediate storage in `in_data`, to save memory. The
 * third, and final, cascade filter operation takes the `in_data` (which is
 * the output from the previous cascade filter) and store the output in
 * `out_data`. Note that the input vector values are changed during the
 * process.
 *
 * @param in Input signal
 * @param out Output signal buffer
 * @param filter_coefficients Filter coefficients
 * @param filter_state Filter state
 */
void AllPassQMF(ArrayView<float, TwoBandFilterBank::kSplitBandSize> in,
                ArrayView<float, TwoBandFilterBank::kSplitBandSize> out,
                ArrayView<const float, kAllPassFilterSize> filter_coefficients,
                ArrayView<float, TwoBandFilterBank::kStateSize> filter_state)
{
    // First loop, use the states stored in memory.
    auto diff = in[0] - filter_state[1];
    out[0] = filter_state[0] + filter_coefficients[0] * diff;

    // For the remaining loops, use previous values.
    for (size_t k = 1; k < TwoBandFilterBank::kSplitBandSize; k++)
    {
        diff = in[k] - out[k - 1];
        out[k] = in[k - 1] + filter_coefficients[0] * diff;
    }

    // Update states.
    filter_state[0] = in[TwoBandFilterBank::kSplitBandSize - 1];  // x[N-1], becomes x[-1] next time
    filter_state[1] = out[TwoBandFilterBank::kSplitBandSize - 1]; // y_1[N-1], becomes y_1[-1] next time

    // Second all-pass cascade; filter from out_data to in_data.
    diff = out[0] - filter_state[3];
    in[0] = filter_state[2] + filter_coefficients[1] * diff;
    for (size_t k = 1; k < TwoBandFilterBank::kSplitBandSize; k++)
    {
        diff = out[k] - in[k - 1];
        in[k] = out[k - 1] + filter_coefficients[1] * diff;
    }

    filter_state[2] = out[TwoBandFilterBank::kSplitBandSize - 1]; // y_1[N-1], becomes y_1[-1] next time
    filter_state[3] = in[TwoBandFilterBank::kSplitBandSize - 1];  // y_2[N-1], becomes y_2[-1] next time

    // Third all-pass cascade; filter from in_data to out_data.
    diff = in[0] - filter_state[5];
    out[0] = filter_state[4] + filter_coefficients[2] * diff;
    for (size_t k = 1; k < TwoBandFilterBank::kSplitBandSize; k++)
    {
        diff = in[k] - out[k - 1];
        out[k] = in[k - 1] + filter_coefficients[2] * diff;
    }
    filter_state[4] = in[TwoBandFilterBank::kSplitBandSize - 1];  // y_2[N-1], becomes y_2[-1] next time
    filter_state[5] = out[TwoBandFilterBank::kSplitBandSize - 1]; // y[N-1], becomes y[-1] next time
}

} // namespace

/**
 * @brief Constructor for ThreeBandFilterBank
 *
 * Because the low-pass filter prototype has half bandwidth it is possible to
 * use a DCT to shift it in both directions at the same time, to the center
 * frequencies [1 / 12, 3 / 12, 5 / 12].
 */
ThreeBandFilterBank::ThreeBandFilterBank() = default;

ThreeBandFilterBank::~ThreeBandFilterBank() = default;

/**
 * @brief Analysis function for the ThreeBandFilterBank
 *
 * The analysis can be separated in these steps:
 *   1. Serial to parallel downsampling by a factor of `kNumBands`.
 *   2. Filtering of `kSparsity` different delayed signals with polyphase
 *      decomposition of the low-pass prototype filter and upsampled by a factor
 *      of `kSparsity`.
 *   3. Modulating with cosines and accumulating to get the desired band.
 *
 * @param in Full band input signal
 * @param out Array of output bands
 */
void ThreeBandFilterBank::Analysis(ArrayView<const float, kFullBandSize> in,
                                   ArrayView<const ArrayView<float>, kNumBands> out)
{
    // Initialize the output to zero.
    for (int band = 0; band < kNumBands; ++band)
    {
        std::fill(out[band].begin(), out[band].end(), 0.0f);
    }

    for (int downsampling_index = 0; downsampling_index < kSubSampling; ++downsampling_index)
    {
        // Downsample to form the filter input.
        std::array<float, kSplitBandSize> in_subsampled;
        for (int k = 0; k < kSplitBandSize; ++k)
        {
            in_subsampled[k] = in[(kSubSampling - 1) - downsampling_index + kSubSampling * k];
        }

        for (int in_shift = 0; in_shift < kStride; ++in_shift)
        {
            // Choose filter, skip zero filters.
            const int index = downsampling_index + in_shift * kSubSampling;
            if (index == kZeroFilterIndex1 || index == kZeroFilterIndex2)
            {
                continue;
            }
            const int filter_index =
                index < kZeroFilterIndex1 ? index : (index < kZeroFilterIndex2 ? index - 1 : index - 2);

            ArrayView<const float, kFilterSize> filter(kFilterCoeffs[filter_index]);
            ArrayView<const float, kDctSize> dct_modulation(kDctModulation[filter_index]);
            ArrayView<float, kMemorySize> state(state_analysis[filter_index]);

            // Filter.
            std::array<float, kSplitBandSize> out_subsampled;
            FilterCore(filter, in_subsampled, in_shift, out_subsampled, state);

            // Band and modulate the output.
            for (int band = 0; band < ThreeBandFilterBank::kNumBands; ++band)
            {
                float *out_band = out[band].data();
                for (int n = 0; n < kSplitBandSize; ++n)
                {
                    out_band[n] += dct_modulation[band] * out_subsampled[n];
                }
            }
        }
    }
}

/**
 * @brief Synthesis function for the ThreeBandFilterBank
 *
 * The synthesis can be separated in these steps:
 *   1. Modulating with cosines.
 *   2. Filtering each one with a polyphase decomposition of the low-pass
 *      prototype filter upsampled by a factor of `kSparsity` and accumulating
 *      `kSparsity` signals with different delays.
 *   3. Parallel to serial upsampling by a factor of `kNumBands`.
 *
 * @param in Array of input bands
 * @param out Full band output signal
 */
void ThreeBandFilterBank::Synthesis(ArrayView<const ArrayView<float>, kNumBands> in,
                                    ArrayView<float, kFullBandSize> out)
{
    std::fill(out.begin(), out.end(), 0.0f);
    for (int upsampling_index = 0; upsampling_index < kSubSampling; ++upsampling_index)
    {
        for (int in_shift = 0; in_shift < kStride; ++in_shift)
        {
            // Choose filter, skip zero filters.
            const int index = upsampling_index + in_shift * kSubSampling;
            if (index == kZeroFilterIndex1 || index == kZeroFilterIndex2)
            {
                continue;
            }
            const int filter_index =
                index < kZeroFilterIndex1 ? index : (index < kZeroFilterIndex2 ? index - 1 : index - 2);

            ArrayView<const float, kFilterSize> filter(kFilterCoeffs[filter_index]);
            ArrayView<const float, kDctSize> dct_modulation(kDctModulation[filter_index]);
            ArrayView<float, kMemorySize> state(state_synthesis[filter_index]);

            // Prepare filter input by modulating the banded input.
            std::array<float, kSplitBandSize> in_subsampled{};
            for (int band = 0; band < kNumBands; ++band)
            {
                const float *in_band = in[band].data();
                for (int n = 0; n < kSplitBandSize; ++n)
                {
                    in_subsampled[n] += dct_modulation[band] * in_band[n];
                }
            }

            // Filter.
            std::array<float, kSplitBandSize> out_subsampled;
            FilterCore(filter, in_subsampled, in_shift, out_subsampled, state);

            // Upsample.
            constexpr float kUpsamplingScaling = kSubSampling;
            for (int k = 0; k < kSplitBandSize; ++k)
            {
                out[upsampling_index + kSubSampling * k] += kUpsamplingScaling * out_subsampled[k];
            }
        }
    }
}

/**
 * @brief Constructor for TwoBandFilterBank
 */
TwoBandFilterBank::TwoBandFilterBank() = default;

TwoBandFilterBank::~TwoBandFilterBank() = default;

/**
 * @brief Analysis function for the TwoBandFilterBank
 *
 * @param in Full band input signal
 * @param out Array of output bands
 */
void TwoBandFilterBank::Analysis(ArrayView<const float, kFullBandSize> in,
                                 ArrayView<const ArrayView<float>, kNumBands> out)
{
    float half_in1[kSplitBandSize];
    float half_in2[kSplitBandSize];
    float filter1[kSplitBandSize];
    float filter2[kSplitBandSize];

    for (size_t i = 0, k = 0; i < kSplitBandSize; i++, k += 2)
    {
        half_in2[i] = in[k];
        half_in1[i] = in[k + 1];
    }

    ArrayView<float, kSplitBandSize> in_even(half_in1);
    ArrayView<float, kSplitBandSize> in_odd(half_in2);
    ArrayView<float, kSplitBandSize> out_even(filter1);
    ArrayView<float, kSplitBandSize> out_odd(filter2);

    // All pass filter even and odd samples, independently.
    AllPassQMF(in_even, out_even, kAllPassFilter1, state_analysis[0]);
    AllPassQMF(in_odd, out_odd, kAllPassFilter2, state_analysis[1]);

    // Take the sum and difference of filtered version of odd and even
    // branches to get lower & upper band.
    for (size_t i = 0; i < kSplitBandSize; i++)
    {
        out[0][i] = (filter1[i] + filter2[i]) * 0.5f;
        out[1][i] = (filter1[i] - filter2[i]) * 0.5f;
    }
}

/**
 * @brief Synthesis function for the TwoBandFilterBank
 *
 * @param in Array of input bands
 * @param out Full band output signal
 */
void TwoBandFilterBank::Synthesis(ArrayView<const ArrayView<float>, kNumBands> in, ArrayView<float, kFullBandSize> out)
{
    float half_in1[kSplitBandSize];
    float half_in2[kSplitBandSize];
    float filter1[kSplitBandSize];
    float filter2[kSplitBandSize];

    for (size_t i = 0; i < kSplitBandSize; i++)
    {
        half_in1[i] = in[0][i] + in[1][i];
        half_in2[i] = in[0][i] - in[1][i];
    }

    ArrayView<float, kSplitBandSize> in_even(half_in1);
    ArrayView<float, kSplitBandSize> in_odd(half_in2);
    ArrayView<float, kSplitBandSize> out_even(filter1);
    ArrayView<float, kSplitBandSize> out_odd(filter2);

    // All pass filter even and odd samples, independently.
    AllPassQMF(in_even, out_even, kAllPassFilter2, state_synthesis[0]);
    AllPassQMF(in_odd, out_odd, kAllPassFilter1, state_synthesis[1]);

    for (size_t i = 0, j = 0; i < kSplitBandSize; i++, j += 2)
    {
        out[j] = filter2[i];
        out[j + 1] = filter1[i];
    }
}

static constexpr size_t kSamplesPerBand = 160;
static constexpr size_t kTwoBandFilterSamplesPerFrame = 320;

SplittingFilter::SplittingFilter(size_t num_channels, size_t num_bands, size_t /* num_frames */)
    : num_bands_(num_bands), two_bands_states_(num_bands_ == 2 ? num_channels : 0),
      three_band_filter_banks_(num_bands_ == 3 ? num_channels : 0)
{
    DBG_ASSERT_COND(num_bands_ == 2 || num_bands_ == 3);
}

SplittingFilter::~SplittingFilter() = default;

void SplittingFilter::Analysis(const ChannelBuffer<float> *data, ChannelBuffer<float> *bands)
{
    DBG_ASSERT_EQ(num_bands_, bands->num_bands());
    DBG_ASSERT_EQ(data->num_channels(), bands->num_channels());
    DBG_ASSERT_EQ(data->num_frames(), bands->num_frames_per_band() * bands->num_bands());

    if (bands->num_bands() == 2)
    {
        TwoBandsAnalysis(data, bands);
    }
    else if (bands->num_bands() == 3)
    {
        ThreeBandsAnalysis(data, bands);
    }
}

void SplittingFilter::Synthesis(const ChannelBuffer<float> *bands, ChannelBuffer<float> *data)
{
    DBG_ASSERT_EQ(num_bands_, bands->num_bands());
    DBG_ASSERT_EQ(data->num_channels(), bands->num_channels());
    DBG_ASSERT_EQ(data->num_frames(), bands->num_frames_per_band() * bands->num_bands());

    if (bands->num_bands() == 2)
    {
        TwoBandsSynthesis(bands, data);
    }
    else if (bands->num_bands() == 3)
    {
        ThreeBandsSynthesis(bands, data);
    }
}

void SplittingFilter::TwoBandsAnalysis(const ChannelBuffer<float> *data, ChannelBuffer<float> *bands)
{
    DBG_ASSERT_EQ(two_bands_states_.size(), data->num_channels());
    DBG_ASSERT_EQ(data->num_frames(), kTwoBandFilterSamplesPerFrame);

    for (size_t i = 0; i < two_bands_states_.size(); ++i)
    {
        two_bands_states_[i].Analysis(ArrayView<const float, TwoBandFilterBank::kFullBandSize>(
                                          data->channels_view()[i].data(), TwoBandFilterBank::kFullBandSize),
                                      ArrayView<const ArrayView<float>, TwoBandFilterBank::kNumBands>(
                                          bands->bands_view(i).data(), TwoBandFilterBank::kNumBands));
    }
}

void SplittingFilter::TwoBandsSynthesis(const ChannelBuffer<float> *bands, ChannelBuffer<float> *data)
{
    DBG_ASSERT_EQ(data->num_channels(), two_bands_states_.size());
    DBG_ASSERT_EQ(data->num_frames(), kTwoBandFilterSamplesPerFrame);

    for (size_t i = 0; i < data->num_channels(); ++i)
    {
        two_bands_states_[i].Synthesis(ArrayView<const ArrayView<float>, TwoBandFilterBank::kNumBands>(
                                           bands->bands_view(i).data(), TwoBandFilterBank::kNumBands),
                                       ArrayView<float, TwoBandFilterBank::kFullBandSize>(
                                           data->channels_view()[i].data(), TwoBandFilterBank::kFullBandSize));
    }
}

void SplittingFilter::ThreeBandsAnalysis(const ChannelBuffer<float> *data, ChannelBuffer<float> *bands)
{
    DBG_ASSERT_EQ(three_band_filter_banks_.size(), data->num_channels());
    DBG_ASSERT_LE(data->num_channels(), three_band_filter_banks_.size());
    DBG_ASSERT_LE(data->num_channels(), bands->num_channels());
    DBG_ASSERT_EQ(data->num_frames(), ThreeBandFilterBank::kFullBandSize);
    DBG_ASSERT_EQ(bands->num_frames(), ThreeBandFilterBank::kFullBandSize);
    DBG_ASSERT_EQ(bands->num_bands(), ThreeBandFilterBank::kNumBands);
    DBG_ASSERT_EQ(bands->num_frames_per_band(), ThreeBandFilterBank::kSplitBandSize);

    for (size_t i = 0; i < three_band_filter_banks_.size(); ++i)
    {
        three_band_filter_banks_[i].Analysis(ArrayView<const float, ThreeBandFilterBank::kFullBandSize>(
                                                 data->channels_view()[i].data(), ThreeBandFilterBank::kFullBandSize),
                                             ArrayView<const ArrayView<float>, ThreeBandFilterBank::kNumBands>(
                                                 bands->bands_view(i).data(), ThreeBandFilterBank::kNumBands));
    }
}

void SplittingFilter::ThreeBandsSynthesis(const ChannelBuffer<float> *bands, ChannelBuffer<float> *data)
{
    DBG_ASSERT_LE(data->num_channels(), three_band_filter_banks_.size());
    DBG_ASSERT_LE(data->num_channels(), bands->num_channels());
    DBG_ASSERT_LE(data->num_channels(), three_band_filter_banks_.size());
    DBG_ASSERT_EQ(data->num_frames(), ThreeBandFilterBank::kFullBandSize);
    DBG_ASSERT_EQ(bands->num_frames(), ThreeBandFilterBank::kFullBandSize);
    DBG_ASSERT_EQ(bands->num_bands(), ThreeBandFilterBank::kNumBands);
    DBG_ASSERT_EQ(bands->num_frames_per_band(), ThreeBandFilterBank::kSplitBandSize);

    for (size_t i = 0; i < data->num_channels(); ++i)
    {
        three_band_filter_banks_[i].Synthesis(ArrayView<const ArrayView<float>, ThreeBandFilterBank::kNumBands>(
                                                  bands->bands_view(i).data(), ThreeBandFilterBank::kNumBands),
                                              ArrayView<float, ThreeBandFilterBank::kFullBandSize>(
                                                  data->channels_view()[i].data(), ThreeBandFilterBank::kFullBandSize));
    }
}