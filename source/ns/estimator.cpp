#include "estimator.h"

static constexpr float kOneByFftSizeBy2Plus1 = 1.f / kFftSizeBy2Plus1;

void Histograms::Clear()
{
    lrt_.fill(0);
    spectral_flatness_.fill(0);
    spectral_diff_.fill(0);
}

void Histograms::Update(const SignalModel &features_)
{
    // Update the histogram for the LRT.
    constexpr float kOneByBinSizeLrt = 1.f / kBinSizeLrt;
    if (features_.lrt < kHistogramSize * kBinSizeLrt && features_.lrt >= 0.f)
    {
        ++lrt_[static_cast<int>(kOneByBinSizeLrt * features_.lrt)];
    }

    // Update histogram for the spectral flatness.
    constexpr float kOneByBinSizeSpecFlat = 1.f / kBinSizeSpecFlat;
    if (features_.spectral_flatness < kHistogramSize * kBinSizeSpecFlat && features_.spectral_flatness >= 0.f)
    {
        ++spectral_flatness_[static_cast<int>(features_.spectral_flatness * kOneByBinSizeSpecFlat)];
    }

    // Update histogram for the spectral difference.
    constexpr float kOneByBinSizeSpecDiff = 1.f / kBinSizeSpecDiff;
    if (features_.spectral_diff < kHistogramSize * kBinSizeSpecDiff && features_.spectral_diff >= 0.f)
    {
        ++spectral_diff_[static_cast<int>(features_.spectral_diff * kOneByBinSizeSpecDiff)];
    }
}

// Identifies the first of the two largest peaks in the histogram.
static void FindFirstOfTwoLargestPeaks(float bin_size, ArrayView<const int, kHistogramSize> spectral_flatness,
                                       float *peak_position, int *peak_weight)
{
    int peak_value = 0;
    int secondary_peak_value = 0;
    *peak_position = 0.f;
    float secondary_peak_position = 0.f;
    *peak_weight = 0;
    int secondary_peak_weight = 0;

    // Identify the two largest peaks.
    for (int i = 0; i < kHistogramSize; ++i)
    {
        const float bin_mid = (i + 0.5f) * bin_size;
        if (spectral_flatness[i] > peak_value)
        {
            // Found new "first" peak candidate.
            secondary_peak_value = peak_value;
            secondary_peak_weight = *peak_weight;
            secondary_peak_position = *peak_position;

            peak_value = spectral_flatness[i];
            *peak_weight = spectral_flatness[i];
            *peak_position = bin_mid;
        }
        else if (spectral_flatness[i] > secondary_peak_value)
        {
            // Found new "second" peak candidate.
            secondary_peak_value = spectral_flatness[i];
            secondary_peak_weight = spectral_flatness[i];
            secondary_peak_position = bin_mid;
        }
    }

    // Merge the peaks if they are close.
    if ((fabs(secondary_peak_position - *peak_position) < 2 * bin_size) &&
        (secondary_peak_weight > 0.5f * (*peak_weight)))
    {
        *peak_weight += secondary_peak_weight;
        *peak_position = 0.5f * (*peak_position + secondary_peak_position);
    }
}

static void UpdateLrt(ArrayView<const int, kHistogramSize> lrt_histogram, float *prior_model_lrt,
                      bool *low_lrt_fluctuations)
{
    float average = 0.f;
    float average_compl = 0.f;
    float average_squared = 0.f;
    int count = 0;

    for (int i = 0; i < 10; ++i)
    {
        float bin_mid = (i + 0.5f) * kBinSizeLrt;
        average += lrt_histogram[i] * bin_mid;
        count += lrt_histogram[i];
    }
    if (count > 0)
    {
        average = average / count;
    }

    for (int i = 0; i < kHistogramSize; ++i)
    {
        float bin_mid = (i + 0.5f) * kBinSizeLrt;
        average_squared += lrt_histogram[i] * bin_mid * bin_mid;
        average_compl += lrt_histogram[i] * bin_mid;
    }
    constexpr float kOneFeatureUpdateWindowSize = 1.f / kFeatureUpdateWindowSize;
    average_squared = average_squared * kOneFeatureUpdateWindowSize;
    average_compl = average_compl * kOneFeatureUpdateWindowSize;

    // Fluctuation limit of LRT feature.
    *low_lrt_fluctuations = average_squared - average * average_compl < 0.05f;

    // Get threshold for LRT feature.
    constexpr float kMaxLrt = 1.f;
    constexpr float kMinLrt = .2f;
    if (*low_lrt_fluctuations)
    {
        // Very low fluctuation, so likely noise.
        *prior_model_lrt = kMaxLrt;
    }
    else
    {
        *prior_model_lrt = std::min(kMaxLrt, std::max(kMinLrt, 1.2f * average));
    }
}

// Extract thresholds for feature parameters and computes the threshold/weights.
void PriorSignalModelEstimator::Update(const Histograms &histograms)
{
    bool low_lrt_fluctuations;
    UpdateLrt(histograms.get_lrt(), &prior_model_.lrt, &low_lrt_fluctuations);

    // For spectral flatness and spectral difference: compute the main peaks of
    // the histograms.
    float spectral_flatness_peak_position;
    int spectral_flatness_peak_weight;
    FindFirstOfTwoLargestPeaks(kBinSizeSpecFlat, histograms.get_spectral_flatness(), &spectral_flatness_peak_position,
                               &spectral_flatness_peak_weight);

    float spectral_diff_peak_position = 0.f;
    int spectral_diff_peak_weight = 0;
    FindFirstOfTwoLargestPeaks(kBinSizeSpecDiff, histograms.get_spectral_diff(), &spectral_diff_peak_position,
                               &spectral_diff_peak_weight);

    // Reject if weight of peaks is not large enough, or peak value too small.
    // Peak limit for spectral flatness (varies between 0 and 1).
    const int use_spec_flat =
        spectral_flatness_peak_weight < 0.3f * 500 || spectral_flatness_peak_position < 0.6f ? 0 : 1;

    // Reject if weight of peaks is not large enough or if fluctuation of the LRT
    // feature are very low, indicating a noise state.
    const int use_spec_diff = spectral_diff_peak_weight < 0.3f * 500 || low_lrt_fluctuations ? 0 : 1;

    // Update the model.
    prior_model_.template_diff_threshold = 1.2f * spectral_diff_peak_position;
    prior_model_.template_diff_threshold = std::min(1.f, std::max(0.16f, prior_model_.template_diff_threshold));

    float one_by_feature_sum = 1.f / (1.f + use_spec_flat + use_spec_diff);
    prior_model_.lrt_weighting = one_by_feature_sum;

    if (use_spec_flat == 1)
    {
        prior_model_.flatness_threshold = 0.9f * spectral_flatness_peak_position;
        prior_model_.flatness_threshold = std::min(.95f, std::max(0.1f, prior_model_.flatness_threshold));
        prior_model_.flatness_weighting = one_by_feature_sum;
    }
    else
    {
        prior_model_.flatness_weighting = 0.f;
    }

    if (use_spec_diff == 1)
    {
        prior_model_.difference_weighting = one_by_feature_sum;
    }
    else
    {
        prior_model_.difference_weighting = 0.f;
    }
}

// Computes the difference measure between input spectrum and a template/learned
// noise spectrum.
float ComputeSpectralDiff(ArrayView<const float, kFftSizeBy2Plus1> conservative_noise_spectrum,
                          ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum, float signal_spectral_sum,
                          float diff_normalization)
{
    // spectral_diff = var(signal_spectrum) - cov(signal_spectrum, magnAvgPause)^2
    // / var(magnAvgPause)

    // Compute average quantities.
    float noise_average = 0.f;
    for (size_t i = 0; i < kFftSizeBy2Plus1; ++i)
    {
        // Conservative smooth noise spectrum from pause frames.
        noise_average += conservative_noise_spectrum[i];
    }
    noise_average = noise_average * kOneByFftSizeBy2Plus1;
    float signal_average = signal_spectral_sum * kOneByFftSizeBy2Plus1;

    // Compute variance and covariance quantities.
    float covariance = 0.f;
    float noise_variance = 0.f;
    float signal_variance = 0.f;
    for (size_t i = 0; i < kFftSizeBy2Plus1; ++i)
    {
        float signal_diff = signal_spectrum[i] - signal_average;
        float noise_diff = conservative_noise_spectrum[i] - noise_average;
        covariance += signal_diff * noise_diff;
        noise_variance += noise_diff * noise_diff;
        signal_variance += signal_diff * signal_diff;
    }
    covariance *= kOneByFftSizeBy2Plus1;
    noise_variance *= kOneByFftSizeBy2Plus1;
    signal_variance *= kOneByFftSizeBy2Plus1;

    // Update of average magnitude spectrum.
    float spectral_diff = signal_variance - (covariance * covariance) / (noise_variance + 0.0001f);
    // Normalize.
    return spectral_diff / (diff_normalization + 0.0001f);
}

// Updates the spectral flatness based on the input spectrum.
void UpdateSpectralFlatness(ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum, float signal_spectral_sum,
                            float *spectral_flatness)
{
    // Compute log of ratio of the geometric to arithmetic mean (handle the log(0)
    // separately).
    constexpr float kAveraging = 0.3f;
    float avg_spect_flatness_num = 0.f;
    for (size_t i = 1; i < kFftSizeBy2Plus1; ++i)
    {
        if (signal_spectrum[i] == 0.f)
        {
            *spectral_flatness -= kAveraging * (*spectral_flatness);
            return;
        }
    }

    for (size_t i = 1; i < kFftSizeBy2Plus1; ++i)
    {
        avg_spect_flatness_num += LogApproximation(signal_spectrum[i]);
    }

    float avg_spect_flatness_denom = signal_spectral_sum - signal_spectrum[0];

    avg_spect_flatness_denom = avg_spect_flatness_denom * kOneByFftSizeBy2Plus1;
    avg_spect_flatness_num = avg_spect_flatness_num * kOneByFftSizeBy2Plus1;

    float spectral_tmp = ExpApproximation(avg_spect_flatness_num) / avg_spect_flatness_denom;

    // Time-avg update of spectral flatness feature.
    *spectral_flatness += kAveraging * (spectral_tmp - *spectral_flatness);
}

// Updates the log LRT measures.
void UpdateSpectralLrt(ArrayView<const float, kFftSizeBy2Plus1> prior_snr,
                       ArrayView<const float, kFftSizeBy2Plus1> post_snr,
                       ArrayView<float, kFftSizeBy2Plus1> avg_log_lrt, float *lrt)
{
    for (size_t i = 0; i < kFftSizeBy2Plus1; ++i)
    {
        float tmp1 = 1.f + 2.f * prior_snr[i];
        float tmp2 = 2.f * prior_snr[i] / (tmp1 + 0.0001f);
        float bessel_tmp = (post_snr[i] + 1.f) * tmp2;
        avg_log_lrt[i] += .5f * (bessel_tmp - LogApproximation(tmp1) - avg_log_lrt[i]);
    }

    float log_lrt_time_avg_k_sum = 0.f;
    for (size_t i = 0; i < kFftSizeBy2Plus1; ++i)
    {
        log_lrt_time_avg_k_sum += avg_log_lrt[i];
    }
    *lrt = log_lrt_time_avg_k_sum * kOneByFftSizeBy2Plus1;
}

// Update the noise features.
void SignalModelEstimator::Update(ArrayView<const float, kFftSizeBy2Plus1> prior_snr,
                                  ArrayView<const float, kFftSizeBy2Plus1> post_snr,
                                  ArrayView<const float, kFftSizeBy2Plus1> conservative_noise_spectrum,
                                  ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum, float signal_spectral_sum,
                                  float signal_energy)
{
    // Compute spectral flatness on input spectrum.
    UpdateSpectralFlatness(signal_spectrum, signal_spectral_sum, &features_.spectral_flatness);

    // Compute difference of input spectrum with learned/estimated noise spectrum.
    float spectral_diff =
        ComputeSpectralDiff(conservative_noise_spectrum, signal_spectrum, signal_spectral_sum, diff_normalization_);
    // Compute time-avg update of difference feature.
    features_.spectral_diff += 0.3f * (spectral_diff - features_.spectral_diff);

    signal_energy_sum_ += signal_energy;

    // Compute histograms for parameter decisions (thresholds and weights for
    // features). Parameters are extracted periodically.
    if (--histogram_analysis_counter_ > 0)
    {
        histograms_.Update(features_);
    }
    else
    {
        // Compute model parameters.
        prior_model_estimator_.Update(histograms_);

        // Clear histograms for next update.
        histograms_.Clear();

        histogram_analysis_counter_ = kFeatureUpdateWindowSize;

        // Update every window:
        // Compute normalization for the spectral difference for next estimation.
        signal_energy_sum_ = signal_energy_sum_ / kFeatureUpdateWindowSize;
        diff_normalization_ = 0.5f * (signal_energy_sum_ + diff_normalization_);
        signal_energy_sum_ = 0.f;
    }

    // Compute the LRT.
    UpdateSpectralLrt(prior_snr, post_snr, features_.avg_log_lrt, &features_.lrt);
}

void SpeechProbabilityEstimator::Update(int32_t num_analyzed_frames, ArrayView<const float, kFftSizeBy2Plus1> prior_snr,
                                        ArrayView<const float, kFftSizeBy2Plus1> post_snr,
                                        ArrayView<const float, kFftSizeBy2Plus1> conservative_noise_spectrum,
                                        ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum,
                                        float signal_spectral_sum, float signal_energy)
{
    // Update models.
    if (num_analyzed_frames < kLongStartupPhaseBlocks)
    {
        signal_model_estimator_.AdjustNormalization(num_analyzed_frames, signal_energy);
    }
    signal_model_estimator_.Update(prior_snr, post_snr, conservative_noise_spectrum, signal_spectrum,
                                   signal_spectral_sum, signal_energy);

    const SignalModel &model = signal_model_estimator_.get_model();
    const PriorSignalModel &prior_model = signal_model_estimator_.get_prior_model();

    // Width parameter in sigmoid map for prior model.
    constexpr float kWidthPrior0 = 4.f;
    // Width for pause region: lower range, so increase width in tanh map.
    constexpr float kWidthPrior1 = 2.f * kWidthPrior0;

    // Average LRT feature: use larger width in tanh map for pause regions.
    float width_prior = model.lrt < prior_model.lrt ? kWidthPrior1 : kWidthPrior0;

    // Compute indicator function: sigmoid map.
    float indicator0 = 0.5f * (tanhf(width_prior * (model.lrt - prior_model.lrt)) + 1.f);

    // Spectral flatness feature: use larger width in tanh map for pause regions.
    width_prior = model.spectral_flatness > prior_model.flatness_threshold ? kWidthPrior1 : kWidthPrior0;

    // Compute indicator function: sigmoid map.
    float indicator1 =
        0.5f * (tanhf(1.f * width_prior * (prior_model.flatness_threshold - model.spectral_flatness)) + 1.f);

    // For template spectrum-difference : use larger width in tanh map for pause
    // regions.
    width_prior = model.spectral_diff < prior_model.template_diff_threshold ? kWidthPrior1 : kWidthPrior0;

    // Compute indicator function: sigmoid map.
    float indicator2 = 0.5f * (tanhf(width_prior * (model.spectral_diff - prior_model.template_diff_threshold)) + 1.f);

    // Combine the indicator function with the feature weights.
    float ind_prior = prior_model.lrt_weighting * indicator0 + prior_model.flatness_weighting * indicator1 +
                      prior_model.difference_weighting * indicator2;

    // Compute the prior probability.
    prior_speech_prob_ += 0.1f * (ind_prior - prior_speech_prob_);

    // Make sure probabilities are within range: keep floor to 0.01.
    prior_speech_prob_ = std::max(std::min(prior_speech_prob_, 1.f), 0.01f);

    // Final speech probability: combine prior model with LR factor:.
    float gain_prior = (1.f - prior_speech_prob_) / (prior_speech_prob_ + 0.0001f);

    std::array<float, kFftSizeBy2Plus1> inv_lrt;
    ExpApproximationSignFlip(model.avg_log_lrt, inv_lrt);
    for (size_t i = 0; i < kFftSizeBy2Plus1; ++i)
    {
        speech_probability_[i] = 1.f / (1.f + gain_prior * inv_lrt[i]);
    }
}

QuantileNoiseEstimator::QuantileNoiseEstimator()
{
    quantile_.fill(0.f);
    density_.fill(0.3f);
    log_quantile_.fill(8.f);

    constexpr float kOneBySimult = 1.f / kSimult;
    for (size_t i = 0; i < kSimult; ++i)
    {
        counter_[i] = static_cast<int>(floor(kLongStartupPhaseBlocks * (i + 1.f) * kOneBySimult));
    }
}

void QuantileNoiseEstimator::Estimate(ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum,
                                      ArrayView<float, kFftSizeBy2Plus1> noise_spectrum)
{
    std::array<float, kFftSizeBy2Plus1> log_spectrum;
    LogApproximation(signal_spectrum, log_spectrum);

    int quantile_index_to_return = -1;
    // Loop over simultaneous estimates.
    for (int s = 0, k = 0; s < kSimult; ++s, k += static_cast<int>(kFftSizeBy2Plus1))
    {
        const float one_by_counter_plus_1 = 1.f / (counter_[s] + 1.f);
        for (int i = 0, j = k; i < static_cast<int>(kFftSizeBy2Plus1); ++i, ++j)
        {
            // Update log quantile estimate.
            const float delta = density_[j] > 1.f ? 40.f / density_[j] : 40.f;

            const float multiplier = delta * one_by_counter_plus_1;
            if (log_spectrum[i] > log_quantile_[j])
            {
                log_quantile_[j] += 0.25f * multiplier;
            }
            else
            {
                log_quantile_[j] -= 0.75f * multiplier;
            }

            // Update density estimate.
            constexpr float kWidth = 0.01f;
            constexpr float kOneByWidthPlus2 = 1.f / (2.f * kWidth);
            if (fabs(log_spectrum[i] - log_quantile_[j]) < kWidth)
            {
                density_[j] = (counter_[s] * density_[j] + kOneByWidthPlus2) * one_by_counter_plus_1;
            }
        }

        if (counter_[s] >= kLongStartupPhaseBlocks)
        {
            counter_[s] = 0;
            if (num_updates_ >= kLongStartupPhaseBlocks)
            {
                quantile_index_to_return = k;
            }
        }

        ++counter_[s];
    }

    // Sequentially update the noise during startup.
    if (num_updates_ < kLongStartupPhaseBlocks)
    {
        // Use the last "s" to get noise during startup that differ from zero.
        quantile_index_to_return = kFftSizeBy2Plus1 * (kSimult - 1);
        ++num_updates_;
    }

    if (quantile_index_to_return >= 0)
    {
        ExpApproximation(ArrayView<const float>(&log_quantile_[quantile_index_to_return], kFftSizeBy2Plus1), quantile_);
    }

    std::copy(quantile_.begin(), quantile_.end(), noise_spectrum.begin());
}

// Log(i).
static constexpr std::array<float, 129> log_table = {
    0.f,       0.f,       0.f,       0.f,       0.f,       1.609438f, 1.791759f, 1.945910f, 2.079442f, 2.197225f,
    2.302585f, 2.397895f, 2.484907f, 2.564949f, 2.639057f, 2.708050f, 2.772589f, 2.833213f, 2.890372f, 2.944439f,
    2.995732f, 3.044522f, 3.091043f, 3.135494f, 3.178054f, 3.218876f, 3.258097f, 3.295837f, 3.332205f, 3.367296f,
    3.401197f, 3.433987f, 3.465736f, 3.496507f, 3.526361f, 3.555348f, 3.583519f, 3.610918f, 3.637586f, 3.663562f,
    3.688879f, 3.713572f, 3.737669f, 3.761200f, 3.784190f, 3.806663f, 3.828641f, 3.850147f, 3.871201f, 3.891820f,
    3.912023f, 3.931826f, 3.951244f, 3.970292f, 3.988984f, 4.007333f, 4.025352f, 4.043051f, 4.060443f, 4.077538f,
    4.094345f, 4.110874f, 4.127134f, 4.143135f, 4.158883f, 4.174387f, 4.189655f, 4.204693f, 4.219508f, 4.234107f,
    4.248495f, 4.262680f, 4.276666f, 4.290460f, 4.304065f, 4.317488f, 4.330733f, 4.343805f, 4.356709f, 4.369448f,
    4.382027f, 4.394449f, 4.406719f, 4.418841f, 4.430817f, 4.442651f, 4.454347f, 4.465908f, 4.477337f, 4.488636f,
    4.499810f, 4.510859f, 4.521789f, 4.532599f, 4.543295f, 4.553877f, 4.564348f, 4.574711f, 4.584968f, 4.595119f,
    4.605170f, 4.615121f, 4.624973f, 4.634729f, 4.644391f, 4.653960f, 4.663439f, 4.672829f, 4.682131f, 4.691348f,
    4.700480f, 4.709530f, 4.718499f, 4.727388f, 4.736198f, 4.744932f, 4.753591f, 4.762174f, 4.770685f, 4.779124f,
    4.787492f, 4.795791f, 4.804021f, 4.812184f, 4.820282f, 4.828314f, 4.836282f, 4.844187f, 4.852030f};

NoiseEstimator::NoiseEstimator(const SuppressionParams &suppression_params) : suppression_params_(suppression_params)
{
    noise_spectrum_.fill(0.f);
    prev_noise_spectrum_.fill(0.f);
    conservative_noise_spectrum_.fill(0.f);
    parametric_noise_spectrum_.fill(0.f);
}

void NoiseEstimator::PrepareAnalysis()
{
    std::copy(noise_spectrum_.begin(), noise_spectrum_.end(), prev_noise_spectrum_.begin());
}

void NoiseEstimator::PreUpdate(int32_t num_analyzed_frames, ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum,
                               float signal_spectral_sum)
{
    quantile_noise_estimator_.Estimate(signal_spectrum, noise_spectrum_);

    if (num_analyzed_frames < kShortStartupPhaseBlocks)
    {
        // Compute simplified noise model during startup.
        const size_t kStartBand = 5;
        float sum_log_i_log_magn = 0.f;
        float sum_log_i = 0.f;
        float sum_log_i_square = 0.f;
        float sum_log_magn = 0.f;
        for (size_t i = kStartBand; i < kFftSizeBy2Plus1; ++i)
        {
            float log_i = log_table[i];
            sum_log_i += log_i;
            sum_log_i_square += log_i * log_i;
            float log_signal = LogApproximation(signal_spectrum[i]);
            sum_log_magn += log_signal;
            sum_log_i_log_magn += log_i * log_signal;
        }

        // Estimate the parameter for the level of the white noise.
        // constexpr float kOneByFftSizeBy2Plus1 = 1.f / kFftSizeBy2Plus1;
        white_noise_level_ += signal_spectral_sum * kOneByFftSizeBy2Plus1 * suppression_params_.over_subtraction_factor;

        // Estimate pink noise parameters.
        float denom = sum_log_i_square * (kFftSizeBy2Plus1 - kStartBand) - sum_log_i * sum_log_i;
        float num = sum_log_i_square * sum_log_magn - sum_log_i * sum_log_i_log_magn;
        float pink_noise_adjustment = num / denom;

        // Constrain the estimated spectrum to be positive.
        pink_noise_adjustment = std::max(pink_noise_adjustment, 0.f);
        pink_noise_numerator_ += pink_noise_adjustment;
        num = sum_log_i * sum_log_magn - (kFftSizeBy2Plus1 - kStartBand) * sum_log_i_log_magn;
        pink_noise_adjustment = num / denom;

        // Constrain the pink noise power to be in the interval [0, 1].
        pink_noise_adjustment = std::max(std::min(pink_noise_adjustment, 1.f), 0.f);

        pink_noise_exp_ += pink_noise_adjustment;

        const float one_by_num_analyzed_frames_plus_1 = 1.f / (num_analyzed_frames + 1.f);

        // Calculate the frequency-independent parts of parametric noise estimate.
        float parametric_exp = 0.f;
        float parametric_num = 0.f;
        if (pink_noise_exp_ > 0.f)
        {
            // Use pink noise estimate.
            parametric_num = ExpApproximation(pink_noise_numerator_ * one_by_num_analyzed_frames_plus_1);
            parametric_num *= num_analyzed_frames + 1.f;
            parametric_exp = pink_noise_exp_ * one_by_num_analyzed_frames_plus_1;
        }

        constexpr float kOneByShortStartupPhaseBlocks = 1.f / kShortStartupPhaseBlocks;
        for (size_t i = 0; i < kFftSizeBy2Plus1; ++i)
        {
            // Estimate the background noise using the white and pink noise
            // parameters.
            if (pink_noise_exp_ == 0.f)
            {
                // Use white noise estimate.
                parametric_noise_spectrum_[i] = white_noise_level_;
            }
            else
            {
                // Use pink noise estimate.
                float use_band = i < kStartBand ? static_cast<float>(kStartBand) : static_cast<float>(i);
                float parametric_denom = PowApproximation(use_band, parametric_exp);
                parametric_noise_spectrum_[i] = parametric_num / parametric_denom;
            }
        }

        // Weight quantile noise with modeled noise.
        for (size_t i = 0; i < kFftSizeBy2Plus1; ++i)
        {
            noise_spectrum_[i] *= num_analyzed_frames;
            float tmp = parametric_noise_spectrum_[i] * (kShortStartupPhaseBlocks - num_analyzed_frames);
            noise_spectrum_[i] += tmp * one_by_num_analyzed_frames_plus_1;
            noise_spectrum_[i] *= kOneByShortStartupPhaseBlocks;
        }
    }
}

void NoiseEstimator::PostUpdate(ArrayView<const float> speech_probability,
                                ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum)
{
    // Time-avg parameter for noise_spectrum update.
    constexpr float kNoiseUpdate = 0.9f;

    float gamma = kNoiseUpdate;
    for (size_t i = 0; i < kFftSizeBy2Plus1; ++i)
    {
        const float prob_speech = speech_probability[i];
        const float prob_non_speech = 1.f - prob_speech;

        // Temporary noise update used for speech frames if update value is less
        // than previous.
        float noise_update_tmp =
            gamma * prev_noise_spectrum_[i] +
            (1.f - gamma) * (prob_non_speech * signal_spectrum[i] + prob_speech * prev_noise_spectrum_[i]);

        // Time-constant based on speech/noise_spectrum state.
        float gamma_old = gamma;

        // Increase gamma for frame likely to be seech.
        constexpr float kProbRange = .2f;
        gamma = prob_speech > kProbRange ? .99f : kNoiseUpdate;

        // Conservative noise_spectrum update.
        if (prob_speech < kProbRange)
        {
            conservative_noise_spectrum_[i] += 0.05f * (signal_spectrum[i] - conservative_noise_spectrum_[i]);
        }

        // Noise_spectrum update.
        if (gamma == gamma_old)
        {
            noise_spectrum_[i] = noise_update_tmp;
        }
        else
        {
            noise_spectrum_[i] =
                gamma * prev_noise_spectrum_[i] +
                (1.f - gamma) * (prob_non_speech * signal_spectrum[i] + prob_speech * prev_noise_spectrum_[i]);
            // Allow for noise_spectrum update downwards: If noise_spectrum update
            // decreases the noise_spectrum, it is safe, so allow it to happen.
            noise_spectrum_[i] = std::min(noise_spectrum_[i], noise_update_tmp);
        }
    }
}