#ifndef AUDIO_PROCESSING_NS_ESTIMATOR_H
#define AUDIO_PROCESSING_NS_ESTIMATOR_H

#include "ns_define.h"
#include <cmath>

constexpr int kHistogramSize = 1000;
constexpr int kSimult = 3;

struct SignalModel
{
    SignalModel() : lrt(0.5f), spectral_diff(0.5f), spectral_flatness(0.5f)
    {
        avg_log_lrt.fill(0.0f);
    }

    SignalModel(const SignalModel &) = delete;
    SignalModel &operator=(const SignalModel &) = delete;

    float lrt;
    float spectral_diff;
    float spectral_flatness;
    // Log LRT factor with time-smoothing.
    std::array<float, kFftSizeBy2Plus1> avg_log_lrt;
};

// Struct for storing the prior signal model parameters.
struct PriorSignalModel
{
    explicit PriorSignalModel(float lrt_initial_value) : lrt(lrt_initial_value)
    {
    }

    PriorSignalModel(const PriorSignalModel &) = delete;
    PriorSignalModel &operator=(const PriorSignalModel &) = delete;

    float lrt;
    float flatness_threshold = .5f;
    float template_diff_threshold = .5f;
    float lrt_weighting = 1.f;
    float flatness_weighting = 0.f;
    float difference_weighting = 0.f;
};

// Class for handling the updating of histograms.
class Histograms
{
  public:
    Histograms()
    {
        Clear();
    }

    Histograms(const Histograms &) = delete;
    Histograms &operator=(const Histograms &) = delete;

    // Clears the histograms.
    void Clear();

    // Extracts thresholds for feature parameters and updates the corresponding
    // histogram.
    void Update(const SignalModel &features_);

    // Methods for accessing the histograms.
    ArrayView<const int, kHistogramSize> get_lrt() const
    {
        return lrt_;
    }

    ArrayView<const int, kHistogramSize> get_spectral_flatness() const
    {
        return spectral_flatness_;
    }

    ArrayView<const int, kHistogramSize> get_spectral_diff() const
    {
        return spectral_diff_;
    }

  private:
    std::array<int, kHistogramSize> lrt_;
    std::array<int, kHistogramSize> spectral_flatness_;
    std::array<int, kHistogramSize> spectral_diff_;
};

// Estimator of the prior signal model parameters.
class PriorSignalModelEstimator
{
  public:
    explicit PriorSignalModelEstimator(float lrt_initial_value) : prior_model_(lrt_initial_value)
    {
    }

    PriorSignalModelEstimator(const PriorSignalModelEstimator &) = delete;
    PriorSignalModelEstimator &operator=(const PriorSignalModelEstimator &) = delete;

    // Updates the model estimate.
    void Update(const Histograms &h);

    // Returns the estimated model.
    const PriorSignalModel &get_prior_model() const
    {
        return prior_model_;
    }

  private:
    PriorSignalModel prior_model_;
};

class SignalModelEstimator
{
  public:
    SignalModelEstimator() : prior_model_estimator_(0.5f)
    {
    }

    SignalModelEstimator(const SignalModelEstimator &) = delete;
    SignalModelEstimator &operator=(const SignalModelEstimator &) = delete;

    // Compute signal normalization during the initial startup phase.
    void AdjustNormalization(int32_t num_analyzed_frames, float signal_energy)
    {
        diff_normalization_ *= num_analyzed_frames;
        diff_normalization_ += signal_energy;
        diff_normalization_ /= (num_analyzed_frames + 1);
    }

    void Update(ArrayView<const float, kFftSizeBy2Plus1> prior_snr, ArrayView<const float, kFftSizeBy2Plus1> post_snr,
                ArrayView<const float, kFftSizeBy2Plus1> conservative_noise_spectrum,
                ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum, float signal_spectral_sum,
                float signal_energy);

    const PriorSignalModel &get_prior_model() const
    {
        return prior_model_estimator_.get_prior_model();
    }
    const SignalModel &get_model()
    {
        return features_;
    }

  private:
    float diff_normalization_ = 0.f;
    float signal_energy_sum_ = 0.f;
    Histograms histograms_;
    int histogram_analysis_counter_ = 500;
    PriorSignalModelEstimator prior_model_estimator_;
    SignalModel features_;
};

// Class for estimating the probability of speech.
class SpeechProbabilityEstimator
{
  public:
    SpeechProbabilityEstimator()
    {
        speech_probability_.fill(0.f);
    }

    SpeechProbabilityEstimator(const SpeechProbabilityEstimator &) = delete;
    SpeechProbabilityEstimator &operator=(const SpeechProbabilityEstimator &) = delete;

    // Compute speech probability.
    void Update(int32_t num_analyzed_frames, ArrayView<const float, kFftSizeBy2Plus1> prior_snr,
                ArrayView<const float, kFftSizeBy2Plus1> post_snr,
                ArrayView<const float, kFftSizeBy2Plus1> conservative_noise_spectrum,
                ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum, float signal_spectral_sum,
                float signal_energy);

    float get_prior_probability() const
    {
        return prior_speech_prob_;
    }

    ArrayView<const float> get_probability()
    {
        return speech_probability_;
    }

  private:
    SignalModelEstimator signal_model_estimator_;
    float prior_speech_prob_ = .5f;
    std::array<float, kFftSizeBy2Plus1> speech_probability_;
};

// For quantile noise estimation.
class QuantileNoiseEstimator
{
  public:
    QuantileNoiseEstimator();
    QuantileNoiseEstimator(const QuantileNoiseEstimator &) = delete;
    QuantileNoiseEstimator &operator=(const QuantileNoiseEstimator &) = delete;

    // Estimate noise.
    void Estimate(ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum,
                  ArrayView<float, kFftSizeBy2Plus1> noise_spectrum);

  private:
    std::array<float, kSimult * kFftSizeBy2Plus1> density_;
    std::array<float, kSimult * kFftSizeBy2Plus1> log_quantile_;
    std::array<float, kFftSizeBy2Plus1> quantile_;
    std::array<int, kSimult> counter_;
    int num_updates_ = 1;
};

// Class for estimating the spectral characteristics of the noise in an incoming
// signal.
class NoiseEstimator
{
  public:
    explicit NoiseEstimator(const SuppressionParams &suppression_params);

    // Prepare the estimator for analysis of a new frame.
    void PrepareAnalysis();

    // Performs the first step of the estimator update.
    void PreUpdate(int32_t num_analyzed_frames, ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum,
                   float signal_spectral_sum);

    // Performs the second step of the estimator update.
    void PostUpdate(ArrayView<const float> speech_probability,
                    ArrayView<const float, kFftSizeBy2Plus1> signal_spectrum);

    // Returns the noise spectral estimate.
    ArrayView<const float, kFftSizeBy2Plus1> get_noise_spectrum() const
    {
        return noise_spectrum_;
    }

    // Returns the noise from the previous frame.
    ArrayView<const float, kFftSizeBy2Plus1> get_prev_noise_spectrum() const
    {
        return prev_noise_spectrum_;
    }

    // Returns a noise spectral estimate based on white and pink noise parameters.
    ArrayView<const float, kFftSizeBy2Plus1> get_parametric_noise_spectrum() const
    {
        return parametric_noise_spectrum_;
    }
    ArrayView<const float, kFftSizeBy2Plus1> get_conservative_noise_spectrum() const
    {
        return conservative_noise_spectrum_;
    }

  private:
    const SuppressionParams &suppression_params_;
    float white_noise_level_ = 0.f;
    float pink_noise_numerator_ = 0.f;
    float pink_noise_exp_ = 0.f;
    std::array<float, kFftSizeBy2Plus1> prev_noise_spectrum_;
    std::array<float, kFftSizeBy2Plus1> conservative_noise_spectrum_;
    std::array<float, kFftSizeBy2Plus1> parametric_noise_spectrum_;
    std::array<float, kFftSizeBy2Plus1> noise_spectrum_;
    QuantileNoiseEstimator quantile_noise_estimator_;
};

#endif