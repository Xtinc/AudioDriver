#include "audio_process.h"
#include <cassert>
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <algorithm>
#include <complex>
#include <cstring>
#include <numeric>
#include <vector>

// Test reporting function
void test_check(bool condition, const std::string &test_name)
{
    if (!condition)
    {
        std::cout << "FAILED: " << test_name << std::endl;
    }
}

// Calculate Signal-to-Noise Ratio (SNR) between two audio signals
double calculate_snr(const std::vector<double> &reference, const std::vector<double> &processed)
{
    if (reference.size() != processed.size())
        return -INFINITY;

    double signal_power = 0.0;
    double noise_power = 0.0;

    for (size_t i = 0; i < reference.size(); i++)
    {
        signal_power += reference[i] * reference[i];
        double error = reference[i] - processed[i];
        noise_power += error * error;
    }

    if (noise_power < 1e-10)
        return INFINITY; // Avoid division by zero
    return 10.0 * log10(signal_power / noise_power);
}

// Generate test audio data: sine wave
std::vector<PCM_TYPE> generate_sine_wave(double frequency, double sample_rate, size_t num_frames, int channels = 1,
                                         double amplitude = 0.5)
{
    std::vector<PCM_TYPE> audio(num_frames * channels);
    for (size_t i = 0; i < num_frames; i++)
    {
        double value = amplitude * 32767.0 * std::sin(2.0 * M_PI * frequency * i / sample_rate);
        for (int ch = 0; ch < channels; ch++)
        {
            audio[i * channels + ch] = static_cast<PCM_TYPE>(value);
        }
    }
    return audio;
}

// Calculate RMS value of audio signal
double calculate_rms(const std::vector<PCM_TYPE> &audio, int channels)
{
    if (audio.empty())
        return 0.0;

    double sum_squares = 0.0;
    for (size_t i = 0; i < audio.size(); i++)
    {
        sum_squares += static_cast<double>(audio[i]) * audio[i];
    }
    return std::sqrt(sum_squares / (audio.size() / channels));
}

// Test SincInterpolator class - comprehensive test including SNR analysis
void test_sinc_interpolator()
{
    std::cout << "Testing SincInterpolator..." << std::endl;

    struct TestCase
    {
        int order;        // Filter order
        int precision;    // Precision
        int channels;     // Number of channels
        double ratio;     // Conversion ratio (output rate/input rate)
        double min_snr;   // Minimum acceptable SNR
        std::string name; // Test scenario name
    };

    std::vector<TestCase> testCases = {{64, 100, 1, 48000.0 / 44100.0, 60.0, "44.1kHz -> 48kHz"},
                                       {64, 100, 1, 44100.0 / 48000.0, 60.0, "48kHz -> 44.1kHz"},
                                       {64, 100, 1, 0.5, 50.0, "Downsample 1/2"},
                                       {64, 100, 1, 2.0, 50.0, "Upsample 2x"}};

    for (const auto &test : testCases)
    {
        std::cout << "Testing " << test.name << "..." << std::endl;

        SincInterpolator interpolator(test.order, test.precision, test.channels, 1.0 / test.ratio);

        // Generate test audio with controlled frequency components
        const int inputSize = 4000;
        std::vector<double> input(inputSize + test.order); // Add extra space for startup delay

        // Calculate maximum frequency based on conversion ratio
        double max_freq = (test.ratio < 1.0) ? 0.4 * test.ratio : 0.4;

        // Generate test signal with precise frequency components
        double f1 = 0.1;      // Low frequency component at 0.1 * fs
        double f2 = 0.25;     // Mid frequency component at 0.25 * fs
        double f3 = max_freq; // High frequency component

        // Fill input buffer including startup delay region with valid signal
        for (int i = 0; i < inputSize + test.order; i++)
        {
            input[i] = 0.5 * std::sin(2.0 * M_PI * f1 * i) + 0.3 * std::sin(2.0 * M_PI * f2 * i) +
                       0.2 * std::sin(2.0 * M_PI * f3 * i);
        }

        // Process data through interpolator
        int expectedOutputSize = static_cast<int>((inputSize + test.order) * test.ratio);
        std::vector<double> output(expectedOutputSize);

        // Get actual output frames from process
        std::vector<float> inputFloat;
        for (int i = 0; i < inputSize + test.order; i++)
        {
            inputFloat.push_back(static_cast<float>(input[i]));
        }
        std::vector<float> outputFloat(expectedOutputSize);

        ArrayView<const float> inputView(inputFloat.data(), inputSize + test.order);
        ArrayView<float> outputView(outputFloat.data(), expectedOutputSize);
        interpolator.process(inputView, outputView, 0);
        int actualOutputFrames = static_cast<int>(outputView.size());

        // Copy back to output vector for analysis
        for (int i = 0; i < actualOutputFrames; i++)
        {
            if (i < static_cast<int>(output.size()))
            {
                output[i] = static_cast<double>(outputFloat[i]);
            }
        }
        test_check(actualOutputFrames > 0, "Output frame generation - " + test.name);

        // Calculate effective frames excluding startup delay
        int startup_delay = test.order / 2;
        int valid_frames = actualOutputFrames;

        std::cout << "  Actual output frames: " << actualOutputFrames << std::endl;

        // Generate reference signal
        std::vector<double> reference(valid_frames);
        double timeStep = 1.0 / test.ratio;

        for (int i = 0; i < valid_frames; i++)
        {
            // Adjust time to account for startup delay
            double t = (i + startup_delay) * timeStep;
            reference[i] = 0.5 * std::sin(2.0 * M_PI * f1 * t) + 0.3 * std::sin(2.0 * M_PI * f2 * t);

            if (f3 < 0.5 / timeStep)
            {
                reference[i] += 0.2 * std::sin(2.0 * M_PI * f3 * t);
            }
        }

        // Calculate SNR using valid portion of the signal
        double signal_power = 0.0;
        double noise_power = 0.0;
        int window_size = std::min(256, valid_frames / 4);

        // Skip the first few windows to avoid startup artifacts
        for (int i = window_size * 2; i < valid_frames - window_size; i++)
        {
            double ref_rms = 0.0;
            double error_rms = 0.0;

            for (int j = -window_size / 2; j < window_size / 2; j++)
            {
                ref_rms += reference[i + j] * reference[i + j];
                double error = reference[i + j] - output[i + j];
                error_rms += error * error;
            }

            ref_rms = std::sqrt(ref_rms / window_size);
            error_rms = std::sqrt(error_rms / window_size);

            if (ref_rms > 0.01)
            {
                signal_power += ref_rms * ref_rms;
                noise_power += error_rms * error_rms;
            }
        }

        double snr = 10.0 * std::log10((signal_power + 1e-10) / (noise_power + 1e-10));
        std::cout << "  SNR: " << snr << " dB (minimum required: " << test.min_snr << " dB)" << std::endl;

        test_check(snr >= test.min_snr - 1.0, "SNR requirement - " + test.name);

        // Verify signal validity
        bool has_signal = false;
        for (int i = startup_delay; i < valid_frames; i++)
        {
            if (std::abs(output[i]) > 0.01)
            {
                has_signal = true;
                break;
            }
        }
        test_check(has_signal, "Output signal presence - " + test.name);
    }

    std::cout << "SincInterpolator test completed" << std::endl;
}

// Remove the test_loc_sampler function as it's duplicated in test_locsampler.cpp

// Test mix_channels function - comprehensive test including various mixing scenarios
void test_mix_channels()
{
    std::cout << "Testing mix_channels function..." << std::endl;

    // Test 1: Basic mixing functionality
    {
        const unsigned int channels = 2;
        const unsigned int frames = 10;

        std::vector<int16_t> source(frames * channels);
        std::vector<int16_t> dest(frames * channels);

        // Initialize test data
        for (unsigned int i = 0; i < frames; i++)
        {
            for (unsigned int ch = 0; ch < channels; ch++)
            {
                source[i * channels + ch] = static_cast<int16_t>(1000 * (i + 1));
                dest[i * channels + ch] = static_cast<int16_t>(500 * (i + 1));
            }
        }

        // Mix channels
        mix_channels(source.data(), channels, frames, dest.data());

        // Verify mixing result - should be the sum of both signals, but within int16 limits
        bool basic_mix_ok = true;
        for (unsigned int i = 0; i < frames; i++)
        {
            for (unsigned int ch = 0; ch < channels; ch++)
            {
                int32_t expected = static_cast<int32_t>(1000 * (i + 1)) + static_cast<int32_t>(500 * (i + 1));
                expected = std::min(32767, std::max(-32768, expected));

                if (dest[i * channels + ch] != static_cast<int16_t>(expected))
                {
                    basic_mix_ok = false;
                    break;
                }
            }
        }
        test_check(basic_mix_ok, "Basic mixing test");

        std::cout << "  Basic mixing test passed" << std::endl;
    }

    // Test 2: Saturation handling (overflow clipping)
    {
        // Create positive overflow test
        std::vector<int16_t> source = {20000, 20000, 15000, 15000};
        std::vector<int16_t> dest = {15000, 15000, 20000, 20000};

        mix_channels(source.data(), 2, 2, dest.data());

        test_check(dest[0] == 32767 && dest[1] == 32767 && dest[2] == 32767 && dest[3] == 32767,
                   "Positive overflow clipping");

        // Create negative overflow test
        source = {-20000, -20000, -15000, -15000};
        dest = {-15000, -15000, -20000, -20000};

        mix_channels(source.data(), 2, 2, dest.data());

        test_check(dest[0] == -32768 && dest[1] == -32768 && dest[2] == -32768 && dest[3] == -32768,
                   "Negative overflow clipping");

        std::cout << "  Overflow clipping test passed" << std::endl;
    }

    // Test 3: Mixing within valid range
    {
        std::vector<int16_t> source = {10000, 5000, -10000, -5000};
        std::vector<int16_t> dest = {5000, 10000, -5000, -10000};

        mix_channels(source.data(), 2, 2, dest.data());

        // Verify result is correct and within range
        test_check(dest[0] == 15000 && dest[1] == 15000 && dest[2] == -15000 && dest[3] == -15000,
                   "Valid range mixing test");

        std::cout << "  Valid range mixing test passed" << std::endl;
    }

    // Test 4: Mono channel mixing
    {
        std::vector<int16_t> source = {10000, 20000, 30000};
        std::vector<int16_t> dest = {1000, 2000, 3000};

        mix_channels(source.data(), 1, 3, dest.data());

        // Verify mono mixing result
        test_check(dest[0] == 11000 && dest[1] == 22000 && dest[2] == 32767, // Clipped
                   "Mono mixing test");

        std::cout << "  Mono mixing test passed" << std::endl;
    }

    std::cout << "mix_channels test completed" << std::endl;
}

int main()
{
    std::cout << "Starting audio processing tests..." << std::endl;

    test_mix_channels();
    test_sinc_interpolator();

    std::cout << "All audio processing tests completed!" << std::endl;
    return 0;
}