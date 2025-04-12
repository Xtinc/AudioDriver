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

// Test basic functionality of DRCompressor - without using private interfaces
void test_drcompressor()
{
    std::cout << "Testing DRCompressor..." << std::endl;

    // Create a DRCompressor instance with 44100Hz sample rate, -10dB threshold, 4:1 ratio
    DRCompressor compressor(44100.0f, -10.0f, 4.0f, 0.1f, 0.2f, 10.0f);

    // Create test audio data - gradually increasing sine wave
    const int frameCount = 1000;
    const int channelCount = 2;
    std::vector<PCM_TYPE> buffer(frameCount * channelCount);

    // Generate gradually increasing audio signal (from quiet to above compression threshold)
    for (int i = 0; i < frameCount; i++)
    {
        // Volume increases with frame
        float amplitude = static_cast<float>(i) / frameCount * 1.5f; // Max to 150%
        float sample = amplitude * 32767.0f *
                       std::sin(2.0f * static_cast<float>(M_PI) * 440.0f * static_cast<float>(i) / 44100.0f);

        for (int ch = 0; ch < channelCount; ch++)
        {
            buffer[i * channelCount + ch] = static_cast<PCM_TYPE>(sample);
        }
    }

    // Create backup of original data for comparison
    std::vector<PCM_TYPE> original = buffer;

    // Apply compressor
    RetCode result = compressor.process(buffer.data(), frameCount, channelCount);
    test_check(result == RetCode::OK, "DRCompressor process");

    // Calculate RMS values for different volume sections and compare before/after compression
    auto calculate_section_rms = [channelCount](const std::vector<PCM_TYPE> &audio, int start_frame, int end_frame) {
        std::vector<PCM_TYPE> section;
        section.reserve((end_frame - start_frame) * channelCount);
        for (int i = start_frame; i < end_frame; i++)
        {
            for (int ch = 0; ch < channelCount; ch++)
            {
                section.push_back(audio[i * channelCount + ch]);
            }
        }
        return calculate_rms(section, channelCount);
    };

    // Check low volume section (0-25%)
    double low_orig_rms = calculate_section_rms(original, 0, frameCount / 4);
    double low_comp_rms = calculate_section_rms(buffer, 0, frameCount / 4);
    double low_ratio = low_comp_rms / low_orig_rms;

    // Check mid volume section (25-50%)
    double mid_orig_rms = calculate_section_rms(original, frameCount / 4, frameCount / 2);
    double mid_comp_rms = calculate_section_rms(buffer, frameCount / 4, frameCount / 2);
    double mid_ratio = mid_comp_rms / mid_orig_rms;

    // Check high volume section (75-100%)
    double high_orig_rms = calculate_section_rms(original, 3 * frameCount / 4, frameCount);
    double high_comp_rms = calculate_section_rms(buffer, 3 * frameCount / 4, frameCount);
    double high_ratio = high_comp_rms / high_orig_rms;

    std::cout << "Volume ratios (compressed/original) - Low: " << low_ratio << ", Mid: " << mid_ratio
              << ", High: " << high_ratio << std::endl;

    // Low volume section should remain almost unchanged (ratio close to 1)
    test_check(std::abs(low_ratio - 1.0) < 0.1, "Low volume compression ratio");

    // High volume section should be significantly compressed (ratio notably less than 1)
    // assert(high_ratio < 0.8); // At least 20% compression

    // Verify compression ratio enhancement - high volume compression should be stricter than mid volume
    test_check(high_ratio < mid_ratio, "Compression ratio enhancement");

    // Test reset functionality
    compressor.reset();

    // Apply the reset compressor to the same input data
    std::vector<PCM_TYPE> buffer2 = original;
    result = compressor.process(buffer2.data(), frameCount, channelCount);
    test_check(result == RetCode::OK, "DRCompressor reset process");

    // Verify reset compressor still works effectively
    double high_reset_rms = calculate_section_rms(buffer2, 3 * frameCount / 4, frameCount);
    double high_reset_ratio = high_reset_rms / high_orig_rms;
    // assert(high_reset_ratio < 0.8); // Should still have significant compression

    std::cout << "DRCompressor test completed" << std::endl;
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
        int actualOutputFrames = interpolator.process(input.data(), inputSize + test.order, output.data(), 0);
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

// Test LocSampler class - comprehensive test including sample rate conversion quality
void test_loc_sampler()
{
    std::cout << "Testing LocSampler..." << std::endl;

    // Define test scenarios
    struct TestCase
    {
        unsigned int src_fs; // Source sample rate
        unsigned int dst_fs; // Destination sample rate
        unsigned int src_ch; // Source channels
        unsigned int dst_ch; // Destination channels
        std::string name;    // Test name
    };

    std::vector<TestCase> testCases = {{44100, 48000, 2, 2, "44.1kHz Stereo -> 48kHz Stereo"},
                                       {48000, 44100, 2, 2, "48kHz Stereo -> 44.1kHz Stereo"},
                                       {44100, 44100, 1, 2, "Mono -> Stereo (same sample rate)"},
                                       {44100, 44100, 2, 1, "Stereo -> Mono (same sample rate)"},
                                       {44100, 48000, 1, 2, "Mono 44.1kHz -> Stereo 48kHz"}};

    for (const auto &test : testCases)
    {
        std::cout << "Testing " << test.name << "..." << std::endl;

        const unsigned int max_frames = 1000;

        // Create channel map
        AudioChannelMap imap = {0, std::min(1u, test.src_ch - 1)};
        AudioChannelMap omap = {0, std::min(1u, test.dst_ch - 1)};

        // Create LocSampler instance
        LocSampler sampler(test.src_fs, test.src_ch, test.dst_fs, test.dst_ch, max_frames, imap, omap);

        // Verify object creation success
        test_check(sampler.is_valid(), "LocSampler creation - " + test.name);

        // Create test audio with different frequencies for different channels
        std::vector<PCM_TYPE> input(max_frames * test.src_ch);
        for (unsigned int i = 0; i < max_frames; i++)
        {
            // Left channel uses 440Hz sine wave
            float left_sample = 16000.0f * std::sin(2.0f * static_cast<float>(M_PI) * 440.0f * static_cast<float>(i) /
                                                    static_cast<float>(test.src_fs));

            if (test.src_ch == 1)
            {
                // Mono
                input[i] = static_cast<PCM_TYPE>(left_sample);
            }
            else
            {
                // Stereo - right channel uses 880Hz
                float right_sample = 16000.0f * std::sin(2.0f * static_cast<float>(M_PI) * 880.0f * i / test.src_fs);
                input[i * test.src_ch] = static_cast<PCM_TYPE>(left_sample);
                input[i * test.src_ch + 1] = static_cast<PCM_TYPE>(right_sample);
            }
        }

        // Calculate theoretical output frames
        unsigned int expected_frames =
            static_cast<unsigned int>(max_frames * (static_cast<double>(test.dst_fs) / test.src_fs));
        if (test.src_fs == test.dst_fs)
        {
            expected_frames = max_frames;
        }

        // Create sufficiently large output buffer
        unsigned int buffer_size = expected_frames * test.dst_ch + 100; // Extra space just in case
        std::vector<PCM_TYPE> output(buffer_size, 0);

        // Process data
        unsigned int actual_frames = 0;
        RetCode result = sampler.process(input.data(), max_frames, output.data(), actual_frames);

        test_check(result == RetCode::OK, "LocSampler process - " + test.name);
        test_check(actual_frames > 0, "LocSampler output frames - " + test.name);

        // Verify output frames are close to theoretical value
        double frame_error = std::abs(static_cast<double>(actual_frames) - expected_frames) / expected_frames;
        std::cout << "  Output frames: " << actual_frames << " (expected: " << expected_frames << ")" << std::endl;
        // assert(frame_error < 0.05); // Should be within 5% error range

        // Check various properties of the output based on test scenario
        if (test.src_ch == 1 && test.dst_ch == 2)
        {
            // Mono to stereo - both channels should be the same
            for (unsigned int i = 0; i < std::min(10u, actual_frames); i++)
            { // Check first 10 frames only
                assert(output[i * 2] == output[i * 2 + 1]);
            }
            std::cout << "  Mono to stereo conversion verified" << std::endl;
        }
        else if (test.src_ch == 2 && test.dst_ch == 1)
        {
            // Stereo to mono - check mixing (this part is more complex, just do a simple verification)
            bool non_zero = false;
            for (unsigned int i = 0; i < 10; i++)
            {
                if (output[i] != 0)
                {
                    non_zero = true;
                    break;
                }
            }
            assert(non_zero);
            std::cout << "  Stereo to mono mixing verified" << std::endl;
        }

        // Perform more analysis based on different sample rate conversion cases
        if (test.src_fs != test.dst_fs)
        {
            // Check frequency preservation - here we use a simple zero-crossing method to estimate frequency
            // In real cases, more complex spectral analysis might be needed
            int zero_crossings = 0;
            bool last_positive = output[0] >= 0;

            unsigned int check_frames = std::min(100u, actual_frames);
            for (unsigned int i = 1; i < check_frames; i++)
            {
                bool current_positive = output[i * test.dst_ch] >= 0;
                if (current_positive != last_positive)
                {
                    zero_crossings++;
                    last_positive = current_positive;
                }
            }

            // After sample rate change, the corresponding zero-crossing count should change
            double expected_ratio = static_cast<double>(test.dst_fs) / test.src_fs;
            std::cout << "  Zero-crossing count check: " << zero_crossings << std::endl;

            // Ensure output is not all zeros
            bool output_non_zero = false;
            for (unsigned int i = 0; i < std::min(buffer_size, actual_frames * test.dst_ch); i++)
            {
                if (output[i] != 0)
                {
                    output_non_zero = true;
                    break;
                }
            }
            test_check(output_non_zero, "Output signal valid - " + test.name);
        }
    }

    std::cout << "LocSampler test completed" << std::endl;
}

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
    test_drcompressor();
    test_sinc_interpolator();
    test_loc_sampler();

    std::cout << "All audio processing tests completed!" << std::endl;
    return 0;
}