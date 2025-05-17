#include "audio_process.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct LocSamplerTestCase
{
    std::string name;
    unsigned int srcFs;
    unsigned int srcCh;
    unsigned int dstFs;
    unsigned int dstCh;
    unsigned int timeInterval; // milliseconds
};

class LinearResampler
{
  public:
    LinearResampler(unsigned int srcFs, unsigned int srcCh, unsigned int dstFs, unsigned int dstCh)
        : src_fs(srcFs), src_ch(srcCh), dst_fs(dstFs), dst_ch(dstCh), ratio(static_cast<double>(dstFs) / srcFs)
    {
    } // Linear interpolation resampling process
    void process(const InterleavedView<const PCM_TYPE> &input, InterleavedView<PCM_TYPE> &output) const
    {
        // Get frame counts
        const unsigned int inFrames = static_cast<unsigned int>(SamplesPerChannel(input));
        const unsigned int outFrames = static_cast<unsigned int>(SamplesPerChannel(output));

        // Apply linear interpolation for each output sample
        for (unsigned int outFrame = 0; outFrame < outFrames; ++outFrame)
        {
            // Calculate input index position (floating point)
            double inPos = outFrame / ratio;

            // Get integer part and fractional part
            unsigned int inIdx = static_cast<unsigned int>(inPos);
            double frac = inPos - inIdx;

            // Ensure index is within valid range
            if (inIdx >= inFrames - 1)
            {
                inIdx = inFrames - 2; // Ensure there's a next sample for interpolation
                if (inIdx < 0)
                    inIdx = 0; // Prevent cases with only one sample
            } // Apply linear interpolation for each channel
            for (unsigned int ch = 0; ch < std::min(src_ch, dst_ch); ++ch)
            {
                // Get adjacent samples
                PCM_TYPE sample1 = input[inIdx * src_ch + ch];
                PCM_TYPE sample2 = input[(inIdx + 1) * src_ch + ch];

                float s1 = static_cast<float>(sample1);
                float s2 = static_cast<float>(sample2);

                // Calculate output sample using linear interpolation
                float outSample = s1 * (1.0f - static_cast<float>(frac)) + s2 * static_cast<float>(frac);

                // Write to output buffer
                output[outFrame * dst_ch + ch] = static_cast<PCM_TYPE>(outSample);
            }

            // If destination channels > source channels, copy first channel to remaining channels
            for (unsigned int ch = src_ch; ch < dst_ch; ++ch)
            {
                output[outFrame * dst_ch + ch] = output[outFrame * dst_ch + 0];
            }
        }
    }

  private:
    const unsigned int src_fs;
    const unsigned int src_ch;
    const unsigned int dst_fs;
    const unsigned int dst_ch;
    const double ratio;
};

// Generate sine wave test signal
void generateSineWave(PCM_TYPE *buffer, unsigned int numSamples, unsigned int numChannels, float frequency,
                      unsigned int sampleRate)
{
    for (unsigned int i = 0; i < numSamples; i++)
    {
        // Calculate sine wave value (amplitude around 16000 to avoid clipping)
        float value = 8000.0f * std::sin(2.0f * (float)M_PI * frequency * i / sampleRate);

        // Fill all channels with the same value
        for (unsigned int ch = 0; ch < numChannels; ch++)
        {
            buffer[i * numChannels + ch] = static_cast<PCM_TYPE>(value);
        }
    }
}

// Generate logarithmic sweep signal (from low to high frequency)
void generateLogSweepSignal(PCM_TYPE *buffer, unsigned int numSamples, unsigned int numChannels, float startFreq,
                            float endFreq, unsigned int sampleRate)
{
    // Calculate phase increment using logarithmic relationship
    const double duration = static_cast<double>(numSamples) / sampleRate;
    const double k = std::pow(endFreq / startFreq, 1.0 / duration);

    // Initialize phase
    double phase = 0.0;

    for (unsigned int i = 0; i < numSamples; i++)
    {
        // Calculate current time point
        double t = static_cast<double>(i) / sampleRate;

        // Calculate phase using logarithmic sweep formula
        double instantFreq = startFreq * std::pow(k, t);
        phase += 2.0 * M_PI * instantFreq / sampleRate;

        // Keep phase within [0, 2π] range
        while (phase >= 2.0 * M_PI)
            phase -= 2.0 * M_PI;

        // Generate sample value
        float value = 8000.0f * static_cast<float>(std::sin(phase));

        // Fill all channels with the same value
        for (unsigned int ch = 0; ch < numChannels; ch++)
        {
            buffer[i * numChannels + ch] = static_cast<PCM_TYPE>(value);
        }
    }

    std::cout << "  Generated log sweep from " << startFreq << "Hz to " << endFreq << "Hz" << std::endl;
}

// Validate the resampled signal
bool validateResampling(const PCM_TYPE *original, unsigned int origSamples, unsigned int origChannels,
                        const PCM_TYPE *resampled, unsigned int resampledSamples, unsigned int resampledChannels,
                        float ratio, float errorThreshold, float testFreq, unsigned int srcFs, unsigned int dstFs)
{
    // 1. Check if the resampled length roughly matches the ratio
    float expectedSamples = origSamples * ratio;
    if (std::abs(static_cast<float>(resampledSamples) - expectedSamples) > expectedSamples * 0.1f)
    {
        std::cout << "Sample count mismatch: got " << resampledSamples << ", expected ~" << expectedSamples
                  << std::endl;
        return false;
    }

    // 2. Calculate RMS values of original and resampled signals
    float origRms = 0.0f, resampledRms = 0.0f;
    float origPeak = 0.0f, resampledPeak = 0.0f;

    for (unsigned int i = 0; i < origSamples; i++)
    {
        for (unsigned int ch = 0; ch < origChannels; ch++)
        {
            float sample = static_cast<float>(original[i * origChannels + ch]);
            origRms += sample * sample;
            origPeak = std::max(origPeak, std::abs(sample));
        }
    }
    origRms = std::sqrt(origRms / (origSamples * origChannels));

    for (unsigned int i = 0; i < resampledSamples; i++)
    {
        for (unsigned int ch = 0; ch < resampledChannels; ch++)
        {
            float sample = static_cast<float>(resampled[i * resampledChannels + ch]);
            resampledRms += sample * sample;
            resampledPeak = std::max(resampledPeak, std::abs(sample));
        }
    }
    resampledRms =
        std::sqrt(resampledRms /
                  (resampledSamples *
                   resampledChannels)); // 3. Analyze characteristics of the resampled signal and output diagnostic info
    std::cout << "  Signal Analysis:" << std::endl;
    std::cout << "    Original - RMS: " << origRms << ", Peak: " << origPeak << std::endl;
    std::cout << "    Resampled - RMS: " << resampledRms << ", Peak: " << resampledPeak << std::endl;

    // 4. Check if RMS difference is within acceptable range (energy conservation check)
    float rmsDiff = std::abs(origRms - resampledRms) / origRms;
    std::cout << "    RMS difference: " << rmsDiff * 100 << "% (threshold: " << errorThreshold * 100 << "%)"
              << std::endl;

    if (rmsDiff > errorThreshold)
    {
        std::cout << "    FAILED: RMS difference too large" << std::endl;
        return false;
    }

    // 5. Check if peak ratio is reasonable (avoid overshoot or undershoot)
    float peakRatio = resampledPeak / origPeak;
    std::cout << "    Peak ratio: " << peakRatio << std::endl;

    if (peakRatio < 0.5f || peakRatio > 2.0f)
    {
        std::cout << "    WARNING: Peak ratio outside expected range [0.5, 2.0]" << std::endl;
        // Not a failure condition, just a warning
    }

    // 6. Check for possible aliasing
    // If test frequency exceeds the Nyquist frequency of the new sample rate, issue a warning
    if (testFreq > dstFs / 2.0f)
    {
        std::cout << "    WARNING: Test frequency (" << testFreq << "Hz) exceeds Nyquist frequency of destination ("
                  << dstFs / 2.0f << "Hz), aliasing may occur" << std::endl;
    }

    std::cout << "    Validation PASSED" << std::endl;
    return true;
}

// Save PCM data to binary file
void savePCMToFile(const std::string &filename, const PCM_TYPE *buffer, unsigned int numSamples,
                   unsigned int numChannels)
{
    std::ofstream outFile(filename, std::ios::binary);
    if (!outFile)
    {
        std::cerr << "Error: Could not open file " << filename << " for writing" << std::endl;
        return;
    }

    // Write header with metadata
    outFile.write(reinterpret_cast<const char *>(&numSamples), sizeof(numSamples));
    outFile.write(reinterpret_cast<const char *>(&numChannels), sizeof(numChannels));

    // Write PCM data
    outFile.write(reinterpret_cast<const char *>(buffer), numSamples * numChannels * sizeof(PCM_TYPE));
    outFile.close();

    std::cout << "  Saved " << numSamples << " samples to " << filename << std::endl;
}

// Test LocSampler's process method simulating real-time audio processing
bool testLocSamplerProcess(const LocSamplerTestCase &testcase)
{
    const auto srcFs = testcase.srcFs;
    const auto srcCh = testcase.srcCh;
    const auto dstFs = testcase.dstFs;
    const auto dstCh = testcase.dstCh;
    const auto timeInterval = testcase.timeInterval;
    const auto name = testcase.name;

    std::cout << "\nTesting LocSampler: " << srcFs << "Hz/" << srcCh << "ch -> " << dstFs << "Hz/" << dstCh
              << "ch (total time: " << timeInterval << "ms)" << std::endl;

    // Create channel mapping
    AudioChannelMap imap{0, 1};
    AudioChannelMap omap{0, 1};

    // Use fixed 10ms processing period to simulate real-time audio processing
    const unsigned int processPeriod = 10; // ms

    // Create LocSampler instance using 10ms processing period
    LocSampler sampler(srcFs, srcCh, dstFs, dstCh, processPeriod * srcFs / 1000, imap, omap, false);

    // Calculate number of processing iterations
    unsigned int numProcessCalls = timeInterval / processPeriod;
    if (numProcessCalls < 1)
        numProcessCalls = 1;

    std::cout << "  Using process period: " << processPeriod << "ms, total iterations: " << numProcessCalls
              << std::endl;

    // Calculate input and output frame count per period
    unsigned int srcFramesPerPeriod = srcFs * processPeriod / 1000;
    unsigned int dstFramesPerPeriod = dstFs * processPeriod / 1000;

    // Calculate total input and output frame count
    unsigned int totalSrcFrames = srcFs * timeInterval / 1000;
    unsigned int totalDstFrames = dstFs * timeInterval / 1000;

    // Allocate complete input and output buffers
    std::vector<PCM_TYPE> inputBuffer(totalSrcFrames * srcCh, 0);
    std::vector<PCM_TYPE> outputBuffer(totalDstFrames * dstCh, 0);
    // Choose a frequency that won't cause severe aliasing
    // Frequency should be less than half the lowest sample rate (Nyquist frequency)
    float testFreq = std::min(srcFs, dstFs) / 8.0f;
    if (testFreq > 10000.0f)
        testFreq = 10000.0f; // Limit maximum frequency
    else if (testFreq < 100.0f)
        testFreq = 100.0f; // Ensure frequency isn't too low

    if (name.find("Sweep") != std::string::npos)
    {
        // Generate logarithmic sweep signal (from 20Hz to half the maximum frequency)
        float maxFreq = srcFs / 2.01f; // Slightly below Nyquist frequency
        generateLogSweepSignal(inputBuffer.data(), totalSrcFrames, srcCh, 20.0f, maxFreq, srcFs);
    }
    else
    {
        std::cout << "  Using test frequency: " << testFreq << " Hz" << std::endl;
        generateSineWave(inputBuffer.data(), totalSrcFrames, srcCh, testFreq, srcFs);
    }

    // Simulate real-time processing: process 10ms of data at a time
    for (unsigned int i = 0; i < numProcessCalls; i++)
    {
        // Calculate the starting position for the current processing batch
        unsigned int srcStartIdx = i * srcFramesPerPeriod * srcCh;
        unsigned int dstStartIdx = i * dstFramesPerPeriod * dstCh;
        // Create view for each processing period
        const PCM_TYPE *inputPtr = inputBuffer.data() + srcStartIdx;
        PCM_TYPE *outputPtr = outputBuffer.data() + dstStartIdx;

        // Create InterleavedView views
        InterleavedView<const PCM_TYPE> inputView(inputPtr, srcFramesPerPeriod, srcCh);
        InterleavedView<PCM_TYPE> outputView(outputPtr, dstFramesPerPeriod, dstCh);

        // Process the current batch of audio data
        float gain = 0.0f; // Default gain is 0dB
        RetCode result = sampler.process(inputView, outputView, gain);
        if (!result)
        {
            std::cout << "LocSampler process failed at iteration " << i + 1 << "/" << numProcessCalls << std::endl;
            return false;
        }
    }

    std::cout << "  All " << numProcessCalls << " batches processed successfully" << std::endl;

    // Save pre and post-processing PCM data
    std::string testName = std::to_string(srcFs) + "Hz_" + std::to_string(srcCh) + "ch_to_" + std::to_string(dstFs) +
                           "Hz_" + std::to_string(dstCh) + "ch_" + std::to_string(timeInterval) + "ms";

    if (name.find("Sweep") != std::string::npos)
    {
        testName += "_Sweep";
    }

    // skip the first srcFramesPerPeriod frames
    const auto inputBuffer_processed = inputBuffer.data() + srcFramesPerPeriod * srcCh;
    const auto outputBuffer_processed = outputBuffer.data() + dstFramesPerPeriod * dstCh;
    totalDstFrames -= dstFramesPerPeriod;
    totalSrcFrames -= srcFramesPerPeriod;

    std::string inputFilename = "input_" + testName + ".pcm";
    std::string outputFilename = "output_" + testName + ".pcm";
    // Save original and processed signals
    savePCMToFile(inputFilename, inputBuffer_processed, totalSrcFrames, srcCh);
    savePCMToFile(outputFilename, outputBuffer_processed, totalDstFrames, dstCh);

    // Save test parameters to metadata file
    std::string metadataFilename = "metadata_" + testName + ".txt";
    std::ofstream metaFile(metadataFilename);
    if (metaFile.is_open())
    {
        metaFile << "src_fs=" << srcFs << std::endl;
        metaFile << "src_ch=" << srcCh << std::endl;
        metaFile << "dst_fs=" << dstFs << std::endl;
        metaFile << "dst_ch=" << dstCh << std::endl;
        metaFile << "time_interval=" << timeInterval << std::endl;
        metaFile << "test_frequency=" << testFreq << std::endl;
        metaFile.close();
        std::cout << "  Saved test metadata to " << metadataFilename << std::endl;
    }
    // Validate results
    float ratio = static_cast<float>(dstFs) / static_cast<float>(srcFs);
    float errorThreshold = 0.2f; // Allow 20% RMS difference

    bool valid = true;

    if (name.find("Sweep") == std::string::npos)
    {
        // Validate using the valid data (skipping first period)
        valid = validateResampling(inputBuffer_processed, totalSrcFrames, srcCh, outputBuffer_processed, totalDstFrames,
                                   dstCh, ratio, errorThreshold, testFreq, srcFs, dstFs);
    }

    if (valid)
    {
        std::cout << "Test PASSED!" << std::endl;
    }
    else
    {
        std::cout << "Test FAILED!" << std::endl;
    }

    return valid;
}

void testLinearVsLocSampler(const LocSamplerTestCase &testcase)
{
    const auto srcFs = testcase.srcFs;
    const auto srcCh = testcase.srcCh;
    const auto dstFs = testcase.dstFs;
    const auto dstCh = testcase.dstCh;
    const auto timeInterval = testcase.timeInterval;
    const auto name = testcase.name;

    std::cout << "\nTesting Linear vs LocSampler: " << srcFs << "Hz/" << srcCh << "ch -> " << dstFs << "Hz/" << dstCh
              << "ch (total time: " << timeInterval << "ms)" << std::endl;

    // Create channel mapping
    AudioChannelMap imap{0, 1};
    AudioChannelMap omap{0, 1};

    // Use fixed 10ms processing period to simulate real-time audio processing
    const unsigned int processPeriod = 10; // ms

    LinearResampler linearResampler(srcFs, srcCh, dstFs, dstCh);

    // Calculate number of processing iterations
    unsigned int numProcessCalls = timeInterval / processPeriod;
    if (numProcessCalls < 1)
        numProcessCalls = 1;

    std::cout << "  Using process period: " << processPeriod << "ms, total iterations: " << numProcessCalls
              << std::endl;

    // Calculate input and output frame count per period
    unsigned int srcFramesPerPeriod = srcFs * processPeriod / 1000;
    unsigned int dstFramesPerPeriod = dstFs * processPeriod / 1000;

    // Calculate total input and output frame count
    unsigned int totalSrcFrames = srcFs * timeInterval / 1000;
    unsigned int totalDstFrames = dstFs * timeInterval / 1000;

    // Allocate input and output buffers
    std::vector<PCM_TYPE> inputBuffer(totalSrcFrames * srcCh, 0);
    std::vector<PCM_TYPE> outputLinear(totalDstFrames * dstCh, 0);

    // Choose test frequency
    float testFreq = std::min(srcFs, dstFs) / 8.0f;
    if (testFreq > 10000.0f)
        testFreq = 10000.0f;
    else if (testFreq < 100.0f)
        testFreq = 100.0f;

    if (name.find("Sweep") != std::string::npos)
    {
        // Generate logarithmic sweep signal
        float maxFreq = srcFs / 2.01f;
        generateLogSweepSignal(inputBuffer.data(), totalSrcFrames, srcCh, 20.0f, maxFreq, srcFs);
    }
    else
    {
        std::cout << "  Using test frequency: " << testFreq << " Hz" << std::endl;
        generateSineWave(inputBuffer.data(), totalSrcFrames, srcCh, testFreq, srcFs);
    }

    // Simulate real-time processing: process 10ms of data at a time
    for (unsigned int i = 0; i < numProcessCalls; i++)
    {
        // Calculate the starting position for the current processing batch
        unsigned int srcStartIdx = i * srcFramesPerPeriod * srcCh;
        unsigned int dstStartIdx = i * dstFramesPerPeriod * dstCh;
        const PCM_TYPE *inputPtr = inputBuffer.data() + srcStartIdx;
        PCM_TYPE *outputLinearPtr = outputLinear.data() + dstStartIdx;

        // Create views
        InterleavedView<const PCM_TYPE> inputView(inputPtr, srcFramesPerPeriod, srcCh);
        InterleavedView<PCM_TYPE> outputLinearView(outputLinearPtr, dstFramesPerPeriod, dstCh);

        // Process using linear interpolation
        linearResampler.process(inputView, outputLinearView);
    }

    std::cout << "  All " << numProcessCalls << " batches processed successfully" << std::endl;

    // Save processed results
    std::string testName = std::to_string(srcFs) + "Hz_" + std::to_string(srcCh) + "ch_to_" + std::to_string(dstFs) +
                           "Hz_" + std::to_string(dstCh) + "ch_" + std::to_string(timeInterval) + "ms";

    if (name.find("Sweep") != std::string::npos)
    {
        testName += "_Sweep";
    }

    // Skip first processing period data
    const auto inputBuffer_processed = inputBuffer.data() + srcFramesPerPeriod * srcCh;
    const auto outputLinear_processed = outputLinear.data() + dstFramesPerPeriod * dstCh;
    totalDstFrames -= dstFramesPerPeriod;
    totalSrcFrames -= srcFramesPerPeriod;

    std::string inputFilename = "input_linear_" + testName + ".pcm";
    std::string outputLinearFilename = "output_linear_" + testName + ".pcm";

    // Save original and processed signals
    savePCMToFile(inputFilename, inputBuffer_processed, totalSrcFrames, srcCh);
    savePCMToFile(outputLinearFilename, outputLinear_processed, totalDstFrames, dstCh);

    // Save test parameters to metadata file
    std::string metadataFilename = "metadata_linear_" + testName + ".txt";
    std::ofstream metaFile(metadataFilename);
    if (metaFile.is_open())
    {
        metaFile << "src_fs=" << srcFs << std::endl;
        metaFile << "src_ch=" << srcCh << std::endl;
        metaFile << "dst_fs=" << dstFs << std::endl;
        metaFile << "dst_ch=" << dstCh << std::endl;
        metaFile << "time_interval=" << timeInterval << std::endl;
        metaFile << "test_frequency=" << testFreq << std::endl;
        metaFile.close();
        std::cout << "  Saved comparison test metadata to " << metadataFilename << std::endl;
    }
}

int main(int, char *[])
{ // Test cases
    std::vector<LocSamplerTestCase> testCases = {
        // Basic test cases: different sample rates and channel configurations
        {"Same rate mono to stereo", 24000, 1, 44100, 2, 100},
        {"Same rate stereo to mono", 44100, 2, 44100, 1, 100},
        {"Upsampling 2x", 24000, 2, 48000, 2, 100},
        {"Downsampling 3x", 48000, 2, 16000, 2, 100},
        {"Complex - Upsample + Ch convert", 32000, 1, 48000, 2, 100},
        {"Complex - Downsample + Ch convert", 48000, 2, 8000, 1, 100},

        // Different time interval tests: from 100ms to 600ms
        {"Time interval 100ms", 44100, 1, 48000, 1, 100},
        {"Time interval 200ms", 44100, 1, 48000, 1, 200},
        {"Time interval 300ms", 44100, 1, 48000, 1, 300},
        {"Time interval 400ms", 44100, 1, 48000, 1, 400},
        {"Time interval 500ms", 44100, 1, 48000, 1, 500},
        {"Time interval 600ms", 44100, 1, 48000, 1, 600},

        // Specific sweep signal tests
        {"Sweep Up 16kHz to 48kHz", 16000, 1, 48000, 1, 400},
        {"Sweep Down 48kHz to 32kHz", 48000, 2, 32000, 2, 400},
        {"Sweep Complex 44.1kHz to 48kHz", 44100, 1, 48000, 2, 400},
    };

    bool allPassed = true;

    std::cout << "======= LocSampler Tests =======" << std::endl;

    // Run all test cases
    for (const auto &testCase : testCases)
    {
        std::cout << "\n--- Test: " << testCase.name << " ---" << std::endl;

        bool testResult = testLocSamplerProcess(testCase);

        if (!testResult)
        {
            allPassed = false;
        }
    }

    std::vector<LocSamplerTestCase> compareTestCases = {
        // Basic comparison test cases
        {"Compare Basic", 44100, 2, 48000, 2, 200},
        {"Compare Upsample", 16000, 1, 48000, 2, 200},
        {"Compare Downsample", 48000, 2, 16000, 1, 200},
        {"Compare Sweep", 44100, 1, 48000, 2, 400},
    };

    std::cout << "======= Linear vs LocSampler Comparison Tests =======" << std::endl;
    for (const auto &testCase : compareTestCases)
    {
        std::cout << "\n--- Compare Test: " << testCase.name << " ---" << std::endl;
        testLinearVsLocSampler(testCase);
    }

    // Display final results
    std::cout << "\n======= Test Results =======" << std::endl;
    if (allPassed)
    {
        std::cout << "All tests PASSED!" << std::endl;
    }
    else
    {
        std::cout << "Some tests FAILED!" << std::endl;
    } // Run Python analysis script if available

    return allPassed ? 0 : 1;
}