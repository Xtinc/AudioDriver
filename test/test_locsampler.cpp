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

// 生成正弦波测试信号
void generateSineWave(PCM_TYPE *buffer, unsigned int numSamples, unsigned int numChannels, float frequency,
                      unsigned int sampleRate)
{
    for (unsigned int i = 0; i < numSamples; i++)
    {
        // 计算正弦波值（振幅为16000左右，避免截断）
        float value = 16000.0f * std::sin(2.0f * (float)M_PI * frequency * i / sampleRate);

        // 为所有通道填充相同值
        for (unsigned int ch = 0; ch < numChannels; ch++)
        {
            buffer[i * numChannels + ch] = static_cast<PCM_TYPE>(value);
        }
    }
}

// 验证重采样后的信号
bool validateResampling(const PCM_TYPE *original, unsigned int origSamples, unsigned int origChannels,
                        const PCM_TYPE *resampled, unsigned int resampledSamples, unsigned int resampledChannels,
                        float ratio, float errorThreshold, float testFreq, unsigned int srcFs, unsigned int dstFs)
{
    // 1. 检查重采样后的长度是否大致匹配比率
    float expectedSamples = origSamples * ratio;
    if (std::abs(static_cast<float>(resampledSamples) - expectedSamples) > expectedSamples * 0.1f)
    {
        std::cout << "Sample count mismatch: got " << resampledSamples << ", expected ~" << expectedSamples
                  << std::endl;
        return false;
    }

    // 2. 计算原始和重采样后信号的RMS值
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
    resampledRms = std::sqrt(resampledRms / (resampledSamples * resampledChannels));

    // 3. 分析重采样信号的特性并输出详细诊断信息
    std::cout << "  Signal Analysis:" << std::endl;
    std::cout << "    Original - RMS: " << origRms << ", Peak: " << origPeak << std::endl;
    std::cout << "    Resampled - RMS: " << resampledRms << ", Peak: " << resampledPeak << std::endl;

    // 4. 检查RMS差异是否在可接受范围内 (能量守恒检查)
    float rmsDiff = std::abs(origRms - resampledRms) / origRms;
    std::cout << "    RMS difference: " << rmsDiff * 100 << "% (threshold: " << errorThreshold * 100 << "%)"
              << std::endl;

    if (rmsDiff > errorThreshold)
    {
        std::cout << "    FAILED: RMS difference too large" << std::endl;
        return false;
    }

    // 5. 检查峰值比例是否合理 (避免过冲或欠冲)
    float peakRatio = resampledPeak / origPeak;
    std::cout << "    Peak ratio: " << peakRatio << std::endl;

    if (peakRatio < 0.5f || peakRatio > 2.0f)
    {
        std::cout << "    WARNING: Peak ratio outside expected range [0.5, 2.0]" << std::endl;
        // 不作为失败条件，只是警告
    }

    // 6. 检查混叠可能性
    // 如果测试频率超过了新采样率的奈奎斯特频率，应该发出警告
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

// 测试LocSampler的process方法，模拟实时音频处理
bool testLocSamplerProcess(unsigned int srcFs, unsigned int srcCh, unsigned int dstFs, unsigned int dstCh,
                           unsigned int timeInterval)
{
    std::cout << "\nTesting LocSampler: " << srcFs << "Hz/" << srcCh << "ch -> " << dstFs << "Hz/" << dstCh
              << "ch (total time: " << timeInterval << "ms)" << std::endl;

    // 创建通道映射
    AudioChannelMap imap{0, 1};
    AudioChannelMap omap{0, 1};

    // 使用固定的10ms处理周期，模拟实时音频处理
    const unsigned int processPeriod = 10; // ms

    // 创建LocSampler实例，使用10ms处理周期
    LocSampler sampler(srcFs, srcCh, dstFs, dstCh, processPeriod, imap, omap);

    // 计算处理次数
    unsigned int numProcessCalls = timeInterval / processPeriod;
    if (numProcessCalls < 1)
        numProcessCalls = 1;

    std::cout << "  Using process period: " << processPeriod << "ms, total iterations: " << numProcessCalls
              << std::endl;

    // 计算每次处理的输入和输出帧数
    unsigned int srcFramesPerPeriod = srcFs * processPeriod / 1000;
    unsigned int dstFramesPerPeriod = dstFs * processPeriod / 1000;

    // 计算总的输入和输出帧数
    unsigned int totalSrcFrames = srcFs * timeInterval / 1000;
    unsigned int totalDstFrames = dstFs * timeInterval / 1000;

    // 分配完整输入和输出缓冲区
    std::vector<PCM_TYPE> inputBuffer(totalSrcFrames * srcCh, 0);
    std::vector<PCM_TYPE> outputBuffer(totalDstFrames * dstCh, 0);

    // 选择一个不会出现严重混叠的频率
    // 频率应该小于最低采样率的一半（奈奎斯特频率）
    float testFreq = std::min(srcFs, dstFs) / 8.0f;
    if (testFreq > 10000.0f)
        testFreq = 10000.0f; // 限制最高频率
    else if (testFreq < 100.0f)
        testFreq = 100.0f; // 确保频率不会太低

    std::cout << "  Using test frequency: " << testFreq << " Hz" << std::endl; // 生成测试信号
    generateSineWave(inputBuffer.data(), totalSrcFrames, srcCh, testFreq, srcFs);

    // 模拟实时处理：每次处理10ms的数据
    for (unsigned int i = 0; i < numProcessCalls; i++)
    {
        // 计算当前处理批次的起始位置
        unsigned int srcStartIdx = i * srcFramesPerPeriod * srcCh;
        unsigned int dstStartIdx = i * dstFramesPerPeriod * dstCh;

        // 创建每个处理周期的视图
        InterleavedView<const PCM_TYPE> inputView(inputBuffer.data() + srcStartIdx, srcFramesPerPeriod, srcCh);
        InterleavedView<PCM_TYPE> outputView(outputBuffer.data() + dstStartIdx, dstFramesPerPeriod, dstCh);

        // 处理当前批次的音频数据
        RetCode result = sampler.process(inputView, outputView);
        if (!result)
        {
            std::cout << "LocSampler process failed at iteration " << i + 1 << "/" << numProcessCalls << std::endl;
            return false;
        }
    }

    std::cout << "  All " << numProcessCalls << " batches processed successfully" << std::endl;

    // 保存处理前后的PCM数据
    std::string testName = std::to_string(srcFs) + "Hz_" + std::to_string(srcCh) + "ch_to_" + std::to_string(dstFs) +
                           "Hz_" + std::to_string(dstCh) + "ch_" + std::to_string(timeInterval) + "ms";

    std::string inputFilename = "input_" + testName + ".pcm";
    std::string outputFilename = "output_" + testName + ".pcm"; // 保存原始信号和处理后信号
    savePCMToFile(inputFilename, inputBuffer.data(), totalSrcFrames, srcCh);
    savePCMToFile(outputFilename, outputBuffer.data(), totalDstFrames, dstCh);

    // 保存测试参数到元数据文件
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
    } // 验证结果
    float ratio = static_cast<float>(dstFs) / static_cast<float>(srcFs);
    float errorThreshold = 0.2f; // 允许20%的RMS差异

    // 从有效数据开始进行验证
    bool valid = validateResampling(inputBuffer.data(), totalSrcFrames, srcCh, outputBuffer.data(), totalDstFrames,
                                    dstCh, ratio, errorThreshold, testFreq, srcFs, dstFs);

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

int main(int, char *[])
{ // 测试用例
    struct LocSamplerTestCase
    {
        std::string name;
        unsigned int srcFs;
        unsigned int srcCh;
        unsigned int dstFs;
        unsigned int dstCh;
        unsigned int timeInterval; // 毫秒
    };

    std::vector<LocSamplerTestCase> testCases = {
        // 基本测试用例：不同采样率和通道配置
        {"Same rate mono to stereo", 24000, 1, 44100, 2, 100},
        {"Same rate stereo to mono", 44100, 2, 44100, 1, 100},
        {"Upsampling 2x", 24000, 2, 48000, 2, 100},
        {"Downsampling 3x", 48000, 2, 16000, 2, 100},
        {"Complex - Upsample + Ch convert", 32000, 1, 48000, 2, 100},
        {"Complex - Downsample + Ch convert", 48000, 2, 8000, 1, 100},

        // 不同时间间隔测试：从100ms到240ms
        {"Time interval 100ms", 44100, 1, 48000, 1, 100},
        {"Time interval 200ms", 44100, 1, 48000, 1, 200},
        {"Time interval 300ms", 44100, 1, 48000, 1, 300},
        {"Time interval 400ms", 44100, 1, 48000, 1, 400},
        {"Time interval 500ms", 44100, 1, 48000, 1, 500},
        {"Time interval 600ms", 44100, 1, 48000, 1, 600},
    };

    bool allPassed = true;

    std::cout << "======= LocSampler Tests =======" << std::endl;

    // 运行所有测试用例
    for (const auto &testCase : testCases)
    {
        std::cout << "\n--- Test: " << testCase.name << " ---" << std::endl;

        bool testResult = testLocSamplerProcess(testCase.srcFs, testCase.srcCh, testCase.dstFs, testCase.dstCh,
                                                testCase.timeInterval);

        if (!testResult)
        {
            allPassed = false;
        }
    }

    // 显示总结果
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