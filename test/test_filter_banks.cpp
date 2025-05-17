#include "audio_filterbanks.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

const float M_PI = 3.14159265358979323846f;

// 生成扫频信号 (Chirp Signal)
std::vector<float> generateSweep(int sampleRate, float startFreq, float endFreq, float duration)
{
    int numSamples = static_cast<int>(sampleRate * duration);
    std::vector<float> sweep(numSamples);

    // 确保频率为正值
    startFreq = std::max(startFreq, 1.0f); // 避免对数运算出错
    endFreq = std::max(endFreq, 1.0f);

    // 计算线性扫频参数
    // float k = (endFreq - startFreq) / duration;
    // 计算对数扫频参数
    float k = std::pow(endFreq / startFreq, 1.0f / duration);

    for (int i = 0; i < numSamples; i++)
    {
        float t = static_cast<float>(i) / sampleRate;
        // 瞬时频率: f(t) = startFreq + k*t
        // 相位: phi(t) = 2*pi*(startFreq*t + k*t^2/2)
        // float phase = 2.0f * M_PI * (startFreq * t + 0.5f * k * t * t);
        if (t <= 0.0f)
            continue; // 避免对0取对数

        // 对数扫频的瞬时频率: f(t) = startFreq * k^t
        // 对数扫频的相位积分: phi(t) = (2*pi*startFreq)/(ln(k)) * (k^t - 1)
        float phase = 0.0f;
        if (std::abs(k - 1.0f) < 1e-6)
        {
            // 如果k接近1（近似线性扫频），使用线性公式避免数值不稳定
            phase = 2.0f * M_PI * startFreq * t;
        }
        else
        {
            phase = 2.0f * M_PI * startFreq * (std::pow(k, t) - 1.0f) / std::log(k);
        }

        sweep[i] = std::sin(phase);
    }

    // 应用淡入淡出以减少边缘效应
    int fadeLength = static_cast<int>(sampleRate * 0.01f);
    for (int i = 0; i < fadeLength && i < numSamples; i++)
    {
        float fade = static_cast<float>(i) / fadeLength;
        sweep[i] *= fade;
        if (i < numSamples - fadeLength)
        {
            sweep[numSamples - i - 1] *= fade;
        }
    }

    return sweep;
}

// 保存数据到文件
void saveToFile(const std::string &filename, const std::vector<float> &data)
{
    std::ofstream file(filename, std::ios::binary);
    if (file.is_open())
    {
        file.write(reinterpret_cast<const char *>(data.data()), data.size() * sizeof(float));
        file.close();
        std::cout << "Data saved to " << filename << std::endl;
    }
    else
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
    }
}

// 测试ThreeBandFilterBank
void testThreeBandFilterBank(int sampleRate)
{
    // 参数
    float startFreq = 10.0f;            // 10Hz
    float endFreq = sampleRate / 2.0f; // Nyquist频率
    float duration = 5.0f;             // 5秒的扫频信号

    // 生成扫频信号
    std::vector<float> sweep = generateSweep(sampleRate, startFreq, endFreq, duration);

    // 准备ThreeBandFilterBank数据结构
    ThreeBandFilterBank filterBank;
    constexpr int fullBandSize = ThreeBandFilterBank::kFullBandSize;
    constexpr int numBands = ThreeBandFilterBank::kNumBands;
    constexpr int splitBandSize = ThreeBandFilterBank::kSplitBandSize;

    // 准备输出数据容器
    std::vector<std::vector<float>> bandOutputs(numBands);
    for (int i = 0; i < numBands; i++)
    {
        bandOutputs[i].resize(sweep.size() / numBands);
    }
    std::vector<float> reconstructed(sweep.size());

    // 处理信号
    for (size_t i = 0; i <= sweep.size() - fullBandSize; i += fullBandSize)
    {
        // 准备输入块
        std::array<float, ThreeBandFilterBank::kFullBandSize> inputBlock;
        for (int j = 0; j < fullBandSize; j++)
        {
            inputBlock[j] = sweep[i + j];
        }

        // 准备子带输出
        std::array<std::array<float, splitBandSize>, numBands> splitBands;
        std::array<ArrayView<float>, numBands> splitBandViews;
        for (int band = 0; band < numBands; band++)
        {
            splitBandViews[band] = ArrayView<float>(splitBands[band].data(), splitBandSize);
        }

        // 分析过程
        filterBank.Analysis(ArrayView<const float, fullBandSize>(inputBlock.data(), fullBandSize),
                            ArrayView<const ArrayView<float>, 3>(splitBandViews.data(), numBands));

        // 保存子带数据
        for (int band = 0; band < numBands; band++)
        {
            for (int j = 0; j < splitBandSize; j++)
            {
                bandOutputs[band][i / numBands + j] = splitBands[band][j];
            }
        }

        // 合成过程
        std::array<float, fullBandSize> outputBlock;
        filterBank.Synthesis(ArrayView<const ArrayView<float>, 3>(splitBandViews.data(), numBands),
                             ArrayView<float, fullBandSize>(outputBlock.data(), fullBandSize));

        // 保存重构数据
        for (int j = 0; j < fullBandSize; j++)
        {
            reconstructed[i + j] = outputBlock[j];
        }
    }

    // 保存结果
    std::string sampleRateStr = std::to_string(sampleRate / 1000);
    saveToFile("three_band_input_" + sampleRateStr + "k.bin", sweep);
    saveToFile("three_band_output_" + sampleRateStr + "k.bin", reconstructed);
    for (int band = 0; band < numBands; band++)
    {
        saveToFile("three_band_" + std::to_string(band) + "_" + sampleRateStr + "k.bin", bandOutputs[band]);
    }
}

// 测试TwoBandFilterBank
void testTwoBandFilterBank(int sampleRate)
{
    // 参数
    float startFreq = 20.0f;           // 20Hz
    float endFreq = sampleRate / 2.0f; // Nyquist频率
    float duration = 4.0f;             // 5秒的扫频信号

    // 生成扫频信号
    std::vector<float> sweep = generateSweep(sampleRate, startFreq, endFreq, duration);

    // 准备TwoBandFilterBank数据结构
    TwoBandFilterBank filterBank;
    const int fullBandSize = TwoBandFilterBank::kFullBandSize;
    const int numBands = TwoBandFilterBank::kNumBands;
    const int splitBandSize = TwoBandFilterBank::kSplitBandSize;

    // 准备输出数据容器
    std::vector<std::vector<float>> bandOutputs(numBands);
    for (int i = 0; i < numBands; i++)
    {
        bandOutputs[i].resize(sweep.size() / numBands);
    }
    std::vector<float> reconstructed(sweep.size());

    // 处理信号
    for (size_t i = 0; i <= sweep.size() - fullBandSize; i += fullBandSize)
    {
        // 准备输入块
        std::array<float, TwoBandFilterBank::kFullBandSize> inputBlock;
        for (int j = 0; j < fullBandSize; j++)
        {
            inputBlock[j] = sweep[i + j];
        }

        // 准备子带输出
        std::array<std::array<float, splitBandSize>, numBands> splitBands;
        std::array<ArrayView<float>, numBands> splitBandViews;
        for (int band = 0; band < numBands; band++)
        {
            splitBandViews[band] = ArrayView<float>(splitBands[band].data(), splitBandSize);
        }

        // 分析过程
        filterBank.Analysis(ArrayView<const float, fullBandSize>(inputBlock.data(), fullBandSize),
                            ArrayView<const ArrayView<float>, 2>(splitBandViews.data(), numBands));

        // 保存子带数据
        for (int band = 0; band < numBands; band++)
        {
            for (int j = 0; j < splitBandSize; j++)
            {
                bandOutputs[band][i / numBands + j] = splitBands[band][j];
            }
        }

        // 合成过程
        std::array<float, fullBandSize> outputBlock;
        filterBank.Synthesis(ArrayView<const ArrayView<float>, 2>(splitBandViews.data(), numBands),
                             ArrayView<float, fullBandSize>(outputBlock.data(), fullBandSize));

        // 保存重构数据
        for (int j = 0; j < fullBandSize; j++)
        {
            reconstructed[i + j] = outputBlock[j];
        }
    }

    // 保存结果
    std::string sampleRateStr = std::to_string(sampleRate / 1000);
    saveToFile("two_band_input_" + sampleRateStr + "k.bin", sweep);
    saveToFile("two_band_output_" + sampleRateStr + "k.bin", reconstructed);
    for (int band = 0; band < numBands; band++)
    {
        saveToFile("two_band_" + std::to_string(band) + "_" + sampleRateStr + "k.bin", bandOutputs[band]);
    }
}

// 主函数
int main()
{
    // 测试不同采样率
    testThreeBandFilterBank(48000); // 48kHz
    testTwoBandFilterBank(32000);   // 32kHz

    std::cout << "All tests completed." << std::endl;
    return 0;
}