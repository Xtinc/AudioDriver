#include "audio_filterbanks.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace
{
const float kTolerance = 1e-6f;
const float kConvergenceTolerance = 0.1f;
const float A_PI = 3.14159265358979323846f;
} // namespace

// 辅助函数：生成正弦波信号
std::vector<float> GenerateSineWave(size_t length, float frequency, float sample_rate, float amplitude = 1.0f)
{
    std::vector<float> signal(length);
    const float omega = 2.0f * A_PI * frequency / sample_rate;
    for (size_t i = 0; i < length; ++i)
    {
        signal[i] = amplitude * std::sin(omega * i);
    }
    return signal;
}

// 辅助函数：生成白噪声
std::vector<float> GenerateWhiteNoise(size_t length, float amplitude = 0.1f, unsigned seed = 42)
{
    std::vector<float> noise(length);
    std::mt19937 generator(seed);
    std::normal_distribution<float> distribution(0.0f, amplitude);

    for (size_t i = 0; i < length; ++i)
    {
        noise[i] = distribution(generator);
    }
    return noise;
}

// 辅助函数：计算均方误差
float CalculateMSE(const std::vector<float> &signal1, const std::vector<float> &signal2)
{
    assert(signal1.size() == signal2.size());
    float mse = 0.0f;
    for (size_t i = 0; i < signal1.size(); ++i)
    {
        float diff = signal1[i] - signal2[i];
        mse += diff * diff;
    }
    return mse / signal1.size();
}

// 测试1：基本构造和初始化
void TestConstruction()
{
    std::cout << "测试 LMSFilter 构造函数... ";

    const size_t filter_length = 32;
    const float step_size = 0.1f;

    LMSFilter filter(filter_length, step_size);

    // 测试初始输出应该为0（因为权重都是0）
    float output = filter.Process(1.0f, 0.0f);
    assert(std::abs(output) < kTolerance);

    std::cout << "通过\n";
}

// 测试2：系统识别 - LMS滤波器学习一个已知的FIR滤波器
void TestSystemIdentification()
{
    std::cout << "测试系统识别... ";

    const size_t filter_length = 8;
    const float step_size = 0.01f;
    const size_t signal_length = 2000;

    // 定义一个已知的FIR滤波器
    std::vector<float> target_filter = {0.1f, 0.3f, 0.5f, 0.2f, -0.1f, 0.05f, 0.02f, 0.01f};

    // 生成输入信号（白噪声）
    auto input_signal = GenerateWhiteNoise(signal_length, 1.0f);

    // 通过目标滤波器生成期望信号
    std::vector<float> desired_signal(signal_length, 0.0f);
    for (size_t n = 0; n < signal_length; ++n)
    {
        for (size_t k = 0; k < target_filter.size() && k <= n; ++k)
        {
            desired_signal[n] += target_filter[k] * input_signal[n - k];
        }
    }

    // 使用LMS滤波器进行自适应
    LMSFilter lms_filter(filter_length, step_size);
    std::vector<float> output_signal(signal_length);
    std::vector<float> error_signal(signal_length);

    for (size_t n = 0; n < signal_length; ++n)
    {
        output_signal[n] = lms_filter.Process(input_signal[n], desired_signal[n]);
        error_signal[n] = desired_signal[n] - output_signal[n];
    }

    // 计算最后500个样本的MSE（收敛后）
    const size_t convergence_start = signal_length - 500;
    float final_mse = 0.0f;
    for (size_t i = convergence_start; i < signal_length; ++i)
    {
        final_mse += error_signal[i] * error_signal[i];
    }
    final_mse /= 500;

    // 验证收敛性：最终MSE应该很小
    assert(final_mse < kConvergenceTolerance);

    std::cout << "通过 (最终MSE: " << final_mse << ")\n";
}

// 测试3：噪声消除
void TestNoiseCancellation()
{
    std::cout << "测试噪声消除... ";

    const size_t filter_length = 16;
    const float step_size = 0.05f;
    const size_t signal_length = 1000;
    const float sample_rate = 1000.0f;

    // 生成干净的信号（100Hz正弦波）
    auto clean_signal = GenerateSineWave(signal_length, 100.0f, sample_rate);

    // 生成噪声信号（300Hz正弦波）
    auto noise_signal = GenerateSineWave(signal_length, 300.0f, sample_rate, 0.5f);

    // 创建含噪声的信号
    std::vector<float> noisy_signal(signal_length);
    for (size_t i = 0; i < signal_length; ++i)
    {
        noisy_signal[i] = clean_signal[i] + noise_signal[i];
    }

    // 使用LMS滤波器进行噪声消除
    LMSFilter noise_canceller(filter_length, step_size);
    std::vector<float> filtered_signal(signal_length);

    for (size_t i = 0; i < signal_length; ++i)
    {
        // 期望信号是含噪声的信号，输入是噪声参考
        filtered_signal[i] = noise_canceller.Process(noise_signal[i], noisy_signal[i]);
    }

    // 计算噪声消除效果（最后300个样本）
    const size_t eval_start = signal_length - 300;
    float mse_before = 0.0f, mse_after = 0.0f;

    for (size_t i = eval_start; i < signal_length; ++i)
    {
        float error_before = noisy_signal[i] - clean_signal[i];
        float error_after = (noisy_signal[i] - filtered_signal[i]) - clean_signal[i];
        mse_before += error_before * error_before;
        mse_after += error_after * error_after;
    }

    mse_before /= 300;
    mse_after /= 300;

    // 验证噪声消除效果：处理后的MSE应该小于处理前
    assert(mse_after < mse_before);

    std::cout << "通过 (噪声减少: " << 10 * std::log10(mse_before / mse_after) << " dB)\n";
}

// 测试4：重置功能
void TestReset()
{
    std::cout << "测试重置功能... ";

    const size_t filter_length = 10;
    const float step_size = 0.1f;

    LMSFilter filter(filter_length, step_size);

    // 运行一些迭代以改变内部状态
    for (int i = 0; i < 50; ++i)
    {
        filter.Process(0.5f, 1.0f);
    }

    // 重置滤波器
    filter.Reset();

    // 验证重置后的输出应该为0
    float output = filter.Process(1.0f, 0.0f);
    assert(std::abs(output) < kTolerance);

    std::cout << "通过\n";
}

// 测试5：步长大小对收敛的影响
void TestStepSizeEffect()
{
    std::cout << "测试步长大小效应... ";

    const size_t filter_length = 4;
    const size_t signal_length = 500;

    // 小步长应该收敛慢但稳定
    LMSFilter filter_small(filter_length, 0.001f);
    // 大步长应该收敛快但可能不稳定
    LMSFilter filter_large(filter_length, 0.1f);

    // 简单的系统：延迟1个样本
    std::vector<float> input_signal = GenerateWhiteNoise(signal_length, 1.0f);

    std::vector<float> error_small(signal_length);
    std::vector<float> error_large(signal_length);

    for (size_t i = 0; i < signal_length; ++i)
    {
        float desired = (i > 0) ? input_signal[i - 1] : 0.0f;

        float output_small = filter_small.Process(input_signal[i], desired);
        float output_large = filter_large.Process(input_signal[i], desired);

        error_small[i] = desired - output_small;
        error_large[i] = desired - output_large;
    }

    // 计算最后100个样本的MSE
    float mse_small = 0.0f, mse_large = 0.0f;
    for (size_t i = signal_length - 100; i < signal_length; ++i)
    {
        mse_small += error_small[i] * error_small[i];
        mse_large += error_large[i] * error_large[i];
    }
    mse_small /= 100;
    mse_large /= 100;

    // 大步长应该在最终收敛得更好（对于这个简单系统）
    assert(mse_large < mse_small * 2.0f); // 允许一些容差

    std::cout << "通过\n";
}

// 测试6：边界条件
void TestBoundaryConditions()
{
    std::cout << "测试边界条件... ";

    // 测试最小滤波器长度
    LMSFilter filter_min(1, 0.1f);
    float output = filter_min.Process(1.0f, 1.0f);
    // 第一次输出应该是0，因为权重初始为0
    assert(std::abs(output) < kTolerance);

    // 第二次应该有非零输出（权重已更新）
    output = filter_min.Process(1.0f, 1.0f);
    assert(std::abs(output) > kTolerance);

    // 测试零输入
    LMSFilter filter_zero(5, 0.1f);
    for (int i = 0; i < 10; ++i)
    {
        output = filter_zero.Process(0.0f, 0.0f);
        assert(std::abs(output) < kTolerance);
    }

    std::cout << "通过\n";
}

int main()
{
    std::cout << "开始 LMSFilter 测试...\n\n";

    try
    {
        TestConstruction();
        TestSystemIdentification();
        TestNoiseCancellation();
        TestReset();
        TestStepSizeEffect();
        TestBoundaryConditions();

        std::cout << "\n所有测试通过！LMSFilter 实现正确。\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "\n测试失败: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "\n测试失败: 未知错误" << std::endl;
        return 1;
    }
}
