#include "../source/audio_wavfile.h"
#include "../source/ns/noise_supression.h"
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

constexpr int SAMPLE_RATE = 16000;
constexpr int FRAME_SIZE_MS = 10;
constexpr int SAMPLES_PER_10MS = SAMPLE_RATE / 100; // 160 samples per 10ms at 16kHz

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "用法: " << argv[0] << " <输入WAV文件> [输出WAV文件]" << std::endl;
        std::cerr << "如果不指定输出文件，将自动命名为 'ns_output.wav'" << std::endl;
        return 1;
    }

    const std::string input_file = argv[1];
    std::string output_file = (argc > 2) ? argv[2] : "ns_output.wav";

    // 打开输入WAV文件
    WavFile input_wav;
    auto ret = input_wav.open(input_file, WavFile::mode::in);
    if (!ret)
    {
        std::cerr << "错误: 无法打开输入WAV文件: " << input_file << std::endl;
        std::cerr << "原因: " << ret.what() << std::endl;
        return 1;
    }

    // 验证输入文件的采样率和通道数
    if (input_wav.sample_rate() != SAMPLE_RATE)
    {
        std::cerr << "错误: 输入WAV文件采样率必须是16kHz，当前是: " << input_wav.sample_rate() << "Hz" << std::endl;
        return 1;
    }

    if (input_wav.channel_number() != 1)
    {
        std::cerr << "错误: 输入WAV文件必须是单通道，当前通道数是: " << input_wav.channel_number() << std::endl;
        return 1;
    }

    // 创建输出WAV文件
    WavFile output_wav;
    ret = output_wav.open(output_file, WavFile::mode::out);
    if (!ret)
    {
        std::cerr << "错误: 无法创建输出WAV文件: " << output_file << std::endl;
        std::cerr << "原因: " << ret.what() << std::endl;
        return 1;
    }

    // 配置输出WAV文件参数与输入保持一致
    output_wav.set_channel_number(input_wav.channel_number());
    output_wav.set_sample_rate(input_wav.sample_rate());
    output_wav.set_bits_per_sample(input_wav.bits_per_sample());

    std::cout << "处理WAV文件 " << input_file << std::endl;
    std::cout << "采样率: " << input_wav.sample_rate() << "Hz" << std::endl;
    std::cout << "通道数: " << input_wav.channel_number() << std::endl;
    std::cout << "位深度: " << input_wav.bits_per_sample() << "bit" << std::endl;
    std::cout << "处理帧大小: " << FRAME_SIZE_MS << "ms (" << SAMPLES_PER_10MS << " 样本)" << std::endl;

    // 创建噪声抑制实例
    // 使用中等强度级别(2)的噪声抑制，单通道
    NoiseSuppressor ns(1, 1);
    ns.SetCaptureOutputUsage(true);

    // 缓冲区用于存放PCM数据
    std::vector<int16_t> pcm_buffer(SAMPLES_PER_10MS);
    // 浮点缓冲区用于传递给噪声抑制算法
    std::array<float, kNsFrameSize> float_buffer;

    // 统计处理的帧数
    size_t frames_processed = 0;
    uint64_t total_samples = input_wav.frame_number();
    uint64_t total_frames = total_samples / SAMPLES_PER_10MS;

    // 主处理循环
    bool finished = false;
    while (!finished)
    {
        // 从输入WAV文件读取10ms的PCM数据
        ret = input_wav.read(pcm_buffer.data(), SAMPLES_PER_10MS);
        if (!ret)
        {
            if (ret.err == RetCode::EREAD)
            {
                std::cout << "到达文件结尾" << std::endl;
                finished = true;
                break;
            }
            else
            {
                std::cerr << "读取输入文件时出错: " << ret.what() << std::endl;
                return 1;
            }
        }

        frames_processed++;
        if (frames_processed % 100 == 0)
        {
            std::cout << "已处理 " << frames_processed << "/" << total_frames << " 帧 ("
                      << (frames_processed * 100.0 / total_frames) << "%)" << std::endl;
        }

        // 将PCM数据转换为浮点数据
        for (int i = 0; i < SAMPLES_PER_10MS; i++)
        {
            float_buffer[i] = static_cast<float>(pcm_buffer[i]) / 32768.0f;
        }

        // 执行噪声抑制
        ns.Analyze(float_buffer);
        ns.Process(float_buffer);

        // 将浮点数据转换回PCM数据
        for (int i = 0; i < SAMPLES_PER_10MS; i++)
        {
            float sample = float_buffer[i] * 32768.0f;
            sample = std::min(32767.0f, std::max(-32768.0f, sample));
            pcm_buffer[i] = static_cast<int16_t>(sample);
        }

        // 将处理后的数据写入输出WAV文件
        ret = output_wav.write(pcm_buffer.data(), SAMPLES_PER_10MS);
        if (!ret)
        {
            std::cerr << "写入输出文件时出错: " << ret.what() << std::endl;
            return 1;
        }
    }

    std::cout << "处理完成！共处理 " << frames_processed << " 帧" << std::endl;
    std::cout << "输出文件保存为: " << output_file << std::endl;

    return 0;
}