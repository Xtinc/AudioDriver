#include "audio_process.h"
#include <cmath>
#include <fstream>
#include <vector>

void test_static_characteristics()
{
    const unsigned int channels = 2;
    const int steps = 1000;
    float sample_rate = 44100.0f;
    DRCompressor comp1(sample_rate, channels, -20.0f, 2.0f, 0.006f, 0.01f, 10.0f);
    DRCompressor comp2(sample_rate, channels, -10.0f, 2.0f, 0.006f, 0.01f, 10.0f);
    DRCompressor comp3(sample_rate, channels, -20.0f, 4.0f, 0.006f, 0.01f, 10.0f);
    DRCompressor comp4(sample_rate, channels, -20.0f, 2.0f, 0.006f, 0.01f, 2.0f);
    std::ofstream outfile("static_characteristics.csv");
    outfile
        << "input_db,comp1_db,comp2_db,comp3_db,comp4_db,input_level,comp1_level,comp2_level,comp3_level,comp4_level"
        << std::endl;

    for (int i = 0; i <= steps; i++)
    {
        float input_db = -60.0f + (60.0f * i) / steps;
        float normalized_level = std::pow(10.0f, input_db / 20.0f);
        PCM_TYPE input_level = static_cast<PCM_TYPE>(normalized_level * 32767.0f);
        input_db = 20.0f * std::log10((float)input_level / 32767.0f);

        PCM_TYPE buffer1[2] = {input_level, input_level};
        PCM_TYPE buffer2[2] = {input_level, input_level};
        PCM_TYPE buffer3[2] = {input_level, input_level};
        PCM_TYPE buffer4[2] = {input_level, input_level};
        float buffer1_float[2] = {buffer1[0] / 32767.0f, buffer1[1] / 32767.0f};
        float buffer2_float[2] = {buffer2[0] / 32767.0f, buffer2[1] / 32767.0f};
        float buffer3_float[2] = {buffer3[0] / 32767.0f, buffer3[1] / 32767.0f};
        float buffer4_float[2] = {buffer4[0] / 32767.0f, buffer4[1] / 32767.0f};

        ChannelBuffer<float> buffer1_cb(1, channels);
        ChannelBuffer<float> buffer2_cb(1, channels);
        ChannelBuffer<float> buffer3_cb(1, channels);
        ChannelBuffer<float> buffer4_cb(1, channels);

        for (unsigned int ch = 0; ch < channels; ch++)
        {
            buffer1_cb.channels_view()[ch][0] = buffer1_float[ch];
            buffer2_cb.channels_view()[ch][0] = buffer2_float[ch];
            buffer3_cb.channels_view()[ch][0] = buffer3_float[ch];
            buffer4_cb.channels_view()[ch][0] = buffer4_float[ch];
        }

        float gain = 0.0f;
        comp1.process(&buffer1_cb, gain);
        comp2.process(&buffer2_cb, gain);
        comp3.process(&buffer3_cb, gain);
        comp4.process(&buffer4_cb, gain);

        // buffer1[0] = static_cast<PCM_TYPE>(buffer1_cb.channels_view()[0][0] * 32767.0f);
        buffer1[1] = static_cast<PCM_TYPE>(buffer1_cb.channels_view()[1][0] * 32767.0f);
        buffer2[0] = static_cast<PCM_TYPE>(buffer2_cb.channels_view()[0][0] * 32767.0f);
        buffer2[1] = static_cast<PCM_TYPE>(buffer2_cb.channels_view()[1][0] * 32767.0f);
        buffer3[0] = static_cast<PCM_TYPE>(buffer3_cb.channels_view()[0][0] * 32767.0f);
        buffer3[1] = static_cast<PCM_TYPE>(buffer3_cb.channels_view()[1][0] * 32767.0f);
        buffer4[0] = static_cast<PCM_TYPE>(buffer4_cb.channels_view()[0][0] * 32767.0f);
        buffer4[1] = static_cast<PCM_TYPE>(buffer4_cb.channels_view()[1][0] * 32767.0f);

        float output_level1 = static_cast<float>(std::abs(buffer1[0])) / 32767.0f;
        float output_level2 = static_cast<float>(std::abs(buffer2[0])) / 32767.0f;
        float output_level3 = static_cast<float>(std::abs(buffer3[0])) / 32767.0f;
        float output_level4 = static_cast<float>(std::abs(buffer4[0])) / 32767.0f;

        float output_db1 = output_level1 > 0 ? 20.0f * std::log10(output_level1) : -100.0f;
        float output_db2 = output_level2 > 0 ? 20.0f * std::log10(output_level2) : -100.0f;
        float output_db3 = output_level3 > 0 ? 20.0f * std::log10(output_level3) : -100.0f;
        float output_db4 = output_level4 > 0 ? 20.0f * std::log10(output_level4) : -100.0f;

        outfile << input_db << "," << output_db1 << "," << output_db2 << "," << output_db3 << "," << output_db4 << ","
                << normalized_level << "," << output_level1 << "," << output_level2 << "," << output_level3 << ","
                << output_level4 << std::endl;
    }

    outfile.close();
}

void test_dynamic_response()
{
    float sample_rate = 44100.0f;
    unsigned int channels = 2;
    DRCompressor fast_comp(sample_rate, channels, -20.0f, 4.0f, 0.001f, 0.01f, 6.0f);
    DRCompressor med_comp(sample_rate, channels, -20.0f, 4.0f, 0.006f, 0.1f, 6.0f);
    DRCompressor slow_comp(sample_rate, channels, -20.0f, 4.0f, 0.1f, 0.5f, 6.0f);

    float test_duration = 8.0f;
    unsigned int samples = static_cast<unsigned int>(sample_rate * test_duration);
    std::vector<PCM_TYPE> input(samples * channels);
    std::vector<PCM_TYPE> output_fast(samples * channels);
    std::vector<PCM_TYPE> output_med(samples * channels);
    std::vector<PCM_TYPE> output_slow(samples * channels);

    for (unsigned int i = 0; i < samples; i++)
    {
        float amplitude;
        if (i < sample_rate * 2)
        {
            amplitude = 0.0f;
        }
        else if (i < sample_rate * 4)
        {
            amplitude = std::pow(10.0f, -40.0f / 20.0f) * 32767.0f;
        }
        else if (i < sample_rate * 6)
        {
            amplitude = std::pow(10.0f, -10.0f / 20.0f) * 32767.0f;
        }
        else
        {
            amplitude = std::pow(10.0f, -40.0f / 20.0f) * 32767.0f;
        }

        for (unsigned int ch = 0; ch < channels; ch++)
        {
            input[i * channels + ch] = static_cast<PCM_TYPE>(amplitude);
            output_fast[i * channels + ch] = input[i * channels + ch];
            output_med[i * channels + ch] = input[i * channels + ch];
            output_slow[i * channels + ch] = input[i * channels + ch];
        }
    }
    const unsigned int block_size = 128;
    for (unsigned int i = 0; i < samples; i += block_size)
    {
        unsigned int frames = std::min(block_size, samples - i);

        ChannelBuffer<float> fast_cb(frames, channels);
        ChannelBuffer<float> med_cb(frames, channels);
        ChannelBuffer<float> slow_cb(frames, channels);

        for (unsigned int f = 0; f < frames; f++)
        {
            for (unsigned int ch = 0; ch < channels; ch++)
            {
                fast_cb.channels_view()[ch][f] = output_fast[(i + f) * channels + ch] / 32767.0f;
                med_cb.channels_view()[ch][f] = output_med[(i + f) * channels + ch] / 32767.0f;
                slow_cb.channels_view()[ch][f] = output_slow[(i + f) * channels + ch] / 32767.0f;
            }
        }

        float gain = 0.0f;
        fast_comp.process(&fast_cb, gain);
        med_comp.process(&med_cb, gain);
        slow_comp.process(&slow_cb, gain);

        for (unsigned int f = 0; f < frames; f++)
        {
            for (unsigned int ch = 0; ch < channels; ch++)
            {
                output_fast[(i + f) * channels + ch] = static_cast<PCM_TYPE>(fast_cb.channels_view()[ch][f] * 32767.0f);
                output_med[(i + f) * channels + ch] = static_cast<PCM_TYPE>(med_cb.channels_view()[ch][f] * 32767.0f);
                output_slow[(i + f) * channels + ch] = static_cast<PCM_TYPE>(slow_cb.channels_view()[ch][f] * 32767.0f);
            }
        }
    }

    std::ofstream outfile("dynamic_response.csv");
    outfile << "time,input_db,fast_comp_db,med_comp_db,slow_comp_db" << std::endl;

    unsigned int decimation = static_cast<unsigned int>(sample_rate / 1000);

    for (unsigned int i = 0; i < samples; i += decimation)
    {
        float time = static_cast<float>(i) / sample_rate;

        float input_amplitude = std::abs(static_cast<float>(input[i * channels])) / 32767.0f;
        float fast_amplitude = std::abs(static_cast<float>(output_fast[i * channels])) / 32767.0f;
        float med_amplitude = std::abs(static_cast<float>(output_med[i * channels])) / 32767.0f;
        float slow_amplitude = std::abs(static_cast<float>(output_slow[i * channels])) / 32767.0f;

        float input_db = input_amplitude > 1e-6f ? 20.0f * std::log10(input_amplitude) : -120.0f;
        float fast_db = fast_amplitude > 1e-6f ? 20.0f * std::log10(fast_amplitude) : -120.0f;
        float med_db = med_amplitude > 1e-6f ? 20.0f * std::log10(med_amplitude) : -120.0f;
        float slow_db = slow_amplitude > 1e-6f ? 20.0f * std::log10(slow_amplitude) : -120.0f;

        outfile << time << "," << input_db << "," << fast_db << "," << med_db << "," << slow_db << std::endl;
    }

    outfile.close();
}

void test_makeup_gain()
{
    float sample_rate = 44100.0f;
    unsigned int channels = 2;
    DRCompressor compressor(sample_rate, channels, -20.0f, 4.0f, 1e-5f, 1e-5f, 6.0f);
    std::ofstream outfile("makeup_gain_test.csv");
    outfile << "input_db,gain,output_db" << std::endl;
    const int steps = 100;
    float input_db = -30.0f;
    float input_level = std::pow(10.0f, input_db / 20.0f) * 32767.0f;
    for (int i = 0; i <= steps; i++)
    {
        float gain = -10.0f + (20.0f * i) / steps;
        PCM_TYPE buffer[2] = {static_cast<PCM_TYPE>(input_level), static_cast<PCM_TYPE>(input_level)};

        float buffer_float[2] = {buffer[0] / 32767.0f, buffer[1] / 32767.0f};

        ChannelBuffer<float> cb(1, channels);
        for (unsigned int ch = 0; ch < channels; ch++)
        {
            cb.channels_view()[ch][0] = buffer_float[ch];
        }

        compressor.process(&cb, gain);

        for (unsigned int ch = 0; ch < channels; ch++)
        {
            buffer[ch] = static_cast<PCM_TYPE>(cb.channels_view()[ch][0] * 32767.0f);
        }

        float output_level = static_cast<float>(std::abs(buffer[0])) / 32767.0f;
        float output_db = output_level > 0 ? 20.0f * std::log10(output_level) : -120.0f;

        outfile << input_db << "," << gain << "," << output_db << std::endl;
    }

    outfile.close();
}

int main()
{
    test_static_characteristics();
    test_dynamic_response();
    test_makeup_gain();
    return 0;
}