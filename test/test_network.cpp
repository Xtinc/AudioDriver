#include "audio_interface.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

static std::ofstream audio_file("record.pcm", std::ios::binary);
static void record_audio(const int16_t *data, unsigned int chan_num, unsigned int frame_num, void *user_ptr)
{
    if (!audio_file.is_open())
    {
        audio_file.open("record.wav", std::ios::binary);
        if (!audio_file)
        {
            std::cerr << "Failed to open audio file for recording." << std::endl;
            return;
        }
    }

    audio_file.write(reinterpret_cast<const char *>(data), chan_num * frame_num * sizeof(int16_t));
    if (!audio_file)
    {
        std::cerr << "Failed to write audio data to file." << std::endl;
    }
}

int main()
{
    AudioCenter center(true);

    // center.create(IToken(20), {"leopard.wav", 10}, AudioBandWidth::Full, AudioPeriodSize::INR_10MS, 1);
    // center.register_callback(IToken(20), record_audio, 1024, false, nullptr);
    center.create(OToken(120), {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 1, true);
    center.prepare();
    center.start();
    center.play("output.wav", 10, 120_otk);
    // center.connect(20_itk, 120_otk, "127.0.0.1");
    std::this_thread::sleep_for(std::chrono::minutes(300));
    return 0;
}