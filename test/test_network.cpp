#include "audio_interface.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

static void test_callback(const int16_t *data, unsigned int chan_num, unsigned int frame_num, void *user_ptr)
{
    auto center = static_cast<AudioCenter *>(user_ptr);
    center->direct_push_pcm(33_itk, 130_otk, chan_num, frame_num, 48000, data);
}

int main()
{
    AudioCenter center(false);
    center.create(120_otk, AudioDeviceName{"null", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2);
    center.create(130_otk, AudioDeviceName{"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2);
    center.create(20_itk, 120_otk);
    center.register_callback(20_itk, test_callback, 960, UsrCallBackMode::OBSERVER, &center);
    center.prepare(false);
    // center.connect(20_itk, 130_otk);
    center.start();
    center.play("dukou.wav", 1, 120_otk);
    // center.set_player_volume(20);
    // center.connect(20_itk, 120_otk, "127.0.0.1");
    std::this_thread::sleep_for(std::chrono::minutes(300));
    return 0;
}