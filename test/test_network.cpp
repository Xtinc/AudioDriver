#include "audio_interface.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

int main()
{
    AudioCenter center(true);
    center.create(10_itk, {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_10MS, 2,
                  StreamFlags::Network | StreamFlags::CodecPCM | StreamFlags::ResetSoft);
    center.create(120_otk, {"default.wav", 10}, AudioBandWidth::Full, AudioPeriodSize::INR_10MS, 2,
                  StreamFlags::Network | StreamFlags::ResetSoft);
    center.prepare(false);
    center.connect(10_itk, 120_otk, "127.0.0.1");
    center.start();
    // center.play("dukou.wav", 1, 120_otk);
    // center.set_player_volume(20);
    // center.connect(20_itk, 120_otk, "127.0.0.1");
    std::this_thread::sleep_for(std::chrono::seconds(30));
    center.pause(120_otk);
    std::this_thread::sleep_for(std::chrono::seconds(30));
    center.resume(120_otk);
    std::this_thread::sleep_for(std::chrono::seconds(30));
    return 0;
}