#include "audio_interface.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

int main()
{
    AudioCenter center(true);
    center.create(10_itk, {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_10MS, 2);
    center.create(120_otk, {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_10MS, 2);
    center.prepare(false);
    center.connect(10_itk, 120_otk, "127.0.0.1");
    center.start();
    // center.set_player_volume(20);

    // center.connect(20_itk, 120_otk, "127.0.0.1");
    std::this_thread::sleep_for(std::chrono::minutes(30));
    AUDIO_DEBUG_PRINT("Exiting test program");
    return 0;
}