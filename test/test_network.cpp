#include "audio_interface.h"
#include <chrono>
#include <thread>

int main()
{
    AudioCenter center(true);

    // auto ias = center.create(IToken(20), {"dukou.wav", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, true);
    auto oas = center.create(OToken(120), {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, true);
    center.prepare();
    center.start();
    center.play("dukou.wav", 30, OToken(120), "192.168.1.6");

    std::this_thread::sleep_for(std::chrono::minutes(300));
    return 0;
}