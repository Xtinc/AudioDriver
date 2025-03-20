#include "audio_interface.h"
#include <chrono>
#include <thread>

int main()
{
    AudioCenter center(false);

    auto ias = center.create(IToken(20), {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2);
    auto oas = center.create(OToken(20), {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2);

    center.connect(ias, oas);
    center.start();

    std::this_thread::sleep_for(std::chrono::seconds(300));
    return 0;
}