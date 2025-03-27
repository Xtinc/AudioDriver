#include "audio_interface.h"
#include <chrono>
#include <thread>

int main()
{
    AudioCenter center(true);

    center.create(IToken(20), {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, true, true);
    center.create(OToken(120), {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, true);
    center.prepare();
    center.start();
    center.connect(20_itk, 120_otk);
    std::this_thread::sleep_for(std::chrono::minutes(300));
    return 0;
}