#include "audio_interface.h"
#include <chrono>
#include <thread>

int main()
{
    AudioCenter center(true);

    center.create(IToken(20), {"dukou.wav", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, true);
    center.create(OToken(120), {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, true);
    center.prepare();
    center.start();
    center.connect(20_itk, 120_otk);

    for (unsigned int i = 0; i < 100; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        center.set_volume(20_itk, i);
    }

    std::this_thread::sleep_for(std::chrono::minutes(300));
    return 0;
}