#include "audio_interface.h"
#include <chrono>
#include <thread>

int main()
{
    AudioCenter center(true);

    auto ias = center.create(IToken(20), {"dukou.wav", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, true);
    auto oas = center.create(OToken(20), {"default", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, true);
    center.prepare();
    center.connect(ias, oas, "127.0.0.1");
    center.start();

    std::this_thread::sleep_for(std::chrono::seconds(300));
    return 0;
}