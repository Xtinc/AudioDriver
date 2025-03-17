#include "audio_stream.h"

int main()
{
    BackgroundService::instance().start();
    // auto net_mgr = std::make_shared<NetWorker>(BG_SERVICE);
    auto ias = std::make_shared<IAStream>(12, AudioDeviceName("dukou_44100.wav", 0), 20, 44100, 2);
    auto oas = std::make_shared<OAStream>(16, AudioDeviceName("N300", 0), 20, 44100, 2);
    auto echo = std::make_shared<IAStream>(22, AudioDeviceName{"echo", 0}, 20, 24000, 1);
    auto wave_oas = std::make_shared<OAStream>(33, AudioDeviceName{"wave.wav", 0}, 20, 44100, 2);

    ias->connect(wave_oas);
    echo->connect(oas);
    wave_oas->register_listener(echo);

    ias->start();
    wave_oas->start();
    echo->start();
    oas->start();

    oas->start();
    std::this_thread::sleep_for(std::chrono::seconds(300));
    BackgroundService::instance().stop();
    return 0;
}