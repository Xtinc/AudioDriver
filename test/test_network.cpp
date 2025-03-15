#include "audio_stream.h"

int main()
{
    BackgroundService::instance().start();
    auto ias = std::make_shared<IAStream>(12, AudioDeviceName("dukou_44100.wav", 0), 20, 44100, 2, true);
    auto oas = std::make_shared<OAStream>(16, AudioDeviceName("N300", 1), 20, 44100, 2, true);

    ias->connect("127.0.0.1", 16);
    // ias->connect(oas);
    ias->start();
    oas->start();
    std::this_thread::sleep_for(std::chrono::seconds(400));
    BackgroundService::instance().stop();
    return 0;
}