#include "audio_stream.h"

int main()
{
    BackgroundService::instance().start();
    auto net_mgr = std::make_shared<NetWorker>(BG_SERVICE);
    auto ias = std::make_shared<IAStream>(12, AudioDeviceName("N300", 0), 20, 44100, 2);
    auto oas = std::make_shared<OAStream>(16, AudioDeviceName("N300", 0), 20, 44100, 2);
    {
        auto ias = std::make_shared<IAStream>(22, AudioDeviceName("dukou_44100.wav", 1), 20, 44100, 2);
        ias->initialize_network(net_mgr);
        ias->start();
    }

    ias->initialize_network(net_mgr);
    oas->initialize_network(net_mgr);

    net_mgr->add_destination(12, 16, "127.0.0.1");
    net_mgr->add_destination(22, 16, "127.0.0.1");

    net_mgr->start();
    ias->start();
    oas->start();
    std::this_thread::sleep_for(std::chrono::seconds(10));
    // BackgroundService::instance().enumerate_devices();
    std::this_thread::sleep_for(std::chrono::seconds(360));
    BackgroundService::instance().stop();
    return 0;
}