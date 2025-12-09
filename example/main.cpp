#include "audio_interface.h"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    std::cout << "Audio Example Application" << std::endl;
    AudioCenter center(true);
    center.prepare();
    center.start();
    std::this_thread::sleep_for(std::chrono::seconds(50));

    return 0;
}
