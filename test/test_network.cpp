#include "audio_interface.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

int main()
{
    AudioCenter center(true);
    center.prepare(true);
    center.connect(100_itk, 200_otk, "127.0.0.1");
    center.start();
    // center.play("dukou.wav", 1, 120_otk);
    // center.set_player_volume(20);
    // center.connect(20_itk, 120_otk, "127.0.0.1");
    std::this_thread::sleep_for(std::chrono::minutes(300));
    return 0;
}