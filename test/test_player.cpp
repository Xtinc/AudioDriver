#include "audio_stream.h"

int main(int argc, char **argv)
{
    AudioCenter center(false);
    center.create(110_otk, AudioDeviceName("default", 0), AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, false,
                  false);
    center.prepare();
    center.connect(USR_DUMMY_IN, USR_DUMMY_OUT);
    center.start();
    center.play("dukou.wav", 0, 110_otk);

    std::this_thread::sleep_for(std::chrono::seconds(5));
    center.set_volume(110_otk, 0);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    center.set_volume(110_otk, 25);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    center.set_volume(110_otk, 45);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    center.set_volume(110_otk, 75);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    center.mute(USR_DUMMY_IN, true);
    center.set_volume(110_otk, 100);
    std::this_thread::sleep_for(std::chrono::minutes(1));
    center.set_volume(110_otk, 60);
    std::this_thread::sleep_for(std::chrono::minutes(4));
    // AUDIO_DEBUG_PRINT("Stop playing: %s", player.stop("dukou_44100.wav").msg);
    return 0;
}