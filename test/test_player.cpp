#include "audio_stream.h"

int main(int argc, char **argv)
{
    AudioPlayer player(USER_MAX_AUDIO_TOKEN);
    auto oas = std::make_shared<OAStream>(12, AudioDeviceName("default", 0), enum2val(AudioPeriodSize::INR_20MS),
                                          enum2val(AudioBandWidth::Full), 2);
    oas->start();
    AUDIO_DEBUG_PRINT("Start playing: %s", player.play("dukou.wav", 1, oas).msg);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    oas->set_volume(0);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    oas->set_volume(25);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    oas->set_volume(50);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    oas->set_volume(75);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    oas->set_volume(100);
    std::this_thread::sleep_for(std::chrono::minutes(4));
    // AUDIO_DEBUG_PRINT("Stop playing: %s", player.stop("dukou_44100.wav").msg);
    return 0;
}