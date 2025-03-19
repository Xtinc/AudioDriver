#include "audio_stream.h"

int main(int argc, char **argv)
{
    BackgroundService::instance().start();
    AudioPlayer player(USER_MAX_AUDIO_TOKEN);
    auto oas = std::make_shared<OAStream>(12, AudioDeviceName("default", 0), enum2val(AudioPeriodSize::INR_20MS),
                                          enum2val(AudioBandWidth::Full), 2);
    oas->start();
    AUDIO_DEBUG_PRINT("Start playing: %s", player.play("dukou_44100.wav", 1, oas).msg);
    std::this_thread::sleep_for(std::chrono::minutes(1));
    AUDIO_DEBUG_PRINT("Stop playing: %s", player.stop("dukou_44100.wav").msg);
    BackgroundService::instance().stop();
    return 0;
}