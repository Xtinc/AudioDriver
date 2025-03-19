#include "audio_interface.h"
#include "audio_monitor.h"
#include "audio_stream.h"

static void print_device_change_info(AudioDeviceEvent event, const AudioDeviceInfo &info)
{
    switch (event)
    {
    case AudioDeviceEvent::Added:
        AUDIO_INFO_PRINT("Device added: %s", info.name.c_str());
        break;
    case AudioDeviceEvent::Removed:
        AUDIO_INFO_PRINT("Device removed: %s", info.name.c_str());
        break;
    case AudioDeviceEvent::StateChanged:
        AUDIO_INFO_PRINT("Device state changed: %s", info.name.c_str());
        break;
    case AudioDeviceEvent::DefaultChanged:
        AUDIO_INFO_PRINT("Default device changed: %s", info.name.c_str());
        break;
    }
}

void start_audio_center()
{
    BackgroundService::instance().start();
}

void stop_audio_center()
{
    BackgroundService::instance().stop();
}

// AudioCenter
AudioCenter::AudioCenter()
{
    monitor = std::make_unique<AudioMonitor>(BG_SERVICE);
    monitor->RegisterCallback(this, print_device_change_info);
    player = std::make_unique<AudioPlayer>(USER_MAX_AUDIO_TOKEN);
}

AudioCenter::~AudioCenter()
{
    monitor->UnregisterCallback(this);
}

IToken AudioCenter::create(IToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                           unsigned int ch)
{
    if (token > USER_MAX_AUDIO_TOKEN)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token);
        return {};
    }

    if (ias_map.find(token) != ias_map.end())
    {
        AUDIO_ERROR_PRINT("Token already exists: %u", token);
        return {};
    }

    AUDIO_DEBUG_PRINT("Create audio input stream: %u", token);
    ias_map[token] = std::make_shared<IAStream>(token, name, enum2val(ps), enum2val(bw), ch);
    return token;
}

OToken AudioCenter::create(OToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                           unsigned int ch)
{
    if (token > USER_MAX_AUDIO_TOKEN)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token);
        return {};
    }

    if (oas_map.find(token) != oas_map.end())
    {
        AUDIO_ERROR_PRINT("Token already exists: %u", token);
        return {};
    }

    AUDIO_DEBUG_PRINT("Create audio output stream: %u", token);
    oas_map[token] = std::make_shared<OAStream>(token, name, enum2val(ps), enum2val(bw), ch);
    return OToken();
}

RetCode AudioCenter::connect(IToken itoken, OToken otoken)
{
    if (itoken > USER_MAX_AUDIO_TOKEN || otoken > USER_MAX_AUDIO_TOKEN)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u -> %u", itoken, otoken);
        return {RetCode::EPARAM, "Invalid token"};
    }

    auto ias = ias_map.find(itoken);
    if (ias == ias_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", itoken);
        return {RetCode::EPARAM, "Invalid input token"};
    }

    auto oas = oas_map.find(otoken);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    return ias->second->connect(oas->second);
}
