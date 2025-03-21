#include "audio_interface.h"
#include "audio_monitor.h"
#include "audio_network.h"
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

// AudioCenter
AudioCenter::AudioCenter(bool enable_network) : center_state(State::INIT)
{
    monitor = std::make_unique<AudioMonitor>(BG_SERVICE);
    monitor->RegisterCallback(this, print_device_change_info);
    player = std::make_unique<AudioPlayer>(USER_MAX_AUDIO_TOKEN);
    if (enable_network)
    {
        net_mgr = std::make_shared<NetWorker>(BG_SERVICE);
    }
}

AudioCenter::~AudioCenter()
{
    auto ret = stop();
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to stop AudioCenter: %s", ret.what());
    }

    monitor->UnregisterCallback(this);
}

IToken AudioCenter::create(IToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                           unsigned int ch, bool enable_network, bool enable_reset)
{
    if (center_state.load() != State::INIT)
    {
        AUDIO_ERROR_PRINT("Cannot create stream, AudioCenter not in INIT state");
        return {};
    }

    if (token > USER_MAX_AUDIO_TOKEN)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return {};
    }

    if (ias_map.find(token) != ias_map.end())
    {
        AUDIO_ERROR_PRINT("Token already exists: %u", token.tok);
        return {};
    }

    AUDIO_DEBUG_PRINT("Create audio input stream: %u", token.tok);
    ias_map[token] = std::make_shared<IAStream>(token, name, enum2val(ps), enum2val(bw), ch, enable_reset);

    if (enable_network)
    {
        ias_map[token]->initialize_network(net_mgr);
    }

    return token;
}

OToken AudioCenter::create(OToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                           unsigned int ch, bool enable_network, bool enable_reset)
{
    if (center_state.load() != State::INIT)
    {
        AUDIO_ERROR_PRINT("Cannot create stream, AudioCenter not in INIT state");
        return {};
    }

    if (token > USER_MAX_AUDIO_TOKEN)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return {};
    }

    if (oas_map.find(token) != oas_map.end())
    {
        AUDIO_ERROR_PRINT("Token already exists: %u", token.tok);
        return {};
    }

    AUDIO_DEBUG_PRINT("Create audio output stream: %u", token.tok);
    oas_map[token] = std::make_shared<OAStream>(token, name, enum2val(ps), enum2val(bw), ch, enable_reset);

    if (enable_network)
    {
        oas_map[token]->initialize_network(net_mgr);
    }

    return token;
}

RetCode AudioCenter::prepare()
{
    State expected = State::INIT;
    if (!center_state.compare_exchange_strong(expected, State::CONNECTING))
    {
        return {RetCode::FAILED, "AudioCenter not in INIT state"};
    }

    AUDIO_INFO_PRINT("AudioCenter prepared for connections");
    return RetCode::OK;
}

RetCode AudioCenter::connect(IToken itoken, OToken otoken)
{
    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        return {RetCode::FAILED, "AudioCenter not in CONNECTING state"};
    }

    if (itoken > USER_MAX_AUDIO_TOKEN || otoken > USER_MAX_AUDIO_TOKEN)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u -> %u", itoken.tok, otoken.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    auto ias = ias_map.find(itoken);
    if (ias == ias_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", itoken.tok);
        return {RetCode::EPARAM, "Invalid input token"};
    }

    auto oas = oas_map.find(otoken);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    return ias->second->connect(oas->second);
}

RetCode AudioCenter::disconnect(IToken itoken, OToken otoken)
{
    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        return {RetCode::FAILED, "AudioCenter not in CONNECTING state"};
    }

    auto ias = ias_map.find(itoken);
    if (ias == ias_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", itoken.tok);
        return {RetCode::EPARAM, "Invalid input token"};
    }

    auto oas = oas_map.find(otoken);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    return ias->second->disconnect(oas->second);
}

RetCode AudioCenter::connect(IToken itoken, OToken otoken, const std::string &ip)
{
    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        return {RetCode::FAILED, "AudioCenter not in CONNECTING state"};
    }

    if (itoken > USER_MAX_AUDIO_TOKEN || otoken > USER_MAX_AUDIO_TOKEN)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u -> %u", itoken.tok, otoken.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    if (!net_mgr)
    {
        AUDIO_ERROR_PRINT("Network not enabled");
        return {RetCode::FAILED, "Network not enabled"};
    }

    auto ias = ias_map.find(itoken);
    if (ias == ias_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", itoken.tok);
        return {RetCode::EPARAM, "Invalid input token"};
    }
    
    return net_mgr->add_destination(itoken, otoken, ip);
}

RetCode AudioCenter::disconnect(IToken itoken, OToken otoken, const std::string &ip)
{
    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        return {RetCode::FAILED, "AudioCenter not in CONNECTING state"};
    }

    if (!net_mgr)
    {
        AUDIO_ERROR_PRINT("Network not enabled");
        return {RetCode::FAILED, "Network not enabled"};
    }

    auto ias = ias_map.find(itoken);
    if (ias == ias_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", itoken.tok);
        return {RetCode::EPARAM, "Invalid input token"};
    }

    return net_mgr->del_destination(itoken, otoken, ip);
}

RetCode AudioCenter::start()
{
    State expected = State::CONNECTING;
    if (!center_state.compare_exchange_strong(expected, State::READY))
    {
        return {RetCode::FAILED, "AudioCenter not ready for start"};
    }

    if (net_mgr)
    {
        auto ret = net_mgr->start();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to start NetWorker: %s", ret.what());
        }
    }

    for (const auto &pair : oas_map)
    {
        auto ret = pair.second->start();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to start OAStream: %u", pair.first);
        }
    }

    for (const auto &pair : ias_map)
    {
        auto ret = pair.second->start();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to start IAStream: %u", pair.first);
        }
    }

    return RetCode::OK;
}

RetCode AudioCenter::stop()
{
    if (center_state.load() != State::READY)
    {
        return {RetCode::NOACTION, "AudioCenter not in READY state"};
    }

    if (net_mgr)
    {
        auto ret = net_mgr->stop();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to stop NetWorker %s", ret.what());
        }
    }

    for (const auto &pair : oas_map)
    {
        auto ret = pair.second->stop();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to stop OAStream: %u", pair.first);
        }
    }

    for (const auto &pair : ias_map)
    {
        auto ret = pair.second->stop();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to stop IAStream: %u", pair.first);
        }
    }

    return RetCode::OK;
}

RetCode AudioCenter::mute(AudioToken token)
{
    if (center_state.load() != State::READY)
    {
        return {RetCode::FAILED, "AudioCenter not in READY state"};
    }

    if (token > USER_MAX_AUDIO_TOKEN)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    auto ias = ias_map.find(token);
    if (ias != ias_map.end())
    {
        ias->second->mute();
        return {RetCode::OK, "IAStream muted"};
    }
    auto oas = oas_map.find(token);
    if (oas != oas_map.end())
    {
        oas->second->mute();
        return {RetCode::OK, "OAStream muted"};
    }

    return {RetCode::FAILED, "Token not found"};
}

RetCode AudioCenter::unmute(AudioToken token)
{
    if (center_state.load() != State::READY)
    {
        return {RetCode::FAILED, "AudioCenter not in READY state"};
    }

    if (token > USER_MAX_AUDIO_TOKEN)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    auto ias = ias_map.find(token);
    if (ias != ias_map.end())
    {
        ias->second->unmute();
        return {RetCode::OK, "IAStream unmuted"};
    }
    auto oas = oas_map.find(token);
    if (oas != oas_map.end())
    {
        oas->second->unmute();
        return {RetCode::OK, "OAStream unmuted"};
    }

    return {RetCode::FAILED, "Token not found"};
}

RetCode AudioCenter::play(const std::string &path, int cycles, OToken otoken)
{
    if (center_state.load() != State::READY)
    {
        return {RetCode::FAILED, "AudioCenter not in READY state"};
    }

    auto oas = oas_map.find(otoken);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    return player->play(path, cycles, oas->second);
}

RetCode AudioCenter::stop(const std::string &path)
{
    if (center_state.load() != State::READY)
    {
        return {RetCode::FAILED, "AudioCenter not in READY state"};
    }

    return player->stop(path);
}
