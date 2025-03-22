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

RetCode AudioCenter::create(IToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                            unsigned int ch, bool enable_network, bool enable_reset)
{
    AUDIO_INFO_PRINT("Creating audio input stream with token %u", token.tok);

    if (center_state.load() != State::INIT)
    {
        AUDIO_ERROR_PRINT("Cannot create stream, AudioCenter not in INIT state");
        return RetCode::ESTATE;
    }

    if (!token)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return RetCode::EPARAM;
    }

    if (ias_map.find(token) != ias_map.end())
    {
        AUDIO_ERROR_PRINT("Token already exists: %u", token.tok);
        return RetCode::NOACTION;
    }

    ias_map[token] = std::make_shared<IAStream>(token, name, enum2val(ps), enum2val(bw), ch, enable_reset);

    if (enable_network)
    {
        ias_map[token]->initialize_network(net_mgr);
        AUDIO_DEBUG_PRINT("Network initialized for input stream %u", token.tok);
    }

    AUDIO_DEBUG_PRINT("Successfully created audio input stream with token %u", token.tok);
    return RetCode::OK;
}

RetCode AudioCenter::create(OToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                            unsigned int ch, bool enable_network, bool enable_reset)
{
    AUDIO_INFO_PRINT("Creating audio output stream with token %u", token.tok);

    if (center_state.load() != State::INIT)
    {
        AUDIO_ERROR_PRINT("Cannot create stream, AudioCenter not in INIT state");
        return RetCode::ESTATE;
    }

    if (!token)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return RetCode::EPARAM;
    }

    if (oas_map.find(token) != oas_map.end())
    {
        AUDIO_ERROR_PRINT("Token already exists: %u", token.tok);
        return RetCode::NOACTION;
    }

    oas_map[token] = std::make_shared<OAStream>(token, name, enum2val(ps), enum2val(bw), ch, enable_reset);

    if (enable_network)
    {
        oas_map[token]->initialize_network(net_mgr);
        AUDIO_DEBUG_PRINT("Network initialized for output stream %u", token.tok);
    }

    AUDIO_DEBUG_PRINT("Successfully created audio output stream with token %u", token.tok);
    return RetCode::OK;
}

RetCode AudioCenter::prepare()
{
    AUDIO_INFO_PRINT("AudioCenter state try to change from INIT to CONNECTING");

    State expected = State::INIT;
    if (!center_state.compare_exchange_strong(expected, State::CONNECTING))
    {
        AUDIO_ERROR_PRINT("AudioCenter not in INIT state");
        return {RetCode::ESTATE, "AudioCenter not in INIT state"};
    }

    AUDIO_INFO_PRINT("AudioCenter successfully prepared - transitioning from INIT to CONNECTING");
    return RetCode::OK;
}

RetCode AudioCenter::connect(IToken itoken, OToken otoken)
{
    AUDIO_INFO_PRINT("Connecting stream %u -> %u", itoken.tok, otoken.tok);

    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in CONNECTING state");
        return {RetCode::ESTATE, "AudioCenter not in CONNECTING state"};
    }

    if (!itoken || !otoken)
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
    AUDIO_INFO_PRINT("Disconnecting stream %u -> %u", itoken.tok, otoken.tok);

    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in CONNECTING state");
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
    AUDIO_INFO_PRINT("Connecting remote stream %u -> %u on %s", itoken.tok, otoken.tok, ip.c_str());

    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in CONNECTING state");
        return {RetCode::ESTATE, "AudioCenter not in CONNECTING state"};
    }

    if (!itoken || !otoken)
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
    AUDIO_INFO_PRINT("Disconnecting remote stream %u -> %u on %s", itoken.tok, otoken.tok, ip.c_str());

    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in CONNECTING state");
        return {RetCode::ESTATE, "AudioCenter not in CONNECTING state"};
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
    AUDIO_INFO_PRINT("Starting AudioCenter - transitioning from CONNECTING to READY");

    State expected = State::CONNECTING;
    if (!center_state.compare_exchange_strong(expected, State::READY))
    {
        AUDIO_ERROR_PRINT("AudioCenter not in CONNECTING state");
        return {RetCode::ESTATE, "AudioCenter not ready for start"};
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

    if (net_mgr)
    {
        auto ret = net_mgr->start();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to start NetWorker: %s", ret.what());
        }
    }

    AUDIO_INFO_PRINT("AudioCenter successfully started - transitioned to READY state");
    return RetCode::OK;
}

RetCode AudioCenter::stop()
{
    AUDIO_INFO_PRINT("Stopping AudioCenter components");
    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
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

    center_state.store(State::INIT);
    AUDIO_INFO_PRINT("AudioCenter successfully stopped - reset to INIT state");
    return RetCode::OK;
}

RetCode AudioCenter::mute(AudioToken token)
{
    AUDIO_INFO_PRINT("Muting audio stream with token %u", token.tok);

    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    if (!token)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    auto ias = ias_map.find(token);
    if (ias != ias_map.end())
    {
        ias->second->mute();
        AUDIO_INFO_PRINT("IAStream %u muted", token.tok);
        return {RetCode::OK, "IAStream muted"};
    }

    auto oas = oas_map.find(token);
    if (oas != oas_map.end())
    {
        oas->second->mute();
        AUDIO_INFO_PRINT("OAStream %u muted", token.tok);
        return {RetCode::OK, "OAStream muted"};
    }

    AUDIO_ERROR_PRINT("Token not found: %u", token.tok);
    return {RetCode::FAILED, "Token not found"};
}

RetCode AudioCenter::unmute(AudioToken token)
{
    AUDIO_INFO_PRINT("Unmuting audio stream with token %u", token.tok);

    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    if (!token)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    auto ias = ias_map.find(token);
    if (ias != ias_map.end())
    {
        ias->second->unmute();
        AUDIO_INFO_PRINT("IAStream %u unmuted", token.tok);
        return {RetCode::OK, "IAStream unmuted"};
    }

    auto oas = oas_map.find(token);
    if (oas != oas_map.end())
    {
        oas->second->unmute();
        AUDIO_ERROR_PRINT("OAStream %u unmuted", token.tok);
        return {RetCode::OK, "OAStream unmuted"};
    }

    AUDIO_ERROR_PRINT("Token not found: %u", token.tok);
    return {RetCode::FAILED, "Token not found"};
}

RetCode AudioCenter::play(const std::string &path, int cycles, OToken otoken)
{
    AUDIO_INFO_PRINT("Playing audio file %s on %u", path.c_str(), otoken.tok);

    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    auto oas = oas_map.find(otoken);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    return player->play(path, cycles, oas->second);
}

RetCode AudioCenter::play(const std::string &name, int cycles, OToken remote_token, const std::string &remote_ip)
{
    AUDIO_INFO_PRINT("Playing audio file %s on %u from %s", name.c_str(), remote_token.tok, remote_ip.c_str());

    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    if (!remote_token)
    {
        AUDIO_ERROR_PRINT("Invalid remote token: %u", remote_token.tok);
        return {RetCode::EPARAM, "Invalid remote token"};
    }

    return player->play(name, cycles, net_mgr, remote_token, remote_ip);
}

RetCode AudioCenter::stop(const std::string &path)
{
    AUDIO_INFO_PRINT("Stopping audio file %s", path.c_str());

    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
        return {RetCode::FAILED, "AudioCenter not in READY state"};
    }

    return player->stop(path);
}
