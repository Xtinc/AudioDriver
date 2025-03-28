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

    monitor->UnregisterCallback();
}

RetCode AudioCenter::create(IToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                            unsigned int ch, bool enable_network, bool enable_reset)
{
    AUDIO_INFO_PRINT("Creating %s input stream with token %u", name.first.c_str(), token.tok);

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

    return RetCode::OK;
}

RetCode AudioCenter::create(OToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                            unsigned int ch, bool enable_network, bool enable_reset)
{
    AUDIO_INFO_PRINT("Creating %s output stream with token %u", name.first.c_str(), token.tok);

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

    return RetCode::OK;
}

RetCode AudioCenter::prepare()
{
    State expected = State::INIT;
    if (!center_state.compare_exchange_strong(expected, State::CONNECTING))
    {
        AUDIO_ERROR_PRINT("AudioCenter not in INIT state");
        return {RetCode::ESTATE, "AudioCenter not in INIT state"};
    }

    ias_map.emplace(USR_DUMMY_IN,
                    std::make_shared<IAStream>(USR_DUMMY_IN, AudioDeviceName("null", 0),
                                               enum2val(AudioPeriodSize::INR_20MS), enum2val(AudioBandWidth::Full), 2));
    oas_map.emplace(USR_DUMMY_OUT,
                    std::make_shared<OAStream>(USR_DUMMY_OUT, AudioDeviceName("null", 0),
                                               enum2val(AudioPeriodSize::INR_20MS), enum2val(AudioBandWidth::Full), 2));
    monitor->RegisterCallback(print_device_change_info);
    AUDIO_DEBUG_PRINT("AudioCenter successfully prepared - transitioning from INIT to CONNECTING");
    return RetCode::OK;
}

RetCode AudioCenter::connect(IToken itoken, OToken otoken, const std::string &ip)
{
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

    if (!ip.empty())
    {
        if (!net_mgr)
        {
            AUDIO_ERROR_PRINT("Network not enabled");
            return {RetCode::FAILED, "Network not enabled"};
        }

        AUDIO_INFO_PRINT("Try to connect stream %u -> %u on %s", itoken.tok, otoken.tok, ip.c_str());
        return net_mgr->add_destination(itoken, otoken, ip);
    }

    auto oas = oas_map.find(otoken);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    AUDIO_INFO_PRINT("Try to connect stream %u -> %u", itoken.tok, otoken.tok);
    return ias->second->connect(oas->second);
}

RetCode AudioCenter::disconnect(IToken itoken, OToken otoken, const std::string &ip)
{
    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in CONNECTING state");
        return {RetCode::ESTATE, "AudioCenter not in CONNECTING state"};
    }

    auto ias = ias_map.find(itoken);
    if (ias == ias_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", itoken.tok);
        return {RetCode::EPARAM, "Invalid input token"};
    }

    if (!ip.empty())
    {
        if (!net_mgr)
        {
            AUDIO_ERROR_PRINT("Network not enabled");
            return {RetCode::FAILED, "Network not enabled"};
        }

        AUDIO_INFO_PRINT("Try to disconnect stream %u -> %u on %s", itoken.tok, otoken.tok, ip.c_str());
        return net_mgr->del_destination(itoken, otoken, ip);
    }

    auto oas = oas_map.find(otoken);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    AUDIO_INFO_PRINT("Try to disconnect stream %u -> %u", itoken.tok, otoken.tok);
    return ias->second->disconnect(oas->second);
}

RetCode AudioCenter::register_callback(IToken token, AudioInputCallBack cb, void *ptr)
{
    if (center_state.load() != State::CONNECTING)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in CONNECTING state");
        return {RetCode::ESTATE, "AudioCenter not in CONNECTING state"};
    }

    if (!token)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    auto ias = ias_map.find(token);
    if (ias == ias_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", token.tok);
        return {RetCode::EPARAM, "Invalid input token"};
    }

    ias->second->register_callback(cb, ptr);
    AUDIO_DEBUG_PRINT("Callback registered for input stream %u", token.tok);
    return RetCode::OK;
}

RetCode AudioCenter::start()
{
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

    AUDIO_DEBUG_PRINT("AudioCenter successfully started - transitioned to READY state");
    return RetCode::OK;
}

RetCode AudioCenter::stop()
{
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
    AUDIO_DEBUG_PRINT("AudioCenter successfully stopped - reset to INIT state");
    return RetCode::OK;
}

RetCode AudioCenter::set_volume(AudioToken token, unsigned int vol)
{
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
        ias->second->set_volume(vol);
        return RetCode::OK;
    }

    auto oas = oas_map.find(token);
    if (oas != oas_map.end())
    {
        oas->second->set_volume(vol);
        return RetCode::OK;
    }

    AUDIO_DEBUG_PRINT("Token not found: %u", token.tok);
    return {RetCode::FAILED, "Token not found"};
}

RetCode AudioCenter::mute(AudioToken token, bool enable)
{
    const char *action = enable ? "Muting" : "Unmuting";

    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("Cannot %s, AudioCenter not in READY state", action);
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    if (!token)
    {
        AUDIO_ERROR_PRINT("Cannot %s, invalid token: %u", action, token.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    auto ias = ias_map.find(token);
    if (ias != ias_map.end())
    {
        enable ? ias->second->mute() : ias->second->unmute();
        return RetCode::OK;
    }

    auto oas = oas_map.find(token);
    if (oas != oas_map.end())
    {
        enable ? oas->second->mute() : oas->second->unmute();
        return RetCode::OK;
    }

    AUDIO_ERROR_PRINT("Cannot %s, token not found: %u", action, token.tok);
    return {RetCode::FAILED, "Token not found"};
}

RetCode AudioCenter::mute(OToken otoken, IToken itoken, bool enable, const std::string &ip)
{
    const char *action = enable ? "Muting" : "Unmuting";

    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("Cannot %s, AudioCenter not in READY state", action);
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    if (!otoken)
    {
        AUDIO_ERROR_PRINT("Cannot %s, invalid token: %u", action, otoken.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    auto oas = oas_map.find(otoken);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    return enable ? oas->second->mute(itoken, ip) : oas->second->unmute(itoken, ip);
}

RetCode AudioCenter::play(const std::string &name, int cycles, OToken otoken, const std::string &ip)
{
    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    if (!otoken)
    {
        AUDIO_ERROR_PRINT("Invalid remote token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid remote token"};
    }

    if (!ip.empty())
    {
        AUDIO_INFO_PRINT("Try to play file %s -> %u from %s", name.c_str(), otoken.tok, ip.c_str());
        return player->play(name, cycles, net_mgr, otoken, ip);
    }

    auto oas = oas_map.find(otoken);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    AUDIO_INFO_PRINT("Try to play file %s -> %u", name.c_str(), otoken.tok);
    return player->play(name, cycles, oas->second);
}

RetCode AudioCenter::stop(const std::string &path)
{
    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    AUDIO_INFO_PRINT("Stopping file %s", path.c_str());
    return player->stop(path);
}
