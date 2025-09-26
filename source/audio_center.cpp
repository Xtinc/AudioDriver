#include "audio_config.h"
#include "audio_interface.h"
#include "audio_monitor.h"
#include "audio_network.h"
#include "audio_stream.h"
#include <iomanip>

// AudioCenter
AudioCenter::AudioCenter(bool enable_network, unsigned short port, const std::string &local_ip)
    : center_state(State::INIT)
{
    config = std::make_unique<INIReader>("audiocenter.ini");
    monitor = std::make_unique<AudioMonitor>(BG_SERVICE);
    player = std::make_shared<AudioPlayer>(WAVE_PLAYER_TOKEN.tok);
    if (enable_network)
    {
        net_mgr = std::make_shared<NetWorker>(BG_SERVICE, port, local_ip);
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

    if (ias_map.find(token.tok) != ias_map.end())
    {
        AUDIO_ERROR_PRINT("Token already exists: %u", token.tok);
        return RetCode::NOACTION;
    }

    ias_map[token.tok] = std::make_shared<IAStream>(token.tok, name, enum2val(ps), enum2val(bw), ch, enable_reset);

    if (enable_network && net_mgr)
    {
        auto ret = ias_map[token.tok]->initialize_network(net_mgr);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to initialize network for input stream %u: %s", token.tok, ret.what());
            return ret;
        }
        AUDIO_DEBUG_PRINT("Network initialized for input stream %u", token.tok);
    }

    return RetCode::OK;
}

RetCode AudioCenter::create(IToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                            unsigned int dev_ch, const AudioChannelMap &imap, bool enable_network)
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

    if (dev_ch < 2)
    {
        AUDIO_ERROR_PRINT("Invalid device channel: %u, when channel map is specfied, at least two channel is necessary",
                          dev_ch);
        return RetCode::EPARAM;
    }

    if (imap[0] >= dev_ch || imap[1] >= dev_ch)
    {
        AUDIO_ERROR_PRINT("Invalid channel map: (%u, %u), total channel number is %u", imap[0], imap[1], dev_ch);
        return RetCode::EPARAM;
    }

    if (ias_map.find(token.tok) != ias_map.end())
    {
        AUDIO_ERROR_PRINT("Token already exists: %u", token.tok);
        return RetCode::NOACTION;
    }

    ias_map[token.tok] = std::make_shared<IAStream>(token.tok, name, enum2val(ps), enum2val(bw), dev_ch, imap);

    if (enable_network && net_mgr)
    {
        auto ret = ias_map[token.tok]->initialize_network(net_mgr);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to initialize network for input stream %u: %s", token.tok, ret.what());
            return ret;
        }
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

    if (oas_map.find(token.tok) != oas_map.end())
    {
        AUDIO_ERROR_PRINT("Token already exists: %u", token.tok);
        return RetCode::NOACTION;
    }

    oas_map[token.tok] = std::make_shared<OAStream>(token.tok, name, enum2val(ps), enum2val(bw), ch, enable_reset);

    if (enable_network && net_mgr)
    {
        auto ret = oas_map[token.tok]->initialize_network(net_mgr);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to initialize network for output stream %u: %s", token.tok, ret.what());
            return ret;
        }
        AUDIO_DEBUG_PRINT("Network initialized for output stream %u", token.tok);
    }

    return RetCode::OK;
}

RetCode AudioCenter::create(OToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                            unsigned int dev_ch, const AudioChannelMap &omap, bool enable_network)
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

    if (dev_ch < 2)
    {
        AUDIO_ERROR_PRINT("Invalid device channel: %u, when channel map is specfied, at least two channel is necessary",
                          dev_ch);
        return RetCode::EPARAM;
    }

    if (omap[0] >= dev_ch || omap[1] >= dev_ch)
    {
        AUDIO_ERROR_PRINT("Invalid channel map: (%u, %u), total channel number is %u", omap[0], omap[1], dev_ch);
        return RetCode::EPARAM;
    }

    if (oas_map.find(token.tok) != oas_map.end())
    {
        AUDIO_ERROR_PRINT("Token already exists: %u", token.tok);
        return RetCode::NOACTION;
    }

    oas_map[token.tok] = std::make_shared<OAStream>(token.tok, name, enum2val(ps), enum2val(bw), dev_ch, false, omap);

    if (enable_network && net_mgr)
    {
        auto ret = oas_map[token.tok]->initialize_network(net_mgr);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to initialize network for output stream %u: %s", token.tok, ret.what());
            return ret;
        }
        AUDIO_DEBUG_PRINT("Network initialized for output stream %u", token.tok);
    }

    return RetCode::OK;
}

RetCode AudioCenter::create(IToken itoken, OToken otoken, bool enable_network)
{
    AUDIO_INFO_PRINT("Creating linked streams with tokens %u -> %u", itoken.tok, otoken.tok);

    if (center_state.load() != State::INIT)
    {
        AUDIO_ERROR_PRINT("Cannot create stream, AudioCenter not in INIT state");
        return RetCode::ESTATE;
    }

    if (!itoken)
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", itoken.tok);
        return RetCode::EPARAM;
    }

    if (!otoken)
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return RetCode::EPARAM;
    }

    auto oas = oas_map.find(otoken.tok);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Output token don't exists: %u", otoken.tok);
        return RetCode::EPARAM;
    }

    if (ias_map.find(itoken.tok) != ias_map.end())
    {
        AUDIO_ERROR_PRINT("Input token already exists: %u", itoken.tok);
        return RetCode::EPARAM;
    }

    if (oas->second->omap == DEFAULT_MONO_MAP)
    {
        ias_map[itoken.tok] =
            std::make_shared<IAStream>(itoken.tok, AudioDeviceName("null", 0), enum2val(AudioPeriodSize::INR_20MS),
                                       enum2val(AudioBandWidth::Full), 1, false);
    }
    else if (oas->second->omap == DEFAULT_DUAL_MAP)
    {
        ias_map[itoken.tok] =
            std::make_shared<IAStream>(itoken.tok, AudioDeviceName("null", 0), enum2val(AudioPeriodSize::INR_20MS),
                                       enum2val(AudioBandWidth::Full), 2, false);
    }
    else
    {
        ias_map[itoken.tok] =
            std::make_shared<IAStream>(itoken.tok, AudioDeviceName("null", 0), enum2val(AudioPeriodSize::INR_20MS),
                                       enum2val(AudioBandWidth::Full), 99, oas->second->omap);
    }

    if (enable_network && net_mgr)
    {
        auto ret = ias_map[itoken.tok]->initialize_network(net_mgr);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to initialize network for input stream %u: %s", itoken.tok, ret.what());
            return ret;
        }
        AUDIO_DEBUG_PRINT("Network initialized for input stream %u", itoken.tok);
    }

    oas->second->register_listener(ias_map[itoken.tok]);

    return RetCode::OK;
}

RetCode AudioCenter::prepare(bool enable_usb_detection)
{
    State expected = State::INIT;
    if (!center_state.compare_exchange_strong(expected, State::CONNECTING))
    {
        AUDIO_ERROR_PRINT("AudioCenter not in INIT state");
        return {RetCode::ESTATE, "AudioCenter not in INIT state"};
    }

    if (!enable_usb_detection)
    {
        AUDIO_INFO_PRINT(
            "AudioCenter successfully prepared - transitioning from INIT to CONNECTING, disabling USB detection");
        return RetCode::OK;
    }

    auto usr_cfg = config->LoadDeviceConfig();
    auto default_usb_in = usr_cfg.input_device_id.empty() ? "null" : usr_cfg.input_device_id;
    auto default_usb_out = usr_cfg.output_device_id.empty() ? "null" : usr_cfg.output_device_id;

    ias_map.emplace(USR_DUMMY_IN.tok, std::make_shared<IAStream>(USR_DUMMY_IN.tok, AudioDeviceName(default_usb_in, 0),
                                                                 enum2val(AudioPeriodSize::INR_20MS),
                                                                 enum2val(AudioBandWidth::Full), 2, false));
    oas_map.emplace(USR_DUMMY_OUT.tok,
                    std::make_shared<OAStream>(USR_DUMMY_OUT.tok, AudioDeviceName(default_usb_out, 0),
                                               enum2val(AudioPeriodSize::INR_20MS), enum2val(AudioBandWidth::Full), 2));
    if (net_mgr)
    {
        ias_map[USR_DUMMY_IN.tok]->initialize_network(net_mgr);
        oas_map[USR_DUMMY_OUT.tok]->initialize_network(net_mgr);
    }

    monitor->RegisterCallback([this](AudioDeviceEvent event, const AudioDeviceInfo &info) {
        auto ias = ias_map.find(USR_DUMMY_IN.tok);
        if (ias == ias_map.end())
        {
            AUDIO_ERROR_PRINT("Dummy input stream not found");
            return;
        }

        auto oas = oas_map.find(USR_DUMMY_OUT.tok);
        if (oas == oas_map.end())
        {
            AUDIO_ERROR_PRINT("Dummy output stream not found");
            return;
        }

        DeviceConfig usr_cfg = config->LoadDeviceConfig();

        switch (event)
        {
        case AudioDeviceEvent::Added:
            AUDIO_INFO_PRINT("New device: %s", info.name.c_str());
            if (info.type == AudioDeviceType::Capture)
            {
                ias->second->restart({info.id, 0});
                usr_cfg.input_device_id = info.id;
                usr_cfg.input_device_name = info.name;
                config->SaveDeviceConfig(usr_cfg);
            }
            else if (info.type == AudioDeviceType::Playback)
            {
                oas->second->restart({info.id, 0});
                usr_cfg.output_device_id = info.id;
                usr_cfg.output_device_name = info.name;
                config->SaveDeviceConfig(usr_cfg);
            }
            else
            {
                ias->second->restart({info.id, 0});
                oas->second->restart({info.id, 0});
                usr_cfg.input_device_id = info.id;
                usr_cfg.input_device_name = info.name;
                usr_cfg.output_device_id = info.id;
                usr_cfg.output_device_name = info.name;
                config->SaveDeviceConfig(usr_cfg);
            }
            break;
        case AudioDeviceEvent::Removed:
            AUDIO_INFO_PRINT("Del device: %s", info.name.c_str());
            if (ias->second->name().first == info.id)
            {
                ias->second->restart({"null", 0});
                usr_cfg.input_device_id = "null";
                usr_cfg.input_device_name = "null";
                config->SaveDeviceConfig(usr_cfg);
            }
            if (oas->second->name().first == info.id)
            {
                oas->second->restart({"null", 0});
                usr_cfg.output_device_id = "null";
                usr_cfg.output_device_name = "null";
                config->SaveDeviceConfig(usr_cfg);
            }
            break;
        default:
            break;
        }
    });
    AUDIO_DEBUG_PRINT(
        "AudioCenter successfully prepared - transitioning from INIT to CONNECTING, enabling USB detection");
    return RetCode::OK;
}

RetCode AudioCenter::connect(IToken itoken, OToken otoken, const std::string &ip, unsigned short port)
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

    auto ias = ias_map.find(itoken.tok);
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
        auto ret = net_mgr->add_destination(itoken.tok, otoken.tok, ip, port);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to add network destination %s for tokens %u->%u: %s", ip.c_str(), itoken.tok,
                              otoken.tok, ret.what());
        }
        return ret;
    }

    auto oas = oas_map.find(otoken.tok);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    AUDIO_INFO_PRINT("Try to connect stream %u -> %u", itoken.tok, otoken.tok);
    auto ret = ias->second->connect(oas->second);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to connect stream %u -> %u: %s", itoken.tok, otoken.tok, ret.what());
    }
    return ret;
}

RetCode AudioCenter::disconnect(IToken itoken, OToken otoken, const std::string &ip, unsigned short port)
{
    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in CONNECTING state");
        return {RetCode::ESTATE, "AudioCenter not in CONNECTING state"};
    }

    auto ias = ias_map.find(itoken.tok);
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
        auto ret = net_mgr->del_destination(itoken.tok, otoken.tok, ip, port);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to remove network destination %s for tokens %u->%u: %s", ip.c_str(), itoken.tok,
                              otoken.tok, ret.what());
        }
        return ret;
    }

    auto oas = oas_map.find(otoken.tok);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    AUDIO_INFO_PRINT("Try to disconnect stream %u -> %u", itoken.tok, otoken.tok);
    auto ret = ias->second->disconnect(oas->second);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to disconnect stream %u -> %u: %s", itoken.tok, otoken.tok, ret.what());
    }
    return ret;
}

RetCode AudioCenter::register_callback(IToken token, AudioInputCallBack cb, unsigned int frames, UsrCallBackMode mode,
                                       void *ptr)
{
    auto state = center_state.load();
    if (state != State::CONNECTING && state != State::INIT)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in CONNECTING or INIT state");
        return {RetCode::ESTATE, "AudioCenter not in CONNECTING or INIT state"};
    }

    if (!token)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    // Find the stream
    auto it = ias_map.find(token.tok);
    if (it == ias_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", token.tok);
        return {RetCode::EPARAM, "Invalid input token"};
    }

    it->second->register_callback(cb, frames, mode, ptr);
    AUDIO_DEBUG_PRINT("Callback registered for input stream %u, mode: %d", token.tok, mode);
    return RetCode::OK;
}

RetCode AudioCenter::direct_push_pcm(IToken itoken, OToken otoken, unsigned int chan, unsigned int frames,
                                     unsigned int sample_rate, const int16_t *data)
{
    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    if (!itoken)
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", itoken.tok);
        return {RetCode::EPARAM, "Invalid input token"};
    }

    auto oas = oas_map.find(otoken.tok);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    if (!data)
    {
        AUDIO_ERROR_PRINT("Invalid data pointer");
        return {RetCode::EPARAM, "Invalid data pointer"};
    }

    return oas->second->direct_push(chan, frames, sample_rate, data, SourceUUID{itoken.tok, 0, 0});
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
            AUDIO_ERROR_PRINT("Failed to start OAStream: %u, %s", pair.first, ret.what());
        }
    }

    for (const auto &pair : ias_map)
    {
        auto ret = pair.second->start();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to start IAStream: %u, %s", pair.first, ret.what());
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

    center_state.store(State::INIT);

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
            AUDIO_ERROR_PRINT("Failed to stop OAStream: %u, %s", pair.first, ret.what());
        }
    }

    for (const auto &pair : ias_map)
    {
        auto ret = pair.second->stop();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to stop IAStream: %u, %s", pair.first, ret.what());
        }
    }

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

    auto ias = ias_map.find(token.tok);
    if (ias != ias_map.end())
    {
        ias->second->set_volume(vol);
        return RetCode::OK;
    }

    auto oas = oas_map.find(token.tok);
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

    auto ias = ias_map.find(token.tok);
    if (ias != ias_map.end())
    {
        enable ? ias->second->mute() : ias->second->unmute();
        return RetCode::OK;
    }

    auto oas = oas_map.find(token.tok);
    if (oas != oas_map.end())
    {
        enable ? oas->second->mute() : oas->second->unmute();
        return RetCode::OK;
    }

    AUDIO_ERROR_PRINT("Cannot %s, token not found: %u", action, token.tok);
    return {RetCode::FAILED, "Token not found"};
}

RetCode AudioCenter::mute(IToken itoken, OToken otoken, bool enable, const std::string &ip)
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

    auto oas = oas_map.find(otoken.tok);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    return enable ? oas->second->mute(itoken.tok, ip) : oas->second->unmute(itoken.tok, ip);
}

RetCode AudioCenter::play(const std::string &name, int cycles, OToken otoken, const std::string &ip,
                          unsigned short port)
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
        if (!net_mgr)
        {
            AUDIO_ERROR_PRINT("Network manager not initialized");
            return {RetCode::FAILED, "Network manager not initialized"};
        }

        AUDIO_INFO_PRINT("Try to play file %s -> %u from %s", name.c_str(), otoken.tok, ip.c_str());
        auto ret = player->play(name, cycles, net_mgr, otoken.tok, ip, port);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to play file %s to %s for token %u: %s", name.c_str(), ip.c_str(), otoken.tok,
                              ret.what());
        }
        return ret;
    }

    auto oas = oas_map.find(otoken.tok);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    AUDIO_INFO_PRINT("Try to play file %s -> %u", name.c_str(), otoken.tok);
    auto ret = player->play(name, cycles, oas->second);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to play file %s to output token %u: %s", name.c_str(), otoken.tok, ret.what());
    }
    return ret;
}

RetCode AudioCenter::stop(const std::string &path)
{
    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    AUDIO_INFO_PRINT("Stopping file %s", path.c_str());
    auto ret = player->stop(path);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to stop file %s: %s", path.c_str(), ret.what());
    }
    return ret;
}

RetCode AudioCenter::set_player_volume(unsigned int vol)
{
    if (center_state.load() != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in READY state");
        return {RetCode::ESTATE, "AudioCenter not in READY state"};
    }

    if (vol > 100)
    {
        AUDIO_ERROR_PRINT("Invalid volume level: %u, volume should be between 0 and 100", vol);
        return {RetCode::EPARAM, "Invalid volume level"};
    }

    AUDIO_INFO_PRINT("Setting player global volume to %u", vol);
    auto ret = player->set_volume(vol);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to set player volume: %s", ret.what());
    }
    return ret;
}