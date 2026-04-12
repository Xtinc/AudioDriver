#include "audio_config.h"
#include "audio_interface.h"
#include "audio_monitor.h"
#include "audio_network.h"
#include "audio_stream.h"
#include <atomic>
#include <iomanip>

static std::atomic<bool> s_audio_debug_enabled{false};

bool audio_is_debug_enabled()
{
    return s_audio_debug_enabled.load(std::memory_order_relaxed);
}

static void default_stream_error_callback(unsigned char token, std::string message, void *user_ptr)
{
    (void)user_ptr; // Unused in default callback
    AUDIO_ERROR_PRINT("Stream error on token %u: %s", token, message.c_str());
}

// AudioCenter
AudioCenter::AudioCenter(bool enable_network, const std::string &local_ip, unsigned short port)
    : center_state(State::INIT), stream_error_cb(default_stream_error_callback), stream_error_cb_ptr(nullptr)
{
    config = std::make_unique<INIReader>("audiocenter.ini");
    s_audio_debug_enabled.store((*config)["Log.DebugPrint"].cast<bool>(false), std::memory_order_relaxed);
    monitor = std::make_unique<AudioMonitor>(BG_SERVICE);
    player = std::make_shared<AudioPlayer>(USER_MAX_AUDIO_TOKEN);
    if (enable_network)
    {
        net_mgr = std::make_shared<NetWorker>(BG_SERVICE, port, local_ip);
    }
}

AudioCenter::~AudioCenter()
{
    // Stop audio processing first
    if (center_state.load() == State::READY)
    {
        auto ret = stop();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to stop AudioCenter: %s", ret.what());
        }
    }

    // Unregister monitor callback
    monitor->UnregisterCallback();
}

RetCode AudioCenter::create(IToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                            unsigned int ch, StreamFlags flags, AudioPriority priority)
{
    AUDIO_INFO_PRINT("Creating %s input stream with token %u, priority %u", name.first.c_str(), token.tok,
                     enum2val(priority));

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

    bool enable_network = has_flag(flags, StreamFlags::Network);

    ResetOrd reset_order;
    if (has_flag(flags, StreamFlags::ResetHard))
    {
        reset_order = ResetOrd::RESET_HARD;
    }
    else if (has_flag(flags, StreamFlags::ResetSoft))
    {
        reset_order = ResetOrd::RESET_SOFT;
    }
    else
    {
        reset_order = ResetOrd::RESET_NONE;
    }

    auto ias_ptr = std::make_shared<IAStream>(token.tok, name, enum2val(ps), enum2val(bw), ch, reset_order, priority);

    if (!ias_ptr->available())
    {
        AUDIO_ERROR_PRINT("Failed to create input stream %u, device not available", token.tok);
        return RetCode::EOPEN;
    }

    ias_map[token.tok] = std::move(ias_ptr);

    if (enable_network && net_mgr)
    {
        AudioCodecType codec = has_flag(flags, StreamFlags::CodecOPUS) ? AudioCodecType::OPUS : AudioCodecType::PCM;
        auto ret = ias_map[token.tok]->initialize_network(net_mgr, codec);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to initialize network for input stream %u: %s", token.tok, ret.what());
            return ret;
        }
        AUDIO_INFO_PRINT("Network initialized for input stream %u with codec: %s", token.tok,
                         codec == AudioCodecType::OPUS ? "OPUS" : "PCM");
    }

    return RetCode::OK;
}

RetCode AudioCenter::create(IToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                            unsigned int dev_ch, const AudioChannelMap &imap, StreamFlags flags, AudioPriority priority)
{
    AUDIO_INFO_PRINT("Creating %s input stream with token %u and channel mapping, priority %u", name.first.c_str(),
                     token.tok, enum2val(priority));

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
        AUDIO_ERROR_PRINT(
            "Invalid device channel: %u, when channel map is specified, at least two channels are necessary", dev_ch);
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

    if (has_flag(flags, StreamFlags::ResetHard) || has_flag(flags, StreamFlags::ResetSoft))
    {
        AUDIO_ERROR_PRINT("Reset flag not supported for input stream with channel mapping (token %u), ignoring",
                          token.tok);
    }

    bool enable_network = has_flag(flags, StreamFlags::Network);

    auto ias_ptr = std::make_shared<IAStream>(token.tok, name, enum2val(ps), enum2val(bw), dev_ch, imap, priority);

    if (!ias_ptr->available())
    {
        AUDIO_ERROR_PRINT("Failed to create input stream %u, device not available", token.tok);
        return RetCode::EOPEN;
    }

    ias_map[token.tok] = std::move(ias_ptr);

    if (enable_network && net_mgr)
    {
        AudioCodecType codec = has_flag(flags, StreamFlags::CodecOPUS) ? AudioCodecType::OPUS : AudioCodecType::PCM;
        auto ret = ias_map[token.tok]->initialize_network(net_mgr, codec);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to initialize network for input stream %u: %s", token.tok, ret.what());
            return ret;
        }
        AUDIO_INFO_PRINT("Network initialized for input stream %u with codec: %s", token.tok,
                         codec == AudioCodecType::OPUS ? "OPUS" : "PCM");
    }

    return RetCode::OK;
}

RetCode AudioCenter::create(OToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                            unsigned int ch, StreamFlags flags)
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

    bool enable_network = has_flag(flags, StreamFlags::Network);

    ResetOrd reset_order;
    if (has_flag(flags, StreamFlags::ResetHard))
    {
        reset_order = ResetOrd::RESET_HARD;
    }
    else if (has_flag(flags, StreamFlags::ResetSoft))
    {
        reset_order = ResetOrd::RESET_SOFT;
    }
    else
    {
        reset_order = ResetOrd::RESET_NONE;
    }

    auto oas_ptr = std::make_shared<OAStream>(token.tok, name, enum2val(ps), enum2val(bw), ch, reset_order);

    if (!oas_ptr->available())
    {
        AUDIO_ERROR_PRINT("Failed to create output stream %u, device not available", token.tok);
        return RetCode::EOPEN;
    }

    oas_map[token.tok] = std::move(oas_ptr);

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
                            unsigned int dev_ch, const AudioChannelMap &omap, StreamFlags flags)
{
    AUDIO_INFO_PRINT("Creating %s output stream with token %u and channel mapping", name.first.c_str(), token.tok);

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
        AUDIO_ERROR_PRINT(
            "Invalid device channel: %u, when channel map is specified, at least two channels are necessary", dev_ch);
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

    if (has_flag(flags, StreamFlags::ResetHard) || has_flag(flags, StreamFlags::ResetSoft))
    {
        AUDIO_ERROR_PRINT("Reset flag not supported for output stream with channel mapping (token %u), ignoring",
                          token.tok);
    }

    bool enable_network = has_flag(flags, StreamFlags::Network);

    auto oas_ptr =
        std::make_shared<OAStream>(token.tok, name, enum2val(ps), enum2val(bw), dev_ch, ResetOrd::RESET_NONE, omap);

    if (!oas_ptr->available())
    {
        AUDIO_ERROR_PRINT("Failed to create output stream %u, device not available", token.tok);
        return RetCode::EOPEN;
    }

    oas_map[token.tok] = std::move(oas_ptr);

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

RetCode AudioCenter::create(IToken itoken, OToken otoken, StreamFlags flags, AudioPriority priority)
{
    AUDIO_INFO_PRINT("Creating linked streams with tokens %u -> %u, priority %u", itoken.tok, otoken.tok,
                     enum2val(priority));

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
        AUDIO_ERROR_PRINT("Output token doesn't exist: %u", otoken.tok);
        return RetCode::EPARAM;
    }

    if (!oas->second->available())
    {
        AUDIO_ERROR_PRINT("Output stream %u not available", otoken.tok);
        return RetCode::EPARAM;
    }

    if (ias_map.find(itoken.tok) != ias_map.end())
    {
        AUDIO_ERROR_PRINT("Input token already exists: %u", itoken.tok);
        return RetCode::EPARAM;
    }

    try
    {
        std::shared_ptr<IAStream> new_stream;
        const auto &oas_ptr = oas->second;
        if (oas_ptr->omap == DEFAULT_MONO_MAP)
        {
            new_stream = std::make_shared<IAStream>(itoken.tok, AudioDeviceName("null_dummy", 0), oas_ptr->ti,
                                                    enum2val(AudioBandWidth::Full), 1, DEFAULT_MONO_MAP, priority);
        }
        else if (oas_ptr->omap == DEFAULT_DUAL_MAP)
        {
            new_stream = std::make_shared<IAStream>(itoken.tok, AudioDeviceName("null_dummy", 0), oas_ptr->ti,
                                                    enum2val(AudioBandWidth::Full), 2, DEFAULT_DUAL_MAP, priority);
        }
        else
        {
            new_stream = std::make_shared<IAStream>(itoken.tok, AudioDeviceName("null_dummy", 0), oas_ptr->ti,
                                                    enum2val(AudioBandWidth::Full), 99, oas_ptr->omap, priority);
        }

        if (has_flag(flags, StreamFlags::Network) && net_mgr)
        {
            auto ret = new_stream->initialize_network(net_mgr, AudioCodecType::OPUS);
            if (!ret)
            {
                AUDIO_ERROR_PRINT("Failed to initialize network for input stream %u: %s", itoken.tok, ret.what());
                return ret;
            }
            AUDIO_DEBUG_PRINT("Network initialized for input stream %u", itoken.tok);
        }

        oas->second->register_listener(new_stream);
        ias_map[itoken.tok] = std::move(new_stream);
        return RetCode::OK;
    }
    catch (const std::bad_alloc &e)
    {
        AUDIO_ERROR_PRINT("Memory allocation failed for input stream %u: %s", itoken.tok, e.what());
        return RetCode::FAILED;
    }
    catch (const std::exception &e)
    {
        AUDIO_ERROR_PRINT("Exception occurred while creating input stream %u: %s", itoken.tok, e.what());
        return RetCode::EXCEPTION;
    }
}

RetCode AudioCenter::prepare(bool enable_usb_detection)
{
    State expected = State::INIT;
    if (!center_state.compare_exchange_strong(expected, State::CONNECTING))
    {
        AUDIO_ERROR_PRINT("AudioCenter not in INIT state");
        return {RetCode::ESTATE, "AudioCenter not in INIT state"};
    }

    if (enable_usb_detection)
    {
        std::string default_usb_in = "null_iusb";
        std::string default_usb_out = "null_ousb";

        auto idev_cfg_name = (*config)["HotPlug.InputDeviceName"].cast<std::string>("");
        if (monitor->DeviceExists(idev_cfg_name))
        {
            default_usb_in = idev_cfg_name;
            AUDIO_INFO_PRINT("Default input device found: %s", default_usb_in.c_str());
        }

        auto odev_cfg_name = (*config)["HotPlug.OutputDeviceName"].cast<std::string>("");
        if (monitor->DeviceExists(odev_cfg_name))
        {
            default_usb_out = odev_cfg_name;
            AUDIO_INFO_PRINT("Default output device found: %s", default_usb_out.c_str());
        }

        ias_map.emplace(USR_DUMMY_IN.tok,
                        std::make_shared<IAStream>(USR_DUMMY_IN.tok, AudioDeviceName(default_usb_in, 0),
                                                   enum2val(AudioPeriodSize::INR_20MS), enum2val(AudioBandWidth::Full),
                                                   2, ResetOrd::RESET_NONE, AudioPriority::MEDIUM));
        oas_map.emplace(USR_DUMMY_OUT.tok,
                        std::make_shared<OAStream>(USR_DUMMY_OUT.tok, AudioDeviceName(default_usb_out, 0),
                                                   enum2val(AudioPeriodSize::INR_20MS), enum2val(AudioBandWidth::Full),
                                                   2, ResetOrd::RESET_NONE));
        if (net_mgr)
        {
            ias_map[USR_DUMMY_IN.tok]->initialize_network(net_mgr, AudioCodecType::OPUS);
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

            switch (event)
            {
            case AudioDeviceEvent::Added:
                AUDIO_INFO_PRINT("New device: %s", info.name.c_str());
                if (info.type == AudioDeviceType::Capture)
                {
                    ias->second->restart({info.id, 0});
                    (*config)["HotPlug.InputDeviceID"] = info.id;
                    (*config)["HotPlug.InputDeviceName"] = info.name + ",0";
                }
                else if (info.type == AudioDeviceType::Playback)
                {
                    oas->second->restart({info.id, 0});
                    (*config)["HotPlug.OutputDeviceID"] = info.id;
                    (*config)["HotPlug.OutputDeviceName"] = info.name + ",0";
                }
                else
                {
                    ias->second->restart({info.id, 0});
                    oas->second->restart({info.id, 0});
                    (*config)["HotPlug.InputDeviceID"] = info.id;
                    (*config)["HotPlug.InputDeviceName"] = info.name + ",0";
                    (*config)["HotPlug.OutputDeviceID"] = info.id;
                    (*config)["HotPlug.OutputDeviceName"] = info.name + ",0";
                }
                config->save();
                break;
            case AudioDeviceEvent::Removed:
                AUDIO_INFO_PRINT("Del device: %s", info.name.c_str());
                if (ias->second->name().first == info.id)
                {
                    ias->second->restart({"null_usb", 0});
                }
                if (oas->second->name().first == info.id)
                {
                    oas->second->restart({"null_usb", 0});
                }
                break;
            default:
                break;
            }
        });
    }

    for (const auto &e : config->get_connections())
    {
        auto ret = connect(IToken(e.itoken), OToken(e.token), e.ip, e.port);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Connection config: connect %u->%u failed: %s", e.itoken, e.token, ret.what());
        }
    }

    AUDIO_DEBUG_PRINT("AudioCenter successfully prepared - transitioning from INIT to CONNECTING, %s USB detection",
                      enable_usb_detection ? "with" : "without");
    return RetCode::OK;
}

void AudioCenter::register_error_callback(StreamErrorCallback cb, void *ptr)
{
    if (center_state.load() != State::INIT)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in INIT state, cannot register error callback");
        return;
    }

    stream_error_cb = cb;
    stream_error_cb_ptr = ptr;
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

        auto ret = net_mgr->add_destination(itoken.tok, otoken.tok, ip, port);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to add network destination %s:%u for tokens %u -> %u: %s", ip.c_str(), port,
                              itoken.tok, otoken.tok, ret.what());
        }
        else
        {
            AUDIO_INFO_PRINT("Successfully added network destination %s:%u for tokens %u -> %u", ip.c_str(), port,
                             itoken.tok, otoken.tok);
        }

        return ret;
    }

    auto oas = oas_map.find(otoken.tok);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    auto ret = ias->second->connect(oas->second);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to connect stream %u -> %u: %s", itoken.tok, otoken.tok, ret.what());
    }
    else
    {
        AUDIO_INFO_PRINT("Successfully connected stream %u -> %u", itoken.tok, otoken.tok);
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
        else
        {
            AUDIO_INFO_PRINT("Successfully removed network destination %s for tokens %u->%u", ip.c_str(), itoken.tok,
                             otoken.tok);
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
    else
    {
        AUDIO_INFO_PRINT("Successfully disconnected stream %u -> %u", itoken.tok, otoken.tok);
    }

    return ret;
}

RetCode AudioCenter::clear(AudioToken token)
{
    State current = center_state.load();
    if (current != State::CONNECTING && current != State::READY)
    {
        AUDIO_ERROR_PRINT("AudioCenter not in CONNECTING or READY state");
        return {RetCode::ESTATE, "AudioCenter not in CONNECTING or READY state"};
    }

    if (!token)
    {
        AUDIO_ERROR_PRINT("Invalid token: %u", token.tok);
        return {RetCode::EPARAM, "Invalid token"};
    }

    // Try as input stream first
    auto ias = ias_map.find(token.tok);
    if (ias != ias_map.end())
    {
        AUDIO_INFO_PRINT("Clearing all connections for input stream %u", token.tok);
        return ias->second->clear_all_connections();
    }

    // Output streams don't maintain connection lists, so just log
    auto oas = oas_map.find(token.tok);
    if (oas != oas_map.end())
    {
        AUDIO_INFO_PRINT("Output stream %u has no connections to clear (connections are managed by input streams)",
                         token.tok);
        return {RetCode::OK, "Output stream has no connections to clear"};
    }

    AUDIO_ERROR_PRINT("Token not found: %u", token.tok);
    return {RetCode::EPARAM, "Token not found"};
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
    AUDIO_DEBUG_PRINT("Callback registered for input stream %u, mode: %d", token.tok, static_cast<int>(mode));
    return RetCode::OK;
}

RetCode AudioCenter::register_filter(IToken token, const std::vector<AudioChannelMap> &channel_maps,
                                     size_t filter_length, float step_size)
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

    it->second->register_filter(std::make_unique<LMSFilterBank>(channel_maps, filter_length, step_size));

    std::ostringstream oss;
    for (const auto &map : channel_maps)
    {
        oss << "\n[ " << map[0] << " <- " << map[1] << " ]";
    }

    AUDIO_DEBUG_PRINT("LMS filter bank registered for input stream %u:%s", token.tok, oss.str().c_str());
    return RetCode::OK;
}

RetCode AudioCenter::direct_push_pcm(IToken itoken, OToken otoken, unsigned int chan, unsigned int frames,
                                     unsigned int sample_rate, const int16_t *data, AudioPriority priority)
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

    return oas->second->direct_push(chan, frames, sample_rate, data, SourceUUID{0, 0, itoken.tok}, priority);
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
        pair.second->set_error_callback(stream_error_cb, stream_error_cb_ptr);
        auto ret = pair.second->start();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to start OAStream: %u, %s", pair.first, ret.what());
        }
    }

    for (const auto &pair : ias_map)
    {
        pair.second->set_error_callback(stream_error_cb, stream_error_cb_ptr);
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

    if ((*config)["Report.Latency"].cast<bool>(false))
    {
        auto interval = (*config)["Report.LatencyInterval"].cast<int>(60);
        if (interval >= 10)
        {
            schedule_latency_query(interval);
        }
    }

    AUDIO_DEBUG_PRINT("AudioCenter successfully started - transitioned to READY state");
    return RetCode::OK;
}

void AudioCenter::schedule_latency_query(int interval_sec)
{
    auto timer = std::make_shared<asio::steady_timer>(BG_SERVICE);
    timer->expires_after(std::chrono::seconds(interval_sec));
    timer->async_wait([this, timer, interval_sec](const asio::error_code &ec) {
        if (ec || center_state.load() != State::READY)
        {
            return;
        }

        for (auto &entry : ias_map)
        {
            if (entry.second)
            {
                entry.second->query_latency();
            }
        }
        for (auto &entry : oas_map)
        {
            if (entry.second)
            {
                entry.second->query_latency();
            }
        }

        schedule_latency_query(interval_sec);
    });
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

RetCode AudioCenter::pause(AudioToken token)
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
        ias->second->pause();
        AUDIO_INFO_PRINT("Input stream %u paused", token.tok);
        return RetCode::OK;
    }

    auto oas = oas_map.find(token.tok);
    if (oas != oas_map.end())
    {
        oas->second->pause();
        AUDIO_INFO_PRINT("Output stream %u paused", token.tok);
        return RetCode::OK;
    }

    AUDIO_INFO_PRINT("Token not found: %u", token.tok);
    return {RetCode::FAILED, "Token not found"};
}

RetCode AudioCenter::resume(AudioToken token)
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
        ias->second->resume();
        AUDIO_INFO_PRINT("Input stream %u resumed", token.tok);
        return RetCode::OK;
    }

    auto oas = oas_map.find(token.tok);
    if (oas != oas_map.end())
    {
        oas->second->resume();
        AUDIO_INFO_PRINT("Output stream %u resumed", token.tok);
        return RetCode::OK;
    }

    AUDIO_INFO_PRINT("Token not found: %u", token.tok);
    return {RetCode::FAILED, "Token not found"};
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
        auto ret = ias->second->set_volume(vol);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to set volume for input stream %u: %s", token.tok, ret.what());
        }
        AUDIO_INFO_PRINT("Token %u volume set to %u", token.tok, vol);
        return ret;
    }

    auto oas = oas_map.find(token.tok);
    if (oas != oas_map.end())
    {
        auto ret = oas->second->set_volume(vol);
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to set volume for input stream %u: %s", token.tok, ret.what());
        }
        AUDIO_INFO_PRINT("Token %u volume set to %u", token.tok, vol);
        return ret;
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

RetCode AudioCenter::play(const std::string &name, int cycles, OToken otoken, AudioPriority priority)
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

    auto oas = oas_map.find(otoken.tok);
    if (oas == oas_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid output token: %u", otoken.tok);
        return {RetCode::EPARAM, "Invalid output token"};
    }

    AUDIO_INFO_PRINT("Try to play file %s -> %u", name.c_str(), otoken.tok);
    auto ret = player->play(name, cycles, oas->second, priority);
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
    if (ret != RetCode::OK && ret != RetCode::NOACTION)
    {
        AUDIO_ERROR_PRINT("Failed to stop file %s: %s", path.c_str(), ret.what());
        return ret;
    }
    return RetCode::OK;
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

    auto ret = player->set_volume(vol);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to set player volume: %s", ret.what());
    }
    AUDIO_INFO_PRINT("Player volume set to %u", vol);
    return ret;
}
