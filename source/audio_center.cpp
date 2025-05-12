#include "audio_interface.h"
#include "audio_monitor.h"
#include "audio_network.h"
#include "audio_stream.h"
#include <iomanip>

static void print_merge_label(const std::vector<uint64_t> &vheader, const std::vector<uint64_t> &hheader,
                              const std::vector<InfoLabel> &matrix, size_t rows, size_t cols)
{
    std::ostringstream oss;
    constexpr int width = 16;

    oss << "+" << std::string(width, '-') << "+";
    for (size_t i = 0; i < cols; i++)
    {
        oss << std::string(width, '-') << "+";
    }
    oss << "\n|    TOKEN@IP    |";

    for (size_t i = 0; i < cols; i++)
    {
        uint8_t token = InfoLabel::extract_token(hheader[i]);
        uint32_t ip = InfoLabel::extract_ip(hheader[i]);
        char buf[20];
        snprintf(buf, sizeof(buf), "%03d@0x%08X", token, ip);
        oss << " " << buf << " |";
    }
    oss << "\n";
    oss << "+" << std::string(width, '-') << "+";
    for (size_t i = 0; i < cols; i++)
    {
        oss << std::string(width, '-') << "+";
    }
    oss << "\n";

    for (size_t i = 0; i < rows; i++)
    {
        uint8_t vtoken = InfoLabel::extract_token(vheader[i]);
        uint32_t vip = InfoLabel::extract_ip(vheader[i]);
        char row_buf[20];
        snprintf(row_buf, sizeof(row_buf), "%03d@0x%08X", vtoken, vip);
        oss << "| " << row_buf << " |";

        for (size_t j = 0; j < cols; j++)
        {
            const auto &ele = matrix[i * cols + j];
            if (ele == InfoLabel{})
            {
                oss << " " << std::setw(width) << "|";
            }
            else
            {
                char status[20];
                snprintf(status, sizeof(status), "      %c%c%c       |", ele.ias_muted() ? 'x' : '-',
                         ele.connected() ? 'C' : '-', ele.oas_muted() ? 'x' : '-');
                oss << status;
            }
        }
        oss << "\n";
    }

    oss << "+" << std::string(width, '-') << "+";
    for (size_t i = 0; i < cols; i++)
    {
        oss << std::string(width, '-') << "+";
    }

    AUDIO_INFO_PRINT("\n%s", oss.str().c_str());
}

static void merge_label(std::vector<InfoLabel> &ias_label, std::vector<InfoLabel> &oas_label)
{
    if (ias_label.empty() && oas_label.empty())
    {
        return;
    }

    std::sort(ias_label.begin(), ias_label.end());
    std::sort(oas_label.begin(), oas_label.end());
    std::vector<InfoLabel> merged_labels;
    merged_labels.reserve(ias_label.size() + oas_label.size());

    size_t i = 0;
    size_t j = 0;

    while (i < ias_label.size() && j < oas_label.size())
    {
        if (ias_label[i] < oas_label[j])
        {
            merged_labels.push_back(ias_label[i++]);
        }
        else if (ias_label[i] > oas_label[j])
        {
            merged_labels.push_back(oas_label[j++]);
        }
        else
        {
            auto temp = ias_label[i++];
            temp.set_connected(oas_label[j].connected());
            temp.set_oas_muted(oas_label[j].oas_muted());
            merged_labels.push_back(temp);
            j++;
        }
    }

    while (i < ias_label.size())
    {
        merged_labels.push_back(ias_label[i++]);
    }

    while (j < oas_label.size())
    {
        merged_labels.push_back(oas_label[j++]);
    }

    std::vector<uint64_t> ias_tokens;
    std::vector<uint64_t> oas_tokens;
    ias_label.reserve(merged_labels.size());
    oas_label.reserve(merged_labels.size());

    uint64_t last_label = 0;
    std::sort(merged_labels.begin(), merged_labels.end(),
              [](const InfoLabel &a, const InfoLabel &b) { return a.ias_composite < b.ias_composite; });

    for (const auto &label : merged_labels)
    {
        auto itok = label.ias_composite;
        if (itok != last_label)
        {
            ias_tokens.push_back(itok);
            last_label = itok;
        }
    }

    last_label = 0;
    std::sort(merged_labels.begin(), merged_labels.end(),
              [](const InfoLabel &a, const InfoLabel &b) { return a.oas_composite < b.oas_composite; });

    for (const auto &label : merged_labels)
    {
        auto otok = label.oas_composite;
        if (otok != last_label)
        {
            oas_tokens.push_back(otok);
            last_label = otok;
        }
    }

    auto ias_cnt = ias_tokens.size();
    auto oas_cnt = oas_tokens.size();

    std::vector<InfoLabel> merged_matrix(ias_cnt * oas_cnt, InfoLabel{});
    std::sort(merged_labels.begin(), merged_labels.end());

    i = 0;
    j = 0;
    last_label = merged_labels[0].ias_composite;

    for (auto ele : merged_labels)
    {
        if (i == ias_cnt)
        {
            break;
        }

        if (ele.ias_composite != last_label)
        {
            last_label = ele.ias_composite;
            i++;
            j = 0;
        }

        while (j < oas_cnt)
        {
            if (ele.oas_composite == oas_tokens[j])
            {
                merged_matrix[i * oas_cnt + j] = ele;
            }
            j++;
        }
    }

    print_merge_label(ias_tokens, oas_tokens, merged_matrix, ias_cnt, oas_cnt);
}

// AudioCenter
AudioCenter::AudioCenter(bool enable_network, unsigned short port) : center_state(State::INIT)
{
    monitor = std::make_unique<AudioMonitor>(BG_SERVICE);
    player = std::make_shared<AudioPlayer>(USER_MAX_AUDIO_TOKEN);
    if (enable_network)
    {
        net_mgr = std::make_shared<NetWorker>(BG_SERVICE, port);
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

    if (enable_network)
    {
        ias_map[token.tok]->initialize_network(net_mgr);
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

    if (enable_network)
    {
        ias_map[token.tok]->initialize_network(net_mgr);
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

    if (enable_network)
    {
        oas_map[token.tok]->initialize_network(net_mgr);
        AUDIO_DEBUG_PRINT("Network initialized for output stream %u", token.tok);
    }

    return RetCode::OK;
}

RetCode AudioCenter::create(OToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                            unsigned int dev_ch, const AudioChannelMap &omap, bool enable_network)
{
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

    if (enable_network)
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

    if (oas->second->omap != DEFAULT_MONO_MAP && oas->second->omap != DEFAULT_DUAL_MAP)
    {
        ias_map[itoken.tok] =
            std::make_shared<IAStream>(itoken.tok, AudioDeviceName("virt", 0), enum2val(AudioPeriodSize::INR_20MS),
                                       enum2val(AudioBandWidth::Full), 99, oas->second->omap);
    }
    else
    {
        ias_map[itoken.tok] =
            std::make_shared<IAStream>(itoken.tok, AudioDeviceName("virt", 0), enum2val(AudioPeriodSize::INR_20MS),
                                       enum2val(AudioBandWidth::Full), 2);
    }

    if (enable_network)
    {
        ias_map[itoken.tok]->initialize_network(net_mgr);
        AUDIO_DEBUG_PRINT("Network initialized for input stream %u", itoken.tok);
    }

    oas->second->register_listener(ias_map[itoken.tok]);

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

    ias_map.emplace(USR_DUMMY_IN.tok,
                    std::make_shared<IAStream>(USR_DUMMY_IN.tok, AudioDeviceName("virt", 0),
                                               enum2val(AudioPeriodSize::INR_20MS), enum2val(AudioBandWidth::Full), 2));
    oas_map.emplace(USR_DUMMY_OUT.tok,
                    std::make_shared<OAStream>(USR_DUMMY_OUT.tok, AudioDeviceName("virt", 0),
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

        switch (event)
        {
        case AudioDeviceEvent::Added:
            AUDIO_INFO_PRINT("New device: %s", info.name.c_str());
            if (info.type == AudioDeviceType::Capture)
            {
                ias->second->restart({info.id, 0});
            }
            else if (info.type == AudioDeviceType::Playback)
            {
                oas->second->restart({info.id, 0});
            }
            else
            {
                ias->second->restart({info.id, 0});
                oas->second->restart({info.id, 0});
            }
            break;
        case AudioDeviceEvent::Removed:
            AUDIO_INFO_PRINT("Del device: %s", info.name.c_str());
            if (ias->second->name().first == info.id)
            {
                ias->second->restart({"virt", 0});
            }
            if (oas->second->name().first == info.id)
            {
                oas->second->restart({"virt", 0});
            }
            break;
        default:
            break;
        }
    });
    AUDIO_DEBUG_PRINT("AudioCenter successfully prepared - transitioning from INIT to CONNECTING");
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

    // Find the stream
    auto it = ias_map.find(token.tok);
    if (it == ias_map.end())
    {
        AUDIO_ERROR_PRINT("Invalid input token: %u", token.tok);
        return {RetCode::EPARAM, "Invalid input token"};
    }

    it->second->register_callback(cb, ptr);
    AUDIO_DEBUG_PRINT("Callback registered for input stream %u", token.tok);
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

    return oas->second->direct_push(itoken.tok, chan, frames, sample_rate, data);
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

    schedule_report_timer();
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

void AudioCenter::schedule_report_timer() const
{
    auto timer = std::make_shared<asio::steady_timer>(BG_SERVICE, std::chrono::seconds(87));
    timer->async_wait([this](const asio::error_code &error) {
        if (!error && center_state.load() == State::READY)
        {
            report_connections();
            schedule_report_timer();
        }
    });
}

void AudioCenter::report_connections() const
{
    std::vector<InfoLabel> ias_label;
    std::vector<InfoLabel> oas_label;
    ias_label.reserve(6);
    oas_label.reserve(6);

    for (const auto &pair : ias_map)
    {
        pair.second->report_conns(ias_label);
    }

    for (const auto &pair : oas_map)
    {
        pair.second->report_conns(oas_label);
    }

    if (net_mgr)
    {
        net_mgr->report_conns(ias_label);
    }

    merge_label(ias_label, oas_label);
}
