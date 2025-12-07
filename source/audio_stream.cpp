#include "audio_stream.h"
#include <cmath>

#if WINDOWS_OS_ENVIRONMENT
#include <combaseapi.h>
#include <timeapi.h>
// CoInitializeEx is not thread-safe
static std::once_flag coinit_flag;
#endif
constexpr unsigned int AUDIO_MAX_RESET_INTERVAL = 30;
constexpr unsigned int AUDIO_MAX_COCURRECY_WORKER = 4;
constexpr unsigned int SESSION_IDLE_TIMEOUT = 1000;

inline float volume2gain(unsigned int vol)
{
    // 0% -> -80dB(mute), 1% -> -40dB, 100% -> +6dB
    if (vol == 0)
    {
        return -80.0f;
    }

    // gain_db = 20 * log10(vol/100)
    float normalized_vol = static_cast<float>(vol) / 100.0f;

    if (vol <= 10)
    {
        // Low volume range: 1%-10% maps to -40dB to -20dB
        return -40.0f + 20.0f * (normalized_vol * 10.0f - 1.0f) / 9.0f;
    }
    else
    {
        // High volume range: 11%-100% maps to -20dB to +6dB using logarithmic curve with offset
        // Base logarithmic curve + offset to make 100% correspond to +6dB
        return 20.0f * std::log10(normalized_vol) + 6.0f;
    }
}

static bool check_wave_file_name(const std::string &dev_name)
{
    return dev_name.size() >= 4 && dev_name.compare(dev_name.size() - 4, 4, ".wav") == 0;
}

// BackgroundService
BackgroundService &BackgroundService::instance()
{
    static BackgroundService instance;
    return instance;
}

BackgroundService::BackgroundService() : work_guard(asio::make_work_guard(io_context))
{
#if WINDOWS_OS_ENVIRONMENT
    std::call_once(coinit_flag, []() { CoInitializeEx(nullptr, COINIT_MULTITHREADED); });
    TIMECAPS tc;
    if (timeGetDevCaps(&tc, sizeof(TIMECAPS)) == TIMERR_NOERROR)
    {
        high_timer_resolution = (std::max)(tc.wPeriodMin, (std::min)(static_cast<UINT>(1), tc.wPeriodMax));

        if (timeBeginPeriod(high_timer_resolution) == TIMERR_NOERROR)
        {
            active_high_res_timer = true;
            AUDIO_INFO_PRINT("System timer resolution set to %u ms", high_timer_resolution);
        }
    }
#endif

    for (size_t i = 0; i < AUDIO_MAX_COCURRECY_WORKER; ++i)
    {
        io_thds.emplace_back([this]() {
#if WINDOWS_OS_ENVIRONMENT
            HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
            bool com_initialized = SUCCEEDED(hr);
#endif
            set_current_thread_scheduler_policy();
            io_context.run();
#if WINDOWS_OS_ENVIRONMENT
            if (com_initialized)
            {
                CoUninitialize();
            }
#endif
        });
    }

    AUDIO_INFO_PRINT("AudioDriver compiled on %s at %s", __DATE__, __TIME__);
}

BackgroundService::~BackgroundService()
{
    work_guard.reset();
    io_context.stop();

    for (auto &thread : io_thds)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }

#if WINDOWS_OS_ENVIRONMENT
    if (active_high_res_timer)
    {
        timeEndPeriod(high_timer_resolution);
        AUDIO_INFO_PRINT("System timer resolution restored to default");
    }
#endif
}

asio::io_context &BackgroundService::context()
{
    return io_context;
}

// OAStream
OAStream::OAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs,
                   unsigned int _ch, bool auto_reset, AudioChannelMap _omap)
    : token(_token), ti(_ti), enable_reset(auto_reset),
      omap(_omap == AudioChannelMap{} ? (_ch == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP) : _omap), fs(_fs),
      ps(fs * ti / 1000), ch(_ch), usr_name(_name), oas_ready(false), exec_timer(BG_SERVICE),
      exec_strand(asio::make_strand(BG_SERVICE)), reset_timer(BG_SERVICE), reset_strand(asio::make_strand(BG_SERVICE)),
      volume(50), muted(false)
{
    DBG_ASSERT_LT(omap[0], ch);
    DBG_ASSERT_LT(omap[1], ch);

    auto ret = create_device(_name);
    if (ret)
    {
        oas_ready = true;
    }
    else
    {
        AUDIO_ERROR_PRINT("Failed to create device: %s", ret.what());
    }
}

RetCode OAStream::initialize_network(const std::shared_ptr<NetWorker> &nw)
{
    if (!nw)
    {
        return {RetCode::FAILED, "Invalid or not ready NetWorker"};
    }

    networker = nw;
    std::weak_ptr<OAStream> weak_self = shared_from_this();
    auto result =
        nw->register_receiver(token, [weak_self](unsigned int channels, unsigned int frames, unsigned int sample_rate,
                                                 const int16_t *data, SourceUUID source_id) {
            if (auto self = weak_self.lock())
            {
                auto ret = self->direct_push(channels, frames, sample_rate, data, source_id);
                if (ret != RetCode::OK)
                {
                    AUDIO_ERROR_PRINT("Failed to push data: %s", ret.what());
                    return;
                }
            }
        });

    if (!result)
    {
        AUDIO_ERROR_PRINT("Failed to register receiver: %s", result.what());
        return {RetCode::FAILED, "Failed to register receiver"};
    }
    return {RetCode::OK, "Network initialized"};
}

OAStream::~OAStream()
{
    (void)stop();

    if (auto np = networker.lock())
    {
        np->unregister_receiver(token);
        AUDIO_INFO_PRINT("Network receiver unregistered for token %u", token);
    }
}

RetCode OAStream::start()
{
    if (!oas_ready)
    {
        return {RetCode::FAILED, "Device not ready"};
    }

    auto ret = odevice->start();
    if (ret)
    {
        execute_loop({}, 0);
    }

    if (enable_reset)
    {
        schedule_auto_reset();
    }

    return ret;
}

RetCode OAStream::stop()
{
    bool expected = true;
    if (!oas_ready.compare_exchange_strong(expected, false))
    {
        return {RetCode::OK, "Device already stopped"};
    }

    exec_timer.cancel();

    if (enable_reset)
    {
        reset_timer.cancel();
    }

    {
        std::lock_guard<std::mutex> grd(session_mtx);
        sessions.clear();
    }

    return odevice->stop();
}

RetCode OAStream::restart(const AudioDeviceName &_name)
{
    stop();

    auto ret = create_device(_name);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to create device: %s", ret.what());
    }

    std::shared_ptr<IAStream> np;
    {
        std::lock_guard<std::mutex> lock(listener_mtx);
        np = listener.lock();
    }

    if (np)
    {
        np->reset2echo(_name, fs, ch);
        np->start();
    }
    oas_ready = true;
    return start();
}

RetCode OAStream::direct_push(unsigned int chan, unsigned int frames, unsigned int sample_rate, const int16_t *data,
                              SourceUUID source_id)
{
    if (!oas_ready)
    {
        return {RetCode::NOACTION, "Device not ready"};
    }

    if (!data)
    {
        return {RetCode::FAILED, "Invalid data pointer"};
    }

    std::lock_guard<std::mutex> grd(session_mtx);
    auto it = sessions.find(source_id);
    if (it == sessions.end())
    {
        unsigned int std_fr = (std::max)(frames, sample_rate * ti / 1000);
        auto imap = chan == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP;
        auto result = sessions.emplace(source_id,
                                       std::make_unique<SessionContext>(sample_rate, chan, fs, ch, std_fr, imap, omap));

        if (!result.second)
        {
            return {RetCode::FAILED, "Failed to create session"};
        }

        it = result.first;
        AUDIO_INFO_PRINT("%u receiver New connection: %u (IP: 0x%08X|0x%08X)", token, source_id.sender_token,
                         source_id.sender_ip, source_id.gateway_ip);
    }

    bool success = it->second->session.store(reinterpret_cast<const char *>(data), frames * chan * sizeof(PCM_TYPE));
    return success ? RetCode::OK : RetCode::NOACTION;
}

bool OAStream::available() const
{
    return oas_ready.load();
}

void OAStream::mute()
{
    muted.store(true);
    AUDIO_INFO_PRINT("Token %u muted", token);
}

void OAStream::unmute()
{
    muted.store(false);
    AUDIO_INFO_PRINT("Token %u unmuted", token);
}

RetCode OAStream::mute(unsigned char itoken, const std::string &ip)
{
    try
    {
        uint32_t ip_address = 0;
        if (!ip.empty())
        {
            asio::ip::address_v4 addr = asio::ip::make_address_v4(ip);
            ip_address = addr.to_uint();
        }

        std::lock_guard<std::mutex> grd(session_mtx);
        int muted_count = 0;

        for (auto &session_pair : sessions)
        {
            if (session_pair.first.sender_token == itoken && session_pair.first.sender_ip == ip_address)
            {
                session_pair.second->enabled = false;
                muted_count++;
            }
        }

        if (muted_count > 0)
        {
            AUDIO_INFO_PRINT("Muted %d sessions with token %u", muted_count, itoken);
            return RetCode::OK;
        }
        else
        {
            AUDIO_INFO_PRINT("No sessions found with token %u to mute", itoken);
            return {RetCode::NOACTION, "Session not found"};
        }
    }
    catch (const std::exception &e)
    {
        AUDIO_ERROR_PRINT("Invalid IP address format: %s - %s", ip.c_str(), e.what());
        return {RetCode::FAILED, "Invalid IP address format"};
    }
}

RetCode OAStream::unmute(unsigned char itoken, const std::string &ip)
{
    try
    {
        uint32_t ip_address = 0;
        if (!ip.empty())
        {
            asio::ip::address_v4 addr = asio::ip::make_address_v4(ip);
            ip_address = addr.to_uint();
        }
        std::lock_guard<std::mutex> grd(session_mtx);
        int unmuted_count = 0;

        for (auto &session_pair : sessions)
        {
            if (session_pair.first.sender_token == itoken && session_pair.first.sender_ip == ip_address)
            {
                session_pair.second->enabled = true;
                unmuted_count++;
            }
        }

        if (unmuted_count > 0)
        {
            AUDIO_INFO_PRINT("Unmuted %d sessions with token %u (all IPs)", unmuted_count, itoken);
            return RetCode::OK;
        }
        else
        {
            AUDIO_INFO_PRINT("No sessions found with token %u to unmute", itoken);
            return {RetCode::NOACTION, "Session not found"};
        }
    }
    catch (const std::exception &e)
    {
        AUDIO_ERROR_PRINT("Invalid IP address format: %s - %s", ip.c_str(), e.what());
        return {RetCode::FAILED, "Invalid IP address format"};
    }
}

RetCode OAStream::set_volume(unsigned int vol)
{
    if (vol > 100)
    {
        AUDIO_ERROR_PRINT("Invalid volume value: %u", vol);
        return {RetCode::EPARAM, "Invalid volume value"};
    }
    volume.store(vol);
    AUDIO_INFO_PRINT("Token %u all volume set to %u, gain: %.2f db", token, vol, volume2gain(vol));
    return RetCode::OK;
}

AudioDeviceName OAStream::name() const
{
    return usr_name;
}

void OAStream::register_listener(const std::shared_ptr<IAStream> &ias)
{
    if (omap != ias->imap)
    {
        AUDIO_ERROR_PRINT("Channel map mismatch: (%u,%u) vs (%u,%u)", omap[0], omap[1], ias->imap[0], ias->imap[1]);
        return;
    }

    if (!ias)
    {
        AUDIO_ERROR_PRINT("Invalid IAStream pointer for token %u", token);
        return;
    }

    auto ret = ias->reset2echo({odevice->hw_name, 0}, fs, ch);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to reset echo: %s", ret.what());
        return;
    }

    std::lock_guard<std::mutex> grd(listener_mtx);
    listener = ias;
    AUDIO_DEBUG_PRINT("Listener registered for token %u", token);
}

void OAStream::unregister_listener()
{
    std::lock_guard<std::mutex> grd(listener_mtx);
    listener.reset();
}

void OAStream::execute_loop(TimePointer tp, unsigned int cnt)
{
    if (!oas_ready)
    {
        return;
    }

    if (cnt == 0)
    {
        tp = std::chrono::steady_clock::now();
    }

    exec_timer.expires_at(tp + std::chrono::milliseconds(ti) * cnt);
    exec_timer.async_wait(asio::bind_executor(exec_strand, [tp, cnt, self = shared_from_this()](asio::error_code ec) {
        if (ec)
        {
            AUDIO_DEBUG_PRINT("Timer error: %s", ec.message().c_str());
            return;
        }

        self->execute_loop(tp, cnt + 1);
        self->process_data();
    }));
}

void OAStream::process_data()
{
    if (!oas_ready)
    {
        return;
    }

    std::memset(databuf.get(), 0, ch * ps * sizeof(PCM_TYPE));

    {
        std::lock_guard<std::mutex> grd(session_mtx);

        for (auto it = sessions.begin(); it != sessions.end();)
        {
            auto &context = it->second;
            unsigned int session_frames = context->sampler.src_fs * ti / 1000;
            unsigned int session_channels = context->sampler.src_ch;
            bool has_data = context->session.load_aside(session_frames * session_channels * sizeof(PCM_TYPE));

            if (!has_data)
            {
                if (context->session.idle_count++ > SESSION_IDLE_TIMEOUT)
                {
                    AUDIO_INFO_PRINT("Removing empty session: %u (IP: 0x%08X|0x%08X)", it->first.sender_token,
                                     it->first.sender_ip, it->first.gateway_ip);
                    it = sessions.erase(it);
                    continue;
                }
                ++it;
                continue;
            }

            context->session.idle_count = 0;

            if (muted || (!context->enabled))
            {
                ++it;
                continue;
            }

            auto *session_source = reinterpret_cast<PCM_TYPE *>(context->session.data());
            InterleavedView<const PCM_TYPE> iview(session_source, session_frames, session_channels);
            InterleavedView<PCM_TYPE> oview(reinterpret_cast<PCM_TYPE *>(mix_buf.get()), ps, ch);

            float session_gain = volume2gain(volume.load());
            auto ret = context->sampler.process(iview, oview, session_gain);
            if (ret != RetCode::OK)
            {
                AUDIO_DEBUG_PRINT("Failed to process data: %s", ret.what());
                ++it;
                continue;
            }

            mix_channels(&oview.data()[0], ch, ps, reinterpret_cast<PCM_TYPE *>(databuf.get()));
            ++it;
        }
    }

    auto ret = odevice->write(databuf.get(), ps * ch * sizeof(PCM_TYPE));
    if (ret == RetCode::ESTATE && enable_reset)
    {
        AUDIO_INFO_PRINT("Device reset detected for token %u", token);
        reset_self();
    }

    std::shared_ptr<IAStream> np;
    {
        std::lock_guard<std::mutex> lock(listener_mtx);
        np = listener.lock();
    }

    if (np)
    {
        (void)np->direct_push(databuf.get(), ps * ch * sizeof(PCM_TYPE));
    }
}

RetCode OAStream::create_device(const AudioDeviceName &_name)
{
    std::unique_ptr<AudioDevice> new_device;
    if (_name.first.substr(0, 4) == "null")
    {
        new_device = make_audio_driver(NULL_OAS, _name, fs, ps, ch, false);
    }
    else if (check_wave_file_name(_name.first))
    {
        new_device = make_audio_driver(WAVE_OAS, _name, fs, ps, ch, false);
    }
    else
    {
        new_device = make_audio_driver(PHSY_OAS, _name, fs, ps, ch, enable_reset);
    }

    if (!new_device)
    {
        return {RetCode::FAILED, "Failed to create audio driver"};
    }

    auto ret = new_device->open();
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to open playback device [%s]: %s", _name.first.c_str(), ret.what());
        return ret;
    }

    unsigned int new_fs = new_device->fs();
    unsigned int new_ch = new_device->ch();
    unsigned int new_ps = new_fs * ti / 1000;

    auto new_databuf = std::make_unique<char[]>(new_ps * new_ch * sizeof(PCM_TYPE));
    auto new_mix_buf = std::make_unique<char[]>(new_ps * new_ch * sizeof(PCM_TYPE));

    {
        std::lock_guard<std::mutex> grd(session_mtx);
        sessions.clear();
        odevice = std::move(new_device);
        fs = new_fs;
        ch = new_ch;
        ps = new_ps;
        databuf = std::move(new_databuf);
        mix_buf = std::move(new_mix_buf);
    }

    return {RetCode::OK, "Device initialized successfully"};
}

void OAStream::schedule_auto_reset()
{
    if (!enable_reset || !oas_ready)
    {
        return;
    }

    constexpr auto reset_interval = std::chrono::minutes(AUDIO_MAX_RESET_INTERVAL);
    reset_timer.expires_from_now(reset_interval);
    reset_timer.async_wait(asio::bind_executor(exec_strand, [self = shared_from_this()](const asio::error_code &ec) {
        if (ec)
        {
            AUDIO_DEBUG_PRINT("OAStream auto reset timer error: %s", ec.message().c_str());
            return;
        }

        AUDIO_INFO_PRINT("Performing scheduled reset for OAStream token %u", self->token);
        self->reset_self();
    }));
}

void OAStream::reset_self()
{
    asio::post(reset_strand, [self = shared_from_this()]() {
        (void)self->stop();
        self->odevice.reset();
        self->odevice = make_audio_driver(PHSY_OAS, self->usr_name, self->fs, self->ps, self->ch, self->enable_reset);

        self->oas_ready = true;
        auto ret = self->start();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to restart OAStream: %s", ret.what());
        }
    });
}

// IAStream
IAStream::IAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs,
                   unsigned int _ch, bool auto_reset)
    : token(_token), ti(_ti), fs(_fs), ps(fs * ti / 1000), ch(_ch), enable_reset(auto_reset), spf_ch(_ch),
      imap(_ch == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP), usr_name(_name), ias_ready(false), exec_timer(BG_SERVICE),
      exec_strand(asio::make_strand(BG_SERVICE)), reset_timer(BG_SERVICE), reset_strand(asio::make_strand(BG_SERVICE)),
      volume(50), muted(false), usr_cb{nullptr, 0, UsrCallBackMode::PROCESSED, nullptr}, cb_timer(BG_SERVICE),
      cb_strand(asio::make_strand(BG_SERVICE))
{
    auto ret = create_device(_name);
    if (ret)
    {
        ias_ready = true;
    }
    else
    {
        AUDIO_ERROR_PRINT("Failed to create device: %s", ret.what());
    }

    dests.reserve(4);
}

IAStream::IAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs,
                   unsigned int dev_ch, AudioChannelMap _imap)
    : token(_token), ti(_ti), fs(_fs), ps(fs * ti / 1000), ch(2), enable_reset(false), spf_ch(dev_ch), imap(_imap),
      usr_name(_name), ias_ready(false), exec_timer(BG_SERVICE), exec_strand(asio::make_strand(BG_SERVICE)),
      reset_timer(BG_SERVICE), reset_strand(asio::make_strand(BG_SERVICE)), volume(50), muted(false),
      usr_cb{nullptr, 0, UsrCallBackMode::PROCESSED, nullptr}, cb_timer(BG_SERVICE),
      cb_strand(asio::make_strand(BG_SERVICE))
{
    DBG_ASSERT_GE(dev_ch, 2);
    DBG_ASSERT_LT(imap[0], dev_ch);
    DBG_ASSERT_LT(imap[1], dev_ch);

    auto ret = create_device(_name);
    if (ret)
    {
        ias_ready = true;
    }
    else
    {
        AUDIO_ERROR_PRINT("Failed to create device: %s", ret.what());
    }

    dests.reserve(4);
}

IAStream::~IAStream()
{
    (void)stop();

    if (auto np = networker.lock())
    {
        np->unregister_sender(token);
        AUDIO_INFO_PRINT("Network sender unregistered for token %u", token);
    }
}

RetCode IAStream::restart(const AudioDeviceName &_name)
{
    if (enable_reset)
    {
        return {RetCode::FAILED, "Auto reset enabled, manual reset not allowed"};
    }

    stop();

    auto ret = create_device(_name);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to create device: %s", ret.what());
    }
    else
    {
        usr_name = _name;
    }

    ias_ready = true;

    return start();
}

RetCode IAStream::reset2echo(const AudioDeviceName &_name, unsigned int _fs, unsigned int _ch)
{
    if (enable_reset)
    {
        return {RetCode::FAILED, "Auto reset enabled, manual reset not allowed"};
    }

    stop();

    auto ret = create_device(_name, _fs, _ch);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to create device: %s", ret.what());
    }

    ias_ready = true;
    return ret;
}

RetCode IAStream::start()
{
    if (!ias_ready)
    {
        return {RetCode::FAILED, "Device not ready"};
    }

    auto ret = idevice->start();
    if (ret)
    {
        execute_loop({}, 0);
    }

    if (enable_reset)
    {
        schedule_auto_reset();
    }

    if (usr_cb.cb)
    {
        schedule_callback();
    }

    return ret;
}

RetCode IAStream::stop()
{
    bool expected = true;
    if (!ias_ready.compare_exchange_strong(expected, false))
    {
        return {RetCode::OK, "Device already stopped"};
    }

    exec_timer.cancel();

    if (enable_reset)
    {
        reset_timer.cancel();
    }

    if (usr_cb.cb)
    {
        cb_timer.cancel();
    }

    return idevice->stop();
}

RetCode IAStream::initialize_network(const std::shared_ptr<NetWorker> &nw, AudioCodecType codec)
{
    if (!nw)
    {
        return {RetCode::FAILED, "Invalid or not ready NetWorker"};
    }

    networker = nw;
    return nw->register_sender(token, ch, fs, codec);
}

RetCode IAStream::connect(const std::shared_ptr<OAStream> &oas)
{
    if (!ias_ready)
    {
        return {RetCode::FAILED, "Device not ready"};
    }

    if (!oas)
    {
        return {RetCode::FAILED, "Invalid output stream"};
    }

    std::lock_guard<std::mutex> grd(dest_mtx);
    if (std::any_of(dests.begin(), dests.end(), [&oas](const std::weak_ptr<OAStream> &wp) { return wp.lock() == oas; }))
    {
        return {RetCode::FAILED, "Already connected to this output stream"};
    }

    dests.emplace_back(oas);
    return {RetCode::OK, "Connection established"};
}

RetCode IAStream::disconnect(const std::shared_ptr<OAStream> &oas)
{
    if (!ias_ready)
    {
        return {RetCode::FAILED, "Device not ready"};
    }

    if (!oas)
    {
        return {RetCode::FAILED, "Invalid output stream"};
    }

    std::lock_guard<std::mutex> grd(dest_mtx);
    dests.erase(std::remove_if(dests.begin(), dests.end(),
                               [oas](const std::weak_ptr<OAStream> &wp) { return wp.lock() == oas; }),
                dests.end());

    return {RetCode::OK, "Connection removed"};
}

RetCode IAStream::direct_push(const char *data, size_t len) const
{
    if (usr_cb.cb && usr_cb.mode == UsrCallBackMode::OBSERVER)
    {
        auto dev_ch = idevice->ch();
        usr_cb.cb(reinterpret_cast<const PCM_TYPE *>(data), dev_ch,
                  static_cast<unsigned int>(len) / (sizeof(PCM_TYPE) * dev_ch), usr_cb.ptr);
    }
    return idevice->write(data, len);
}

bool IAStream::available() const
{
    return ias_ready.load();
}

void IAStream::mute()
{
    muted.store(true);
    AUDIO_INFO_PRINT("Token %u muted", token);
}

void IAStream::unmute()
{
    muted.store(false);
    AUDIO_INFO_PRINT("Token %u unmuted", token);
}

RetCode IAStream::set_volume(unsigned int vol)
{
    if (vol > 100)
    {
        AUDIO_ERROR_PRINT("Invalid volume value: %u", vol);
        return {RetCode::EPARAM, "Invalid volume value"};
    }
    AUDIO_INFO_PRINT("Token %u volume set to %u, gain: %.2f db", token, vol, volume2gain(vol));
    volume.store(vol);
    return RetCode::OK;
}

unsigned int IAStream::get_volume() const
{
    return volume.load();
}

AudioDeviceName IAStream::name() const
{
    return usr_name;
}

void IAStream::register_callback(AudioInputCallBack cb, unsigned int required_frames, UsrCallBackMode mode, void *ptr)
{
    usr_cb.cb = cb;
    usr_cb.req_frs = required_frames;
    usr_cb.mode = mode;
    usr_cb.ptr = ptr;
    session = std::make_unique<SessionData>(required_frames * sizeof(PCM_TYPE), 2, spf_ch);
}

void IAStream::register_filter(filter_ptr &&lms_filter)
{
    if (!lms_filter)
    {
        AUDIO_ERROR_PRINT("Attempting to register null LMS filter for token %u", token);
        return;
    }

    // Validate that device has enough channels for the filter
    if (idevice && idevice->ch() < 2)
    {
        AUDIO_ERROR_PRINT("LMS filter requires at least 2 channels, device has %u channels", idevice->ch());
        return;
    }

    filter_bank = std::move(lms_filter);
}

void IAStream::execute_loop(TimePointer tp, unsigned int cnt)
{
    if (!ias_ready)
    {
        return;
    }

    if (cnt == 0)
    {
        tp = std::chrono::steady_clock::now();
    }

    exec_timer.expires_at(tp + std::chrono::milliseconds(ti) * cnt);
    exec_timer.async_wait(asio::bind_executor(exec_strand, [tp, cnt, self = shared_from_this()](asio::error_code ec) {
        if (ec)
        {
            AUDIO_DEBUG_PRINT("Timer error: %s", ec.message().c_str());
            return;
        }
        self->execute_loop(tp, cnt + 1);
        if (self->process_data() == RetCode::INVSEEK)
        {
            self->ias_ready = false;
        }
    }));
}

RetCode IAStream::process_data()
{
    if (!ias_ready)
    {
        return RetCode::FAILED;
    }

    auto dev_fr = idevice->fs() * ti / 1000;
    auto ret = idevice->read(dev_buf.get(), dev_fr * idevice->ch() * sizeof(PCM_TYPE));

    if (ret == RetCode::ESTATE && enable_reset)
    {
        AUDIO_INFO_PRINT("Device reset detected for token %u", token);
        reset_self();
        return RetCode::OK;
    }

    if (ret != RetCode::OK && ret != RetCode::NOACTION)
    {
        AUDIO_DEBUG_PRINT("Failed to read data: %s", ret.what());
        return ret;
    }

    // raw data from device
    if (usr_cb.cb && usr_cb.mode == UsrCallBackMode::RAW)
    {
        session->store(reinterpret_cast<const char *>(dev_buf.get()), dev_fr * idevice->ch() * sizeof(PCM_TYPE));
    }

    if (muted)
    {
        return RetCode::OK;
    }

    InterleavedView<PCM_TYPE> iview(reinterpret_cast<PCM_TYPE *>(dev_buf.get()), dev_fr, idevice->ch());
    InterleavedView<PCM_TYPE> oview(reinterpret_cast<PCM_TYPE *>(usr_buf.get()), ps, ch);

    if (filter_bank)
    {
        filter_bank->process(iview);
    }

    auto volume_gain = volume2gain(volume.load());
    ret = sampler->process(iview, oview, volume_gain);

    if (ret != RetCode::OK)
    {
        AUDIO_DEBUG_PRINT("Failed to resample data: %s", ret.what());
        return ret;
    }

    PCM_TYPE *src = &oview.data()[0];

    // processed data from device
    if (usr_cb.cb && usr_cb.mode == UsrCallBackMode::PROCESSED)
    {
        session->store(reinterpret_cast<const char *>(src), ps * ch * sizeof(PCM_TYPE));
    }

    for (const auto &dest : dests)
    {
        if (auto np = dest.lock())
        {
            np->direct_push(ch, ps, fs, src, SourceUUID{token, 0, 0});
        }
    }

    if (auto np = networker.lock())
    {
        np->send_audio(token, src, ps);
    }

    return RetCode::OK;
}

RetCode IAStream::create_device(const AudioDeviceName &_name)
{
    std::unique_ptr<AudioDevice> new_device;
    if (_name.first == "echo")
    {
        new_device = make_audio_driver(ECHO_IAS, _name, fs, ps, spf_ch, false);
    }
    else if (_name.first.substr(0, 4) == "null")
    {
        new_device = make_audio_driver(NULL_IAS, _name, fs, ps, spf_ch, false);
    }
    else if (check_wave_file_name(_name.first))
    {
        new_device = make_audio_driver(WAVE_IAS, _name, fs, ps, spf_ch, false);
    }
    else
    {
        new_device = make_audio_driver(PHSY_IAS, _name, fs, ps, spf_ch, enable_reset);
    }

    if (!new_device)
    {
        return {RetCode::FAILED, "Failed to create audio driver"};
    }

    auto ret = new_device->open();
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to open capture device [%s]: %s", _name.first.c_str(), ret.what());
        return ret;
    }

    return swap_device(new_device);
}

RetCode IAStream::create_device(const AudioDeviceName &_name, unsigned int _fs, unsigned int _ch)
{
    auto new_device = make_audio_driver(ECHO_IAS, _name, _fs, _fs * ti / 1000, _ch, false);
    if (!new_device)
    {
        return {RetCode::FAILED, "Failed to create audio driver"};
    }

    auto ret = new_device->open();
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to open capture device [%s]: %s", _name.first.c_str(), ret.what());
        return ret;
    }
    return swap_device(new_device);
}

RetCode IAStream::swap_device(idevice_ptr &new_device)
{
    auto dev_fr = new_device->fs() * ti / 1000;
    auto new_dev_buf = std::make_unique<char[]>(dev_fr * new_device->ch() * sizeof(PCM_TYPE));
    auto new_usr_buf = std::make_unique<char[]>(ps * ch * sizeof(PCM_TYPE));

    auto new_sampler = std::make_unique<LocSampler>(new_device->fs(), new_device->ch(), fs, ch, dev_fr, imap,
                                                    ch == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP);

    idevice = std::move(new_device);
    dev_buf = std::move(new_dev_buf);
    usr_buf = std::move(new_usr_buf);
    sampler = std::move(new_sampler);

    return {RetCode::OK, "Device initialized successfully"};
}

void IAStream::schedule_auto_reset()
{
    if (!enable_reset || !ias_ready)
    {
        return;
    }

    constexpr auto reset_interval = std::chrono::minutes(AUDIO_MAX_RESET_INTERVAL);
    reset_timer.expires_from_now(reset_interval);
    reset_timer.async_wait(asio::bind_executor(exec_strand, [self = shared_from_this()](const asio::error_code &ec) {
        if (ec)
        {
            AUDIO_DEBUG_PRINT("Auto reset timer error: %s", ec.message().c_str());
            return;
        }

        AUDIO_INFO_PRINT("Performing scheduled reset for stream token %u", self->token);
        self->reset_self();
    }));
}

void IAStream::schedule_callback()
{
    if (!ias_ready)
    {
        return;
    }

    auto req_ti = usr_cb.req_frs * 1000 / fs;
    cb_timer.expires_from_now(std::chrono::microseconds(req_ti * 1000 - 100));
    cb_timer.async_wait(asio::bind_executor(cb_strand, [self = shared_from_this()](const asio::error_code &ec) {
        if (ec)
        {
            AUDIO_DEBUG_PRINT("Callback timer error: %s", ec.message().c_str());
            return;
        }

        auto req_ch = self->usr_cb.mode == UsrCallBackMode::RAW ? self->idevice->ch() : self->ch;
        if (self->session->load_aside(self->usr_cb.req_frs * req_ch * sizeof(PCM_TYPE)))
        {
            self->usr_cb.cb(reinterpret_cast<PCM_TYPE *>(self->session->data()), req_ch, self->usr_cb.req_frs,
                            self->usr_cb.ptr);
        }
        self->schedule_callback();
    }));
}

void IAStream::reset_self()
{
    asio::post(reset_strand, [self = shared_from_this()]() {
        (void)self->stop();
        self->idevice.reset();
        self->idevice = make_audio_driver(PHSY_IAS, self->usr_name, self->fs, self->ps, self->ch, self->enable_reset);
        auto ret = self->idevice->open();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to open capture device [%s]: %s", self->usr_name.first.c_str(), ret.what());
        }
        self->ias_ready = true;
        ret = self->start();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to start capture device [%s]: %s", self->usr_name.first.c_str(), ret.what());
        }
    });
}

// AudioPlayer
AudioPlayer::AudioPlayer(unsigned char _token) : token(_token), preemptive(0), volume(50)
{
}

AudioPlayer::~AudioPlayer() = default;

RetCode AudioPlayer::play(const std::string &name, int cycles, const std::shared_ptr<OAStream> &sink)
{
    if (preemptive.load(std::memory_order_relaxed) > 5)
    {
        return {RetCode::FAILED, "Too many player streams"};
    }

    bool stream_found = true;
    {
        std::lock_guard<std::mutex> grd(mtx);
        stream_found = sounds.find(name) != sounds.cend();
    }

    if (stream_found)
    {
        return {RetCode::FAILED, "Stream already exists"};
    }

    auto *raw_sender = new IAStream(static_cast<unsigned char>(token + preemptive), AudioDeviceName(name, cycles),
                                    enum2val(AudioPeriodSize::INR_20MS), enum2val(AudioBandWidth::Full), 2, false);

    auto audio_sender = std::shared_ptr<IAStream>(raw_sender, [self = shared_from_this(), name](const IAStream *ptr) {
        self->preemptive.fetch_sub(1, std::memory_order_relaxed);
        std::lock_guard<std::mutex> grd(self->mtx);
        auto iter = self->sounds.find(name);
        if (iter != self->sounds.cend())
        {
            self->sounds.erase(iter);
        }

        delete ptr;
    });

    preemptive.fetch_add(1, std::memory_order_relaxed);
    sounds.emplace(name, audio_sender);

    audio_sender->set_volume(volume.load());

    auto ret = audio_sender->connect(sink);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to connect to sink: %s", ret.what());
    }
    return audio_sender->start();
}

RetCode AudioPlayer::stop(const std::string &name)
{
    std::shared_ptr<IAStream> sender;
    {
        std::lock_guard<std::mutex> grd(mtx);
        auto iter = sounds.find(name);
        if (iter == sounds.end())
        {
            return {RetCode::NOACTION, "Stream not found"};
        }

        sender = iter->second.lock();
        if (!sender)
        {
            sounds.erase(iter);
            return {RetCode::NOACTION, "Stream already expired"};
        }
    }
    return sender->stop();
}

RetCode AudioPlayer::set_volume(unsigned int vol)
{
    if (vol > 100)
    {
        AUDIO_ERROR_PRINT("Invalid volume value: %u", vol);
        return {RetCode::EPARAM, "Invalid volume value"};
    }

    volume.store(vol);

    std::lock_guard<std::mutex> grd(mtx);
    for (auto &sound_pair : sounds)
    {
        if (auto stream = sound_pair.second.lock())
        {
            stream->set_volume(vol);
        }
    }

    return RetCode::OK;
}