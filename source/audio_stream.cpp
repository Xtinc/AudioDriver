#include "audio_stream.h"

#if WINDOWS_OS_ENVIRONMENT
#include <combaseapi.h>
#include <timeapi.h>
// CoInitializeEx is not thread-safe
static std::once_flag coinit_flag;
#endif
constexpr unsigned int AUDIO_MAX_RESET_INTERVAL = 30;
constexpr unsigned int AUDIO_MAX_COCURRECY_WORKER = 2;
constexpr unsigned int SESSION_IDLE_TIMEOUT = 1000;
constexpr AudioChannelMap DEFAULT_DUAL_MAP = {0, 1};
constexpr AudioChannelMap DEFAULT_MONO_MAP = {0, 0};

inline float volume2gain(unsigned int vol)
{
    return vol < 50 ? 0.68f * vol - 34.0f : 0.12f * vol - 4.0f;
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
                   unsigned int _ch, bool auto_reset)
    : token(_token), ti(_ti), enable_reset(auto_reset), fs(_fs), ps(fs * ti / 1000), ch(_ch), usr_name(_name),
      oas_ready(false), exec_timer(BG_SERVICE), exec_strand(asio::make_strand(BG_SERVICE)), reset_timer(BG_SERVICE),
      reset_strand(asio::make_strand(BG_SERVICE)), volume(50), muted(false)
{
    auto ret = create_device(_name);
    if (ret)
    {
        oas_ready = true;
    }
    else
    {
        AUDIO_ERROR_PRINT("Failed to create device: %s", ret.what());
    }
    compressor = std::make_unique<DRCompressor>(static_cast<float>(fs));
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
        nw->register_receiver(token, [weak_self](uint8_t sender_id, unsigned int channels, unsigned int frames,
                                                 unsigned int sample_rate, const int16_t *data, uint32_t source_ip) {
            if (auto self = weak_self.lock())
            {
                self->direct_push(sender_id, channels, frames, sample_rate, data, source_ip);
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

RetCode OAStream::reset(const AudioDeviceName &_name)
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
        np->reset(_name, fs, ch);
    }

    compressor = std::make_unique<DRCompressor>(static_cast<float>(fs));
    oas_ready = true;
    return start();
}

RetCode OAStream::direct_push(unsigned char itoken, unsigned int chan, unsigned int frames, unsigned int sample_rate,
                              const int16_t *data, uint32_t source_ip)
{
    if (!oas_ready)
    {
        return {RetCode::NOACTION, "Device not ready"};
    }

    if (!data)
    {
        return {RetCode::FAILED, "Invalid data pointer"};
    }

    uint64_t composite_key = InfoLabel::make_composite(source_ip, itoken);
    std::lock_guard<std::mutex> grd(session_mtx);
    auto it = sessions.find(composite_key);
    if (it == sessions.end())
    {
        unsigned int std_fr = (std::max)(frames, sample_rate * ti / 1000);
        auto imap = chan == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP;
        auto omap = ch == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP;
        auto result = sessions.emplace(composite_key,
                                       std::make_unique<SessionContext>(sample_rate, chan, fs, ch, std_fr, imap, omap));

        if (!result.second)
        {
            return {RetCode::FAILED, "Failed to create session"};
        }

        it = result.first;
        AUDIO_INFO_PRINT("%u receiver New connection: %u (IP: 0x%08X)", token, itoken, source_ip);
    }

    bool success = it->second->session.store(reinterpret_cast<const char *>(data), frames * chan * sizeof(PCM_TYPE));
    return success ? RetCode::OK : RetCode::NOACTION;
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
    if (ip.empty())
    {
        std::lock_guard<std::mutex> grd(session_mtx);
        int muted_count = 0;

        for (auto &session_pair : sessions)
        {
            if ((session_pair.first & 0xFF) == itoken)
            {
                session_pair.second->enabled = false;
                muted_count++;
            }
        }

        if (muted_count > 0)
        {
            AUDIO_INFO_PRINT("Muted %d sessions with token %u (all IPs)", muted_count, itoken);
            return RetCode::OK;
        }
        else
        {
            AUDIO_INFO_PRINT("No sessions found with token %u to mute", itoken);
            return {RetCode::NOACTION, "Session not found"};
        }
    }

    uint32_t ip_address = 0;
    try
    {
        asio::ip::address_v4 addr = asio::ip::make_address_v4(ip);
        ip_address = addr.to_uint();

        uint64_t composite_key = (static_cast<uint64_t>(ip_address) << 32) | itoken;

        std::lock_guard<std::mutex> grd(session_mtx);
        auto it = sessions.find(composite_key);
        if (it != sessions.end())
        {
            it->second->enabled = false;
            AUDIO_INFO_PRINT("Muted session from IP %s with token %u", ip.c_str(), itoken);
            return RetCode::OK;
        }
        else
        {
            AUDIO_ERROR_PRINT("Cannot find session from IP %s with token %u", ip.c_str(), itoken);
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
    if (ip.empty())
    {
        std::lock_guard<std::mutex> grd(session_mtx);
        int unmuted_count = 0;

        for (auto &session_pair : sessions)
        {
            if ((session_pair.first & 0xFF) == itoken)
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

    uint32_t ip_address = 0;
    try
    {
        asio::ip::address_v4 addr = asio::ip::make_address_v4(ip);
        ip_address = addr.to_uint();

        uint64_t composite_key = (static_cast<uint64_t>(ip_address) << 32) | itoken;

        std::lock_guard<std::mutex> grd(session_mtx);
        auto it = sessions.find(composite_key);
        if (it != sessions.end())
        {
            it->second->enabled = true;
            AUDIO_INFO_PRINT("Unmuted session from IP %s with token %u", ip.c_str(), itoken);
            return RetCode::OK;
        }
        else
        {
            AUDIO_ERROR_PRINT("Cannot find session from IP %s with token %u", ip.c_str(), itoken);
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
    AUDIO_INFO_PRINT("Token %u volume set to %u, gain: %.2f db", token, vol, volume2gain(vol));
    volume.store(vol);
    return RetCode::OK;
}

unsigned int OAStream::get_volume() const
{
    return volume.load();
}

void OAStream::register_listener(const std::shared_ptr<IAStream> &ias)
{
    ias->reset({odevice->hw_name, 0}, fs, ch);
    {
        std::lock_guard<std::mutex> grd(listener_mtx);
        listener = ias;
    }
}

void OAStream::unregister_listener()
{
    std::lock_guard<std::mutex> grd(listener_mtx);
    listener.reset();
}

void OAStream::report_conns(std::vector<InfoLabel> &result)
{
    auto oas_muted = muted.load();
    std::lock_guard<std::mutex> grd(session_mtx);
    for (const auto &session_pair : sessions)
    {
        result.emplace_back(session_pair.first, InfoLabel::make_composite(0, token),
                            session_pair.second->enabled, false, oas_muted);
    }
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
            bool has_data = context->session.load_aside(session_frames * context->session.chan * sizeof(PCM_TYPE));

            if (!has_data)
            {
                if (context->session.idle_count++ > SESSION_IDLE_TIMEOUT)
                {
                    AUDIO_INFO_PRINT("Removing empty session: %u (IP: 0x%08X)", static_cast<uint8_t>(it->first & 0xFF),
                                     static_cast<uint32_t>(it->first >> 32));
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

            unsigned int output_frames = ps;
            auto ret =
                context->sampler.process(reinterpret_cast<const PCM_TYPE *>(context->session.data()), session_frames,
                                         reinterpret_cast<PCM_TYPE *>(mix_buf.get()), output_frames);

            if (ret != RetCode::OK && ret != RetCode::NOACTION)
            {
                AUDIO_DEBUG_PRINT("Failed to process data: %s", ret.what());
                ++it;
                continue;
            }

            auto src = (ret == RetCode::NOACTION) ? reinterpret_cast<const PCM_TYPE *>(context->session.data())
                                                  : reinterpret_cast<const PCM_TYPE *>(mix_buf.get());

            if (output_frames != ps)
            {
                AUDIO_DEBUG_PRINT("Frames mismatch: %u -> %u", output_frames, ps);
                output_frames = std::min(output_frames, ps);
            }

            mix_channels(src, ch, output_frames, reinterpret_cast<PCM_TYPE *>(databuf.get()));
            ++it;
        }
    }

    auto gain = volume.load();
    if (gain != 50)
    {
        compressor->process(reinterpret_cast<PCM_TYPE *>(databuf.get()), ps, ch, volume2gain(gain));
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
        np->direct_push(databuf.get(), ps * ch * sizeof(PCM_TYPE));
    }
}

RetCode OAStream::create_device(const AudioDeviceName &_name)
{
    std::unique_ptr<AudioDevice> new_device;
    if (_name.first == "null")
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

        self->compressor->reset();
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
    : token(_token), ti(_ti), fs(_fs), ps(fs * ti / 1000), ch(_ch), enable_reset(auto_reset), usr_name(_name),
      ias_ready(false), exec_timer(BG_SERVICE), exec_strand(asio::make_strand(BG_SERVICE)), reset_timer(BG_SERVICE),
      reset_strand(asio::make_strand(BG_SERVICE)), volume(50), muted(false), usr_cb(nullptr), usr_ptr(nullptr)
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

    compressor = std::make_unique<DRCompressor>(static_cast<float>(fs));
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

RetCode IAStream::reset(const AudioDeviceName &_name)
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

    compressor->reset();
    ias_ready = true;

    return start();
}

RetCode IAStream::reset(const AudioDeviceName &_name, unsigned int _fs, unsigned int _ch)
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

    compressor = std::make_unique<DRCompressor>(static_cast<float>(fs));
    ias_ready = true;
    return start();
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

    return idevice->stop();
}

RetCode IAStream::initialize_network(const std::shared_ptr<NetWorker> &nw)
{
    if (!nw)
    {
        return {RetCode::FAILED, "Invalid or not ready NetWorker"};
    }

    networker = nw;
    return nw->register_sender(token, ch, fs);
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
    return idevice->write(data, len);
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

void IAStream::register_callback(AudioInputCallBack cb, void *ptr)
{
    usr_cb = cb;
    usr_ptr = ptr;
}

void IAStream::report_conns(std::vector<InfoLabel> &result)
{
    auto ias_muted = muted.load();
    std::lock_guard<std::mutex> grd(dest_mtx);
    for (auto iter : dests)
    {
        if (auto np = iter.lock())
        {
            result.emplace_back(0, token, 0, np->token, false, ias_muted, false);
        }
    }
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

    if (muted)
    {
        return RetCode::OK;
    }

    ret = sampler->process(reinterpret_cast<PCM_TYPE *>(dev_buf.get()), dev_fr,
                           reinterpret_cast<PCM_TYPE *>(usr_buf.get()), dev_fr);

    if (ret != RetCode::OK && ret != RetCode::NOACTION)
    {
        AUDIO_DEBUG_PRINT("Failed to resample data: %s", ret.what());
        return ret;
    }

    PCM_TYPE *src = (ret == RetCode::NOACTION) ? reinterpret_cast<PCM_TYPE *>(dev_buf.get())
                                               : reinterpret_cast<PCM_TYPE *>(usr_buf.get());

    auto gain = volume.load();
    if (gain != 50)
    {
        compressor->process(src, dev_fr, ch, volume2gain(gain));
    }

    for (const auto &dest : dests)
    {
        if (auto np = dest.lock())
        {
            np->direct_push(token, ch, dev_fr, fs, src);
        }
    }

    if (auto np = networker.lock())
    {
        np->send_audio(token, src, dev_fr);
    }

    if (usr_cb)
    {
        usr_cb(src, ch, dev_fr, usr_ptr);
    }

    return RetCode::OK;
}

RetCode IAStream::create_device(const AudioDeviceName &_name)
{
    std::unique_ptr<AudioDevice> new_device;
    if (_name.first == "echo")
    {
        new_device = make_audio_driver(ECHO_IAS, _name, fs, ps, ch, false);
    }
    else if (_name.first == "null")
    {
        new_device = make_audio_driver(NULL_IAS, _name, fs, ps, ch, false);
    }
    else if (check_wave_file_name(_name.first))
    {
        new_device = make_audio_driver(WAVE_IAS, _name, fs, ps, ch, false);
    }
    else
    {
        new_device = make_audio_driver(PHSY_IAS, _name, fs, ps, ch, enable_reset);
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

    auto new_sampler = std::make_unique<LocSampler>(new_device->fs(), new_device->ch(), fs, ch, dev_fr,
                                                    new_device->ch() == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP,
                                                    ch == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP);

    if (!new_sampler->is_valid())
    {
        AUDIO_ERROR_PRINT("Failed to create sampler for device [%s]", new_device->hw_name.c_str());
        return {RetCode::FAILED, "Failed to create sampler"};
    }

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
        self->compressor->reset();
        self->ias_ready = true;
        ret = self->start();
        if (!ret)
        {
            AUDIO_ERROR_PRINT("Failed to start capture device [%s]: %s", self->usr_name.first.c_str(), ret.what());
        }
    });
}

// AudioPlayer
AudioPlayer::AudioPlayer(unsigned char _token) : token(_token), preemptive(0)
{
}

AudioPlayer::~AudioPlayer() = default;

RetCode AudioPlayer::play(const std::string &name, int cycles, const std::shared_ptr<OAStream> &sink) 
{
    if (preemptive > 5)
    {
        return {RetCode::FAILED, "Too many player streams"};
    }

    auto *raw_sender = new IAStream(token + preemptive, AudioDeviceName(name, cycles),
                                    enum2val(AudioPeriodSize::INR_20MS), enum2val(AudioBandWidth::Full), 2);

    auto audio_sender = std::shared_ptr<IAStream>(raw_sender, [self = shared_from_this(), name](const IAStream *ptr) {
        --(self->preemptive);
        std::lock_guard<std::mutex> grd(self->mtx);
        auto iter = self->sounds.find(name);
        if (iter != self->sounds.cend())
        {
            self->sounds.erase(iter);
        }

        delete ptr;
    });

    ++preemptive;
    {
        std::lock_guard<std::mutex> grd(mtx);
        if (sounds.find(name) != sounds.cend())
        {
            return {RetCode::FAILED, "Stream already exists"};
        }
        sounds.emplace(name, audio_sender);
    }

    audio_sender->connect(sink);
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

RetCode AudioPlayer::play(const std::string &name, int cycles, const std::shared_ptr<NetWorker> &networker,
                          uint8_t remote_token, const std::string &remote_ip)
{
    if (!networker)
    {
        return {RetCode::FAILED, "NetWorker not provided"};
    }

    if (remote_ip.empty())
    {
        return {RetCode::FAILED, "Remote IP not provided"};
    }

    if (preemptive > 5)
    {
        return {RetCode::FAILED, "Too many player streams"};
    }

    auto *raw_sender = new IAStream(token + preemptive, AudioDeviceName(name, cycles),
                                    enum2val(AudioPeriodSize::INR_40MS), enum2val(AudioBandWidth::Full), 2);

    auto audio_sender = std::shared_ptr<IAStream>(raw_sender, [this, name](const IAStream *ptr) {
        --preemptive;
        std::lock_guard<std::mutex> grd(mtx);
        auto iter = sounds.find(name);
        if (iter != sounds.cend())
        {
            sounds.erase(iter);
        }

        delete ptr;
        AUDIO_INFO_PRINT("Remote stream for file %s released", name.c_str());
    });

    auto init_result = audio_sender->initialize_network(networker);
    if (!init_result)
    {
        AUDIO_ERROR_PRINT("Failed to initialize network for sender: %s", init_result.what());
        return init_result;
    }

    auto add_result = networker->add_destination(audio_sender->token, remote_token, remote_ip);
    if (!add_result)
    {
        AUDIO_ERROR_PRINT("Failed to add destination %s for token %u: %s", remote_ip.c_str(), remote_token,
                          add_result.what());
        return add_result;
    }

    ++preemptive;
    {
        std::lock_guard<std::mutex> grd(mtx);
        if (sounds.find(name) != sounds.cend())
        {
            --preemptive;
            networker->del_destination(audio_sender->token, remote_token, remote_ip);
            return {RetCode::FAILED, "Stream already exists"};
        }
        sounds.emplace(name, audio_sender);
    }

    auto start_result = audio_sender->start();
    if (!start_result)
    {
        std::lock_guard<std::mutex> grd(mtx);
        sounds.erase(name);
        --preemptive;
        networker->del_destination(audio_sender->token, remote_token, remote_ip);
        AUDIO_ERROR_PRINT("Failed to start streaming: %s", start_result.what());
        return start_result;
    }

    AUDIO_INFO_PRINT("Started remote playback of %s to %s for token %u", name.c_str(), remote_ip.c_str(), remote_token);

    return {RetCode::OK, "Remote playback started"};
}