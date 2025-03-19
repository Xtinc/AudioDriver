#include "audio_stream.h"

#if WINDOWS_OS_ENVIRONMENT
#include <combaseapi.h>
#include <timeapi.h>
// CoInitializeEx is not thread-safe
static std::once_flag coinit_flag;
#endif

constexpr unsigned int AUDIO_MAX_COCURRECY_WORKER = 2;
constexpr unsigned int SESSION_IDLE_TIMEOUT = 1000;
constexpr AudioChannelMap DEFAULT_DUAL_MAP = {0, 1};
constexpr AudioChannelMap DEFAULT_MONO_MAP = {0, 0};

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
    std::call_once(coinit_flag, []() { CoInitializeEx(NULL, COINIT_APARTMENTTHREADED); });
    TIMECAPS tc;
    if (timeGetDevCaps(&tc, sizeof(TIMECAPS)) == TIMERR_NOERROR)
    {
        high_timer_resolution = (std::max)(tc.wPeriodMin, (std::min)((UINT)1, tc.wPeriodMax));

        if (timeBeginPeriod(high_timer_resolution) == TIMERR_NOERROR)
        {
            active_high_res_timer = true;
            AUDIO_INFO_PRINT("System timer resolution set to %u ms", high_timer_resolution);
        }
    }
#endif
}

BackgroundService::~BackgroundService()
{
    stop();
#if WINDOWS_OS_ENVIRONMENT
    if (active_high_res_timer)
    {
        timeEndPeriod(high_timer_resolution);
        AUDIO_INFO_PRINT("System timer resolution restored to default");
    }
#endif
}

void BackgroundService::start()
{
    for (size_t i = 0; i < AUDIO_MAX_COCURRECY_WORKER; ++i)
    {
        io_thds.emplace_back([this]() { io_context.run(); });
    }
}

void BackgroundService::stop()
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
    io_thds.clear();
}

asio::io_context &BackgroundService::context()
{
    return io_context;
}

// OAStream
OAStream::OAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs,
                   unsigned int _ch)
    : token(_token), ti(_ti), fs(_fs), ps(fs * ti / 1000), ch(_ch), oas_ready(false), timer(BG_SERVICE),
      exec_strand(asio::make_strand(BG_SERVICE))
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

    return ret;
}

RetCode OAStream::stop()
{
    bool expected = true;
    if (!oas_ready.compare_exchange_strong(expected, false))
    {
        return {RetCode::OK, "Device already stopped"};
    }

    timer.cancel();

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

    unsigned int std_fr = sample_rate * ti / 1000;
    uint64_t composite_key = (static_cast<uint64_t>(source_ip) << 32) | itoken;

    std::lock_guard<std::mutex> grd(session_mtx);
    auto it = sessions.find(composite_key);
    if (it == sessions.end())
    {
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

    bool success = it->second->session.store((const char *)data, frames * chan * sizeof(PCM_TYPE));
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

    timer.expires_at(tp + std::chrono::milliseconds(ti) * cnt);
    timer.async_wait(asio::bind_executor(exec_strand, [tp, cnt, self = shared_from_this()](asio::error_code ec) {
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

            if (muted)
            {
                break;
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

    odevice->write(databuf.get(), ps * ch * sizeof(PCM_TYPE));

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
    auto new_device = check_wave_file_name(_name.first) ? make_audio_driver(WAVE_OAS, _name, fs, ps, ch)
                                                        : make_audio_driver(PHSY_OAS, _name, fs, ps, ch);

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

// IAStream
IAStream::IAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs,
                   unsigned int _ch)
    : ias_ready(false), token(_token), ti(_ti), fs(_fs), ps(fs * ti / 1000), ch(_ch), timer(BG_SERVICE),
      exec_strand(asio::make_strand(BG_SERVICE))
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
    stop();

    auto ret = create_device(_name);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to create device: %s", ret.what());
    }
    ias_ready = true;
    return start();
}

RetCode IAStream::reset(const AudioDeviceName &_name, unsigned int _fs, unsigned int _ch)
{
    stop();

    auto ret = create_device(_name, _fs, _ch);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to create device: %s", ret.what());
    }
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

    return ret;
}

RetCode IAStream::stop()
{
    bool expected = true;
    if (!ias_ready.compare_exchange_strong(expected, false))
    {
        return {RetCode::OK, "Device already stopped"};
    }

    timer.cancel();
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

RetCode IAStream::direct_push(const char *data, size_t len)
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

    timer.expires_at(tp + std::chrono::milliseconds(ti) * cnt);
    timer.async_wait(asio::bind_executor(exec_strand, [tp, cnt, self = shared_from_this()](asio::error_code ec) {
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

    if (ret != RetCode::OK && ret != RetCode::NOACTION)
    {
        AUDIO_DEBUG_PRINT("Failed to read data: %s", ret.what());
        return ret;
    }

    if (muted)
    {
        return RetCode::OK;
    }

    ret = sampler->process(reinterpret_cast<const PCM_TYPE *>(dev_buf.get()), dev_fr,
                           reinterpret_cast<PCM_TYPE *>(usr_buf.get()), dev_fr);

    if (ret != RetCode::OK && ret != RetCode::NOACTION)
    {
        AUDIO_DEBUG_PRINT("Failed to resample data: %s", ret.what());
        return ret;
    }

    const PCM_TYPE *src = (ret == RetCode::NOACTION) ? reinterpret_cast<const PCM_TYPE *>(dev_buf.get())
                                                     : reinterpret_cast<const PCM_TYPE *>(usr_buf.get());
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

    return RetCode::OK;
}

RetCode IAStream::create_device(const AudioDeviceName &_name)
{
    std::unique_ptr<AudioDevice> new_device;
    if (_name.first == "echo")
    {
        new_device = make_audio_driver(ECHO_IAS, _name, fs, ps, ch);
    }
    else if (check_wave_file_name(_name.first))
    {
        new_device = make_audio_driver(WAVE_IAS, _name, fs, ps, ch);
    }
    else
    {
        new_device = make_audio_driver(PHSY_IAS, _name, fs, ps, ch);
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
    auto new_device = make_audio_driver(ECHO_IAS, _name, _fs, _fs * ti / 1000, _ch);
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

// AudioPlayer
AudioPlayer::AudioPlayer(unsigned char _token) : token(_token), preemptive(0), current_index(0), sequence_active(false)
{
}

AudioPlayer::~AudioPlayer()
{
    stop_sequence();
}

RetCode AudioPlayer::play(const std::string &name, int cycles, const std::shared_ptr<OAStream> &sink)
{
    return play_tmpl(name, cycles, nullptr, sink);
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

RetCode AudioPlayer::play_sequence(const std::vector<std::string> &file_list, const std::shared_ptr<OAStream> &sink)
{
    if (file_list.empty())
    {
        return {RetCode::FAILED, "Empty file list"};
    }

    if (!sink)
    {
        return {RetCode::FAILED, "Invalid output stream"};
    }

    stop_sequence();

    {
        std::lock_guard<std::mutex> grd(mtx);
        sequence_files = file_list;
        current_index = 0;
        sequence_sink = sink;
        sequence_active = true;
    }

    return play_tmpl(file_list[0], 1, &AudioPlayer::on_file_complete, sink);
}

RetCode AudioPlayer::stop_sequence()
{
    std::vector<std::string> files_to_stop;

    {
        std::lock_guard<std::mutex> grd(mtx);
        if (!sequence_active)
        {
            return {RetCode::NOACTION, "No active sequence"};
        }

        files_to_stop = sequence_files;
        sequence_active = false;
        sequence_files.clear();
        current_index = 0;
        sequence_sink.reset();
    }

    for (const auto &file : files_to_stop)
    {
        stop(file);
    }

    return {RetCode::OK, "Sequence playback stopped"};
}

void AudioPlayer::on_file_complete(const std::string &name)
{
    std::string next_file;
    std::shared_ptr<OAStream> sink;

    {
        std::lock_guard<std::mutex> grd(mtx);
        if (!sequence_active)
        {
            return;
        }

        current_index++;

        if (current_index < sequence_files.size())
        {
            next_file = sequence_files[current_index];
            sink = sequence_sink;
        }
        else
        {
            sequence_active = false;
            sequence_files.clear();
            current_index = 0;
            sequence_sink.reset();
            return;
        }
    }

    if (!next_file.empty() && sink)
    {
        RetCode result = play_tmpl(next_file, 1, &AudioPlayer::on_file_complete, sink);
        if (!result)
        {
            AUDIO_ERROR_PRINT("Failed to play next file in sequence: %s - %s", next_file.c_str(), result.what());
            on_file_complete(next_file);
        }
    }
}
