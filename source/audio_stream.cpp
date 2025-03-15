#include "audio_stream.h"
#include <thread>

#if WINDOWS_OS_ENVIRONMENT
#include <combaseapi.h>
#include <timeapi.h>
#endif

static std::once_flag coinit_flag;
static constexpr AudioChannelMap DEFAULT_DUAL_MAP = {0, 1};
static constexpr AudioChannelMap DEFAULT_MONO_MAP = {0, 0};

static bool check_wave_file_name(const std::string &dev_name)
{
    return dev_name.size() >= 4 && dev_name.compare(dev_name.size() - 4, 4, ".wav") == 0;
}

template <typename StreamType, typename ReadyFlag>
void execute_stream_loop(StreamType *stream, ReadyFlag &ready_flag, asio::steady_timer &timer, asio_strand &exec_strand,
                         TimePointer tp, unsigned int cnt, unsigned int ti)
{
    if (!ready_flag)
    {
        return;
    }

    if (cnt == 0)
    {
        tp = std::chrono::steady_clock::now();
    }

    timer.expires_at(tp + std::chrono::milliseconds(ti) * cnt);
    timer.async_wait(
        asio::bind_executor(exec_strand, [stream, &ready_flag, &timer, &exec_strand, tp, cnt, ti](asio::error_code ec) {
            if (ec)
            {
                return;
            }
            execute_stream_loop(stream, ready_flag, timer, exec_strand, tp, cnt + 1, ti);
            stream->process_data();
        }));
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
    const auto thread_count = std::thread::hardware_concurrency();

    for (size_t i = 0; i < thread_count; ++i)
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
    : token(_token), ti(_ti), fs(_fs), ps(fs * ti / 1000), ch(_ch), timer(BG_SERVICE),
      exec_strand(asio::make_strand(BG_SERVICE)), oas_ready(false)
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

OAStream::~OAStream()
{
    (void)stop();
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
    return odevice->stop();
}

RetCode OAStream::reset(const AudioDeviceName &_name)
{
    stop();
    {
        std::lock_guard<std::mutex> grd(recv_mtx);
        loc_sessions.clear();
    }

    auto ret = create_device(_name);
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Failed to create device: %s", ret.what());
    }

    oas_ready = true;
    return start();
}

RetCode OAStream::direct_push(unsigned char token, unsigned int chan, unsigned int frames, unsigned int sample_rate,
                              const int16_t *data)
{
    if (!oas_ready)
    {
        return {RetCode::FAILED, "Device not ready"};
    }

    if (!data)
    {
        return {RetCode::FAILED, "Invalid data pointer"};
    }

    unsigned int std_fr = sample_rate * ti / 1000;

    std::lock_guard<std::mutex> grd(recv_mtx);
    if (loc_sessions.find(token) == loc_sessions.end())
    {
        auto io_map = chan == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP;
        loc_sessions.emplace(token,
                             std::make_unique<SessionContext>(sample_rate, chan, fs, ch, std_fr, io_map, io_map));
        AUDIO_INFO_PRINT("new connection: %u", token);
    }
    loc_sessions.at(token)->session.store((const char *)data, frames * chan * sizeof(PCM_TYPE));
    return {RetCode::OK, "Success"};
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
    timer.async_wait(asio::bind_executor(exec_strand, [this, tp, cnt](asio::error_code ec) {
        if (ec)
        {
            return;
        }
        execute_loop(tp, cnt + 1);
        process_data();
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
        std::lock_guard<std::mutex> grd(recv_mtx);

        for (auto it = loc_sessions.begin(); it != loc_sessions.end();)
        {
            auto &context = it->second;
            unsigned int session_frames = context->sampler.src_fs * ti / 1000;
            bool has_data = context->session.load_aside(session_frames * context->session.chan * sizeof(PCM_TYPE));
            if (!has_data)
            {
                if (context->session.idle_count++ > 1000)
                {
                    AUDIO_INFO_PRINT("removing empty session: %u", it->first);
                    it = loc_sessions.erase(it);
                    continue;
                }

                ++it;
                continue;
            }

            context->session.idle_count = 0;
            unsigned int output_frames = ps;
            auto ret =
                context->sampler.process(reinterpret_cast<const PCM_TYPE *>(context->session.data()), session_frames,
                                         reinterpret_cast<PCM_TYPE *>(mix_buf.get()), output_frames);
            if (ret != RetCode::OK && ret != RetCode::NOACTION)
            {
                AUDIO_DEBUG_PRINT("Failed to resample data: %s", ret.what());
                ++it;
                continue;
            }

            auto src = (ret == RetCode::NOACTION) ? reinterpret_cast<const PCM_TYPE *>(context->session.data())
                                                  : reinterpret_cast<const PCM_TYPE *>(mix_buf.get());

            if (output_frames != ps)
            {
                AUDIO_DEBUG_PRINT("Resample frames mismatch: %u -> %u", output_frames, ps);
                output_frames = std::min(output_frames, ps);
            }

            mix_channels(src, ch, context->session.chan, output_frames, reinterpret_cast<PCM_TYPE *>(databuf.get()));
            ++it;
        }
    }

    odevice->write(databuf.get(), ps * ch * sizeof(PCM_TYPE));
}

RetCode OAStream::create_device(const AudioDeviceName &_name)
{
    auto new_device = check_wave_file_name(_name.first) ? make_audio_driver(WAVE_OAS, _name, fs, ps, ch)
                                                        : make_audio_driver(PHSY_OAS, _name, fs, ps, ch);

    auto ret = new_device->open();
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Open playback device [%s] failed: %s", _name.first.c_str(), ret.what());
        return ret;
    }

    unsigned int new_fs = new_device->fs();
    unsigned int new_ch = new_device->ch();
    unsigned int new_ps = new_fs * ti / 1000;

    auto new_databuf = std::make_unique<char[]>(new_ps * new_ch * sizeof(PCM_TYPE));
    auto new_mix_buf = std::make_unique<char[]>(new_ps * new_ch * sizeof(PCM_TYPE));

    odevice = std::move(new_device);
    fs = new_fs;
    ch = new_ch;
    ps = new_ps;
    databuf = std::move(new_databuf);
    mix_buf = std::move(new_mix_buf);

    return {RetCode::OK, "Device initialized successfully"};
}

// IAStream
IAStream::IAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs,
                   unsigned int _ch, bool enable_network)
    : ias_ready(false), token(_token), ti(_ti), fs(_fs), ps(fs * ti / 1000), ch(_ch), timer(BG_SERVICE),
      exec_strand(asio::make_strand(BG_SERVICE)), network_enabled(enable_network)
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

    if (enable_network)
    {
        packet_header.magic_num = NET_MAGIC_NUM;
        packet_header.sender_id = token;
        packet_header.receiver_id = 0; // Broadcast
        packet_header.encoder_fmt = 1; // ADPCM
        packet_header.channels = ch;
        packet_header.sample_rate = fs;
        packet_header.sequence = 0;  // Will be updated on each send
        packet_header.timestamp = 0; // Will be updated on each send

        try
        {
            encoder = std::make_unique<NetEncoder>(ch, ps);
            usocket = std::make_unique<asio::ip::udp::socket>(BG_SERVICE);
            usocket->open(asio::ip::udp::v4());
            // Set UDP socket options for better performance
            asio::socket_base::send_buffer_size option(262144); // 256KB
            asio::error_code ec;
            usocket->set_option(option, ec);
            if (ec)
            {
                AUDIO_DEBUG_PRINT("Failed to set socket buffer size: %s", ec.message().c_str());
            }
        }
        catch (const std::exception &e)
        {
            network_enabled = false;
            AUDIO_ERROR_PRINT("Failed to create UDP socket: %s", e.what());
        }
    }

    AUDIO_INFO_PRINT("Network functionality %s", network_enabled ? "enabled" : "disabled");
    loc_dests.reserve(4);
}

IAStream::~IAStream()
{
    (void)stop();

    if (usocket && usocket->is_open())
    {
        asio::error_code ec;
        usocket->close(ec);
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

    loc_dests.emplace_back(oas);
    return {RetCode::OK, "Connection established"};
}

RetCode IAStream::connect(const std::string &ip, uint16_t port)
{
    if (!network_enabled)
    {
        return {RetCode::FAILED, "Network functionality is disabled or socket not available"};
    }

    asio::ip::udp::resolver resolver(BG_SERVICE);
    asio::error_code ec;

    auto dest = *resolver.resolve(asio::ip::udp::v4(), ip, std::to_string(port), ec).begin();
    if (ec)
    {
        AUDIO_ERROR_PRINT("%s\n", ec.message().c_str());
        return {RetCode::FAILED, "Failed to add network destination"};
    }
    std::lock_guard<std::mutex> grd(dest_mtx);
    net_dests.push_back(std::move(dest));
    AUDIO_INFO_PRINT("Added network destination: %s:%d", ip.c_str(), port);
    return {RetCode::OK, "Network destination added"};
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

    if (ret != RetCode::OK)
    {
        AUDIO_DEBUG_PRINT("Failed to read data: %s", ret.what());
        return ret;
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
    for (const auto &dest : loc_dests)
    {
        if (auto np = dest.lock())
        {
            np->direct_push(token, ch, dev_fr, fs, src);
        }
    }

    if (network_enabled)
    {
        size_t encoded_size;
        const uint8_t *encoded_data = encoder->encode(src, dev_fr, encoded_size);
        if (!encoded_data)
        {
            AUDIO_DEBUG_PRINT("Failed to encode data");
            return RetCode::OK;
        }

        packet_header.sequence++;
        packet_header.timestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        // gather-scatter operation
        std::array<asio::const_buffer, 2> buffers = {asio::buffer(&packet_header, sizeof(packet_header)),
                                                     asio::buffer(encoded_data, encoded_size)};
        std::lock_guard<std::mutex> grd(dest_mtx);
        for (const auto &endpoint : net_dests)
        {
            usocket->async_send_to(
                buffers, endpoint,
                [self = shared_from_this(), endpoint](const asio::error_code &error, std::size_t bytes_transferred) {
                    if (error)
                    {
                        if (error == asio::error::connection_refused || error == asio::error::connection_reset)
                        {
                            // Consider removing this destination automatically
                            std::lock_guard<std::mutex> grd(self->dest_mtx);
                            auto it = std::find(self->net_dests.begin(), self->net_dests.end(), endpoint);
                            if (it != self->net_dests.end())
                            {
                                AUDIO_INFO_PRINT("Removing unreachable destination: %s:%d",
                                                 it->address().to_string().c_str(), it->port());
                                self->net_dests.erase(it);
                            }
                        }
                        else
                        {
                            AUDIO_DEBUG_PRINT("Send error: %s", error.message().c_str());
                        }
                    }
                });
        }
    }

    return RetCode::OK;
}

RetCode IAStream::create_device(const AudioDeviceName &_name)
{
    auto new_device = check_wave_file_name(_name.first) ? make_audio_driver(WAVE_IAS, _name, fs, ps, ch)
                                                        : make_audio_driver(PHSY_IAS, _name, fs, ps, ch);

    auto ret = new_device->open();
    if (!ret)
    {
        AUDIO_ERROR_PRINT("Open capture device [%s] failed: %s", _name.first.c_str(), ret.what());
        return ret;
    }

    auto dev_fr = new_device->fs() * ti / 1000;
    auto new_dev_buf = std::make_unique<char[]>(dev_fr * new_device->ch() * sizeof(PCM_TYPE));
    auto new_usr_buf = std::make_unique<char[]>(ps * ch * sizeof(PCM_TYPE));

    auto new_sampler = std::make_unique<LocSampler>(new_device->fs(), new_device->ch(), fs, ch, dev_fr,
                                                    new_device->ch() == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP,
                                                    ch == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP);

    if (!new_sampler->is_valid())
    {
        AUDIO_ERROR_PRINT("Failed to create sampler for device [%s]", _name.first.c_str());
        return {RetCode::FAILED, "Failed to create sampler"};
    }

    idevice = std::move(new_device);
    dev_buf = std::move(new_dev_buf);
    usr_buf = std::move(new_usr_buf);
    sampler = std::move(new_sampler);

    return {RetCode::OK, "Device initialized successfully"};
}