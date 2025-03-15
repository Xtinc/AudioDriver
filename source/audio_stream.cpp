#include "audio_stream.h"
#include <thread>

#if WINDOWS_OS_ENVIRONMENT
#include <combaseapi.h>
#include <timeapi.h>
// CoInitializeEx is not thread-safe
static std::once_flag coinit_flag;
#endif

using udp = asio::ip::udp;

static bool check_wave_file_name(const std::string &dev_name)
{
    return dev_name.size() >= 4 && dev_name.compare(dev_name.size() - 4, 4, ".wav") == 0;
}

inline constexpr uint16_t token2port(unsigned char token)
{
    return (uint16_t)(0xccu << 8) + (uint16_t)token;
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
                   unsigned int _ch, bool enable_network)
    : token(_token), ti(_ti), fs(_fs), ps(fs * ti / 1000), ch(_ch), network_enabled(false), oas_ready(false),
      timer(BG_SERVICE), exec_strand(asio::make_strand(BG_SERVICE))
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

    if (enable_network && oas_ready)
    {
        try
        {
            net_buf = std::make_unique<char[]>(NETWORK_MAX_BUFFER_SIZE);
            usocket = std::make_unique<udp::socket>(BG_SERVICE, udp::endpoint(udp::v4(), token2port(token)));
            asio::socket_base::receive_buffer_size option(262144); // 256KB
            asio::error_code ec;
            usocket->set_option(option, ec);
            if (ec)
            {
                AUDIO_DEBUG_PRINT("Failed to set receive buffer size: %s", ec.message().c_str());
            }
            network_enabled = true;
            AUDIO_INFO_PRINT("UDP listener started on port %u", token2port(token));
        }
        catch (const std::exception &e)
        {
            AUDIO_ERROR_PRINT("Failed to initialize network components: %s", e.what());
        }
    }

    AUDIO_INFO_PRINT("Network functionality %s", network_enabled ? "enabled" : "disabled");
}

OAStream::~OAStream()
{
    try
    {
        (void)stop();

        if (usocket && usocket->is_open())
        {
            asio::error_code ec;
            usocket->close(ec);
            if (ec)
            {
                AUDIO_DEBUG_PRINT("Error closing socket: %s", ec.message().c_str());
            }
        }
    }
    catch (const std::exception &e)
    {
        AUDIO_ERROR_PRINT("Exception in OAStream destructor: %s", e.what());
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
        if (network_enabled)
        {
            network_loop();
        }
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

    if (network_enabled)
    {
        asio::error_code ec;
        usocket->cancel(ec);
        if (ec)
        {
            AUDIO_DEBUG_PRINT("Error canceling socket operations: %s", ec.message().c_str());
        }
    }

    {
        std::lock_guard<std::mutex> grd(loc_mutex);
        loc_sessions.clear();
    }

    {
        std::lock_guard<std::mutex> grd(net_mutex);
        net_sessions.clear();
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

    oas_ready = true;
    return start();
}

RetCode OAStream::direct_push(unsigned char token, unsigned int chan, unsigned int frames, unsigned int sample_rate,
                              const int16_t *data)
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

    std::lock_guard<std::mutex> grd(loc_mutex);
    auto it = loc_sessions.find(token);
    if (it == loc_sessions.end())
    {
        auto io_map = chan == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP;
        auto result = loc_sessions.emplace(
            token, std::make_unique<LocSessionContext>(sample_rate, chan, fs, ch, std_fr, io_map, io_map));

        if (!result.second)
        {
            return {RetCode::FAILED, "Failed to create session"};
        }

        it = result.first;
        AUDIO_INFO_PRINT("New connection: %u", token);
    }

    bool success = it->second->session.store((const char *)data, frames * chan * sizeof(PCM_TYPE));
    return success ? RetCode::OK : RetCode{RetCode::NOACTION, "Failed to store data (buffer may be full)"};
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
    timer.async_wait(asio::bind_executor(exec_strand, [tp, cnt, this, self = shared_from_this()](asio::error_code ec) {
        if (ec)
        {
            AUDIO_DEBUG_PRINT("Timer error: %s", ec.message().c_str());
            return;
        }

        execute_loop(tp, cnt + 1);
        write_data_to_dev();
    }));
}

void OAStream::write_data_to_dev()
{
    if (!oas_ready)
    {
        return;
    }

    std::memset(databuf.get(), 0, ch * ps * sizeof(PCM_TYPE));
    {
        std::lock_guard<std::mutex> grd(loc_mutex);
        if (!loc_sessions.empty())
        {
            process_session(loc_sessions, "local");
        }
    }

    {
        std::lock_guard<std::mutex> grd(net_mutex);
        if (!net_sessions.empty())
        {
            process_session(net_sessions, "network");
        }
    }

    odevice->write(databuf.get(), ps * ch * sizeof(PCM_TYPE));
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

    odevice = std::move(new_device);
    fs = new_fs;
    ch = new_ch;
    ps = new_ps;
    databuf = std::move(new_databuf);
    mix_buf = std::move(new_mix_buf);

    return {RetCode::OK, "Device initialized successfully"};
}

void OAStream::network_loop()
{
    usocket->async_receive_from(asio::buffer(net_buf.get(), NETWORK_MAX_BUFFER_SIZE), net_sender,
                                [this, self_ptr = shared_from_this()](std::error_code ec, std::size_t bytes) {
                                    if (ec)
                                    {
                                        if (ec == asio::error::operation_aborted)
                                        {
                                            return;
                                        }
                                        AUDIO_DEBUG_PRINT("Network receive error: %s", ec.message().c_str());
                                        network_loop();
                                        return;
                                    }

                                    store_data_to_buf(bytes);
                                    network_loop();
                                });
}

void OAStream::store_data_to_buf(size_t bytes)
{
    if (!oas_ready)
    {
        return;
    }

    if (!NetPacketHeader::validate(net_buf.get(), bytes))
    {
        AUDIO_DEBUG_PRINT("Invalid network packet");
        return;
    }

    auto sender = static_cast<uint8_t>(net_buf[0]);
    auto net_ch = static_cast<unsigned int>(net_buf[1]);
    auto net_fs = enum2val(byte_to_bandwidth(net_buf[2]));
    auto adpcm_data = net_buf.get() + sizeof(NetPacketHeader);
    size_t adpcm_size = bytes - sizeof(NetPacketHeader);
    auto net_fr = static_cast<unsigned int>(adpcm_size * 4 / (net_ch * sizeof(PCM_TYPE)) + 1);
    NetSessionContext *session = nullptr;

    {
        std::lock_guard<std::mutex> grd(net_mutex);
        auto session_it = net_sessions.find(sender);

        if (session_it == net_sessions.end())
        {
            auto new_session = std::make_unique<NetSessionContext>(net_fs, net_ch, fs, ch, net_fr);
            session = new_session.get();
            net_sessions.emplace(sender, std::move(new_session));
            AUDIO_INFO_PRINT("New network session from sender: %u", sender);
        }
        else
        {
            session = session_it->second.get();
        }

        unsigned int decode_frames = 0;
        auto decode_data =
            session->decoder.decode(reinterpret_cast<const uint8_t *>(adpcm_data), adpcm_size, decode_frames);

        if (decode_data && decode_frames > 0)
        {
            session->session.store(reinterpret_cast<const char *>(decode_data),
                                   decode_frames * net_ch * sizeof(PCM_TYPE));
        }
    }
}

template <typename SessionMap> void OAStream::process_session(SessionMap &sessions, const char *session_type)
{
    for (auto it = sessions.begin(); it != sessions.end();)
    {
        auto &context = it->second;
        unsigned int session_frames = context->sampler.src_fs * ti / 1000;
        bool has_data = context->session.load_aside(session_frames * context->session.chan * sizeof(PCM_TYPE));

        if (!has_data)
        {
            if (context->session.idle_count++ > SESSION_IDLE_TIMEOUT)
            {
                AUDIO_INFO_PRINT("Removing empty %s session: %u", session_type, it->first);
                it = sessions.erase(it);
                continue;
            }
            ++it;
            continue;
        }

        context->session.idle_count = 0;
        unsigned int output_frames = ps;
        auto ret = context->sampler.process(reinterpret_cast<const PCM_TYPE *>(context->session.data()), session_frames,
                                            reinterpret_cast<PCM_TYPE *>(mix_buf.get()), output_frames);

        if (ret != RetCode::OK && ret != RetCode::NOACTION)
        {
            AUDIO_DEBUG_PRINT("Failed to process %s data: %s", session_type, ret.what());
            ++it;
            continue;
        }

        auto src = (ret == RetCode::NOACTION) ? reinterpret_cast<const PCM_TYPE *>(context->session.data())
                                              : reinterpret_cast<const PCM_TYPE *>(mix_buf.get());

        if (output_frames != ps)
        {
            AUDIO_DEBUG_PRINT("%s frames mismatch: %u -> %u", session_type, output_frames, ps);
            output_frames = std::min(output_frames, ps);
        }

        mix_channels(src, ch, context->session.chan, output_frames, reinterpret_cast<PCM_TYPE *>(databuf.get()));
        ++it;
    }
}

// IAStream
IAStream::IAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs,
                   unsigned int _ch, bool enable_network)
    : ias_ready(false), token(_token), ti(_ti), fs(_fs), ps(fs * ti / 1000), ch(_ch), timer(BG_SERVICE),
      exec_strand(asio::make_strand(BG_SERVICE)), network_enabled(false)
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

    if (enable_network && ias_ready)
    {
        packet_header.sender_id = token;
        packet_header.channels = ch;
        packet_header.magic_num = NET_MAGIC_NUM;
        packet_header.sample_rate = fs / 1000;
        packet_header.sequence = 0;  // Will be updated on each send
        packet_header.timestamp = 0; // Will be updated on each send

        try
        {
            encoder = std::make_unique<NetEncoder>(ch, ps);
            usocket = std::make_unique<udp::socket>(BG_SERVICE);
            usocket->open(udp::v4());
            network_enabled = true;
        }
        catch (const std::exception &e)
        {
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

RetCode IAStream::connect(const std::string &ip, uint8_t token)
{
    if (!network_enabled)
    {
        return {RetCode::FAILED, "Network functionality is disabled or socket not available"};
    }

    if (ip.empty())
    {
        return {RetCode::FAILED, "Invalid IP address"};
    }

    udp::resolver resolver(BG_SERVICE);
    asio::error_code ec;

    auto port = token2port(token);
    auto endpoints = resolver.resolve(udp::v4(), ip, std::to_string(port), ec);
    if (ec)
    {
        AUDIO_ERROR_PRINT("Failed to resolve address %s:%d: %s", ip.c_str(), port, ec.message().c_str());
        return {RetCode::FAILED, "Failed to resolve address"};
    }

    auto dest = *endpoints.begin();
    {
        std::lock_guard<std::mutex> grd(dest_mtx);

        if (std::find(net_dests.begin(), net_dests.end(), dest) != net_dests.end())
        {
            AUDIO_INFO_PRINT("Destination already exists: %s:%d", ip.c_str(), port);
            return {RetCode::OK, "Destination already exists"};
        }

        net_dests.push_back(std::move(dest));
    }

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
    timer.async_wait(asio::bind_executor(exec_strand, [tp, cnt, this, self = shared_from_this()](asio::error_code ec) {
        if (ec)
        {
            AUDIO_DEBUG_PRINT("Timer error: %s", ec.message().c_str());
            return;
        }
        execute_loop(tp, cnt + 1);
        if (process_data() == RetCode::INVSEEK)
        {
            ias_ready = false;
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
            AUDIO_DEBUG_PRINT("Failed to encode data: buffer may be empty or invalid");
            return RetCode::OK;
        }

        packet_header.sequence++;
        packet_header.timestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
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
                    if (!error)
                    {
                        return;
                    }

                    if (error == asio::error::connection_refused || error == asio::error::connection_reset)
                    {
                        // Consider removing this destination automatically
                        std::lock_guard<std::mutex> grd(self->dest_mtx);
                        auto it = std::find(self->net_dests.begin(), self->net_dests.end(), endpoint);
                        if (it != self->net_dests.end())
                        {
                            AUDIO_INFO_PRINT("Removed unreachable destination: %s:%d",
                                             it->address().to_string().c_str(), it->port());
                            self->net_dests.erase(it);
                        }
                    }
                    else
                    {
                        AUDIO_DEBUG_PRINT("Send error: %s", error.message().c_str());
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