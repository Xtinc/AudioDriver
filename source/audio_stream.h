#ifndef AUDIO_STREAM_HEADER
#define AUDIO_STREAM_HEADER

#include "asio.hpp"
#include "audio_driver.h"
#include "audio_interface.h"
#include "audio_network.h"
#include "audio_process.h"
#include <atomic>
#include <functional>
#include <map>
#include <mutex>

#define BG_SERVICE (BackgroundService::instance().context())

using SessionData = KFifo;
using TimePointer = std::chrono::steady_clock::time_point;
using sampler_ptr = std::unique_ptr<LocSampler>;
using session_ptr = std::unique_ptr<SessionData>;
using usocket_ptr = std::unique_ptr<asio::ip::udp::socket>;
using asio_strand = asio::strand<asio::io_context::executor_type>;

constexpr unsigned int SESSION_IDLE_TIMEOUT = 1000;
constexpr unsigned int NETWORK_MAX_FRAMES = enum2val(AudioPeriodSize::INR_40MS) * enum2val(AudioBandWidth::Full) / 1000;
constexpr unsigned int NETWORK_MAX_BUFFER_SIZE = NETWORK_MAX_FRAMES + 2 * sizeof(NetPacketHeader);
constexpr AudioChannelMap DEFAULT_DUAL_MAP = {0, 1};
constexpr AudioChannelMap DEFAULT_MONO_MAP = {0, 0};

class BackgroundService
{
  public:
    static BackgroundService &instance();
    void start();
    void stop();
    asio::io_context &context();

  private:
    BackgroundService();
    ~BackgroundService();

    asio::io_context io_context;
    std::vector<std::thread> io_thds;
    asio::executor_work_guard<asio::io_context::executor_type> work_guard;
#if WINDOWS_OS_ENVIRONMENT
    bool active_high_res_timer = false;
    UINT high_timer_resolution = 1;
#endif
};

class OAStream : public std::enable_shared_from_this<OAStream>
{
    struct LocSessionContext
    {
        SessionData session;
        LocSampler sampler;

        LocSessionContext(unsigned int src_fs, unsigned int src_ch, unsigned int dst_fs, unsigned int dst_ch,
                          unsigned int max_frames, const AudioChannelMap &imap, const AudioChannelMap &omap)
            : session(max_frames * sizeof(PCM_TYPE), 3, src_ch),
              sampler(src_fs, src_ch, dst_fs, dst_ch, max_frames, imap, omap)
        {
        }
    };

    struct NetSessionContext : public LocSessionContext
    {
        NetSessionContext(unsigned int src_fs, unsigned int src_ch, unsigned int dst_fs, unsigned int dst_ch,
                          unsigned int max_frames)
            : LocSessionContext(src_fs, src_ch, dst_fs, dst_ch, max_frames,
                                src_ch == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP,
                                dst_ch == 1 ? DEFAULT_MONO_MAP : DEFAULT_DUAL_MAP),
              decoder(dst_ch, NETWORK_MAX_FRAMES)
        {
        }
        NetDecoder decoder;
    };

    using obuffer_ptr = std::unique_ptr<char[]>;
    using loc_context = std::unique_ptr<LocSessionContext>;
    using net_context = std::unique_ptr<NetSessionContext>;
    using odevice_ptr = std::unique_ptr<AudioDevice>;

  public:
    OAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs, unsigned int _ch,
             bool enable_network);
    ~OAStream();

    RetCode start();
    RetCode stop();
    RetCode reset(const AudioDeviceName &_name);
    RetCode direct_push(unsigned char token, unsigned int chan, unsigned int frames, unsigned int sample_rate,
                        const int16_t *data);

  private:
    void execute_loop(TimePointer tp, unsigned int cnt);
    void write_data_to_dev();
    void network_loop();
    void store_data_to_buf(size_t bytes);

    RetCode create_device(const AudioDeviceName &_name);

    template <typename SessionMap> void process_session(SessionMap &sessions, const char *session_type);

  public:
    const unsigned char token;
    const unsigned int ti;

  private:
    unsigned int fs;
    unsigned int ps;
    unsigned int ch;

    bool network_enabled;
    std::atomic_bool oas_ready;
    asio::steady_timer timer;
    asio_strand exec_strand;

    obuffer_ptr net_buf;
    obuffer_ptr mix_buf;
    obuffer_ptr databuf;
    odevice_ptr odevice;

    usocket_ptr usocket;
    std::mutex loc_mutex;
    std::mutex net_mutex;
    asio::ip::udp::endpoint net_sender;
    std::map<uint8_t, loc_context> loc_sessions;
    std::map<uint8_t, net_context> net_sessions;
};

class IAStream : public std::enable_shared_from_this<IAStream>
{
    using encoder_ptr = std::unique_ptr<NetEncoder>;

    using ibuffer_ptr = std::unique_ptr<char[]>;
    using idevice_ptr = std::unique_ptr<AudioDevice>;
    using loc_endpoints = std::vector<std::weak_ptr<OAStream>>;
    using net_endpoints = std::vector<asio::ip::udp::endpoint>;

  public:
    IAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs, unsigned int _ch,
             bool enable_network);
    ~IAStream();

    RetCode start();
    RetCode stop();
    RetCode reset(const AudioDeviceName &_name);
    RetCode connect(const std::shared_ptr<OAStream> &oas);
    RetCode connect(const std::string &ip, uint16_t port);

  private:
    void execute_loop(TimePointer tp, unsigned int cnt);
    RetCode process_data();
    RetCode create_device(const AudioDeviceName &_name);

  public:
    const unsigned char token;
    const unsigned int ti;
    const unsigned int fs;
    const unsigned int ps;
    const unsigned int ch;

  private:
    bool network_enabled;

    std::atomic_bool ias_ready;
    asio::steady_timer timer;
    asio_strand exec_strand;

    ibuffer_ptr usr_buf;
    ibuffer_ptr dev_buf;
    idevice_ptr idevice;
    sampler_ptr sampler;
    encoder_ptr encoder;
    usocket_ptr usocket;

    std::mutex dest_mtx;
    net_endpoints net_dests;
    loc_endpoints loc_dests;
    NetPacketHeader packet_header;
};

#endif