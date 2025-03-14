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
using asio_strand = asio::strand<asio::io_context::executor_type>;

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

class OAStream
{
    struct SessionContext
    {
        SessionData session;
        LocSampler sampler;

        SessionContext(unsigned int src_fs, unsigned int src_ch, unsigned int dst_fs, unsigned int dst_ch,
                       unsigned int max_frames, const AudioChannelMap &imap, const AudioChannelMap &omap)
            : session(max_frames * sizeof(PCM_TYPE), 3, src_ch),
              sampler(src_fs, src_ch, dst_fs, dst_ch, max_frames, imap, omap)
        {
        }
    };

    using obuffer_ptr = std::unique_ptr<char[]>;
    using context_ptr = std::unique_ptr<SessionContext>;
    using odevice_ptr = std::unique_ptr<AudioDevice>;

  public:
    OAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs, unsigned int _ch);
    ~OAStream();

    RetCode start();
    RetCode stop();
    RetCode reset(const AudioDeviceName &_name);
    RetCode direct_push(unsigned char token, unsigned int chan, unsigned int frames, unsigned int sample_rate,
                        const int16_t *data);

  private:
    void execute_loop(TimePointer tp, unsigned int cnt);
    void process_data();
    RetCode create_device(const AudioDeviceName &_name);

  public:
    const unsigned char token;
    const unsigned int ti;

  private:
    unsigned int fs;
    unsigned int ps;
    unsigned int ch;

    asio::steady_timer timer;
    asio_strand exec_strand;

    obuffer_ptr mix_buf;
    obuffer_ptr databuf;
    odevice_ptr odevice;
    std::mutex recv_mtx;
    std::map<uint8_t, context_ptr> loc_sessions;
    std::atomic_bool oas_ready;
};

class IAStream : public std::enable_shared_from_this<IAStream>
{
    using ibuffer_ptr = std::unique_ptr<char[]>;
    using session_ptr = std::unique_ptr<SessionData>;
    using idevice_ptr = std::unique_ptr<AudioDevice>;
    using loc_endpoints = std::vector<std::weak_ptr<OAStream>>;

  public:
    IAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs, unsigned int _ch);
    ~IAStream();

    RetCode start();
    RetCode stop();
    RetCode reset(const AudioDeviceName &_name);
    RetCode connect(const std::shared_ptr<OAStream> &oas);

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
    std::atomic_bool ias_ready;
    asio::steady_timer timer;
    asio_strand exec_strand;

    ibuffer_ptr usr_buf;
    ibuffer_ptr dev_buf;
    idevice_ptr idevice;
    sampler_ptr sampler;

    loc_endpoints loc_dests;
};

#endif