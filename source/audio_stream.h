#ifndef AUDIO_STREAM_HEADER
#define AUDIO_STREAM_HEADER

#include "asio.hpp"
#include "audio_driver.h"
#include "audio_interface.h"
#include "audio_monitor.h"
#include "audio_network.h"
#include "audio_process.h"

#define BG_SERVICE (BackgroundService::instance().context())

using SessionData = KFifo;
using TimePointer = std::chrono::steady_clock::time_point;
using sampler_ptr = std::unique_ptr<LocSampler>;
using session_ptr = std::unique_ptr<SessionData>;
using network_ptr = std::weak_ptr<NetWorker>;
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

class IAStream;

class OAStream : public std::enable_shared_from_this<OAStream>
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
    using istream_ptr = std::weak_ptr<IAStream>;

  public:
    OAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs, unsigned int _ch);
    ~OAStream();

    RetCode start();
    RetCode stop();
    RetCode initialize_network(const std::shared_ptr<NetWorker> &nw);
    RetCode reset(const AudioDeviceName &_name);
    RetCode direct_push(unsigned char token, unsigned int chan, unsigned int frames, unsigned int sample_rate,
                        const int16_t *data, uint32_t source_ip = 0);

    void mute();
    void unmute();

    void register_listener(const std::shared_ptr<IAStream> &ias);
    void unregister_listener();

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

    std::atomic_bool oas_ready;
    asio::steady_timer timer;
    asio_strand exec_strand;

    obuffer_ptr mix_buf;
    obuffer_ptr databuf;
    odevice_ptr odevice;

    std::mutex session_mtx;
    std::map<uint64_t, context_ptr> sessions;
    network_ptr networker;
    std::mutex listener_mtx;
    istream_ptr listener;
    std::atomic_bool muted;
};

class IAStream : public std::enable_shared_from_this<IAStream>
{
    using ibuffer_ptr = std::unique_ptr<char[]>;
    using idevice_ptr = std::unique_ptr<AudioDevice>;
    using destinations = std::vector<std::weak_ptr<OAStream>>;

  public:
    IAStream(unsigned char _token, const AudioDeviceName &_name, unsigned int _ti, unsigned int _fs, unsigned int _ch,
             bool auto_reset = false);
    ~IAStream();

    RetCode start();
    RetCode stop();
    RetCode initialize_network(const std::shared_ptr<NetWorker> &nw);
    RetCode reset(const AudioDeviceName &_name, unsigned int _fs, unsigned int _ch);
    RetCode connect(const std::shared_ptr<OAStream> &oas);
    RetCode disconnect(const std::shared_ptr<OAStream> &oas);
    RetCode direct_push(const char *data, size_t len);

    void mute();
    void unmute();

  private:
    void execute_loop(TimePointer tp, unsigned int cnt);
    RetCode process_data();
    RetCode create_device(const AudioDeviceName &_name);
    RetCode create_device(const AudioDeviceName &_name, unsigned int _fs, unsigned int _ch);
    RetCode swap_device(idevice_ptr &new_device);

    RetCode reset();
    void schedule_auto_reset();

  public:
    const unsigned char token;
    const unsigned int ti;
    const unsigned int fs;
    const unsigned int ps;
    const unsigned int ch;
    const bool auto_reset_enabled;

  private:
    std::atomic_bool ias_ready;
    asio::steady_timer exec_timer;
    asio_strand exec_strand;
    asio::steady_timer reset_timer;

    ibuffer_ptr usr_buf;
    ibuffer_ptr dev_buf;
    idevice_ptr idevice;
    sampler_ptr sampler;

    std::mutex dest_mtx;
    destinations dests;
    network_ptr networker;
    std::atomic_bool muted;
};

class AudioPlayer
{
  public:
    AudioPlayer(unsigned char _token);
    ~AudioPlayer();

    RetCode play(const std::string &name, int cycles, const std::shared_ptr<OAStream> &sink);
    RetCode stop(const std::string &name);

    RetCode play_sequence(const std::vector<std::string> &file_list, const std::shared_ptr<OAStream> &sink);
    RetCode stop_sequence();

  private:
    using CompletionCallback = void (AudioPlayer::*)(const std::string &name);

    template <typename... T>
    RetCode play_tmpl(const std::string &name, int cycles, CompletionCallback on_complete, T... args)
    {
        if (preemptive > 5)
        {
            return {RetCode::FAILED, "Too many player streams"};
        }

        IAStream *raw_sender = new IAStream(token + preemptive, AudioDeviceName(name, cycles),
                                            enum2val(AudioPeriodSize::INR_20MS), enum2val(AudioBandWidth::Full), 2);

        auto audio_sender = std::shared_ptr<IAStream>(raw_sender, [this, name, on_complete](IAStream *ptr) {
            preemptive--;
            std::lock_guard<std::mutex> grd(mtx);
            auto iter = sounds.find(name);
            if (iter != sounds.cend())
            {
                sounds.erase(iter);
            }

            if (on_complete)
            {
                (this->*on_complete)(name);
            }

            delete ptr;
        });

        preemptive++;
        {
            std::lock_guard<std::mutex> grd(mtx);
            if (sounds.find(name) != sounds.cend())
            {
                return {RetCode::FAILED, "Stream already exists"};
            }
            sounds.emplace(name, audio_sender);
        }

        audio_sender->connect(args...);
        return audio_sender->start();
    }

    void on_file_complete(const std::string &name);

  private:
    const unsigned char token;
    std::mutex mtx;
    std::atomic_int preemptive;
    std::map<std::string, std::weak_ptr<IAStream>> sounds;

    std::vector<std::string> sequence_files;
    size_t current_index;
    std::shared_ptr<OAStream> sequence_sink;
    bool sequence_active;
};

#endif