#ifndef AUDIO_DRIVER_HEADER
#define AUDIO_DRIVER_HEADER

#include "audio_interface.h"
#include "audio_network.h"
#include <atomic>
#include <thread>

#define PHSY_IAS 0x61
#define PHSY_OAS 0x62
#define WAVE_IAS 0x63
#define WAVE_OAS 0x64
#define ECHO_IAS 0x65

class AudioDevice
{
  public:
    virtual ~AudioDevice() = default;
    virtual RetCode open() = 0;
    virtual RetCode start() = 0;
    virtual RetCode stop() = 0;
    virtual RetCode write(const char *data, size_t len) = 0;
    virtual RetCode read(char *data, size_t len) = 0;

    unsigned int fs() const
    {
        return dev_fs;
    }
    unsigned int ps() const
    {
        return dev_ps;
    }
    unsigned int ch() const
    {
        return dev_ch;
    }
    unsigned int max_channels() const
    {
        return max_ch;
    }
    unsigned int min_channels() const
    {
        return min_ch;
    }

  public:
    const std::string hw_name;
    const bool is_capture_dev;

  protected:
    AudioDevice(const std::string &name, bool capture, unsigned int fs, unsigned int ps, unsigned int ch)
        : hw_name(name), is_capture_dev(capture), hstate(STREAM_CLOSED), dev_fs(fs), dev_ps(ps), dev_ch(ch), max_ch(0),
          min_ch(0) {};

  protected:
    enum Mode
    {
        STREAM_STOPPED,
        STREAM_RUNNING,
        STREAM_CLOSED,
        STREAM_OPENED
    };

    using AtomicMode = std::atomic<Mode>;
    using thd_ptr = std::unique_ptr<std::thread>;
    using buf_ptr = std::unique_ptr<KFifo>;

    unsigned int dev_fs;
    unsigned int dev_ps;
    unsigned int dev_ch;
    unsigned int max_ch;
    unsigned int min_ch;

    AtomicMode hstate;
    thd_ptr worker_td;
    buf_ptr io_buffer;
};

void set_current_thread_scheduler_policy();

std::unique_ptr<AudioDevice> make_audio_driver(int type, const AudioDeviceName &name, unsigned int fs, unsigned int ps,
                                               unsigned int ch);

#endif