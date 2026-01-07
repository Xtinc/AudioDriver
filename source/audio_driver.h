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
#define NULL_IAS 0x66
#define NULL_OAS 0x67

enum class ResetOrd
{
    RESET_NONE,
    RESET_SOFT,
    RESET_HARD
};

struct TimerCounter
{
    TimerCounter(const std::string &_name, unsigned int _threshold)
        : counter(0), avg_interval(0), name(_name), threshold(1000 * _threshold),
          last_print_time(std::chrono::steady_clock::now()), last_cycle_time(std::chrono::steady_clock::now()),
          sample_count(0)
    {
    }

    bool operator()()
    {
        auto now = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::microseconds>(now - last_cycle_time).count();
        last_cycle_time = now;

        updateAverage(static_cast<unsigned int>(diff));

        // Print the average interval every minute if the counter exceeds 50 per minute
        if (now - last_print_time > std::chrono::minutes(1) && counter > 50)
        {
            AUDIO_ERROR_PRINT("%s time out %u in last 1 minutes. avg interval: %uus", name.c_str(), counter,
                              avg_interval);
            last_print_time = now;
            counter = 0;
        }

        if (sample_count < 100)
        {
            // Skip the first 100 samples to stabilize the average
            last_cycle_time = now;
            return false;
        }

        if (5 * diff > 7 * threshold || 5 * diff > 7 * avg_interval)
        {
            counter++;
            return true;
        }

        return false;
    }

  private:
    void updateAverage(unsigned int current_interval)
    {
        ++sample_count;

        if (sample_count == 1)
        {
            avg_interval = current_interval;
        }
        else if (sample_count <= 100)
        {
            avg_interval = ((avg_interval * (sample_count - 1)) + current_interval) / sample_count;
        }
        else
        {
            avg_interval = (avg_interval * 99 + current_interval) / 100;
        }
    }

    unsigned int counter;
    unsigned int avg_interval;
    const std::string &name;
    unsigned int threshold;
    std::chrono::steady_clock::time_point last_print_time;
    std::chrono::steady_clock::time_point last_cycle_time;
    unsigned int sample_count;
};

class AudioDevice
{
  public:
    virtual ~AudioDevice() = default;
    virtual RetCode open() = 0;
    virtual RetCode start() = 0;
    virtual RetCode stop() = 0;
    virtual RetCode write(const char *data, size_t len) = 0;
    virtual RetCode read(char *data, size_t len) = 0;

    bool is_running() const
    {
        return hstate == STREAM_RUNNING;
    }

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
    const ResetOrd rst_order;

  protected:
    AudioDevice(const std::string &name, bool capture, unsigned int fs, unsigned int ps, unsigned int ch,
                ResetOrd reset_order)
        : hw_name(name), is_capture_dev(capture), rst_order(reset_order), dev_fs(fs), dev_ps(ps), dev_ch(ch), max_ch(0),
          min_ch(0), hstate(STREAM_CLOSED) {};

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
    using cnt_ptr = std::unique_ptr<TimerCounter>;

    unsigned int dev_fs;
    unsigned int dev_ps;
    unsigned int dev_ch;
    unsigned int max_ch;
    unsigned int min_ch;

    AtomicMode hstate;
    thd_ptr worker_td;
    buf_ptr io_buffer;
    cnt_ptr timer_cnt;
};

void set_current_thread_scheduler_policy();

std::unique_ptr<AudioDevice> make_audio_driver(int type, const AudioDeviceName &name, unsigned int fs, unsigned int ps,
                                               unsigned int ch, ResetOrd reset_order);

#endif