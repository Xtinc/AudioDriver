#ifndef AUDIO_DRIVER_HEADER
#define AUDIO_DRIVER_HEADER

#include "audio_interface.h"
#include "audio_network.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>
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

template <size_t N = 20> class Histogram
{
    static_assert(N > 2, "Histogram must have at least 3 buckets (including underflow and overflow)");

  public:
    Histogram(const std::string &_name) : ffactor(0.99), name(_name), count(0)
    {
        std::fill(bcnts.begin(), bcnts.end(), 1.0 / N);
        constexpr auto min_val = 0.0;
        constexpr auto max_val = 1.0;
        constexpr auto delta = (max_val - min_val) / N;
        for (size_t i = 0; i < N + 1; i++)
        {
            scale[i] = min_val + i * delta;
        }
    }

    void add(double val)
    {
        double sum = 0.0;
        for (auto &cnt : bcnts)
        {
            cnt *= ffactor;
            sum += cnt;
        }

        size_t bucket_idx;
        if (val < scale[0])
        {
            bucket_idx = 0;
            scale[0] = val;
        }
        else if (val > scale[N])
        {
            bucket_idx = N - 1;
            scale[N] = val;
        }
        else
        {
            auto iter = std::lower_bound(scale.begin(), scale.begin() + N + 1, val);
            bucket_idx = std::distance(scale.begin(), iter);
            if (bucket_idx > 0)
            {
                bucket_idx--;
            }
        }

        bcnts[bucket_idx] += 2.0 - ffactor - sum;

        if (count++ % 100 == 0)
        {
            adjust_scale();
        }
    }

    std::string print() const
    {
        constexpr size_t BUFFER_SZ = 50 * N;
        char buffer[BUFFER_SZ]{};
        size_t offset = 0;

        for (size_t i = 0; i < N; i++)
        {
            if (bcnts[i] < 5e-4)
            {
                continue;
            }

            int len = snprintf(buffer + offset, BUFFER_SZ - offset, "[%6.2e, %6.2e): %6.2f%%\n", scale[i], scale[i + 1],
                               bcnts[i] * 100.0);

            if (len < 0 || offset + len >= BUFFER_SZ)
            {
                break;
            }
            offset += static_cast<size_t>(len);
        }

        return std::string(buffer);
    }

    void adjust_scale()
    {
        std::array<double, N> density;
        for (size_t i = 0; i < N; i++)
        {
            double width = scale[i + 1] - scale[i];
            density[i] = width > 1e-10 ? bcnts[i] / width : 0.0;
        }

        for (size_t i = 1; i < N; i++)
        {
            double width = scale[i + 1] - scale[i - 1];
            if (width < 1e-10)
            {
                continue;
            }
            double ratio = density[i - 1] > 1e-10 ? density[i] / density[i - 1] : 1.0;
            double move_factor = 0.5 * std::tanh(std::log(ratio));
            double new_boundary = scale[i] + move_factor * (scale[i + 1] - scale[i - 1]);
            new_boundary = std::max(scale[i - 1] + width * 0.1, std::min(scale[i + 1] - width * 0.1, new_boundary));
            scale[i] = 0.8 * scale[i] + 0.2 * new_boundary; // Smooth the boundary movement
        }

        double total = 0.0;
        for (const auto &cnt : bcnts)
        {
            total += cnt;
        }

        double expected = total / N;

        if (bcnts[0] > 1.5 * expected && scale[0] > 1e-10)
        {
            double expand = (scale[1] - scale[0]) * 0.5;
            scale[0] -= expand;
        }

        if (bcnts[N - 1] > 1.5 * expected && scale[N] < 1e10)
        {
            double expand = (scale[N] - scale[N - 1]) * 0.5;
            scale[N] += expand;
        }
    }

  private:
    const std::string name;
    const double ffactor;
    std::array<double, N> bcnts;
    std::array<double, N + 1> scale;
    uint32_t count;
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