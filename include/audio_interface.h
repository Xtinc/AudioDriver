#ifndef AUDIO_INTERFACE_H
#define AUDIO_INTERFACE_H

#include <array>
#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <map>
#include <memory>
#include <string>

#if defined(_WIN32) || defined(_WIN64)
#include <sdkddkver.h>
#if _WIN32_WINNT >= 0x0600 // Windows Vista
#define WINDOWS_OS_ENVIRONMENT 1
#else
#define WINDOWS_OS_ENVIRONMENT 0
#endif
#else
#define WINDOWS_OS_ENVIRONMENT 0
#endif

#if defined(__linux__) || defined(__linux) || defined(linux)
#define LINUX_OS_ENVIRONMENT 1
#else
#define LINUX_OS_ENVIRONMENT 0
#endif

#define USER_MAX_AUDIO_TOKEN 200
#define NETWORK_AUDIO_TRANS_PORT 52282

typedef int16_t PCM_TYPE;
typedef std::pair<std::string, unsigned int> AudioDeviceName;
typedef std::array<unsigned int, 2> AudioChannelMap;
typedef void (*AudioInputCallBack)(const int16_t *data, unsigned int chan_num, unsigned int frame_num, void *user_ptr);

struct RetCode
{
    enum Code
    {
        FAILED = -10,
        EOPEN,
        NOPEN,
        INVFMT,
        EWRITE,
        EREAD,
        INVSEEK,
        EPARAM,
        EXCEPTION,
        ESTATE,
        OK = 0,
        NOACTION
    };

    Code err;
    const char *msg;

    constexpr RetCode(Code e) : err(e), msg(get_default_message(e))
    {
    }
    template <size_t N> constexpr RetCode(Code e, const char (&literal)[N]) : err(e), msg(literal)
    {
    }

    constexpr bool operator==(Code code) const
    {
        return err == code;
    }
    constexpr bool operator!=(Code code) const
    {
        return err != code;
    }
    constexpr bool operator==(const RetCode &other) const
    {
        return err == other.err;
    }
    constexpr bool operator!=(const RetCode &other) const
    {
        return err != other.err;
    }

    constexpr explicit operator bool() const
    {
        return err == OK;
    }
    constexpr const char *what() const
    {
        return msg;
    }

  private:
    static constexpr const char *get_default_message(Code e)
    {
        switch (e)
        {
        case OK:
            return "Success";
        case FAILED:
            return "Operation failed";
        case EOPEN:
            return "Error opening device";
        case NOPEN:
            return "Device not opened";
        case INVFMT:
            return "Invalid format";
        case EWRITE:
            return "Write error";
        case EREAD:
            return "Read error";
        case INVSEEK:
            return "Invalid seek";
        case EXCEPTION:
            return "Throw exception";
        case ESTATE:
            return "Invalid state";
        case EPARAM:
            return "Invalid parameter";
        case NOACTION:
            return "No action";
        default:
            return "Unknown error";
        }
    }
};

enum class AudioBandWidth : unsigned int
{
    Unknown = 0,
    Narrow = 8000,
    Wide = 16000,
    SemiSuperWide = 24000,
    CDQuality = 44100,
    Full = 48000
};

enum class AudioPeriodSize : unsigned int
{
    INR_05MS = 0x05,
    INR_10MS = 0x0a,
    INR_20MS = 0x14,
    INR_40MS = 0x28
};

struct AudioToken
{
    static constexpr unsigned char INVALID_TOKEN = 0xFF;

    unsigned char tok;

    constexpr AudioToken() noexcept : tok(INVALID_TOKEN)
    {
    }

    constexpr explicit AudioToken(unsigned char t) noexcept : tok(t)
    {
    }

    constexpr operator unsigned char() const noexcept
    {
        return tok;
    }

    constexpr bool operator==(const AudioToken &other) const noexcept
    {
        return tok == other.tok;
    }

    constexpr bool operator!=(const AudioToken &other) const noexcept
    {
        return tok != other.tok;
    }
};

struct IToken : public AudioToken
{
    constexpr IToken() noexcept : AudioToken(INVALID_TOKEN)
    {
    }
    constexpr explicit IToken(unsigned char t) noexcept : AudioToken(t <= USER_MAX_AUDIO_TOKEN / 2 ? t : INVALID_TOKEN)
    {
    }
    using AudioToken::operator==;
    using AudioToken::operator!=;
};

struct OToken : public AudioToken
{
    constexpr OToken() noexcept : AudioToken(INVALID_TOKEN)
    {
    }
    constexpr explicit OToken(unsigned char t) noexcept : AudioToken(t > USER_MAX_AUDIO_TOKEN / 2 ? t : INVALID_TOKEN)
    {
    }
    using AudioToken::operator==;
    using AudioToken::operator!=;
};

constexpr IToken operator""_itk(unsigned long long val) noexcept
{
    return IToken(static_cast<unsigned char>(val));
}

constexpr OToken operator""_otk(unsigned long long val) noexcept
{
    return OToken(static_cast<unsigned char>(val));
}

class IAStream;
class OAStream;
class NetWorker;
class AudioPlayer;
class AudioMonitor;

class AudioCenter
{
  public:
    AudioCenter(bool enable_network = false);
    ~AudioCenter();

    RetCode create(IToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps, unsigned int ch,
                   bool enable_network = false, bool enable_reset = false);
    RetCode create(OToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps, unsigned int ch,
                   bool enable_network = false, bool enable_reset = false);

    RetCode prepare();
    RetCode connect(IToken itoken, OToken otoken, const std::string &ip = "");
    RetCode disconnect(IToken itoken, OToken otoken, const std::string &ip = "");
    RetCode register_callback(IToken token, AudioInputCallBack cb, void *ptr);

    RetCode start();
    RetCode stop();
    
    RetCode set_volume(AudioToken token, unsigned int vol);
    RetCode mute(AudioToken token, bool enable);
    RetCode mute(OToken token, IToken itoken, bool enable, const std::string &ip = "");
    RetCode play(const std::string &name, int cycles, OToken otoken, const std::string &ip = "");
    RetCode stop(const std::string &path);

  private:
    enum class State
    {
        INIT,
        CONNECTING,
        READY
    };

    std::atomic<State> center_state;
    std::map<unsigned char, std::shared_ptr<IAStream>> ias_map;
    std::map<unsigned char, std::shared_ptr<OAStream>> oas_map;

    std::shared_ptr<NetWorker> net_mgr;
    std::unique_ptr<AudioMonitor> monitor;
    std::unique_ptr<AudioPlayer> player;
};

#define AUDIO_INFO_PRINT(fmt, ...) printf("[INF] %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define AUDIO_ERROR_PRINT(fmt, ...) printf("[ERR] %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define AUDIO_DEBUG_PRINT(fmt, ...) printf("[DEB] %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#endif