#ifndef AUDIO_INTERFACE_H
#define AUDIO_INTERFACE_H

#include <array>
#include <cinttypes>
#include <cstdio>
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

typedef int16_t PCM_TYPE;
typedef std::pair<std::string, unsigned int> AudioDeviceName;
typedef std::array<unsigned int, 2> AudioChannelMap;

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

#define NETWORK_AUDIO_TRANS_PORT 52282

#define AUDIO_INFO_PRINT(fmt, ...)                                                                                     \
    printf("[INF] %s(%d): " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__);                                               \
    printf("\n")
#define AUDIO_ERROR_PRINT(fmt, ...)                                                                                    \
    printf("[ERR] %s(%d): " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__);                                               \
    printf("\n")
#define AUDIO_DEBUG_PRINT(fmt, ...)                                                                                    \
    printf("[DEB] %s(%d): " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__);                                               \
    printf("\n")

#endif