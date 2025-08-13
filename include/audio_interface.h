#ifndef AUDIO_INTERFACE_H
#define AUDIO_INTERFACE_H

/**
 * @file audio_interface.h
 * @brief Main interface for the AudioDriver library
 *
 * This header defines the core classes and types used in the AudioDriver library,
 * including AudioCenter which serves as the main entry point to the library functionality.
 */

#include "audio_message.h"
#include <array>
#include <atomic>
#include <cinttypes>
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

constexpr unsigned char USER_MAX_AUDIO_TOKEN = 201;
constexpr unsigned short NETWORK_AUDIO_TRANS_PORT = 52282;

typedef int16_t PCM_TYPE;
typedef std::pair<std::string, unsigned int> AudioDeviceName;
typedef std::array<unsigned int, 2> AudioChannelMap;

/**
 * @typedef AudioInputCallBack
 * @brief Function prototype for audio input callbacks
 *
 * @param data Pointer to PCM audio data
 * @param chan_num Number of channels
 * @param frame_num Number of frames
 * @param user_ptr User data pointer passed during callback registration
 */
typedef void (*AudioInputCallBack)(const int16_t *data, unsigned int chan_num, unsigned int frame_num, void *user_ptr);

/**
 * @enum UsrCallBackMode
 * @brief Callback modes for audio input processing
 * RAW: Raw data from real device
 * PROCESSED: Processed audio data from real device
 * OBSERVER: Raw Audio data from listener
 */
enum class UsrCallBackMode
{
    RAW,
    PROCESSED,
    OBSERVER
};

/**
 * @struct RetCode
 * @brief Return code for audio operations with error information
 */
struct RetCode
{
    /**
     * @enum Code
     * @brief Enumeration of return code values
     */
    enum Code
    {
        FAILED = -11, /**< Operation failed */
        EOPEN,        /**< Error opening device */
        NOPEN,        /**< Device not opened */
        INVFMT,       /**< Invalid format */
        EWRITE,       /**< Write error */
        EREAD,        /**< Read error */
        INVSEEK,      /**< Invalid seek */
        EPARAM,       /**< Invalid parameter */
        EXCEPTION,    /**< Exception occurred */
        ESTATE,       /**< Invalid state */
        ETIMEOUT,     /**< Operation timeout */
        OK = 0,       /**< Success */
        NOACTION      /**< No action performed */
    };

    Code err;        /**< The error code */
    const char *msg; /**< Error message */

    /**
     * @brief Construct a RetCode with a code and default message
     * @param e Error code
     */
    constexpr RetCode(Code e) : err(e), msg(get_default_message(e))
    {
    }

    /**
     * @brief Construct a RetCode with a code and custom message
     * @param e Error code
     * @param literal Custom error message
     */
    template <size_t N> constexpr RetCode(Code e, const char (&literal)[N]) : err(e), msg(literal)
    {
    }

    /**
     * @brief Compare with error code
     * @param code Error code to compare with
     * @return true if equal
     */
    constexpr bool operator==(Code code) const
    {
        return err == code;
    }

    /**
     * @brief Compare with error code
     * @param code Error code to compare with
     * @return true if not equal
     */
    constexpr bool operator!=(Code code) const
    {
        return err != code;
    }

    /**
     * @brief Compare with another RetCode
     * @param other RetCode to compare with
     * @return true if equal
     */
    constexpr bool operator==(const RetCode &other) const
    {
        return err == other.err;
    }

    /**
     * @brief Compare with another RetCode
     * @param other RetCode to compare with
     * @return true if not equal
     */
    constexpr bool operator!=(const RetCode &other) const
    {
        return err != other.err;
    }

    /**
     * @brief Boolean conversion operator
     * @return true if code is OK
     */
    constexpr explicit operator bool() const
    {
        return err == OK;
    }

    /**
     * @brief Get error message
     * @return Error message string
     */
    constexpr const char *what() const
    {
        return msg;
    }

  private:
    /**
     * @brief Get default error message for a code
     * @param e Error code
     * @return Default message for the code
     */
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
        case ETIMEOUT:
            return "Operation timeout";
        default:
            return "Unknown error";
        }
    }
};

/**
 * @enum AudioBandWidth
 * @brief Audio bandwidth (sample rate) options
 */
enum class AudioBandWidth : unsigned int
{
    Unknown = 0,           /**< Unknown bandwidth */
    Narrow = 8000,         /**< Narrow band (8 kHz) */
    Wide = 16000,          /**< Wide band (16 kHz) */
    SemiSuperWide = 24000, /**< Semi-super-wide band (24 kHz) */
    CDQuality = 44100,     /**< CD quality (44.1 kHz) */
    Full = 48000           /**< Full band (48 kHz) */
};

/**
 * @enum AudioPeriodSize
 * @brief Audio period size options
 */
enum class AudioPeriodSize : unsigned int
{
    INR_05MS = 0x05, /**< 5ms period size */
    INR_10MS = 0x0a, /**< 10ms period size */
    INR_20MS = 0x14, /**< 20ms period size */
    INR_40MS = 0x28  /**< 40ms period size */
};

/**
 * @struct AudioToken
 * @brief Base class for audio tokens
 */
struct AudioToken
{
    /**
     * @brief Token value representing an invalid token
     */
    static constexpr unsigned char INVALID_TOKEN = 0xFF;

    unsigned char tok; /**< Token value */

    /**
     * @brief Default constructor creates an invalid token
     */
    constexpr AudioToken() noexcept : tok(INVALID_TOKEN)
    {
    }

    /**
     * @brief Construct a token with a specific value
     * @param t Token value
     */
    constexpr explicit AudioToken(unsigned char t) noexcept : tok(t)
    {
    }

    /**
     * @brief Convert to unsigned char
     * @return Token value as unsigned char
     */
    constexpr explicit operator bool() const noexcept
    {
        return tok != INVALID_TOKEN;
    }

    /**
     * @brief Compare with another token
     * @param other Token to compare with
     * @return true if equal
     */
    constexpr bool operator==(const AudioToken &other) const noexcept
    {
        return tok == other.tok;
    }

    /**
     * @brief Compare with another token
     * @param other Token to compare with
     * @return true if not equal
     */
    constexpr bool operator!=(const AudioToken &other) const noexcept
    {
        return tok != other.tok;
    }
};

/**
 * @struct IToken
 * @brief Input audio token (first half of token space)
 */
struct IToken : public AudioToken
{
    /**
     * @brief Default constructor creates an invalid input token
     */
    constexpr IToken() noexcept : AudioToken(INVALID_TOKEN)
    {
    }

    /**
     * @brief Construct an input token with a specific value
     * @param t Token value (must be <= USER_MAX_AUDIO_TOKEN/2)
     */
    constexpr explicit IToken(unsigned char t) noexcept : AudioToken(t <= USER_MAX_AUDIO_TOKEN / 2 ? t : INVALID_TOKEN)
    {
    }

    /**
     * @brief Construct an input token from an AudioToken
     * @param t AudioToken to convert
     * @note This constructor allows conversion from AudioToken to IToken
     *       if the value is within the invalid range for input tokens.
     */
    constexpr explicit IToken(AudioToken t) noexcept : AudioToken(t)
    {
    }

    using AudioToken::operator==;
    using AudioToken::operator!=;
};

/**
 * @struct OToken
 * @brief Output audio token (second half of token space)
 */
struct OToken : public AudioToken
{
    /**
     * @brief Default constructor creates an invalid output token
     */
    constexpr OToken() noexcept : AudioToken(INVALID_TOKEN)
    {
    }

    /**
     * @brief Construct an output token with a specific value
     * @param t Token value (must be > USER_MAX_AUDIO_TOKEN/2 and < USER_MAX_AUDIO_TOKEN)
     */
    constexpr explicit OToken(unsigned char t) noexcept
        : AudioToken(t < USER_MAX_AUDIO_TOKEN ? (t > USER_MAX_AUDIO_TOKEN / 2 ? t : INVALID_TOKEN) : INVALID_TOKEN)
    {
    }

    /**
     * @brief Construct an output token from an AudioToken
     * @param t AudioToken to convert
     * @note This constructor allows conversion from AudioToken to OToken
     *       if the value is within the invalid range for input tokens.
     */
    constexpr explicit OToken(AudioToken t) noexcept : AudioToken(t)
    {
    }

    using AudioToken::operator==;
    using AudioToken::operator!=;
};

/**
 * @brief User-defined literal for creating input tokens
 * @param val Numeric token value
 * @return Input token
 */
constexpr IToken operator""_itk(unsigned long long val) noexcept
{
    return IToken(static_cast<unsigned char>(val));
}

/**
 * @brief User-defined literal for creating output tokens
 * @param val Numeric token value
 * @return Output token
 */
constexpr OToken operator""_otk(unsigned long long val) noexcept
{
    return OToken(static_cast<unsigned char>(val));
}

/**
 * @brief Constant for dummy input token
 */
constexpr auto USR_DUMMY_IN = 100_itk;

/**
 * @brief Constant for dummy output token
 */
constexpr auto USR_DUMMY_OUT = 200_otk;

/**
 * @brief Token for audio player
 */
constexpr auto WAVE_PLAYER_TOKEN = IToken(AudioToken(USER_MAX_AUDIO_TOKEN));

// Forward declarations
class IAStream;
class OAStream;
class NetWorker;
class AudioPlayer;
class AudioMonitor;

/**
 * @class AudioCenter
 * @brief Main class for managing audio streams, connections, and playback
 *
 * AudioCenter is the primary interface for the AudioDriver library, providing
 * methods to create, connect, and manage audio streams for both input and output.
 * It also supports network audio transmission and audio file playback.
 */
class AudioCenter
{
  public:
    /**
     * @brief Constructor for AudioCenter
     * @param enable_network Whether to enable network functionality
     * @param port Network port to use (ignored if enable_network is false)
     * @param local_ip Local IP address to use for network identification (empty for default)
     */
    AudioCenter(bool enable_network = false, unsigned short port = NETWORK_AUDIO_TRANS_PORT,
                const std::string &local_ip = "");

    /**
     * @brief Destructor for AudioCenter
     */
    ~AudioCenter();

    /**
     * @brief Creates an input stream
     * @param token Input token to associate with the stream
     * @param name Audio device name and index
     * @param bw Audio bandwidth (sample rate)
     * @param ps Audio period size
     * @param ch Number of channels
     * @param enable_network Whether to enable network functionality for this stream
     * @param enable_reset Whether to enable auto reset for this stream
     * @return RetCode indicating success or failure
     */
    RetCode create(IToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps, unsigned int ch,
                   bool enable_network = false, bool enable_reset = false);

    /**
     * @brief Creates an input stream with custom channel mapping
     * @param token Input token to associate with the stream
     * @param name Audio device name and index
     * @param bw Audio bandwidth (sample rate)
     * @param ps Audio period size
     * @param dev_ch Number of channels to open device
     * @param imap Channel mapping between user channel and device channel
     * @param enable_network Whether to enable network functionality for this stream
     * @param enable_reset Whether to enable auto reset for this stream
     * @return RetCode indicating success or failure
     */
    RetCode create(IToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                   unsigned int dev_ch, const AudioChannelMap &imap, bool enable_network = false);

    /**
     * @brief Creates an output stream
     * @param token Output token to associate with the stream
     * @param name Audio device name and index
     * @param bw Audio bandwidth (sample rate)
     * @param ps Audio period size
     * @param ch Number of channels
     * @param enable_network Whether to enable network functionality for this stream
     * @param enable_reset Whether to enable auto reset for this stream
     * @return RetCode indicating success or failure
     */
    RetCode create(OToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps, unsigned int ch,
                   bool enable_network = false, bool enable_reset = false);

    /**
     * @brief Creates an output stream with custom channel mapping
     * @param token Output token to associate with the stream
     * @param name Audio device name and index
     * @param bw Audio bandwidth (sample rate)
     * @param ps Audio period size
     * @param dev_ch Number of channels to open device
     * @param omap Channel mapping between user channel and device channel
     * @param enable_network Whether to enable network functionality for this stream
     * @param enable_reset Whether to enable auto reset for this stream
     * @return RetCode indicating success or failure
     */
    RetCode create(OToken token, const AudioDeviceName &name, AudioBandWidth bw, AudioPeriodSize ps,
                   unsigned int dev_ch, const AudioChannelMap &omap, bool enable_network = false);

    /**
     * @brief Creates a link between input and output streams
     * @param itoken Input token
     * @param otoken Output token
     * @param enable_network Whether to enable network functionality for this link
     * @return RetCode indicating success or failure
     */
    RetCode create(IToken itoken, OToken otoken, bool enable_network = false);

    /**
     * @brief Prepares the AudioCenter for use
     *
     * Must be called after creating streams and before connecting them.
     * This initializes internal components and transitions the AudioCenter to the CONNECTING state.
     * @param enable_usb_detection Whether to enable USB detection
     * @return RetCode indicating success or failure
     */
    RetCode prepare(bool enable_usb_detection = true);

    /**
     * @brief Connects an input stream to an output stream
     *
     * @param itoken Input token
     * @param otoken Output token
     * @param ip IP address for network connection (empty string for local)
     * @param port Network port to use
     * @return RetCode indicating success or failure
     */
    RetCode connect(IToken itoken, OToken otoken, const std::string &ip = "",
                    unsigned short port = NETWORK_AUDIO_TRANS_PORT);

    /**
     * @brief Disconnects an input stream from an output stream
     *
     * @param itoken Input token
     * @param otoken Output token
     * @param ip IP address for network connection (empty string for local)
     * @param port Network port to use
     * @return RetCode indicating success or failure
     */
    RetCode disconnect(IToken itoken, OToken otoken, const std::string &ip = "",
                       unsigned short port = NETWORK_AUDIO_TRANS_PORT);

    /**
     * @brief Registers a callback function for an input stream
     *
     * The callback will be invoked when new audio data is available from the input stream.
     *
     * @param token Input token
     * @param cb Callback function
     * @param required_frames Number of frames required for the callback
     * @param mode decide callback mode, 0: raw, 1: processed, 2: from listener
     * @param ptr User data pointer to be passed to the callback function
     * @return RetCode indicating success or failure
     */
    RetCode register_callback(IToken token, AudioInputCallBack cb, unsigned int required_frames, UsrCallBackMode mode,
                              void *ptr);

    /**
     * @brief Directly pushes PCM data to an output stream
     *
     * @param itoken Input token (source)
     * @param otoken Output token (destination)
     * @param chan Number of channels in the data
     * @param frames Number of frames in the data
     * @param sample_rate Sample rate of the data
     * @param data Pointer to the PCM data
     * @return RetCode indicating success or failure
     */
    RetCode direct_push_pcm(IToken itoken, OToken otoken, unsigned int chan, unsigned int frames,
                            unsigned int sample_rate, const int16_t *data);

    /**
     * @brief Starts the AudioCenter and all associated streams
     *
     * Transitions the AudioCenter from CONNECTING to READY state.
     *
     * @return RetCode indicating success or failure
     */
    RetCode start();

    /**
     * @brief Stops the AudioCenter and all associated streams
     *
     * Transitions the AudioCenter back to INIT state.
     *
     * @return RetCode indicating success or failure
     */
    RetCode stop();

    /**
     * @brief Sets the volume for a stream
     *
     * @param token Audio token (can be either input or output)
     * @param vol Volume level (0-100)
     * @return RetCode indicating success or failure
     */
    RetCode set_volume(AudioToken token, unsigned int vol);

    /**
     * @brief Sets the volume for a specific session in an output stream
     *
     * @param otoken Output token
     * @param itoken Input token of the session to adjust
     * @param vol Volume level (0-100)
     * @param ip IP address of the session (empty string for all IPs with the given input token)
     * @return RetCode indicating success or failure
     */
    RetCode set_session_volume(OToken otoken, IToken itoken, unsigned int vol, const std::string &ip = "");

    /**
     * @brief Mutes or unmutes a stream
     *
     * @param token Audio token (can be either input or output)
     * @param enable true to mute, false to unmute
     * @return RetCode indicating success or failure
     */
    RetCode mute(AudioToken token, bool enable);

    /**
     * @brief Mutes or unmutes a specific connection between input and output streams
     *
     * @param itoken Input token
     * @param otoken Output token
     * @param enable true to mute, false to unmute
     * @param ip IP address for network connection (empty string for local)
     * @return RetCode indicating success or failure
     */
    RetCode mute(IToken itoken, OToken otoken, bool enable, const std::string &ip = "");

    /**
     * @brief Plays an audio file through an output stream
     *
     * @param name Path to the audio file
     * @param cycles Number of times to play the file (0 = infinite)
     * @param otoken Output token
     * @param ip IP address for network playback (empty string for local)
     * @param port Network port to use
     * @return RetCode indicating success or failure
     */
    RetCode play(const std::string &name, int cycles, OToken otoken, const std::string &ip = "",
                 unsigned short port = NETWORK_AUDIO_TRANS_PORT);

    /**
     * @brief Stops playing an audio file
     *
     * @param path Path to the audio file to stop
     * @return RetCode indicating success or failure
     */
    RetCode stop(const std::string &path);

    /**
     * @brief Sets the global volume for audio file playback
     *
     * This affects all currently playing and future audio files.
     *
     * @param vol Volume level (0-100)
     * @return RetCode indicating success or failure
     */
    RetCode set_player_volume(unsigned int vol);

  private:
    /**
     * @brief Schedules the regular connection report timer
     */
    void schedule_report_timer() const;

    /**
     * @brief Reports active connections
     *
     * Generates and prints a table showing the current connections between
     * input and output streams.
     */
    void report_connections() const;

  private:
    /**
     * @enum State
     * @brief State of the AudioCenter
     */
    enum class State
    {
        INIT,       /**< Initialized state */
        CONNECTING, /**< Streams are being connected */
        READY       /**< Ready for audio processing */
    };

    std::atomic<State> center_state;                            /**< Current state of the AudioCenter */
    std::map<unsigned char, std::shared_ptr<IAStream>> ias_map; /**< Map of input streams */
    std::map<unsigned char, std::shared_ptr<OAStream>> oas_map; /**< Map of output streams */

    std::shared_ptr<NetWorker> net_mgr;    /**< Network manager */
    std::unique_ptr<AudioMonitor> monitor; /**< Audio device monitor */
    std::shared_ptr<AudioPlayer> player;   /**< Audio player */
};

#endif