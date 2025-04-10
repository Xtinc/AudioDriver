#ifndef AUDIO_INTERFACE_C_H
#define AUDIO_INTERFACE_C_H

#ifdef __cplusplus
extern "C"
{
#endif

#define USR_DUMMY_IN_TOKEN 100
#define USR_DUMMY_OUT_TOKEN 200

#include <stdbool.h>
#include <stdint.h>
    typedef unsigned char audio_token_t;
    typedef audio_token_t input_token_t;
    typedef audio_token_t output_token_t;

    /**
     * @enum audio_ret_code_t
     * @brief Return codes for audio functions
     */
    typedef enum
    {
        AUDIO_RC_FAILED = -11, /**< Operation failed */
        AUDIO_RC_EOPEN,        /**< Error opening device */
        AUDIO_RC_NOPEN,        /**< Device not opened */
        AUDIO_RC_INVFMT,       /**< Invalid format */
        AUDIO_RC_EWRITE,       /**< Write error */
        AUDIO_RC_EREAD,        /**< Read error */
        AUDIO_RC_INVSEEK,      /**< Invalid seek */
        AUDIO_RC_EPARAM,       /**< Invalid parameter */
        AUDIO_RC_EXCEPTION,    /**< Exception occurred */
        AUDIO_RC_ESTATE,       /**< Invalid state */
        AUDIO_RC_ETIMEOUT,     /**< Operation timeout */
        AUDIO_RC_OK = 0,       /**< Success */
        AUDIO_RC_NOACTION      /**< No action performed */
    } audio_ret_code_t;

    /**
     * @enum audio_bandwidth_t
     * @brief Audio bandwidth (sample rate) options
     */
    typedef enum
    {
        AUDIO_BW_UNKNOWN = 0,             /**< Unknown bandwidth */
        AUDIO_BW_NARROW = 8000,           /**< Narrow band (8 kHz) */
        AUDIO_BW_WIDE = 16000,            /**< Wide band (16 kHz) */
        AUDIO_BW_SEMI_SUPER_WIDE = 24000, /**< Semi-super-wide band (24 kHz) */
        AUDIO_BW_CD_QUALITY = 44100,      /**< CD quality (44.1 kHz) */
        AUDIO_BW_FULL = 48000             /**< Full band (48 kHz) */
    } audio_bandwidth_t;

    /**
     * @enum audio_period_size_t
     * @brief Audio period size options
     */
    typedef enum
    {
        AUDIO_PS_05MS = 0x05, /**< 5ms period size */
        AUDIO_PS_10MS = 0x0a, /**< 10ms period size */
        AUDIO_PS_20MS = 0x14, /**< 20ms period size */
        AUDIO_PS_40MS = 0x28  /**< 40ms period size */
    } audio_period_size_t;

    /**
     * @typedef audio_input_callback_t
     * @brief Callback function for audio input
     *
     * @param data Pointer to PCM audio data
     * @param chan_num Number of channels
     * @param frame_num Number of frames
     * @param user_ptr User data pointer passed during callback registration
     */
    typedef void (*audio_input_callback_t)(const int16_t *data, unsigned int chan_num, unsigned int frame_num,
                                           void *user_ptr);

    /**
     * @struct audio_device_name_t
     * @brief Structure representing an audio device name
     */
    typedef struct
    {
        char device_id[256];       /**< Device ID or name */
        unsigned int device_index; /**< Device index */
    } audio_device_name_t;

    /**
     * @typedef audio_center_t
     * @brief Opaque handle to an AudioCenter instance
     */
    typedef struct audio_center_handle *audio_center_t;

    /*
     * AudioCenter API
     */

    /**
     * @brief Creates a new AudioCenter instance
     *
     * @param enable_network Whether to enable network functionality
     * @param port Network port to use (ignored if enable_network is false)
     * @return Handle to the created AudioCenter instance
     */
    audio_center_t audio_center_create(bool enable_network, uint16_t port);

    /**
     * @brief Destroys an AudioCenter instance and frees associated resources
     *
     * @param center Handle to the AudioCenter instance to destroy
     */
    void audio_center_destroy(audio_center_t center);

    /**
     * @brief Creates an input stream
     *
     * @param center Handle to the AudioCenter instance
     * @param token Input token to associate with the stream
     * @param name Audio device name and index
     * @param bw Audio bandwidth (sample rate)
     * @param ps Audio period size
     * @param ch Number of channels
     * @param enable_network Whether to enable network functionality for this stream
     * @param enable_reset Whether to enable auto reset for this stream
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_create_input_stream(audio_center_t center, input_token_t token,
                                                      const audio_device_name_t *name, audio_bandwidth_t bw,
                                                      audio_period_size_t ps, unsigned int ch, bool enable_network,
                                                      bool enable_reset);

    /**
     * @brief Creates an output stream
     *
     * @param center Handle to the AudioCenter instance
     * @param token Output token to associate with the stream
     * @param name Audio device name and index
     * @param bw Audio bandwidth (sample rate)
     * @param ps Audio period size
     * @param ch Number of channels
     * @param enable_network Whether to enable network functionality for this stream
     * @param enable_reset Whether to enable auto reset for this stream
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_create_output_stream(audio_center_t center, output_token_t token,
                                                       const audio_device_name_t *name, audio_bandwidth_t bw,
                                                       audio_period_size_t ps, unsigned int ch, bool enable_network,
                                                       bool enable_reset);

    /**
     * @brief Creates a link between input and output streams
     *
     * @param center Handle to the AudioCenter instance
     * @param itoken Input token
     * @param otoken Output token
     * @param enable_network Whether to enable network functionality for this link
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_create_io_link(audio_center_t center, input_token_t itoken, output_token_t otoken,
                                                 bool enable_network);

    /**
     * @brief Prepares the AudioCenter for use
     *
     * Must be called after creating streams and before connecting them
     *
     * @param center Handle to the AudioCenter instance
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_prepare(audio_center_t center);

    /**
     * @brief Connects an input stream to an output stream
     *
     * @param center Handle to the AudioCenter instance
     * @param itoken Input token
     * @param otoken Output token
     * @param ip IP address for network connection (empty string for local)
     * @param port Network port to use
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_connect(audio_center_t center, input_token_t itoken, output_token_t otoken,
                                          const char *ip, uint16_t port);

    /**
     * @brief Disconnects an input stream from an output stream
     *
     * @param center Handle to the AudioCenter instance
     * @param itoken Input token
     * @param otoken Output token
     * @param ip IP address for network connection (empty string for local)
     * @param port Network port to use
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_disconnect(audio_center_t center, input_token_t itoken, output_token_t otoken,
                                             const char *ip, uint16_t port);

    /**
     * @brief Registers a callback function for an input stream
     *
     * @param center Handle to the AudioCenter instance
     * @param token Input token
     * @param cb Callback function
     * @param ptr User data pointer to be passed to the callback function
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_register_callback(audio_center_t center, input_token_t token,
                                                    audio_input_callback_t cb, void *ptr);

    /**
     * @brief Directly pushes PCM data to an output stream
     *
     * @param center Handle to the AudioCenter instance
     * @param itoken Input token (source)
     * @param otoken Output token (destination)
     * @param chan Number of channels in the data
     * @param frames Number of frames in the data
     * @param sample_rate Sample rate of the data
     * @param data Pointer to the PCM data
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_direct_push_pcm(audio_center_t center, input_token_t itoken, output_token_t otoken,
                                                  unsigned int chan, unsigned int frames, unsigned int sample_rate,
                                                  const int16_t *data);

    /**
     * @brief Starts the AudioCenter and all associated streams
     *
     * @param center Handle to the AudioCenter instance
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_start(audio_center_t center);

    /**
     * @brief Stops the AudioCenter and all associated streams
     *
     * @param center Handle to the AudioCenter instance
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_stop(audio_center_t center);

    /**
     * @brief Sets the volume for a stream
     *
     * @param center Handle to the AudioCenter instance
     * @param token Audio token (can be either input or output)
     * @param vol Volume level (0-100)
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_set_volume(audio_center_t center, audio_token_t token, unsigned int vol);

    /**
     * @brief Mutes or unmutes a stream
     *
     * @param center Handle to the AudioCenter instance
     * @param token Audio token (can be either input or output)
     * @param enable true to mute, false to unmute
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_mute(audio_center_t center, audio_token_t token, bool enable);

    /**
     * @brief Mutes or unmutes a specific connection between input and output streams
     *
     * @param center Handle to the AudioCenter instance
     * @param itoken Input token
     * @param otoken Output token
     * @param enable true to mute, false to unmute
     * @param ip IP address for network connection (empty string for local)
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_mute_connection(audio_center_t center, input_token_t itoken, output_token_t otoken,
                                                  bool enable, const char *ip);

    /**
     * @brief Plays an audio file through an output stream
     *
     * @param center Handle to the AudioCenter instance
     * @param name Path to the audio file
     * @param cycles Number of times to play the file (0 = infinite)
     * @param otoken Output token
     * @param ip IP address for network playback (empty string for local)
     * @param port Network port to use
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_play(audio_center_t center, const char *name, int cycles, output_token_t otoken,
                                       const char *ip, uint16_t port);

    /**
     * @brief Stops playing an audio file
     *
     * @param center Handle to the AudioCenter instance
     * @param path Path to the audio file to stop
     * @return audio_ret_code_t Return code indicating success or failure
     */
    audio_ret_code_t audio_center_stop_play(audio_center_t center, const char *path);

    /**
     * @brief Gets a human-readable error message for a return code
     *
     * @param code The return code to get the message for
     * @return const char* Error message string
     */
    const char *audio_get_error_message(audio_ret_code_t code);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_INTERFACE_C_H */