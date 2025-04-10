/**
 * @file audio_interface_c.cpp
 * @brief Implementation of C interface for AudioDriver library
 *
 * This file contains the implementation of C API functions that
 * wrap the C++ AudioDriver library functionality.
 */

#include "audio_interface_c.h"
#include "audio_interface.h"
#include <cstring>
#include <string>

/**
 * @brief Structure definition for the opaque audio_center_handle
 */
struct audio_center_handle
{
    AudioCenter *center; /**< Pointer to the C++ AudioCenter instance */
};

// C接口与C++接口之间的转换函数

/**
 * @brief Convert C++ RetCode to C audio_ret_code_t
 * @param rc C++ RetCode to convert
 * @return Equivalent C audio_ret_code_t
 */
static audio_ret_code_t convert_retcode(const RetCode &rc)
{
    return static_cast<audio_ret_code_t>(rc.err);
}

/**
 * @brief Convert C audio_bandwidth_t to C++ AudioBandWidth
 * @param bw C audio_bandwidth_t to convert
 * @return Equivalent C++ AudioBandWidth
 */
static AudioBandWidth convert_bandwidth(audio_bandwidth_t bw)
{
    switch (bw)
    {
    case AUDIO_BW_UNKNOWN:
        return AudioBandWidth::Unknown;
    case AUDIO_BW_NARROW:
        return AudioBandWidth::Narrow;
    case AUDIO_BW_WIDE:
        return AudioBandWidth::Wide;
    case AUDIO_BW_SEMI_SUPER_WIDE:
        return AudioBandWidth::SemiSuperWide;
    case AUDIO_BW_CD_QUALITY:
        return AudioBandWidth::CDQuality;
    case AUDIO_BW_FULL:
        return AudioBandWidth::Full;
    default:
        return AudioBandWidth::Unknown;
    }
}

/**
 * @brief Convert C audio_period_size_t to C++ AudioPeriodSize
 * @param ps C audio_period_size_t to convert
 * @return Equivalent C++ AudioPeriodSize
 */
static AudioPeriodSize convert_period_size(audio_period_size_t ps)
{
    switch (ps)
    {
    case AUDIO_PS_05MS:
        return AudioPeriodSize::INR_05MS;
    case AUDIO_PS_10MS:
        return AudioPeriodSize::INR_10MS;
    case AUDIO_PS_20MS:
        return AudioPeriodSize::INR_20MS;
    case AUDIO_PS_40MS:
        return AudioPeriodSize::INR_40MS;
    default:
        return AudioPeriodSize::INR_20MS;
    }
}

/**
 * @brief Convert C audio_device_name_t to C++ AudioDeviceName
 * @param name Pointer to C audio_device_name_t to convert
 * @return Equivalent C++ AudioDeviceName
 */
static AudioDeviceName convert_device_name(const audio_device_name_t *name)
{
    return AudioDeviceName(name->device_id, name->device_index);
}

/**
 * @brief Convert C input_token_t to C++ IToken
 * @param token C input_token_t to convert
 * @return Equivalent C++ IToken
 */
static IToken convert_input_token(input_token_t token)
{
    return IToken(token);
}

/**
 * @brief Convert C output_token_t to C++ OToken
 * @param token C output_token_t to convert
 * @return Equivalent C++ OToken
 */
static OToken convert_output_token(output_token_t token)
{
    return OToken(token);
}

/**
 * @brief Convert C audio_token_t to C++ AudioToken
 * @param token C audio_token_t to convert
 * @return Equivalent C++ AudioToken
 */
static AudioToken convert_audio_token(audio_token_t token)
{
    return AudioToken(token);
}

// C API 实现

extern "C"
{

    audio_center_t audio_center_create(bool enable_network, uint16_t port)
    {
        audio_center_handle *handle = new audio_center_handle();
        handle->center = new AudioCenter(enable_network, port);
        return handle;
    }

    void audio_center_destroy(audio_center_t center)
    {
        if (center)
        {
            delete center->center;
            delete center;
        }
    }

    audio_ret_code_t audio_center_create_input_stream(audio_center_t center, input_token_t token,
                                                      const audio_device_name_t *name, audio_bandwidth_t bw,
                                                      audio_period_size_t ps, unsigned int ch, bool enable_network,
                                                      bool enable_reset)
    {
        if (!center || !name)
        {
            return AUDIO_RC_EPARAM;
        }

        RetCode rc =
            center->center->create(convert_input_token(token), convert_device_name(name), convert_bandwidth(bw),
                                   convert_period_size(ps), ch, enable_network, enable_reset);

        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_create_output_stream(audio_center_t center, output_token_t token,
                                                       const audio_device_name_t *name, audio_bandwidth_t bw,
                                                       audio_period_size_t ps, unsigned int ch, bool enable_network,
                                                       bool enable_reset)
    {
        if (!center || !name)
        {
            return AUDIO_RC_EPARAM;
        }

        RetCode rc =
            center->center->create(convert_output_token(token), convert_device_name(name), convert_bandwidth(bw),
                                   convert_period_size(ps), ch, enable_network, enable_reset);

        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_create_io_link(audio_center_t center, input_token_t itoken, output_token_t otoken,
                                                 bool enable_network)
    {
        if (!center)
        {
            return AUDIO_RC_EPARAM;
        }

        RetCode rc = center->center->create(convert_input_token(itoken), convert_output_token(otoken), enable_network);

        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_prepare(audio_center_t center)
    {
        if (!center)
        {
            return AUDIO_RC_EPARAM;
        }

        RetCode rc = center->center->prepare();
        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_connect(audio_center_t center, input_token_t itoken, output_token_t otoken,
                                          const char *ip, uint16_t port)
    {
        if (!center)
        {
            return AUDIO_RC_EPARAM;
        }

        std::string ip_str = ip ? ip : "";
        RetCode rc = center->center->connect(convert_input_token(itoken), convert_output_token(otoken), ip_str, port);

        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_disconnect(audio_center_t center, input_token_t itoken, output_token_t otoken,
                                             const char *ip, uint16_t port)
    {
        if (!center)
        {
            return AUDIO_RC_EPARAM;
        }

        std::string ip_str = ip ? ip : "";
        RetCode rc =
            center->center->disconnect(convert_input_token(itoken), convert_output_token(otoken), ip_str, port);

        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_register_callback(audio_center_t center, input_token_t token,
                                                    audio_input_callback_t cb, void *ptr)
    {
        if (!center)
        {
            return AUDIO_RC_EPARAM;
        }

        RetCode rc = center->center->register_callback(convert_input_token(token),
                                                       reinterpret_cast<AudioInputCallBack>(cb), ptr);

        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_direct_push_pcm(audio_center_t center, input_token_t itoken, output_token_t otoken,
                                                  unsigned int chan, unsigned int frames, unsigned int sample_rate,
                                                  const int16_t *data)
    {
        if (!center || !data)
        {
            return AUDIO_RC_EPARAM;
        }

        RetCode rc = center->center->direct_push_pcm(convert_input_token(itoken), convert_output_token(otoken), chan,
                                                     frames, sample_rate, data);

        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_start(audio_center_t center)
    {
        if (!center)
        {
            return AUDIO_RC_EPARAM;
        }

        RetCode rc = center->center->start();
        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_stop(audio_center_t center)
    {
        if (!center)
        {
            return AUDIO_RC_EPARAM;
        }

        RetCode rc = center->center->stop();
        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_set_volume(audio_center_t center, audio_token_t token, unsigned int vol)
    {
        if (!center)
        {
            return AUDIO_RC_EPARAM;
        }

        RetCode rc = center->center->set_volume(convert_audio_token(token), vol);
        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_mute(audio_center_t center, audio_token_t token, bool enable)
    {
        if (!center)
        {
            return AUDIO_RC_EPARAM;
        }

        RetCode rc = center->center->mute(convert_audio_token(token), enable);
        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_mute_connection(audio_center_t center, input_token_t itoken, output_token_t otoken,
                                                  bool enable, const char *ip)
    {
        if (!center)
        {
            return AUDIO_RC_EPARAM;
        }

        std::string ip_str = ip ? ip : "";
        RetCode rc = center->center->mute(convert_input_token(itoken), convert_output_token(otoken), enable, ip_str);

        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_play(audio_center_t center, const char *name, int cycles, output_token_t otoken,
                                       const char *ip, uint16_t port)
    {
        if (!center || !name)
        {
            return AUDIO_RC_EPARAM;
        }

        std::string name_str(name);
        std::string ip_str = ip ? ip : "";
        RetCode rc = center->center->play(name_str, cycles, convert_output_token(otoken), ip_str, port);

        return convert_retcode(rc);
    }

    audio_ret_code_t audio_center_stop_play(audio_center_t center, const char *path)
    {
        if (!center || !path)
        {
            return AUDIO_RC_EPARAM;
        }

        std::string path_str(path);
        RetCode rc = center->center->stop(path_str);
        return convert_retcode(rc);
    }

    const char *audio_get_error_message(audio_ret_code_t code)
    {
        RetCode::Code cpp_code = static_cast<RetCode::Code>(code);
        RetCode rc(cpp_code);
        return rc.what();
    }

} // extern "C"