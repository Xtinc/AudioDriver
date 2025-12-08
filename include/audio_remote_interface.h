#ifndef AUDIO_REMOTE_INTERFACE_H
#define AUDIO_REMOTE_INTERFACE_H

#include "audio_interface.h"

/**
 * @brief Audio remote control client
 *
 * Provides network-based remote control of the audio system using Pimpl idiom to hide implementation details.
 * Only depends on standard library and basic types, no ASIO dependencies exposed.
 */
class AudioRemoteClient
{
  public:
    /**
     * @brief Constructor
     * @param server_ip Server IP address
     * @param server_port Server port number
     */
    AudioRemoteClient(const std::string &server_ip, unsigned short server_port = NETWORK_AUDIO_SERVICE_PORT);

    /**
     * @brief Destructor
     */
    ~AudioRemoteClient();

    // Disable copy
    AudioRemoteClient(const AudioRemoteClient &) = delete;
    AudioRemoteClient &operator=(const AudioRemoteClient &) = delete;

    /**
     * @brief Connect audio stream to remote endpoint
     * @param itoken Input stream token
     * @param otoken Output stream token
     * @param ip Remote IP address
     * @param port Remote port number
     * @return RetCode Operation result
     */
    RetCode connect_stream(IToken itoken, OToken otoken, const std::string &ip = "",
                           unsigned short port = NETWORK_AUDIO_TRANS_PORT);

    /**
     * @brief Disconnect audio stream connection
     * @param itoken Input stream token
     * @param otoken Output stream token
     * @param ip Remote IP address
     * @param port Remote port number
     * @return RetCode Operation result
     */
    RetCode disconnect_stream(IToken itoken, OToken otoken, const std::string &ip = "",
                              unsigned short port = NETWORK_AUDIO_TRANS_PORT);

    /**
     * @brief Set audio stream volume
     * @param token Audio stream token
     * @param volume Volume value (0-100)
     * @return RetCode Operation result
     */
    RetCode set_volume(AudioToken token, unsigned int volume);

    /**
     * @brief Mute/unmute single audio stream
     * @param token Audio stream token
     * @param enable true=mute, false=unmute
     * @return RetCode Operation result
     */
    RetCode mute(AudioToken token, bool enable);

    /**
     * @brief Mute/unmute specific audio connection
     * @param itoken Input stream token
     * @param otoken Output stream token
     * @param enable true=mute, false=unmute
     * @param ip Remote IP address (optional, empty means all IPs)
     * @return RetCode Operation result
     */
    RetCode mute(IToken itoken, OToken otoken, bool enable, const std::string &ip = "");

    /**
     * @brief Play audio file to specified output stream
     * @param name Audio file path
     * @param cycles Play count (-1 for loop, 0 for once)
     * @param otoken Output stream token
     * @return RetCode Operation result
     */
    RetCode play(const std::string &name, int cycles, OToken otoken);

    /**
     * @brief Stop playing specified audio file
     * @param path Audio file path
     * @return RetCode Operation result
     */
    RetCode stop_play(const std::string &path);

    /**
     * @brief Set player global volume
     * @param volume Volume value (0-100)
     * @return RetCode Operation result
     */
    RetCode set_player_volume(unsigned int volume);

  private:
    std::unique_ptr<RPCClient> rpc_client;
};

#endif