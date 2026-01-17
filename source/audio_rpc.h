#ifndef AUDIO_RPC_H
#define AUDIO_RPC_H

#include "asio.hpp"
#include "audio_interface.h"
#include <functional>
#include <mutex>

enum class RPCCommand : uint16_t
{
    CONNECT = 1,
    DISCONNECT,
    SET_VOLUME,
    MUTE,
    PLAY,
    STOP_PLAY,
    SET_PLAYER_VOLUME,
    RESPONSE
};

struct RPCRequest
{
    struct Header
    {
        uint16_t command;
        uint16_t sequence_id;
        uint32_t payload_size;
        uint8_t checksum;
    };

    Header header;
    std::vector<uint8_t> payload;
};

struct RPCResponse
{
    uint16_t sequence_id;
    int32_t result_code;
    std::string message;
};

class RPCService : public std::enable_shared_from_this<RPCService>
{
  public:
    using RequestHandler = std::function<RetCode(const std::vector<uint8_t> &)>;

    RPCService(asio::io_context &io_context, unsigned short port);
    ~RPCService();

    RetCode start();
    RetCode stop();
    void register_handler(RPCCommand cmd, RequestHandler handler);

  private:
    class Session;

    void start_accept();
    void handle_request(std::shared_ptr<Session> session, const RPCRequest &request);
    bool can_accept_connection() const;
    void cleanup_dead_sessions();

    asio::io_context &io_context_;
    asio::ip::tcp::acceptor acceptor_;
    std::atomic_bool running_;
    std::map<RPCCommand, RequestHandler> handlers_; // Protected by handler_strand_
    std::mutex sessions_mutex_;
    std::vector<std::weak_ptr<Session>> active_sessions_;
    asio::strand<asio::io_context::executor_type> handler_strand_;
};

class RPCClient
{
  public:
    RPCClient(asio::io_context &io_context, const std::string &server_ip, unsigned short server_port);
    ~RPCClient();

    RetCode connect_stream(IToken itoken, OToken otoken, const std::string &ip, unsigned short port);
    RetCode disconnect_stream(IToken itoken, OToken otoken, const std::string &ip, unsigned short port);
    RetCode set_volume(AudioToken token, unsigned int volume);
    RetCode mute(AudioToken token, bool enable);
    RetCode mute(IToken itoken, OToken otoken, bool enable, const std::string &ip);
    RetCode play(const std::string &name, int cycles, OToken otoken, AudioPriority priority = AudioPriority::MEDIUM);
    RetCode stop_play(const std::string &path);
    RetCode set_player_volume(unsigned int volume);

  private:
    RetCode send_request(RPCCommand cmd, const std::vector<uint8_t> &payload);

    template <typename T> void append_to_payload(std::vector<uint8_t> &payload, const T &value)
    {
        const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&value);
        payload.insert(payload.end(), ptr, ptr + sizeof(T));
    }

    asio::io_context &io_context_;
    asio::ip::tcp::socket socket_;
    std::string server_ip_;
    unsigned short server_port_;
    std::atomic<uint16_t> sequence_id_;
    std::mutex socket_mutex_;
};

#endif
