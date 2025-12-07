#include "audio_rpc.h"
#include <algorithm>
#include <cstring>
#include <thread>

using asio::ip::tcp;
static constexpr size_t RPC_HEADER_SIZE = sizeof(RPCRequest::Header);
static constexpr size_t RPC_MAX_PAYLOAD_SIZE = 1024;
static constexpr size_t RPC_MAX_CONNECTIONS = 16;
static constexpr size_t RPC_MAX_IP_LENGTH = 256;
static constexpr size_t RPC_MAX_PATH_LENGTH = 512;

// Compute RPC checksum
static uint8_t compute_rpc_checksum(const RPCRequest::Header &header, const std::vector<uint8_t> &payload)
{
    uint8_t checksum = 0;

    // XOR command (2 bytes)
    checksum ^= static_cast<uint8_t>(header.command & 0xFF);
    checksum ^= static_cast<uint8_t>((header.command >> 8) & 0xFF);

    // XOR sequence_id (2 bytes)
    checksum ^= static_cast<uint8_t>(header.sequence_id & 0xFF);
    checksum ^= static_cast<uint8_t>((header.sequence_id >> 8) & 0xFF);

    // XOR payload_size (4 bytes)
    checksum ^= static_cast<uint8_t>(header.payload_size & 0xFF);
    checksum ^= static_cast<uint8_t>((header.payload_size >> 8) & 0xFF);
    checksum ^= static_cast<uint8_t>((header.payload_size >> 16) & 0xFF);
    checksum ^= static_cast<uint8_t>((header.payload_size >> 24) & 0xFF);

    // XOR payload
    for (const auto &byte : payload)
    {
        checksum ^= byte;
    }

    return checksum;
}

// Serialize RPC response
static std::vector<uint8_t> serialize_rpc_response(const RPCResponse &response)
{
    std::string safe_message = response.message;
    if (safe_message.size() > 256)
    {
        safe_message.resize(256);
    }

    std::vector<uint8_t> payload;
    payload.resize(sizeof(int32_t) + sizeof(uint32_t) + safe_message.size());

    size_t offset = 0;
    std::memcpy(payload.data() + offset, &response.result_code, sizeof(int32_t));
    offset += sizeof(int32_t);

    uint32_t msg_len = static_cast<uint32_t>(safe_message.size());
    std::memcpy(payload.data() + offset, &msg_len, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    if (msg_len > 0)
    {
        std::memcpy(payload.data() + offset, safe_message.data(), msg_len);
    }

    RPCRequest::Header header;
    header.command = static_cast<uint16_t>(RPCCommand::RESPONSE);
    header.sequence_id = response.sequence_id;
    header.payload_size = static_cast<uint32_t>(payload.size());
    header.checksum = compute_rpc_checksum(header, payload);

    std::vector<uint8_t> buffer;
    buffer.resize(RPC_HEADER_SIZE + payload.size());
    std::memcpy(buffer.data(), &header, RPC_HEADER_SIZE);
    std::memcpy(buffer.data() + RPC_HEADER_SIZE, payload.data(), payload.size());

    return buffer;
}

static RetCode deserialize_rpc_response(const RPCRequest::Header &header, const std::vector<uint8_t> &payload,
                                        RPCResponse &response)
{
    if (header.command != static_cast<uint16_t>(RPCCommand::RESPONSE))
    {
        return {RetCode::FAILED, "Invalid response command"};
    }

    if (compute_rpc_checksum(header, payload) != header.checksum)
    {
        return {RetCode::FAILED, "Checksum mismatch"};
    }

    if (payload.size() < sizeof(int32_t) + sizeof(uint32_t))
    {
        return {RetCode::FAILED, "Response payload too small"};
    }

    size_t offset = 0;
    std::memcpy(&response.result_code, payload.data() + offset, sizeof(int32_t));
    offset += sizeof(int32_t);

    uint32_t msg_len;
    std::memcpy(&msg_len, payload.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    if (offset + msg_len > payload.size())
    {
        return {RetCode::FAILED, "Invalid message length"};
    }

    if (msg_len > 0)
    {
        response.message.assign(payload.begin() + offset, payload.begin() + offset + msg_len);
    }

    response.sequence_id = header.sequence_id;
    return RetCode::OK;
}

// RPC service session class
class RPCService::Session : public std::enable_shared_from_this<Session>
{
  public:
    Session(tcp::socket socket, RPCService *service)
        : socket_(std::move(socket)), service_(service), closed_(false)
    {
    }

    ~Session()
    {
        close();
    }

    void start()
    {
        read_header();
    }

    bool is_alive() const
    {
        return !closed_.load() && socket_.is_open();
    }

    void send_response(const RPCResponse &response)
    {
        if (closed_.load())
        {
            return;
        }

        auto buffer = std::make_shared<std::vector<uint8_t>>(serialize_rpc_response(response));

        asio::async_write(socket_, asio::buffer(*buffer),
                          [buffer, self = shared_from_this()](const asio::error_code &ec, size_t) {
                              if (ec)
                              {
                                  self->close();
                              }
                          });
    }

    void close()
    {
        bool expected = false;
        if (closed_.compare_exchange_strong(expected, true))
        {
            std::error_code ec;
            if (socket_.is_open())
            {
                socket_.shutdown(asio::ip::tcp::socket::shutdown_both, ec);
                socket_.close(ec);
            }
        }
    }

  private:
    void read_header()
    {
        auto self = shared_from_this();
        asio::async_read(socket_, asio::buffer(&header_, RPC_HEADER_SIZE), [self](const asio::error_code &ec, size_t) {
            if (ec || self->closed_.load())
            {
                self->close();
                return;
            }

            if (self->header_.payload_size <= RPC_MAX_PAYLOAD_SIZE)
            {
                self->read_payload();
            }
            else
            {
                self->close();
            }
        });
    }

    void read_payload()
    {
        if (header_.payload_size == 0)
        {
            if (compute_rpc_checksum(header_, {}) == header_.checksum)
            {
                RPCRequest request;
                request.header = header_;
                service_->handle_request(shared_from_this(), request);
                read_header();
            }
            else
            {
                close();
            }
            return;
        }

        payload_.resize(header_.payload_size);
        auto self = shared_from_this();
        asio::async_read(socket_, asio::buffer(payload_), [self](const asio::error_code &ec, size_t) {
            if (ec || self->closed_.load())
            {
                self->close();
                return;
            }

            if (compute_rpc_checksum(self->header_, self->payload_) == self->header_.checksum)
            {
                RPCRequest request;
                request.header = self->header_;
                request.payload = self->payload_;
                self->service_->handle_request(self, request);
                self->read_header();
            }
            else
            {
                self->close();
            }
        });
    }

    tcp::socket socket_;
    RPCService *service_;
    RPCRequest::Header header_;
    std::vector<uint8_t> payload_;
    std::atomic_bool closed_;
};

RPCService::RPCService(asio::io_context &io_context, unsigned short port)
    : io_context_(io_context), acceptor_(io_context, tcp::endpoint(tcp::v4(), port)), running_(false),
      handler_strand_(asio::make_strand(io_context))
{
}

RPCService::~RPCService()
{
    stop();
}

RetCode RPCService::start()
{
    if (running_.exchange(true))
    {
        return {RetCode::NOACTION, "RPC service already running"};
    }

    start_accept();
    AUDIO_INFO_PRINT("RPC service started on port %u", acceptor_.local_endpoint().port());
    return RetCode::OK;
}

RetCode RPCService::stop()
{
    if (!running_.exchange(false))
    {
        return {RetCode::NOACTION, "RPC service not running"};
    }

    // Close acceptor
    std::error_code ec;
    acceptor_.close(ec);
    if (ec && ec != asio::error::bad_descriptor)
    {
        AUDIO_ERROR_PRINT("Error closing RPC acceptor: %s", ec.message().c_str());
    }

    // Close all sessions
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        for (auto &weak_session : active_sessions_)
        {
            if (auto session = weak_session.lock())
            {
                session->close();
            }
        }
        active_sessions_.clear();
    }

    // Wait for async operations to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    AUDIO_INFO_PRINT("RPC service stopped");
    return RetCode::OK;
}

void RPCService::register_handler(RPCCommand cmd, RequestHandler handler)
{
    // No lock needed - handlers are registered before start()
    handlers_[cmd] = std::move(handler);
}

void RPCService::start_accept()
{
    if (!running_.load())
    {
        return;
    }

    acceptor_.async_accept([this](const asio::error_code &ec, asio::ip::tcp::socket socket) {
        if (!running_.load())
        {
            return;
        }

        if (!ec)
        {
            cleanup_dead_sessions();

            if (can_accept_connection())
            {
                auto session = std::make_shared<Session>(std::move(socket), this);
                {
                    std::lock_guard<std::mutex> lock(sessions_mutex_);
                    active_sessions_.push_back(session);
                }
                session->start();
            }
            else
            {
                AUDIO_DEBUG_PRINT("Connection limit reached, rejecting connection");
                socket.close();
            }
        }
        else if (ec != asio::error::operation_aborted)
        {
            AUDIO_ERROR_PRINT("Accept error: %s", ec.message().c_str());
        }

        if (running_.load())
        {
            start_accept();
        }
    });
}

bool RPCService::can_accept_connection() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(sessions_mutex_));
    size_t active_count =
        std::count_if(active_sessions_.begin(), active_sessions_.end(), [](const std::weak_ptr<Session> &wp) {
            auto sp = wp.lock();
            return sp && sp->is_alive();
        });
    return active_count < RPC_MAX_CONNECTIONS;
}

void RPCService::cleanup_dead_sessions()
{
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    active_sessions_.erase(std::remove_if(active_sessions_.begin(), active_sessions_.end(),
                                          [](const std::weak_ptr<Session> &wp) { return wp.expired(); }),
                           active_sessions_.end());
}

void RPCService::handle_request(std::shared_ptr<Session> session, const RPCRequest &request)
{
    // Post to strand to serialize all request handling
    asio::post(handler_strand_, [this, session, request]() {
        RPCResponse response;
        response.sequence_id = request.header.sequence_id;

        // No lock needed - all accesses are serialized via handler_strand_
        auto it = handlers_.find(static_cast<RPCCommand>(request.header.command));

        if (it != handlers_.end())
        {
            // Handler execution is now serialized via handler_strand_
            RetCode ret = it->second(request.payload);
            response.result_code = ret.err;
            response.message = ret.what();
        }
        else
        {
            response.result_code = RetCode::FAILED;
            response.message = "Unknown command";
        }

        session->send_response(response);
    });
}

RPCClient::RPCClient(asio::io_context &io_context, const std::string &server_ip, unsigned short server_port)
    : io_context_(io_context), socket_(io_context_), server_ip_(server_ip), server_port_(server_port), sequence_id_(0)
{
}

RPCClient::~RPCClient()
{
    asio::error_code ec;
    socket_.close(ec);
    AUDIO_DEBUG_PRINT("RPC client destroyed, socket closed");
}

RetCode RPCClient::send_request(RPCCommand cmd, const std::vector<uint8_t> &payload)
{
    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (payload.size() > RPC_MAX_PAYLOAD_SIZE)
    {
        return {RetCode::EPARAM, "Payload too large"};
    }

    try
    {
        if (!socket_.is_open())
        {
            tcp::resolver resolver(io_context_);
            auto endpoints = resolver.resolve(server_ip_, std::to_string(server_port_));
            asio::connect(socket_, endpoints);
            AUDIO_INFO_PRINT("Connected to RPC server %s:%u", server_ip_.c_str(), server_port_);
        }

        RPCRequest::Header header;
        header.command = static_cast<uint16_t>(cmd);
        header.sequence_id = sequence_id_++;
        header.payload_size = static_cast<uint32_t>(payload.size());
        header.checksum = compute_rpc_checksum(header, payload);

        std::vector<uint8_t> buffer;
        buffer.resize(RPC_HEADER_SIZE + payload.size());
        std::memcpy(buffer.data(), &header, RPC_HEADER_SIZE);
        if (!payload.empty())
        {
            std::memcpy(buffer.data() + RPC_HEADER_SIZE, payload.data(), payload.size());
        }

        asio::write(socket_, asio::buffer(buffer));

        RPCRequest::Header resp_header;
        asio::read(socket_, asio::buffer(&resp_header, RPC_HEADER_SIZE));

        if (resp_header.sequence_id != header.sequence_id)
        {
            return {RetCode::FAILED, "Sequence ID mismatch"};
        }

        if (resp_header.payload_size > RPC_MAX_PAYLOAD_SIZE)
        {
            return {RetCode::FAILED, "Response payload too large"};
        }

        std::vector<uint8_t> resp_payload(resp_header.payload_size);
        if (resp_header.payload_size > 0)
        {
            asio::read(socket_, asio::buffer(resp_payload));
        }

        RPCResponse response;
        auto ret = deserialize_rpc_response(resp_header, resp_payload, response);
        if (!ret)
        {
            return ret;
        }

        return static_cast<RetCode::Code>(response.result_code);
    }
    catch (const asio::system_error &e)
    {
        asio::error_code ec;
        socket_.close(ec);
        AUDIO_ERROR_PRINT("RPC network error: %s", e.what());
        return {RetCode::EXCEPTION, "Network error"};
    }
    catch (const std::exception &e)
    {
        asio::error_code ec;
        socket_.close(ec);
        AUDIO_ERROR_PRINT("RPC exception: %s", e.what());
        return RetCode::EXCEPTION;
    }
}

RetCode RPCClient::connect_stream(IToken itoken, OToken otoken, const std::string &ip, unsigned short port)
{
    if (ip.size() > RPC_MAX_IP_LENGTH)
    {
        return {RetCode::EPARAM, "IP address too long"};
    }

    std::vector<uint8_t> payload;
    payload.reserve(4 + ip.size());
    payload.push_back(itoken.tok);
    payload.push_back(otoken.tok);
    append_to_payload(payload, port);
    payload.insert(payload.end(), ip.begin(), ip.end());

    return send_request(RPCCommand::CONNECT, payload);
}

RetCode RPCClient::disconnect_stream(IToken itoken, OToken otoken, const std::string &ip, unsigned short port)
{
    if (ip.size() > RPC_MAX_IP_LENGTH)
    {
        return {RetCode::EPARAM, "IP address too long"};
    }

    std::vector<uint8_t> payload;
    payload.reserve(4 + ip.size());
    payload.push_back(itoken.tok);
    payload.push_back(otoken.tok);
    append_to_payload(payload, port);
    payload.insert(payload.end(), ip.begin(), ip.end());

    return send_request(RPCCommand::DISCONNECT, payload);
}

RetCode RPCClient::set_volume(AudioToken token, unsigned int volume)
{
    if (volume > 100)
    {
        return {RetCode::EPARAM, "Volume out of range (0-100)"};
    }

    std::vector<uint8_t> payload;
    payload.reserve(5);
    payload.push_back(token.tok);
    append_to_payload(payload, volume);

    return send_request(RPCCommand::SET_VOLUME, payload);
}

RetCode RPCClient::mute(AudioToken token, bool enable)
{
    std::vector<uint8_t> payload;
    payload.reserve(3);
    payload.push_back(token.tok);
    payload.push_back(0xFF);
    payload.push_back(enable ? 1 : 0);

    return send_request(RPCCommand::MUTE, payload);
}

RetCode RPCClient::mute(IToken itoken, OToken otoken, bool enable, const std::string &ip)
{
    if (ip.size() > RPC_MAX_IP_LENGTH)
    {
        return {RetCode::EPARAM, "IP address too long"};
    }

    std::vector<uint8_t> payload;
    payload.reserve(3 + ip.size());
    payload.push_back(itoken.tok);
    payload.push_back(otoken.tok);
    payload.push_back(enable ? 1 : 0);
    payload.insert(payload.end(), ip.begin(), ip.end());

    return send_request(RPCCommand::MUTE, payload);
}

RetCode RPCClient::play(const std::string &name, int cycles, OToken otoken)
{
    if (name.size() > RPC_MAX_PATH_LENGTH)
    {
        return {RetCode::EPARAM, "File path too long"};
    }

    std::vector<uint8_t> payload;
    payload.reserve(5 + name.size());
    payload.push_back(otoken.tok);
    append_to_payload(payload, static_cast<int32_t>(cycles));
    payload.insert(payload.end(), name.begin(), name.end());

    return send_request(RPCCommand::PLAY, payload);
}

RetCode RPCClient::stop_play(const std::string &path)
{
    if (path.size() > RPC_MAX_PATH_LENGTH)
    {
        return {RetCode::EPARAM, "File path too long"};
    }

    std::vector<uint8_t> payload(path.begin(), path.end());
    return send_request(RPCCommand::STOP_PLAY, payload);
}

RetCode RPCClient::set_player_volume(unsigned int volume)
{
    if (volume > 100)
    {
        return {RetCode::EPARAM, "Volume out of range (0-100)"};
    }

    std::vector<uint8_t> payload;
    payload.reserve(sizeof(uint32_t));
    append_to_payload(payload, volume);

    return send_request(RPCCommand::SET_PLAYER_VOLUME, payload);
}