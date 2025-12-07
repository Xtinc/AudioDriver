#include "audio_network.h"
#include "audio_stream.h"
#include <algorithm>
#include <cstring>
#include <set>

// KFifo
KFifo::KFifo(size_t blk_sz, size_t blk_num, unsigned int channel)
    : chan(channel), buf_length(channel * blk_sz), max_length(channel * blk_sz * blk_num), idle_count(0), head(0),
      tail(0), length(0), memory_addr(nullptr), buffer_addr(nullptr)
{

    memory_addr = new char[max_length];
    buffer_addr = new char[buf_length];
}

KFifo::~KFifo()
{
    delete[] buffer_addr;
    delete[] memory_addr;
}

bool KFifo::store(const char *input_addr, size_t write_length)
{
    if (!input_addr || write_length == 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lck(io_mtx);
    size_t space = available_space();
    size_t actual_write = std::min(write_length, space);

    if (actual_write == 0)
    {
        return false;
    }

    if (max_length - tail >= actual_write)
    {
        memcpy(memory_addr + tail, input_addr, actual_write);
        tail = (tail + actual_write == max_length) ? 0 : tail + actual_write;
    }
    else
    {
        size_t first_part = max_length - tail;
        memcpy(memory_addr + tail, input_addr, first_part);
        memcpy(memory_addr, input_addr + first_part, actual_write - first_part);
        tail = actual_write - first_part;
    }

    length += actual_write;
    return actual_write == write_length;
}

bool KFifo::load(char *output_addr, size_t read_length)
{
    std::lock_guard<std::mutex> lck(io_mtx);
    if (!output_addr || read_length > buf_length)
    {
        return false;
    }

    auto actual_read = std::min<size_t>(read_length, length);
    if (actual_read == 0)
    {
        return false;
    }

    if (max_length - head >= actual_read)
    {
        memcpy(output_addr, memory_addr + head, actual_read);
        head = (head + actual_read == max_length) ? 0 : head + actual_read;
    }
    else
    {
        size_t first_part = max_length - head;
        memcpy(output_addr, memory_addr + head, first_part);
        memcpy(output_addr + first_part, memory_addr, actual_read - first_part);
        head = actual_read - first_part;
    }

    length -= actual_read;
    return true;
}

bool KFifo::store_aside(size_t write_length)
{
    return store(buffer_addr, write_length);
}

bool KFifo::load_aside(size_t read_length)
{
    if (read_length == 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lck(io_mtx);
    if (read_length > length)
    {
        return false;
    }

    if (max_length - head >= read_length)
    {
        memcpy(buffer_addr, memory_addr + head, read_length);
        head = (head + read_length == max_length) ? 0 : head + read_length;
    }
    else
    {
        size_t first_part = max_length - head;
        memcpy(buffer_addr, memory_addr + head, first_part);
        memcpy(buffer_addr + first_part, memory_addr, read_length - first_part);
        head = read_length - first_part;
    }

    length -= read_length;
    return true;
}

// NetState implementation
NetState::NetState() : last_report_time(std::chrono::steady_clock::now())
{
}

void NetState::reset()
{
    last_sequence = 0;
    last_timestamp = 0;
    last_arrival_time = 0;
    packets_received = 0;
    packets_lost = 0;
    total_jitter = 0.0;
    max_jitter = 0.0;
    packets_out_of_order = 0;
    period_packets_received = 0;
    period_packets_lost = 0;
    period_total_jitter = 0.0;
    period_packets_out_of_order = 0;
    highest_sequence_seen = 0;
    first_packet = true;
    last_report_time = std::chrono::steady_clock::now();
}

bool NetState::update(uint32_t sequence, uint64_t timestamp, NetStatInfos &stats)
{
    auto now = std::chrono::steady_clock::now();
    uint64_t arrival_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    packets_received++;
    period_packets_received++;

    if (first_packet)
    {
        first_packet = false;
        last_sequence = sequence;
        highest_sequence_seen = sequence;
        last_timestamp = timestamp;
        last_arrival_time = arrival_time;
        return false;
    }

    if (sequence < highest_sequence_seen)
    {
        packets_out_of_order++;
        period_packets_out_of_order++;
    }
    else
    {
        if (sequence > last_sequence + 1)
        {
            uint32_t lost = sequence - last_sequence - 1;
            packets_lost += lost;
            period_packets_lost += lost;
        }

        highest_sequence_seen = sequence;
    }

    if (timestamp > last_timestamp)
    {
        uint64_t send_interval = timestamp - last_timestamp;
        uint64_t arrival_interval = arrival_time - last_arrival_time;

        double jitter = std::abs(static_cast<double>(arrival_interval) - static_cast<double>(send_interval));

        total_jitter += jitter;
        period_total_jitter += jitter;
        max_jitter = std::max(max_jitter, jitter);
    }

    last_sequence = sequence;
    last_timestamp = timestamp;
    last_arrival_time = arrival_time;

    if (now - last_report_time < std::chrono::minutes(1))
    {
        return false;
    }

    if (period_packets_received + period_packets_lost > 0)
    {
        stats.packet_loss_rate =
            static_cast<double>(period_packets_lost) / (period_packets_received + period_packets_lost) * 100.0;
    }

    if (period_packets_received > 0)
    {
        stats.average_jitter = period_total_jitter / period_packets_received;
    }

    stats.packets_received = period_packets_received;
    stats.packets_lost = period_packets_lost;
    stats.max_jitter = max_jitter;
    stats.packets_out_of_order = period_packets_out_of_order;

    period_packets_received = 0;
    period_packets_lost = 0;
    period_packets_out_of_order = 0;
    period_total_jitter = 0.0;
    last_report_time = now;

    return true;
}

// NetWorker
using udp = asio::ip::udp;

static RetCode resolve_endpoint(const std::string &ip, uint16_t port, asio::ip::udp::endpoint &endpoint)
{
    try
    {
        asio::ip::udp::resolver resolver(BG_SERVICE.get_executor());
        asio::error_code ec;

        auto endpoints = resolver.resolve(asio::ip::udp::v4(), ip, std::to_string(port), ec);
        if (ec)
        {
            AUDIO_ERROR_PRINT("Failed to resolve address %s:%d: %s", ip.c_str(), port, ec.message().c_str());
            return {RetCode::FAILED, "Failed to resolve address"};
        }

        endpoint = *endpoints.begin();
        return {RetCode::OK, "Address resolved"};
    }
    catch (const std::exception &e)
    {
        AUDIO_ERROR_PRINT("Exception resolving address: %s", e.what());
        return {RetCode::EXCEPTION, "Exception resolving address"};
    }
}

NetWorker::NetWorker(asio::io_context &ioc, uint16_t port, const std::string &local_ip)
    : retry_count(0), io_context(ioc), running(false), local_session_ip(0),
      receive_buffer(new char[NETWORK_MAX_BUFFER_SIZE])
{
    try
    {
        // Create receive socket and bind to specific port
        receive_socket = std::make_unique<udp::socket>(io_context);
        receive_socket->open(udp::v4());

        asio::socket_base::receive_buffer_size option_recv(262144); // 256KB
        asio::error_code ec;
        receive_socket->set_option(option_recv, ec);
        receive_socket->set_option(asio::socket_base::reuse_address(true), ec);

        receive_socket->bind(udp::endpoint(udp::v4(), port), ec);
        if (ec)
        {
            AUDIO_ERROR_PRINT("Failed to bind receive socket on port %d: %s", port, ec.message().c_str());
        }

        // Create send socket without binding to specific port
        send_socket = std::make_unique<udp::socket>(io_context);
        send_socket->open(udp::v4());

        asio::socket_base::send_buffer_size option_send(262144); // 256KB
        send_socket->set_option(option_send, ec);
        send_socket->set_option(asio::socket_base::reuse_address(true), ec);

        asio::error_code addr_ec;
        auto addr = asio::ip::make_address_v4(local_ip, addr_ec);
        if (!addr_ec)
        {
            local_session_ip = addr.to_uint();
        }

        AUDIO_INFO_PRINT("NetWorker initialized - receive port: %d", port);
    }
    catch (const std::exception &e)
    {
        AUDIO_ERROR_PRINT("Failed to create sockets: %s", e.what());
    }
}

NetWorker::~NetWorker()
{
    stop();
}

RetCode NetWorker::start()
{
    if (running)
    {
        return {RetCode::NOACTION, "Already running"};
    }

    if (!receive_socket || !receive_socket->is_open() || !send_socket || !send_socket->is_open())
    {
        return {RetCode::FAILED, "Sockets not available"};
    }

    running = true;
    start_receive_loop();

    return {RetCode::OK, "Started"};
}

RetCode NetWorker::stop()
{
    if (!running)
    {
        return {RetCode::NOACTION, "Not running"};
    }

    running = false;

    if (receive_socket && receive_socket->is_open())
    {
        asio::error_code ec;
        receive_socket->cancel(ec);
    }

    if (send_socket && send_socket->is_open())
    {
        asio::error_code ec;
        send_socket->cancel(ec);
    }

    {
        std::lock_guard<std::mutex> lock(senders_mutex);
        senders.clear();
    }

    receivers.clear();
    decoders.clear();

    AUDIO_INFO_PRINT("NetWorker stopped");
    return {RetCode::OK, "Stopped"};
}

RetCode NetWorker::register_receiver(uint8_t token, ReceiveCallback callback)
{
    if (!callback)
    {
        return {RetCode::FAILED, "Invalid callback"};
    }

    if (!receive_socket || !receive_socket->is_open())
    {
        return {RetCode::FAILED, "NetWorker not ready"};
    }

    auto it = receivers.find(token);
    if (it != receivers.end())
    {
        it->second = callback;
        return {RetCode::OK, "Receiver callback updated"};
    }
    else
    {
        receivers.emplace(token, std::move(callback));
        AUDIO_DEBUG_PRINT("Registered receiver for token %u", token);
        return {RetCode::OK, "Receiver registered"};
    }
}

RetCode NetWorker::unregister_receiver(uint8_t token)
{
    auto it = receivers.find(token);
    if (it == receivers.end())
    {
        return {RetCode::NOACTION, "Receiver not found"};
    }

    receivers.erase(it);
    AUDIO_DEBUG_PRINT("Unregistered receiver for token %u", token);
    return {RetCode::OK, "Receiver unregistered"};
}

RetCode NetWorker::register_sender(uint8_t sender_id, unsigned int channels, unsigned int sample_rate,
                                   AudioCodecType codec)
{
    if (!send_socket || !send_socket->is_open())
    {
        return {RetCode::FAILED, "Send socket not available"};
    }

    std::lock_guard<std::mutex> lock(senders_mutex);
    if (senders.find(sender_id) != senders.end())
    {
        return {RetCode::FAILED, "Sender ID already exists"};
    }

    auto result = senders.emplace(sender_id, SenderContext(channels, sample_rate, codec));
    if (!result.second)
    {
        senders.erase(sender_id);
        return {RetCode::FAILED, "Failed to create sender context"};
    }

    if (codec == AudioCodecType::OPUS && !result.first->second.encoder)
    {
        senders.erase(sender_id);
        return {RetCode::FAILED, "Failed to create Opus encoder"};
    }

    AUDIO_INFO_PRINT("Registered sender %u with codec type: %s", sender_id,
                     codec == AudioCodecType::OPUS ? "OPUS" : "PCM");

    return {RetCode::OK, "Sender registered"};
}

RetCode NetWorker::unregister_sender(uint8_t sender_id)
{
    std::lock_guard<std::mutex> lock(senders_mutex);
    auto removed = senders.erase(sender_id) > 0;

    if (removed)
    {
        AUDIO_DEBUG_PRINT("Unregistered sender for token %u", sender_id);
        return {RetCode::OK, "Sender unregistered"};
    }

    return {RetCode::NOACTION, "Sender not found"};
}

RetCode NetWorker::add_destination(uint8_t sender_id, uint8_t receiver_token, const std::string &ip, uint16_t port)
{
    if (ip.empty())
    {
        return {RetCode::FAILED, "Invalid IP address"};
    }

    asio::ip::udp::endpoint endpoint;
    RetCode res = resolve_endpoint(ip, port, endpoint);
    if (res != RetCode::OK)
    {
        return res;
    }

    std::lock_guard<std::mutex> lock(senders_mutex);
    auto iter = senders.find(sender_id);
    if (iter == senders.end())
    {
        return {RetCode::FAILED, "Sender not registered"};
    }

    auto &dest_list = iter->second.destinations;

    Destination new_dest(endpoint, receiver_token);

    if (std::find(dest_list.begin(), dest_list.end(), new_dest) != dest_list.end())
    {
        return {RetCode::OK, "Destination already exists"};
    }

    dest_list.push_back(std::move(new_dest));
    AUDIO_INFO_PRINT("Added destination %s for sender %u to receiver %u", ip.c_str(), sender_id, receiver_token);

    return {RetCode::OK, "Destination added"};
}

RetCode NetWorker::del_destination(uint8_t sender_id, uint8_t receiver_token, const std::string &ip, uint16_t port)
{
    if (ip.empty())
    {
        return {RetCode::FAILED, "Invalid IP address"};
    }

    asio::ip::udp::endpoint endpoint;
    RetCode res = resolve_endpoint(ip, port, endpoint);
    if (res != RetCode::OK)
    {
        return res;
    }

    std::lock_guard<std::mutex> lock(senders_mutex);
    auto iter = senders.find(sender_id);
    if (iter == senders.end())
    {
        return {RetCode::FAILED, "Sender not registered"};
    }

    auto &dest_list = iter->second.destinations;
    Destination target(endpoint, receiver_token);

    auto dest_iter = std::find(dest_list.begin(), dest_list.end(), target);
    if (dest_iter == dest_list.end())
    {
        return {RetCode::NOACTION, "Destination not found"};
    }

    dest_list.erase(dest_iter);
    AUDIO_INFO_PRINT("Removed destination %s for sender %u to receiver %u", ip.c_str(), sender_id, receiver_token);

    return {RetCode::OK, "Destination removed"};
}

RetCode NetWorker::send_audio(uint8_t sender_id, const int16_t *data, unsigned int frames)
{
    if (!data || frames == 0)
    {
        return {RetCode::FAILED, "Invalid input parameters"};
    }

    if (!running)
    {
        return {RetCode::FAILED, "NetWorker not running"};
    }

    std::lock_guard<std::mutex> lock(senders_mutex);
    auto it = senders.find(sender_id);
    if (it == senders.end())
    {
        return {RetCode::FAILED, "Sender not registered"};
    }

    auto &context = it->second;
    if (!context.encode_buffer)
    {
        return {RetCode::FAILED, "Buffer not available"};
    }

    const uint8_t *payload_data = nullptr;
    size_t payload_size = 0;

    if (context.codec_type == AudioCodecType::OPUS)
    {
        if (!context.encoder)
        {
            return {RetCode::FAILED, "Encoder not available"};
        }

        int encoded_size =
            opus_encode(context.encoder, data, frames, context.encode_buffer.get(), NETWORK_MAX_BUFFER_SIZE);
        if (encoded_size < 0)
        {
            AUDIO_ERROR_PRINT("Opus encoding failed: %s", opus_strerror(encoded_size));
            return {RetCode::EPARAM, "Failed to encode audio data"};
        }
        payload_data = context.encode_buffer.get();
        payload_size = static_cast<size_t>(encoded_size);
    }
    else // PCM
    {
        size_t pcm_size = frames * context.channels * sizeof(int16_t);
        if (pcm_size > NETWORK_MAX_BUFFER_SIZE)
        {
            return {RetCode::FAILED, "PCM data too large"};
        }
        std::memcpy(context.encode_buffer.get(), data, pcm_size);
        payload_data = context.encode_buffer.get();
        payload_size = pcm_size;
    }

    uint32_t sequence = context.isequence++;
    uint64_t timestamp =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();

    // Send data packets to all destinations
    for (const auto &dest : context.destinations)
    {
        send_data_packet(dest, sender_id, dest.receiver_token, sequence, timestamp, payload_data, payload_size,
                         static_cast<uint8_t>(context.channels), context.sample_rate, context.codec_type);
    }

    return {RetCode::OK, "Data sent"};
}

void NetWorker::start_receive_loop()
{
    auto self = shared_from_this();
    auto endpoint = std::make_shared<asio::ip::udp::endpoint>();
    receive_socket->async_receive_from(asio::buffer(receive_buffer.get(), NETWORK_MAX_BUFFER_SIZE), *endpoint,
                                       [self, endpoint](const asio::error_code &error, std::size_t bytes_transferred) {
                                           self->handle_receive(error, bytes_transferred, *endpoint);
                                       });
}

void NetWorker::handle_receive(const asio::error_code &error, std::size_t bytes_transferred,
                               const asio::ip::udp::endpoint &sender_endpoint)
{
    if (error)
    {
        if (error != asio::error::operation_aborted)
        {
            AUDIO_DEBUG_PRINT("Receive error: %s", error.message().c_str());
            retry_receive_with_backoff();
        }
        return;
    }

    auto magic_num = static_cast<uint8_t>(receive_buffer.get()[0]);
    if (bytes_transferred >= sizeof(DataPacket) &&
        (magic_num == NET_AUDIO_MAGIC_OPUS || magic_num == NET_AUDIO_MAGIC_PCM))
    {
        auto data_header = reinterpret_cast<const DataPacket *>(receive_buffer.get());
        const auto *payload = reinterpret_cast<const uint8_t *>(receive_buffer.get() + sizeof(DataPacket));
        size_t payload_size = bytes_transferred - sizeof(DataPacket);
        auto gateway_ip = sender_endpoint.address().is_v4() ? sender_endpoint.address().to_v4().to_uint() : 0;
        process_and_deliver_audio(data_header, payload, payload_size,
                                  SourceUUID{data_header->session_ip, gateway_ip, data_header->sender_id});
    }

    if (running)
    {
        start_receive_loop();
    }
}

void NetWorker::retry_receive_with_backoff()
{
    int delay_ms = std::min(100 * (1 << retry_count), 30000);
    retry_count = std::min(retry_count + 1, 8);

    auto self = shared_from_this();
    auto timer = std::make_shared<asio::steady_timer>(io_context);
    timer->expires_after(std::chrono::milliseconds(delay_ms));
    timer->async_wait([self, timer](const asio::error_code &ec) {
        if (!ec && self->running)
        {
            self->start_receive_loop();
        }
    });
}

NetWorker::DecoderContext &NetWorker::get_decoder(SourceUUID sid, unsigned int channels, unsigned int sample_rate)
{
    auto it = decoders.find(sid);
    if (it == decoders.end())
    {
        auto result = decoders.emplace(sid, DecoderContext(channels, sample_rate));
        return result.first->second;
    }

    return it->second;
}

void NetWorker::process_and_deliver_audio(const DataPacket *header, const uint8_t *opus_data, size_t opus_size,
                                          SourceUUID ssid)
{
    if (!opus_data || opus_size == 0)
    {
        return;
    }

    auto receiver_id = header->receiver_id;
    auto channels = header->channels;
    auto sample_rate = header->sample_rate;
    auto sequence = header->sequence;
    auto timestamp = header->timestamp;
    auto codec_type = (header->magic_num == NET_AUDIO_MAGIC_PCM) ? AudioCodecType::PCM : AudioCodecType::OPUS;

    auto &decoder_context = get_decoder(ssid, channels, sample_rate);
    NetStatInfos stats{};
    if (decoder_context.update_stats(sequence, timestamp, stats) && stats.packets_received > 0)
    {
        AUDIO_INFO_PRINT("NETSTATS(0x%08X|0x%08X:%u) : [loss] %.2f%%, [jitter] %.2fms, [received] %u, [lost] %u, "
                         "[out-of-order] %u",
                         ssid.sender_ip, ssid.gateway_ip, ssid.sender_token, stats.packet_loss_rate,
                         stats.average_jitter, stats.packets_received, stats.packets_lost, stats.packets_out_of_order);
    }

    int decoded_frames = 0;
    const int16_t *audio_data = nullptr;

    if (codec_type == AudioCodecType::OPUS)
    {
        if (!decoder_context.decoder || !decoder_context.decode_buffer)
        {
            AUDIO_ERROR_PRINT("Decoder not available for sender %u (session 0x%08X|0x%08X)", ssid.sender_token,
                              ssid.sender_ip, ssid.gateway_ip);
            return;
        }
        decoded_frames = opus_decode(decoder_context.decoder, opus_data, static_cast<opus_int32>(opus_size),
                                     decoder_context.decode_buffer.get(), NETWORK_MAX_FRAMES, 0);
        if (decoded_frames < 0)
        {
            AUDIO_ERROR_PRINT("Failed to decode Opus data from sender %u (session 0x%08X|0x%08X): %s",
                              ssid.sender_token, ssid.sender_ip, ssid.gateway_ip, opus_strerror(decoded_frames));
            return;
        }
        audio_data = decoder_context.decode_buffer.get();
    }
    else // PCM
    {
        decoded_frames = static_cast<int>(opus_size / (channels * sizeof(int16_t)));
        audio_data = reinterpret_cast<const int16_t *>(opus_data);
    }

    auto receiver_it = receivers.find(receiver_id);
    if (receiver_it != receivers.end())
    {
        receiver_it->second(channels, static_cast<unsigned int>(decoded_frames), sample_rate, audio_data, ssid);
    }
    else
    {
        AUDIO_DEBUG_PRINT("No receiver registered for token %u", receiver_id);
    }
}

void NetWorker::send_data_packet(const Destination &dest, uint8_t sender_id, uint8_t receiver_id, uint32_t sequence,
                                 uint64_t timestamp, const uint8_t *data, size_t size, uint8_t channels,
                                 uint32_t sample_rate, AudioCodecType codec_type)
{
    if (!running || !data || size == 0)
    {
        return;
    }

    DataPacket header{};
    header.magic_num = (codec_type == AudioCodecType::PCM) ? NET_AUDIO_MAGIC_PCM : NET_AUDIO_MAGIC_OPUS;
    header.sender_id = sender_id;
    header.receiver_id = receiver_id;
    header.channels = channels;
    header.sample_rate = sample_rate;
    header.sequence = sequence;
    header.session_ip = local_session_ip;
    header.timestamp = timestamp;

    std::array<asio::const_buffer, 2> buffers = {asio::buffer(&header, sizeof(header)), asio::buffer(data, size)};

    send_socket->async_send_to(buffers, dest.endpoint,
                               [dest, sequence](const asio::error_code &error, std::size_t /*bytes_sent*/) {
                                   if (error)
                                   {
                                       AUDIO_DEBUG_PRINT("Failed to send audio packet %u to %s:%d: %s", sequence,
                                                         dest.endpoint.address().to_string().c_str(),
                                                         dest.endpoint.port(), error.message().c_str());
                                   }
                               });
}