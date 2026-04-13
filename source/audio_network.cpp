#include "audio_network.h"
#include "audio_stream.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

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

double KFifo::write_water_level(double quantile) const
{
    return whist.quantile(quantile);
}

double KFifo::read_water_level(double quantile) const
{
    return rhist.quantile(quantile);
}

bool KFifo::store(const char *input_addr, size_t write_length)
{
    if (!input_addr || write_length == 0)
    {
        return false;
    }

    if (write_length > max_length)
    {
        return false;
    }

    std::lock_guard<std::mutex> lck(io_mtx);
    whist.add(static_cast<double>(length));
    size_t space = available_space();
    if (write_length > space)
    {
        size_t drop_length = write_length - space;
        head = (head + drop_length) % max_length;
        length -= drop_length;
    }

    if (max_length - tail >= write_length)
    {
        memcpy(memory_addr + tail, input_addr, write_length);
        tail = (tail + write_length == max_length) ? 0 : tail + write_length;
    }
    else
    {
        size_t first_part = max_length - tail;
        memcpy(memory_addr + tail, input_addr, first_part);
        memcpy(memory_addr, input_addr + first_part, write_length - first_part);
        tail = write_length - first_part;
    }

    length += write_length;
    return true;
}

bool KFifo::load(char *output_addr, size_t read_length)
{
    std::lock_guard<std::mutex> lck(io_mtx);
    if (!output_addr || read_length > buf_length)
    {
        return false;
    }

    rhist.add(static_cast<double>(length));
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
    rhist.add(static_cast<double>(length));
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
void NetState::update(uint32_t sequence, uint64_t timestamp, NetStatInfos &stats)
{
    auto now = std::chrono::steady_clock::now();
    uint64_t arrival_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    stats.seq_gap = 0;

    period_packets_received++;

    if (first_packet)
    {
        first_packet = false;
        highest_sequence_seen = sequence;
        last_timestamp = timestamp;
        last_arrival_time = arrival_time;
        stats.seq_gap = 0;
        return;
    }

    // Signed difference handles uint32_t wrap-around correctly
    int32_t seq_diff = static_cast<int32_t>(sequence - highest_sequence_seen);
    stats.seq_gap = seq_diff;

    if (seq_diff < 0)
    {
        period_packets_out_of_order++;
    }
    else if (seq_diff > 1)
    {
        // Gap: seq_diff-1 packets have been lost
        period_packets_lost += static_cast<uint32_t>(seq_diff - 1);
        highest_sequence_seen = sequence;
    }
    else if (seq_diff == 1)
    {
        highest_sequence_seen = sequence;
    }
    // seq_diff == 0: duplicate, ignore

    // Jitter (RFC 3550 §6.4.1): only for non-duplicate, non-out-of-order packets
    if (seq_diff > 0 && last_timestamp > 0 && timestamp > 0)
    {
        int64_t send_interval = static_cast<int64_t>(timestamp) - static_cast<int64_t>(last_timestamp);
        int64_t arrival_interval = static_cast<int64_t>(arrival_time) - static_cast<int64_t>(last_arrival_time);
        double jitter = std::abs(static_cast<double>(arrival_interval - send_interval));
        period_total_jitter += jitter;
        period_max_jitter = std::max(period_max_jitter, jitter);
        period_jitter_samples++;
    }

    last_timestamp = timestamp;
    last_arrival_time = arrival_time;
}

bool NetState::snapshot(NetStatInfos &stats)
{
    uint32_t total = period_packets_received + period_packets_lost;
    if (total == 0 && period_packets_out_of_order == 0)
    {
        return false;
    }

    stats.packet_loss_rate = (total > 0) ? static_cast<double>(period_packets_lost) / total * 100.0 : 0.0;
    stats.average_jitter = (period_jitter_samples > 0) ? period_total_jitter / period_jitter_samples : 0.0;
    stats.max_jitter = period_max_jitter;
    stats.packets_received = period_packets_received;
    stats.packets_lost = period_packets_lost;
    stats.packets_out_of_order = period_packets_out_of_order;
    stats.seq_gap = 0;

    period_packets_received = 0;
    period_packets_lost = 0;
    period_packets_out_of_order = 0;
    period_total_jitter = 0.0;
    period_max_jitter = 0.0;
    period_jitter_samples = 0;

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
    : receive_strand(asio::make_strand(ioc)), running(false), local_session_ip(0),
      receive_buffer(new char[NETWORK_MAX_BUFFER_SIZE])
{
    try
    {
        // Create receive socket and bind to specific port
        receive_socket = std::make_unique<udp::socket>(ioc);
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
        send_socket = std::make_unique<udp::socket>(ioc);
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

    return {RetCode::OK, "Sender registered"};
}

RetCode NetWorker::unregister_sender(uint8_t sender_id)
{
    std::lock_guard<std::mutex> lock(senders_mutex);
    auto removed = senders.erase(sender_id) > 0;
    return removed ? RetCode{RetCode::OK, "Sender unregistered"} : RetCode{RetCode::NOACTION, "Sender not found"};
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

RetCode NetWorker::clear_all_destinations(uint8_t sender_id)
{
    std::lock_guard<std::mutex> lock(senders_mutex);
    auto iter = senders.find(sender_id);
    if (iter == senders.end())
    {
        return {RetCode::NOACTION, "Sender not registered"};
    }

    size_t count = iter->second.destinations.size();
    iter->second.destinations.clear();

    if (count > 0)
    {
        AUDIO_INFO_PRINT("Cleared %zu destination(s) for sender %u", count, sender_id);
        return {RetCode::OK, "All destinations cleared"};
    }

    return {RetCode::NOACTION, "No destinations to clear"};
}

RetCode NetWorker::send_audio(uint8_t sender_id, const int16_t *data, unsigned int frames, AudioPriority priority)
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
                         static_cast<uint8_t>(context.channels), context.sample_rate, context.codec_type, priority);
    }

    return {RetCode::OK, "Data sent"};
}

void NetWorker::start_receive_loop()
{
    auto self = shared_from_this();
    receive_socket->async_receive_from(
        asio::buffer(receive_buffer.get(), NETWORK_MAX_BUFFER_SIZE), receive_endpoint,
        asio::bind_executor(receive_strand, [self](const asio::error_code &error, std::size_t bytes_transferred) {
            self->handle_receive(error, bytes_transferred);
        }));
}

void NetWorker::handle_receive(const asio::error_code &error, std::size_t bytes_transferred)
{
    if (error)
    {
        if (error != asio::error::operation_aborted)
        {
            AUDIO_DEBUG_PRINT("Receive error: %s", error.message().c_str());
            if (running)
            {
                start_receive_loop();
            }
        }
        return;
    }

    auto magic_num = static_cast<uint8_t>(receive_buffer.get()[0]);
    if (bytes_transferred >= sizeof(DataPacket) && DataPacket::is_valid_magic_num(magic_num))
    {
        auto data_header = reinterpret_cast<const DataPacket *>(receive_buffer.get());
        const auto *payload = reinterpret_cast<const uint8_t *>(receive_buffer.get() + sizeof(DataPacket));
        size_t payload_size = bytes_transferred - sizeof(DataPacket);
        auto gateway_ip = receive_endpoint.address().is_v4() ? receive_endpoint.address().to_v4().to_uint() : 0;
        process_and_deliver_audio(data_header, payload, payload_size,
                                  SourceUUID{data_header->session_ip, gateway_ip, data_header->sender_id});
    }

    if (running)
    {
        start_receive_loop();
    }
}

void NetWorker::query_stats()
{
    auto self = shared_from_this();
    asio::post(receive_strand, [self]() {
        size_t buf_size = 64 + self->decoders.size() * 192;
        auto buffer = new char[buf_size];
        auto ptr = buffer;
        size_t remain = buf_size;
        unsigned int line_no = 0;

        for (auto &entry : self->decoders)
        {
            auto &source_id = entry.first;
            auto &ctx = entry.second;
            NetStatInfos stats{};
            if (!ctx.stats.snapshot(stats))
            {
                continue;
            }

            if (line_no == 0)
            {
                int len = snprintf(ptr, remain, "NETSTATS");
                if (len > 0)
                {
                    size_t used = static_cast<size_t>(len);
                    if (used >= remain)
                    {
                        used = remain - 1;
                    }
                    ptr += used;
                    remain -= used;
                }
            }

            line_no++;

            int len =
                snprintf(ptr, remain,
                         "\n    [%u]. 0x%08X|0x%08X:%u loss=%.2f%% jit_avg=%.2fms jit_max=%.2fms rx=%u lost=%u ooo=%u",
                         line_no, source_id.sender_ip, source_id.gateway_ip, source_id.sender_token,
                         stats.packet_loss_rate, stats.average_jitter, stats.max_jitter, stats.packets_received,
                         stats.packets_lost, stats.packets_out_of_order);
            if (len > 0)
            {
                size_t used = static_cast<size_t>(len);
                if (used >= remain)
                {
                    used = remain - 1;
                }
                ptr += used;
                remain -= used;
            }
        }

        if (line_no > 0)
        {
            AUDIO_INFO_PRINT("%s", buffer);
        }

        delete[] buffer;
    });
}

NetWorker::DecoderContext &NetWorker::get_decoder(SourceUUID sid, unsigned int channels, unsigned int sample_rate,
                                                  AudioCodecType codec)
{
    auto it = decoders.find(sid);
    if (it == decoders.end())
    {
        auto result = decoders.emplace(sid, DecoderContext(channels, sample_rate, codec));
        return result.first->second;
    }

    return it->second;
}

void NetWorker::process_and_deliver_audio(const DataPacket *header, const uint8_t *opus_data, size_t opus_size,
                                          SourceUUID ssid)
{
    if (!header || !opus_data || opus_size == 0)
    {
        return;
    }

    const auto receiver_id = header->receiver_id;
    const auto channels = header->channels;
    const auto sample_rate = header->sample_rate;
    const auto sequence = header->sequence;
    const auto timestamp = header->timestamp;

    if (channels == 0 || sample_rate == 0)
    {
        return;
    }

    AudioCodecType codec_type = DataPacket::decode_magic_codec(header->magic_num);
    AudioPriority priority = DataPacket::decode_magic_priority(header->magic_num);

    // Look up receiver once; every delivery path below reuses this reference
    auto receiver_it = receivers.find(receiver_id);
    if (receiver_it == receivers.end())
    {
        AUDIO_DEBUG_PRINT("No receiver registered for token %u", receiver_id);
        return;
    }
    const auto &deliver = receiver_it->second;

    auto &ctx = get_decoder(ssid, channels, sample_rate, codec_type);

    NetStatInfos stats{};
    ctx.update_stats(sequence, timestamp, stats);

    // First packet may report seq_gap == 0 and must be accepted.
    // Duplicate packets (seq_gap == 0 after first decode) and out-of-order
    // packets (seq_gap < 0) are dropped to avoid inflating playout latency.
    if (stats.seq_gap < 0 || (stats.seq_gap == 0 && ctx.last_frames > 0))
    {
        AUDIO_DEBUG_PRINT("Drop stale packet seq=%u gap=%d sender=%u", sequence, stats.seq_gap, ssid.sender_token);
        return;
    }

    if (codec_type == AudioCodecType::OPUS)
    {
        if (!ctx.decoder || !ctx.decode_buffer)
        {
            AUDIO_ERROR_PRINT("Decoder not available for sender %u (session 0x%08X|0x%08X)", ssid.sender_token,
                              ssid.sender_ip, ssid.gateway_ip);
            return;
        }

        // Concealment before decoding the current packet
        // seq_gap == 1 : consecutive, nothing to do
        // seq_gap >= 2 : gap detected
        //   · FEC  — Opus embeds redundancy for the packet immediately preceding
        //             the current one; decode with decode_fec=1 to recover it.
        //   · PLC  — any further missing packets (gap > 2) are concealed by
        //             calling opus_decode with a null payload; this keeps the
        //             decoder state continuous.
        // seq_gap <= 0 : first packet, duplicate, or out-of-order — skip.
        if (stats.seq_gap >= 2 && ctx.last_frames > 0)
        {
            // FEC: recover the packet immediately before this one
            int fec_frames = opus_decode(ctx.decoder, opus_data, static_cast<opus_int32>(opus_size),
                                         ctx.fec_buffer.get(), ctx.last_frames, /*decode_fec=*/1);
            if (fec_frames > 0)
            {
                deliver(channels, static_cast<unsigned int>(fec_frames), sample_rate, ctx.fec_buffer.get(), ssid,
                        priority);
                // AUDIO_DEBUG_PRINT("FEC recovered %d frames before seq %u (sender %u)", fec_frames, sequence,
                //                   ssid.sender_token);
            }

            // PLC: conceal any remaining missing packets (gap > 2)
            for (int32_t i = 1; i < stats.seq_gap - 1; ++i)
            {
                int plc_frames =
                    opus_decode(ctx.decoder, nullptr, 0, ctx.fec_buffer.get(), ctx.last_frames, /*decode_fec=*/0);
                if (plc_frames > 0)
                {
                    deliver(channels, static_cast<unsigned int>(plc_frames), sample_rate, ctx.fec_buffer.get(), ssid,
                            priority);
                }
            }
        }

        int decoded_frames = opus_decode(ctx.decoder, opus_data, static_cast<opus_int32>(opus_size),
                                         ctx.decode_buffer.get(), NETWORK_MAX_FRAMES, /*decode_fec=*/0);
        if (decoded_frames < 0)
        {
            AUDIO_ERROR_PRINT("Opus decode failed for sender %u (session 0x%08X|0x%08X): %s", ssid.sender_token,
                              ssid.sender_ip, ssid.gateway_ip, opus_strerror(decoded_frames));
            return;
        }

        if (stats.seq_gap >= 0)
        {
            ctx.last_frames = decoded_frames;
        }

        deliver(channels, static_cast<unsigned int>(decoded_frames), sample_rate, ctx.decode_buffer.get(), ssid,
                priority);
    }
    else // PCM — pass through directly
    {
        int decoded_frames = static_cast<int>(opus_size / (channels * sizeof(int16_t)));
        if (decoded_frames <= 0)
        {
            return;
        }
        deliver(channels, static_cast<unsigned int>(decoded_frames), sample_rate,
                reinterpret_cast<const int16_t *>(opus_data), ssid, priority);
    }
}

void NetWorker::send_data_packet(const Destination &dest, uint8_t sender_id, uint8_t receiver_id, uint32_t sequence,
                                 uint64_t timestamp, const uint8_t *data, size_t size, uint8_t channels,
                                 uint32_t sample_rate, AudioCodecType codec_type, AudioPriority priority)
{
    if (!running || !data || size == 0)
    {
        return;
    }

    DataPacket header{};
    header.magic_num = DataPacket::encode_magic_num(codec_type, priority);
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