#include "audio_network.h"
#include "audio_stream.h"

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
    memset(buffer_addr, 0, buf_length);
    return load(buffer_addr, read_length);
}

// Standard IMA ADPCM index table and step table
static constexpr int ADPCM_INDEX_TABLE[16] = {-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8};
static constexpr int ADPCM_STEP_TABLE[89] = {
    7,    8,     9,     10,    11,    12,    13,    14,    16,    17,    19,    21,    23,    25,   28,
    31,   34,    37,    41,    45,    50,    55,    60,    66,    73,    80,    88,    97,    107,  118,
    130,  143,   157,   173,   190,   209,   230,   253,   279,   307,   337,   371,   408,   449,  494,
    544,  598,   658,   724,   796,   876,   963,   1060,  1166,  1282,  1411,  1552,  1707,  1878, 2066,
    2272, 2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,  7132,  7845, 8630,
    9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767};
static constexpr size_t ADPCM_HEADER_SIZE_PER_CHANNEL = 4;

// Encoder
NetEncoder::NetEncoder(unsigned int channels, unsigned int max_frames) : channels(channels), max_frames(max_frames)
{
    encode_states.resize(channels);
    size_t buffer_size = calculate_encoded_size(max_frames);
    encode_buffer = std::make_unique<uint8_t[]>(buffer_size);
    reset();
}

void NetEncoder::reset() noexcept
{
    for (auto &state : encode_states)
    {
        state.predictor = 0;
        state.step_index = 0;
    }
}

size_t NetEncoder::calculate_encoded_size(unsigned int frames) const noexcept
{
    return channels * ADPCM_HEADER_SIZE_PER_CHANNEL + (frames * channels + 1) / 2;
}

uint8_t NetEncoder::encode_sample(int16_t sample, NetEncoder::State &state)
{
    int diff = sample - state.predictor;
    uint8_t code = 0;

    if (diff < 0)
    {
        code = 8;
        diff = -diff;
    }

    int step = ADPCM_STEP_TABLE[state.step_index];

    int temp = step;
    if (diff >= temp)
    {
        code |= 4;
        diff -= temp;
    }
    temp = step >> 1;
    if (diff >= temp)
    {
        code |= 2;
        diff -= temp;
    }
    temp = step >> 2;
    if (diff >= temp)
    {
        code |= 1;
    }

    diff = 0;
    if (code & 4)
        diff += step;
    if (code & 2)
        diff += step >> 1;
    if (code & 1)
        diff += step >> 2;
    diff += step >> 3;

    if (code & 8)
        state.predictor -= diff;
    else
        state.predictor += diff;

    if (state.predictor > 32767)
        state.predictor = 32767;
    else if (state.predictor < -32768)
        state.predictor = -32768;

    state.step_index += ADPCM_INDEX_TABLE[code & 15];

    if (state.step_index < 0)
        state.step_index = 0;
    else if (state.step_index > 88)
        state.step_index = 88;

    return code & 15;
}

const uint8_t *NetEncoder::encode(const int16_t *pcm_data, unsigned int frames, size_t &out_size)
{
    if (!pcm_data || frames == 0)
    {
        out_size = 0;
        return nullptr;
    }

    frames = std::min(frames, max_frames);
    out_size = calculate_encoded_size(frames);
    uint8_t *out = encode_buffer.get();

    for (unsigned int ch = 0; ch < channels; ch++)
    {
        auto predictor_value = static_cast<int16_t>(encode_states[ch].predictor);
        out[0] = predictor_value & 0xFF;
        out[1] = (predictor_value >> 8) & 0xFF;
        out += 2;

        *out++ = static_cast<uint8_t>(encode_states[ch].step_index);
        *out++ = 0; // Reserved byte
    }

    unsigned int samples_to_encode = frames * channels;
    for (unsigned int i = 0; i < samples_to_encode; i += 2)
    {
        unsigned int ch = i % channels;
        uint8_t nibble1 = encode_sample(pcm_data[i], encode_states[ch]);
        uint8_t nibble2 = 0;
        if (i + 1 < samples_to_encode)
        {
            ch = (i + 1) % channels;
            nibble2 = encode_sample(pcm_data[i + 1], encode_states[ch]);
        }
        *out++ = nibble1 | (nibble2 << 4);
    }

    return encode_buffer.get();
}

// Decoder
NetDecoder::NetDecoder(unsigned int channels, unsigned int max_frames) : channels(channels), max_frames(max_frames)
{
    decode_states.resize(channels);
    decode_buffer = std::make_unique<int16_t[]>(max_frames * channels);
    reset();
}

void NetDecoder::reset() noexcept
{
    for (auto &state : decode_states)
    {
        state.predictor = 0;
        state.step_index = 0;
    }
}

int16_t NetDecoder::decode_sample(uint8_t code, NetDecoder::State &state)
{
    int step = ADPCM_STEP_TABLE[state.step_index];

    int diff = step >> 3;
    if (code & 1)
        diff += step >> 2;
    if (code & 2)
        diff += step >> 1;
    if (code & 4)
        diff += step;

    if (code & 8)
        state.predictor -= diff;
    else
        state.predictor += diff;

    if (state.predictor > 32767)
        state.predictor = 32767;
    else if (state.predictor < -32768)
        state.predictor = -32768;

    state.step_index += ADPCM_INDEX_TABLE[code & 15];

    if (state.step_index < 0)
        state.step_index = 0;
    else if (state.step_index > 88)
        state.step_index = 88;

    return static_cast<int16_t>(state.predictor);
}

const int16_t *NetDecoder::decode(const uint8_t *adpcm_data, size_t adpcm_size, unsigned int &out_frames)
{
    if (!adpcm_data || adpcm_size <= channels * ADPCM_HEADER_SIZE_PER_CHANNEL)
    {
        out_frames = 0;
        return nullptr;
    }

    size_t header_size = channels * ADPCM_HEADER_SIZE_PER_CHANNEL;
    size_t adpcm_data_size = adpcm_size - header_size;
    size_t adpcm_samples = adpcm_data_size * 2; // Each byte contains two 4-bit samples

    out_frames = static_cast<unsigned int>(adpcm_samples / channels);
    out_frames = std::min(out_frames, max_frames);

    std::memset(decode_buffer.get(), 0, out_frames * channels * sizeof(int16_t));

    const uint8_t *in = adpcm_data;
    for (unsigned int ch = 0; ch < channels; ch++)
    {
        int16_t predictor_value = in[0] | (in[1] << 8);
        decode_states[ch].predictor = predictor_value;
        in += 2;

        decode_states[ch].step_index = *in++;
        in++; // Skip reserved byte
    }

    unsigned int sample_index = 0;
    for (unsigned int i = 0; i < adpcm_data_size && sample_index < out_frames * channels; i++)
    {
        uint8_t byte = adpcm_data[header_size + i];

        unsigned int ch = sample_index % channels;
        decode_buffer[sample_index++] = decode_sample(byte & 0x0F, decode_states[ch]);
        if (sample_index < out_frames * channels)
        {
            ch = sample_index % channels;
            decode_buffer[sample_index++] = decode_sample((byte >> 4) & 0x0F, decode_states[ch]);
        }
    }

    return decode_buffer.get();
}

// Packet header validation
bool NetPacketHeader::validate(const char *data, size_t length)
{
    if (length < sizeof(NetPacketHeader))
    {
        AUDIO_DEBUG_PRINT("Received undersized packet (%zu bytes)", length);
        return false;
    }

    auto header = *reinterpret_cast<const NetPacketHeader *>(data);

    if (header.magic_num != NET_MAGIC_NUM)
    {
        AUDIO_DEBUG_PRINT("Invalid packet magic number: 0x%02X", header.magic_num);
        return false;
    }

    if (header.channels == 0 || header.channels > 2)
    {
        AUDIO_DEBUG_PRINT("Invalid channel count: %u", header.channels);
        return false;
    }

    if (byte_to_bandwidth(header.sample_rate) == AudioBandWidth::Unknown)
    {
        AUDIO_DEBUG_PRINT("Invalid sample rate: %u", header.sample_rate);
        return false;
    }

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

NetWorker::NetWorker(asio::io_context &io_context)
    : retry_count(0), io_context(io_context), running(false), receive_buffer(new char[NETWORK_MAX_BUFFER_SIZE]),
      stats_timer(io_context)
{
    try
    {
        socket = std::make_unique<udp::socket>(io_context);
        socket->open(udp::v4());

        asio::socket_base::send_buffer_size option_send(262144);    // 256KB
        asio::socket_base::receive_buffer_size option_recv(262144); // 256KB
        asio::error_code ec;
        socket->set_option(option_send, ec);
        socket->set_option(option_recv, ec);
        socket->set_option(asio::socket_base::reuse_address(true), ec);

        socket->bind(udp::endpoint(udp::v4(), NETWORK_AUDIO_TRANS_PORT), ec);
        if (ec)
        {
            AUDIO_ERROR_PRINT("Failed to bind socket: %s", ec.message().c_str());
        }

        AUDIO_INFO_PRINT("NetWorker initialized on port %d", socket->local_endpoint().port());
    }
    catch (const std::exception &e)
    {
        AUDIO_ERROR_PRINT("Failed to create socket: %s", e.what());
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

    if (!socket || !socket->is_open())
    {
        return {RetCode::FAILED, "Socket not available"};
    }

    running = true;
    start_receive_loop();
    start_stats_loop();

    return {RetCode::OK, "Started"};
}

RetCode NetWorker::stop()
{
    if (!running)
    {
        return {RetCode::NOACTION, "Not running"};
    }

    running = false;

    if (socket && socket->is_open())
    {
        asio::error_code ec;
        socket->cancel(ec);
    }

    stats_timer.cancel();

    {
        std::lock_guard<std::mutex> lock(receivers_mutex);
        receivers.clear();
    }

    {
        std::lock_guard<std::mutex> lock(senders_mutex);
        senders.clear();
    }

    AUDIO_INFO_PRINT("NetWorker stopped");
    return {RetCode::OK, "Stopped"};
}

void NetWorker::report()
{
    std::lock_guard<std::mutex> lock(receivers_mutex);

    for (auto &pair : decoders)
    {
        auto stats = pair.second.get_period_stats();
        auto token = pair.first;

        AUDIO_INFO_PRINT("NETSTATS(%u) : [loss] %.2f%%, [jitter] %.2fms, [received] %u, [lost] %u, [out-of-order] %u",
                         token, stats.packet_loss_rate, stats.average_jitter, stats.packets_received,
                         stats.packets_lost, stats.packets_out_of_order);
    }
}

RetCode NetWorker::register_receiver(uint8_t token, ReceiveCallback callback)
{
    if (!callback)
    {
        return {RetCode::FAILED, "Invalid callback"};
    }

    if (!socket || !socket->is_open())
    {
        return {RetCode::FAILED, "NetWorker not ready"};
    }

    std::lock_guard<std::mutex> lock(receivers_mutex);

    auto result = receivers.emplace(token, ReceiverContext(std::move(callback)));
    if (!result.second)
    {
        result.first->second = std::move(callback);
        return {RetCode::OK, "Receiver callback updated"};
    }

    AUDIO_DEBUG_PRINT("Registered receiver for token %u", token);
    return {RetCode::OK, "Receiver registered"};
}

RetCode NetWorker::unregister_receiver(uint8_t token)
{
    std::lock_guard<std::mutex> lock(receivers_mutex);

    auto it = receivers.find(token);
    if (it == receivers.end())
    {
        return {RetCode::NOACTION, "Receiver not found"};
    }

    receivers.erase(it);
    AUDIO_DEBUG_PRINT("Unregistered receiver for token %u", token);
    return {RetCode::OK, "Receiver unregistered"};
}

void NetWorker::report_conns(std::vector<InfoLabel>& result)
{
    std::lock_guard<std::mutex> lock(senders_mutex);
    for (const auto &pair : senders)
    {
        const auto &context = pair.second;
        for (const auto &dest : context.destinations)
        {
            result.emplace_back(0, pair.first, dest.endpoint.address().to_v4().to_uint(), dest.receiver_token,
                                true, false, false);
        }
    }
}

RetCode NetWorker::register_sender(uint8_t sender_id, unsigned int channels, unsigned int sample_rate)
{
    if (!socket || !socket->is_open())
    {
        return {RetCode::FAILED, "Socket not available"};
    }

    std::lock_guard<std::mutex> lock(senders_mutex);
    if (senders.find(sender_id) != senders.end())
    {
        return {RetCode::FAILED, "Sender ID already exists"};
    }

    senders.emplace(sender_id, SenderContext(channels, sample_rate));
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

    if (!is_ready())
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

    size_t encoded_size;
    const uint8_t *encoded_data = context.encoder->encode(data, frames, encoded_size);
    if (!encoded_data || encoded_size == 0)
    {
        return {RetCode::FAILED, "Encoding failed"};
    }

    NetPacketHeader header{};
    header.sender_id = sender_id;
    header.channels = static_cast<uint8_t>(context.channels);
    header.magic_num = NET_MAGIC_NUM;
    header.sample_rate = context.sample_rate / 1000;
    header.sequence = context.isequence++;
    header.timestamp =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();

    std::array<asio::const_buffer, 2> buffers{asio::buffer(&header, sizeof(header)),
                                              asio::buffer(encoded_data, encoded_size)};

    for (const auto &dest : context.destinations)
    {
        header.receiver_id = dest.receiver_token;
        socket->async_send_to(buffers, dest.endpoint, [](const asio::error_code &error, std::size_t /*bytes*/) {
            if (error)
            {
                AUDIO_DEBUG_PRINT("Send error: %s", error.message().c_str());
            }
        });
    }

    return {RetCode::OK, "Data sent"};
}

void NetWorker::start_receive_loop()
{
    if (!is_ready())
    {
        return;
    }
    auto self = shared_from_this();
    socket->async_receive_from(asio::buffer(receive_buffer.get(), NETWORK_MAX_BUFFER_SIZE), sender_endpoint,
                               [self](const asio::error_code &error, std::size_t bytes_transferred) {
                                   self->handle_receive(error, bytes_transferred);
                               });
}

void NetWorker::start_stats_loop()
{
    auto self = shared_from_this();
    stats_timer.expires_after(std::chrono::minutes(1));
    stats_timer.async_wait([self](const asio::error_code &ec) {
        if (!ec && self->running)
        {
            self->report();
            self->start_stats_loop();
        }
    });
}

void NetWorker::handle_receive(const asio::error_code &error, std::size_t bytes_transferred)
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

    const NetPacketHeader *header = nullptr;
    uint8_t sender_id = 0;
    uint8_t receiver_id = 0;
    uint8_t channels = 0;
    AudioBandWidth sample_enum = AudioBandWidth::Unknown;
    const uint8_t *adpcm_data = nullptr;
    size_t adpcm_size = 0;

    if (bytes_transferred < sizeof(NetPacketHeader))
    {
        goto exit;
    }

    header = reinterpret_cast<const NetPacketHeader *>(receive_buffer.get());

    if (header->magic_num != NET_MAGIC_NUM)
    {
        goto exit;
    }

    sender_id = header->sender_id;
    receiver_id = header->receiver_id;
    channels = header->channels;
    sample_enum = byte_to_bandwidth(header->sample_rate);

    if (sample_enum == AudioBandWidth::Unknown)
    {
        goto exit;
    }

    adpcm_data = reinterpret_cast<const uint8_t *>(receive_buffer.get() + sizeof(NetPacketHeader));
    adpcm_size = bytes_transferred - sizeof(NetPacketHeader);

    {
        auto &context = get_decoder(sender_id, channels);
        context.update_stats(header->sequence, header->timestamp);

        unsigned int decode_frames = 0;
        auto decoded_data = context.decoder->decode(adpcm_data, adpcm_size, decode_frames);

        if (decoded_data && decode_frames > 0)
        {
            std::lock_guard<std::mutex> lock(receivers_mutex);
            auto it = receivers.find(receiver_id);
            if (it != receivers.end() && (it->second))
            {
                uint32_t ip_address =
                    sender_endpoint.address().is_v4() ? sender_endpoint.address().to_v4().to_uint() : 0;
                (it->second)(sender_id, channels, decode_frames, enum2val(sample_enum), decoded_data, ip_address);
            }
        }
    }

exit:
    if (running)
    {
        start_receive_loop();
    }
}

void NetWorker::retry_receive_with_backoff()
{
    int delay_ms = std::min(100 * (1 << retry_count), 30000);

    auto self = shared_from_this();
    auto timer = std::make_shared<asio::steady_timer>(io_context);
    timer->expires_after(std::chrono::milliseconds(delay_ms));
    timer->async_wait([self, timer](const asio::error_code &ec) {
        if (!ec)
        {
            self->retry_count++;
            self->start_receive_loop();
        }
    });
}

NetWorker::DecoderContext &NetWorker::get_decoder(uint8_t sender_id, unsigned int channels)
{
    std::lock_guard<std::mutex> lock(decoders_mutex);

    auto it = decoders.find(sender_id);
    if (it == decoders.end())
    {
        auto result = decoders.emplace(sender_id, DecoderContext(channels, NETWORK_MAX_FRAMES));
        return result.first->second;
    }

    return it->second;
}

void NetWorker::DecoderContext::update_stats(uint32_t sequence, uint64_t timestamp)
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
        return;
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
}

NetStatInfos NetWorker::DecoderContext::get_period_stats()
{
    NetStatInfos stats{};

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
    last_report_time = std::chrono::steady_clock::now();

    return stats;
}