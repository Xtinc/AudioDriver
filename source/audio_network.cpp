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

void NetState::update(uint32_t sequence, uint64_t timestamp)
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

NetStatInfos NetState::get_period_stats()
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

// Standard IMA ADPCM index table and step table
static constexpr int ADPCM_INDEX_TABLE_8BIT[256] = {
    // 0-15
    0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1,
    // 16-31
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    // 32-47
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    // 48-63
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    // 64-79
    2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
    // 80-95
    4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5,
    // 96-111
    6, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10,
    // 112-127
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    // 128-143
    0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1,
    // 144-159
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    // 160-175
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    // 176-191
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    // 192-207
    2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
    // 208-223
    4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5,
    // 224-239F
    6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7,
    // 240-255
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};
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
    return channels * ADPCM_HEADER_SIZE_PER_CHANNEL + (frames * channels);
}

uint8_t NetEncoder::encode_sample(int16_t sample, NetEncoder::State &state)
{
    int diff = sample - state.predictor;
    uint8_t code = 0;

    if (diff < 0)
    {
        code = 0x80;
        diff = -diff;
    }

    int step = ADPCM_STEP_TABLE[state.step_index];

    for (int i = 6; i >= 0; i--)
    {
        int temp = step >> (6 - i);
        if (diff >= temp)
        {
            code |= (1 << i);
            diff -= temp;
        }
    }

    diff = 0;
    for (int i = 6; i >= 0; i--)
    {
        if (code & (1 << i))
        {
            diff += step >> (6 - i);
        }
    }
    diff += step >> 7;
    // std::cout<<"code:\t"<<(int)code<<"\t\tdiff:\t"<<recon_diff<<std::endl;

    if (code & 0x80)
        state.predictor -= diff;
    else
        state.predictor += diff;

    if (state.predictor > 32767)
        state.predictor = 32767;
    else if (state.predictor < -32768)
        state.predictor = -32768;

    state.step_index += ADPCM_INDEX_TABLE_8BIT[code];

    if (state.step_index < 0)
        state.step_index = 0;
    else if (state.step_index > 88)
        state.step_index = 88;

    return code;
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
    for (unsigned int i = 0; i < samples_to_encode; i++)
    {
        unsigned int ch = i % channels;
        *out++ = encode_sample(pcm_data[i], encode_states[ch]);
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

    int diff = step >> 7;

    for (int i = 0; i < 7; i++)
    {
        if (code & (1 << i))
        {
            diff += step >> (6 - i);
        }
    }

    if (code & 0x80)
        state.predictor -= diff;
    else
        state.predictor += diff;

    if (state.predictor > 32767)
        state.predictor = 32767;
    else if (state.predictor < -32768)
        state.predictor = -32768;

    state.step_index += ADPCM_INDEX_TABLE_8BIT[code];

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
    out_frames = static_cast<unsigned int>(adpcm_data_size / channels);
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
        decode_buffer[sample_index++] = decode_sample(byte, decode_states[ch]);
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

// FEC Packet header validation
bool FecPacketHeader::validate(const char *data, size_t length)
{
    if (length < sizeof(FecPacketHeader))
    {
        AUDIO_DEBUG_PRINT("Received undersized FEC packet (%zu bytes)", length);
        return false;
    }

    auto header = *reinterpret_cast<const FecPacketHeader *>(data);

    if (header.magic_num != FEC_MAGIC_NUM)
    {
        AUDIO_DEBUG_PRINT("Invalid FEC packet magic number: 0x%02X", header.magic_num);
        return false;
    }

    if (header.packet_count == 0 || header.packet_count > FEC_GROUP_SIZE)
    {
        AUDIO_DEBUG_PRINT("Invalid FEC packet count: %u", header.packet_count);
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

NetWorker::NetWorker(asio::io_context &io_context, uint16_t port)
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

        socket->bind(udp::endpoint(udp::v4(), port), ec);
        if (ec)
        {
            AUDIO_ERROR_PRINT("Failed to bind socket on port %d: %s", port, ec.message().c_str());
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

void NetWorker::report_conns(std::vector<InfoLabel> &result)
{
    std::lock_guard<std::mutex> lock(senders_mutex);
    for (const auto &pair : senders)
    {
        const auto &context = pair.second;
        for (const auto &dest : context.destinations)
        {
            result.emplace_back(0, pair.first, dest.endpoint.address().to_v4().to_uint(), dest.receiver_token, true,
                                false, false);
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
    header.sample_rate = static_cast<uint8_t>((context.sample_rate) / 1000);
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

    // FEC处理
    if (context.fec_enabled)
    {
        // 将完整数据包添加到FEC编码器
        std::vector<char> packet_data(sizeof(header) + encoded_size);
        std::memcpy(packet_data.data(), &header, sizeof(header));
        std::memcpy(packet_data.data() + sizeof(header), encoded_data, encoded_size);

        // 添加数据包到FEC编码
        context.fec_encoder->add_packet(packet_data.data(), packet_data.size(), header.sequence);

        // 获取当前FEC组大小
        unsigned int current_group_size = context.fec_encoder->get_group_size();

        // 每隔一定数量的包发送一个FEC包
        if (header.sequence % current_group_size == current_group_size - 1)
        {
            size_t fec_size;
            const char *fec_data = context.fec_encoder->generate_fec_packet(fec_size);

            if (fec_data && fec_size > 0)
            {
                send_fec_packet(sender_id, fec_data, fec_size);
                context.fec_encoder->reset();
            }
        }
    }

    return {RetCode::OK, "Data sent"};
}

// 发送FEC数据包
void NetWorker::send_fec_packet(uint8_t sender_id, const char *data, size_t length)
{
    if (!data || length == 0 || !is_ready())
    {
        return;
    }

    auto it = senders.find(sender_id);
    if (it == senders.end())
    {
        return;
    }

    auto &context = it->second; // 创建FEC包头
    FecPacketHeader fec_header{};
    fec_header.magic_num = FEC_MAGIC_NUM;
    fec_header.sender_id = sender_id;
    fec_header.group_id = static_cast<uint8_t>(context.isequence % 256);

    // 获取当前FEC组大小
    unsigned int current_group_size = context.fec_encoder->get_group_size();
    fec_header.packet_count = static_cast<uint8_t>(current_group_size);
    fec_header.seq_base = context.isequence - current_group_size; // 计算组的起始序列号
    fec_header.fec_sequence = context.isequence;
    fec_header.timestamp =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();

    std::array<asio::const_buffer, 2> buffers{asio::buffer(&fec_header, sizeof(fec_header)),
                                              asio::buffer(data, length)};

    for (const auto &dest : context.destinations)
    {
        socket->async_send_to(buffers, dest.endpoint, [](const asio::error_code &error, std::size_t /*bytes*/) {
            if (error)
            {
                AUDIO_DEBUG_PRINT("FEC send error: %s", error.message().c_str());
            }
        });
    }

    AUDIO_DEBUG_PRINT("Sent FEC packet for sender %u, group %u, base seq %u", sender_id, fec_header.group_id,
                      fec_header.seq_base);
}

// 处理FEC数据包
void NetWorker::handle_fec_packet(const FecPacketHeader *header, const char *data, size_t bytes)
{
    if (!header || !data || bytes == 0)
    {
        return;
    }

    uint8_t sender_id = header->sender_id;

    // 检查是否有相应发送者的解码器
    std::lock_guard<std::mutex> lock(decoders_mutex);
    auto it = decoders.find(sender_id);
    if (it == decoders.end())
    {
        return;
    }

    // 尝试恢复丢失的包
    auto &decoder_context = it->second;
    if (decoder_context.fec_decoder->process_fec_packet(header, data, bytes))
    {
        // 查看是否有恢复的包
        const RecoveryPacket *recovered = decoder_context.fec_decoder->get_recovered_packet();

        while (recovered)
        {
            // 恢复的包应该是一个完整的网络包，包含头部和数据
            if (recovered->data.size() >= sizeof(NetPacketHeader))
            {
                const NetPacketHeader *rec_header = reinterpret_cast<const NetPacketHeader *>(recovered->data.data());

                // 验证恢复的包是否有效
                if (rec_header->magic_num == NET_MAGIC_NUM)
                {
                    // 解码恢复的音频数据
                    const uint8_t *rec_adpcm_data =
                        reinterpret_cast<const uint8_t *>(recovered->data.data() + sizeof(NetPacketHeader));
                    size_t rec_adpcm_size = recovered->data.size() - sizeof(NetPacketHeader);

                    uint32_t ip_address =
                        sender_endpoint.address().is_v4() ? sender_endpoint.address().to_v4().to_uint() : 0;
                    AudioBandWidth sample_enum = byte_to_bandwidth(rec_header->sample_rate);

                    // 使用通用方法处理恢复的音频数据
                    process_and_deliver_audio(rec_header->sender_id, rec_header->receiver_id, rec_header->channels,
                                              rec_header->sequence, rec_header->timestamp, rec_adpcm_data,
                                              rec_adpcm_size, ip_address, sample_enum);

                    AUDIO_DEBUG_PRINT("Delivered recovered packet seq %u from sender %u to receiver %u",
                                      rec_header->sequence, rec_header->sender_id, rec_header->receiver_id);
                }
            }

            // 检查是否还有其他恢复的包
            recovered = decoder_context.fec_decoder->get_recovered_packet();
        }
    }

    // 定期清理过期的包
    decoder_context.fec_decoder->cleanup();
}

// FEC Encoder
FecEncoder::FecEncoder(unsigned int max_packet_size)
    : fec_buffer(max_packet_size, 0), packet_mask(FEC_GROUP_SIZE, false), packet_count(0),
      max_packet_size(max_packet_size), current_group_id(0), base_sequence(0), group_size(FEC_GROUP_SIZE)
{
}

void FecEncoder::add_packet(const char *data, size_t length, uint32_t sequence)
{
    if (length > max_packet_size || !data)
    {
        return;
    }

    // 首个包初始化FEC组
    if (packet_count == 0)
    {
        std::fill(fec_buffer.begin(), fec_buffer.end(), char(0));
        std::fill(packet_mask.begin(), packet_mask.end(), false);
        base_sequence = sequence;
    }

    unsigned int packet_index = sequence - base_sequence;

    // 检查是否在当前FEC组内
    if (packet_index >= group_size)
    {
        // 如果序列号超出当前组范围，重置组
        if (packet_count > 0)
        {
            reset();
        }
        base_sequence = sequence;
        packet_index = 0;
    }

    // 标记已处理
    packet_mask[packet_index] = true;

    // 异或操作 - 简单FEC实现
    for (size_t i = 0; i < length; ++i)
    {
        fec_buffer[i] ^= data[i];
    }

    packet_count++; // 防止packet_count溢出
    if (packet_count > group_size)
    {
        packet_count = group_size;
    }
}

const char *FecEncoder::generate_fec_packet(size_t &out_size)
{
    if (packet_count < 2) // 至少需要两个数据包才能生成有效的FEC包
    {
        out_size = 0;
        return nullptr;
    }

    out_size = max_packet_size;
    current_group_id++; // 更新组ID

    // 成功生成FEC包，但不重置缓冲区，让下一组数据继续使用
    return fec_buffer.data();
}

void FecEncoder::reset()
{
    std::fill(fec_buffer.begin(), fec_buffer.end(), char(0));
    std::fill(packet_mask.begin(), packet_mask.end(), false);
    packet_count = 0;
}

// 设置FEC组大小
void FecEncoder::set_group_size(unsigned int size)
{
    if (size > 0 && size <= FEC_GROUP_SIZE)
    {
        if (size != group_size)
        {
            // 如果改变了组大小，重置编码器状态
            reset();
            group_size = size;
            packet_mask.resize(group_size, false);
        }
    }
}

// 获取当前FEC组大小
unsigned int FecEncoder::get_group_size() const
{
    return group_size;
}

// 设置包缓冲大小
void FecEncoder::set_packet_size(unsigned int size)
{
    if (size > 0 && size != max_packet_size)
    {
        max_packet_size = size;
        fec_buffer.resize(max_packet_size, 0);
        // 改变缓冲区大小后需要重置状态
        reset();
    }
}

// FEC Decoder
FecDecoder::FecDecoder(unsigned int max_packet_size) : max_packet_size(max_packet_size)
{
}

void FecDecoder::process_data_packet(uint32_t sequence, const char *data, size_t length)
{
    // 存储最近收到的数据包以便FEC恢复
    if (data && length > 0 && length <= max_packet_size)
    {
        // 检查是否已经存在相同序列号的包
        for (const auto &packet : packet_buffer)
        {
            if (packet.sequence == sequence)
            {
                return; // 已经存在该序列号，无需再存
            }
        }

        packet_buffer.emplace_back(sequence, data, length);

        // 保持缓冲区大小合理
        if (packet_buffer.size() > FEC_GROUP_SIZE * 3)
        {
            packet_buffer.pop_front();
        }
    }
}

bool FecDecoder::process_fec_packet(const FecPacketHeader *header, const char *fec_data, size_t fec_length)
{
    if (!header || !fec_data || fec_length == 0)
    {
        return false;
    }

    // 获取FEC组信息
    uint32_t seq_base = header->seq_base;
    uint8_t packet_count = header->packet_count;

    // 标记组中哪些包我们已经接收到
    std::vector<bool> received_packets(packet_count, false);
    std::vector<const RecoveryPacket *> group_packets;
    uint32_t missing_seq = 0;
    int missing_count = 0;

    // 找出当前FEC组中哪些包已经接收到
    for (uint32_t i = 0; i < packet_count; ++i)
    {
        uint32_t seq = seq_base + i;
        bool found = false;

        for (const auto &packet : packet_buffer)
        {
            if (packet.sequence == seq)
            {
                received_packets[i] = true;
                group_packets.push_back(&packet);
                found = true;
                break;
            }
        }

        if (!found)
        {
            missing_count++;
            missing_seq = seq; // 记录缺失的序列号
        }
    }

    // 只能恢复单个丢失的包
    if (missing_count == 1)
    {
        // 尝试恢复丢失的包
        if (try_recover_packet(missing_seq, fec_data, group_packets))
        {
            return true;
        }
    }

    return false;
}

bool FecDecoder::try_recover_packet(uint32_t missing_seq, const char *fec_data,
                                    const std::vector<const RecoveryPacket *> &group_packets)
{
    // 已经恢复过这个序列号，避免重复恢复
    for (const auto &recovered : recovered_packets)
    {
        if (recovered.sequence == missing_seq)
        {
            return false;
        }
    }

    // 使用FEC数据恢复丢失的包
    std::vector<char> recovered_data(max_packet_size, 0);

    // 初始化恢复数据为FEC数据
    std::copy(fec_data, fec_data + max_packet_size, recovered_data.begin());

    // 通过异或操作恢复丢失的包
    for (const auto &packet : group_packets)
    {
        const size_t len = std::min(packet->data.size(), recovered_data.size());
        for (size_t i = 0; i < len; ++i)
        {
            recovered_data[i] ^= packet->data[i];
        }
    }

    // 将恢复的包加入恢复队列
    recovered_packets.emplace_back(missing_seq, recovered_data.data(), recovered_data.size());

    // 保持恢复包队列大小合理
    if (recovered_packets.size() > 20)
    {
        recovered_packets.pop_front();
    }

    AUDIO_DEBUG_PRINT("Recovered packet with sequence: %u", missing_seq);
    return true;
}

const RecoveryPacket *FecDecoder::get_recovered_packet()
{
    if (recovered_packets.empty())
    {
        return nullptr;
    }

    const RecoveryPacket *packet = &recovered_packets.front();
    recovered_packets.pop_front();
    return packet;
}

void FecDecoder::cleanup(std::chrono::milliseconds max_age)
{
    auto now = std::chrono::steady_clock::now();

    // 清理过期的数据包
    while (!packet_buffer.empty())
    {
        const auto &oldest = packet_buffer.front();
        if (now - oldest.arrival_time > max_age)
        {
            packet_buffer.pop_front();
        }
        else
        {
            break;
        }
    }

    // 清理过期的恢复包
    while (!recovered_packets.empty())
    {
        const auto &oldest = recovered_packets.front();
        if (now - oldest.arrival_time > max_age)
        {
            recovered_packets.pop_front();
        }
        else
        {
            break;
        }
    }
}

// NetWorker 的 FEC 相关方法
RetCode NetWorker::enable_fec(uint8_t sender_id, bool enable)
{
    std::lock_guard<std::mutex> lock(senders_mutex);
    auto it = senders.find(sender_id);
    if (it == senders.end())
    {
        return {RetCode::FAILED, "Sender not registered"};
    }

    it->second.fec_enabled = enable;

    AUDIO_INFO_PRINT("FEC %s for sender %u", enable ? "enabled" : "disabled", sender_id);
    return RetCode::OK;
}

RetCode NetWorker::set_fec_params(uint8_t sender_id, unsigned int group_size)
{
    if (group_size == 0 || group_size > FEC_GROUP_SIZE)
    {
        return {RetCode::FAILED, "Invalid FEC group size"};
    }

    std::lock_guard<std::mutex> lock(senders_mutex);
    auto it = senders.find(sender_id);
    if (it == senders.end())
    {
        return {RetCode::FAILED, "Sender not registered"};
    }

    // 应用新的FEC参数
    auto &context = it->second;

    // 先保存原始的启用状态
    bool was_enabled = context.fec_enabled;

    // 临时禁用FEC，避免在配置过程中触发FEC数据包的发送
    context.fec_enabled = false;

    // 设置FEC组大小
    context.fec_encoder->set_group_size(group_size);

    // 恢复FEC启用状态
    context.fec_enabled = was_enabled;

    AUDIO_INFO_PRINT("Updated FEC parameters for sender %u, group size: %u", sender_id, group_size);

    return {RetCode::OK, "FEC parameters updated"};
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

    // 首先检查这是一个普通数据包还是FEC包
    if (bytes_transferred >= 1)
    {
        uint8_t magic_num = *reinterpret_cast<const uint8_t *>(receive_buffer.get());

        // 处理FEC包
        if (magic_num == FEC_MAGIC_NUM && bytes_transferred >= sizeof(FecPacketHeader))
        {
            const FecPacketHeader *fec_header = reinterpret_cast<const FecPacketHeader *>(receive_buffer.get());

            if (FecPacketHeader::validate(receive_buffer.get(), bytes_transferred))
            {
                const char *fec_data = receive_buffer.get() + sizeof(FecPacketHeader);
                size_t fec_data_size = bytes_transferred - sizeof(FecPacketHeader);

                handle_fec_packet(fec_header, fec_data, fec_data_size);
            }
            goto exit;
        }
    }

    // 处理普通音频数据包
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
        // 将接收的数据包添加到FEC解码器以备恢复使用
        auto &context = get_decoder(sender_id, channels);
        context.fec_decoder->process_data_packet(header->sequence, receive_buffer.get(), bytes_transferred);

        // 处理并分发音频数据
        uint32_t ip_address = sender_endpoint.address().is_v4() ? sender_endpoint.address().to_v4().to_uint() : 0;
        process_and_deliver_audio(sender_id, receiver_id, channels, header->sequence, header->timestamp, adpcm_data,
                                  adpcm_size, ip_address, sample_enum);
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

// 处理并分发已解码的音频数据
void NetWorker::process_and_deliver_audio(uint8_t sender_id, uint8_t receiver_id, uint8_t channels, uint32_t sequence,
                                          uint64_t timestamp, const uint8_t *adpcm_data, size_t adpcm_size,
                                          uint32_t source_ip, AudioBandWidth sample_enum)
{
    auto &decoder_context = get_decoder(sender_id, channels);

    // 更新统计信息
    decoder_context.update_stats(sequence, timestamp);

    // 解码音频数据
    unsigned int decode_frames = 0;
    auto decoded_data = decoder_context.decoder->decode(adpcm_data, adpcm_size, decode_frames);

    if (decoded_data && decode_frames > 0)
    {
        std::lock_guard<std::mutex> lock(receivers_mutex);
        auto it = receivers.find(receiver_id);
        if (it != receivers.end() && it->second)
        {
            (it->second)(sender_id, channels, decode_frames, enum2val(sample_enum), decoded_data, source_ip);
        }
    }
}