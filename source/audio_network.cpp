#include "audio_network.h"
#include "audio_stream.h"
#include <algorithm>
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
        int16_t predictor_value = static_cast<int16_t>(in[0] | (in[1] << 8));
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

// NetWorker
using udp = asio::ip::udp;

// FEC Processor implementation
FECProcessor::FECProcessor() : fec_size(0), has_fec(false), base_sequence(0), packet_count(0)
{
    reset();
}

void FECProcessor::reset()
{
    memset(received, false, sizeof(received));
    has_fec = false;
    fec_size = 0;
    packet_count = 0;
    memset(packet_sizes, 0, sizeof(packet_sizes));
}

bool FECProcessor::add_packet(const uint8_t *data, size_t size, uint32_t sequence)
{
    if (packet_count >= GROUP_SIZE || size > MAX_PACKET_SIZE)
        return false;

    if (packet_count == 0)
    {
        base_sequence = sequence;
        fec_size = size;
        memset(fec_data, 0, MAX_PACKET_SIZE);
    }
    else if (size != fec_size)
    {
        return false; // All packets must have the same size
    }

    memcpy(packets[packet_count], data, size);
    packet_sizes[packet_count] = size;

    // XOR into FEC data
    for (size_t i = 0; i < size; ++i)
    {
        fec_data[i] ^= data[i];
    }

    packet_count++;
    return true;
}

bool FECProcessor::add_received_packet(const uint8_t *data, size_t size, uint32_t sequence)
{
    uint32_t group_base = (sequence / GROUP_SIZE) * GROUP_SIZE;
    uint32_t group_index = sequence - group_base;

    if (group_index >= GROUP_SIZE || size > MAX_PACKET_SIZE)
        return false;

    // If it's a new group, reset
    if (base_sequence != group_base)
    {
        reset();
        base_sequence = group_base;
    }

    if (!received[group_index])
    {
        memcpy(packets[group_index], data, size);
        packet_sizes[group_index] = size;
        received[group_index] = true;
        return true;
    }
    return false;
}

bool FECProcessor::add_fec_packet(const uint8_t *fec, size_t size)
{
    if (size > MAX_PACKET_SIZE)
        return false;

    if (!has_fec)
    {
        memcpy(fec_data, fec, size);
        fec_size = size;
        has_fec = true;
        return true;
    }
    return false;
}

bool FECProcessor::can_recover() const
{
    if (!has_fec)
        return false;

    int missing_count = 0;
    for (bool i : received)
    {
        if (!i)
            missing_count++;
    }
    return missing_count == 1; // Can only recover one missing packet
}

bool FECProcessor::recover_missing(uint8_t *output, size_t &size)
{
    if (!can_recover())
        return false;

    int missing_idx = -1;
    for (size_t i = 0; i < GROUP_SIZE; ++i)
    {
        if (!received[i])
        {
            missing_idx = static_cast<int>(i);
            break;
        }
    }

    if (missing_idx < 0)
        return false;

    size = fec_size;
    memcpy(output, fec_data, fec_size);

    // XOR all received packets to recover missing packet
    for (size_t i = 0; i < GROUP_SIZE; ++i)
    {
        if (i != static_cast<size_t>(missing_idx) && received[i])
        {
            size_t min_size = std::min(packet_sizes[i], size);
            for (size_t j = 0; j < min_size; ++j)
            {
                output[j] ^= packets[i][j];
            }
        }
    }

    return true;
}

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

NetWorker::NetWorker(asio::io_context &io_context, uint16_t port, const std::string &local_ip)
    : retry_count(0), io_context(io_context), running(false), local_session_id(0),
      receive_buffer(new char[NETWORK_MAX_BUFFER_SIZE]), stats_timer(io_context)
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

        asio::error_code addr_ec;
        auto addr = asio::ip::make_address_v4(local_ip, addr_ec);
        if (!addr_ec)
        {
            local_session_id = addr.to_uint();
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
    receivers.clear();

    {
        std::lock_guard<std::mutex> lock(senders_mutex);
        senders.clear();
    }

    {
        std::lock_guard<std::mutex> lock(decoders_mutex);
        decoders.clear();
    }

    AUDIO_INFO_PRINT("NetWorker stopped");
    return {RetCode::OK, "Stopped"};
}

void NetWorker::report()
{
    std::lock_guard<std::mutex> lock(decoders_mutex);
    for (auto &pair : decoders)
    {
        auto stats = pair.second.get_period_stats();
        auto composite_key = pair.first;

        if (stats.packets_received > 0)
        {
            AUDIO_INFO_PRINT(
                "NETSTATS(0x%08X:%u) : [loss] %.2f%%, [jitter] %.2fms, [received] %u, [lost] %u, [out-of-order] %u",
                InfoLabel::extract_ip(composite_key), InfoLabel::extract_token(composite_key), stats.packet_loss_rate,
                stats.average_jitter, stats.packets_received, stats.packets_lost, stats.packets_out_of_order);
        }
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
        return {RetCode::EPARAM, "Failed to encode audio data"};
    }

    uint32_t sequence = context.isequence++;
    uint64_t timestamp =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    auto channels = static_cast<uint8_t>(context.channels);
    auto sample_rate = static_cast<uint8_t>((context.sample_rate) / 1000);

    // Send original data packets
    for (const auto &dest : context.destinations)
    {
        send_data_packet(dest, sender_id, dest.receiver_token, sequence, timestamp, encoded_data, encoded_size,
                         channels, sample_rate, false);
    }

    // Add to FEC group
    if (context.fec_processor->add_packet(encoded_data, encoded_size, sequence))
    {
        // If FEC group is complete, send FEC packet
        if (context.fec_processor->is_complete())
        {
            uint32_t fec_sequence = context.fec_processor->get_base_sequence() + FECProcessor::GROUP_SIZE;
            for (const auto &dest : context.destinations)
            {
                send_data_packet(dest, sender_id, dest.receiver_token, fec_sequence, timestamp,
                                 context.fec_processor->get_fec_data(), context.fec_processor->get_fec_size(), channels,
                                 sample_rate, true);
            }
            context.fec_processor->reset(); // Reset FEC group
        }
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
            self->send_rtt_probes();
            self->start_stats_loop();
        }
    });
}

void NetWorker::send_rtt_probes()
{
    if (!is_ready())
    {
        return;
    }

    std::lock_guard<std::mutex> lock(senders_mutex);

    uint64_t current_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    std::set<asio::ip::udp::endpoint> unique_endpoints;
    for (const auto &sender_pair : senders)
    {
        for (const auto &dest : sender_pair.second.destinations)
        {
            unique_endpoints.insert(dest.endpoint);
        }
    }

    for (const auto &endpoint : unique_endpoints)
    {
        ProbePacket probe{};
        probe.magic_num = NET_PROBE_MAGIC;
        probe.sender_id = 0;
        probe.receiver_id = 0;
        probe.is_response = 0;
        probe.sequence = probe_sequence++;
        probe.timestamp = current_time;

        socket->async_send_to(asio::buffer(&probe, sizeof(probe)), endpoint,
                              [endpoint](const asio::error_code &error, std::size_t /*bytes*/) {
                                  if (error)
                                  {
                                      AUDIO_DEBUG_PRINT("RTT probe send error to %s:%d: %s",
                                                        endpoint.address().to_string().c_str(), endpoint.port(),
                                                        error.message().c_str());
                                  }
                              });
    }
}

void NetWorker::process_probe_packet(const ProbePacket *probe, const asio::ip::udp::endpoint &endpoint)
{
    if (!probe || !is_ready())
    {
        return;
    }

    if (probe->magic_num != NET_PROBE_MAGIC)
    {
        return;
    }

    if (probe->is_response == 0)
    {
        ProbePacket response{};
        response.magic_num = NET_PROBE_MAGIC;
        response.sender_id = probe->receiver_id;
        response.receiver_id = probe->sender_id;
        response.is_response = 1;
        response.sequence = probe->sequence;
        response.timestamp = probe->timestamp;

        socket->async_send_to(asio::buffer(&response, sizeof(response)), endpoint,
                              [](const asio::error_code &error, std::size_t /*bytes*/) {
                                  if (error)
                                  {
                                      AUDIO_DEBUG_PRINT("RTT probe response error: %s", error.message().c_str());
                                  }
                              });
    }
    else
    {
        auto now = std::chrono::steady_clock::now();
        uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        uint64_t send_time = probe->timestamp;
        auto rtt = static_cast<double>(current_time - send_time);
        uint32_t endpoint_ip = endpoint.address().is_v4() ? endpoint.address().to_v4().to_uint() : 0;
        auto &metrics = rtt_data[endpoint_ip];

        metrics.last_rtt = rtt;
        metrics.sample_count++;
        metrics.last_update_time = now;

        if (metrics.sample_count == 1)
        {
            metrics.avg_rtt = rtt;
        }
        else
        {
            metrics.avg_rtt = RttMetrics::alpha * rtt + (1.0 - RttMetrics::alpha) * metrics.avg_rtt;
        }

        AUDIO_INFO_PRINT("RTT to %X08: %.2f ms (avg: %.2f ms)", endpoint_ip, rtt, metrics.avg_rtt);
    }
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

    if (bytes_transferred >= sizeof(ProbePacket))
    {
        auto magic_num = static_cast<uint8_t>(receive_buffer.get()[0]);

        if (magic_num == NET_PROBE_MAGIC)
        {
            auto probe = reinterpret_cast<const ProbePacket *>(receive_buffer.get());
            process_probe_packet(probe, sender_endpoint);
        }
        else if (magic_num == NET_AUDIO_MAGIC || magic_num == NET_FEC_MAGIC)
        {
            auto data_header = reinterpret_cast<const DataPacket *>(receive_buffer.get());
            const auto *payload = reinterpret_cast<const uint8_t *>(receive_buffer.get() + sizeof(DataPacket));
            size_t payload_size = bytes_transferred - sizeof(DataPacket);
            auto sender_id = sender_endpoint.address().is_v4() ? sender_endpoint.address().to_v4().to_uint() : 0;
            process_data_packet(data_header, payload, payload_size, sender_id);
        }
    }

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

NetWorker::DecoderContext &NetWorker::get_decoder(uint8_t sender_id, uint32_t session_id, unsigned int channels)
{
    std::lock_guard<std::mutex> lock(decoders_mutex);

    uint64_t composite_key = InfoLabel::make_composite(session_id, sender_id);
    auto it = decoders.find(composite_key);
    if (it == decoders.end())
    {
        auto result = decoders.emplace(composite_key, DecoderContext(channels, NETWORK_MAX_FRAMES));
        return result.first->second;
    }

    return it->second;
}

// Unified packet processing methods
void NetWorker::process_data_packet(const DataPacket *header, const uint8_t *payload, size_t payload_size, uint32_t sender_ip)
{
    if (!validate_packet(header, payload, payload_size))
    {
        return;
    }

    uint32_t combined_session_id = (sender_ip & 0xFFFF0000) | (header->session_id & 0x0000FFFF);

    if (header->is_fec)
    {
        handle_fec_packet(header, payload, payload_size, combined_session_id);
    }
    else
    {
        handle_audio_packet(header, payload, payload_size, combined_session_id);
    }
}

bool NetWorker::validate_packet(const DataPacket *header, const uint8_t *payload, size_t payload_size)
{
    return header && payload && payload_size > 0 && header->channels > 0 && header->channels <= 2 &&
           payload_size <= FECProcessor::MAX_PACKET_SIZE;
}

void NetWorker::handle_audio_packet(const DataPacket *header, const uint8_t *payload, size_t payload_size, uint32_t combined_session_id)
{
    AudioBandWidth sample_enum = byte_to_bandwidth(header->sample_rate);
    if (sample_enum == AudioBandWidth::Unknown)
    {
        return;
    }

    auto &decoder_context = get_decoder(header->sender_id, combined_session_id, header->channels);

    // Add to FEC recovery group
    decoder_context.fec_processor->add_received_packet(payload, payload_size, header->sequence);

    // Process and deliver audio immediately
    process_and_deliver_audio(header->sender_id, header->receiver_id, header->channels, header->sequence,
                              header->timestamp, payload, payload_size, combined_session_id, sample_enum);
}

void NetWorker::handle_fec_packet(const DataPacket *header, const uint8_t *payload, size_t payload_size, uint32_t combined_session_id)
{
    AudioBandWidth sample_enum = byte_to_bandwidth(header->sample_rate);
    if (sample_enum == AudioBandWidth::Unknown)
    {
        return;
    }

    auto &decoder_context = get_decoder(header->sender_id, combined_session_id, header->channels);

    // Add FEC data and try recovery
    if (decoder_context.fec_processor->add_fec_packet(payload, payload_size))
    {
        if (decoder_context.fec_processor->can_recover())
        {
            static thread_local uint8_t recovered_data[FECProcessor::MAX_PACKET_SIZE];
            size_t recovered_size;
            if (decoder_context.fec_processor->recover_missing(recovered_data, recovered_size))
            {
                // Find which packet was missing - FEC sequence is GROUP_SIZE positions after the base
                uint32_t group_base = decoder_context.fec_processor->get_base_sequence();

                // Find the missing packet index and recover it
                for (uint32_t i = 0; i < FECProcessor::GROUP_SIZE; ++i)
                {
                    if (!decoder_context.fec_processor->is_packet_received(i))
                    {
                        uint32_t missing_seq = group_base + i;
                        process_and_deliver_audio(header->sender_id, header->receiver_id, header->channels, missing_seq,
                                                  header->timestamp, recovered_data, recovered_size, combined_session_id,
                                                  sample_enum);
                        break; // Only recover one missing packet
                    }
                }
            }
        }
    }
}

void NetWorker::process_and_deliver_audio(uint8_t sender_id, uint8_t receiver_id, uint8_t channels, uint32_t sequence,
                                          uint64_t timestamp, const uint8_t *adpcm_data, size_t adpcm_size,
                                          uint32_t session_id, AudioBandWidth sample_enum)
{
    if (!adpcm_data || adpcm_size == 0)
    {
        return;
    }

    auto &decoder_context = get_decoder(sender_id, session_id, channels);
    decoder_context.update_stats(sequence, timestamp);

    unsigned int frames = 0;
    const int16_t *decoded_pcm = decoder_context.decoder->decode(adpcm_data, adpcm_size, frames);
    if (!decoded_pcm || frames == 0)
    {
        AUDIO_ERROR_PRINT("Failed to decode audio data from sender %u (session 0x%08X)", sender_id, session_id);
        return;
    }

    auto receiver_it = receivers.find(receiver_id);
    if (receiver_it != receivers.end())
    {
        receiver_it->second(sender_id, channels, frames, enum2val(sample_enum), decoded_pcm, session_id);
    }
    else
    {
        AUDIO_DEBUG_PRINT("No receiver registered for token %u", receiver_id);
    }
}

void NetWorker::send_data_packet(const Destination &dest, uint8_t sender_id, uint8_t receiver_id, uint32_t sequence,
                                 uint64_t timestamp, const uint8_t *data, size_t size, uint8_t channels,
                                 uint8_t sample_rate, bool is_fec)
{
    if (!is_ready() || !data || size == 0)
    {
        return;
    }

    DataPacket header{};
    header.magic_num = is_fec ? NET_FEC_MAGIC : NET_AUDIO_MAGIC;
    header.sender_id = sender_id;
    header.receiver_id = receiver_id;
    header.is_fec = is_fec ? 1 : 0;
    header.channels = channels;
    header.sample_rate = sample_rate;
    header.sequence = sequence;
    header.session_id = local_session_id;
    header.timestamp = timestamp;

    std::array<asio::const_buffer, 2> buffers = {asio::buffer(&header, sizeof(header)), asio::buffer(data, size)};

    socket->async_send_to(
        buffers, dest.endpoint, [dest, sequence, is_fec](const asio::error_code &error, std::size_t /*bytes_sent*/) {
            if (error)
            {
                AUDIO_DEBUG_PRINT("Failed to send %s packet %u to %s:%d: %s", is_fec ? "FEC" : "audio", sequence,
                                  dest.endpoint.address().to_string().c_str(), dest.endpoint.port(),
                                  error.message().c_str());
            }
        });
}