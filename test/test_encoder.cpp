#include "audio_network.h"
#include "audio_wavfile.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// Calculate Signal-to-Noise Ratio (SNR) between original and decoded data
double calculate_snr(const std::vector<int16_t> &original, const std::vector<int16_t> &decoded)
{
    if (original.size() != decoded.size() || original.empty())
    {
        return 0.0f;
    }

    double signal_power = 0.0;
    double noise_power = 0.0;

    for (size_t i = 0; i < original.size(); ++i)
    {
        signal_power += static_cast<double>(original[i]) * original[i];
        double diff = static_cast<double>(original[i]) - decoded[i];
        noise_power += diff * diff;
    }

    if (noise_power < 1e-10 || signal_power < 1e-10)
    {
        return 0.0;
    }

    return 10.0 * log10(signal_power / noise_power);
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input_wav> <output_wav>" << std::endl;
        return 1;
    }

    const std::string input_file = argv[1];
    const std::string output_file = argv[2];

    // Open input WAV file
    WavFile input_wav;
    RetCode ret = input_wav.open(input_file, WavFile::mode::in);
    if (ret.err != RetCode::OK)
    {
        std::cerr << "Failed to open input file: " << ret.msg << std::endl;
        return 1;
    }

    // Get WAV file properties
    const uint16_t channels = input_wav.channel_number();
    const uint32_t sample_rate = input_wav.sample_rate();
    const uint64_t total_frames = input_wav.frame_number();

    std::cout << "Input WAV Info:" << std::endl;
    std::cout << "  Channels: " << channels << std::endl;
    std::cout << "  Sample Rate: " << sample_rate << " Hz" << std::endl;
    std::cout << "  Total Frames: " << total_frames << std::endl;
    std::cout << "  Bits Per Sample: " << input_wav.bits_per_sample() << " bits" << std::endl;

    // Create buffers for encoding and decoding
    const unsigned int buffer_frames = 1024; // Frames per processing block
    std::vector<int16_t> input_buffer(buffer_frames * channels);
    std::vector<int16_t> output_buffer(buffer_frames * channels);

    // Create encoder and decoder
    NetEncoder encoder(channels, buffer_frames);
    NetDecoder decoder(channels, buffer_frames);

    // Create output WAV file
    WavFile output_wav;
    output_wav.set_channel_number(channels);
    output_wav.set_sample_rate(sample_rate);
    output_wav.set_bits_per_sample(16); // 16-bit PCM
    ret = output_wav.open(output_file, WavFile::mode::out);
    if (ret.err != RetCode::OK)
    {
        std::cerr << "Failed to create output file: " << output_file << std::endl;
        return 1;
    }

    // Variables for overall SNR calculation
    double total_signal_power = 0.0;
    double total_noise_power = 0.0;
    uint64_t total_processed_frames = 0;

    // Process data in blocks
    for (uint64_t frame_pos = 0; frame_pos < total_frames; frame_pos += buffer_frames)
    {
        // Adjust size for the last block
        unsigned int current_frames =
            static_cast<unsigned int>(std::min(buffer_frames, (uint32_t)(total_frames - frame_pos)));

        if (current_frames == 0)
            break;

        // Read input frames
        input_wav.seek(frame_pos);
        ret = input_wav.read(input_buffer.data(), current_frames);
        if (ret.err != RetCode::OK)
        {
            std::cerr << "Error reading WAV file" << std::endl;
            return 1;
        }

        // Encode PCM data to ADPCM
        size_t encoded_size;
        const uint8_t *encoded_data = encoder.encode(input_buffer.data(), current_frames, encoded_size);

        if (!encoded_data)
        {
            std::cerr << "Error encoding data" << std::endl;
            return 1;
        }

        // Decode ADPCM back to PCM
        unsigned int decoded_frames;
        const int16_t *decoded_data = decoder.decode(encoded_data, encoded_size, decoded_frames);

        if (!decoded_data || decoded_frames == 0)
        {
            std::cerr << "Error decoding data" << std::endl;
            return 1;
        }

        // Copy data to output buffer
        for (unsigned int i = 0; i < current_frames * channels; ++i)
        {
            output_buffer[i] = decoded_data[i];
        }

        // Write to output WAV file
        output_wav.write(output_buffer.data(), current_frames);

        // Calculate SNR for current block
        std::vector<int16_t> original(input_buffer.begin(), input_buffer.begin() + current_frames * channels);
        std::vector<int16_t> decoded(output_buffer.begin(), output_buffer.begin() + current_frames * channels);

        double current_snr = calculate_snr(original, decoded);

        // Accumulate for overall SNR
        for (size_t i = 0; i < current_frames * channels; ++i)
        {
            total_signal_power += static_cast<double>(original[i]) * original[i];
            double diff = static_cast<double>(original[i]) - decoded[i];
            total_noise_power += diff * diff;
        }

        total_processed_frames += current_frames;

        // Display progress
        if (frame_pos % (buffer_frames * 10) == 0 || frame_pos + current_frames >= total_frames)
        {
            auto progress = 100.0 * static_cast<double>(frame_pos + current_frames) / total_frames;
            std::cout << "\rProgress: " << std::fixed << std::setprecision(1) << progress
                      << "%, Current Block SNR: " << std::setprecision(2) << current_snr << " dB   " << std::flush;
        }
    }

    // Calculate overall SNR
    auto overall_snr = (total_noise_power < 1e-10 || total_signal_power < 1e-10)
                           ? 0.0f
                           : 10.0 * log10(total_signal_power / total_noise_power);

    std::cout << "\nProcessing complete!" << std::endl;
    std::cout << "Overall SNR: " << std::fixed << std::setprecision(2) << overall_snr << " dB" << std::endl;
    std::cout << "Original size: " << (total_frames * channels * 2) << " bytes" << std::endl;

    // Calculate theoretical ADPCM size (4 bytes header per channel + 4 bits per sample)
    size_t adpcm_size = channels * 4 + (total_frames * channels + 1) / 2;
    std::cout << "ADPCM encoded size: " << adpcm_size << " bytes" << std::endl;
    std::cout << "Compression ratio: " << std::fixed << std::setprecision(2)
              << (static_cast<float>(total_frames * channels * 2) / adpcm_size) << ":1" << std::endl;

    return 0;
}