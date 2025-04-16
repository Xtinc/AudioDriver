#include "audio_network.h"
#include "audio_wavfile.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

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
    
    // 收集每个块的SNR值进行统计
    std::vector<double> snr_values;
    
    // 用于记录能量水平和对应的SNR
    std::vector<std::pair<double, double>> energy_snr_pairs;

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

        // 计算当前块的信号能量
        double block_signal_power = 0.0;
        for (const auto& sample : original) {
            block_signal_power += static_cast<double>(sample) * sample;
        }
        block_signal_power /= original.size();
        
        double current_snr = calculate_snr(original, decoded);
        
        // 只有当信号能量大于阈值时才记录SNR，避免静音段的影响
        if (block_signal_power > 100.0) {
            snr_values.push_back(current_snr);
            energy_snr_pairs.emplace_back(block_signal_power, current_snr);
        }

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

    // 计算SNR统计信息
    std::cout << "\n\nSNR Distribution Statistics:" << std::endl;
    
    if (snr_values.empty()) {
        std::cout << "No valid SNR values collected." << std::endl;
    } else {
        // 排序用于百分位数计算
        std::sort(snr_values.begin(), snr_values.end());
        
        double min_snr = snr_values.front();
        double max_snr = snr_values.back();
        double median_snr = snr_values[snr_values.size() / 2];
        double sum = std::accumulate(snr_values.begin(), snr_values.end(), 0.0);
        double mean = sum / snr_values.size();
        
        // 计算标准差
        double sq_sum = std::inner_product(snr_values.begin(), snr_values.end(), snr_values.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / snr_values.size() - mean * mean);
        
        // 计算百分位数
        auto percentile = [&snr_values](double p) -> double {
            size_t idx = static_cast<size_t>(p * snr_values.size() / 100);
            return snr_values[idx];
        };
        
        std::cout << "  Total blocks with valid SNR: " << snr_values.size() << std::endl;
        std::cout << "  Min SNR: " << std::fixed << std::setprecision(2) << min_snr << " dB" << std::endl;
        std::cout << "  Max SNR: " << max_snr << " dB" << std::endl;
        std::cout << "  Average SNR: " << mean << " dB" << std::endl;
        std::cout << "  Median SNR: " << median_snr << " dB" << std::endl;
        std::cout << "  Standard Deviation: " << stdev << " dB" << std::endl;
        std::cout << "  10th percentile: " << percentile(10) << " dB" << std::endl;
        std::cout << "  25th percentile: " << percentile(25) << " dB" << std::endl;
        std::cout << "  75th percentile: " << percentile(75) << " dB" << std::endl;
        std::cout << "  90th percentile: " << percentile(90) << " dB" << std::endl;
        
        // 打印SNR分布直方图
        std::cout << "\nSNR Distribution Histogram:" << std::endl;
        
        const int num_bins = 10;
        const double bin_width = (max_snr - min_snr) / num_bins;
        std::vector<int> histogram(num_bins, 0);
        
        for (double snr : snr_values) {
            int bin = static_cast<int>((snr - min_snr) / bin_width);
            if (bin >= num_bins) bin = num_bins - 1;
            histogram[bin]++;
        }
        
        const int max_bar_width = 50;
        int max_count = *std::max_element(histogram.begin(), histogram.end());
        
        for (int i = 0; i < num_bins; i++) {
            double bin_start = min_snr + i * bin_width;
            double bin_end = bin_start + bin_width;
            int bar_width = static_cast<int>(static_cast<double>(histogram[i]) * max_bar_width / max_count);
            
            std::cout << std::fixed << std::setprecision(1) << std::setw(5) << bin_start << " - " 
                      << std::setw(5) << bin_end << " dB: " << std::string(bar_width, '#') 
                      << " (" << histogram[i] << ")" << std::endl;
        }
        
        // 输出信号能量与SNR的关系
        std::cout << "\nSignal Energy vs SNR (sample of points):" << std::endl;
        
        // 对能量-SNR对按能量排序
        std::sort(energy_snr_pairs.begin(), energy_snr_pairs.end());
        
        // 仅显示有代表性的点（最多20个点）
        const size_t max_points = 20;
        size_t step = energy_snr_pairs.size() / std::min(max_points, energy_snr_pairs.size());
        if (step < 1) step = 1;
        
        std::cout << "  Energy Level (dB)  |  SNR (dB)" << std::endl;
        std::cout << " ----------------------------" << std::endl;
        
        for (size_t i = 0; i < energy_snr_pairs.size(); i += step) {
            double energy_db = 10.0 * log10(energy_snr_pairs[i].first + 1e-10);
            std::cout << "  " << std::fixed << std::setprecision(2) << std::setw(10) 
                      << energy_db << "        |  " << std::setw(6) 
                      << energy_snr_pairs[i].second << std::endl;
        }
    }

    // Calculate overall SNR
    auto overall_snr = (total_noise_power < 1e-10 || total_signal_power < 1e-10)
                           ? 0.0f
                           : 10.0 * log10(total_signal_power / total_noise_power);

    std::cout << "\nProcessing complete!" << std::endl;
    std::cout << "Overall SNR: " << std::fixed << std::setprecision(2) << overall_snr << " dB" << std::endl;
    std::cout << "Original size: " << (total_frames * channels * 2) << " bytes" << std::endl;
    std::cout << "Encoded size: " << (total_frames * channels) << " bytes (8-bit ADPCM)" << std::endl;
    std::cout << "Compression ratio: 2:1" << std::endl;
    
    return 0;
}