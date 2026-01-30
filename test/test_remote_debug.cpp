#include "audio_interface.h"
#include "audio_remote_interface.h"
#include "audio_wavfile.h"
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// Default local audio port for receiving network audio
constexpr unsigned short DEFAULT_LOCAL_AUDIO_PORT = 9000;

// Color codes for terminal output
#ifdef _WIN32
#include <windows.h>
#define COLOR_RESET ""
#define COLOR_GREEN ""
#define COLOR_YELLOW ""
#define COLOR_RED ""
#define COLOR_CYAN ""
#define COLOR_MAGENTA ""
#else
#define COLOR_RESET "\033[0m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RED "\033[31m"
#define COLOR_CYAN "\033[36m"
#define COLOR_MAGENTA "\033[35m"
#endif

/**
 * @brief Audio data analyzer with window-based analysis
 */
class AudioAnalyzer
{
  public:
    struct WindowStats
    {
        double peak_value;           // Peak amplitude in window (%)
        double rms_energy;           // Root mean square energy (%)
        double avg_energy;           // Average energy (%)
        uint64_t window_number;      // Window index
        uint32_t frame_count;        // Frames in this window
        uint32_t sample_count;       // Total samples in window
        double zero_crossing_rate;   // Zero crossings per second
        double crest_factor;         // Peak/RMS ratio (dB)
        double clipping_rate;        // Percentage of clipped samples
        double dynamic_range;        // Max - Min amplitude (dB)
        bool has_anomaly;            // Detected audio anomaly (clipping/distortion)
        std::string anomaly_reason;  // Description of detected anomaly
        std::string quality_level;   // Quality assessment: Excellent/Good/Acceptable/Poor
    };

    AudioAnalyzer(unsigned int window_ms = 100, unsigned int sample_rate = 48000, unsigned int channels = 2)
        : window_ms_(window_ms), sample_rate_(sample_rate), channels_(channels),
          total_windows_(0), total_energy_(0.0), max_peak_(0.0),
          samples_per_window_(sample_rate * channels * window_ms / 1000),
          current_sample_count_(0)
    {
        window_buffer_.reserve(samples_per_window_);
    }

    /**
     * @brief Add a frame to the analysis window
     * @param data Audio data buffer
     * @param channels Number of channels
     * @param frames Number of frames
     * @return Optional window statistics if window is complete
     */
    std::unique_ptr<WindowStats> add_frame(const int16_t *data, unsigned int channels, unsigned int frames)
    {
        if (!data || frames == 0 || channels == 0)
        {
            return nullptr;
        }

        unsigned int sample_count = frames * channels;
        
        // Add samples to window buffer
        for (unsigned int i = 0; i < sample_count; ++i)
        {
            window_buffer_.push_back(data[i]);
            current_sample_count_++;
        }
        
        window_frame_count_++;
        
        // Check if window is complete
        if (current_sample_count_ >= samples_per_window_)
        {
            auto stats = analyze_window();
            reset_window();
            return std::make_unique<WindowStats>(stats);
        }
        
        return nullptr;
    }

    /**
     * @brief Force analyze current window (useful at end of recording)
     */
    std::unique_ptr<WindowStats> flush_window()
    {
        if (window_buffer_.empty())
        {
            return nullptr;
        }
        auto stats = analyze_window();
        reset_window();
        return std::make_unique<WindowStats>(stats);
    }

    void reset()
    {
        total_windows_ = 0;
        total_energy_ = 0.0;
        max_peak_ = 0.0;
        reset_window();
    }

    uint64_t get_total_windows() const
    {
        return total_windows_;
    }
    double get_max_peak() const
    {
        return max_peak_;
    }
    double get_avg_energy() const
    {
        return total_windows_ > 0 ? total_energy_ / total_windows_ : 0.0;
    }
    unsigned int get_window_ms() const
    {
        return window_ms_;
    }

  private:
    void reset_window()
    {
        window_buffer_.clear();
        current_sample_count_ = 0;
        window_frame_count_ = 0;
    }

    WindowStats analyze_window()
    {
        WindowStats stats;
        stats.window_number = total_windows_++;
        stats.frame_count = window_frame_count_;
        stats.sample_count = static_cast<uint32_t>(window_buffer_.size());

        double sum_squares = 0.0;
        double sum_abs = 0.0;
        int16_t peak = 0;
        int16_t min_val = 32767;
        int16_t max_val = -32768;
        uint32_t zero_crossings = 0;
        uint32_t clipped_samples = 0;
        const int16_t clip_threshold = 32000; // ~97.6% of max amplitude

        for (size_t i = 0; i < window_buffer_.size(); ++i)
        {
            int16_t sample = window_buffer_[i];
            sum_squares += static_cast<double>(sample) * sample;
            sum_abs += std::abs(static_cast<double>(sample));

            if (std::abs(sample) > std::abs(peak))
            {
                peak = sample;
            }
            
            if (sample > max_val) max_val = sample;
            if (sample < min_val) min_val = sample;
            
            // Count zero crossings
            if (i > 0 && ((window_buffer_[i-1] >= 0 && sample < 0) || 
                         (window_buffer_[i-1] < 0 && sample >= 0)))
            {
                zero_crossings++;
            }
            
            // Count clipped samples
            if (std::abs(sample) >= clip_threshold)
            {
                clipped_samples++;
            }
        }

        // Basic statistics
        stats.peak_value = std::abs(peak) / 32768.0 * 100.0;
        stats.rms_energy = std::sqrt(sum_squares / stats.sample_count) / 32768.0 * 100.0;
        stats.avg_energy = (sum_abs / stats.sample_count) / 32768.0 * 100.0;
        
        // Zero Crossing Rate (per second)
        double window_duration_sec = window_ms_ / 1000.0;
        stats.zero_crossing_rate = zero_crossings / window_duration_sec;
        
        // Crest Factor (Peak/RMS in dB)
        if (stats.rms_energy > 0.01) {
            stats.crest_factor = 20.0 * std::log10(stats.peak_value / stats.rms_energy);
        } else {
            stats.crest_factor = 0.0;
        }
        
        // Clipping Rate
        stats.clipping_rate = static_cast<double>(clipped_samples) / stats.sample_count * 100.0;
        
        // Dynamic Range (dB)
        int32_t range = static_cast<int32_t>(max_val) - static_cast<int32_t>(min_val);
        if (range > 0) {
            stats.dynamic_range = 20.0 * std::log10(range / 65536.0) + 96.33; // Normalize to dB
        } else {
            stats.dynamic_range = 0.0;
        }
        
        // Quality assessment
        if (stats.peak_value >= 30.0 && stats.peak_value <= 70.0 &&
            stats.rms_energy >= 10.0 && stats.rms_energy <= 40.0 &&
            stats.crest_factor >= 10.0 && stats.crest_factor <= 15.0 &&
            stats.clipping_rate == 0.0 && stats.zero_crossing_rate < 5000.0) {
            stats.quality_level = "Excellent";
        } else if (stats.peak_value > 20.0 && stats.crest_factor < 18.0 && stats.clipping_rate < 0.1) {
            stats.quality_level = "Good";
        } else if (stats.peak_value > 10.0 && stats.crest_factor < 20.0 && stats.clipping_rate < 1.0) {
            stats.quality_level = "Acceptable";
        } else {
            stats.quality_level = "Poor";
        }
        
        // Intelligent anomaly detection with detailed diagnosis
        std::vector<std::string> anomaly_reasons;
        
        // Pattern 1: Packet loss (high ZCR with normal peak)
        if (stats.zero_crossing_rate > 8000.0 && stats.peak_value > 20.0 && stats.peak_value < 80.0) {
            anomaly_reasons.push_back("Packet Loss");
        }
        // Pattern 2: Clipping distortion
        else if (stats.clipping_rate > 1.0) {
            anomaly_reasons.push_back("Clipping");
        }
        // Pattern 3: Decoding error (sudden high crest factor)
        else if (stats.crest_factor > 20.0 && stats.peak_value > 50.0) {
            anomaly_reasons.push_back("Decode Error");
        }
        // Pattern 4: Over-compression (low peak with normal RMS)
        else if (stats.peak_value < 20.0 && stats.rms_energy > 10.0) {
            anomaly_reasons.push_back("Over-Compressed");
        }
        // Pattern 5: Quantization loss (low dynamic range)
        else if (stats.dynamic_range < 30.0 && stats.peak_value > 10.0) {
            anomaly_reasons.push_back("Quant Loss");
        }
        // Pattern 6: High distortion without clipping
        else if (stats.crest_factor > 20.0) {
            anomaly_reasons.push_back("Distortion");
        }
        // Pattern 7: General discontinuity
        else if (stats.zero_crossing_rate > 8000.0) {
            anomaly_reasons.push_back("Discontinuity");
        }
        
        stats.has_anomaly = !anomaly_reasons.empty();
        if (stats.has_anomaly) {
            stats.anomaly_reason = "";
            for (size_t i = 0; i < anomaly_reasons.size(); ++i) {
                if (i > 0) stats.anomaly_reason += "+";
                stats.anomaly_reason += anomaly_reasons[i];
            }
        } else {
            stats.anomaly_reason = "";
        }

        if (stats.peak_value > max_peak_)
        {
            max_peak_ = stats.peak_value;
        }
        total_energy_ += stats.rms_energy;

        return stats;
    }

    unsigned int window_ms_;
    unsigned int sample_rate_;
    unsigned int channels_;
    uint64_t total_windows_;
    double total_energy_;
    double max_peak_;
    
    // Window buffer
    std::vector<int16_t> window_buffer_;
    unsigned int samples_per_window_;
    unsigned int current_sample_count_;
    uint32_t window_frame_count_;
};

/**
 * @brief Remote audio recorder and debugger
 */
class RemoteAudioDebugger
{
  public:
    RemoteAudioDebugger(const std::string &server_ip, unsigned short server_port)
        : server_ip_(server_ip), server_port_(server_port), recording_(false), connected_(false),
          local_center_(nullptr), local_port_(DEFAULT_LOCAL_AUDIO_PORT)
    {
    }

    ~RemoteAudioDebugger()
    {
        stop_recording();
        disconnect_stream();
        cleanup_local_audio();
    }

    /**
     * @brief Connect to remote RPC server
     */
    bool connect_to_server()
    {
        try
        {
            std::cout << COLOR_CYAN << "Connecting to remote server " << server_ip_ << ":" << server_port_ << "..."
                      << COLOR_RESET << std::endl;

            remote_client_ = std::make_unique<AudioRemoteClient>(server_ip_, server_port_);
            std::cout << COLOR_GREEN << "✓ Connected to remote server" << COLOR_RESET << std::endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << COLOR_RED << "✗ Failed to connect: " << e.what() << COLOR_RESET << std::endl;
            return false;
        }
    }

    /**
     * @brief Initialize local audio center for receiving audio
     */
    bool init_local_audio(unsigned short local_port = DEFAULT_LOCAL_AUDIO_PORT)
    {
        try
        {
            local_port_ = local_port;
            std::cout << COLOR_CYAN << "Initializing local audio system on port " << local_port_ << "..."
                      << COLOR_RESET << std::endl;

            local_center_ = new AudioCenter(true, "0.0.0.0", local_port_);

            // Create local output stream to receive network data (fixed token 101)
            auto ret1 = local_center_->create(OToken(101), AudioDeviceName("null", 0), AudioBandWidth::Full,
                                              AudioPeriodSize::INR_20MS, 2, StreamFlags::Network | StreamFlags::CodecOPUS);
            if (!ret1)
            {
                std::cerr << COLOR_RED << "✗ Failed to create local output stream: " << ret1.what() << COLOR_RESET
                          << std::endl;
                return false;
            }

            // Create local input stream as listener (linked to output stream, fixed token 1)
            // This automatically sets up the listener relationship: OAStream -> IAStream
            auto ret2 = local_center_->create(IToken(1), OToken(101), StreamFlags::None);
            if (!ret2)
            {
                std::cerr << COLOR_RED << "✗ Failed to create linked input stream: " << ret2.what() << COLOR_RESET
                          << std::endl;
                return false;
            }

            // Prepare and start
            local_center_->prepare(false);
            local_center_->start();

            std::cout << COLOR_GREEN << "✓ Local audio system initialized" << COLOR_RESET << std::endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << COLOR_RED << "✗ Failed to initialize local audio: " << e.what() << COLOR_RESET << std::endl;
            return false;
        }
    }

    /**
     * @brief Connect remote stream to local stream
     */
    bool connect_stream(IToken remote_itoken, OToken remote_otoken)
    {
        if (!remote_client_)
        {
            std::cerr << COLOR_RED << "✗ Not connected to remote server" << COLOR_RESET << std::endl;
            return false;
        }

        if (!local_center_)
        {
            std::cerr << COLOR_RED << "✗ Local audio not initialized. Run 'init' first" << COLOR_RESET << std::endl;
            return false;
        }

        std::cout << COLOR_CYAN << "Connecting remote stream " << (int)remote_itoken.tok << " -> "
                  << (int)remote_otoken.tok << " to local port " << local_port_ << "..." << COLOR_RESET
                  << std::endl;

        // Use local machine's IP for receiving (server_ip_ is where we're sending the connection request)
        auto ret = remote_client_->connect_stream(remote_itoken, remote_otoken, "0.0.0.0", local_port_);
        if (ret == RetCode::OK)
        {
            connected_ = true;
            remote_itoken_ = remote_itoken;
            remote_otoken_ = remote_otoken;
            std::cout << COLOR_GREEN << "✓ Stream connected successfully" << COLOR_RESET << std::endl;
            return true;
        }
        else
        {
            std::cerr << COLOR_RED << "✗ Failed to connect stream: " << ret.what() << COLOR_RESET << std::endl;
            return false;
        }
    }

    /**
     * @brief Disconnect stream
     */
    bool disconnect_stream()
    {
        if (!connected_ || !remote_client_)
        {
            return true;
        }

        std::cout << COLOR_CYAN << "Disconnecting stream..." << COLOR_RESET << std::endl;
        auto ret =
            remote_client_->disconnect_stream(remote_itoken_, remote_otoken_, server_ip_, NETWORK_AUDIO_TRANS_PORT);

        if (ret == RetCode::OK)
        {
            connected_ = false;
            std::cout << COLOR_GREEN << "✓ Stream disconnected" << COLOR_RESET << std::endl;
            return true;
        }
        else
        {
            std::cerr << COLOR_RED << "✗ Failed to disconnect: " << ret.what() << COLOR_RESET << std::endl;
            return false;
        }
    }

    /**
     * @brief Start recording audio to WAV file
     */
    bool start_recording(const std::string &output_file, bool show_realtime_stats, unsigned int window_ms)
    {
        if (!local_center_)
        {
            std::cerr << COLOR_RED << "✗ Local audio not initialized" << COLOR_RESET << std::endl;
            return false;
        }

        if (recording_)
        {
            std::cerr << COLOR_RED << "✗ Already recording" << COLOR_RESET << std::endl;
            return false;
        }

        // Open WAV file for writing
        wav_file_ = std::make_unique<WavFile>();
        wav_file_->set_channel_number(2);
        wav_file_->set_sample_rate(48000);
        wav_file_->set_bits_per_sample(16);

        auto ret = wav_file_->open(output_file, WavFile::out);
        if (ret != RetCode::OK)
        {
            std::cerr << COLOR_RED << "✗ Failed to open WAV file: " << ret.what() << COLOR_RESET << std::endl;
            wav_file_.reset();
            return false;
        }

        // Reset analyzer with window size
        analyzer_ = AudioAnalyzer(window_ms, 48000, 2);
        analyzer_.reset();
        show_realtime_stats_ = show_realtime_stats;
        recording_ = true;
        output_file_ = output_file;

        // Register callback to receive audio data (fixed local token 1)
        // Use OBSERVER mode because data comes via direct_push from OAStream listener
        auto callback_ret = local_center_->register_callback(
            IToken(1),
            [](const int16_t *data, unsigned int chan_num, unsigned int frame_num, void *user_ptr) {
                auto *debugger = static_cast<RemoteAudioDebugger *>(user_ptr);
                debugger->audio_callback(data, chan_num, frame_num);
            },
            960, UsrCallBackMode::OBSERVER, this);

        if (callback_ret != RetCode::OK)
        {
            std::cerr << COLOR_RED << "✗ Failed to register callback: " << callback_ret.what() << COLOR_RESET
                      << std::endl;
            recording_ = false;
            wav_file_.reset();
            return false;
        }

        std::cout << COLOR_GREEN << "✓ Started recording to " << output_file << COLOR_RESET << std::endl;
        if (show_realtime_stats_)
        {
            std::cout << COLOR_YELLOW << "Real-time statistics enabled" << COLOR_RESET << std::endl;
        }

        return true;
    }

    /**
     * @brief Stop recording
     */
    bool stop_recording()
    {
        if (!recording_)
        {
            return false;
        }

        recording_ = false;
        wav_file_.reset();

        std::cout << COLOR_GREEN << "\n✓ Recording stopped: " << output_file_ << COLOR_RESET << std::endl;
        print_summary_stats();

        return true;
    }

    /**
     * @brief Set volume on remote stream
     */
    bool set_volume(AudioToken token, unsigned int volume)
    {
        if (!remote_client_)
        {
            std::cerr << COLOR_RED << "✗ Not connected to remote server" << COLOR_RESET << std::endl;
            return false;
        }

        auto ret = remote_client_->set_volume(token, volume);
        if (ret == RetCode::OK)
        {
            std::cout << COLOR_GREEN << "✓ Volume set to " << volume << "%" << COLOR_RESET << std::endl;
            return true;
        }
        else
        {
            std::cerr << COLOR_RED << "✗ Failed to set volume: " << ret.what() << COLOR_RESET << std::endl;
            return false;
        }
    }

    /**
     * @brief Mute/unmute remote stream
     */
    bool set_mute(AudioToken token, bool muted)
    {
        if (!remote_client_)
        {
            std::cerr << COLOR_RED << "✗ Not connected to remote server" << COLOR_RESET << std::endl;
            return false;
        }

        auto ret = remote_client_->mute(token, muted);
        if (ret == RetCode::OK)
        {
            std::cout << COLOR_GREEN << "✓ Stream " << (muted ? "muted" : "unmuted") << COLOR_RESET << std::endl;
            return true;
        }
        else
        {
            std::cerr << COLOR_RED << "✗ Failed to set mute: " << ret.what() << COLOR_RESET << std::endl;
            return false;
        }
    }

    /**
     * @brief Play audio file on remote system
     */
    bool play_audio(const std::string &file_path, int cycles, OToken otoken, AudioPriority priority)
    {
        if (!remote_client_)
        {
            std::cerr << COLOR_RED << "✗ Not connected to remote server" << COLOR_RESET << std::endl;
            return false;
        }

        auto ret = remote_client_->play(file_path, cycles, otoken, priority);
        if (ret == RetCode::OK)
        {
            std::cout << COLOR_GREEN << "✓ Playing " << file_path << " on remote" << COLOR_RESET << std::endl;
            return true;
        }
        else
        {
            std::cerr << COLOR_RED << "✗ Failed to play: " << ret.what() << COLOR_RESET << std::endl;
            return false;
        }
    }

    bool is_recording() const
    {
        return recording_;
    }

  private:
    void audio_callback(const int16_t *data, unsigned int chan_num, unsigned int frame_num)
    {
        if (!recording_ || !wav_file_)
        {
            return;
        }

        // Write to WAV file
        wav_file_->write(data, frame_num);

        // Add frame to analyzer window
        auto stats = analyzer_.add_frame(data, chan_num, frame_num);

        // Display statistics when window is complete
        if (stats && show_realtime_stats_)
        {
            std::cout << "Win " << std::setw(4) << stats->window_number 
                      << " (" << std::setw(2) << stats->frame_count << "f)" 
                      << " | Peak:" << std::setw(5) << std::fixed << std::setprecision(1) << stats->peak_value << "%"
                      << " RMS:" << std::setw(5) << std::fixed << std::setprecision(1) << stats->rms_energy << "%"
                      << " | CF:" << std::setw(4) << std::fixed << std::setprecision(1) << stats->crest_factor << "dB"
                      << " Clip:" << std::setw(4) << std::fixed << std::setprecision(1) << stats->clipping_rate << "%"
                      << " ZCR:" << std::setw(5) << std::fixed << std::setprecision(0) << stats->zero_crossing_rate << "Hz";
            
            // Show quality level
            std::cout << " [" << stats->quality_level << "]";
            
            // Show anomaly if detected
            if (stats->has_anomaly)
            {
                std::cout << " ⚠" << stats->anomaly_reason;
            }
            
            std::cout << "\r" << std::flush;
        }
    }

    void print_summary_stats()
    {
        // Flush any remaining data in window
        auto final_stats = analyzer_.flush_window();
        
        std::cout << "\n" << COLOR_MAGENTA << "═══════════════════════════════════════════" << COLOR_RESET << std::endl;
        std::cout << COLOR_MAGENTA << "          Recording Statistics" << COLOR_RESET << std::endl;
        std::cout << COLOR_MAGENTA << "═══════════════════════════════════════════" << COLOR_RESET << std::endl;
        std::cout << "Analysis Windows: " << analyzer_.get_total_windows() << " (" << analyzer_.get_window_ms() << "ms each)" << std::endl;
        std::cout << "Max Peak:         " << std::fixed << std::setprecision(2) << analyzer_.get_max_peak() << "%"
                  << std::endl;
        std::cout << "Average Energy:   " << std::fixed << std::setprecision(2) << analyzer_.get_avg_energy() << "%"
                  << std::endl;
        std::cout << "Output File:      " << output_file_ << std::endl;
        std::cout << COLOR_MAGENTA << "═══════════════════════════════════════════" << COLOR_RESET << std::endl;
    }

    void cleanup_local_audio()
    {
        if (local_center_)
        {
            delete local_center_;
            local_center_ = nullptr;
        }
    }

    std::string server_ip_;
    unsigned short server_port_;
    std::unique_ptr<AudioRemoteClient> remote_client_;
    AudioCenter *local_center_;
    unsigned short local_port_;

    bool recording_;
    bool connected_;
    bool show_realtime_stats_;
    IToken remote_itoken_;
    OToken remote_otoken_;

    std::unique_ptr<WavFile> wav_file_;
    std::string output_file_;
    AudioAnalyzer analyzer_;
};

/**
 * @brief Print help information
 */
void print_help()
{
    std::cout << "\n" << COLOR_CYAN << "═══════════════════════════════════════════════════════════" << COLOR_RESET
              << std::endl;
    std::cout << COLOR_CYAN << "    Remote Audio Debugger - Command Reference" << COLOR_RESET << std::endl;
    std::cout << COLOR_CYAN << "═══════════════════════════════════════════════════════════" << COLOR_RESET
              << std::endl;
    std::cout << "\nConnection Commands:" << std::endl;
    std::cout << "  connect <server_ip> <server_port>  - Connect to remote RPC server" << std::endl;
    std::cout << "  init [local_port]                  - Initialize local (default port: 9000)" << std::endl;
    std::cout << "  link <remote_in> <remote_out>      - Link remote stream to local" << std::endl;
    std::cout << "  unlink                             - Disconnect stream" << std::endl;

    std::cout << "\nRecording Commands:" << std::endl;
    std::cout << "  record <filename> [stats] [window_ms] - Start recording" << std::endl;
    std::cout << "                                       stats: show real-time window stats" << std::endl;
    std::cout << "                                       window_ms: analysis window size (default: 100ms)" << std::endl;
    std::cout << "  stop                               - Stop recording" << std::endl;

    std::cout << "\nControl Commands:" << std::endl;
    std::cout << "  volume <token> <0-100>             - Set volume" << std::endl;
    std::cout << "  mute <token> <on|off>              - Mute/unmute stream" << std::endl;
    std::cout << "  play <file> <token> [cycles]       - Play audio file on remote" << std::endl;

    std::cout << "\nGeneral Commands:" << std::endl;
    std::cout << "  help                               - Show this help" << std::endl;
    std::cout << "  quit/exit                          - Exit program" << std::endl;

    std::cout << "\nExamples:" << std::endl;
    std::cout << "  > connect 192.168.1.100 8080" << std::endl;
    std::cout << "  > init                           # Use default port 9000" << std::endl;
    std::cout << "  > link 1 101                     # Link remote tokens 1->101" << std::endl;
    std::cout << "  > record output.wav stats 200    # 200ms analysis window" << std::endl;
    std::cout << "  > volume 1 80                    # Control remote token 1" << std::endl;
    std::cout << "  > stop" << std::endl;
    std::cout << COLOR_CYAN << "═══════════════════════════════════════════════════════════" << COLOR_RESET
              << std::endl;
}

/**
 * @brief Parse command line input
 */
std::vector<std::string> parse_command(const std::string &input)
{
    std::vector<std::string> tokens;
    std::istringstream iss(input);
    std::string token;

    while (iss >> token)
    {
        tokens.push_back(token);
    }

    return tokens;
}

/**
 * @brief Main command loop
 */
void run_command_loop()
{
    std::unique_ptr<RemoteAudioDebugger> debugger;
    bool running = true;

    std::cout << COLOR_GREEN << "\n╔════════════════════════════════════════════════╗" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN << "║   Remote Audio Debugger v1.0                   ║" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN << "║   Type 'help' for available commands           ║" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN << "╚════════════════════════════════════════════════╝" << COLOR_RESET << std::endl;

    while (running)
    {
        std::cout << "\n" << COLOR_YELLOW << "debug> " << COLOR_RESET;
        std::string input;
        std::getline(std::cin, input);

        if (input.empty())
        {
            continue;
        }

        auto tokens = parse_command(input);
        if (tokens.empty())
        {
            continue;
        }

        std::string cmd = tokens[0];

        try
        {
            if (cmd == "quit" || cmd == "exit")
            {
                if (debugger && debugger->is_recording())
                {
                    std::cout << COLOR_YELLOW << "Stopping recording before exit..." << COLOR_RESET << std::endl;
                    debugger->stop_recording();
                }
                running = false;
            }
            else if (cmd == "help")
            {
                print_help();
            }
            else if (cmd == "connect")
            {
                if (tokens.size() < 3)
                {
                    std::cerr << COLOR_RED << "Usage: connect <server_ip> <server_port>" << COLOR_RESET << std::endl;
                    continue;
                }
                std::string ip = tokens[1];
                unsigned short port = static_cast<unsigned short>(std::stoi(tokens[2]));

                debugger = std::make_unique<RemoteAudioDebugger>(ip, port);
                debugger->connect_to_server();
            }
            else if (cmd == "init")
            {
                if (!debugger)
                {
                    std::cerr << COLOR_RED << "Connect to server first" << COLOR_RESET << std::endl;
                    continue;
                }
                unsigned short port = DEFAULT_LOCAL_AUDIO_PORT;
                if (tokens.size() > 1)
                {
                    port = static_cast<unsigned short>(std::stoi(tokens[1]));
                }
                debugger->init_local_audio(port);
            }
            else if (cmd == "link")
            {
                if (!debugger)
                {
                    std::cerr << COLOR_RED << "Connect to server first" << COLOR_RESET << std::endl;
                    continue;
                }
                if (tokens.size() < 3)
                {
                    std::cerr << COLOR_RED << "Usage: link <remote_in> <remote_out>"
                              << COLOR_RESET << std::endl;
                    continue;
                }
                IToken itoken(static_cast<unsigned char>(std::stoi(tokens[1])));
                OToken otoken(static_cast<unsigned char>(std::stoi(tokens[2])));

                debugger->connect_stream(itoken, otoken);
            }
            else if (cmd == "unlink")
            {
                if (!debugger)
                {
                    std::cerr << COLOR_RED << "No debugger instance" << COLOR_RESET << std::endl;
                    continue;
                }
                debugger->disconnect_stream();
            }
            else if (cmd == "record")
            {
                if (!debugger)
                {
                    std::cerr << COLOR_RED << "Connect to server first" << COLOR_RESET << std::endl;
                    continue;
                }
                if (tokens.size() < 2)
                {
                    std::cerr << COLOR_RED << "Usage: record <filename> [stats] [window_ms]" << COLOR_RESET << std::endl;
                    continue;
                }
                std::string filename = tokens[1];
                bool show_stats = false;
                unsigned int window_ms = 100; // Default 100ms
                
                // Parse optional parameters
                for (size_t i = 2; i < tokens.size(); ++i)
                {
                    if (tokens[i] == "stats")
                    {
                        show_stats = true;
                    }
                    else
                    {
                        try {
                            window_ms = std::stoi(tokens[i]);
                            if (window_ms < 10 || window_ms > 1000)
                            {
                                std::cerr << COLOR_YELLOW << "Warning: window_ms should be 10-1000ms, using default 100ms" << COLOR_RESET << std::endl;
                                window_ms = 100;
                            }
                        } catch (...) {
                            std::cerr << COLOR_YELLOW << "Warning: invalid window_ms, using default 100ms" << COLOR_RESET << std::endl;
                        }
                    }
                }
                
                debugger->start_recording(filename, show_stats, window_ms);
            }
            else if (cmd == "stop")
            {
                if (!debugger)
                {
                    std::cerr << COLOR_RED << "No debugger instance" << COLOR_RESET << std::endl;
                    continue;
                }
                debugger->stop_recording();
            }
            else if (cmd == "volume")
            {
                if (!debugger)
                {
                    std::cerr << COLOR_RED << "Connect to server first" << COLOR_RESET << std::endl;
                    continue;
                }
                if (tokens.size() < 3)
                {
                    std::cerr << COLOR_RED << "Usage: volume <token> <0-100>" << COLOR_RESET << std::endl;
                    continue;
                }
                unsigned char token_val = static_cast<unsigned char>(std::stoi(tokens[1]));
                unsigned int volume = std::stoi(tokens[2]);
                AudioToken token;
                token.tok = token_val;
                debugger->set_volume(token, volume);
            }
            else if (cmd == "mute")
            {
                if (!debugger)
                {
                    std::cerr << COLOR_RED << "Connect to server first" << COLOR_RESET << std::endl;
                    continue;
                }
                if (tokens.size() < 3)
                {
                    std::cerr << COLOR_RED << "Usage: mute <token> <on|off>" << COLOR_RESET << std::endl;
                    continue;
                }
                unsigned char token_val = static_cast<unsigned char>(std::stoi(tokens[1]));
                bool muted = (tokens[2] == "on" || tokens[2] == "true" || tokens[2] == "1");
                AudioToken token;
                token.tok = token_val;
                debugger->set_mute(token, muted);
            }
            else if (cmd == "play")
            {
                if (!debugger)
                {
                    std::cerr << COLOR_RED << "Connect to server first" << COLOR_RESET << std::endl;
                    continue;
                }
                if (tokens.size() < 3)
                {
                    std::cerr << COLOR_RED << "Usage: play <file> <token> [cycles]" << COLOR_RESET << std::endl;
                    continue;
                }
                std::string file = tokens[1];
                OToken otoken(static_cast<unsigned char>(std::stoi(tokens[2])));
                int cycles = (tokens.size() > 3) ? std::stoi(tokens[3]) : 0;
                debugger->play_audio(file, cycles, otoken, AudioPriority::MEDIUM);
            }
            else
            {
                std::cerr << COLOR_RED << "Unknown command: " << cmd << " (type 'help' for commands)" << COLOR_RESET
                          << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << COLOR_RED << "Error: " << e.what() << COLOR_RESET << std::endl;
        }
    }

    std::cout << COLOR_GREEN << "\nGoodbye!" << COLOR_RESET << std::endl;
}

/**
 * @brief Main function
 */
int main(int argc, char *argv[])
{
    // Enable Windows console color support
#ifdef _WIN32
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut != INVALID_HANDLE_VALUE)
    {
        DWORD dwMode = 0;
        if (GetConsoleMode(hOut, &dwMode))
        {
            dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
            SetConsoleMode(hOut, dwMode);
        }
    }
#endif

    std::cout << "Starting Remote Audio Debugger..." << std::endl;

    try
    {
        run_command_loop();
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}
