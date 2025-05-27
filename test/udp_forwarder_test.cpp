#include <asio.hpp>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using asio::ip::udp;
using namespace std;

struct ForwardRule
{
    string listen_ip;
    int listen_port;
    string target_ip;
    int target_port;
};

// Thread-local statistics to avoid cache ping-pong
struct ThreadLocalStats
{
    uint64_t packets_received = 0;
    uint64_t packets_forwarded = 0;
    uint64_t bytes_received = 0;
    uint64_t bytes_forwarded = 0;
    uint64_t forward_errors = 0;
};

// Global statistics aggregator
struct GlobalStatistics
{
    mutable mutex stats_mutex;
    vector<shared_ptr<ThreadLocalStats>> thread_stats;

    void register_thread_stats(shared_ptr<ThreadLocalStats> stats)
    {
        lock_guard<mutex> lock(stats_mutex);
        thread_stats.push_back(stats);
    }

    void get_aggregated_stats(uint64_t &packets_received, uint64_t &packets_forwarded, uint64_t &bytes_received,
                              uint64_t &bytes_forwarded, uint64_t &forward_errors) const
    {
        lock_guard<mutex> lock(stats_mutex);
        packets_received = packets_forwarded = bytes_received = bytes_forwarded = forward_errors = 0;

        for (const auto &stats : thread_stats)
        {
            packets_received += stats->packets_received;
            packets_forwarded += stats->packets_forwarded;
            bytes_received += stats->bytes_received;
            bytes_forwarded += stats->bytes_forwarded;
            forward_errors += stats->forward_errors;
        }
    }
};

class UDPForwarder
{
  public:
    UDPForwarder(asio::io_context &io_context, const ForwardRule &rule, GlobalStatistics &global_stats)
        : io_context_(io_context), rule_(rule), listen_socket_(io_context), forward_socket_(io_context),
          global_stats_(global_stats), local_stats_(make_shared<ThreadLocalStats>()),
          target_endpoint_(asio::ip::address::from_string(rule.target_ip),
                           static_cast<unsigned short>(rule.target_port))
    {
        // Register thread-local stats with global aggregator
        global_stats_.register_thread_stats(local_stats_);

        // Bind listening port
        udp::endpoint listen_endpoint(asio::ip::address::from_string(rule.listen_ip),
                                      static_cast<unsigned short>(rule.listen_port));
        listen_socket_.open(udp::v4());
        listen_socket_.bind(listen_endpoint);

        // Open forward socket once
        forward_socket_.open(udp::v4());

        cout << "UDP Forwarder started: " << rule.listen_ip << ":" << rule.listen_port << " -> " << rule.target_ip
             << ":" << rule.target_port << endl;

        start_receive();
    }

  private:
    void start_receive()
    {
        listen_socket_.async_receive_from(asio::buffer(recv_buffer_), sender_endpoint_,
                                          [this](asio::error_code ec, size_t bytes_transferred) {
                                              if (!ec && bytes_transferred > 0)
                                              {
                                                  // Update thread-local statistics
                                                  local_stats_->packets_received++;
                                                  local_stats_->bytes_received += bytes_transferred;

                                                  // Forward data to target address
                                                  forward_data(bytes_transferred);
                                              }
                                              start_receive(); // Continue receiving next packet
                                          });
    }

    void forward_data(size_t bytes)
    {
        // Create a dedicated buffer for this send operation to avoid race conditions
        auto send_buffer = make_shared<vector<char>>(recv_buffer_.begin(), recv_buffer_.begin() + bytes);

        // Reuse the existing forward socket instead of creating new ones
        forward_socket_.async_send_to(asio::buffer(*send_buffer), target_endpoint_,
                                      [this, send_buffer, bytes](asio::error_code ec, size_t bytes_sent) {
                                          if (ec)
                                          {
                                              local_stats_->forward_errors++;
                                              cerr << "Forward failed: " << ec.message() << endl;
                                          }
                                          else
                                          {
                                              local_stats_->packets_forwarded++;
                                              local_stats_->bytes_forwarded += bytes_sent;
                                              // Reduce console output for better performance
                                              // cout << "Forwarded " << bytes_sent << " bytes to " << rule_.target_ip
                                              // << ":"
                                              //      << rule_.target_port << endl;
                                          }
                                          // send_buffer will be automatically destroyed here
                                      });
    }
    asio::io_context &io_context_;
    ForwardRule rule_;
    udp::socket listen_socket_;
    udp::socket forward_socket_;
    udp::endpoint target_endpoint_;
    udp::endpoint sender_endpoint_;
    array<char, 1024> recv_buffer_;
    GlobalStatistics &global_stats_;
    shared_ptr<ThreadLocalStats> local_stats_;
};

vector<ForwardRule> read_config(const string &filename)
{
    vector<ForwardRule> rules;
    ifstream file(filename);
    string line;

    if (!file.is_open())
    {
        cerr << "Cannot open config file: " << filename << endl;
        return rules;
    }

    cout << "Reading config file: " << filename << endl;

    while (getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue; // Skip empty lines and comments

        // Format: Listen_IP:Port -> Target_IP:Port
        size_t arrow_pos = line.find(" -> ");
        if (arrow_pos == string::npos)
        {
            cerr << "Config line format error: " << line << endl;
            continue;
        }

        string listen_part = line.substr(0, arrow_pos);
        string target_part = line.substr(arrow_pos + 4);

        // Parse listen address
        size_t colon_pos = listen_part.find_last_of(':');
        if (colon_pos == string::npos)
        {
            cerr << "Listen address format error: " << listen_part << endl;
            continue;
        }

        string listen_ip = listen_part.substr(0, colon_pos);
        int listen_port = stoi(listen_part.substr(colon_pos + 1));

        // Parse target address
        colon_pos = target_part.find_last_of(':');
        if (colon_pos == string::npos)
        {
            cerr << "Target address format error: " << target_part << endl;
            continue;
        }

        string target_ip = target_part.substr(0, colon_pos);
        int target_port = stoi(target_part.substr(colon_pos + 1));

        ForwardRule rule{listen_ip, listen_port, target_ip, target_port};
        rules.push_back(rule);

        cout << "Added forward rule: " << listen_ip << ":" << listen_port << " -> " << target_ip << ":" << target_port
             << endl;
    }

    return rules;
}

int main()
{
    try
    {
        cout << "UDP Data Forwarder (Multi-threaded Version)" << endl;
        cout << "Enter config file path (default: config.txt): ";

        string config_file;
        getline(cin, config_file);
        if (config_file.empty())
        {
            config_file = "config.txt";
        }

        // Read forwarding rules
        vector<ForwardRule> rules = read_config(config_file);

        if (rules.empty())
        {
            cout << "No valid forwarding rules found, exiting" << endl;
            return 1;
        }

        // Get system thread count for creating multiple io_context
        unsigned int thread_count = max(2u, thread::hardware_concurrency());
        cout << "Using " << thread_count << " worker threads" << endl;

        // Create multiple io_context for multi-threading support
        vector<asio::io_context> io_contexts(thread_count);
        vector<unique_ptr<UDPForwarder>> forwarders;
        GlobalStatistics global_stats;

        // Create forwarders for each rule, round-robin assignment to different io_context
        size_t io_index = 0;
        for (const auto &rule : rules)
        {
            try
            {
                forwarders.push_back(make_unique<UDPForwarder>(io_contexts[io_index], rule, global_stats));
                io_index = (io_index + 1) % thread_count;
            }
            catch (const exception &e)
            {
                cerr << "Failed to create forwarder: " << e.what() << endl;
            }
        }

        if (forwarders.empty())
        {
            cout << "No forwarders created successfully, exiting" << endl;
            return 1;
        }

        // Create multiple worker threads
        vector<thread> worker_threads;
        for (auto &io_context : io_contexts)
        {
            worker_threads.emplace_back([&io_context]() { io_context.run(); });
        }

        // Start statistics display thread
        atomic<bool> stop_stats{false};
        thread stats_thread([&global_stats, &stop_stats]() {
            while (!stop_stats)
            {
                this_thread::sleep_for(chrono::seconds(5));
                if (!stop_stats)
                {
                    uint64_t packets_received, packets_forwarded, bytes_received, bytes_forwarded, forward_errors;
                    global_stats.get_aggregated_stats(packets_received, packets_forwarded, bytes_received,
                                                      bytes_forwarded, forward_errors);

                    cout << "\n=== Forwarding Statistics ===" << endl;
                    cout << "Packets received: " << packets_received << endl;
                    cout << "Packets forwarded: " << packets_forwarded << endl;
                    cout << "Bytes received: " << bytes_received << endl;
                    cout << "Bytes forwarded: " << bytes_forwarded << endl;
                    cout << "Forward errors: " << forward_errors << endl;
                    cout << "=============================" << endl;
                }
            }
        });

        cout << "Press Enter to stop forwarder..." << endl;

        // Wait for user input
        cin.get();

        cout << "Stopping forwarder..." << endl;

        // Stop statistics thread
        stop_stats = true;

        // Stop all io_context
        for (auto &io_context : io_contexts)
        {
            io_context.stop();
        }

        // Wait for all worker threads to finish
        for (auto &worker : worker_threads)
        {
            worker.join();
        }

        // Wait for statistics thread to finish
        stats_thread.join();

        // Display final statistics
        uint64_t packets_received, packets_forwarded, bytes_received, bytes_forwarded, forward_errors;
        global_stats.get_aggregated_stats(packets_received, packets_forwarded, bytes_received, bytes_forwarded,
                                          forward_errors);

        cout << "\n=== Final Statistics ===" << endl;
        cout << "Packets received: " << packets_received << endl;
        cout << "Packets forwarded: " << packets_forwarded << endl;
        cout << "Bytes received: " << bytes_received << endl;
        cout << "Bytes forwarded: " << bytes_forwarded << endl;
        cout << "Forward errors: " << forward_errors << endl;
        cout << "Forwarder stopped" << endl;
    }
    catch (const exception &e)
    {
        cerr << "Program exception: " << e.what() << endl;
        return 1;
    }

    return 0;
}