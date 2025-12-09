#include "audio_interface.h"
#include "audio_remote_interface.h"
#include <chrono>
#include <iostream>
#include <thread>

// Test configuration
constexpr const char *TEST_SERVER_IP = "127.0.0.1";
constexpr unsigned short TEST_SERVER_PORT = 8080;
constexpr unsigned short TEST_AUDIO_PORT = 9000;

// Global AudioCenter pointer for cleanup
static AudioCenter *g_center = nullptr;

// Helper function to print test results
void print_test_result(const char *test_name, bool passed)
{
    std::cout << "[" << (passed ? "PASS" : "FAIL") << "] " << test_name << std::endl;
}

// Initialize AudioCenter and create test streams
bool initialize_audio_center()
{
    try
    {
        std::cout << "Initializing AudioCenter..." << std::endl;

        // Create AudioCenter with network enabled
        g_center = new AudioCenter(true, "0.0.0.0", TEST_AUDIO_PORT);

        // Create input stream (token 1)
        auto ret1 = g_center->create(IToken(1), AudioDeviceName("null", 0), AudioBandWidth::Full,
                                     AudioPeriodSize::INR_20MS, 2, StreamFlags::Network | StreamFlags::CodecOPUS);
        if (!ret1)
        {
            std::cerr << "Failed to create input stream 1: " << ret1.what() << std::endl;
            return false;
        }
        std::cout << "  - Created input stream (token 1)" << std::endl;

        // Create output stream (token 101) - must be > 100
        auto ret2 = g_center->create(OToken(101), AudioDeviceName("null", 0), AudioBandWidth::Full,
                                     AudioPeriodSize::INR_20MS, 2, StreamFlags::Network);
        if (!ret2)
        {
            std::cerr << "Failed to create output stream 101: " << ret2.what() << std::endl;
            return false;
        }
        std::cout << "  - Created output stream (token 101)" << std::endl;

        // Prepare (disable USB detection for test)
        auto ret3 = g_center->prepare(false);
        if (!ret3)
        {
            std::cerr << "Failed to prepare: " << ret3.what() << std::endl;
            return false;
        }
        std::cout << "  - AudioCenter prepared" << std::endl;

        // Start AudioCenter
        auto ret4 = g_center->start();
        if (!ret4)
        {
            std::cerr << "Failed to start: " << ret4.what() << std::endl;
            return false;
        }
        std::cout << "  - AudioCenter started" << std::endl;

        // Enable RPC service
        auto ret5 = g_center->enable_rpc(TEST_SERVER_PORT);
        if (!ret5)
        {
            std::cerr << "Failed to enable RPC: " << ret5.what() << std::endl;
            return false;
        }
        std::cout << "  - RPC service enabled on port " << TEST_SERVER_PORT << std::endl;

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception during initialization: " << e.what() << std::endl;
        return false;
    }
}

// Cleanup AudioCenter
void cleanup_audio_center()
{
    if (g_center)
    {
        std::cout << "\nCleaning up AudioCenter..." << std::endl;

        // Wait briefly for any in-flight operations
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        delete g_center;
        g_center = nullptr;
        std::cout << "  - AudioCenter cleaned up" << std::endl;
    }
}

// Test 1: Basic connection
bool test_basic_connection()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);
        std::cout << "  - Client created successfully" << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 2: Connect stream
bool test_connect_stream()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        IToken itoken(1);
        OToken otoken(101);
        auto ret = client.connect_stream(itoken, otoken, TEST_SERVER_IP, TEST_AUDIO_PORT);

        std::cout << "  - Connect result: " << (ret ? "OK" : ret.what()) << std::endl;

        if (ret)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        return static_cast<bool>(ret);
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 3: Disconnect stream
bool test_disconnect_stream()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        IToken itoken(1);
        OToken otoken(101);
        auto ret = client.disconnect_stream(itoken, otoken, TEST_SERVER_IP, TEST_AUDIO_PORT);

        std::cout << "  - Disconnect result: " << (ret ? "OK" : ret.what()) << std::endl;
        return static_cast<bool>(ret);
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 4: Set volume
bool test_set_volume()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        AudioToken token(1);
        auto ret = client.set_volume(token, 75);

        std::cout << "  - Set volume result: " << (ret ? "OK" : ret.what()) << std::endl;
        return static_cast<bool>(ret);
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 5: Mute single token
bool test_mute_single()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        AudioToken token(1);
        auto ret1 = client.mute(token, true);
        std::cout << "  - Mute result: " << (ret1 ? "OK" : ret1.what()) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto ret2 = client.mute(token, false);
        std::cout << "  - Unmute result: " << (ret2 ? "OK" : ret2.what()) << std::endl;

        return ret1 && ret2;
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 6: Mute connection
bool test_mute_connection()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        IToken itoken(1);
        OToken otoken(101);
        auto ret1 = client.mute(itoken, otoken, true, "192.168.1.100");
        std::cout << "  - Mute connection result: " << (ret1 ? "OK" : ret1.what()) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto ret2 = client.mute(itoken, otoken, false, "192.168.1.100");
        std::cout << "  - Unmute connection result: " << (ret2 ? "OK" : ret2.what()) << std::endl;

        return ret1 && ret2;
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 7: Play audio
bool test_play_audio()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        OToken otoken(101);
        auto ret = client.play("/path/to/test.wav", 1, otoken);

        std::cout << "  - Play result: " << (ret ? "OK" : ret.what()) << std::endl;
        return static_cast<bool>(ret);
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 8: Stop play
bool test_stop_play()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        auto ret = client.stop_play("/path/to/test.wav");

        std::cout << "  - Stop play result: " << (ret ? "OK" : ret.what()) << std::endl;
        return static_cast<bool>(ret);
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 9: Set player volume
bool test_set_player_volume()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        auto ret = client.set_player_volume(80);

        std::cout << "  - Set player volume result: " << (ret ? "OK" : ret.what()) << std::endl;
        return static_cast<bool>(ret);
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 10: Invalid parameters
bool test_invalid_parameters()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        // Test invalid volume (>100)
        AudioToken token(1);
        auto ret1 = client.set_volume(token, 150);
        bool test1 = !ret1 && ret1.err == RetCode::EPARAM;
        std::cout << "  - Invalid volume test: " << (test1 ? "PASS" : "FAIL") << std::endl;

        // Test invalid player volume
        auto ret2 = client.set_player_volume(200);
        bool test2 = !ret2 && ret2.err == RetCode::EPARAM;
        std::cout << "  - Invalid player volume test: " << (test2 ? "PASS" : "FAIL") << std::endl;

        return test1 && test2;
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 11: Multiple operations
bool test_multiple_operations()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        IToken itoken(1);
        OToken otoken(101);
        AudioToken token(1);

        // Connect
        auto ret1 = client.connect_stream(itoken, otoken, "192.168.1.100", TEST_AUDIO_PORT);
        std::cout << "  - Connect: " << (ret1 ? "OK" : ret1.what()) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Set volume
        auto ret2 = client.set_volume(token, 60);
        std::cout << "  - Set volume: " << (ret2 ? "OK" : ret2.what()) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Mute
        auto ret3 = client.mute(token, true);
        std::cout << "  - Mute: " << (ret3 ? "OK" : ret3.what()) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Unmute
        auto ret4 = client.mute(token, false);
        std::cout << "  - Unmute: " << (ret4 ? "OK" : ret4.what()) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Disconnect
        auto ret5 = client.disconnect_stream(itoken, otoken, "192.168.1.100", TEST_AUDIO_PORT);
        std::cout << "  - Disconnect: " << (ret5 ? "OK" : ret5.what()) << std::endl;

        return ret1 && ret2 && ret3 && ret4 && ret5;
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

// Test 12: Stress test - multiple rapid requests
bool test_stress()
{
    try
    {
        AudioRemoteClient client(TEST_SERVER_IP, TEST_SERVER_PORT);

        AudioToken token(1);
        int success_count = 0;
        constexpr int ITERATIONS = 10;

        for (int i = 0; i < ITERATIONS; ++i)
        {
            auto ret = client.set_volume(token, 50 + (i % 50));
            if (ret)
            {
                success_count++;
            }
        }

        std::cout << "  - Completed " << success_count << "/" << ITERATIONS << " requests" << std::endl;
        return success_count == ITERATIONS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "  - Exception: " << e.what() << std::endl;
        return false;
    }
}

int main(int argc, char *argv[])
{
    std::cout << "========================================" << std::endl;
    std::cout << "      RPC Functionality Test Suite      " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // Initialize AudioCenter and create streams
    if (!initialize_audio_center())
    {
        std::cerr << "Failed to initialize AudioCenter. Exiting..." << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "Server configuration:" << std::endl;
    std::cout << "  IP:   " << TEST_SERVER_IP << std::endl;
    std::cout << "  Port: " << TEST_SERVER_PORT << std::endl;
    std::cout << std::endl;

    std::cout << "Press Enter to start tests...";
    std::cin.get();
    std::cout << std::endl;

    int passed = 0;
    int failed = 0;

    // Run all tests
    struct Test
    {
        const char *name;
        bool (*func)();
    };

    Test tests[] = {
        {"Basic Connection", test_basic_connection},
        {"Connect Stream", test_connect_stream},
        {"Disconnect Stream", test_disconnect_stream},
        {"Set Volume", test_set_volume},
        {"Mute Single Token", test_mute_single},
        {"Mute Connection", test_mute_connection},
        {"Play Audio", test_play_audio},
        {"Stop Play", test_stop_play},
        {"Set Player Volume", test_set_player_volume},
        {"Invalid Parameters", test_invalid_parameters},
        {"Multiple Operations", test_multiple_operations},
        {"Stress Test", test_stress},
    };

    for (const auto &test : tests)
    {
        std::cout << "Running: " << test.name << std::endl;
        bool result = test.func();
        print_test_result(test.name, result);

        if (result)
        {
            passed++;
        }
        else
        {
            failed++;
        }

        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "========================================" << std::endl;
    std::cout << "Test Summary:" << std::endl;
    std::cout << "  Total:  " << (passed + failed) << std::endl;
    std::cout << "  Passed: " << passed << std::endl;
    std::cout << "  Failed: " << failed << std::endl;
    std::cout << "========================================" << std::endl;

    // Cleanup
    cleanup_audio_center();
    return failed == 0 ? 0 : 1;
}
