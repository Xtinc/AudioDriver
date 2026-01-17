#include "audio_rpc.h"
#include <iostream>

int main()
{
    std::cout << "Testing RPC Priority Support\n";
    std::cout << "============================\n\n";
    
    try {
        asio::io_context io_context;
        
        // Create RPC client
        RPCClient client(io_context, "127.0.0.1", 8080);
        
        // Test play with different priorities
        OToken otoken(101);
        
        std::cout << "Testing play with HIGH priority:\n";
        auto ret1 = client.play("/path/to/test_high.wav", 1, otoken, AudioPriority::HIGH);
        std::cout << "  Result: " << (ret1 ? "OK" : ret1.what()) << "\n\n";
        
        std::cout << "Testing play with MEDIUM priority (default):\n";
        auto ret2 = client.play("/path/to/test_medium.wav", 1, otoken);
        std::cout << "  Result: " << (ret2 ? "OK" : ret2.what()) << "\n\n";
        
        std::cout << "Testing play with LOW priority:\n";
        auto ret3 = client.play("/path/to/test_low.wav", 1, otoken, AudioPriority::LOW);
        std::cout << "  Result: " << (ret3 ? "OK" : ret3.what()) << "\n\n";
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "RPC Priority interface test completed.\n";
    return 0;
}