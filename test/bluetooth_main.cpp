#include "bt/bluetooth_agent.h"
#if defined(AUDIO_DRIVER_HAVE_DBUS)

#include <algorithm>
#include <atomic>
#include <csignal>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

class BluetoothController
{
  public:
    BluetoothController() : running_(false), has_pending_request_(false)
    {
        commands_["help"] = [this](const std::vector<std::string> &args) { return cmd_help(args); };
        commands_["list"] = [this](const std::vector<std::string> &args) { return cmd_list(args); };
        commands_["show"] = [this](const std::vector<std::string> &args) { return cmd_show(args); };
        commands_["scan"] = [this](const std::vector<std::string> &args) { return cmd_scan(args); };
        commands_["pair"] = [this](const std::vector<std::string> &args) { return cmd_pair(args); };
        commands_["connect"] = [this](const std::vector<std::string> &args) { return cmd_connect(args); };
        commands_["disconnect"] = [this](const std::vector<std::string> &args) { return cmd_disconnect(args); };
        commands_["remove"] = [this](const std::vector<std::string> &args) { return cmd_remove(args); };
        commands_["power"] = [this](const std::vector<std::string> &args) { return cmd_power(args); };
        commands_["pairable"] = [this](const std::vector<std::string> &args) { return cmd_pairable(args); };
        commands_["discoverable"] = [this](const std::vector<std::string> &args) { return cmd_discoverable(args); };
        commands_["accept"] = [this](const std::vector<std::string> &args) { return cmd_accept(args); };
        commands_["reject"] = [this](const std::vector<std::string> &args) { return cmd_reject(args); };
        commands_["quit"] = [this](const std::vector<std::string> &args) { return cmd_quit(args); };
        commands_["exit"] = [this](const std::vector<std::string> &args) { return cmd_quit(args); };
    }

    ~BluetoothController()
    {
        shutdown();
    }

    bool initialize()
    {
        agent_ = std::make_unique<BluetoothAgent>();

        if (!agent_->initialize())
        {
            std::cerr << "Failed to initialize Bluetooth agent" << std::endl;
            return false;
        }

        // 注册配对请求回调
        agent_->set_pairing_request_callback(
            [this](const PairingRequest &request) { handle_pairing_request(request); });

        agent_->start_dbus_loop();
        running_ = true;
        std::cout << "Bluetooth Controller initialized successfully" << std::endl;
        return true;
    }

    void shutdown()
    {
        if (agent_)
        {
            agent_->stop_dbus_loop();
            agent_.reset();
        }
        running_ = false;
    }

    void run()
    {
        std::cout << "Bluetooth Controller v1.0" << std::endl;
        std::cout << "Type 'help' for available commands" << std::endl;

        std::string input;
        while (running_)
        {
            std::cout << "[bluetooth]# ";

            if (!std::getline(std::cin, input))
            {
                break;
            }

            if (input.empty())
            {
                continue;
            }

            auto args = parse_command(input);
            if (!args.empty())
            {
                execute_command(args);
            }
        }
    }

  private:
    void print_prompt()
    {
        std::cout << "[bluetooth]# " << std::flush;
    }

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

    bool execute_command(const std::vector<std::string> &args)
    {
        if (args.empty())
        {
            return false;
        }

        const std::string &command = args[0];
        auto it = commands_.find(command);

        if (it != commands_.end())
        {
            return it->second(args);
        }
        else
        {
            std::cout << "Unknown command: " << command << std::endl;
            std::cout << "Type 'help' for available commands" << std::endl;
            return false;
        }
    }

    std::string find_device_by_address(const std::string &address)
    {
        auto devices = agent_->list();
        auto it = std::find_if(devices.begin(), devices.end(),
                               [&address](const BluetoothDevice &dev) { return dev.address == address; });

        return (it != devices.end()) ? it->path : "";
    }

    void handle_pairing_request(const PairingRequest &request)
    {
        std::lock_guard<std::mutex> lock(request_mutex_);

        std::cout << "\n\n========================================" << std::endl;
        std::cout << "PAIRING REQUEST" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Device: " << request.device_name << " (" << request.device_address << ")" << std::endl;

        switch (request.type)
        {
        case PairingRequestType::PIN_CODE:
            std::cout << "Type: Request PIN Code" << std::endl;
            std::cout << "\nPlease enter PIN code, then type:" << std::endl;
            std::cout << "  accept <pincode>  - Accept with PIN code" << std::endl;
            std::cout << "  reject            - Reject pairing" << std::endl;
            pending_request_ = request;
            has_pending_request_ = true;
            break;

        case PairingRequestType::PASSKEY:
            std::cout << "Type: Request Passkey" << std::endl;
            std::cout << "\nPlease enter 6-digit passkey (000000-999999), then type:" << std::endl;
            std::cout << "  accept <passkey>  - Accept with passkey" << std::endl;
            std::cout << "  reject            - Reject pairing" << std::endl;
            pending_request_ = request;
            has_pending_request_ = true;
            break;

        case PairingRequestType::CONFIRMATION:
            std::cout << "Type: Confirm Pairing" << std::endl;
            std::cout << "Passkey: " << std::setfill('0') << std::setw(6) << request.passkey << std::endl;
            std::cout << "\nPlease verify the passkey matches on both devices, then type:" << std::endl;
            std::cout << "  accept  - Confirm pairing" << std::endl;
            std::cout << "  reject  - Reject pairing" << std::endl;
            pending_request_ = request;
            has_pending_request_ = true;
            break;

        case PairingRequestType::DISPLAY_PASSKEY:
            std::cout << "Type: Display Passkey" << std::endl;
            std::cout << "Passkey: " << std::setfill('0') << std::setw(6) << request.passkey << std::endl;
            std::cout << "\nPlease enter this passkey on the other device." << std::endl;
            break;

        case PairingRequestType::DISPLAY_PINCODE:
            std::cout << "Type: Display PIN Code" << std::endl;
            std::cout << "\nPairing in progress..." << std::endl;
            break;

        case PairingRequestType::AUTHORIZATION:
            std::cout << "Type: Authorization Request" << std::endl;
            std::cout << "\nDevice is requesting authorization." << std::endl;
            break;
        }

        std::cout << "========================================\n" << std::endl;
        print_prompt();
    }

    bool cmd_accept(const std::vector<std::string> &args)
    {
        std::lock_guard<std::mutex> lock(request_mutex_);

        if (!has_pending_request_)
        {
            std::cout << "No pending pairing request" << std::endl;
            return false;
        }

        bool success = false;

        switch (pending_request_.type)
        {
        case PairingRequestType::PIN_CODE:
            if (args.size() < 2)
            {
                std::cout << "Usage: accept <pincode>" << std::endl;
                return false;
            }
            agent_->set_pincode_result(true, args[1]);
            std::cout << "PIN code sent: " << args[1] << std::endl;
            success = true;
            break;

        case PairingRequestType::PASSKEY:
            if (args.size() < 2)
            {
                std::cout << "Usage: accept <passkey>" << std::endl;
                return false;
            }
            try
            {
                uint32_t passkey = std::stoul(args[1]);
                if (passkey > 999999)
                {
                    std::cout << "Passkey must be between 000000 and 999999" << std::endl;
                    return false;
                }
                agent_->set_passkey_result(true, passkey);
                std::cout << "Passkey sent: " << std::setfill('0') << std::setw(6) << passkey << std::endl;
                success = true;
            }
            catch (...)
            {
                std::cout << "Invalid passkey format" << std::endl;
                return false;
            }
            break;

        case PairingRequestType::CONFIRMATION:
            agent_->set_confirmation_result(true);
            std::cout << "Pairing confirmed" << std::endl;
            success = true;
            break;

        default:
            std::cout << "This request type does not require acceptance" << std::endl;
            return false;
        }

        if (success)
        {
            has_pending_request_ = false;
        }

        return success;
    }

    bool cmd_reject(const std::vector<std::string> &args)
    {
        std::lock_guard<std::mutex> lock(request_mutex_);

        if (!has_pending_request_)
        {
            std::cout << "No pending pairing request" << std::endl;
            return false;
        }

        switch (pending_request_.type)
        {
        case PairingRequestType::PIN_CODE:
            agent_->set_pincode_result(false, "");
            std::cout << "PIN code request rejected" << std::endl;
            break;

        case PairingRequestType::PASSKEY:
            agent_->set_passkey_result(false, 0);
            std::cout << "Passkey request rejected" << std::endl;
            break;

        case PairingRequestType::CONFIRMATION:
            agent_->set_confirmation_result(false);
            std::cout << "Pairing confirmation rejected" << std::endl;
            break;

        default:
            std::cout << "This request type cannot be rejected" << std::endl;
            return false;
        }

        has_pending_request_ = false;
        return true;
    }

    // Command handlers
    bool cmd_help(const std::vector<std::string> &args)
    {
        std::cout << "\nAvailable commands:\n"
                  << "  help                    Show this help message\n"
                  << "  list                    List available devices\n"
                  << "  show [MAC]              Show device information\n"
                  << "  scan on|off             Start/stop device discovery\n"
                  << "  pair MAC                Pair with device\n"
                  << "  connect MAC             Connect to device\n"
                  << "  disconnect MAC          Disconnect from device\n"
                  << "  remove MAC              Remove device\n"
                  << "  power on|off            Set adapter power state\n"
                  << "  pairable on|off         Set adapter pairable state\n"
                  << "  discoverable on|off     Set adapter discoverable state\n"
                  << "  accept [code]           Accept pairing request (with PIN/passkey if needed)\n"
                  << "  reject                  Reject pairing request\n"
                  << "  quit|exit               Exit program\n"
                  << std::endl;
        return true;
    }

    bool cmd_list(const std::vector<std::string> &args)
    {
        auto devices = agent_->list();

        if (devices.empty())
        {
            std::cout << "No devices found" << std::endl;
            return true;
        }

        std::cout << "\nDevices:\n";
        std::cout << std::setw(18) << "Address" << " " << std::setw(20) << "Name" << " " << std::setw(6) << "RSSI"
                  << " " << std::setw(8) << "Paired" << " " << std::setw(10) << "Connected" << std::endl;
        std::cout << std::string(70, '-') << std::endl;

        for (const auto &device : devices)
        {
            std::cout << std::setw(18) << device.address << " " << std::setw(20)
                      << (device.name.empty() ? device.alias : device.name) << " " << std::setw(6) << device.rssi << " "
                      << std::setw(8) << (device.paired ? "Yes" : "No") << " " << std::setw(10)
                      << (device.connected ? "Yes" : "No") << std::endl;
        }
        std::cout << std::endl;
        return true;
    }

    bool cmd_show(const std::vector<std::string> &args)
    {
        std::string target_device;

        if (args.size() < 2)
        {
            auto devices = agent_->list();
            if (!devices.empty())
            {
                target_device = devices[0].path;
            }
            else
            {
                std::cout << "No devices available" << std::endl;
                return true;
            }
        }
        else
        {
            target_device = find_device_by_address(args[1]);
            if (target_device.empty())
            {
                target_device = args[1];
            }
        }

        auto devices = agent_->list();
        auto it = std::find_if(devices.begin(), devices.end(), [&target_device](const BluetoothDevice &dev) {
            return dev.path == target_device || dev.address == target_device;
        });

        if (it == devices.end())
        {
            std::cout << "Device not found: " << target_device << std::endl;
            return false;
        }

        const auto &dev = *it;
        std::cout << "\nDevice Information:\n"
                  << "  Path:         " << dev.path << "\n"
                  << "  Address:      " << dev.address << "\n"
                  << "  Name:         " << dev.name << "\n"
                  << "  Alias:        " << dev.alias << "\n"
                  << "  RSSI:         " << dev.rssi << "\n"
                  << "  Paired:       " << (dev.paired ? "Yes" : "No") << "\n"
                  << "  Connected:    " << (dev.connected ? "Yes" : "No") << "\n"
                  << "  Trusted:      " << (dev.trusted ? "Yes" : "No") << "\n"
                  << "  Blocked:      " << (dev.blocked ? "Yes" : "No") << "\n";

        if (!dev.uuids.empty())
        {
            std::cout << "  UUIDs:\n";
            for (const auto &uuid : dev.uuids)
            {
                std::cout << "    " << BluetoothDevice::translate_uuid(uuid) << "\n";
            }
        }
        std::cout << std::endl;
        return true;
    }

    bool cmd_scan(const std::vector<std::string> &args)
    {
        if (args.size() < 2)
        {
            std::cout << "Usage: scan on|off" << std::endl;
            return false;
        }

        if (args[1] == "on")
        {
            if (agent_->start_scan())
            {
                std::cout << "Discovery started" << std::endl;
            }
            else
            {
                std::cout << "Failed to start discovery" << std::endl;
            }
        }
        else if (args[1] == "off")
        {
            if (agent_->stop_scan())
            {
                std::cout << "Discovery stopped" << std::endl;
            }
            else
            {
                std::cout << "Failed to stop discovery" << std::endl;
            }
        }
        else
        {
            std::cout << "Invalid argument. Use 'on' or 'off'" << std::endl;
            return false;
        }

        return true;
    }

    bool cmd_pair(const std::vector<std::string> &args)
    {
        if (args.size() < 2)
        {
            std::cout << "Usage: pair MAC_ADDRESS" << std::endl;
            return false;
        }

        std::string device_path = find_device_by_address(args[1]);
        if (device_path.empty())
        {
            std::cout << "Device not found: " << args[1] << std::endl;
            std::cout << "Please run 'scan on' first to discover devices" << std::endl;
            return false;
        }

        std::cout << "Attempting to pair with " << args[1] << "..." << std::endl;
        std::cout << "Please respond to any pairing requests that appear." << std::endl;

        return agent_->pair(device_path);
    }

    bool cmd_connect(const std::vector<std::string> &args)
    {
        if (args.size() < 2)
        {
            std::cout << "Usage: connect MAC_ADDRESS" << std::endl;
            return false;
        }

        std::string device_path = find_device_by_address(args[1]);
        if (device_path.empty())
        {
            std::cout << "Device not found: " << args[1] << std::endl;
            return false;
        }

        std::cout << "Attempting to connect to " << args[1] << "..." << std::endl;
        if (agent_->connect(device_path))
        {
            std::cout << "Connection successful" << std::endl;
        }
        else
        {
            std::cout << "Connection failed" << std::endl;
        }

        return true;
    }

    bool cmd_disconnect(const std::vector<std::string> &args)
    {
        if (args.size() < 2)
        {
            std::cout << "Usage: disconnect MAC_ADDRESS" << std::endl;
            return false;
        }

        std::string device_path = find_device_by_address(args[1]);
        if (device_path.empty())
        {
            std::cout << "Device not found: " << args[1] << std::endl;
            return false;
        }

        std::cout << "Disconnecting from " << args[1] << "..." << std::endl;
        if (agent_->disconnect(device_path))
        {
            std::cout << "Disconnection successful" << std::endl;
        }
        else
        {
            std::cout << "Disconnection failed" << std::endl;
        }

        return true;
    }

    bool cmd_remove(const std::vector<std::string> &args)
    {
        if (args.size() < 2)
        {
            std::cout << "Usage: remove MAC_ADDRESS" << std::endl;
            return false;
        }

        std::string device_path = find_device_by_address(args[1]);
        if (device_path.empty())
        {
            std::cout << "Device not found: " << args[1] << std::endl;
            return false;
        }

        std::cout << "Removing device " << args[1] << "..." << std::endl;
        if (agent_->remove(device_path))
        {
            std::cout << "Device removed successfully" << std::endl;
        }
        else
        {
            std::cout << "Failed to remove device" << std::endl;
        }

        return true;
    }

    bool cmd_power(const std::vector<std::string> &args)
    {
        if (args.size() < 2)
        {
            std::cout << "Usage: power on|off" << std::endl;
            return false;
        }

        bool power_on = (args[1] == "on");
        if (args[1] != "on" && args[1] != "off")
        {
            std::cout << "Invalid argument. Use 'on' or 'off'" << std::endl;
            return false;
        }

        if (agent_->set_powered(power_on))
        {
            std::cout << "Adapter power set to " << (power_on ? "on" : "off") << std::endl;
        }
        else
        {
            std::cout << "Failed to set adapter power" << std::endl;
        }

        return true;
    }

    bool cmd_pairable(const std::vector<std::string> &args)
    {
        if (args.size() < 2)
        {
            std::cout << "Usage: pairable on|off" << std::endl;
            return false;
        }

        bool pairable = (args[1] == "on");
        if (args[1] != "on" && args[1] != "off")
        {
            std::cout << "Invalid argument. Use 'on' or 'off'" << std::endl;
            return false;
        }

        if (agent_->set_pairable(pairable))
        {
            std::cout << "Adapter pairable set to " << (pairable ? "on" : "off") << std::endl;
        }
        else
        {
            std::cout << "Failed to set adapter pairable state" << std::endl;
        }

        return true;
    }

    bool cmd_discoverable(const std::vector<std::string> &args)
    {
        if (args.size() < 2)
        {
            std::cout << "Usage: discoverable on|off" << std::endl;
            return false;
        }

        bool discoverable = (args[1] == "on");
        if (args[1] != "on" && args[1] != "off")
        {
            std::cout << "Invalid argument. Use 'on' or 'off'" << std::endl;
            return false;
        }

        if (agent_->set_discoverable(discoverable))
        {
            std::cout << "Adapter discoverable set to " << (discoverable ? "on" : "off") << std::endl;
        }
        else
        {
            std::cout << "Failed to set adapter discoverable state" << std::endl;
        }

        return true;
    }

    bool cmd_quit(const std::vector<std::string> &args)
    {
        std::cout << "Goodbye!" << std::endl;
        running_ = false;
        return true;
    }

  private:
    std::unique_ptr<BluetoothAgent> agent_;
    bool running_;
    std::map<std::string, std::function<bool(const std::vector<std::string> &)>> commands_;

    // 配对请求相关
    std::mutex request_mutex_;
    PairingRequest pending_request_;
    bool has_pending_request_;
};

std::unique_ptr<BluetoothController> g_controller;

void signal_handler(int signal)
{
    if (g_controller)
    {
        std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
        g_controller->shutdown();
        g_controller.reset();
    }
    exit(signal);
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try
    {
        g_controller = std::make_unique<BluetoothController>();

        if (!g_controller->initialize())
        {
            std::cerr << "Failed to initialize Bluetooth controller" << std::endl;
            return 1;
        }

        g_controller->run();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

#else

#include <iostream>

int main()
{
    std::cout << "DBus support not available. Please compile with DBUS support." << std::endl;
    return 1;
}

#endif
