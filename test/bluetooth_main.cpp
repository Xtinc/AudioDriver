#include "bt/bluetooth_agent.h"
#if defined(AUDIO_DRIVER_HAVE_DBUS)

#include <algorithm>
#include <csignal>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <mutex>

// 异步操作任务结构
struct AsyncTask {
    std::function<void()> task;
    std::string description;
};

class BluetoothController
{
  public:
    BluetoothController() : running_(false), task_running_(false)
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
        commands_["quit"] = [this](const std::vector<std::string> &args) { return cmd_quit(args); };
        commands_["exit"] = [this](const std::vector<std::string> &args) { return cmd_quit(args); };
        // 移除 yes/no 命令，因为不再需要手动确认
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

        agent_->start_dbus_loop();
        
        // 启动后台任务处理线程
        task_running_ = true;
        task_thread_ = std::thread(&BluetoothController::task_worker, this);
        
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
        
        // 停止后台线程
        task_running_ = false;
        task_cv_.notify_all();
        
        if (task_thread_.joinable())
        {
            task_thread_.join();
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
            std::cout << "[bluetooth]# ";  // 简化提示符，不再需要配对状态

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
    // 后台任务工作线程
    void task_worker()
    {
        while (task_running_)
        {
            std::unique_lock<std::mutex> lock(task_mutex_);
            
            // 等待任务或退出信号
            task_cv_.wait(lock, [this] { 
                return !task_queue_.empty() || !task_running_; 
            });
            
            if (!task_running_)
                break;
                
            if (!task_queue_.empty())
            {
                AsyncTask task = task_queue_.front();
                task_queue_.pop();
                lock.unlock();
                
                // 执行任务
                try 
                {
                    task.task();
                }
                catch (const std::exception& e)
                {
                    std::cout << "\nTask failed: " << e.what() << std::endl;
                    print_prompt();
                }
            }
        }
    }
    
    // 添加异步任务
    void add_async_task(const AsyncTask& task)
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        task_queue_.push(task);
        task_cv_.notify_one();
    }
    
    // 打印提示符
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
                  << "  quit|exit               Exit program\n" << std::endl;
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
                target_device = args[1]; // 可能是设备路径
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
                std::cout << "    " << uuid << "\n";
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
        std::cout << "Pairing will be automatically confirmed" << std::endl;

        // 创建异步配对任务
        AsyncTask task;
        task.description = "Pairing with " + args[1];
        task.task = [this, device_path, mac_addr = args[1]]() {
            bool success = agent_->pair(device_path);
            
            std::cout << "\n" << (success ? "Pairing successful" : "Pairing failed") 
                      << " with " << mac_addr << std::endl;
            print_prompt();
        };
        
        add_async_task(task);
        return true;
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
    
    // 后台任务相关
    std::atomic<bool> task_running_;
    std::thread task_thread_;
    std::queue<AsyncTask> task_queue_;
    std::mutex task_mutex_;
    std::condition_variable task_cv_;
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
