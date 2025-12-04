#if defined(AUDIO_DRIVER_HAVE_DBUS)
#ifndef BLUETOOTH_AGENT_H
#define BLUETOOTH_AGENT_H

#include "audio_message.h"
#include <atomic>
#include <condition_variable>
#include <dbus/dbus.h>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct DBusConnection;

struct BluetoothDevice
{
    std::string path;
    std::string address;
    std::string name;
    std::string alias;
    int16_t rssi;
    bool paired;
    bool connected;
    bool trusted;
    bool blocked;
    std::vector<std::string> uuids;
    static std::string translate_uuid(const std::string &uuid);
};

// 验证请求类型枚举
enum class PairingRequestType
{
    PIN_CODE,        // 请求输入 PIN 码
    PASSKEY,         // 请求输入密钥
    CONFIRMATION,    // 请求确认配对（显示密钥）
    DISPLAY_PINCODE, // 显示 PIN 码
    DISPLAY_PASSKEY, // 显示密钥
    AUTHORIZATION    // 请求授权
};

// 验证请求信息
struct PairingRequest
{
    PairingRequestType type;
    std::string device_path;
    std::string device_name;
    std::string device_address;
    uint32_t passkey; // 用于 CONFIRMATION 和 DISPLAY_PASSKEY

    PairingRequest() : type(PairingRequestType::CONFIRMATION), passkey(0)
    {
    }
};

// 回调函数类型定义
using PairingRequestCallback = std::function<void(const PairingRequest &request)>;

class BluetoothAgent
{
  public:
    BluetoothAgent(const std::string &adapter_path = "/org/bluez/hci0");
    ~BluetoothAgent();

    bool initialize();
    void start_dbus_loop();
    void stop_dbus_loop();
    bool is_running() const;

    bool set_powered(bool powered);
    bool set_pairable(bool pairable);
    bool set_discoverable(bool discoverable);

    bool start_scan();
    bool stop_scan();

    bool pair(const std::string &device_path);
    bool connect(const std::string &device_path);
    bool disconnect(const std::string &device_path);
    bool remove(const std::string &device_path);
    bool set_trusted(const std::string &device_path, bool trusted = true);

    std::vector<BluetoothDevice> list() const;

    void handle_dev_add(DBusMessage *msg);
    void handle_dev_chg(DBusMessage *msg);
    void handle_dev_del(DBusMessage *msg);
    DBusHandlerResult handle_message(DBusMessage *msg);

    // 注册配对请求回调
    void set_pairing_request_callback(PairingRequestCallback callback);

    // 外部程序调用的接口：设置验证码确认结果
    void set_confirmation_result(bool accept);
    void set_passkey_result(bool accept, uint32_t passkey = 0);
    void set_pincode_result(bool accept, const std::string &pincode = "");

  private:
    bool set_adapter_property(const char *property, bool value);
    bool set_device_property(const char *device_path, const char *property, bool value);
    std::string get_device_property(const char *device_path, const char *property);
    DBusMessage *call_method(const char *path, const char *interface, const char *method, int first_arg_type, ...);

    void load_existing_devices();
    void update_dev_from_message(const char *object_path, DBusMessageIter *args);
    void update_device_property(const char *path, DBusMessageIter *iter);
    void update_device_property(BluetoothDevice &device, const char *key, DBusMessageIter *iter);

    bool register_agent();
    DBusHandlerResult handle_request_pincode(DBusMessage *msg);
    DBusHandlerResult handle_display_pincode(DBusMessage *msg);
    DBusHandlerResult handle_request_passkey(DBusMessage *msg);
    DBusHandlerResult handle_display_passkey(DBusMessage *msg);
    DBusHandlerResult handle_request_confirmation(DBusMessage *msg);
    DBusHandlerResult handle_request_authorization(DBusMessage *msg);
    DBusHandlerResult handle_authorize_service(DBusMessage *msg);
    DBusHandlerResult handle_cancel(DBusMessage *msg);

    // 通知外部程序配对请求
    void notify_pairing_request(const PairingRequest &request);

    // 等待外部确认的辅助函数
    bool wait_for_confirmation(int timeout_seconds = 10);
    bool wait_for_passkey(uint32_t &passkey, int timeout_seconds = 10);
    bool wait_for_pincode(std::string &pincode, int timeout_seconds = 10);

  private:
    DBusConnection *connection_;
    DBusConnection *event_connection_;
    std::string adapter_path_;
    mutable std::mutex devices_mutex_;
    std::vector<BluetoothDevice> devices_;
    std::atomic<bool> running_;
    bool scanning_;
    std::thread dbus_thread_;

    // Promise 相关的成员变量
    std::mutex confirmation_mutex_;
    std::condition_variable confirmation_cv_;
    std::unique_ptr<std::promise<bool>> confirmation_promise_;
    std::unique_ptr<std::promise<std::pair<bool, uint32_t>>> passkey_promise_;
    std::unique_ptr<std::promise<std::pair<bool, std::string>>> pincode_promise_;
    bool confirmation_set_;
    bool passkey_set_;
    bool pincode_set_;

    // 回调函数
    PairingRequestCallback pairing_request_callback_;
    std::mutex callback_mutex_;
};

#endif
#endif