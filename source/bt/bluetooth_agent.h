#if defined(AUDIO_DRIVER_HAVE_DBUS)
#ifndef BLUETOOTH_AGENT_H
#define BLUETOOTH_AGENT_H

#include "audio_message.h"
#include <atomic>
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

enum class PairingRequestType
{
    PIN_CODE,
    PASSKEY,
    CONFIRMATION,
    DISPLAY_PINCODE,
    DISPLAY_PASSKEY,
    AUTHORIZATION
};

struct PairingRequest
{
    PairingRequestType type;
    std::string device_path;
    std::string device_name;
    std::string device_address;
    uint32_t passkey;

    PairingRequest() : type(PairingRequestType::CONFIRMATION), passkey(0)
    {
    }
};

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

    void set_pairing_request_callback(PairingRequestCallback callback);
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

    void notify_pairing_request(const PairingRequest &request);

    using PasskeyResult = std::pair<bool, uint32_t>;
    using PincodeResult = std::pair<bool, std::string>;
    using ConfirmationFuture = std::future<bool>;
    using PasskeyFuture = std::future<PasskeyResult>;
    using PincodeFuture = std::future<PincodeResult>;

  private:
    DBusConnection *connection_;
    DBusConnection *event_connection_;
    std::string adapter_path_;
    mutable std::mutex devices_mutex_;
    std::vector<BluetoothDevice> devices_;
    std::atomic<bool> running_;
    bool scanning_;
    std::thread dbus_thread_;
    std::mutex confirmation_mutex_;
    std::unique_ptr<std::promise<bool>> confirmation_promise_;
    std::unique_ptr<std::promise<PasskeyResult>> passkey_promise_;
    std::unique_ptr<std::promise<PincodeResult>> pincode_promise_;

    PairingRequestCallback pairing_request_callback_;
    std::mutex callback_mutex_;
};

#endif
#endif