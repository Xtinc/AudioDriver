#if defined(AUDIO_DRIVER_HAVE_DBUS)
#ifndef BLUETOOTH_AGENT_H
#define BLUETOOTH_AGENT_H

/**
 * @file bluetooth_agent.h
 * @brief Bluetooth device management interface using BlueZ D-Bus API
 *
 * This header defines the BluetoothAgent class and related types for managing
 * Bluetooth devices, including discovery, pairing, connection, and device property management.
 */

#include "audio_message.h"
#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct DBusConnection;
struct DBusMessage;
struct DBusMessageIter;

/**
 * @struct BluetoothDevice
 * @brief Represents a Bluetooth device with its properties
 *
 * This structure contains all relevant information about a discovered or paired
 * Bluetooth device, including address, name, connection state, and supported services.
 */
struct BluetoothDevice
{
    std::string path;               /**< D-Bus object path of the device */
    std::string address;            /**< Bluetooth MAC address (e.g., "00:11:22:33:44:55") */
    std::string name;               /**< Device name advertised by the device */
    std::string alias;              /**< User-friendly alias for the device */
    int16_t rssi;                   /**< Received Signal Strength Indicator (dBm) */
    bool paired;                    /**< True if device is paired */
    bool connected;                 /**< True if device is currently connected */
    bool trusted;                   /**< True if device is trusted (auto-connection allowed) */
    bool blocked;                   /**< True if device is blocked */
    std::vector<std::string> uuids; /**< List of supported service UUIDs */

    /**
     * @brief Translates a Bluetooth service UUID to human-readable name
     * @param uuid Service UUID string (e.g., "0000110a-0000-1000-8000-00805f9b34fb")
     * @return Human-readable service name with UUID, or just UUID if unknown
     */
    static std::string translate_uuid(const std::string &uuid);
};

/**
 * @enum PairingRequestType
 * @brief Types of pairing requests that may be received during Bluetooth pairing
 */
enum class PairingRequestType
{
    PIN_CODE,        /**< Request to enter a PIN code (legacy pairing) */
    PASSKEY,         /**< Request to enter a 6-digit passkey */
    CONFIRMATION,    /**< Request to confirm a displayed passkey matches on both devices */
    DISPLAY_PINCODE, /**< Display a PIN code for the user to enter on remote device */
    DISPLAY_PASSKEY, /**< Display a 6-digit passkey for the user to enter on remote device */
    AUTHORIZATION    /**< Request authorization for connection/service access */
};

/**
 * @struct PairingRequest
 * @brief Contains information about a pairing request from a remote device
 *
 * This structure is passed to the pairing request callback to notify the application
 * of user interaction requirements during the pairing process.
 */
struct PairingRequest
{
    PairingRequestType type;    /**< Type of pairing request */
    std::string device_path;    /**< D-Bus object path of the requesting device */
    std::string device_name;    /**< Name or alias of the requesting device */
    std::string device_address; /**< Bluetooth address of the requesting device */
    uint32_t passkey;           /**< Passkey to display (only valid for DISPLAY_PASSKEY and CONFIRMATION types) */

    PairingRequest() : type(PairingRequestType::CONFIRMATION), passkey(0)
    {
    }
};

/**
 * @typedef PairingRequestCallback
 * @brief Callback function type for pairing request notifications
 *
 * Applications should register this callback to receive notifications when user
 * interaction is required during the pairing process. The callback should handle
 * the request according to its type and call the appropriate response method
 * (set_confirmation_result, set_passkey_result, or set_pincode_result).
 *
 * @param request The pairing request information
 */
using PairingRequestCallback = std::function<void(const PairingRequest &request)>;

/**
 * @class BluetoothAgent
 * @brief Main class for managing Bluetooth operations via BlueZ D-Bus interface
 *
 * This class provides a high-level interface for Bluetooth operations including:
 * - Device discovery (scanning)
 * - Device pairing and authentication
 * - Connection management
 * - Device property queries and updates
 * - Automatic device list synchronization
 *
 * The agent handles all D-Bus communication with BlueZ and provides asynchronous
 * notifications for device changes and pairing requests.
 */
class BluetoothAgent
{
  public:
    /**
     * @brief Constructs a BluetoothAgent for the specified adapter
     * @param adapter_path D-Bus object path of the Bluetooth adapter (default: "/org/bluez/hci0")
     */
    BluetoothAgent(const std::string &adapter_path = "/org/bluez/hci0");

    /**
     * @brief Destructor - cleans up D-Bus connections and stops event loop
     */
    ~BluetoothAgent();

    /**
     * @brief Initializes the Bluetooth agent and registers with BlueZ
     *
     * This method must be called before any other operations. It:
     * - Establishes D-Bus connections
     * - Registers the agent with BlueZ
     * - Sets up signal filters for device events
     * - Loads existing paired devices
     * - Configures adapter properties (powered, pairable, discoverable)
     *
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize();

    /**
     * @brief Starts the D-Bus event loop in a separate thread
     *
     * This must be called after initialize() to begin receiving device events
     * and signals from BlueZ. The event loop runs until stop_dbus_loop() is called.
     */
    void start_dbus_loop();

    /**
     * @brief Stops the D-Bus event loop and waits for the thread to complete
     */
    void stop_dbus_loop();

    /**
     * @brief Checks if the D-Bus event loop is running
     * @return true if the event loop is active, false otherwise
     */
    bool is_running() const;

    /**
     * @brief Sets the powered state of the Bluetooth adapter
     * @param powered true to power on the adapter, false to power off
     * @return true if the operation succeeded, false otherwise
     */
    bool set_powered(bool powered);

    /**
     * @brief Sets whether the adapter is pairable
     * @param pairable true to allow pairing, false to disallow
     * @return true if the operation succeeded, false otherwise
     */
    bool set_pairable(bool pairable);

    /**
     * @brief Sets whether the adapter is discoverable by other devices
     * @param discoverable true to make discoverable, false to hide
     * @return true if the operation succeeded, false otherwise
     */
    bool set_discoverable(bool discoverable);

    /**
     * @brief Starts scanning for nearby Bluetooth devices
     *
     * Discovered devices will be added to the internal device list and can be
     * retrieved using list(). Device events are delivered via the D-Bus event loop.
     *
     * @return true if scanning started successfully, false otherwise
     */
    bool start_scan();

    /**
     * @brief Stops scanning for Bluetooth devices
     * @return true if scanning stopped successfully, false otherwise
     */
    bool stop_scan();

    /**
     * @brief Initiates pairing with a device
     *
     * This is an asynchronous operation that may trigger pairing request callbacks
     * if user interaction is required. The device will be automatically set as
     * trusted after successful pairing.
     *
     * @param device_path D-Bus object path of the device to pair with
     * @return true if pairing succeeded, false otherwise
     * @note This method blocks until pairing completes or times out (15 seconds)
     */
    bool pair(const std::string &device_path);

    /**
     * @brief Connects to a paired device
     *
     * The device must be paired before calling this method. The device will be
     * automatically set as trusted before connection.
     *
     * @param device_path D-Bus object path of the device to connect to
     * @return true if connection succeeded, false otherwise
     */
    bool connect(const std::string &device_path);

    /**
     * @brief Disconnects from a connected device
     *
     * The device remains paired and trusted after disconnection.
     *
     * @param device_path D-Bus object path of the device to disconnect from
     * @return true if disconnection succeeded, false otherwise
     */
    bool disconnect(const std::string &device_path);

    /**
     * @brief Removes (unpairs) a device and disconnects if necessary
     *
     * This removes all pairing information and removes the device from the adapter.
     * The device is automatically disconnected and untrusted before removal.
     *
     * @param device_path D-Bus object path of the device to remove
     * @return true if removal succeeded, false otherwise
     */
    bool remove(const std::string &device_path);

    /**
     * @brief Sets the trusted state of a device
     *
     * Trusted devices can automatically reconnect without user confirmation.
     *
     * @param device_path D-Bus object path of the device
     * @param trusted true to trust the device, false to untrust
     * @return true if the operation succeeded, false otherwise
     */
    bool set_trusted(const std::string &device_path, bool trusted = true);

    /**
     * @brief Gets the current list of known Bluetooth devices
     *
     * This includes all discovered, paired, and connected devices known to the adapter.
     *
     * @return Vector of BluetoothDevice structures containing device information
     * @note The returned list is a snapshot; use the pairing callback for real-time updates
     */
    std::vector<BluetoothDevice> list() const;

    void handle_dev_add(DBusMessage *msg);
    void handle_dev_chg(DBusMessage *msg);
    void handle_dev_del(DBusMessage *msg);

    int handle_message(DBusMessage *msg);

    /**
     * @brief Registers a callback for pairing request notifications
     *
     * The callback will be invoked when user interaction is required during pairing,
     * such as entering a PIN code, confirming a passkey, or authorizing a connection.
     * The application should respond by calling the appropriate set_*_result method.
     *
     * @param callback Function to call when a pairing request is received
     */
    void set_pairing_request_callback(PairingRequestCallback callback);

    /**
     * @brief Responds to a confirmation pairing request
     *
     * Call this method in response to a PairingRequestType::CONFIRMATION request
     * received via the pairing request callback.
     *
     * @param accept true to accept the pairing, false to reject
     */
    void set_confirmation_result(bool accept);

    /**
     * @brief Responds to a passkey pairing request
     *
     * Call this method in response to a PairingRequestType::PASSKEY request
     * received via the pairing request callback.
     *
     * @param accept true to accept and provide passkey, false to reject
     * @param passkey 6-digit passkey to use for pairing (0-999999)
     */
    void set_passkey_result(bool accept, uint32_t passkey = 0);

    /**
     * @brief Responds to a PIN code pairing request
     *
     * Call this method in response to a PairingRequestType::PIN_CODE request
     * received via the pairing request callback.
     *
     * @param accept true to accept and provide PIN, false to reject
     * @param pincode PIN code string to use for pairing
     */
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
    int handle_request_pincode(DBusMessage *msg);
    int handle_display_pincode(DBusMessage *msg);
    int handle_request_passkey(DBusMessage *msg);
    int handle_display_passkey(DBusMessage *msg);
    int handle_request_confirmation(DBusMessage *msg);
    int handle_request_authorization(DBusMessage *msg);
    int handle_authorize_service(DBusMessage *msg);
    int handle_cancel(DBusMessage *msg);

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