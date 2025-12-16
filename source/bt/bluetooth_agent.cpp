#include "bluetooth_agent.h"
#if defined(AUDIO_DRIVER_HAVE_DBUS)

#include <dbus/dbus.h>

#define BLUEZ_SERVICE "org.bluez"
#define AGENT_INTERFACE "org.bluez.Agent1"
#define AGENT_MANAGER_INTERFACE "org.bluez.AgentManager1"
#define ADAPTER_INTERFACE "org.bluez.Adapter1"
#define DEVICE_INTERFACE "org.bluez.Device1"
#define PROPERTIES_INTERFACE "org.freedesktop.DBus.Properties"
#define OBJECT_MANAGER_INTERFACE "org.freedesktop.DBus.ObjectManager"
#define AGENT_PATH "/org/bluez/agent"
#define AGENT_SERVICE "org.bluez.agent"
#define DBUS_TIMEOUT_SHORT 5000
#define DBUS_TIMEOUT_LONG 15000
#define DBUS_POLL_INTERVAL 100
#define PAIRING_TIMEOUT_SECONDS 15

#include <algorithm>
#include <cstring>
#include <map>
#include <sstream>

static DBusHandlerResult signal_filter_callback(DBusConnection *, DBusMessage *msg, void *user_data)
{
    BluetoothAgent *agent = static_cast<BluetoothAgent *>(user_data);
    if (dbus_message_is_signal(msg, "org.freedesktop.DBus.ObjectManager", "InterfacesAdded"))
    {
        agent->handle_dev_add(msg);
    }
    else if (dbus_message_is_signal(msg, "org.freedesktop.DBus.ObjectManager", "InterfacesRemoved"))
    {
        agent->handle_dev_del(msg);
    }
    else if (dbus_message_is_signal(msg, "org.freedesktop.DBus.Properties", "PropertiesChanged"))
    {
        agent->handle_dev_chg(msg);
    }
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

static DBusHandlerResult message_handler_callback(DBusConnection *, DBusMessage *msg, void *user_data)
{
    BluetoothAgent *agent = static_cast<BluetoothAgent *>(user_data);
    return static_cast<DBusHandlerResult>(agent->handle_message(msg));
}

static void send_simple_reply(DBusConnection *conn, DBusMessage *msg)
{
    DBusMessage *reply = dbus_message_new_method_return(msg);
    if (reply)
    {
        dbus_connection_send(conn, reply, nullptr);
        dbus_message_unref(reply);
    }
}

BluetoothAgent::BluetoothAgent(const std::string &adapter_path)
    : connection_(nullptr), event_connection_(nullptr), adapter_path_(adapter_path), running_(false), scanning_(false)
{
}

BluetoothAgent::~BluetoothAgent()
{
    stop_dbus_loop();
    if (event_connection_)
    {
        dbus_connection_unref(event_connection_);
    }
    if (connection_)
    {
        dbus_connection_unref(connection_);
    }
}

bool BluetoothAgent::initialize()
{
    DBusError err;
    dbus_error_init(&err);

    connection_ = dbus_bus_get(DBUS_BUS_SYSTEM, &err);
    if (dbus_error_is_set(&err))
    {
        AUDIO_ERROR_PRINT("Failed to get system bus for main connection: %s - %s", err.name, err.message);
        dbus_error_free(&err);
        return false;
    }

    if (!connection_)
    {
        AUDIO_ERROR_PRINT("Failed to get system bus: connection is null");
        return false;
    }

    event_connection_ = dbus_bus_get(DBUS_BUS_SYSTEM, &err);
    if (dbus_error_is_set(&err))
    {
        AUDIO_ERROR_PRINT("Failed to get system bus for event connection: %s - %s", err.name, err.message);
        dbus_error_free(&err);
        return false;
    }

    if (!event_connection_)
    {
        AUDIO_ERROR_PRINT("Failed to get system bus: event connection is null");
        return false;
    }

    dbus_bus_add_match(event_connection_, "type='signal',interface='org.freedesktop.DBus.ObjectManager'", &err);
    if (dbus_error_is_set(&err))
    {
        AUDIO_ERROR_PRINT("Failed to add ObjectManager signal match: %s - %s", err.name, err.message);
        dbus_error_free(&err);
        return false;
    }

    dbus_bus_add_match(event_connection_, "type='signal',interface='org.freedesktop.DBus.Properties'", &err);
    if (dbus_error_is_set(&err))
    {
        AUDIO_ERROR_PRINT("Failed to add Properties signal match: %s - %s", err.name, err.message);
        dbus_error_free(&err);
        return false;
    }

    dbus_connection_add_filter(event_connection_, signal_filter_callback, this, nullptr);

    DBusObjectPathVTable vtable;
    memset(&vtable, 0, sizeof(vtable));
    vtable.message_function = message_handler_callback;

    if (!dbus_connection_register_object_path(connection_, AGENT_PATH, &vtable, this))
    {
        AUDIO_ERROR_PRINT("Failed to register object path for agent");
        return false;
    }

    AUDIO_INFO_PRINT("Object path registered for Bluetooth agent");

    if (!register_agent())
    {
        AUDIO_ERROR_PRINT("Failed to register Bluetooth agent");
        return false;
    }

    AUDIO_INFO_PRINT("Bluetooth agent registered successfully");

    set_powered(true);
    set_pairable(true);
    set_discoverable(true);

    load_existing_devices();

    return true;
}

void BluetoothAgent::start_dbus_loop()
{
    running_ = true;
    dbus_thread_ = std::thread([this]() {
        while (running_)
        {
            dbus_connection_read_write_dispatch(event_connection_, DBUS_POLL_INTERVAL);
        }
    });
}

void BluetoothAgent::stop_dbus_loop()
{
    running_ = false;
    if (dbus_thread_.joinable())
    {
        dbus_thread_.join();
    }
}

bool BluetoothAgent::is_running() const
{
    return running_;
}

bool BluetoothAgent::set_powered(bool powered)
{
    if (set_adapter_property("Powered", powered))
    {
        AUDIO_INFO_PRINT("Adapter powered set to %s", powered ? "true" : "false");
        return true;
    }
    return false;
}

bool BluetoothAgent::set_pairable(bool pairable)
{
    if (set_adapter_property("Pairable", pairable))
    {
        AUDIO_INFO_PRINT("Adapter pairable set to %s", pairable ? "true" : "false");
        return true;
    }
    return false;
}

bool BluetoothAgent::set_discoverable(bool discoverable)
{
    if (set_adapter_property("Discoverable", discoverable))
    {
        AUDIO_INFO_PRINT("Adapter discoverable set to %s", discoverable ? "true" : "false");
        return true;
    }
    return false;
}

bool BluetoothAgent::start_scan()
{
    DBusMessage *reply = call_method(adapter_path_.c_str(), ADAPTER_INTERFACE, "StartDiscovery", DBUS_TYPE_INVALID);
    if (reply)
    {
        dbus_message_unref(reply);
        scanning_ = true;
        AUDIO_INFO_PRINT("Discovery started");
        return true;
    }
    return false;
}

bool BluetoothAgent::stop_scan()
{
    DBusMessage *reply = call_method(adapter_path_.c_str(), ADAPTER_INTERFACE, "StopDiscovery", DBUS_TYPE_INVALID);
    if (reply)
    {
        dbus_message_unref(reply);
        scanning_ = false;
        AUDIO_INFO_PRINT("Discovery stopped");
        return true;
    }
    return false;
}

bool BluetoothAgent::pair(const std::string &device_path)
{
    AUDIO_INFO_PRINT("Pairing device: %s", device_path.c_str());

    DBusMessage *msg = dbus_message_new_method_call(BLUEZ_SERVICE, device_path.c_str(), DEVICE_INTERFACE, "Pair");
    if (!msg)
    {
        AUDIO_ERROR_PRINT("Failed to create Pair method call");
        return false;
    }

    DBusPendingCall *pending = nullptr;
    if (!dbus_connection_send_with_reply(connection_, msg, &pending, DBUS_TIMEOUT_LONG))
    {
        AUDIO_ERROR_PRINT("Failed to send Pair method call");
        dbus_message_unref(msg);
        return false;
    }

    dbus_message_unref(msg);

    if (!pending)
    {
        AUDIO_ERROR_PRINT("Pending call is null");
        return false;
    }

    while (!dbus_pending_call_get_completed(pending))
    {
        dbus_connection_read_write_dispatch(connection_, DBUS_POLL_INTERVAL);
    }

    DBusMessage *reply = dbus_pending_call_steal_reply(pending);
    dbus_pending_call_unref(pending);

    bool success = false;
    if (reply)
    {
        if (dbus_message_get_type(reply) == DBUS_MESSAGE_TYPE_ERROR)
        {
            const char *error_name = dbus_message_get_error_name(reply);
            AUDIO_ERROR_PRINT("Failed to pair device: %s", error_name ? error_name : "unknown error");
        }
        else
        {
            AUDIO_INFO_PRINT("Device paired: %s", device_path.c_str());
            success = true;

            if (!set_trusted(device_path, true))
            {
                AUDIO_ERROR_PRINT("Failed to set device as trusted after pairing");
            }
        }
        dbus_message_unref(reply);
    }
    else
    {
        AUDIO_ERROR_PRINT("No reply received for Pair request");
    }

    return success;
}

bool BluetoothAgent::connect(const std::string &device_path)
{
    AUDIO_INFO_PRINT("Connecting to device: %s", device_path.c_str());
    set_trusted(device_path, true);
    DBusMessage *reply = call_method(device_path.c_str(), DEVICE_INTERFACE, "Connect", DBUS_TYPE_INVALID);
    if (reply)
    {
        dbus_message_unref(reply);
        AUDIO_INFO_PRINT("Device connected: %s", device_path.c_str());
        return true;
    }
    AUDIO_ERROR_PRINT("Failed to connect to device: %s", device_path.c_str());
    return false;
}

bool BluetoothAgent::disconnect(const std::string &device_path)
{
    AUDIO_INFO_PRINT("Disconnecting device: %s", device_path.c_str());
    DBusMessage *reply = call_method(device_path.c_str(), DEVICE_INTERFACE, "Disconnect", DBUS_TYPE_INVALID);
    if (reply)
    {
        dbus_message_unref(reply);
        AUDIO_INFO_PRINT("Device disconnected: %s", device_path.c_str());
        return true;
    }
    AUDIO_ERROR_PRINT("Failed to disconnect device: %s", device_path.c_str());
    return false;
}

bool BluetoothAgent::set_device_property(const char *device_path, const char *property, bool value)
{
    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(BLUEZ_SERVICE, device_path, PROPERTIES_INTERFACE, "Set");
    if (!msg)
    {
        return false;
    }

    DBusMessageIter iter, variant;
    dbus_message_iter_init_append(msg, &iter);
    const char *iface = DEVICE_INTERFACE;
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &iface);
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &property);

    dbus_bool_t dbus_value = value ? TRUE : FALSE;
    dbus_message_iter_open_container(&iter, DBUS_TYPE_VARIANT, "b", &variant);
    dbus_message_iter_append_basic(&variant, DBUS_TYPE_BOOLEAN, &dbus_value);
    dbus_message_iter_close_container(&iter, &variant);

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(connection_, msg, DBUS_TIMEOUT_SHORT, &err);
    dbus_message_unref(msg);

    bool success = !dbus_error_is_set(&err);
    if (!success)
    {
        AUDIO_DEBUG_PRINT("Failed to set device property %s: %s", property, err.message);
        dbus_error_free(&err);
    }

    if (reply)
    {
        dbus_message_unref(reply);
    }
    return success;
}

bool BluetoothAgent::remove(const std::string &device_path)
{
    AUDIO_INFO_PRINT("Removing device: %s", device_path.c_str());

    set_trusted(device_path, false);

    DBusMessage *reply = call_method(device_path.c_str(), DEVICE_INTERFACE, "Disconnect", DBUS_TYPE_INVALID);
    if (reply)
    {
        dbus_message_unref(reply);
    }

    const char *path = device_path.c_str();
    reply = call_method(adapter_path_.c_str(), ADAPTER_INTERFACE, "RemoveDevice", DBUS_TYPE_OBJECT_PATH, &path,
                        DBUS_TYPE_INVALID);
    if (reply)
    {
        dbus_message_unref(reply);
        // maybe updated from signal, but ensure removal here
        std::lock_guard<std::mutex> lock(devices_mutex_);
        auto it = std::remove_if(devices_.begin(), devices_.end(),
                                 [device_path](const BluetoothDevice &dev) { return dev.path == device_path; });
        AUDIO_INFO_PRINT("Device removed: %s", device_path.c_str());
        return true;
    }

    AUDIO_ERROR_PRINT("Failed to remove device: %s", device_path.c_str());
    return false;
}

bool BluetoothAgent::set_trusted(const std::string &device_path, bool trusted)
{
    AUDIO_INFO_PRINT("Setting device trusted to %s: %s", trusted ? "true" : "false", device_path.c_str());

    if (set_device_property(device_path.c_str(), "Trusted", trusted))
    {
        AUDIO_INFO_PRINT("Device trusted set to %s: %s", trusted ? "true" : "false", device_path.c_str());
        return true;
    }
    AUDIO_ERROR_PRINT("Failed to set device trusted: %s", device_path.c_str());
    return false;
}

std::vector<BluetoothDevice> BluetoothAgent::list() const
{
    std::lock_guard<std::mutex> lock(devices_mutex_);
    return devices_;
}

bool BluetoothAgent::set_adapter_property(const char *property, bool value)
{
    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(BLUEZ_SERVICE, adapter_path_.c_str(), PROPERTIES_INTERFACE, "Set");
    if (!msg)
    {
        AUDIO_ERROR_PRINT("Failed to create method call for Set");
        return false;
    }

    DBusMessageIter iter, variant;
    dbus_message_iter_init_append(msg, &iter);
    const char *iface = ADAPTER_INTERFACE;
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &iface);
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &property);

    dbus_bool_t dbus_value = value ? TRUE : FALSE;
    dbus_message_iter_open_container(&iter, DBUS_TYPE_VARIANT, "b", &variant);
    dbus_message_iter_append_basic(&variant, DBUS_TYPE_BOOLEAN, &dbus_value);
    dbus_message_iter_close_container(&iter, &variant);

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(connection_, msg, DBUS_TIMEOUT_SHORT, &err);
    dbus_message_unref(msg);

    bool success = !dbus_error_is_set(&err);
    if (!success)
    {
        AUDIO_ERROR_PRINT("Failed to set adapter property: %s - %s", err.name, err.message);
        dbus_error_free(&err);
    }

    if (reply)
    {
        dbus_message_unref(reply);
    }
    return success;
}

std::string BluetoothAgent::get_device_property(const char *device_path, const char *property)
{
    DBusMessage *msg = dbus_message_new_method_call(BLUEZ_SERVICE, device_path, PROPERTIES_INTERFACE, "Get");
    if (!msg)
    {
        AUDIO_ERROR_PRINT("Failed to create method call for Get");
        return "";
    }

    DBusMessageIter iter;
    dbus_message_iter_init_append(msg, &iter);
    const char *iface = DEVICE_INTERFACE;
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &iface);
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &property);

    DBusError err;
    dbus_error_init(&err);
    DBusMessage *reply = dbus_connection_send_with_reply_and_block(connection_, msg, DBUS_TIMEOUT_SHORT, &err);
    dbus_message_unref(msg);

    std::string result;
    if (reply)
    {
        DBusMessageIter args, variant;
        if (dbus_message_iter_init(reply, &args) && dbus_message_iter_get_arg_type(&args) == DBUS_TYPE_VARIANT)
        {
            dbus_message_iter_recurse(&args, &variant);
            if (dbus_message_iter_get_arg_type(&variant) == DBUS_TYPE_STRING)
            {
                const char *value;
                dbus_message_iter_get_basic(&variant, &value);
                result = value;
            }
        }
        dbus_message_unref(reply);
    }

    if (dbus_error_is_set(&err))
    {
        dbus_error_free(&err);
    }

    return result;
}

DBusMessage *BluetoothAgent::call_method(const char *path, const char *interface, const char *method,
                                         int first_arg_type, ...)
{
    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(BLUEZ_SERVICE, path, interface, method);
    if (!msg)
    {
        AUDIO_ERROR_PRINT("Failed to create method call for %s.%s", interface, method);
        return nullptr;
    }

    if (first_arg_type != DBUS_TYPE_INVALID)
    {
        va_list args;
        va_start(args, first_arg_type);
        if (!dbus_message_append_args_valist(msg, first_arg_type, args))
        {
            AUDIO_ERROR_PRINT("Failed to append arguments for %s.%s", interface, method);
            dbus_message_unref(msg);
            va_end(args);
            return nullptr;
        }
        va_end(args);
    }

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(connection_, msg, DBUS_TIMEOUT_LONG, &err);
    dbus_message_unref(msg);

    if (dbus_error_is_set(&err))
    {
        AUDIO_ERROR_PRINT("Failed to call method %s.%s: %s - %s", interface, method, err.name, err.message);
        dbus_error_free(&err);
        return nullptr;
    }

    return reply;
}

void BluetoothAgent::update_dev_from_message(const char *object_path, DBusMessageIter *iter)
{
    if (dbus_message_iter_get_arg_type(iter) != DBUS_TYPE_ARRAY)
    {
        return;
    }

    DBusMessageIter array_iter;
    dbus_message_iter_recurse(iter, &array_iter);

    while (dbus_message_iter_get_arg_type(&array_iter) == DBUS_TYPE_DICT_ENTRY)
    {
        DBusMessageIter entry_iter;
        const char *interface;

        dbus_message_iter_recurse(&array_iter, &entry_iter);
        dbus_message_iter_get_basic(&entry_iter, &interface);

        if (std::string(interface) == DEVICE_INTERFACE)
        {
            dbus_message_iter_next(&entry_iter);
            update_device_property(object_path, &entry_iter);
            return;
        }
        dbus_message_iter_next(&array_iter);
    }
}

void BluetoothAgent::update_device_property(const char *path, DBusMessageIter *iter)
{
    if (dbus_message_iter_get_arg_type(iter) != DBUS_TYPE_ARRAY)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(devices_mutex_);

    auto dev_iter =
        std::find_if(devices_.begin(), devices_.end(), [path](const BluetoothDevice &dev) { return dev.path == path; });

    bool is_new = (dev_iter == devices_.end());

    if (is_new)
    {
        BluetoothDevice new_device{};
        new_device.path = path;
        devices_.push_back(new_device);
        dev_iter = devices_.end() - 1;
        AUDIO_INFO_PRINT("New device discovered: %s", path);
    }

    DBusMessageIter dict_iter;
    dbus_message_iter_recurse(iter, &dict_iter);

    while (dbus_message_iter_get_arg_type(&dict_iter) == DBUS_TYPE_DICT_ENTRY)
    {
        DBusMessageIter entry_iter;
        const char *property_name;

        dbus_message_iter_recurse(&dict_iter, &entry_iter);
        dbus_message_iter_get_basic(&entry_iter, &property_name);
        dbus_message_iter_next(&entry_iter);

        update_device_property(*dev_iter, property_name, &entry_iter);
        dbus_message_iter_next(&dict_iter);
    }
}

void BluetoothAgent::update_device_property(BluetoothDevice &device, const char *key, DBusMessageIter *iter)
{
    if (dbus_message_iter_get_arg_type(iter) != DBUS_TYPE_VARIANT)
    {
        return;
    }

    DBusMessageIter variant_iter;
    dbus_message_iter_recurse(iter, &variant_iter);

    if (strcmp(key, "Address") == 0 && dbus_message_iter_get_arg_type(&variant_iter) == DBUS_TYPE_STRING)
    {
        const char *address;
        dbus_message_iter_get_basic(&variant_iter, &address);
        device.address = address;
    }
    else if (strcmp(key, "Name") == 0 && dbus_message_iter_get_arg_type(&variant_iter) == DBUS_TYPE_STRING)
    {
        const char *name;
        dbus_message_iter_get_basic(&variant_iter, &name);
        device.name = name;
    }
    else if (strcmp(key, "Alias") == 0 && dbus_message_iter_get_arg_type(&variant_iter) == DBUS_TYPE_STRING)
    {
        const char *alias;
        dbus_message_iter_get_basic(&variant_iter, &alias);
        device.alias = alias;
    }
    else if (strcmp(key, "RSSI") == 0 && dbus_message_iter_get_arg_type(&variant_iter) == DBUS_TYPE_INT16)
    {
        int16_t rssi;
        dbus_message_iter_get_basic(&variant_iter, &rssi);
        device.rssi = rssi;
    }
    else if (strcmp(key, "Paired") == 0 && dbus_message_iter_get_arg_type(&variant_iter) == DBUS_TYPE_BOOLEAN)
    {
        dbus_bool_t paired;
        dbus_message_iter_get_basic(&variant_iter, &paired);
        device.paired = paired ? true : false;
    }
    else if (strcmp(key, "Connected") == 0 && dbus_message_iter_get_arg_type(&variant_iter) == DBUS_TYPE_BOOLEAN)
    {
        dbus_bool_t connected;
        dbus_message_iter_get_basic(&variant_iter, &connected);
        device.connected = connected ? true : false;
    }
    else if (strcmp(key, "Trusted") == 0 && dbus_message_iter_get_arg_type(&variant_iter) == DBUS_TYPE_BOOLEAN)
    {
        dbus_bool_t trusted;
        dbus_message_iter_get_basic(&variant_iter, &trusted);
        device.trusted = trusted ? true : false;
    }
    else if (strcmp(key, "Blocked") == 0 && dbus_message_iter_get_arg_type(&variant_iter) == DBUS_TYPE_BOOLEAN)
    {
        dbus_bool_t blocked;
        dbus_message_iter_get_basic(&variant_iter, &blocked);
        device.blocked = blocked ? true : false;
    }
    else if (strcmp(key, "UUIDs") == 0 && dbus_message_iter_get_arg_type(&variant_iter) == DBUS_TYPE_ARRAY)
    {
        DBusMessageIter array_iter;
        dbus_message_iter_recurse(&variant_iter, &array_iter);
        device.uuids.clear();

        while (dbus_message_iter_get_arg_type(&array_iter) == DBUS_TYPE_STRING)
        {
            const char *uuid;
            dbus_message_iter_get_basic(&array_iter, &uuid);
            device.uuids.push_back(uuid);
            dbus_message_iter_next(&array_iter);
        }
    }
}

void BluetoothAgent::handle_dev_add(DBusMessage *msg)
{
    DBusMessageIter args;
    if (!dbus_message_iter_init(msg, &args) || dbus_message_iter_get_arg_type(&args) != DBUS_TYPE_OBJECT_PATH)
    {
        return;
    }

    const char *object_path;
    dbus_message_iter_get_basic(&args, &object_path);

    if (strstr(object_path, "/org/bluez/") == NULL)
    {
        return;
    }

    dbus_message_iter_next(&args);
    update_dev_from_message(object_path, &args);
}

void BluetoothAgent::handle_dev_del(DBusMessage *msg)
{
    DBusMessageIter args;
    if (!dbus_message_iter_init(msg, &args) || dbus_message_iter_get_arg_type(&args) != DBUS_TYPE_OBJECT_PATH)
    {
        return;
    }

    const char *object_path;
    dbus_message_iter_get_basic(&args, &object_path);

    if (strstr(object_path, "/org/bluez/") == NULL)
    {
        return;
    }

    AUDIO_INFO_PRINT("Device removed from system: %s", object_path);

    std::lock_guard<std::mutex> lock(devices_mutex_);
    auto it = std::remove_if(devices_.begin(), devices_.end(),
                             [object_path](const BluetoothDevice &dev) { return dev.path == object_path; });

    if (it != devices_.end())
    {
        devices_.erase(it, devices_.end());
        AUDIO_INFO_PRINT("Device removed from list: %s", object_path);
    }
}

void BluetoothAgent::handle_dev_chg(DBusMessage *msg)
{
    const char *object_path = dbus_message_get_path(msg);

    if (strstr(object_path, "/org/bluez/") == NULL)
    {
        return;
    }

    DBusMessageIter iter;
    if (!dbus_message_iter_init(msg, &iter))
    {
        return;
    }

    const char *interface = nullptr;
    if (dbus_message_iter_get_arg_type(&iter) == DBUS_TYPE_STRING)
    {
        dbus_message_iter_get_basic(&iter, &interface);
        if (!interface || strcmp(interface, DEVICE_INTERFACE) != 0)
        {
            return;
        }
        dbus_message_iter_next(&iter);
    }

    update_device_property(object_path, &iter);
}

int BluetoothAgent::handle_message(DBusMessage *msg)
{
    const char *interface = dbus_message_get_interface(msg);
    const char *member = dbus_message_get_member(msg);
    AUDIO_DEBUG_PRINT("Received message: %s.%s", interface ? interface : "unknown", member ? member : "unknown");

    if (dbus_message_is_method_call(msg, AGENT_INTERFACE, "RequestPinCode"))
    {
        return handle_request_pincode(msg);
    }
    else if (dbus_message_is_method_call(msg, AGENT_INTERFACE, "DisplayPinCode"))
    {
        return handle_display_pincode(msg);
    }
    else if (dbus_message_is_method_call(msg, AGENT_INTERFACE, "RequestPasskey"))
    {
        return handle_request_passkey(msg);
    }
    else if (dbus_message_is_method_call(msg, AGENT_INTERFACE, "DisplayPasskey"))
    {
        return handle_display_passkey(msg);
    }
    else if (dbus_message_is_method_call(msg, AGENT_INTERFACE, "RequestConfirmation"))
    {
        return handle_request_confirmation(msg);
    }
    else if (dbus_message_is_method_call(msg, AGENT_INTERFACE, "RequestAuthorization"))
    {
        return handle_request_authorization(msg);
    }
    else if (dbus_message_is_method_call(msg, AGENT_INTERFACE, "AuthorizeService"))
    {
        return handle_authorize_service(msg);
    }
    else if (dbus_message_is_method_call(msg, AGENT_INTERFACE, "Cancel"))
    {
        return handle_cancel(msg);
    }
    else if (dbus_message_is_method_call(msg, AGENT_INTERFACE, "Release"))
    {
        AUDIO_INFO_PRINT("Agent is being released by BlueZ");
        send_simple_reply(connection_, msg);
        return DBUS_HANDLER_RESULT_HANDLED;
    }

    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

bool BluetoothAgent::register_agent()
{
    DBusError err;
    dbus_error_init(&err);

    const char *capability = "KeyboardDisplay";
    const char *agent_path = AGENT_PATH;

    DBusMessage *msg =
        dbus_message_new_method_call(BLUEZ_SERVICE, "/org/bluez", AGENT_MANAGER_INTERFACE, "RegisterAgent");
    if (!msg)
    {
        AUDIO_ERROR_PRINT("Failed to create method call for RegisterAgent");
        return false;
    }

    dbus_message_append_args(msg, DBUS_TYPE_OBJECT_PATH, &agent_path, DBUS_TYPE_STRING, &capability, DBUS_TYPE_INVALID);
    DBusMessage *reply = dbus_connection_send_with_reply_and_block(connection_, msg, DBUS_TIMEOUT_SHORT, &err);
    dbus_message_unref(msg);

    if (dbus_error_is_set(&err))
    {
        AUDIO_ERROR_PRINT("Failed to register agent: %s - %s", err.name, err.message);
        dbus_error_free(&err);
        return false;
    }

    if (reply)
    {
        dbus_message_unref(reply);
    }

    msg = dbus_message_new_method_call(BLUEZ_SERVICE, "/org/bluez", AGENT_MANAGER_INTERFACE, "RequestDefaultAgent");
    if (!msg)
    {
        AUDIO_ERROR_PRINT("Failed to create method call for RequestDefaultAgent");
        return false;
    }

    dbus_message_append_args(msg, DBUS_TYPE_OBJECT_PATH, &agent_path, DBUS_TYPE_INVALID);
    reply = dbus_connection_send_with_reply_and_block(connection_, msg, DBUS_TIMEOUT_SHORT, &err);
    dbus_message_unref(msg);

    if (dbus_error_is_set(&err))
    {
        AUDIO_ERROR_PRINT("Failed to set default agent: %s - %s", err.name, err.message);
        dbus_error_free(&err);
        return false;
    }

    if (reply)
    {
        dbus_message_unref(reply);
    }

    return true;
}

int BluetoothAgent::handle_request_pincode(DBusMessage *msg)
{
    const char *device_path;
    if (!dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_INVALID))
    {
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }

    AUDIO_INFO_PRINT("PIN Code Request - Device: %s, waiting for external input...", device_path);

    PairingRequest request;
    request.type = PairingRequestType::PIN_CODE;
    request.device_path = device_path;
    request.device_address = get_device_property(device_path, "Address");
    request.device_name = get_device_property(device_path, "Name");
    if (request.device_name.empty())
    {
        request.device_name = get_device_property(device_path, "Alias");
    }
    notify_pairing_request(request);

    PincodeFuture future;
    std::unique_lock<std::mutex> lock(confirmation_mutex_);
    pincode_promise_ = std::make_unique<std::promise<PincodeResult>>();
    future = pincode_promise_->get_future();
    lock.unlock();

    auto status = future.wait_for(std::chrono::seconds(PAIRING_TIMEOUT_SECONDS));

    if (status != std::future_status::timeout)
    {
        auto result = future.get();
        if (result.first)
        {
            const char *pin = result.second.c_str();
            AUDIO_INFO_PRINT("PIN Code accepted: %s", pin);
            DBusMessage *reply = dbus_message_new_method_return(msg);
            dbus_message_append_args(reply, DBUS_TYPE_STRING, &pin, DBUS_TYPE_INVALID);
            dbus_connection_send(connection_, reply, nullptr);
            dbus_message_unref(reply);
        }
        else
        {
            AUDIO_INFO_PRINT("PIN Code rejected");
            DBusMessage *error = dbus_message_new_error(msg, "org.bluez.Error.Rejected", "PIN Code rejected");
            dbus_connection_send(connection_, error, nullptr);
            dbus_message_unref(error);
        }
    }
    else
    {
        AUDIO_INFO_PRINT("PIN Code timeout, rejecting by default");
        DBusMessage *error = dbus_message_new_error(msg, "org.bluez.Error.Rejected", "PIN Code timeout");
        dbus_connection_send(connection_, error, nullptr);
        dbus_message_unref(error);
    }

    lock.lock();
    pincode_promise_.reset();
    return DBUS_HANDLER_RESULT_HANDLED;
}

int BluetoothAgent::handle_display_pincode(DBusMessage *msg)
{
    const char *device_path;
    const char *pincode;
    if (!dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_STRING, &pincode,
                               DBUS_TYPE_INVALID))
    {
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }

    AUDIO_INFO_PRINT("Display PIN Code - Device: %s, PIN: %s", device_path, pincode);

    PairingRequest request;
    request.type = PairingRequestType::DISPLAY_PINCODE;
    request.device_path = device_path;
    request.device_address = get_device_property(device_path, "Address");
    request.device_name = get_device_property(device_path, "Name");
    if (request.device_name.empty())
    {
        request.device_name = get_device_property(device_path, "Alias");
    }
    notify_pairing_request(request);

    send_simple_reply(connection_, msg);
    return DBUS_HANDLER_RESULT_HANDLED;
}

int BluetoothAgent::handle_request_passkey(DBusMessage *msg)
{
    const char *device_path;
    if (!dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_INVALID))
    {
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }

    AUDIO_INFO_PRINT("Passkey Request - Device: %s, waiting for external input...", device_path);

    PairingRequest request;
    request.type = PairingRequestType::PASSKEY;
    request.device_path = device_path;
    request.device_address = get_device_property(device_path, "Address");
    request.device_name = get_device_property(device_path, "Name");
    if (request.device_name.empty())
    {
        request.device_name = get_device_property(device_path, "Alias");
    }
    notify_pairing_request(request);

    PasskeyFuture future;
    std::unique_lock<std::mutex> lock(confirmation_mutex_);
    passkey_promise_ = std::make_unique<std::promise<PasskeyResult>>();
    future = passkey_promise_->get_future();
    lock.unlock();

    auto status = future.wait_for(std::chrono::seconds(PAIRING_TIMEOUT_SECONDS));

    if (status != std::future_status::timeout)
    {
        auto result = future.get();
        if (result.first)
        {
            dbus_uint32_t pk = result.second;
            AUDIO_INFO_PRINT("Passkey accepted: %06u", pk);
            DBusMessage *reply = dbus_message_new_method_return(msg);
            dbus_message_append_args(reply, DBUS_TYPE_UINT32, &pk, DBUS_TYPE_INVALID);
            dbus_connection_send(connection_, reply, nullptr);
            dbus_message_unref(reply);
        }
        else
        {
            AUDIO_INFO_PRINT("Passkey rejected");
            DBusMessage *error = dbus_message_new_error(msg, "org.bluez.Error.Rejected", "Passkey rejected");
            dbus_connection_send(connection_, error, nullptr);
            dbus_message_unref(error);
        }
    }
    else
    {
        AUDIO_INFO_PRINT("Passkey timeout, rejecting by default");
        DBusMessage *error = dbus_message_new_error(msg, "org.bluez.Error.Rejected", "Passkey timeout");
        dbus_connection_send(connection_, error, nullptr);
        dbus_message_unref(error);
    }

    lock.lock();
    passkey_promise_.reset();
    return DBUS_HANDLER_RESULT_HANDLED;
}

int BluetoothAgent::handle_display_passkey(DBusMessage *msg)
{
    const char *device_path;
    dbus_uint32_t passkey;
    dbus_uint16_t entered;
    if (!dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_UINT32, &passkey,
                               DBUS_TYPE_UINT16, &entered, DBUS_TYPE_INVALID))
    {
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }

    AUDIO_INFO_PRINT("Display Passkey - Device: %s, Passkey: %06u (entered: %u digits)", device_path, passkey, entered);

    PairingRequest request;
    request.type = PairingRequestType::DISPLAY_PASSKEY;
    request.device_path = device_path;
    request.device_address = get_device_property(device_path, "Address");
    request.device_name = get_device_property(device_path, "Name");
    if (request.device_name.empty())
    {
        request.device_name = get_device_property(device_path, "Alias");
    }
    request.passkey = passkey;
    notify_pairing_request(request);

    send_simple_reply(connection_, msg);
    return DBUS_HANDLER_RESULT_HANDLED;
}

int BluetoothAgent::handle_request_confirmation(DBusMessage *msg)
{
    const char *device_path;
    dbus_uint32_t passkey;
    if (!dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_UINT32, &passkey,
                               DBUS_TYPE_INVALID))
    {
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }

    AUDIO_INFO_PRINT("Pairing Confirmation - Device: %s, Passkey: %06u, waiting for external confirmation...",
                     device_path, passkey);

    PairingRequest request;
    request.type = PairingRequestType::CONFIRMATION;
    request.device_path = device_path;
    request.device_address = get_device_property(device_path, "Address");
    request.device_name = get_device_property(device_path, "Name");
    if (request.device_name.empty())
    {
        request.device_name = get_device_property(device_path, "Alias");
    }
    request.passkey = passkey;
    notify_pairing_request(request);

    ConfirmationFuture future;
    std::unique_lock<std::mutex> lock(confirmation_mutex_);
    confirmation_promise_ = std::make_unique<std::promise<bool>>();
    future = confirmation_promise_->get_future();
    lock.unlock();

    auto status = future.wait_for(std::chrono::seconds(PAIRING_TIMEOUT_SECONDS));

    if (status != std::future_status::timeout)
    {
        bool accept = future.get();
        if (accept)
        {
            AUDIO_INFO_PRINT("Pairing confirmed by external program");
            DBusMessage *reply = dbus_message_new_method_return(msg);
            if (reply)
            {
                dbus_connection_send(connection_, reply, nullptr);
                dbus_message_unref(reply);
            }
        }
        else
        {
            AUDIO_INFO_PRINT("Pairing rejected by external program");
            DBusMessage *error = dbus_message_new_error(msg, "org.bluez.Error.Rejected", "Pairing rejected");
            dbus_connection_send(connection_, error, nullptr);
            dbus_message_unref(error);
        }
    }
    else
    {
        AUDIO_INFO_PRINT("Pairing confirmation timeout, rejecting by default");
        DBusMessage *error = dbus_message_new_error(msg, "org.bluez.Error.Rejected", "Confirmation timeout");
        dbus_connection_send(connection_, error, nullptr);
        dbus_message_unref(error);
    }

    lock.lock();
    confirmation_promise_.reset();
    return DBUS_HANDLER_RESULT_HANDLED;
}

int BluetoothAgent::handle_request_authorization(DBusMessage *msg)
{
    const char *device_path;
    if (!dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_INVALID))
    {
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }

    AUDIO_INFO_PRINT("Authorization Request - Device: %s (auto-accepting)", device_path);

    PairingRequest request;
    request.type = PairingRequestType::AUTHORIZATION;
    request.device_path = device_path;
    request.device_address = get_device_property(device_path, "Address");
    request.device_name = get_device_property(device_path, "Name");
    if (request.device_name.empty())
    {
        request.device_name = get_device_property(device_path, "Alias");
    }
    notify_pairing_request(request);

    send_simple_reply(connection_, msg);
    return DBUS_HANDLER_RESULT_HANDLED;
}

int BluetoothAgent::handle_authorize_service(DBusMessage *msg)
{
    const char *device_path;
    const char *uuid;
    if (!dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_STRING, &uuid,
                               DBUS_TYPE_INVALID))
    {
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }

    AUDIO_INFO_PRINT("Service Authorization - Device: %s, UUID: %s (auto-accepting)", device_path, uuid);
    send_simple_reply(connection_, msg);
    return DBUS_HANDLER_RESULT_HANDLED;
}

int BluetoothAgent::handle_cancel(DBusMessage *msg)
{
    AUDIO_INFO_PRINT("Pairing operation cancelled");

    {
        std::lock_guard<std::mutex> lock(confirmation_mutex_);
        if (confirmation_promise_)
        {
            try
            {
                confirmation_promise_->set_value(false);
            }
            catch (...)
            {
            }
            confirmation_promise_.reset();
        }
        if (passkey_promise_)
        {
            try
            {
                passkey_promise_->set_value(PasskeyResult(false, 0));
            }
            catch (...)
            {
            }
            passkey_promise_.reset();
        }
        if (pincode_promise_)
        {
            try
            {
                pincode_promise_->set_value(PincodeResult(false, ""));
            }
            catch (...)
            {
            }
            pincode_promise_.reset();
        }
    }

    send_simple_reply(connection_, msg);
    return DBUS_HANDLER_RESULT_HANDLED;
}

void BluetoothAgent::set_pairing_request_callback(PairingRequestCallback callback)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    pairing_request_callback_ = callback;
    AUDIO_INFO_PRINT("Pairing request callback registered");
}

void BluetoothAgent::notify_pairing_request(const PairingRequest &request)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (pairing_request_callback_)
    {
        try
        {
            pairing_request_callback_(request);
        }
        catch (const std::exception &e)
        {
            AUDIO_ERROR_PRINT("Exception in pairing request callback: %s", e.what());
        }
        catch (...)
        {
            AUDIO_ERROR_PRINT("Unknown exception in pairing request callback");
        }
    }
}

void BluetoothAgent::set_confirmation_result(bool accept)
{
    std::lock_guard<std::mutex> lock(confirmation_mutex_);
    if (confirmation_promise_)
    {
        try
        {
            confirmation_promise_->set_value(accept);
        }
        catch (const std::future_error &)
        {
            AUDIO_ERROR_PRINT("Confirmation already set");
        }
        AUDIO_INFO_PRINT("Confirmation result set: %s", accept ? "accepted" : "rejected");
    }
}

void BluetoothAgent::set_passkey_result(bool accept, uint32_t passkey)
{
    std::lock_guard<std::mutex> lock(confirmation_mutex_);
    if (passkey_promise_)
    {
        try
        {
            passkey_promise_->set_value(PasskeyResult(accept, passkey));
        }
        catch (const std::future_error &)
        {
            AUDIO_ERROR_PRINT("Passkey already set");
        }
        AUDIO_INFO_PRINT("Passkey result set: %s, passkey: %u", accept ? "accepted" : "rejected", passkey);
    }
}

void BluetoothAgent::set_pincode_result(bool accept, const std::string &pincode)
{
    std::lock_guard<std::mutex> lock(confirmation_mutex_);
    if (pincode_promise_)
    {
        try
        {
            pincode_promise_->set_value(PincodeResult(accept, pincode));
        }
        catch (const std::future_error &)
        {
            AUDIO_ERROR_PRINT("Pincode already set");
        }
        AUDIO_INFO_PRINT("Pincode result set: %s, pincode: %s", accept ? "accepted" : "rejected", pincode.c_str());
    }
}

void BluetoothAgent::load_existing_devices()
{
    AUDIO_INFO_PRINT("Loading existing devices...");

    DBusMessage *msg = dbus_message_new_method_call(BLUEZ_SERVICE, "/", OBJECT_MANAGER_INTERFACE, "GetManagedObjects");

    if (!msg)
    {
        AUDIO_ERROR_PRINT("Failed to create GetManagedObjects method call");
        return;
    }

    DBusError err;
    dbus_error_init(&err);
    DBusMessage *reply = dbus_connection_send_with_reply_and_block(connection_, msg, DBUS_TIMEOUT_SHORT, &err);
    dbus_message_unref(msg);

    if (dbus_error_is_set(&err))
    {
        AUDIO_ERROR_PRINT("Failed to get managed objects: %s - %s", err.name, err.message);
        dbus_error_free(&err);
        return;
    }

    if (!reply)
    {
        AUDIO_ERROR_PRINT("No reply from GetManagedObjects");
        return;
    }

    DBusMessageIter iter, array_iter;
    if (!dbus_message_iter_init(reply, &iter) || dbus_message_iter_get_arg_type(&iter) != DBUS_TYPE_ARRAY)
    {
        dbus_message_unref(reply);
        return;
    }

    dbus_message_iter_recurse(&iter, &array_iter);

    int device_count = 0;
    while (dbus_message_iter_get_arg_type(&array_iter) == DBUS_TYPE_DICT_ENTRY)
    {
        DBusMessageIter dict_entry_iter;
        const char *object_path;

        dbus_message_iter_recurse(&array_iter, &dict_entry_iter);
        dbus_message_iter_get_basic(&dict_entry_iter, &object_path);

        if (strstr(object_path, "/org/bluez/") != NULL)
        {
            dbus_message_iter_next(&dict_entry_iter);
            update_dev_from_message(object_path, &dict_entry_iter);
            device_count++;
        }

        dbus_message_iter_next(&array_iter);
    }

    dbus_message_unref(reply);
    AUDIO_INFO_PRINT("Loaded %d existing devices", device_count);
}

std::string BluetoothDevice::translate_uuid(const std::string &uuid)
{
    static const std::map<std::string, std::string> uuid_map = {
        // Generic UUIDs
        {"00001800-0000-1000-8000-00805f9b34fb", "Generic Access"},
        {"00001801-0000-1000-8000-00805f9b34fb", "Generic Attribute"},

        // Audio/Video UUIDs
        {"0000110a-0000-1000-8000-00805f9b34fb", "A2DP Source"},
        {"0000110b-0000-1000-8000-00805f9b34fb", "A2DP Sink"},
        {"0000110c-0000-1000-8000-00805f9b34fb", "AVRCP Target"},
        {"0000110d-0000-1000-8000-00805f9b34fb", "Advanced Audio"},
        {"0000110e-0000-1000-8000-00805f9b34fb", "AVRCP Controller"},
        {"0000110f-0000-1000-8000-00805f9b34fb", "AVRCP"},

        // Telephony UUIDs
        {"0000111e-0000-1000-8000-00805f9b34fb", "Handsfree"},
        {"0000111f-0000-1000-8000-00805f9b34fb", "Handsfree Audio Gateway"},
        {"00001108-0000-1000-8000-00805f9b34fb", "Headset"},
        {"00001112-0000-1000-8000-00805f9b34fb", "Headset Audio Gateway"},
        {"00001131-0000-1000-8000-00805f9b34fb", "Headset HS"},

        // HID UUIDs
        {"00001124-0000-1000-8000-00805f9b34fb", "HID"},
        {"00001200-0000-1000-8000-00805f9b34fb", "PnP Information"},

        // Serial Port UUIDs
        {"00001101-0000-1000-8000-00805f9b34fb", "Serial Port"},

        // Object Push UUIDs
        {"00001105-0000-1000-8000-00805f9b34fb", "OBEX Object Push"},
        {"00001106-0000-1000-8000-00805f9b34fb", "OBEX File Transfer"},

        // Network UUIDs
        {"00001115-0000-1000-8000-00805f9b34fb", "PANU"},
        {"00001116-0000-1000-8000-00805f9b34fb", "NAP"},
        {"00001117-0000-1000-8000-00805f9b34fb", "GN"},

        // Other common UUIDs
        {"0000112d-0000-1000-8000-00805f9b34fb", "SIM Access"},
        {"0000112f-0000-1000-8000-00805f9b34fb", "Phonebook Access PCE"},
        {"00001130-0000-1000-8000-00805f9b34fb", "Phonebook Access PSE"},
        {"00001132-0000-1000-8000-00805f9b34fb", "Message Access Server"},
        {"00001133-0000-1000-8000-00805f9b34fb", "Message Notification Server"},
        {"00001134-0000-1000-8000-00805f9b34fb", "Message Access Profile"},
    };

    auto it = uuid_map.find(uuid);
    if (it != uuid_map.end())
    {
        return it->second + " (" + uuid + ")";
    }

    return uuid;
}

#endif