#include "bluetooth_agent.h"
#if defined(AUDIO_DRIVER_HAVE_DBUS)

#define BLUEZ_SERVICE "org.bluez"
#define AGENT_INTERFACE "org.bluez.Agent1"
#define AGENT_MANAGER_INTERFACE "org.bluez.AgentManager1"
#define ADAPTER_INTERFACE "org.bluez.Adapter1"
#define DEVICE_INTERFACE "org.bluez.Device1"
#define PROPERTIES_INTERFACE "org.freedesktop.DBus.Properties"
#define OBJECT_MANAGER_INTERFACE "org.freedesktop.DBus.ObjectManager"
#define AGENT_PATH "/org/bluez/agent"
#define AGENT_SERVICE "org.bluez.agent"

#include <algorithm>
#include <cstring>
#include <sstream>

static DBusHandlerResult signal_filter_callback(DBusConnection *, DBusMessage *msg, void *user_data)
{
    BluetoothAgent *agent = static_cast<BluetoothAgent *>(user_data);
    if (dbus_message_is_signal(msg, "org.freedesktop.DBus.ObjectManager", "InterfacesAdded"))
    {
        agent->handle_dev_add(msg);
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
    return agent->handle_message(msg);
}

BluetoothAgent::BluetoothAgent(const std::string &adapter_path)
    : connection_(nullptr), adapter_path_(adapter_path), running_(false), scanning_(false)
{
}

BluetoothAgent::~BluetoothAgent()
{
    stop_dbus_loop();
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
        AUDIO_ERROR_PRINT("Failed to get system bus: %s - %s", err.name, err.message);
        dbus_error_free(&err);
        return false;
    }

    if (!connection_)
    {
        AUDIO_ERROR_PRINT("Failed to get system bus: connection is null");
        return false;
    }

    dbus_bus_add_match(connection_, "type='signal',interface='org.freedesktop.DBus.ObjectManager'", &err);
    if (dbus_error_is_set(&err))
    {
        AUDIO_ERROR_PRINT("Failed to add signal match: %s - %s", err.name, err.message);
        dbus_error_free(&err);
        return false;
    }

    dbus_connection_add_filter(connection_, signal_filter_callback, this, nullptr);

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

    return true;
}

void BluetoothAgent::start_dbus_loop()
{
    running_ = true;
    dbus_thread_ = std::thread([this]() {
        while (running_)
        {
            dbus_connection_read_write_dispatch(connection_, 100);
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
    DBusMessage *reply = call_method(device_path.c_str(), DEVICE_INTERFACE, "Pair", DBUS_TYPE_INVALID);
    if (reply)
    {
        dbus_message_unref(reply);
        AUDIO_INFO_PRINT("Device paired: %s", device_path.c_str());
        return true;
    }
    AUDIO_ERROR_PRINT("Failed to pair device: %s", device_path.c_str());
    return false;
}

bool BluetoothAgent::connect(const std::string &device_path)
{
    AUDIO_INFO_PRINT("Connecting to device: %s", device_path.c_str());
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

bool BluetoothAgent::remove(const std::string &device_path)
{
    AUDIO_INFO_PRINT("Removing device: %s", device_path.c_str());
    const char *path = device_path.c_str();
    DBusMessage *reply = call_method(adapter_path_.c_str(), ADAPTER_INTERFACE, "RemoveDevice", DBUS_TYPE_OBJECT_PATH,
                                     &path, DBUS_TYPE_INVALID);
    if (reply)
    {
        dbus_message_unref(reply);
        AUDIO_INFO_PRINT("Device removed: %s", device_path.c_str());
        return true;
    }
    AUDIO_ERROR_PRINT("Failed to remove device: %s", device_path.c_str());
    return false;
}

std::vector<BluetoothDevice> BluetoothAgent::list() const
{
    std::lock_guard<std::mutex> lock(devices_mutex_);
    return devices_;
}

// void BluetoothAgent::info(const std::string &device_path) const
// {
//     std::lock_guard<std::mutex> lock(devices_mutex_);

//     auto dev_iter = std::find_if(devices_.begin(), devices_.end(),
//                                  [device_path](const BluetoothDevice &dev) { return dev.path == device_path; });
//     if (dev_iter != devices_.end())
//     {
//         const BluetoothDevice &dev = *dev_iter;
//         std::stringstream ss;
//         ss << "Device Info:\n"
//            << "Path: " << dev.path << "\n"
//            << "Address: " << dev.address << "\n"
//            << "Name: " << dev.name << "\n"
//            << "Alias: " << dev.alias << "\n"
//            << "RSSI: " << dev.rssi << "\n"
//            << "Paired: " << (dev.paired ? "true" : "false") << "\n"
//            << "Connected: " << (dev.connected ? "true" : "false") << "\n"
//            << "Trusted: " << (dev.trusted ? "true" : "false") << "\n"
//            << "Blocked: " << (dev.blocked ? "true" : "false") << "\n"
//            << "UUIDs:\n";
//         for (const auto &uuid : dev.uuids)
//         {
//             ss << "  " << uuid << "\n";
//         }
//         AUDIO_INFO_PRINT("%s", ss.str().c_str());
//     }
//     else
//     {
//         AUDIO_ERROR_PRINT("Device not found: %s", device_path.c_str());
//     }
// }

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

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(connection_, msg, 5000, &err);
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
    DBusMessage *reply = dbus_connection_send_with_reply_and_block(connection_, msg, 5000, &err);
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

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(connection_, msg, 30000, &err);
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

    BluetoothDevice device{};
    device.path = path;
    DBusMessageIter dict_iter;
    dbus_message_iter_recurse(iter, &dict_iter);

    while (dbus_message_iter_get_arg_type(&dict_iter) == DBUS_TYPE_DICT_ENTRY)
    {
        DBusMessageIter entry_iter;
        const char *property_name;

        dbus_message_iter_recurse(&dict_iter, &entry_iter);
        dbus_message_iter_get_basic(&entry_iter, &property_name);
        dbus_message_iter_next(&entry_iter);

        update_device_property(device, property_name, &entry_iter);
        dbus_message_iter_next(&dict_iter);
    }

    std::lock_guard<std::mutex> lock(devices_mutex_);
    auto dev_iter =
        std::find_if(devices_.begin(), devices_.end(), [path](const BluetoothDevice &dev) { return dev.path == path; });
    if (dev_iter == devices_.end())
    {
        devices_.push_back(std::move(device));
    }
    else
    {
        *dev_iter = std::move(device);
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
    if (strstr(object_path, "/dev_") == NULL)
    {
        return;
    }

    dbus_message_iter_next(&args);
    update_dev_from_message(object_path, &args);
}

void BluetoothAgent::handle_dev_chg(DBusMessage *msg)
{
    const char *object_path = dbus_message_get_path(msg);
    if (strstr(object_path, "/dev_") == NULL)
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

DBusHandlerResult BluetoothAgent::handle_message(DBusMessage *msg)
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
        DBusMessage *reply = dbus_message_new_method_return(msg);
        dbus_connection_send(connection_, reply, nullptr);
        dbus_message_unref(reply);
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
    DBusMessage *reply = dbus_connection_send_with_reply_and_block(connection_, msg, 5000, &err);
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
    reply = dbus_connection_send_with_reply_and_block(connection_, msg, 5000, &err);
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

DBusHandlerResult BluetoothAgent::handle_request_pincode(DBusMessage *msg)
{
    const char *device_path;
    if (dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_INVALID))
    {
        const char *pincode = "1234";
        AUDIO_INFO_PRINT("Received RequestPinCode for device: %s, send PIN: %s", device_path, pincode);
        DBusMessage *reply = dbus_message_new_method_return(msg);
        dbus_message_append_args(reply, DBUS_TYPE_STRING, &pincode, DBUS_TYPE_INVALID);
        dbus_connection_send(connection_, reply, nullptr);
        dbus_message_unref(reply);
        return DBUS_HANDLER_RESULT_HANDLED;
    }
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

DBusHandlerResult BluetoothAgent::handle_display_pincode(DBusMessage *msg)
{
    const char *device_path;
    const char *pincode;
    if (dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_STRING, &pincode,
                              DBUS_TYPE_INVALID))
    {
        AUDIO_INFO_PRINT("Received DisplayPinCode for device: %s, PIN: %s", device_path, pincode);
        DBusMessage *reply = dbus_message_new_method_return(msg);
        dbus_connection_send(connection_, reply, nullptr);
        dbus_message_unref(reply);
        return DBUS_HANDLER_RESULT_HANDLED;
    }
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

DBusHandlerResult BluetoothAgent::handle_request_passkey(DBusMessage *msg)
{
    const char *device_path;
    if (dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_INVALID))
    {
        AUDIO_INFO_PRINT("Received RequestPasskey for device: %s", device_path);
        dbus_uint32_t passkey = 123456;
        AUDIO_INFO_PRINT("Sending Passkey: %06u", passkey);

        DBusMessage *reply = dbus_message_new_method_return(msg);
        dbus_message_append_args(reply, DBUS_TYPE_UINT32, &passkey, DBUS_TYPE_INVALID);
        dbus_connection_send(connection_, reply, nullptr);
        dbus_message_unref(reply);
        return DBUS_HANDLER_RESULT_HANDLED;
    }
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

DBusHandlerResult BluetoothAgent::handle_display_passkey(DBusMessage *msg)
{
    const char *device_path;
    dbus_uint32_t passkey;
    dbus_uint16_t entered;
    if (dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_UINT32, &passkey,
                              DBUS_TYPE_UINT16, &entered, DBUS_TYPE_INVALID))
    {
        AUDIO_INFO_PRINT("Received DisplayPasskey for device: %s, Passkey: %06u, Entered: %u", device_path, passkey,
                         entered);
        DBusMessage *reply = dbus_message_new_method_return(msg);
        dbus_connection_send(connection_, reply, nullptr);
        dbus_message_unref(reply);
        return DBUS_HANDLER_RESULT_HANDLED;
    }
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

DBusHandlerResult BluetoothAgent::handle_request_confirmation(DBusMessage *msg)
{
    const char *device_path;
    dbus_uint32_t passkey;
    if (dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_UINT32, &passkey,
                              DBUS_TYPE_INVALID))
    {
        AUDIO_INFO_PRINT("Received RequestConfirmation for device: %s, Passkey: %06u", device_path, passkey);
        DBusMessage *reply = dbus_message_new_method_return(msg);
        dbus_connection_send(connection_, reply, nullptr);
        dbus_message_unref(reply);
        return DBUS_HANDLER_RESULT_HANDLED;
    }
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

DBusHandlerResult BluetoothAgent::handle_request_authorization(DBusMessage *msg)
{
    const char *device_path;
    if (dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_INVALID))
    {
        AUDIO_INFO_PRINT("Received RequestAuthorization for device: %s", device_path);
        DBusMessage *reply = dbus_message_new_method_return(msg);
        dbus_connection_send(connection_, reply, nullptr);
        dbus_message_unref(reply);
        return DBUS_HANDLER_RESULT_HANDLED;
    }
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

DBusHandlerResult BluetoothAgent::handle_authorize_service(DBusMessage *msg)
{
    const char *device_path;
    const char *uuid;
    if (dbus_message_get_args(msg, nullptr, DBUS_TYPE_OBJECT_PATH, &device_path, DBUS_TYPE_STRING, &uuid,
                              DBUS_TYPE_INVALID))
    {
        AUDIO_INFO_PRINT("Received AuthorizeService for device: %s, UUID: %s", device_path, uuid);
        DBusMessage *reply = dbus_message_new_method_return(msg);
        dbus_connection_send(connection_, reply, nullptr);
        dbus_message_unref(reply);
        return DBUS_HANDLER_RESULT_HANDLED;
    }
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

DBusHandlerResult BluetoothAgent::handle_cancel(DBusMessage *msg)
{
    AUDIO_INFO_PRINT("Received Cancel");
    DBusMessage *reply = dbus_message_new_method_return(msg);
    dbus_connection_send(connection_, reply, nullptr);
    dbus_message_unref(reply);
    return DBUS_HANDLER_RESULT_HANDLED;
}

#endif