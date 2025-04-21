#include "audio_monitor.h"
#include "audio_interface.h"

#if WINDOWS_OS_ENVIRONMENT
// must be included after mmdeviceapi.h
#include <functiondiscoverykeys_devpkey.h>

#include <utility>
std::string AudioMonitor::WideToUtf8(const std::wstring &wstr)
{
    if (wstr.empty())
    {
        return {};
    }

    int size_needed = WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, nullptr, 0, nullptr, nullptr);
    std::string str(size_needed, 0);
    WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, &str[0], size_needed, nullptr, nullptr);

    // Remove NULL terminator from the end of the string
    if (!str.empty() && str.back() == '\0')
    {
        str.pop_back();
    }

    return str;
}

std::wstring AudioMonitor::Utf8ToWide(const std::string &str)
{
    if (str.empty())
    {
        return {};
    }

    int size_needed = MultiByteToWideChar(CP_UTF8, 0, str.c_str(), -1, nullptr, 0);
    std::wstring wstr(size_needed, 0);
    MultiByteToWideChar(CP_UTF8, 0, str.c_str(), -1, &wstr[0], size_needed);

    // Remove NULL terminator from the end of the string
    if (!wstr.empty() && wstr.back() == L'\0')
    {
        wstr.pop_back();
    }

    return wstr;
}

// DeviceNotificationClient implementation
DeviceNotificationClient::DeviceNotificationClient(
    std::function<void(AudioDeviceEvent, const AudioDeviceInfo &)> callback)
    : callback_(std::move(callback)), ref_count_(1), enumerator_(nullptr)
{
    HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                                  reinterpret_cast<void **>(&enumerator_));

    if (SUCCEEDED(hr) && enumerator_)
    {
        enumerator_->RegisterEndpointNotificationCallback(this);
    }
}

DeviceNotificationClient::~DeviceNotificationClient()
{
    if (enumerator_)
    {
        enumerator_->UnregisterEndpointNotificationCallback(this);
        enumerator_->Release();
        enumerator_ = nullptr;
    }
}

ULONG STDMETHODCALLTYPE DeviceNotificationClient::AddRef()
{
    return InterlockedIncrement(&ref_count_);
}

ULONG STDMETHODCALLTYPE DeviceNotificationClient::Release()
{
    ULONG ref = InterlockedDecrement(&ref_count_);
    if (ref == 0)
    {
        delete this;
    }
    return ref;
}

HRESULT STDMETHODCALLTYPE DeviceNotificationClient::QueryInterface(REFIID riid, void **ppvObject)
{
    if (riid == __uuidof(IUnknown) || riid == __uuidof(IMMNotificationClient))
    {
        *ppvObject = static_cast<IMMNotificationClient *>(this);
        AddRef();
        return S_OK;
    }

    *ppvObject = nullptr;
    return E_NOINTERFACE;
}

HRESULT STDMETHODCALLTYPE DeviceNotificationClient::OnDeviceStateChanged(LPCWSTR pwstrDeviceId, DWORD dwNewState)
{
    bool is_active = false;

    switch (dwNewState)
    {
    case DEVICE_STATE_ACTIVE:
        is_active = true;
        break;
    case DEVICE_STATE_DISABLED:
    case DEVICE_STATE_NOTPRESENT:
    case DEVICE_STATE_UNPLUGGED:
        is_active = false;
        break;
    }

    // Get device information
    AudioDeviceInfo device_info = GetDeviceInfo(pwstrDeviceId);
    device_info.is_active = is_active;

    if (callback_)
    {
        callback_(AudioDeviceEvent::StateChanged, device_info);
    }

    return S_OK;
}

HRESULT STDMETHODCALLTYPE DeviceNotificationClient::OnDeviceAdded(LPCWSTR pwstrDeviceId)
{
    AudioDeviceInfo device_info = GetDeviceInfo(pwstrDeviceId);

    if (callback_)
    {
        callback_(AudioDeviceEvent::Added, device_info);
    }

    return S_OK;
}

HRESULT STDMETHODCALLTYPE DeviceNotificationClient::OnDeviceRemoved(LPCWSTR pwstrDeviceId)
{
    AudioDeviceInfo device_info;
    device_info.id = AudioMonitor::WideToUtf8(std::wstring(pwstrDeviceId));

    if (callback_)
    {
        callback_(AudioDeviceEvent::Removed, device_info);
    }

    return S_OK;
}

HRESULT STDMETHODCALLTYPE DeviceNotificationClient::OnDefaultDeviceChanged(EDataFlow flow, ERole role,
                                                                           LPCWSTR pwstrDefaultDeviceId)
{
    // Only focus on the default audio endpoint device
    if (role != eConsole)
    {
        return S_OK;
    }

    AudioDeviceType type;
    if (flow == eRender)
    {
        type = AudioDeviceType::Playback;
    }
    else if (flow == eCapture)
    {
        type = AudioDeviceType::Capture;
    }
    else
    {
        return S_OK;
    }

    AudioDeviceInfo device_info;
    if (pwstrDefaultDeviceId)
    {
        device_info = GetDeviceInfo(pwstrDefaultDeviceId, type);
        device_info.is_default = true;
    }
    else
    {
        // No default device
        device_info.type = type;
        device_info.is_default = false;
    }

    if (callback_)
    {
        callback_(AudioDeviceEvent::DefaultChanged, device_info);
    }

    return S_OK;
}

HRESULT STDMETHODCALLTYPE DeviceNotificationClient::OnPropertyValueChanged(LPCWSTR /*pwstrDeviceId*/,
                                                                           const PROPERTYKEY /*key*/)
{
    // We do not handle property changes
    return S_OK;
}

AudioDeviceInfo DeviceNotificationClient::GetDeviceInfo(LPCWSTR pwstrDeviceId, AudioDeviceType type) const
{
    AudioDeviceInfo info;
    IMMDevice *device = nullptr;
    IPropertyStore *props = nullptr;
    HRESULT hr;

    // Convert wide character device ID to UTF-8 string
    info.id = AudioMonitor::WideToUtf8(std::wstring(pwstrDeviceId));

    if (!enumerator_)
    {
        return info;
    }

    // Get device
    hr = enumerator_->GetDevice(pwstrDeviceId, &device);
    if (FAILED(hr) || !device)
    {
        return info;
    }

    // Determine device type
    IMMEndpoint *endpoint = nullptr;
    hr = device->QueryInterface(__uuidof(IMMEndpoint), reinterpret_cast<void **>(&endpoint));
    if (SUCCEEDED(hr) && endpoint)
    {
        EDataFlow flow;
        hr = endpoint->GetDataFlow(&flow);
        if (SUCCEEDED(hr))
        {
            if (flow == eRender)
            {
                info.type = AudioDeviceType::Playback;
            }
            else if (flow == eCapture)
            {
                info.type = AudioDeviceType::Capture;
            }
        }
        endpoint->Release();
    }

    // If a type is specified and does not match, return
    if (type != AudioDeviceType::All && type != info.type)
    {
        device->Release();
        return info;
    }

    // Get device properties
    hr = device->OpenPropertyStore(STGM_READ, &props);
    if (SUCCEEDED(hr) && props)
    {
        PROPVARIANT var;
        PropVariantInit(&var);

        // Get friendly name
        hr = props->GetValue(PKEY_Device_FriendlyName, &var);
        if (SUCCEEDED(hr) && var.vt == VT_LPWSTR)
        {
            info.name = AudioMonitor::WideToUtf8(var.pwszVal);
        }
        PropVariantClear(&var);

        props->Release();
    }

    // Check device status
    DWORD state;
    hr = device->GetState(&state);
    if (SUCCEEDED(hr))
    {
        info.is_active = (state == DEVICE_STATE_ACTIVE);
    }

    // Check if it is the default device
    IMMDevice *default_device = nullptr;
    EDataFlow flow = (info.type == AudioDeviceType::Playback) ? eRender : eCapture;
    hr = enumerator_->GetDefaultAudioEndpoint(flow, eConsole, &default_device);
    if (SUCCEEDED(hr) && default_device)
    {
        LPWSTR default_id = nullptr;
        hr = default_device->GetId(&default_id);
        if (SUCCEEDED(hr) && default_id)
        {
            info.is_default = (wcscmp(pwstrDeviceId, default_id) == 0);
            CoTaskMemFree(default_id);
        }
        default_device->Release();
    }

    device->Release();
    return info;
}

AudioDeviceInfo AudioMonitor::GetDefaultDevice(AudioDeviceType type)
{
    if (!enumerator_)
    {
        return {};
    }

    // If all types are requested, default to returning playback device
    EDataFlow flow = (type == AudioDeviceType::Capture) ? eCapture : eRender;

    IMMDevice *device = nullptr;
    HRESULT hr = enumerator_->GetDefaultAudioEndpoint(flow, eConsole, &device);
    if (SUCCEEDED(hr) && device)
    {
        AudioDeviceInfo info = GetDeviceInfo(device, type);
        info.is_default = true;
        device->Release();
        return info;
    }

    return {};
}

bool AudioMonitor::DeviceExists(const std::string &device_id) const
{
    if (!enumerator_ || device_id.empty())
    {
        return false;
    }

    // Convert UTF-8 string to wide character
    std::wstring wide_id = Utf8ToWide(device_id);

    IMMDevice *device = nullptr;
    HRESULT hr = enumerator_->GetDevice(wide_id.c_str(), &device);
    if (SUCCEEDED(hr) && device)
    {
        device->Release();
        return true;
    }

    return false;
}

#elif LINUX_OS_ENVIRONMENT
#include <alsa/asoundlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_set>

static std::array<std::string, 12> blacklist = {"null",
                                                "pulse",
                                                "default",
                                                "Rate Converter Plugin",
                                                "JACK Audio Connection",
                                                "Open Sound System",
                                                "Plugin",
                                                "jack",
                                                "oss",
                                                "dsnoop",
                                                "dmix",
                                                "dsp"};

static bool is_blacklisted(const char *name)
{
    if (name == nullptr)
    {
        return true;
    }

    for (const auto &blacklisted : blacklist)
    {
        if (strncmp(name, blacklisted.c_str(), blacklisted.size()) == 0 || strstr(name, blacklisted.c_str()) != nullptr)
        {
            return true;
        }
    }
    return false;
}

// Helper function: Convert ALSA device name to human-readable name
static std::string get_device_description(const char *device_name)
{
    // Validate input
    if (!device_name || strlen(device_name) == 0)
    {
        return "Unknown Device";
    }

    // If not in hw:X or plughw:X format, return the original name
    if (strncmp(device_name, "hw:", 3) != 0 && strncmp(device_name, "plughw:", 7) != 0)
    {
        return device_name;
    }

    // Extract card number
    int card_num = -1;
    if (sscanf(device_name, "hw:%d", &card_num) != 1 && sscanf(device_name, "plughw:%d", &card_num) != 1)
    {
        return device_name;
    }

    // Build card control name
    char card_name[32];
    snprintf(card_name, sizeof(card_name), "hw:%d", card_num);

    snd_ctl_t *handle;
    std::string result = device_name;

    // Try to open the control interface
    int err = snd_ctl_open(&handle, card_name, SND_CTL_READONLY);
    if (err >= 0)
    {
        snd_ctl_card_info_t *info;
        snd_ctl_card_info_alloca(&info);

        if (snd_ctl_card_info(handle, info) >= 0)
        {
            const char *card_name = snd_ctl_card_info_get_name(info);
            const char *card_id = snd_ctl_card_info_get_id(info);
            const char *card_longname = snd_ctl_card_info_get_longname(info);

            // Prefer detailed name if available
            if (card_longname && strlen(card_longname) > 0)
            {
                result = card_longname;
            }
            // Next use card name
            else if (card_name && strlen(card_name) > 0)
            {
                result = card_name;
            }
            // Finally use card ID
            else if (card_id && strlen(card_id) > 0)
            {
                result = card_id;
            }
        }

        snd_ctl_close(handle);
    }

    return result;
}

// Helper function: Get ALSA device type
static AudioDeviceType get_device_type(const char *device_name)
{
    if (!device_name || strlen(device_name) == 0 || is_blacklisted(device_name))
    {
        return AudioDeviceType::All;
    }

    bool supports_playback = false;
    bool supports_capture = false;

    // Try to open the device in playback mode
    snd_pcm_t *pcm;
    int err = snd_pcm_open(&pcm, device_name, SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);
    if (err >= 0)
    {
        supports_playback = true;
        snd_pcm_close(pcm);
    }
    else if (err != -EBUSY) // If device is busy, it might still support playback
    {
        // Check if it's a permission issue
        if (err == -EACCES)
        {
            // Might be a permission issue, assume it supports playback
            supports_playback = true;
        }
    }

    // Try to open the device in capture mode
    err = snd_pcm_open(&pcm, device_name, SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
    if (err >= 0)
    {
        supports_capture = true;
        snd_pcm_close(pcm);
    }
    else if (err != -EBUSY) // If device is busy, it might still support capture
    {
        // Check if it's a permission issue
        if (err == -EACCES)
        {
            // Might be a permission issue, assume it supports capture
            supports_capture = true;
        }
    }

    // Return device type based on supported modes
    if (supports_playback && supports_capture)
    {
        return AudioDeviceType::All;
    }
    else if (supports_playback)
    {
        return AudioDeviceType::Playback;
    }
    else if (supports_capture)
    {
        return AudioDeviceType::Capture;
    }

    // Unable to determine type, default to playback
    return AudioDeviceType::Playback;
}

// Helper function: Check if the device is the default device
static bool is_default_device(const std::string &device_id, AudioDeviceType type)
{
    // Try to get the default device from asoundrc or system configuration
    // For simple detection, we can use the "default" alias

    snd_pcm_t *pcm = nullptr;
    snd_pcm_stream_t stream_type =
        (type == AudioDeviceType::Capture) ? SND_PCM_STREAM_CAPTURE : SND_PCM_STREAM_PLAYBACK;

    // First open the default device
    int err = snd_pcm_open(&pcm, "default", stream_type, SND_PCM_NONBLOCK);
    if (err < 0)
    {
        return false; // Cannot open default device
    }

    // Get default device information
    snd_pcm_info_t *info;
    snd_pcm_info_alloca(&info);
    err = snd_pcm_info(pcm, info);

    bool is_default = false;
    if (err >= 0)
    {
        // Get card number and device number
        int card = snd_pcm_info_get_card(info);
        int device = snd_pcm_info_get_device(info);

        char default_hw_id[32];
        snprintf(default_hw_id, sizeof(default_hw_id), "hw:%d,%d", card, device);

        // Also check plughw format
        char default_plughw_id[32];
        snprintf(default_plughw_id, sizeof(default_plughw_id), "plughw:%d,%d", card, device);

        // Check if device ID matches
        is_default = (device_id == default_hw_id || device_id == default_plughw_id);
    }

    snd_pcm_close(pcm);
    return is_default;
}

// UdevNotificationHandler implementation
UdevNotificationHandler::UdevNotificationHandler(
    std::function<void(AudioDeviceEvent, const AudioDeviceInfo &)> callback)
    : callback_(callback), udev_(nullptr), monitor_(nullptr), stream_descriptor_(nullptr), running_(false)
{
    // Initialize udev
    udev_ = udev_new();
}

UdevNotificationHandler::~UdevNotificationHandler()
{
    Stop();

    if (udev_)
    {
        udev_unref(udev_);
    }
}

bool UdevNotificationHandler::Start(asio::io_context &io_context)
{
    if (!udev_ || running_)
    {
        return false;
    }

    // Create monitor
    monitor_ = udev_monitor_new_from_netlink(udev_, "udev");
    if (!monitor_)
    {
        return false;
    }

    // Set filter to only receive sound card device events
    udev_monitor_filter_add_match_subsystem_devtype(monitor_, "sound", nullptr);
    udev_monitor_enable_receiving(monitor_);

    int fd = udev_monitor_get_fd(monitor_);

    // Create ASIO descriptor
    stream_descriptor_ = new asio::posix::stream_descriptor(io_context, fd);

    // Start asynchronous read
    running_ = true;
    StartMonitoring();
    return true;
}

void UdevNotificationHandler::Stop()
{
    running_ = false;

    if (monitor_)
    {
        udev_monitor_unref(monitor_);
        monitor_ = nullptr;
    }

    if (stream_descriptor_)
    {
        stream_descriptor_->cancel();

        delete stream_descriptor_;
        stream_descriptor_ = nullptr;
    }
}

void UdevNotificationHandler::StartMonitoring()
{
    if (running_ && stream_descriptor_)
    {
        stream_descriptor_->async_wait(asio::posix::stream_descriptor::wait_read, [this](const asio::error_code &ec) {
            if (!ec && running_)
            {
                HandleUdevEvent();
            }
        });
    }
}

void UdevNotificationHandler::HandleUdevEvent()
{
    if (!running_ || !monitor_)
    {
        return;
    }

    // Receive event
    struct udev_device *dev = udev_monitor_receive_device(monitor_);
    if (!dev)
    {
        StartMonitoring();
        return;
    }

    std::unique_ptr<struct udev_device, decltype(&udev_device_unref)> dev_ptr(dev, &udev_device_unref);

    if (IsAudioDevice(dev))
    {
        const char *action = udev_device_get_action(dev);
        if (action)
        {
            AudioDeviceEvent event;
            if (strcmp(action, "add") == 0)
            {
                event = AudioDeviceEvent::Added;
            }
            else if (strcmp(action, "remove") == 0)
            {
                event = AudioDeviceEvent::Removed;
            }
            else if (strcmp(action, "change") == 0)
            {
                event = AudioDeviceEvent::StateChanged;
            }
            else
            {
                StartMonitoring();
                return;
            }

            AudioDeviceInfo device_info = GetDeviceInfo(dev, event);

            if (callback_ && !device_info.id.empty())
            {
                callback_(event, device_info);
            }
        }
    }

    StartMonitoring();
}

bool UdevNotificationHandler::IsAudioDevice(struct udev_device *dev)
{
    // Check if the device belongs to the sound subsystem
    const char *subsystem = udev_device_get_subsystem(dev);
    if (!subsystem || strcmp(subsystem, "sound") != 0)
    {
        return false;
    }

    // Check if the device type is a sound card or audio device
    const char *sysname = udev_device_get_sysname(dev);
    if (!sysname)
    {
        return false;
    }

    // Sound card devices usually start with "card"
    if (strncmp(sysname, "card", 4) == 0)
    {
        return true;
    }

    return false;
}

AudioDeviceInfo UdevNotificationHandler::GetDeviceInfo(struct udev_device *dev, AudioDeviceEvent event)
{
    AudioDeviceInfo info;

    if (!dev)
    {
        return info;
    }

    const char *dev_path = udev_device_get_devpath(dev);
    if (dev_path)
    {
        info.id = dev_path;
    }

    const char *sysname = udev_device_get_sysname(dev);
    if (!sysname)
    {
        return info;
    }

    usleep(600000); // 60ms delay to ensure device is ready

    int card_num = -1;
    if (sscanf(sysname, "card%d", &card_num) == 1)
    {
        char card_id[32];
        snprintf(card_id, sizeof(card_id), "hw:%d,0", card_num);
        info.id = card_id;

        info.name = get_device_description(card_id);
        if (info.name.empty())
        {
            info.name = card_id;
        }

        if (event == AudioDeviceEvent::Removed)
        {
            info.is_active = false;
            info.type = AudioDeviceType::All;
            info.is_default = false;
        }
        else
        {
            snd_pcm_t *pcm_play = nullptr;
            bool can_play = (snd_pcm_open(&pcm_play, card_id, SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK) >= 0);
            if (can_play && pcm_play)
            {
                snd_pcm_close(pcm_play);
            }

            snd_pcm_t *pcm_cap = nullptr;
            bool can_capture = (snd_pcm_open(&pcm_cap, card_id, SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK) >= 0);
            if (can_capture && pcm_cap)
            {
                snd_pcm_close(pcm_cap);
            }

            if (can_play && can_capture)
                info.type = AudioDeviceType::All;
            else if (can_play)
                info.type = AudioDeviceType::Playback;
            else if (can_capture)
                info.type = AudioDeviceType::Capture;
            else
                info.type = AudioDeviceType::All;

            info.is_active = can_play || can_capture;

            info.is_default = false;
        }
    }
    else
    {
        info.name = sysname;
        info.type = AudioDeviceType::All;
        info.is_active = false;
        info.is_default = false;
    }

    return info;
}

#endif

// Cross-platform AudioDeviceMonitor implementation

AudioMonitor::AudioMonitor(asio::io_context &io_context)
    : io_context_(io_context)
#if WINDOWS_OS_ENVIRONMENT
      ,
      enumerator_(nullptr), notification_client_(nullptr)
#elif LINUX_OS_ENVIRONMENT
      ,
      udev_handler_(nullptr)
#endif
{
#if WINDOWS_OS_ENVIRONMENT
    HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                                  reinterpret_cast<void **>(&enumerator_));

    if (SUCCEEDED(hr) && enumerator_)
    {
        // Create notification client
        notification_client_ = new DeviceNotificationClient(
            [this](AudioDeviceEvent event, const AudioDeviceInfo &info) { HandleDeviceChange(event, info); });
    }
#elif LINUX_OS_ENVIRONMENT
    // Initialize udev handler
    udev_handler_ = new UdevNotificationHandler(
        [this](AudioDeviceEvent event, const AudioDeviceInfo &device_info) { HandleDeviceChange(event, device_info); });

    if (udev_handler_)
    {
        udev_handler_->Start(io_context_);
    }
#endif
}

AudioMonitor::~AudioMonitor()
{
#if WINDOWS_OS_ENVIRONMENT
    if (notification_client_)
    {
        notification_client_->Release();
        notification_client_ = nullptr;
    }

    if (enumerator_)
    {
        enumerator_->Release();
        enumerator_ = nullptr;
    }
#elif LINUX_OS_ENVIRONMENT
    if (udev_handler_)
    {
        udev_handler_->Stop();
        delete udev_handler_;
    }
#endif
}

bool AudioMonitor::RegisterCallback(DeviceChangeCallback callback)
{
    if (!callback)
        return false;

    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(callback);
    return true;
}

bool AudioMonitor::UnregisterCallback()
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_)
    {
        callback_ = nullptr;
        return true;
    }
    return false;
}

void AudioMonitor::HandleDeviceChange(AudioDeviceEvent event, const AudioDeviceInfo &device_info)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_)
    {
        callback_(event, device_info);
    }
}

#if WINDOWS_OS_ENVIRONMENT
std::vector<AudioDeviceInfo> AudioMonitor::EnumerateDevices(AudioDeviceType type)
{
    std::vector<AudioDeviceInfo> devices;

    if (!enumerator_)
    {
        return devices;
    }

    // Determine which type of device to enumerate
    EDataFlow flow;
    switch (type)
    {
    case AudioDeviceType::Playback:
        flow = eRender;
        break;
    case AudioDeviceType::Capture:
        flow = eCapture;
        break;
    default:
        // For All type, we need to enumerate both playback and capture devices separately
        {
            auto playback_devices = EnumerateDevices(AudioDeviceType::Playback);
            auto capture_devices = EnumerateDevices(AudioDeviceType::Capture);
            devices.insert(devices.end(), playback_devices.begin(), playback_devices.end());
            devices.insert(devices.end(), capture_devices.begin(), capture_devices.end());
            return devices;
        }
    }

    IMMDeviceCollection *device_collection = nullptr;
    HRESULT hr = enumerator_->EnumAudioEndpoints(
        flow, DEVICE_STATE_ACTIVE | DEVICE_STATE_DISABLED | DEVICE_STATE_NOTPRESENT | DEVICE_STATE_UNPLUGGED,
        &device_collection);
    if (FAILED(hr) || !device_collection)
    {
        return devices;
    }

    UINT count;
    hr = device_collection->GetCount(&count);
    if (FAILED(hr))
    {
        device_collection->Release();
        return devices;
    }

    for (UINT i = 0; i < count; ++i)
    {
        IMMDevice *device = nullptr;
        hr = device_collection->Item(i, &device);
        if (SUCCEEDED(hr) && device)
        {
            AudioDeviceInfo info = GetDeviceInfo(device, type);
            devices.push_back(info);
            device->Release();
        }
    }

    device_collection->Release();
    return devices;
}

AudioDeviceInfo AudioMonitor::GetDeviceInfo(IMMDevice *device, AudioDeviceType type) const
{
    AudioDeviceInfo info;
    IPropertyStore *props = nullptr;
    HRESULT hr;

    // Get device ID
    LPWSTR device_id = nullptr;
    hr = device->GetId(&device_id);
    if (SUCCEEDED(hr) && device_id)
    {
        info.id = WideToUtf8(std::wstring(device_id));
        CoTaskMemFree(device_id);
    }

    // Determine device type
    IMMEndpoint *endpoint = nullptr;
    hr = device->QueryInterface(__uuidof(IMMEndpoint), reinterpret_cast<void **>(&endpoint));
    if (SUCCEEDED(hr) && endpoint)
    {
        EDataFlow flow;
        hr = endpoint->GetDataFlow(&flow);
        if (SUCCEEDED(hr))
        {
            if (flow == eRender)
            {
                info.type = AudioDeviceType::Playback;
            }
            else if (flow == eCapture)
            {
                info.type = AudioDeviceType::Capture;
            }
        }
        endpoint->Release();
    }

    // If a type is specified and does not match, return
    if (type != AudioDeviceType::All && type != info.type)
    {
        return info;
    }

    // Get device properties
    hr = device->OpenPropertyStore(STGM_READ, &props);
    if (SUCCEEDED(hr) && props)
    {
        PROPVARIANT var;
        PropVariantInit(&var);

        // Get friendly name
        hr = props->GetValue(PKEY_Device_FriendlyName, &var);
        if (SUCCEEDED(hr) && var.vt == VT_LPWSTR)
        {
            info.name = WideToUtf8(var.pwszVal);
        }
        PropVariantClear(&var);

        props->Release();
    }

    // Check device status
    DWORD state;
    hr = device->GetState(&state);
    if (SUCCEEDED(hr))
    {
        info.is_active = (state == DEVICE_STATE_ACTIVE);
    }

    // Check if it is the default device
    IMMDevice *default_device = nullptr;
    EDataFlow flow = (info.type == AudioDeviceType::Playback) ? eRender : eCapture;
    hr = enumerator_->GetDefaultAudioEndpoint(flow, eConsole, &default_device);
    if (SUCCEEDED(hr) && default_device)
    {
        LPWSTR default_id = nullptr;
        hr = default_device->GetId(&default_id);
        if (SUCCEEDED(hr) && default_id)
        {
            info.is_default = (info.id == WideToUtf8(std::wstring(default_id)));
            CoTaskMemFree(default_id);
        }
        default_device->Release();
    }

    return info;
}

#elif LINUX_OS_ENVIRONMENT
std::vector<AudioDeviceInfo> AudioMonitor::EnumerateDevices(AudioDeviceType type)
{
    std::vector<AudioDeviceInfo> devices;

    void **hints;
    int err = snd_device_name_hint(-1, "pcm", &hints);
    if (err < 0)
    {
        return devices;
    }

    // Track already added device IDs to avoid duplicates
    std::unordered_set<std::string> added_device_ids;

    void **n = hints;
    while (*n != nullptr)
    {
        char *name = snd_device_name_get_hint(*n, "NAME");

        if (name != nullptr && !is_blacklisted(name))
        {
            char *desc = snd_device_name_get_hint(*n, "DESC");
            char *ioid = snd_device_name_get_hint(*n, "IOID");
            if (strncmp(name, "hw:", 3) == 0)
            {
                // Avoid adding the same device multiple times
                if (added_device_ids.find(name) != added_device_ids.end())
                {
                    goto cleanup;
                }

                AudioDeviceInfo info;
                info.id = name;
                added_device_ids.insert(info.id);

                // Prefer device description as name
                if (desc != nullptr && strlen(desc) > 0)
                {
                    info.name = desc;

                    // Remove newlines and excess spaces
                    info.name.erase(std::remove(info.name.begin(), info.name.end(), '\n'), info.name.end());
                    while (info.name.find("  ") != std::string::npos)
                    {
                        info.name.replace(info.name.find("  "), 2, " ");
                    }
                }
                else
                {
                    // Try to get a more friendly name
                    info.name = get_device_description(name);
                }

                // Determine device type
                if (ioid != nullptr)
                {
                    std::string ioid_str(ioid);
                    if (ioid_str == "Output")
                    {
                        info.type = AudioDeviceType::Playback;
                    }
                    else if (ioid_str == "Input")
                    {
                        info.type = AudioDeviceType::Capture;
                    }
                    else
                    {
                        info.type = AudioDeviceType::All;
                    }
                }
                else
                {
                    info.type = get_device_type(name);
                }

                // Filter devices by requested type
                bool add_device = (type == AudioDeviceType::All) ||
                                  (type == AudioDeviceType::Playback &&
                                   (info.type == AudioDeviceType::Playback || info.type == AudioDeviceType::All)) ||
                                  (type == AudioDeviceType::Capture &&
                                   (info.type == AudioDeviceType::Capture || info.type == AudioDeviceType::All));

                if (add_device)
                {
                    // Check device active status
                    snd_pcm_t *pcm = nullptr;
                    snd_pcm_stream_t stream_type =
                        (info.type == AudioDeviceType::Capture) ? SND_PCM_STREAM_CAPTURE : SND_PCM_STREAM_PLAYBACK;

                    info.is_active = (snd_pcm_open(&pcm, name, stream_type, SND_PCM_NONBLOCK) >= 0);
                    if (pcm)
                    {
                        snd_pcm_close(pcm);
                    }

                    // Check if it's the default device
                    info.is_default = is_default_device(info.id, info.type);

                    devices.push_back(info);
                }
            }

            if (desc != nullptr)
            {
                free(desc);
            }
            if (ioid != nullptr)
            {
                free(ioid);
            }
        }

    cleanup:
        if (name != nullptr)
        {
            free(name);
        }
        n++;
    }

    snd_device_name_free_hint(hints);

    // Sort device list with default devices first
    std::sort(devices.begin(), devices.end(), [](const AudioDeviceInfo &a, const AudioDeviceInfo &b) {
        if (a.is_default != b.is_default)
        {
            return a.is_default;
        }
        return a.name < b.name;
    });

    return devices;
}

AudioDeviceInfo AudioMonitor::GetDefaultDevice(AudioDeviceType type)
{
    // First try to open the default device directly to get information
    snd_pcm_t *pcm = nullptr;
    snd_pcm_stream_t stream_type =
        (type == AudioDeviceType::Capture) ? SND_PCM_STREAM_CAPTURE : SND_PCM_STREAM_PLAYBACK;

    int err = snd_pcm_open(&pcm, "default", stream_type, SND_PCM_NONBLOCK);
    if (err >= 0 && pcm)
    {
        // Get default device information
        snd_pcm_info_t *info;
        snd_pcm_info_alloca(&info);

        if (snd_pcm_info(pcm, info) >= 0)
        {
            int card = snd_pcm_info_get_card(info);
            int device = snd_pcm_info_get_device(info);

            char hw_id[32];
            snprintf(hw_id, sizeof(hw_id), "hw:%d,%d", card, device);

            snd_pcm_close(pcm);

            // Get complete information for this device
            std::vector<AudioDeviceInfo> devices = EnumerateDevices(type);
            for (const auto &dev : devices)
            {
                if (dev.id == hw_id || dev.id.find(hw_id) != std::string::npos)
                {
                    return dev;
                }
            }
        }

        snd_pcm_close(pcm);
    }

    // Alternative method: look for a device marked as default in the enumerated devices
    std::vector<AudioDeviceInfo> devices = EnumerateDevices(type);
    for (const auto &device : devices)
    {
        if (device.is_default)
        {
            return device;
        }
    }

    // If no default device is found, return the first device or an empty device
    if (!devices.empty())
    {
        return devices[0];
    }

    // Return an empty device
    return AudioDeviceInfo();
}

bool AudioMonitor::DeviceExists(const std::string &device_id)
{
    if (device_id.empty())
    {
        return false;
    }

    // First try to open the device directly
    snd_pcm_t *pcm = nullptr;
    bool exists = false;

    // Try to open in playback mode
    int err = snd_pcm_open(&pcm, device_id.c_str(), SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);
    if (err >= 0)
    {
        exists = true;
        snd_pcm_close(pcm);
        return exists;
    }

    // Try to open in capture mode
    err = snd_pcm_open(&pcm, device_id.c_str(), SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
    if (err >= 0)
    {
        exists = true;
        snd_pcm_close(pcm);
        return exists;
    }

    // If direct open fails, enumerate devices
    if (!exists)
    {
        std::vector<AudioDeviceInfo> devices = EnumerateDevices();
        exists = std::any_of(devices.begin(), devices.end(),
                             [&device_id](const AudioDeviceInfo &device) { return device.id == device_id; });
    }

    return exists;
}

#endif