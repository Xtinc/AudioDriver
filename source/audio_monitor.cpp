#include "audio_monitor.h"
#include "audio_interface.h"

#if WINDOWS_OS_ENVIRONMENT
// must be included after mmdeviceapi.h
#include <functiondiscoverykeys_devpkey.h>
std::string AudioDeviceMonitor::WideToUtf8(const std::wstring &wstr)
{
    if (wstr.empty())
    {
        return std::string();
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

std::wstring AudioDeviceMonitor::Utf8ToWide(const std::string &str)
{
    if (str.empty())
    {
        return std::wstring();
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
    : callback_(callback), ref_count_(1), enumerator_(nullptr)
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
    AudioDeviceInfo device_info;
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
    device_info = GetDeviceInfo(pwstrDeviceId);
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
    device_info.id = AudioDeviceMonitor::WideToUtf8(std::wstring(pwstrDeviceId));

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

HRESULT STDMETHODCALLTYPE DeviceNotificationClient::OnPropertyValueChanged(LPCWSTR pwstrDeviceId, const PROPERTYKEY key)
{
    // We do not handle property changes
    return S_OK;
}

AudioDeviceInfo DeviceNotificationClient::GetDeviceInfo(LPCWSTR pwstrDeviceId, AudioDeviceType type)
{
    AudioDeviceInfo info;
    IMMDevice *device = nullptr;
    IPropertyStore *props = nullptr;
    HRESULT hr;

    // Convert wide character device ID to UTF-8 string
    info.id = AudioDeviceMonitor::WideToUtf8(std::wstring(pwstrDeviceId));

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
    hr = device->QueryInterface(__uuidof(IMMEndpoint), (void **)&endpoint);
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
            info.name = AudioDeviceMonitor::WideToUtf8(var.pwszVal);
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

AudioDeviceInfo AudioDeviceMonitor::GetDefaultDevice(AudioDeviceType type)
{
    if (!enumerator_)
    {
        return AudioDeviceInfo();
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

    return AudioDeviceInfo();
}

bool AudioDeviceMonitor::DeviceExists(const std::string &device_id)
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

static std::array<std::string, 12> blakclist = {"null",
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

    for (const auto &blacklisted : blakclist)
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
    snd_ctl_t *handle;
    std::string result = device_name;

    // Validate input
    if (!device_name || strlen(device_name) == 0)
    {
        return "Unknown Device";
    }

    // Try to open the control interface for the card
    int err = snd_ctl_open(&handle, device_name, SND_CTL_READONLY);
    if (err >= 0)
    {
        snd_ctl_card_info_t *info;
        snd_ctl_card_info_alloca(&info);

        if (snd_ctl_card_info(handle, info) >= 0)
        {
            const char *name = snd_ctl_card_info_get_name(info);
            if (name && strlen(name) > 0)
            {
                result = name;
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

    snd_pcm_t *pcm;
    if (snd_pcm_open(&pcm, device_name, SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK) >= 0)
    {
        supports_playback = true;
        snd_pcm_close(pcm);
    }

    if (snd_pcm_open(&pcm, device_name, SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK) >= 0)
    {
        supports_capture = true;
        snd_pcm_close(pcm);
    }

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

    return AudioDeviceType::Playback;
}

// Helper function: Check if the device is the default device
static bool is_default_device(const std::string &device_id, AudioDeviceType type)
{
    void **hints;
    bool is_default = false;

    if (device_id.empty() || snd_device_name_hint(-1, "pcm", &hints) < 0)
    {
        return false;
    }

    void **n = hints;
    while (*n != nullptr)
    {
        char *name = snd_device_name_get_hint(*n, "NAME");
        char *ioid = snd_device_name_get_hint(*n, "IOID");

        if (name != nullptr)
        {
            std::string name_str(name);

            // Only compare the ID part (usually hw:X,Y or plughw:X,Y)
            if (name_str == device_id)
            {
                // Check if IOID matches the type
                bool is_valid_ioid = true;
                if (ioid != nullptr)
                {
                    std::string ioid_str(ioid);
                    is_valid_ioid = (type == AudioDeviceType::Playback && ioid_str == "Output") ||
                                    (type == AudioDeviceType::Capture && ioid_str == "Input") ||
                                    (type == AudioDeviceType::All);
                }

                // Default device is usually the first matching device
                if (is_valid_ioid)
                {
                    is_default = true;
                    free(name);
                    if (ioid)
                        free(ioid);
                    break;
                }
            }

            free(name);
        }

        if (ioid != nullptr)
            free(ioid);

        n++;
    }

    snd_device_name_free_hint(hints);
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
        udev_unref(udev_);
}

bool UdevNotificationHandler::Start(asio::io_context &io_context)
{
    if (!udev_ || running_)
        return false;

    // Create monitor
    monitor_ = udev_monitor_new_from_netlink(udev_, "udev");
    if (!monitor_)
        return false;

    // Set filter to only receive sound card device events
    udev_monitor_filter_add_match_subsystem_devtype(monitor_, "sound", nullptr);
    udev_monitor_enable_receiving(monitor_);

    int fd = udev_monitor_get_fd(monitor_);

    // Create ASIO descriptor
    stream_descriptor_ = new asio::posix::stream_descriptor(io_context, fd);

    // Start asynchronous read
    running_ = true;
    stream_descriptor_->async_wait(asio::posix::stream_descriptor::wait_read, [this](const asio::error_code &ec) {
        if (!ec && running_)
        {
            HandleUdevEvent();
        }
    });

    return true;
}

void UdevNotificationHandler::Stop()
{
    running_ = false;

    if (stream_descriptor_)
    {
        stream_descriptor_->cancel();
        delete stream_descriptor_;
        stream_descriptor_ = nullptr;
    }

    if (monitor_)
    {
        udev_monitor_unref(monitor_);
        monitor_ = nullptr;
    }
}

void UdevNotificationHandler::HandleUdevEvent()
{
    if (!running_ || !monitor_)
        return;

    // Receive event
    struct udev_device *dev = udev_monitor_receive_device(monitor_);
    if (dev)
    {
        // Check if it is an audio device
        if (IsAudioDevice(dev))
        {
            const char *action = udev_device_get_action(dev);
            if (!action)
            {
                udev_device_unref(dev);
                goto continue_monitoring;
            }

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
                udev_device_unref(dev);
                goto continue_monitoring;
            }

            // Get device information
            AudioDeviceInfo device_info = GetDeviceInfo(dev);

            // Call callback
            if (callback_ && !device_info.id.empty())
            {
                callback_(event, device_info);
            }
        }

        udev_device_unref(dev);
    }

continue_monitoring:
    // Continue listening
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

bool UdevNotificationHandler::IsAudioDevice(struct udev_device *dev)
{
    // Check if the device belongs to the sound subsystem
    const char *subsystem = udev_device_get_subsystem(dev);
    if (!subsystem || strcmp(subsystem, "sound") != 0)
        return false;

    // Check if the device type is a sound card or audio device
    const char *sysname = udev_device_get_sysname(dev);
    if (!sysname)
        return false;

    // Sound card devices usually start with "card"
    if (strncmp(sysname, "card", 4) == 0)
        return true;

    return false;
}

AudioDeviceInfo UdevNotificationHandler::GetDeviceInfo(struct udev_device *dev)
{
    AudioDeviceInfo info;

    if (!dev)
    {
        return info;
    }

    // Get device ID
    const char *dev_path = udev_device_get_devpath(dev);
    if (dev_path)
    {
        info.id = dev_path;
    }

    // Get device name
    const char *sysname = udev_device_get_sysname(dev);
    if (sysname)
    {
        // Extract card number from sysname
        int card_num = -1;
        if (sscanf(sysname, "card%d", &card_num) == 1)
        {
            char card_id[32];
            snprintf(card_id, sizeof(card_id), "hw:%d", card_num);

            // Get more descriptive name if possible
            info.name = get_device_description(card_id);

            // If we couldn't get a proper name, use the card ID
            if (info.name.empty())
            {
                info.name = card_id;
            }

            // Assume the device is available but verify
            snd_pcm_t *pcm = nullptr;
            info.is_active = (snd_pcm_open(&pcm, card_id, SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK) >= 0);
            if (pcm)
                snd_pcm_close(pcm);

            // Determine device type
            info.type = get_device_type(card_id);

            // Check if it is the default device
            info.is_default = is_default_device(card_id, info.type);
        }
        else
        {
            info.name = sysname;
            info.type = AudioDeviceType::All;
            info.is_active = false;
            info.is_default = false;
        }
    }

    return info;
}

#endif

// Cross-platform AudioDeviceMonitor implementation
std::shared_ptr<AudioDeviceMonitor> AudioDeviceMonitor::Create(asio::io_context &io_context)
{
    auto monitor = std::shared_ptr<AudioDeviceMonitor>(new AudioDeviceMonitor(io_context));
    monitor->Start();
    return monitor;
}

AudioDeviceMonitor::AudioDeviceMonitor(asio::io_context &io_context)
    : io_context_(io_context)
#if WINDOWS_OS_ENVIRONMENT
      ,
      enumerator_(nullptr), notification_client_(nullptr)
#elif LINUX_OS_ENVIRONMENT
      ,
      udev_handler_(nullptr), polling_timer_(std::make_unique<asio::steady_timer>(io_context))
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

    // Store initial device list
    last_device_list_ = EnumerateDevices();

#endif
}

AudioDeviceMonitor::~AudioDeviceMonitor()
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
    if (polling_timer_)
    {
        polling_timer_->cancel();
    }

    if (udev_handler_)
    {
        udev_handler_->Stop();
        delete udev_handler_;
    }
#endif
}

bool AudioDeviceMonitor::RegisterCallback(void *owner, DeviceChangeCallback callback)
{
    if (!owner || !callback)
        return false;

    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    callbacks_[owner] = std::move(callback);
    return true;
}

bool AudioDeviceMonitor::UnregisterCallback(void *owner)
{
    if (!owner)
        return false;

    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    return callbacks_.erase(owner) > 0;
}

void AudioDeviceMonitor::HandleDeviceChange(AudioDeviceEvent event, const AudioDeviceInfo &device_info)
{
    // Use strand to ensure callbacks are executed sequentially on the IO thread
    asio::post(io_context_, [this, event, device_info]() {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        for (const auto &pair : callbacks_)
        {
            pair.second(event, device_info);
        }
    });
}

#if WINDOWS_OS_ENVIRONMENT
std::vector<AudioDeviceInfo> AudioDeviceMonitor::EnumerateDevices(AudioDeviceType type)
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

AudioDeviceInfo AudioDeviceMonitor::GetDeviceInfo(IMMDevice *device, AudioDeviceType type)
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
    hr = device->QueryInterface(__uuidof(IMMEndpoint), (void **)&endpoint);
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

void AudioDeviceMonitor::Start()
{
}

#elif LINUX_OS_ENVIRONMENT
std::vector<AudioDeviceInfo> AudioDeviceMonitor::EnumerateDevices(AudioDeviceType type)
{
    std::vector<AudioDeviceInfo> devices;

    void **hints;
    int err = snd_device_name_hint(-1, "pcm", &hints);
    if (err < 0)
    {
        return devices;
    }

    void **n = hints;
    while (*n != nullptr)
    {
        char *name = snd_device_name_get_hint(*n, "NAME");
        char *desc = snd_device_name_get_hint(*n, "DESC");
        char *ioid = snd_device_name_get_hint(*n, "IOID");

        if (name != nullptr && !is_blacklisted(name))
        {
            if (strncmp(name, "hw:", 3) == 0 || strncmp(name, "plughw:", 7) == 0)
            {
                AudioDeviceInfo info;
                info.id = name;

                if (desc != nullptr && strlen(desc) > 0)
                {
                    info.name = desc;
                }
                else
                {
                    info.name = name;
                }

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

                bool add_device = (type == AudioDeviceType::All) ||
                                  (type == AudioDeviceType::Playback &&
                                   (info.type == AudioDeviceType::Playback || info.type == AudioDeviceType::All)) ||
                                  (type == AudioDeviceType::Capture &&
                                   (info.type == AudioDeviceType::Capture || info.type == AudioDeviceType::All));

                if (add_device)
                {
                    snd_pcm_t *pcm = nullptr;
                    snd_pcm_stream_t stream_type =
                        (info.type == AudioDeviceType::Capture) ? SND_PCM_STREAM_CAPTURE : SND_PCM_STREAM_PLAYBACK;

                    info.is_active = (snd_pcm_open(&pcm, name, stream_type, SND_PCM_NONBLOCK) >= 0);
                    if (pcm)
                    {
                        snd_pcm_close(pcm);
                    }

                    info.is_default = is_default_device(info.id, info.type);

                    // std::string readable_name = get_device_description(name);
                    // if (!readable_name.empty() && readable_name != name)
                    // {
                    //     info.name = readable_name;
                    // }

                    devices.push_back(info);
                }
            }
        }

        if (name != nullptr)
            free(name);
        if (desc != nullptr)
            free(desc);
        if (ioid != nullptr)
            free(ioid);

        n++;
    }

    snd_device_name_free_hint(hints);
    return devices;
}

AudioDeviceInfo AudioDeviceMonitor::GetDefaultDevice(AudioDeviceType type)
{
    // Get the first default device
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

bool AudioDeviceMonitor::DeviceExists(const std::string &device_id)
{
    std::vector<AudioDeviceInfo> devices = EnumerateDevices();

    return std::any_of(devices.begin(), devices.end(),
                       [&device_id](const AudioDeviceInfo &device) { return device.id == device_id; });
}

void AudioDeviceMonitor::StartPollingTimer()
{
    polling_timer_->expires_after(std::chrono::seconds(5));
    polling_timer_->async_wait([this](const asio::error_code &ec) {
        if (!ec)
        {
            PollDeviceChanges();

            if (polling_timer_)
            {
                StartPollingTimer();
            }
        }
    });
}

void AudioDeviceMonitor::PollDeviceChanges()
{
    // Get the current device list
    std::vector<AudioDeviceInfo> current_devices = EnumerateDevices();

    // Find added devices
    for (const auto &current_device : current_devices)
    {
        auto it = std::find_if(
            last_device_list_.begin(), last_device_list_.end(),
            [&current_device](const AudioDeviceInfo &old_device) { return old_device.id == current_device.id; });

        if (it == last_device_list_.end())
        {
            // Found a new device
            HandleDeviceChange(AudioDeviceEvent::Added, current_device);
        }
        else if (it->is_active != current_device.is_active)
        {
            // Device state changed
            HandleDeviceChange(AudioDeviceEvent::StateChanged, current_device);
        }
        else if (it->is_default != current_device.is_default && current_device.is_default)
        {
            // Default device changed
            HandleDeviceChange(AudioDeviceEvent::DefaultChanged, current_device);
        }
    }

    // Find removed devices
    for (const auto &old_device : last_device_list_)
    {
        auto it = std::find_if(
            current_devices.begin(), current_devices.end(),
            [&old_device](const AudioDeviceInfo &current_device) { return current_device.id == old_device.id; });

        if (it == current_devices.end())
        {
            // Device was removed
            HandleDeviceChange(AudioDeviceEvent::Removed, old_device);
        }
    }

    // Update device list
    last_device_list_ = std::move(current_devices);
}

void AudioDeviceMonitor::Start()
{
    StartPollingTimer();
}
#endif