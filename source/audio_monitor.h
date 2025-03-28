#ifndef AUDIO_DEVICE_MONITOR_HEADER
#define AUDIO_DEVICE_MONITOR_HEADER

#include "asio.hpp"
#include "audio_interface.h"
#include <functional>
#include <mutex>
#include <vector>

enum class AudioDeviceEvent
{
    Added,         // Device added
    Removed,       // Device removed
    StateChanged,  // Device state changed
    DefaultChanged // Default device changed
};

enum class AudioDeviceType
{
    Playback, // Playback device
    Capture,  // Capture device
    All       // All devices
};

struct AudioDeviceInfo
{
    std::string id;         // Device ID
    std::string name;       // Device name
    AudioDeviceType type;   // Device type
    bool is_default{false}; // Is default device
    bool is_active{false};  // Is active
};

// Device change callback type - cross-platform definition
using DeviceChangeCallback = std::function<void(AudioDeviceEvent, const AudioDeviceInfo &)>;

#if WINDOWS_OS_ENVIRONMENT
#include <mmdeviceapi.h>

// Windows device notification callback interface implementation
class DeviceNotificationClient : public IMMNotificationClient
{
  public:
    explicit DeviceNotificationClient(std::function<void(AudioDeviceEvent, const AudioDeviceInfo &)> callback);
    ~DeviceNotificationClient();

    // IUnknown methods
    ULONG STDMETHODCALLTYPE AddRef() override;
    ULONG STDMETHODCALLTYPE Release() override;
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void **ppvObject) override;

    // IMMNotificationClient methods
    HRESULT STDMETHODCALLTYPE OnDeviceStateChanged(LPCWSTR pwstrDeviceId, DWORD dwNewState) override;
    HRESULT STDMETHODCALLTYPE OnDeviceAdded(LPCWSTR pwstrDeviceId) override;
    HRESULT STDMETHODCALLTYPE OnDeviceRemoved(LPCWSTR pwstrDeviceId) override;
    HRESULT STDMETHODCALLTYPE OnDefaultDeviceChanged(EDataFlow flow, ERole role, LPCWSTR pwstrDefaultDeviceId) override;
    HRESULT STDMETHODCALLTYPE OnPropertyValueChanged(LPCWSTR pwstrDeviceId, const PROPERTYKEY key) override;

  private:
    AudioDeviceInfo GetDeviceInfo(LPCWSTR pwstrDeviceId, AudioDeviceType type = AudioDeviceType::All);

    std::function<void(AudioDeviceEvent, const AudioDeviceInfo &)> callback_;
    LONG ref_count_;
    IMMDeviceEnumerator *enumerator_;
};

#elif LINUX_OS_ENVIRONMENT
// ALSA device monitoring implementation
#include <libudev.h>

// Udev device notification handler class
class UdevNotificationHandler
{
  public:
    explicit UdevNotificationHandler(std::function<void(AudioDeviceEvent, const AudioDeviceInfo &)> callback);
    ~UdevNotificationHandler();

    // Start listening
    bool Start(asio::io_context &io_context);
    void Stop();

  private:
    void StartMonitoring();
    void HandleUdevEvent();
    bool IsAudioDevice(struct udev_device *dev);
    AudioDeviceInfo GetDeviceInfo(struct udev_device *dev, AudioDeviceEvent event);

    std::function<void(AudioDeviceEvent, const AudioDeviceInfo &)> callback_;
    struct udev *udev_;
    struct udev_monitor *monitor_;
    asio::posix::stream_descriptor *stream_descriptor_;
    bool running_;
};

#endif

// Audio device monitor class - cross-platform definition
class AudioMonitor
{
  public:
    explicit AudioMonitor(asio::io_context &io_context);
    ~AudioMonitor();

    // Get list of all devices
    std::vector<AudioDeviceInfo> EnumerateDevices(AudioDeviceType type = AudioDeviceType::All);

    // Get default device
    AudioDeviceInfo GetDefaultDevice(AudioDeviceType type);

    // Check if device exists
    bool DeviceExists(const std::string &device_id);

    // Register device change callback
    bool RegisterCallback(DeviceChangeCallback callback);

    // Unregister device change callback
    bool UnregisterCallback();

#if WINDOWS_OS_ENVIRONMENT
    // String conversion utility methods
    static std::string WideToUtf8(const std::wstring &wstr);
    static std::wstring Utf8ToWide(const std::string &str);
#endif

  private:
    // Internal device change handler function
    void HandleDeviceChange(AudioDeviceEvent event, const AudioDeviceInfo &device_info);

#if WINDOWS_OS_ENVIRONMENT
    AudioDeviceInfo GetDeviceInfo(IMMDevice *device, AudioDeviceType type = AudioDeviceType::All);
#endif

  private:
    asio::io_context &io_context_;

#if WINDOWS_OS_ENVIRONMENT
    IMMDeviceEnumerator *enumerator_;
    DeviceNotificationClient *notification_client_;
#elif LINUX_OS_ENVIRONMENT
    UdevNotificationHandler *udev_handler_;
#endif

    std::mutex callback_mutex_;
    DeviceChangeCallback callback_;
};

#endif // AUDIO_DEVICE_MONITOR_HEADER