#include "audio_driver.h"
#include "audio_wavfile.h"
#include <codecvt>

#if LINUX_OS_ENVIRONMENT
#include <alsa/asoundlib.h>
#include <climits>

static std::string normailzed_device_name(const AudioDeviceName &name)
{
    if (name.first == "default" || name.first.empty())
    {
        return "default";
    }

    if (name.first.find("hw:") == 0 && name.first.find(",") != std::string::npos)
    {
        return name.first;
    }

    return std::string("hw:CARD=") + name.first + ",DEV=" + std::to_string(name.second);
}

void set_current_thread_scheduler_policy()
{
    auto thread = pthread_self();
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
    pthread_setschedparam(thread, SCHED_RR, &param);
    pthread_setname_np(thread, "audio_thd");
}

class AlsaDriver final : public AudioDevice
{
  public:
    AlsaDriver(const std::string &name, bool capture, unsigned int fs, unsigned int ps, unsigned int ch,
               bool enable_strict_recover);
    ~AlsaDriver() override;

    RetCode open() override;
    RetCode start() override;
    RetCode stop() override;
    RetCode write(const char *data, size_t len) override;
    RetCode read(char *data, size_t len) override;

  private:
    int xrun_recovery(int err);

    void write_loop();
    void read_loop();

  private:
    bool mmap_mode;
    snd_pcm_t *handle;
};

AlsaDriver::AlsaDriver(const std::string &name, bool capture, unsigned int fs, unsigned int ps, unsigned int ch,
                       bool enable_strict_recover)
    : AudioDevice(name, capture, fs, ps, ch, enable_strict_recover), mmap_mode(true), handle(nullptr)
{
}

AlsaDriver::~AlsaDriver()
{
    (void)stop();
    if (hstate != STREAM_CLOSED)
    {
        snd_pcm_close(handle);
    }
}

RetCode AlsaDriver::open()
{
    auto dev_ti = dev_ps * 1000 / dev_fs;
    auto result =
        snd_pcm_open(&handle, hw_name.c_str(), is_capture_dev ? SND_PCM_STREAM_CAPTURE : SND_PCM_STREAM_PLAYBACK, 0);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] open failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EOPEN;
    }
    hstate = STREAM_OPENED;

    snd_pcm_hw_params_t *hw_params = nullptr;
    snd_pcm_hw_params_alloca(&hw_params);
    result = snd_pcm_hw_params_any(handle, hw_params);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] hw_params_any failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EPARAM;
    }

    result = snd_pcm_hw_params_set_access(handle, hw_params, SND_PCM_ACCESS_MMAP_INTERLEAVED);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] set mmap access failed: %s", hw_name.c_str(), snd_strerror(result));
        mmap_mode = false;
    }

    result = snd_pcm_hw_params_set_format(handle, hw_params, SND_PCM_FORMAT_S16_LE);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] set format failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EPARAM;
    }

    result = snd_pcm_hw_params_set_rate_near(handle, hw_params, &dev_fs, 0);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] set rate failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EPARAM;
    }

    result = snd_pcm_hw_params_get_channels_max(hw_params, &max_ch);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] get max channel failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EPARAM;
    }

    result = snd_pcm_hw_params_get_channels_min(hw_params, &min_ch);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] get min channel failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EPARAM;
    }

    if (dev_ch < min_ch || dev_ch > max_ch)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] invalid channel count: %u, reset to minimal channels", hw_name.c_str(),
                          dev_ch);
        dev_ch = min_ch;
    }

    result = snd_pcm_hw_params_set_channels(handle, hw_params, dev_ch);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] set channel failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EPARAM;
    }

    int dir = 0;
    snd_pcm_uframes_t period_size = dev_fs * dev_ti / 1000;
    result = snd_pcm_hw_params_set_period_size_near(handle, hw_params, &period_size, &dir);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] set period size failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EPARAM;
    }
    dev_ps = period_size;

    unsigned int period_number = 2;
    result = snd_pcm_hw_params_set_periods_near(handle, hw_params, &period_number, &dir);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] set period number failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EPARAM;
    }

    result = snd_pcm_hw_params(handle, hw_params);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] set hw params failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EPARAM;
    }

    snd_pcm_sw_params_t *sw_params = NULL;
    snd_pcm_sw_params_alloca(&sw_params);
    snd_pcm_sw_params_current(handle, sw_params);
    snd_pcm_sw_params_set_start_threshold(handle, sw_params, period_size);
    snd_pcm_sw_params_set_stop_threshold(handle, sw_params, ULONG_MAX);
    snd_pcm_sw_params_set_silence_threshold(handle, sw_params, 0);

    snd_pcm_uframes_t val;
    snd_pcm_sw_params_get_boundary(sw_params, &val);
    snd_pcm_sw_params_set_silence_size(handle, sw_params, val);

    result = snd_pcm_sw_params(handle, sw_params);
    if (result < 0)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] set sw params failed: %s", hw_name.c_str(), snd_strerror(result));
        return RetCode::EPARAM;
    }

    io_buffer = std::make_unique<KFifo>(dev_ps * sizeof(PCM_TYPE), 3, dev_ch);
    AUDIO_INFO_PRINT("ALSA device [%s] opened. fs = %u, ps = %u, chan = %u, min_chan = %u, max_chan = %u",
                     hw_name.c_str(), dev_fs, dev_ps, dev_ch, min_ch, max_ch);

    timer_cnt = std::make_unique<TimerCounter>(hw_name, dev_ti);
    return RetCode::OK;
}

RetCode AlsaDriver::start()
{
    Mode expected = STREAM_OPENED;
    if (!hstate.compare_exchange_strong(expected, STREAM_RUNNING) &&
        !(expected = STREAM_STOPPED, hstate.compare_exchange_strong(expected, STREAM_RUNNING)))
    {
        return {RetCode::FAILED, "stream not opened or stopped"};
    }

    if (is_capture_dev)
    {
        (void)snd_pcm_drop(handle);
    }

    auto state = snd_pcm_state(handle);
    if (state != SND_PCM_STATE_PREPARED)
    {
        auto result = snd_pcm_prepare(handle);
        if (result < 0)
        {
            hstate = STREAM_STOPPED;
            AUDIO_ERROR_PRINT("ALSA device [%s] prepare failed: %s", hw_name.c_str(), snd_strerror(result));
            return {RetCode::FAILED, "snd_pcm_prepare failed"};
        }
    }

    worker_td = std::make_unique<std::thread>([this] {
        set_current_thread_scheduler_policy();
        return is_capture_dev ? read_loop() : write_loop();
    });

    return RetCode::OK;
}

RetCode AlsaDriver::stop()
{
    Mode expected = STREAM_RUNNING;
    if (!hstate.compare_exchange_strong(expected, STREAM_STOPPED) && hstate != STREAM_STOPPED)
    {
        return RetCode::NOACTION;
    }

    if (worker_td && worker_td->joinable())
    {
        worker_td->join();
    }

    if (is_capture_dev)
    {
        auto result = snd_pcm_drop(handle);
        if (result < 0)
        {
            AUDIO_ERROR_PRINT("ALSA device [%s] drop failed: %s", hw_name.c_str(), snd_strerror(result));
            return RetCode::FAILED;
        }
    }
    else
    {
        auto result = snd_pcm_drain(handle);
        if (result < 0)
        {
            AUDIO_ERROR_PRINT("ALSA device [%s] drain failed: %s", hw_name.c_str(), snd_strerror(result));
            return RetCode::FAILED;
        }
    }

    return RetCode::OK;
}

RetCode AlsaDriver::write(const char *data, size_t len)
{
    if (is_capture_dev)
    {
        return {RetCode::EWRITE, "Capture device does not support write"};
    }

    if (hstate == STREAM_STOPPED)
    {
        return {RetCode::ESTATE, "Stream stopped"};
    }

    if (!io_buffer->store(data, len))
    {
        return {RetCode::NOACTION, "Buffer full"};
    }
    return {RetCode::OK, "Success"};
}

RetCode AlsaDriver::read(char *data, size_t len)
{
    if (!is_capture_dev)
    {
        return {RetCode::EREAD, "Playback device does not support read"};
    }

    if (hstate == STREAM_STOPPED)
    {
        return {RetCode::ESTATE, "Stream stopped"};
    }

    if (!io_buffer->load(data, len))
    {
        return {RetCode::NOACTION, "Buffer empty"};
    }
    return RetCode::OK;
}

int AlsaDriver::xrun_recovery(int err)
{
    if (err == -EPIPE)
    {
        AUDIO_DEBUG_PRINT("ALSA device [%s] underrun/overrun detected", hw_name.c_str());
        err = snd_pcm_prepare(handle);
        if (err < 0)
        {
            AUDIO_ERROR_PRINT("ALSA device [%s] prepare after underrun failed: %s", hw_name.c_str(), snd_strerror(err));
        }
        return err;
    }
    else if (err == -ESTRPIPE)
    {
        AUDIO_DEBUG_PRINT("ALSA device [%s] suspended", hw_name.c_str());
        int retry_count = 0;
        while ((err = snd_pcm_resume(handle)) == -EAGAIN && retry_count++ < 10)
        {
            usleep(100000); // 100ms
        }
        if (err < 0)
        {
            err = snd_pcm_prepare(handle);
            if (err < 0)
            {
                AUDIO_ERROR_PRINT("ALSA device [%s] prepare after suspend failed: %s", hw_name.c_str(),
                                  snd_strerror(err));
                return err;
            }
        }
        return 0;
    }

    AUDIO_ERROR_PRINT("ALSA device [%s] error: %s", hw_name.c_str(), snd_strerror(err));
    return err;
}

void AlsaDriver::write_loop()
{
    bool ok = true;
    while (ok && hstate == STREAM_RUNNING)
    {
        auto cptr = dev_ps;
        auto data = io_buffer->data();
        if (!io_buffer->load_aside(dev_ps * dev_ch * sizeof(PCM_TYPE)))
        {
            AUDIO_DEBUG_PRINT("ALSA device [%s] buffer underflow", hw_name.c_str());
        }

        while (cptr > 0 && ok)
        {
            auto result = mmap_mode ? snd_pcm_mmap_writei(handle, data, cptr) : snd_pcm_writei(handle, data, cptr);
            if (result == -EAGAIN)
            {
                usleep(1000);
                continue;
            }
            if (result < 0)
            {
                if (xrun_recovery(result) < 0)
                {
                    ok = false;
                    break;
                }
                continue;
            }
            data += result * dev_ch * sizeof(PCM_TYPE);
            cptr -= result;
        }

        if ((*timer_cnt)())
        {
            AUDIO_DEBUG_PRINT("Alsa device [%s] write loop timeout", hw_name.c_str());
            ok = !strict_recover;
        }
    }

    hstate = STREAM_STOPPED;
    (void)snd_pcm_drop(handle);
    if (!ok)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] write loop exited with error", hw_name.c_str());
    }
}

void AlsaDriver::read_loop()
{
    bool ok = true;
    while (ok && hstate == STREAM_RUNNING)
    {
        auto cptr = dev_ps;
        auto data = io_buffer->data();
        while (cptr > 0 && ok)
        {
            auto result = mmap_mode ? snd_pcm_mmap_readi(handle, data, cptr) : snd_pcm_readi(handle, data, cptr);
            if (result == -EAGAIN)
            {
                usleep(1000);
                continue;
            }
            if (result < 0)
            {
                if (xrun_recovery(result) < 0)
                {
                    ok = false;
                    break;
                }
                continue;
            }
            data += result * dev_ch * sizeof(PCM_TYPE);
            cptr -= result;
        }

        if (ok && !io_buffer->store_aside(dev_ps * dev_ch * sizeof(PCM_TYPE)))
        {
            AUDIO_DEBUG_PRINT("ALSA device [%s] buffer overflow", hw_name.c_str());
        }

        if ((*timer_cnt)())
        {
            AUDIO_DEBUG_PRINT("ALSA device [%s] read loop timeout", hw_name.c_str());
            ok = !strict_recover;
        }
    }

    hstate = STREAM_STOPPED;
    (void)snd_pcm_drop(handle);

    if (!ok)
    {
        AUDIO_ERROR_PRINT("ALSA device [%s] read loop exited with error", hw_name.c_str());
    }
}

#elif WINDOWS_OS_ENVIRONMENT

#ifndef INITGUID
#define INITGUID
#endif
#include <audioclient.h>
#include <mmdeviceapi.h>
// must include after mmdeviceapi.h
#include <functiondiscoverykeys_devpkey.h>

void set_current_thread_scheduler_policy()
{
    auto thread = GetCurrentThread();
    SetThreadPriority(thread, THREAD_PRIORITY_ABOVE_NORMAL);
}

template <class T> void SafeRelease(__deref_inout_opt T **ppT)
{
    T *pTTemp = *ppT; // temp copy
    *ppT = nullptr;   // zero the input
    if (pTTemp)
    {
        pTTemp->Release();
    }
}

static std::wstring utf8_to_utf16(const std::string &str)
{
    if (str.empty())
    {
        return {};
    }

    int size_needed = MultiByteToWideChar(CP_UTF8, 0, str.c_str(), static_cast<int>(str.size()), nullptr, 0);
    std::wstring wstr(size_needed, 0);
    MultiByteToWideChar(CP_UTF8, 0, str.c_str(), static_cast<int>(str.size()), &wstr[0], size_needed);
    return wstr;
}

static std::string utf16_to_utf8(const std::wstring &wstr)
{
    if (wstr.empty())
    {
        return {};
    }

    int size_needed =
        WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), static_cast<int>(wstr.size()), nullptr, 0, nullptr, nullptr);
    std::string str(size_needed, 0);
    WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), static_cast<int>(wstr.size()), &str[0], size_needed, nullptr,
                        nullptr);
    return str;
}

static std::pair<std::wstring, std::wstring> normailzed_device_name(const std::string &name, bool capture)
{
    HRESULT hr = S_OK;
    IMMDeviceEnumerator *enumerator = nullptr;
    IMMDeviceCollection *collection = nullptr;
    IMMDevice *device = nullptr;
    IPropertyStore *store = nullptr;
    LPWSTR pwszID = nullptr;
    std::pair<std::wstring, std::wstring> ret{L"default", L"default"};

    hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                          reinterpret_cast<LPVOID *>(&enumerator));
    if (FAILED(hr))
    {
        goto SAFE_EXIT;
    }

    hr = enumerator->EnumAudioEndpoints(capture ? eCapture : eRender, DEVICE_STATE_ACTIVE, &collection);
    if (FAILED(hr))
    {
        goto SAFE_EXIT;
    }

    UINT count;
    hr = collection->GetCount(&count);
    if (FAILED(hr))
    {
        goto SAFE_EXIT;
    }

    if (count == 0)
    {
        AUDIO_INFO_PRINT("No endpoints found.");
        goto SAFE_EXIT;
    }

    for (ULONG i = 0; i < count; i++)
    {
        hr = collection->Item(i, &device);
        if (FAILED(hr))
        {
            continue;
        }

        hr = device->GetId(&pwszID);
        if (FAILED(hr))
        {
            continue;
        }

        hr = device->OpenPropertyStore(STGM_READ, &store);
        if (FAILED(hr))
        {
            CoTaskMemFree(pwszID);
            continue;
        }

        PROPVARIANT varName;
        PropVariantInit(&varName);
        hr = store->GetValue(PKEY_Device_FriendlyName, &varName);
        if (SUCCEEDED(hr) && varName.vt != VT_EMPTY)
        {
            std::wstring friendly_name = varName.pwszVal;
            if (friendly_name.find(utf8_to_utf16(name)) != std::wstring::npos)
            {
                ret.first = std::move(friendly_name);
                ret.second = pwszID;
                break;
            }
        }

        PropVariantClear(&varName);
        CoTaskMemFree(pwszID);
        SafeRelease(&store);
        SafeRelease(&device);
    }

SAFE_EXIT:
    SafeRelease(&enumerator);
    SafeRelease(&collection);
    SafeRelease(&device);
    SafeRelease(&store);
    return ret;
}

class WasapiDriver final : public AudioDevice
{
  public:
    WasapiDriver(const std::string &friendly_name, const std::wstring &device_name, bool capture, unsigned int fs,
                 unsigned int ps, unsigned int ch, bool enable_strict_recover);
    ~WasapiDriver() override;

    RetCode open() override;
    RetCode start() override;
    RetCode stop() override;
    RetCode write(const char *data, size_t len) override;
    RetCode read(char *data, size_t len) override;

  private:
    void write_loop();
    void read_loop();

  private:
    std::wstring uuid_name;
    HANDLE h_event;
    IAudioClient2 *audio_client;
    IAudioRenderClient *render;
    IAudioCaptureClient *capture;
};

WasapiDriver::WasapiDriver(const std::string &friendly_name, const std::wstring &device_name, bool capture,
                           unsigned int fs, unsigned int ps, unsigned int ch, bool enable_strict_recover)
    : AudioDevice(friendly_name, capture, fs, ps, ch, enable_strict_recover), uuid_name(device_name), h_event(nullptr),
      audio_client(nullptr), render(nullptr), capture(nullptr)
{
}

WasapiDriver::~WasapiDriver()
{
    (void)stop();

    if (h_event != nullptr)
    {
        CloseHandle(h_event);
        h_event = nullptr;
    }

    SafeRelease(&audio_client);
    SafeRelease(&render);
    SafeRelease(&capture);
}

RetCode WasapiDriver::open()
{
    RetCode ret = {RetCode::OK, "Success"};
    IMMDeviceEnumerator *enumerator = nullptr;
    IMMDevice *device = nullptr;
    WAVEFORMATEX wfx = {};

    auto hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                               reinterpret_cast<LPVOID *>(&enumerator));
    if (FAILED(hr))
    {
        ret = {RetCode::FAILED, "CoCreateInstance failed"};
        goto SAFE_EXIT;
    }

    if (hw_name == "default")
    {
        hr = enumerator->GetDefaultAudioEndpoint(is_capture_dev ? eCapture : eRender, eConsole, &device);
    }
    else
    {
        hr = enumerator->GetDevice(uuid_name.c_str(), &device);
    }

    if (FAILED(hr))
    {
        ret = {RetCode::FAILED, "IMMDeviceEnumerator::GetDefaultAudioEndpoint failed"};
        goto SAFE_EXIT;
    }

    hr = device->Activate(__uuidof(IAudioClient2), CLSCTX_ALL, nullptr, reinterpret_cast<LPVOID *>(&audio_client));
    if (FAILED(hr))
    {
        ret = {RetCode::FAILED, "IMMDevice::Activate failed"};
        goto SAFE_EXIT;
    }

    wfx.wFormatTag = WAVE_FORMAT_PCM;
    wfx.nChannels = static_cast<WORD>(dev_ch);
    max_ch = min_ch = dev_ch;
    wfx.nSamplesPerSec = dev_fs;
    wfx.wBitsPerSample = sizeof(PCM_TYPE) * 8;
    wfx.nBlockAlign = wfx.nChannels * wfx.wBitsPerSample / 8;
    wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;

    DWORD flags = (AUDCLNT_STREAMFLAGS_EVENTCALLBACK | AUDCLNT_STREAMFLAGS_RATEADJUST |
                   AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM | AUDCLNT_STREAMFLAGS_SRC_DEFAULT_QUALITY);
    hr = audio_client->Initialize(AUDCLNT_SHAREMODE_SHARED, flags, 0, 0, &wfx, nullptr);
    if (FAILED(hr))
    {
        return {RetCode::FAILED, "IAudioClient::Initialize failed"};
    }

    hr = is_capture_dev ? audio_client->GetService(__uuidof(IAudioCaptureClient), reinterpret_cast<LPVOID *>(&capture))
                        : audio_client->GetService(__uuidof(IAudioRenderClient), reinterpret_cast<LPVOID *>(&render));
    if (FAILED(hr))
    {
        ret = {RetCode::FAILED, "IAudioClient::GetService failed"};
        goto SAFE_EXIT;
    }

    hr = audio_client->GetBufferSize(&dev_ps);
    if (FAILED(hr))
    {
        ret = {RetCode::FAILED, "IAudioClient::GetBufferSize failed"};
        goto SAFE_EXIT;
    }

    h_event = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (h_event == nullptr)
    {
        ret = {RetCode::FAILED, "Create Event failed"};
        goto SAFE_EXIT;
    }
    hr = audio_client->SetEventHandle(h_event);
    if (FAILED(hr))
    {
        ret = {RetCode::FAILED, "IAudioClient::SetEventHandle failed"};
        goto SAFE_EXIT;
    }
    hstate = STREAM_OPENED;
    io_buffer = std::make_unique<KFifo>(dev_ps * sizeof(PCM_TYPE), 4, dev_ch);
    AUDIO_INFO_PRINT("WASAPI %s device [%s] opened. fs = %u, ps = %u, chan = %u, min_chan = %u, max_chan = %u",
                     capture ? "capture" : "playback", hw_name.c_str(), dev_fs, dev_ps, dev_ch, min_ch, max_ch);
    timer_cnt = std::make_unique<TimerCounter>(hw_name, 0xffff);

SAFE_EXIT:
    SafeRelease(&enumerator);
    SafeRelease(&device);
    return ret;
}

RetCode WasapiDriver::start()
{
    Mode expected = STREAM_OPENED;
    if (!hstate.compare_exchange_strong(expected, STREAM_RUNNING) &&
        !(expected = STREAM_STOPPED, hstate.compare_exchange_strong(expected, STREAM_RUNNING)))
    {
        return {RetCode::FAILED, "stream not opened or stopped"};
    }

    auto hr = audio_client->Start();
    if (FAILED(hr))
    {
        return {RetCode::FAILED, "IAudioClient::Start failed"};
    }

    worker_td = std::make_unique<std::thread>([this] {
        set_current_thread_scheduler_policy();
        return is_capture_dev ? read_loop() : write_loop();
    });
    return {RetCode::OK, "Success"};
}

RetCode WasapiDriver::stop()
{
    Mode expected = STREAM_RUNNING;
    if (!hstate.compare_exchange_strong(expected, STREAM_STOPPED) && hstate != STREAM_STOPPED)
    {
        return {RetCode::FAILED, "Not running"};
    }

    SetEvent(h_event);
    auto hr = audio_client->Stop();

    if (worker_td && worker_td->joinable())
    {
        worker_td->join();
    }

    if (h_event != nullptr)
    {
        CloseHandle(h_event);
        h_event = nullptr;
    }

    return FAILED(hr) ? RetCode{RetCode::FAILED, "IAudioClient::Stop failed"} : RetCode{RetCode::OK, "Success"};
}

RetCode WasapiDriver::write(const char *data, size_t len)
{
    if (is_capture_dev)
    {
        return {RetCode::FAILED, "Capture device does not support write"};
    }

    if (hstate == STREAM_STOPPED)
    {
        return {RetCode::ESTATE, "Stream stopped"};
    }

    if (!io_buffer->store(data, len))
    {
        return {RetCode::NOACTION, "Buffer full"};
    }
    return {RetCode::OK, "Success"};
}

RetCode WasapiDriver::read(char *data, size_t len)
{
    if (!is_capture_dev)
    {
        return {RetCode::EREAD, "Playback device does not support read"};
    }

    if (hstate == STREAM_STOPPED)
    {
        return {RetCode::ESTATE, "Stream stopped"};
    }

    if (!io_buffer->load(data, len))
    {
        return {RetCode::NOACTION, "Buffer empty"};
    }

    return {RetCode::OK, "Success"};
}

void WasapiDriver::write_loop()
{
    bool ok = true;
    while (ok && hstate == STREAM_RUNNING)
    {
        DWORD wait_result = WaitForSingleObject(h_event, 100);
        if (wait_result == WAIT_TIMEOUT)
        {
            if (hstate != STREAM_RUNNING)
            {
                break;
            }
            continue;
        }
        else if (wait_result != WAIT_OBJECT_0)
        {
            AUDIO_ERROR_PRINT("WASAPI device [%s] wait failed", hw_name.c_str());
            continue;
        }

        UINT32 padding = 0;
        auto hr = audio_client->GetCurrentPadding(&padding);
        if (FAILED(hr))
        {
            ok = false;
            break;
        }

        UINT32 frames = dev_ps - padding;
        if (frames == 0)
        {
            continue;
        }

        BYTE *data;
        hr = render->GetBuffer(frames, &data);
        if (FAILED(hr))
        {
            ok = false;
            break;
        }

        if (!io_buffer->load(reinterpret_cast<char *>(data), frames * dev_ch * sizeof(PCM_TYPE)))
        {
            AUDIO_DEBUG_PRINT("WASAPI device [%s] buffer underflow", hw_name.c_str());
        }

        hr = render->ReleaseBuffer(frames, 0);
        if (FAILED(hr))
        {
            ok = false;
            break;
        }

        if ((*timer_cnt)())
        {
            AUDIO_DEBUG_PRINT("WASAPI device [%s] write loop timeout", hw_name.c_str());
            ok = !strict_recover;
        }
    }

    if (!ok)
    {
        hstate = STREAM_STOPPED;
        (void)audio_client->Stop();
        AUDIO_ERROR_PRINT("WASAPI device [%s] write loop exited with error", hw_name.c_str());
    }
}

void WasapiDriver::read_loop()
{
    bool ok = true;
    while (ok && hstate == STREAM_RUNNING)
    {
        DWORD wait_result = WaitForSingleObject(h_event, 100);
        if (wait_result == WAIT_TIMEOUT)
        {
            if (hstate != STREAM_RUNNING)
            {
                break;
            }
            continue;
        }
        else if (wait_result != WAIT_OBJECT_0)
        {
            AUDIO_ERROR_PRINT("WASAPI device [%s] wait failed", hw_name.c_str());
            continue;
        }

        UINT32 packet_sz = 0;
        do
        {
            auto hr = capture->GetNextPacketSize(&packet_sz);
            if (FAILED(hr))
            {
                AUDIO_ERROR_PRINT("WASAPI device [%s] GetNextPacketSize failed: 0x%08x", hw_name.c_str(), hr);
                ok = false;
                break;
            }

            if (packet_sz == 0)
            {
                break;
            }

            BYTE *data;
            DWORD flags;
            UINT32 frames;

            hr = capture->GetBuffer(&data, &frames, &flags, nullptr, nullptr);
            if (FAILED(hr))
            {
                AUDIO_ERROR_PRINT("WASAPI device [%s] GetBuffer failed: 0x%08x", hw_name.c_str(), hr);
                ok = false;
                break;
            }

            if (frames != 0)
            {
                if (flags & AUDCLNT_BUFFERFLAGS_SILENT)
                {
                    if (!io_buffer->store_aside(frames * dev_ch * sizeof(PCM_TYPE)))
                    {
                        AUDIO_DEBUG_PRINT("WASAPI device [%s] buffer overflow", hw_name.c_str());
                    }
                }
                else
                {
                    if (!io_buffer->store(reinterpret_cast<char *>(data), frames * dev_ch * sizeof(PCM_TYPE)))
                    {
                        AUDIO_DEBUG_PRINT("WASAPI device [%s] buffer overflow", hw_name.c_str());
                    }
                }
            }

            hr = capture->ReleaseBuffer(frames);
            if (FAILED(hr))
            {
                AUDIO_ERROR_PRINT("WASAPI device [%s] ReleaseBuffer failed: 0x%08x", hw_name.c_str(), hr);
                ok = false;
                break;
            }
        } while (packet_sz > 0);

        if ((*timer_cnt)())
        {
            AUDIO_DEBUG_PRINT("WASAPI device [%s] read loop timeout", hw_name.c_str());
            ok = !strict_recover;
        }
    }

    if (!ok)
    {
        hstate = STREAM_STOPPED;
        (void)audio_client->Stop();
        AUDIO_ERROR_PRINT("WASAPI device [%s] read loop exited with error", hw_name.c_str());
    }
}

#endif

// Wave file device
class WaveDevice final : public AudioDevice
{
  public:
    WaveDevice(const std::string &name, bool capture, unsigned int fs, unsigned int ps, unsigned int ch,
               unsigned int cyc);
    ~WaveDevice() override;

    RetCode open() override;
    RetCode start() override;
    RetCode stop() override;
    RetCode write(const char *data, size_t len) override;
    RetCode read(char *data, size_t len) override;

  private:
    unsigned int cycles;
    std::unique_ptr<WavFile> wav_file;
};

WaveDevice::WaveDevice(const std::string &name, bool capture, unsigned int fs, unsigned int ps, unsigned int ch,
                       unsigned int cyc)
    : AudioDevice(name, capture, fs, ps, ch, false), cycles(cyc), wav_file(std::make_unique<WavFile>())
{
}

WaveDevice::~WaveDevice()
{
    (void)stop();
}

RetCode WaveDevice::open()
{
    WavFile::mode open_mode = is_capture_dev ? WavFile::in : WavFile::out;

    auto ret = wav_file->open(hw_name, open_mode);
    if (!ret)
    {
        return ret;
    }

    if (open_mode == WavFile::out)
    {
        wav_file->set_channel_number(static_cast<uint16_t>(dev_ch));
        wav_file->set_sample_rate(dev_fs);
        wav_file->set_bits_per_sample(sizeof(PCM_TYPE) * 8);
    }
    else
    {
        dev_ch = wav_file->channel_number();
        dev_fs = wav_file->sample_rate();
    }

    min_ch = max_ch = dev_ch;

    hstate = STREAM_OPENED;
    AUDIO_INFO_PRINT("WAV %s file device [%s] opened. fs = %u, ps = %u, chan = %u", is_capture_dev ? "IN" : "OUT",
                     hw_name.c_str(), dev_fs, dev_ps, dev_ch);
    return {RetCode::OK, "Success"};
}

RetCode WaveDevice::start()
{
    Mode expected = STREAM_OPENED;
    if (!hstate.compare_exchange_strong(expected, STREAM_RUNNING) &&
        !(expected = STREAM_STOPPED, hstate.compare_exchange_strong(expected, STREAM_RUNNING)))
    {
        return {RetCode::FAILED, "stream not opened or stopped"};
    }

    return {RetCode::OK, "Success"};
}

RetCode WaveDevice::stop()
{
    Mode expected = STREAM_RUNNING;
    if (!hstate.compare_exchange_strong(expected, STREAM_STOPPED))
    {
        return {RetCode::OK, "Not running"};
    }

    return {RetCode::OK, "Success"};
}

RetCode WaveDevice::write(const char *data, size_t len)
{
    if (is_capture_dev)
    {
        return {RetCode::FAILED, "Capture device does not support write"};
    }

    if (hstate != STREAM_RUNNING)
    {
        return {RetCode::FAILED, "Device not running"};
    }

    size_t frames = len / (dev_ch * sizeof(PCM_TYPE));
    if (frames == 0)
    {
        return {RetCode::OK, "No data to write"};
    }

    return wav_file->write(reinterpret_cast<const PCM_TYPE *>(data), frames);
}

RetCode WaveDevice::read(char *data, size_t len)
{
    if (!is_capture_dev)
    {
        return {RetCode::EREAD, "Playback device does not support read"};
    }

    if (hstate != STREAM_RUNNING)
    {
        return {RetCode::FAILED, "Device not running"};
    }

    size_t frames = len / (dev_ch * sizeof(PCM_TYPE));
    if (frames == 0)
    {
        return {RetCode::OK, "No data requested"};
    }

    auto ret = wav_file->read(reinterpret_cast<PCM_TYPE *>(data), frames);
    if (ret == RetCode::INVSEEK && cycles > 0)
    {
        AUDIO_INFO_PRINT("WAV file [%s] reached end, seeking to beginning, cycle %u", hw_name.c_str(), cycles--);
        wav_file->seek(0);
        ret = wav_file->read(reinterpret_cast<PCM_TYPE *>(data), frames);
    }

    return ret;
}

// Echo device
class SimuDevice final : public AudioDevice
{
  public:
    SimuDevice(const std::string &name, bool capture, unsigned int fs, unsigned int ps, unsigned int ch);
    ~SimuDevice() override;

    RetCode open() override;
    RetCode start() override;
    RetCode stop() override;
    RetCode write(const char *data, size_t len) override;
    RetCode read(char *data, size_t len) override;
};

SimuDevice::SimuDevice(const std::string &name, bool capture, unsigned int fs, unsigned int ps, unsigned int ch)
    : AudioDevice(name, capture, fs, ps, ch, false)
{
}

SimuDevice::~SimuDevice()
{
    (void)stop();
}

RetCode SimuDevice::open()
{
    hstate = STREAM_OPENED;
    io_buffer = std::make_unique<KFifo>(dev_ps * sizeof(PCM_TYPE), 4, dev_ch);
    AUDIO_INFO_PRINT("Simu device [%s] opened. fs = %u, ps = %u, chan = %u", hw_name.c_str(), dev_fs, dev_ps, dev_ch);
    return {RetCode::OK, "Success"};
}

RetCode SimuDevice::start()
{
    Mode expected = STREAM_OPENED;
    if (!hstate.compare_exchange_strong(expected, STREAM_RUNNING) &&
        !(expected = STREAM_STOPPED, hstate.compare_exchange_strong(expected, STREAM_RUNNING)))
    {
        return {RetCode::FAILED, "stream not opened or stopped"};
    }

    AUDIO_INFO_PRINT("Simu device [%s] started", hw_name.c_str());
    return {RetCode::OK, "Success"};
}

RetCode SimuDevice::stop()
{
    Mode expected = STREAM_RUNNING;
    if (!hstate.compare_exchange_strong(expected, STREAM_STOPPED))
    {
        return {RetCode::OK, "Not running"};
    }

    AUDIO_INFO_PRINT("Simu device [%s] stopped", hw_name.c_str());
    return {RetCode::OK, "Success"};
}

RetCode SimuDevice::write(const char *data, size_t len)
{
    if (hstate != STREAM_RUNNING)
    {
        return {RetCode::FAILED, "Device not running"};
    }

    return io_buffer->store(data, len) ? RetCode::OK : RetCode::NOACTION;
}

RetCode SimuDevice::read(char *data, size_t len)
{
    if (hstate != STREAM_RUNNING)
    {
        return {RetCode::FAILED, "Device not running"};
    }

    return io_buffer->load(data, len) ? RetCode::OK : RetCode::NOACTION;
}

// Null device
class NullDevice final : public AudioDevice
{
  public:
    NullDevice(const std::string &name, bool capture, unsigned int fs, unsigned int ps, unsigned int ch);
    ~NullDevice() override;

    RetCode open() override;
    RetCode start() override;
    RetCode stop() override;
    RetCode write(const char *data, size_t len) override;
    RetCode read(char *data, size_t len) override;
};

NullDevice::NullDevice(const std::string &name, bool capture, unsigned int fs, unsigned int ps, unsigned int ch)
    : AudioDevice(name, capture, fs, ps, ch, false)
{
}

NullDevice::~NullDevice()
{
    (void)stop();
}

RetCode NullDevice::open()
{
    hstate = STREAM_OPENED;
    AUDIO_INFO_PRINT("Null device [%s] opened. fs = %u, ps = %u, chan = %u", hw_name.c_str(), dev_fs, dev_ps, dev_ch);
    return {RetCode::OK, "Success"};
}

RetCode NullDevice::start()
{
    Mode expected = STREAM_OPENED;
    if (!hstate.compare_exchange_strong(expected, STREAM_RUNNING) &&
        !(expected = STREAM_STOPPED, hstate.compare_exchange_strong(expected, STREAM_RUNNING)))
    {
        return {RetCode::FAILED, "stream not opened or stopped"};
    }

    AUDIO_DEBUG_PRINT("Null device [%s] started", hw_name.c_str());
    return {RetCode::OK, "Success"};
}

RetCode NullDevice::stop()
{
    Mode expected = STREAM_RUNNING;
    if (!hstate.compare_exchange_strong(expected, STREAM_STOPPED))
    {
        return {RetCode::OK, "Not running"};
    }

    AUDIO_DEBUG_PRINT("Null device [%s] stopped", hw_name.c_str());
    return {RetCode::OK, "Success"};
}

RetCode NullDevice::write(const char * /*data*/, size_t /*len*/)
{
    if (hstate != STREAM_RUNNING)
    {
        return {RetCode::FAILED, "Device not running"};
    }

    return {RetCode::OK, "Success"};
}

RetCode NullDevice::read(char * /*data*/, size_t /*len*/)
{
    if (hstate != STREAM_RUNNING)
    {
        return {RetCode::FAILED, "Device not running"};
    }

    return {RetCode::OK, "Success"};
}

std::unique_ptr<AudioDevice> make_audio_driver(int type, const AudioDeviceName &name, unsigned int fs, unsigned int ps,
                                               unsigned int ch, bool enable_strict_recover)
{
    if (type == PHSY_IAS)
    {
#if WINDOWS_OS_ENVIRONMENT
        auto ret = normailzed_device_name(name.first, true);
        return std::make_unique<WasapiDriver>(utf16_to_utf8(ret.first), ret.second, true, fs, ps, ch,
                                              enable_strict_recover);
#elif LINUX_OS_ENVIRONMENT
        return std::make_unique<AlsaDriver>(normailzed_device_name(name), true, fs, ps, ch, enable_strict_recover);
#endif
    }
    else if (type == PHSY_OAS)
    {
#if WINDOWS_OS_ENVIRONMENT
        auto ret = normailzed_device_name(name.first, false);
        return std::make_unique<WasapiDriver>(utf16_to_utf8(ret.first), ret.second, false, fs, ps, ch,
                                              enable_strict_recover);
#elif LINUX_OS_ENVIRONMENT
        return std::make_unique<AlsaDriver>(normailzed_device_name(name), false, fs, ps, ch, enable_strict_recover);
#endif
    }
    else if (type == WAVE_IAS)
    {
        return std::make_unique<WaveDevice>(name.first, true, fs, ps, ch, name.second);
    }
    else if (type == WAVE_OAS)
    {
        return std::make_unique<WaveDevice>(name.first, false, fs, ps, ch, name.second);
    }
    else if (type == ECHO_IAS)
    {
        return std::make_unique<SimuDevice>(name.first + "_echo", true, fs, ps, ch);
    }
    else if (type == NULL_IAS)
    {
        return std::make_unique<NullDevice>(name.first, true, fs, ps, ch);
    }
    else
    {
        return std::make_unique<NullDevice>(name.first, false, fs, ps, ch);
    }
}