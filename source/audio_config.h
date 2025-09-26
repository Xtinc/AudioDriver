#ifndef AUDIO_CONFIG_H
#define AUDIO_CONFIG_H

#include <list>
#include <map>
#include <string>
#include <mutex>

struct DeviceConfig
{
    std::string input_device_id;
    std::string input_device_name;
    std::string output_device_id;
    std::string output_device_name;
};

class INIReader
{
  public:
    struct Section
    {
        std::string name;
        std::map<std::string, std::string> values;
    };

    explicit INIReader(const std::string &filename);
    ~INIReader() = default;

    bool SaveDeviceConfig(const DeviceConfig &config);
    DeviceConfig LoadDeviceConfig();

  private:
    std::string get_string(const std::string &section, const std::string &key, const std::string &default_value = "");
    void set_string(const std::string &section, const std::string &key, const std::string &value);
    bool save();
    bool load();
    bool exists() const;

    std::string trim(const std::string &str);
    std::list<Section>::iterator find_section(const std::string &section_name);

  private:
    std::mutex mutex_;
    std::string filename_;
    std::list<Section> sections_;
};

#endif // AUDIO_CONFIG_H