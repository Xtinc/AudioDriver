#include "audio_config.h"
#include "audio_interface.h"
#include <algorithm>
#include <fstream>

INIReader::INIReader(const std::string &filename) : filename_(filename)
{
    if (!load())
    {
        AUDIO_ERROR_PRINT("Failed to load INI file: %s", filename.c_str());
    }
}

std::string INIReader::get_string(const std::string &section, const std::string &key, const std::string &default_value)
{
    auto section_it = find_section(section);
    if (section_it == sections_.end())
        return default_value;

    auto key_it = section_it->values.find(key);
    return (key_it != section_it->values.end()) ? key_it->second : default_value;
}

void INIReader::set_string(const std::string &section, const std::string &key, const std::string &value)
{
    auto section_it = find_section(section);
    if (section_it == sections_.end())
    {
        sections_.push_back({section, {}});
        section_it = std::prev(sections_.end());
    }
    section_it->values[key] = value;
}

bool INIReader::save()
{
    std::ofstream file(filename_);
    if (!file.is_open())
        return false;

    for (const auto &section : sections_)
    {
        file << "[" << section.name << "]\n";
        for (const auto &kv : section.values)
            file << kv.first << "=" << kv.second << "\n";
        file << "\n";
    }
    return true;
}

bool INIReader::load()
{
    std::ifstream file(filename_);
    if (!file.is_open())
        return false;

    sections_.clear();
    std::string line;
    Section *current_section = nullptr;

    while (std::getline(file, line))
    {
        line = trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#')
            continue;

        if (line.front() == '[' && line.back() == ']')
        {
            sections_.push_back({line.substr(1, line.length() - 2), {}});
            current_section = &sections_.back();
            continue;
        }

        size_t pos = line.find('=');
        if (pos != std::string::npos && current_section)
        {
            current_section->values[trim(line.substr(0, pos))] = trim(line.substr(pos + 1));
        }
    }
    return true;
}

std::string INIReader::trim(const std::string &str) const
{
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
        return "";
    size_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}

std::list<INIReader::Section>::iterator INIReader::find_section(const std::string &section_name)
{
    return std::find_if(sections_.begin(), sections_.end(), [&](const Section &s) { return s.name == section_name; });
}

bool INIReader::SaveDeviceConfig(const DeviceConfig &config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    set_string("Audio", "InputDeviceID", config.input_device_id);
    set_string("Audio", "InputDeviceName", config.input_device_name);
    set_string("Audio", "OutputDeviceID", config.output_device_id);
    set_string("Audio", "OutputDeviceName", config.output_device_name);

    bool result = save();
    AUDIO_DEBUG_PRINT("%s", result ? "Device config saved successfully" : "Failed to save device config");
    return result;
}

DeviceConfig INIReader::LoadDeviceConfig()
{
    std::lock_guard<std::mutex> lock(mutex_);
    DeviceConfig config{get_string("Audio", "InputDeviceID", ""), get_string("Audio", "InputDeviceName", ""),
                        get_string("Audio", "OutputDeviceID", ""), get_string("Audio", "OutputDeviceName", "")};

    AUDIO_DEBUG_PRINT("Device config loaded successfully");
    return config;
}