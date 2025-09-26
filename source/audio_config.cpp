#include "audio_config.h"
#include "audio_interface.h"
#include <fstream>
#include <sstream>

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
    {
        return default_value;
    }

    auto key_it = section_it->values.find(key);
    if (key_it == section_it->values.end())
    {
        return default_value;
    }

    return key_it->second;
}

void INIReader::set_string(const std::string &section, const std::string &key, const std::string &value)
{
    auto section_it = find_section(section);
    if (section_it == sections_.end())
    {
        Section new_section;
        new_section.name = section;
        sections_.push_back(new_section);
        section_it = std::prev(sections_.end());
    }

    section_it->values[key] = value;
}

bool INIReader::save()
{
    std::ofstream file(filename_);
    if (!file.is_open())
    {
        return false;
    }

    for (const auto &section : sections_)
    {
        file << "[" << section.name << "]" << std::endl;
        for (const auto &key_pair : section.values)
        {
            file << key_pair.first << "=" << key_pair.second << std::endl;
        }
        file << std::endl;
    }

    return true;
}

bool INIReader::load()
{
    std::ifstream file(filename_);
    if (!file.is_open())
    {
        return false;
    }

    sections_.clear();
    std::string line;
    std::string current_section;
    auto current_section_it = sections_.end();

    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.empty() || line[0] == ';' || line[0] == '#')
            continue;

        if (line[0] == '[' && line.back() == ']')
        {
            current_section = line.substr(1, line.length() - 2);
            Section new_section;
            new_section.name = current_section;
            sections_.push_back(new_section);
            current_section_it = std::prev(sections_.end());
            continue;
        }

        size_t pos = line.find('=');
        if (pos != std::string::npos && current_section_it != sections_.end())
        {
            std::string key = trim(line.substr(0, pos));
            std::string value = trim(line.substr(pos + 1));
            current_section_it->values[key] = value;
        }
    }

    return true;
}

bool INIReader::exists() const
{
    std::ifstream file(filename_);
    return file.good();
}

std::string INIReader::trim(const std::string &str)
{
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
        return "";

    size_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}

std::list<INIReader::Section>::iterator INIReader::find_section(const std::string &section_name)
{
    return std::find_if(sections_.begin(), sections_.end(),
                        [&section_name](const Section &section) { return section.name == section_name; });
}

bool INIReader::SaveDeviceConfig(const DeviceConfig &config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    set_string("Audio", "InputDeviceID", config.input_device_id);
    set_string("Audio", "InputDeviceName", config.input_device_name);
    set_string("Audio", "OutputDeviceID", config.output_device_id);
    set_string("Audio", "OutputDeviceName", config.output_device_name);

    bool result = save();
    if (result)
    {
        AUDIO_DEBUG_PRINT("Device config saved successfully");
    }
    else
    {
        AUDIO_ERROR_PRINT("Failed to save device config");
    }

    return result;
}

DeviceConfig INIReader::LoadDeviceConfig()
{
    std::lock_guard<std::mutex> lock(mutex_);
    DeviceConfig config;

    config.input_device_id = get_string("Audio", "InputDeviceID", "");
    config.input_device_name = get_string("Audio", "InputDeviceName", "");
    config.output_device_id = get_string("Audio", "OutputDeviceID", "");
    config.output_device_name = get_string("Audio", "OutputDeviceName", "");

    AUDIO_DEBUG_PRINT("Device config loaded successfully");
    return config;
}