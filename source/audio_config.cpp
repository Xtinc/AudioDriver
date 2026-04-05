#include "audio_config.h"
#include "audio_interface.h"
#include <fstream>

INIReader::INIReader(const std::string &filename) : filename_(filename)
{
    (void)load();
    data_["HotPlug.InputDeviceID"] = "";
    data_["HotPlug.InputDeviceName"] = "";
    data_["HotPlug.OutputDeviceID"] = "";
    data_["HotPlug.OutputDeviceName"] = "";
    data_["Report.Latency"] = "false";
    data_["Report.LatencyInterval"] = "60";
}

bool INIReader::save()
{
    std::ofstream file(filename_);
    if (!file.is_open())
    {
        return false;
    }

    std::string cur_section;
    for (const auto &kv : data_)
    {
        size_t dot = kv.first.find('.');
        if (dot == std::string::npos)
        {
            continue;
        }
        std::string sec = kv.first.substr(0, dot);
        std::string key = kv.first.substr(dot + 1);
        if (sec != cur_section)
        {
            if (!cur_section.empty())
            {
                file << "\n";
            }
            file << "[" << sec << "]\n";
            cur_section = sec;
        }
        file << key << "=" << kv.second << "\n";
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

    data_.clear();
    std::string line;
    std::string current_section;

    while (std::getline(file, line))
    {
        line = trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#')
        {
            continue;
        }

        if (line.front() == '[' && line.back() == ']')
        {
            current_section = line.substr(1, line.length() - 2);
            continue;
        }

        size_t pos = line.find('=');
        if (pos != std::string::npos && !current_section.empty())
        {
            data_[current_section + "." + trim(line.substr(0, pos))] = trim(line.substr(pos + 1));
        }
    }
    return true;
}

std::string INIReader::trim(const std::string &str) const
{
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
    {
        return "";
    }
    size_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}
