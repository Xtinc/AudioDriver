#include "audio_config.h"
#include "audio_interface.h"
#include <fstream>

INIReader::INIReader(const std::string &filename) : filename_(filename)
{
    if (!load())
    {
        data_["HotPlug.InputDeviceID"] = "";
        data_["HotPlug.InputDeviceName"] = "";
        data_["HotPlug.OutputDeviceID"] = "";
        data_["HotPlug.OutputDeviceName"] = "";
        data_["Report.Latency"] = "false";
        data_["Report.LatencyInterval"] = "60";
        data_["Log.DebugPrint"] = "false";
        (void)save();
    }
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
        if (sec.find(':') != std::string::npos)
        {
            continue;
        }
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

        if (current_section.empty())
        {
            continue;
        }

        size_t pos = line.find('=');
        if (pos != std::string::npos)
        {
            data_[current_section + "." + trim(line.substr(0, pos))] = trim(line.substr(pos + 1));
        }
    }
    return true;
}

std::vector<INIReader::ConnectionEntry> INIReader::get_connections() const
{
    // Keys have the form "Connection:<itoken>.<index>.<field>"
    const std::string match = "Connection:";
    std::map<std::string, ConnectionEntry> entries;

    for (const auto &kv : data_)
    {
        if (kv.first.compare(0, match.size(), match) != 0)
        {
            continue;
        }
        std::string rest = kv.first.substr(match.size()); // "<itoken>.<index>.<field>"
        size_t dot = rest.rfind('.');
        if (dot == std::string::npos)
        {
            continue;
        }
        const std::string group = rest.substr(0, dot);
        const std::string field = rest.substr(dot + 1);
        auto &e = entries[group];
        try
        {
            if (field == "token")
            {
                int v = std::stoi(kv.second);
                if (v >= 0 && v <= 254)
                    e.token = static_cast<unsigned char>(v);
            }
            else if (field == "ip")
            {
                e.ip = kv.second;
            }
            else if (field == "port")
            {
                e.port = static_cast<unsigned short>(std::stoi(kv.second));
            }
        }
        catch (...)
        {
        }
    }

    std::vector<ConnectionEntry> result;
    for (auto &p : entries)
    {
        if (p.second.token == 0xFF) // token field was absent or invalid
        {
            continue;
        }
        size_t dot = p.first.find('.');
        if (dot == std::string::npos)
        {
            continue;
        }
        try
        {
            int ival = std::stoi(p.first.substr(0, dot));
            if (ival < 0 || ival > 254)
            {
                continue;
            }
            p.second.itoken = static_cast<unsigned char>(ival);
            result.push_back(std::move(p.second));
        }
        catch (...)
        {
        }
    }
    return result;
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
