#ifndef AUDIO_CONFIG_H
#define AUDIO_CONFIG_H

#include <map>
#include <string>

class INIReader
{
  public:
    struct Proxy
    {
        std::map<std::string, std::string> &data_;
        const std::string key_;

        Proxy &operator=(const std::string &v)
        {
            data_[key_] = v;
            return *this;
        }
        Proxy &operator=(int v)
        {
            data_[key_] = std::to_string(v);
            return *this;
        }
        Proxy &operator=(float v)
        {
            data_[key_] = std::to_string(v);
            return *this;
        }
        Proxy &operator=(bool v)
        {
            data_[key_] = v ? "true" : "false";
            return *this;
        }

        template <typename T> T cast(T default_val = T{}) const;
    };

    explicit INIReader(const std::string &filename);
    ~INIReader() = default;

    Proxy operator[](const std::string &key)
    {
        return {data_, key};
    }
    bool save();

  private:
    bool load();
    std::string trim(const std::string &str) const;

    std::string filename_;
    std::map<std::string, std::string> data_;
};

template <> inline std::string INIReader::Proxy::cast<std::string>(std::string dv) const
{
    auto it = data_.find(key_);
    return it != data_.end() ? it->second : dv;
}

template <> inline int INIReader::Proxy::cast<int>(int dv) const
{
    auto it = data_.find(key_);
    if (it == data_.end())
    {
        return dv;
    }
    try
    {
        return std::stoi(it->second);
    }
    catch (...)
    {
        return dv;
    }
}

template <> inline float INIReader::Proxy::cast<float>(float dv) const
{
    auto it = data_.find(key_);
    if (it == data_.end())
    {
        return dv;
    }
    try
    {
        return std::stof(it->second);
    }
    catch (...)
    {
        return dv;
    }
}

template <> inline bool INIReader::Proxy::cast<bool>(bool dv) const
{
    auto it = data_.find(key_);
    if (it == data_.end())
    {
        return dv;
    }
    const auto &v = it->second;
    return v == "true" || v == "True";
}

#endif // AUDIO_CONFIG_H