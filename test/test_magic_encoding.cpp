#include "audio_network.h"
#include <iomanip>
#include <iostream>

int main()
{
    std::cout << "Testing Magic Number Encoding\n";
    std::cout << "============================\n\n";

    // Test all combinations of codec and priority
    AudioCodecType codecs[] = {AudioCodecType::OPUS, AudioCodecType::PCM};
    AudioPriority priorities[] = {AudioPriority::LOW, AudioPriority::MEDIUM, AudioPriority::HIGH};

    const char *codec_names[] = {"OPUS", "PCM"};
    const char *priority_names[] = {"LOW", "MEDIUM", "HIGH"};

    std::cout << "Encoding Test:\n";
    std::cout << "Codec\t\tPriority\tMagic Num\tBinary\n";
    std::cout << "-----\t\t--------\t---------\t------\n";

    for (int c = 0; c < 2; c++)
    {
        for (int p = 0; p < 3; p++)
        {
            uint8_t magic = DataPacket::encode_magic_num(codecs[c], priorities[p]);

            std::cout << codec_names[c] << "\t\t" << priority_names[p] << "\t\t0x" << std::hex << std::uppercase
                      << std::setfill('0') << std::setw(2) << (int)magic << "\t\t";

            // Print binary representation
            for (int i = 7; i >= 0; i--)
            {
                std::cout << ((magic >> i) & 1);
            }
            std::cout << std::endl;
        }
    }

    std::cout << "\nDecoding Test:\n";
    std::cout << "Magic Num\tDecoded Codec\tDecoded Priority\tValid\n";
    std::cout << "---------\t-------------\t----------------\t-----\n";

    for (int c = 0; c < 2; c++)
    {
        for (int p = 0; p < 3; p++)
        {
            uint8_t magic = DataPacket::encode_magic_num(codecs[c], priorities[p]);

            AudioCodecType decoded_codec = DataPacket::decode_magic_codec(magic);
            AudioPriority decoded_priority = DataPacket::decode_magic_priority(magic);
            bool valid = DataPacket::is_valid_magic_num(magic);

            std::cout << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << (int)magic
                      << "\t\t";

            if (decoded_codec == AudioCodecType::OPUS)
                std::cout << "OPUS";
            else
                std::cout << "PCM";
            std::cout << "\t\t";

            if (decoded_priority == AudioPriority::LOW)
                std::cout << "LOW";
            else if (decoded_priority == AudioPriority::MEDIUM)
                std::cout << "MEDIUM";
            else
                std::cout << "HIGH";
            std::cout << "\t\t\t";

            std::cout << (valid ? "YES" : "NO") << std::endl;
        }
    }

    return 0;
}