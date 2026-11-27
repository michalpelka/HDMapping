#pragma once

// Synthetic pcap captures for the rosbags tests: UDP payloads wrapped in Ethernet/IPv4/UDP frames, optionally split
// into IPv4 fragments, written as a classic pcap file.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace pcap_test
{

    namespace fs = std::filesystem;

    inline void putLe(std::vector<uint8_t>& out, size_t offset, uint64_t value, int bytes)
    {
        for (int i = 0; i < bytes; ++i)
            out[offset + i] = static_cast<uint8_t>(value >> (8 * i));
    }

    inline void putBe16(std::vector<uint8_t>& out, size_t offset, uint16_t value)
    {
        out[offset] = static_cast<uint8_t>(value >> 8);
        out[offset + 1] = static_cast<uint8_t>(value);
    }

    // Wraps a UDP payload in Ethernet/IPv4/UDP, split into IPv4 fragments carrying at
    // most `fragment_bytes` (a multiple of 8) of IP payload each; 0 = unfragmented.
    inline std::vector<std::vector<uint8_t>> udpFrames(
        const std::vector<uint8_t>& payload, uint16_t dport, uint16_t ip_id, size_t fragment_bytes = 0)
    {
        std::vector<uint8_t> datagram(8 + payload.size());
        putBe16(datagram, 0, 7502);
        putBe16(datagram, 2, dport);
        putBe16(datagram, 4, static_cast<uint16_t>(datagram.size()));
        std::copy(payload.begin(), payload.end(), datagram.begin() + 8);

        if (fragment_bytes == 0)
            fragment_bytes = datagram.size();
        std::vector<std::vector<uint8_t>> frames;
        for (size_t offset = 0; offset < datagram.size(); offset += fragment_bytes)
        {
            const size_t n = std::min(fragment_bytes, datagram.size() - offset);
            const bool more = offset + n < datagram.size();
            std::vector<uint8_t> f(14 + 20 + n, 0);
            putBe16(f, 12, 0x0800);
            f[14] = 0x45;
            putBe16(f, 16, static_cast<uint16_t>(20 + n));
            putBe16(f, 18, ip_id);
            putBe16(f, 20, static_cast<uint16_t>((more ? 0x2000 : 0) | (offset / 8)));
            f[22] = 64;
            f[23] = 17;
            const uint8_t src[4] = { 169, 254, 0, 2 };
            const uint8_t dst[4] = { 169, 254, 0, 1 };
            std::memcpy(&f[26], src, 4);
            std::memcpy(&f[30], dst, 4);
            std::copy(datagram.begin() + offset, datagram.begin() + offset + n, f.begin() + 34);
            frames.push_back(std::move(f));
        }
        return frames;
    }

    // A classic pcap of Ethernet frames; record i is stamped 1000 + i seconds plus `fraction` microseconds, or nanoseconds
    // with `nanoseconds` (the 0xA1B23C4D magic).
    inline void writePcap(
        const fs::path& path, const std::vector<std::vector<uint8_t>>& frames, bool nanoseconds = false, uint32_t fraction = 0)
    {
        std::vector<uint8_t> out(24, 0);
        putLe(out, 0, nanoseconds ? 0xA1B23C4D : 0xA1B2C3D4, 4);
        putLe(out, 4, 2, 2);
        putLe(out, 6, 4, 2);
        putLe(out, 16, 65535, 4);
        putLe(out, 20, 1, 4); // Ethernet
        for (size_t i = 0; i < frames.size(); ++i)
        {
            std::vector<uint8_t> record(16, 0);
            putLe(record, 0, 1000 + i, 4);
            putLe(record, 4, fraction, 4);
            putLe(record, 8, frames[i].size(), 4);
            putLe(record, 12, frames[i].size(), 4);
            out.insert(out.end(), record.begin(), record.end());
            out.insert(out.end(), frames[i].begin(), frames[i].end());
        }
        std::ofstream(path, std::ios::binary).write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
    }

} // namespace pcap_test
