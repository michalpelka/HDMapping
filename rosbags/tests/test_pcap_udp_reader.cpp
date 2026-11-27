// See rosbags/McapWriter.h for why this needs to come before any standard
// header (including <doctest.h>) gets a chance to lock in the project-wide
// _HAS_STD_BYTE=0 MSVC workaround.
#include "McapWriter.h"

// doctest's main comes from test_mcap_writer.cpp, linked into the same binary.
#include <doctest.h>

#include "PcapUdpReader.h"
#include "pcap_test_frames.h"

#include <zstd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

    struct Datagram
    {
        uint16_t dst_port = 0;
        std::vector<uint8_t> payload;
        int64_t capture_time_ns = 0;
    };

    std::vector<Datagram> readAll(const fs::path& path, rosbags::PcapUdpStats& stats)
    {
        rosbags::PcapUdpReader reader(path);
        INFO(reader.error());
        REQUIRE(reader.isOpen());
        std::vector<Datagram> out;
        REQUIRE(reader.read(
            [&](const rosbags::UdpDatagram& d)
            {
                out.push_back({ d.dst_port, std::vector<uint8_t>(d.payload, d.payload + d.size), d.capture_time_ns });
            }));
        stats = reader.stats();
        return out;
    }

    std::vector<uint8_t> payload(size_t size, uint8_t seed)
    {
        std::vector<uint8_t> p(size);
        for (size_t i = 0; i < size; ++i)
            p[i] = static_cast<uint8_t>(seed + i * 7);
        return p;
    }

    // Two datagrams, the second in three IPv4 fragments, and an ARP frame between them.
    std::vector<std::vector<uint8_t>> sampleFrames()
    {
        auto frames = pcap_test::udpFrames(payload(100, 1), 2368, 1);
        std::vector<uint8_t> arp(42, 0);
        pcap_test::putBe16(arp, 12, 0x0806);
        frames.push_back(arp);
        const auto fragmented = pcap_test::udpFrames(payload(1000, 2), 10110, 2, 400);
        frames.insert(frames.end(), fragmented.begin(), fragmented.end());
        return frames;
    }

} // namespace

TEST_CASE("PcapUdpReader: datagrams, ports and capture times; fragments reassembled, other frames counted")
{
    const fs::path path = fs::temp_directory_path() / "pcap_udp_reader_plain.pcap";
    pcap_test::writePcap(path, sampleFrames(), false, 250);

    rosbags::PcapUdpStats stats;
    const auto datagrams = readAll(path, stats);
    REQUIRE(datagrams.size() == 2);
    CHECK(datagrams[0].dst_port == 2368);
    CHECK(datagrams[0].payload == payload(100, 1));
    CHECK(datagrams[0].capture_time_ns == 1000LL * 1000000000 + 250000); // record 0, 250 us
    CHECK(datagrams[1].dst_port == 10110);
    CHECK(datagrams[1].payload == payload(1000, 2));
    CHECK(datagrams[1].capture_time_ns == 1004LL * 1000000000 + 250000); // the last fragment is record 4
    CHECK(stats.records == 5);
    CHECK(stats.non_udp == 1);
    CHECK(stats.reassembled_datagrams == 1);
    CHECK(stats.dropped_fragments == 0);
    fs::remove(path);
}

TEST_CASE("PcapUdpReader: nanosecond timestamps")
{
    const fs::path path = fs::temp_directory_path() / "pcap_udp_reader_ns.pcap";
    pcap_test::writePcap(path, pcap_test::udpFrames(payload(10, 3), 2368, 1), true, 123);

    rosbags::PcapUdpStats stats;
    const auto datagrams = readAll(path, stats);
    REQUIRE(datagrams.size() == 1);
    CHECK(datagrams[0].capture_time_ns == 1000LL * 1000000000 + 123);
    fs::remove(path);
}

TEST_CASE("PcapUdpReader: a zstd-compressed capture reads the same as the plain one")
{
    const fs::path plain = fs::temp_directory_path() / "pcap_udp_reader_zst.pcap";
    const fs::path compressed = fs::temp_directory_path() / "pcap_udp_reader_zst.pcap.zst";
    pcap_test::writePcap(plain, sampleFrames());
    std::ifstream in(plain, std::ios::binary);
    const std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::vector<char> packed(ZSTD_compressBound(bytes.size()));
    const size_t packed_size = ZSTD_compress(packed.data(), packed.size(), bytes.data(), bytes.size(), 3);
    REQUIRE_FALSE(ZSTD_isError(packed_size));
    std::ofstream(compressed, std::ios::binary).write(packed.data(), static_cast<std::streamsize>(packed_size));

    rosbags::PcapUdpStats plain_stats, compressed_stats;
    const auto expected = readAll(plain, plain_stats);
    const auto actual = readAll(compressed, compressed_stats);
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i)
    {
        CHECK(actual[i].dst_port == expected[i].dst_port);
        CHECK(actual[i].payload == expected[i].payload);
        CHECK(actual[i].capture_time_ns == expected[i].capture_time_ns);
    }
    CHECK(compressed_stats.records == plain_stats.records);
    CHECK(compressed_stats.bytes_read == plain_stats.bytes_read);
    CHECK(compressed_stats.compressed_bytes_read == packed_size);
    fs::remove(plain);
    fs::remove(compressed);
}

TEST_CASE("PcapUdpReader: pcapng is refused with a conversion hint")
{
    const fs::path path = fs::temp_directory_path() / "pcap_udp_reader.pcapng";
    std::vector<uint8_t> block(28, 0);
    pcap_test::putLe(block, 0, 0x0A0D0D0A, 4);
    std::ofstream(path, std::ios::binary).write(reinterpret_cast<const char*>(block.data()), static_cast<std::streamsize>(block.size()));
    rosbags::PcapUdpReader reader(path);
    CHECK_FALSE(reader.isOpen());
    CHECK(reader.error().find("editcap") != std::string::npos);
    fs::remove(path);
}
