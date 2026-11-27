#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

namespace rosbags
{

// One UDP datagram out of a capture, reassembled when it arrived in IPv4
// fragments. `payload` is valid only during the callback that receives it.
struct UdpDatagram
{
	const uint8_t* payload = nullptr;
	size_t size = 0;
	uint32_t src_ip = 0; // host byte order
	uint32_t dst_ip = 0;
	uint16_t src_port = 0;
	uint16_t dst_port = 0;
	int64_t capture_time_ns = 0; // the pcap record time (Unix epoch) of the last fragment
};

struct PcapUdpStats
{
	uint64_t bytes_read = 0; // record headers and payloads consumed so far (uncompressed), for progress
	uint64_t compressed_bytes_read = 0; // of the file itself; equals bytes_read + 24 for an uncompressed capture
	uint64_t records = 0;
	uint64_t non_udp = 0; // not IPv4, not UDP, or too short for a UDP header
	uint64_t reassembled_datagrams = 0;
	uint64_t dropped_fragments = 0; // IPv4 fragments of a datagram that never completed
};

// Streams the UDP datagrams of a classic libpcap capture (tcpdump, Wireshark
// saved as "pcap", ouster-cli, Hesai's PandarView), optionally zstd-compressed
// as a whole (.pcap.zst, e.g. lidar_recorder chunks). Microsecond and
// nanosecond timestamps, either byte order; Ethernet (with VLAN tags), Linux
// cooked (SLL, SLL2), BSD loopback and raw-IP link types; IPv4 only.
// Fragmented datagrams (a 128-beam lidar packet is far above a 1500-byte MTU)
// are reassembled. pcapng is refused with a conversion hint.
class PcapUdpReader
{
public:
	explicit PcapUdpReader(const std::filesystem::path& path);
	~PcapUdpReader();

	PcapUdpReader(const PcapUdpReader&) = delete;
	PcapUdpReader& operator=(const PcapUdpReader&) = delete;

	bool isOpen() const;
	const std::string& error() const;

	// Hands every UDP datagram to `on_datagram`, in file order. A torn final
	// record (capture killed mid-write) ends the file quietly. Returns false on
	// an I/O, decompression or format error (see error()).
	bool read(const std::function<void(const UdpDatagram&)>& on_datagram);
	// Makes read() return after the datagram being handled; callable from the callback.
	void requestStop();

	const PcapUdpStats& stats() const;

private:
	struct Impl;
	std::unique_ptr<Impl> impl_;
};

} // namespace rosbags
