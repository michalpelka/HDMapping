#include "PcapUdpReader.h"

#include <zstd.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <map>
#include <tuple>
#include <vector>

namespace rosbags
{

namespace
{

// pcap link-layer types handled by ipOffset().
constexpr uint32_t kLinkNull = 0;
constexpr uint32_t kLinkEthernet = 1;
constexpr uint32_t kLinkRaw = 101;
constexpr uint32_t kLinkLoop = 108;
constexpr uint32_t kLinkLinuxSll = 113;
constexpr uint32_t kLinkIpv4 = 228;
constexpr uint32_t kLinkLinuxSll2 = 276;

// Larger records are not plausible captures of a sensor (and guard the buffer
// against a corrupt length field).
constexpr uint32_t kMaxRecordBytes = 256 * 1024;

// Datagrams waiting for their missing fragments. A sensor interleaves at most a
// lidar and an IMU datagram, so anything beyond this is fragments whose
// datagram will never complete (packet loss): the oldest is dropped.
constexpr size_t kMaxPendingDatagrams = 64;

constexpr uint32_t kZstdMagic = 0xFD2FB528;

uint16_t be16(const uint8_t* p)
{
	return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

uint32_t be32(const uint8_t* p)
{
	return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) | (static_cast<uint32_t>(p[2]) << 8) | p[3];
}

uint32_t swap32(uint32_t v)
{
	return (v >> 24) | ((v >> 8) & 0xFF00) | ((v << 8) & 0xFF0000) | (v << 24);
}

} // namespace

struct PcapUdpReader::Impl
{
	PcapUdpStats stats;
	std::string error;
	bool open = false;
	bool stop = false;

	std::ifstream in;
	std::vector<char> file_buffer;

	// zstd-compressed capture: compressed input and decompressed output windows.
	ZSTD_DStream* zstd = nullptr;
	std::vector<uint8_t> zin, zout;
	ZSTD_inBuffer zin_view{nullptr, 0, 0};
	size_t zout_pos = 0, zout_size = 0;
	bool zstd_done = false;

	bool swapped = false;
	bool nanoseconds = false;
	uint32_t linktype = 0;

	struct PendingDatagram
	{
		std::map<uint32_t, std::vector<uint8_t>> parts; // byte offset -> payload
		uint32_t total = 0; // known once the last fragment arrives
		uint64_t age = 0;
	};
	std::map<std::tuple<uint32_t, uint32_t, uint16_t>, PendingDatagram> pending;
	uint64_t pending_clock = 0;

	std::vector<uint8_t> record;
	std::vector<uint8_t> datagram;
	std::function<void(const UdpDatagram&)> on_datagram;

	~Impl()
	{
		if(zstd)
			ZSTD_freeDStream(zstd);
	}

	bool fail(std::string message)
	{
		if(error.empty())
			error = std::move(message);
		// Release the file: a rejected capture must not stay locked (Windows) for the reader's lifetime.
		in.close();
		open = false;
		return false;
	}

	// Reads exactly n bytes of the (decompressed) capture. False at the end of
	// the data, or on a decompression error (which also sets `error`).
	bool readBytes(void* dst, size_t n)
	{
		if(!zstd)
		{
			if(!in.read(static_cast<char*>(dst), static_cast<std::streamsize>(n)))
				return false;
			stats.compressed_bytes_read += n;
			return true;
		}
		auto* out = static_cast<uint8_t*>(dst);
		while(n > 0)
		{
			if(zout_pos == zout_size)
			{
				if(!refillZstd())
					return false;
				continue;
			}
			const size_t take = std::min(n, zout_size - zout_pos);
			std::memcpy(out, zout.data() + zout_pos, take);
			zout_pos += take;
			out += take;
			n -= take;
		}
		return true;
	}

	// Decompresses the next window of output. False at the end of the stream or on an error.
	bool refillZstd()
	{
		while(true)
		{
			if(zin_view.pos == zin_view.size)
			{
				if(zstd_done)
					return false;
				in.read(reinterpret_cast<char*>(zin.data()), static_cast<std::streamsize>(zin.size()));
				const auto got = static_cast<size_t>(in.gcount());
				if(got == 0)
				{
					zstd_done = true;
					return false;
				}
				stats.compressed_bytes_read += got;
				zin_view = {zin.data(), got, 0};
			}
			ZSTD_outBuffer out{zout.data(), zout.size(), 0};
			const size_t rc = ZSTD_decompressStream(zstd, &out, &zin_view);
			if(ZSTD_isError(rc))
			{
				error = std::string("zstd: ") + ZSTD_getErrorName(rc);
				zstd_done = true;
				return false;
			}
			if(out.pos > 0)
			{
				zout_pos = 0;
				zout_size = out.pos;
				return true;
			}
		}
	}

	uint32_t u32(const uint8_t* p) const
	{
		uint32_t v;
		std::memcpy(&v, p, sizeof(v));
		return swapped ? swap32(v) : v;
	}

	// Locates the IPv4 header inside a captured frame. Returns false for anything
	// that is not IPv4 (ARP, IPv6, ...).
	bool ipOffset(const uint8_t* frame, size_t size, size_t& offset) const
	{
		switch(linktype)
		{
		case kLinkEthernet:
		{
			size_t off = 12;
			if(size < off + 2)
				return false;
			uint16_t ethertype = be16(frame + off);
			while(ethertype == 0x8100 || ethertype == 0x88A8 || ethertype == 0x9100) // VLAN / QinQ tags
			{
				off += 4;
				if(size < off + 2)
					return false;
				ethertype = be16(frame + off);
			}
			offset = off + 2;
			return ethertype == 0x0800;
		}
		case kLinkLinuxSll:
			offset = 16;
			return size >= 16 && be16(frame + 14) == 0x0800;
		case kLinkLinuxSll2:
			offset = 20;
			return size >= 20 && be16(frame) == 0x0800;
		case kLinkNull:
		case kLinkLoop:
			offset = 4;
			return size >= 4 && (u32(frame) == 2 || swap32(u32(frame)) == 2); // AF_INET, host order of the capturing machine
		default: // kLinkRaw, kLinkIpv4
			offset = 0;
			return size >= 1 && (frame[0] >> 4) == 4;
		}
	}

	void handleUdp(const uint8_t* ip, const uint8_t* udp, size_t size, int64_t time_ns)
	{
		if(size < 8)
		{
			++stats.non_udp;
			return;
		}
		UdpDatagram d;
		d.src_ip = be32(ip + 12);
		d.dst_ip = be32(ip + 16);
		d.src_port = be16(udp);
		d.dst_port = be16(udp + 2);
		d.size = std::min<size_t>(std::max<size_t>(be16(udp + 4), 8), size) - 8;
		d.payload = udp + 8;
		d.capture_time_ns = time_ns;
		if(on_datagram)
			on_datagram(d);
	}

	// Feeds one IPv4 fragment in; once every byte of its datagram has arrived,
	// hands the reassembled UDP datagram on and forgets it.
	void handleFragment(const uint8_t* ip, size_t ihl, size_t ip_length, int64_t time_ns)
	{
		const auto key = std::make_tuple(be32(ip + 12), be32(ip + 16), be16(ip + 4));
		const uint16_t flags_offset = be16(ip + 6);
		const bool more_fragments = (flags_offset & 0x2000) != 0;
		const uint32_t offset = static_cast<uint32_t>(flags_offset & 0x1FFF) * 8;
		const size_t length = ip_length - ihl;

		auto [it, inserted] = pending.try_emplace(key);
		PendingDatagram& d = it->second;
		if(inserted)
			d.age = pending_clock++;
		d.parts[offset].assign(ip + ihl, ip + ihl + length);
		if(!more_fragments)
			d.total = offset + static_cast<uint32_t>(length);

		if(d.total != 0)
		{
			uint32_t covered = 0;
			for(const auto& [part_offset, bytes] : d.parts)
			{
				if(part_offset > covered)
					break; // hole
				covered = std::max<uint32_t>(covered, part_offset + static_cast<uint32_t>(bytes.size()));
			}
			if(covered >= d.total)
			{
				datagram.assign(d.total, 0);
				for(const auto& [part_offset, bytes] : d.parts)
				{
					if(part_offset >= d.total)
						break;
					std::memcpy(datagram.data() + part_offset, bytes.data(), std::min<size_t>(bytes.size(), d.total - part_offset));
				}
				pending.erase(it);
				++stats.reassembled_datagrams;
				handleUdp(ip, datagram.data(), datagram.size(), time_ns);
				return;
			}
		}

		if(pending.size() > kMaxPendingDatagrams)
		{
			auto oldest = std::min_element(
				pending.begin(),
				pending.end(),
				[](const auto& a, const auto& b)
				{
					return a.second.age < b.second.age;
				});
			stats.dropped_fragments += oldest->second.parts.size();
			pending.erase(oldest);
		}
	}

	void handleFrame(const uint8_t* frame, size_t size, int64_t time_ns)
	{
		size_t off = 0;
		if(!ipOffset(frame, size, off) || size < off + 20)
		{
			++stats.non_udp;
			return;
		}
		const uint8_t* ip = frame + off;
		const size_t ihl = static_cast<size_t>(ip[0] & 0x0F) * 4;
		const size_t ip_length = std::min<size_t>(be16(ip + 2), size - off);
		if((ip[0] >> 4) != 4 || ihl < 20 || ip_length < ihl || ip[9] != 17 /* UDP */)
		{
			++stats.non_udp;
			return;
		}
		if((be16(ip + 6) & 0x3FFF) != 0) // more-fragments flag or a non-zero offset
			handleFragment(ip, ihl, ip_length, time_ns);
		else
			handleUdp(ip, ip + ihl, ip_length - ihl, time_ns);
	}
};

PcapUdpReader::PcapUdpReader(const std::filesystem::path& path)
	: impl_(std::make_unique<Impl>())
{
	Impl& d = *impl_;
	d.file_buffer.resize(1 << 20);
	d.in.rdbuf()->pubsetbuf(d.file_buffer.data(), static_cast<std::streamsize>(d.file_buffer.size()));
	d.in.open(path, std::ios::binary);
	if(!d.in.is_open())
	{
		d.fail("cannot open " + path.string());
		return;
	}

	uint8_t magic_bytes[4];
	if(!d.in.read(reinterpret_cast<char*>(magic_bytes), sizeof(magic_bytes)))
	{
		d.fail(path.string() + " is too short to be a pcap file");
		return;
	}
	uint32_t file_magic;
	std::memcpy(&file_magic, magic_bytes, sizeof(file_magic));
	d.in.seekg(0);
	if(file_magic == kZstdMagic)
	{
		d.zstd = ZSTD_createDStream();
		if(!d.zstd || ZSTD_isError(ZSTD_initDStream(d.zstd)))
		{
			d.fail("cannot create a zstd decompressor for " + path.string());
			return;
		}
		d.zin.resize(ZSTD_DStreamInSize());
		d.zout.resize(ZSTD_DStreamOutSize());
	}

	uint8_t header[24];
	if(!d.readBytes(header, sizeof(header)))
	{
		d.fail(d.error.empty() ? path.string() + " is too short to be a pcap file" : path.string() + ": " + d.error);
		return;
	}
	uint32_t magic;
	std::memcpy(&magic, header, sizeof(magic));
	switch(magic)
	{
	case 0xA1B2C3D4:
		break;
	case 0xA1B23C4D:
		d.nanoseconds = true;
		break;
	case 0xD4C3B2A1:
		d.swapped = true;
		break;
	case 0x4D3CB2A1:
		d.swapped = d.nanoseconds = true;
		break;
	case 0x0A0D0D0A:
		d.fail(path.string() + " is pcapng, which is not supported -- convert it with: editcap -F pcap in.pcapng out.pcap");
		return;
	default:
		d.fail(path.string() + " is not a pcap file");
		return;
	}
	d.linktype = d.u32(header + 20) & 0x0FFFFFFF; // upper bits: FCS length flags
	switch(d.linktype)
	{
	case kLinkNull:
	case kLinkEthernet:
	case kLinkRaw:
	case kLinkLoop:
	case kLinkLinuxSll:
	case kLinkIpv4:
	case kLinkLinuxSll2:
		break;
	default:
		d.fail("unsupported pcap link type " + std::to_string(d.linktype));
		return;
	}

	d.open = true;
}

PcapUdpReader::~PcapUdpReader() = default;

bool PcapUdpReader::isOpen() const
{
	return impl_->open;
}

const std::string& PcapUdpReader::error() const
{
	return impl_->error;
}

void PcapUdpReader::requestStop()
{
	impl_->stop = true;
}

const PcapUdpStats& PcapUdpReader::stats() const
{
	return impl_->stats;
}

bool PcapUdpReader::read(const std::function<void(const UdpDatagram&)>& on_datagram)
{
	Impl& d = *impl_;
	if(!d.open)
		return false;
	d.on_datagram = on_datagram;

	uint8_t record_header[16];
	while(!d.stop)
	{
		if(!d.readBytes(record_header, sizeof(record_header)))
			break; // end of file; a torn final header (capture killed mid-write) ends it too
		const uint32_t captured = d.u32(record_header + 8);
		if(captured > kMaxRecordBytes)
			return d.fail("corrupt pcap record #" + std::to_string(d.stats.records + 1) + ": " + std::to_string(captured) + " bytes");
		d.record.resize(captured);
		if(!d.readBytes(d.record.data(), captured))
			break; // torn final record
		++d.stats.records;
		d.stats.bytes_read += sizeof(record_header) + captured;
		const int64_t seconds = d.u32(record_header);
		const int64_t fraction = d.u32(record_header + 4);
		const int64_t time_ns = seconds * 1000000000 + (d.nanoseconds ? fraction : fraction * 1000);
		d.handleFrame(d.record.data(), d.record.size(), time_ns);
	}
	for(const auto& [key, datagram] : d.pending)
		d.stats.dropped_fragments += datagram.parts.size();
	d.pending.clear();
	if(!d.error.empty()) // a decompression error
		return d.fail(d.error);
	return true;
}

} // namespace rosbags
