// The rosbags headers come first: they re-enable std::byte on MSVC before any standard header (see rosbags/McapWriter.h).
#include "hesai_decoder.h"

#include "PcapUdpReader.h"

#include "driver_param.h"
#include "udp_parser.h"

#include <algorithm>
#include <cstring>
#include <numbers>

namespace fs = std::filesystem;
using namespace hesai::lidar;

struct HesaiDecoder::Impl
{
    fs::path correction;
    bool decode_points = true;
    DriverParam param;
    UdpParser<LidarPointXYZICRT> parser;
    LidarDecodedFrame<LidarPointXYZICRT> frame;
    bool ready = false; // the model-specific parser exists and the correction is loaded
    uint32_t packet_index = 0;
    LidarImuData last_imu;
    std::vector<rosbags::McapPoint> points;

    void prepare()
    {
        parser.setFrameRightMemorySpace(frame);
        parser.SetPcapPlay(param.input_param.source_type);
        parser.SetFrameAzimuth(param.decoder_param.frame_start_azimuth);
        parser.LoadCorrectionFile(correction.string());
        ready = true;
    }

    // As the SDK driver does once a frame is complete: points of all packets packed together, then FrameProcess.
    void emitFrame(HesaiDecoder& owner)
    {
        if (decode_points && frame.points_num == 0)
        {
            uint32_t points_num = 0;
            for (uint32_t i = 0; i < frame.packet_num; ++i)
            {
                if (points_num != i * frame.per_points_num)
                    std::memmove(
                        frame.points + points_num, frame.points + i * frame.per_points_num, frame.getPointSize() * frame.valid_points[i]);
                points_num += frame.valid_points[i];
            }
            frame.points_num = points_num;
        }
        if (decode_points && frame.points_num > 0)
        {
            parser.FrameProcess(frame);
            points.clear();
            points.reserve(frame.points_num);
            for (uint32_t i = 0; i < frame.points_num; ++i)
            {
                const auto& p = frame.points[i];
                if (p.x == 0.f && p.y == 0.f && p.z == 0.f)
                    continue; // a "no return" slot, which the SDK still emits at the origin
                rosbags::McapPoint out;
                out.x = p.x;
                out.y = p.y;
                out.z = p.z;
                out.intensity = static_cast<float>(p.intensity);
                out.ring = p.ring;
                out.timestamp = p.timestamp;
                points.push_back(out);
            }
            if (!points.empty() && owner.onFrame)
                owner.onFrame(std::move(points));
            points = {};
        }
        frame.Update();
        packet_index = 0;
    }

    void emitImu(HesaiDecoder& owner)
    {
        if (!frame.imu_config.flag || last_imu.isSameImuValue(frame.imu_config))
            return;
        last_imu = frame.imu_config;
        if (!owner.onImu)
            return;
        // SDK units: acceleration in g, angular velocity in deg/s; mandeye stores g and rad/s.
        constexpr double kDegToRad = std::numbers::pi / 180.0;
        const LidarImuData& d = frame.imu_config;
        rosbags::McapImuSample s;
        s.timestamp = d.timestamp;
        s.acc_x = static_cast<float>(d.imu_accel_x);
        s.acc_y = static_cast<float>(d.imu_accel_y);
        s.acc_z = static_cast<float>(d.imu_accel_z);
        s.gyro_x = static_cast<float>(d.imu_ang_vel_x * kDegToRad);
        s.gyro_y = static_cast<float>(d.imu_ang_vel_y * kDegToRad);
        s.gyro_z = static_cast<float>(d.imu_ang_vel_z * kDegToRad);
        owner.onImu(s);
    }

    // A packet that is not the one splitting the frame: its points are computed into the frame's next slot.
    void addPacket()
    {
        if (frame.packet_num == packet_index)
            return; // the packet carried no points (e.g. an IMU-only packet)
        if (decode_points)
            parser.ComputeXYZI(frame, packet_index);
        ++packet_index;
    }
};

HesaiDecoder::HesaiDecoder(const fs::path& correction, bool decode_points)
    : impl_(std::make_unique<Impl>())
{
    impl_->correction = correction;
    impl_->decode_points = decode_points;
    impl_->param.input_param.source_type = DATA_FROM_ROS_PACKET;
    impl_->param.input_param.correction_file_path = correction.string();
    impl_->frame.fParam.Init(impl_->param);
}

HesaiDecoder::~HesaiDecoder() = default;

void HesaiDecoder::push(const uint8_t* payload, size_t size)
{
    Impl& d = *impl_;
    if (size == 0 || size > kBufSize)
        return; // not a Hesai point cloud packet (the SDK's packet buffer holds kBufSize bytes)
    const UdpPacket packet(payload, static_cast<uint16_t>(size));

    if (!d.ready)
    {
        // The first packet only picks the parser for the lidar model; then it is decoded like any other, so a decoder
        // per file (one per chunk) loses no packet.
        d.parser.DecodePacket(d.frame, packet);
        if (d.parser.GetGeneralParser() == nullptr)
            return;
        d.prepare();
    }

    const int result = d.parser.DecodePacket(d.frame, packet);
    d.emitImu(*this);
    if (result != 0)
        return;
    if (d.frame.scan_complete)
    {
        d.emitFrame(*this);
        // The packet that completed the frame is the first packet of the next one.
        if (d.parser.DecodePacket(d.frame, packet) == 0)
            d.addPacket();
        return;
    }
    d.addPacket();
}

void HesaiDecoder::finish()
{
    if (impl_->ready && impl_->packet_index > 0)
        impl_->emitFrame(*this);
}

std::string HesaiDecoder::error() const
{
    if (impl_->ready && !impl_->parser.isSetCorrectionSucc())
        return "cannot use the angle correction " + impl_->correction.string();
    return {};
}

std::vector<fs::path> hesai_recorder_chunks(const fs::path& dir)
{
    std::vector<fs::path> zst, plain;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(dir, ec))
    {
        const std::string n = e.path().filename().string();
        if (n.rfind("lidar", 0) != 0)
            continue;
        if (n.ends_with(".pcap.zst"))
            zst.push_back(e.path());
        else if (n.ends_with(".pcap"))
            plain.push_back(e.path());
    }
    for (const auto& p : plain)
        if (std::find(zst.begin(), zst.end(), fs::path(p.string() + ".zst")) == zst.end())
            zst.push_back(p);
    std::sort(zst.begin(), zst.end());
    return zst;
}

bool decode_hesai_capture(
    const fs::path& file, uint16_t udp_port, HesaiDecoder& decoder, std::string& error, const std::function<void(uint64_t)>& on_bytes)
{
    rosbags::PcapUdpReader pcap(file);
    if (!pcap.isOpen())
    {
        error = pcap.error();
        return false;
    }
    const bool ok = pcap.read(
        [&](const rosbags::UdpDatagram& d)
        {
            if (on_bytes)
                on_bytes(pcap.stats().compressed_bytes_read);
            if (d.dst_port == udp_port)
                decoder.push(d.payload, d.size);
        });
    if (!ok)
        error = pcap.error();
    return ok;
}
