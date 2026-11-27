// hesai_pcap_to_laz: Hesai pcap captures (.pcap, or zstd-compressed .pcap.zst as lidar_recorder writes them) ->
// mandeye session directory, as mcap_to_laz and ouster_pcap_to_laz write one.
//
// UDP payloads are pushed straight into the Hesai LiDAR SDK's decoder with the lidar's angle correction; points and IMU
// samples come out on the lidar clock and are converted as mandeye_controller's Hesai client does (acceleration in g,
// angular velocity deg/s -> rad/s). A lidar_recorder session directory stands for its lidarNNNN.pcap.zst chunks, and
// its gnssNNNN.nmea files (lidar-clock times, already in the mandeye format) are copied next to the result.

// See rosbags/McapWriter.h: the rosbags headers come before any standard header.
#include "MandeyeSessionChunker.h"
#include "MandeyeSessionWriter.h"
#include "PcapUdpReader.h"

#include "hesai_lidar_sdk.hpp"

#include "console_progress.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using namespace hesai::lidar;

namespace
{
    // lidarNNNN.pcap.zst (or .pcap when there is no .zst of the same chunk) of a lidar_recorder session, in order.
    // Incomplete .part chunks are skipped.
    std::vector<fs::path> sessionChunks(const fs::path& dir)
    {
        std::vector<fs::path> zst, plain;
        for (const auto& e : fs::directory_iterator(dir))
        {
            const std::string n = e.path().filename().string();
            if (n.rfind("lidar", 0) != 0)
                continue;
            if (n.ends_with(".pcap.zst"))
                zst.push_back(e.path());
            else if (n.ends_with(".pcap"))
                plain.push_back(e.path());
            else if (n.ends_with(".part"))
                spdlog::warn("Skipping incomplete chunk {}", n);
        }
        for (const auto& p : plain)
            if (std::find(zst.begin(), zst.end(), fs::path(p.string() + ".zst")) == zst.end())
                zst.push_back(p);
        std::sort(zst.begin(), zst.end());
        return zst;
    }

    void print_usage(const char* argv0)
    {
        spdlog::error("Usage: {} <input.pcap[.zst]> [<input.pcap[.zst]> ...] <output_session_dir> [options]", argv0);
        spdlog::error("  input         a Hesai capture (classic pcap, optionally zstd-compressed as a whole), or a");
        spdlog::error("                lidar_recorder session directory (its lidarNNNN.pcap.zst chunks, in order);");
        spdlog::error("                several inputs are read in the order given into one session");
        spdlog::error("  output_session_dir  written as mandeye lidarNNNN.laz + imuNNNN.csv chunk pairs,");
        spdlog::error("                      ready to open in lidar_odometry_step_1");
        spdlog::error("Options:");
        spdlog::error("  --chunk-seconds <float>  chunk length in seconds (default: 10, mandeye's own");
        spdlog::error("                           cadence; 0 = one single chunk)");
        spdlog::error("  --correction <file>      the lidar's angle correction (default: correction.dat next to");
        spdlog::error("                           the first input)");
        spdlog::error("  --udp-port <n>           point cloud UDP destination port (default: 2368)");
        spdlog::error("");
        spdlog::error("The lidar ring is kept as the LAS extra-bytes attribute 'ring'. No calibration.json or .sn");
        spdlog::error("file is written, so lidar_odometry_step_1 applies no extrinsics to the result.");
    }
} // namespace

int main(const int argc, const char** argv)
{
    std::vector<std::string> positionals;
    double chunk_seconds = 10.0;
    fs::path correction;
    int udp_port = 2368;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        const bool hasValue = i + 1 < argc;
        if (arg == "--chunk-seconds" && hasValue)
        {
            const std::string value = argv[++i];
            try
            {
                chunk_seconds = std::stod(value);
            } catch (const std::exception&)
            {
                spdlog::error("Invalid --chunk-seconds '{}' (expected a number)", value);
                return EXIT_FAILURE;
            }
            if (!std::isfinite(chunk_seconds) || chunk_seconds < 0.0)
            {
                spdlog::error("Invalid --chunk-seconds '{}' (expected >= 0; 0 = one single chunk)", value);
                return EXIT_FAILURE;
            }
        }
        else if (arg == "--correction" && hasValue)
            correction = argv[++i];
        else if (arg == "--udp-port" && hasValue)
        {
            const std::string value = argv[++i];
            try
            {
                udp_port = std::stoi(value);
            } catch (const std::exception&)
            {
                spdlog::error("Invalid --udp-port '{}' (expected a number)", value);
                return EXIT_FAILURE;
            }
        }
        else if (!arg.starts_with("--"))
            positionals.push_back(arg);
        else
        {
            spdlog::error("Unrecognized argument '{}'", arg);
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    // Every positional is an input except the last, the session dir.
    if (positionals.size() < 2)
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    const fs::path session_dir = positionals.back();
    positionals.pop_back();

    // A lidar_recorder session directory stands for its chunks; its metadata and GNSS files live next to them.
    std::vector<fs::path> inputs;
    std::vector<fs::path> recorder_dirs;
    for (const auto& input : positionals)
    {
        if (fs::is_directory(input))
        {
            const auto chunks = sessionChunks(input);
            if (chunks.empty())
            {
                spdlog::error("No lidarNNNN.pcap.zst or .pcap in {}", input);
                return EXIT_FAILURE;
            }
            inputs.insert(inputs.end(), chunks.begin(), chunks.end());
            recorder_dirs.emplace_back(input);
        }
        else if (!fs::exists(input))
        {
            spdlog::error("Input file {} does not exist", input);
            return EXIT_FAILURE;
        }
        else
            inputs.emplace_back(input);
    }
    if (correction.empty())
        correction =
            (fs::is_directory(positionals.front()) ? fs::path(positionals.front()) : inputs.front().parent_path()) / "correction.dat";
    if (!fs::is_regular_file(correction))
    {
        spdlog::error("No angle correction at {} - pass --correction", correction.string());
        return EXIT_FAILURE;
    }

    rosbags::MandeyeSessionWriter writer(session_dir, /*with_rgb=*/false, /*with_ring=*/true);
    if (!writer.isOpen())
    {
        spdlog::error("Cannot write session directory: {}", writer.error());
        return EXIT_FAILURE;
    }
    rosbags::MandeyeSessionChunker chunker(writer, chunk_seconds);
    spdlog::info("Reading {} input(s) with angle correction {}", inputs.size(), correction.string());

    // Both SDK callbacks run on the SDK's single processing thread, so the chunker needs no locking.
    std::atomic<bool> write_ok{ true };
    uint64_t frames = 0;
    std::vector<rosbags::McapPoint> points;
    HesaiLidarSdk<LidarPointXYZICRT> sdk;
    sdk.RegRecvCallback(
        std::function<void(const LidarDecodedFrame<LidarPointXYZICRT>&)>(
            [&](const LidarDecodedFrame<LidarPointXYZICRT>& frame)
            {
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
                if (!points.empty() && write_ok && !chunker.addPoints(points))
                    write_ok = false;
                ++frames;
            }));
    sdk.RegRecvCallback(
        std::function<void(const LidarImuData&)>(
            [&](const LidarImuData& d)
            {
                // SDK units: acceleration in g, angular velocity in deg/s; mandeye stores g and rad/s.
                constexpr double kDegToRad = std::numbers::pi / 180.0;
                rosbags::McapImuSample s;
                s.timestamp = d.timestamp;
                s.acc_x = static_cast<float>(d.imu_accel_x);
                s.acc_y = static_cast<float>(d.imu_accel_y);
                s.acc_z = static_cast<float>(d.imu_accel_z);
                s.gyro_x = static_cast<float>(d.imu_ang_vel_x * kDegToRad);
                s.gyro_y = static_cast<float>(d.imu_ang_vel_y * kDegToRad);
                s.gyro_z = static_cast<float>(d.imu_ang_vel_z * kDegToRad);
                chunker.addImu(s);
            }));

    DriverParam param;
    param.input_param.source_type = DATA_FROM_ROS_PACKET; // packets are pushed in below
    param.input_param.correction_file_path = correction.string();
    param.decoder_param.pcap_play_in_loop = false;
    sdk.Init(param); // returns at once; its init thread takes the first packet to detect the model

    // Reader: pushes the point cloud UDP payloads into the SDK's packet ring, waiting while it is full.
    std::atomic<bool> read_ok{ true };
    uint64_t pushed = 0;
    std::thread reader(
        [&]
        {
            uint64_t input_bytes = 0;
            for (const auto& path : inputs)
            {
                std::error_code ec;
                input_bytes += fs::file_size(path, ec);
            }
            ConsoleProgress progress("Reading pcap (bytes)", input_bytes);
            uint64_t bytes_before = 0;
            for (const auto& path : inputs)
            {
                rosbags::PcapUdpReader pcap(path);
                if (!pcap.isOpen())
                {
                    spdlog::error("Cannot read {}: {}", path.string(), pcap.error());
                    read_ok = false;
                    continue;
                }
                const bool ok = pcap.read(
                    [&](const rosbags::UdpDatagram& d)
                    {
                        progress.update(bytes_before + pcap.stats().compressed_bytes_read);
                        if (d.dst_port != udp_port)
                            return;
                        while (sdk.OriginPacketIsBufferFull())
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        sdk.PushOriginPacketBuffer(d.payload, static_cast<int>(d.size), static_cast<uint64_t>(d.capture_time_ns / 1000));
                        ++pushed;
                    });
                if (!ok)
                {
                    spdlog::warn("{}: {} - kept what was read", path.string(), pcap.error());
                    read_ok = false;
                }
                if (pcap.stats().dropped_fragments > 0)
                    spdlog::warn(
                        "{}: {} IP fragment(s) dropped: their datagram never completed (packet loss during capture)",
                        path.string(),
                        pcap.stats().dropped_fragments);
                bytes_before += pcap.stats().compressed_bytes_read;
            }
            progress.finish(bytes_before);
        });

    sdk.Start(); // blocks until the decoder is initialised
    reader.join();
    if (sdk.lidar_ptr_->GetInitFinish(FailInit))
    {
        spdlog::error("The Hesai SDK failed to initialise (no point packets on UDP port {}?)", udp_port);
        sdk.Stop();
        return EXIT_FAILURE;
    }
    // Let the decoder drain the ring, then give the parser a moment for the last frame.
    while (!sdk.lidar_ptr_->origin_packets_buffer_.empty())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    sdk.Stop();

    if (!write_ok)
    {
        spdlog::error("Failed writing lidar chunks: {}", writer.error());
        return EXIT_FAILURE;
    }
    if (chunker.chunks() == 0)
    {
        spdlog::error("No points decoded from {} packet(s) on UDP port {} - nothing to write", pushed, udp_port);
        return EXIT_FAILURE;
    }
    if (!chunker.finish())
    {
        spdlog::error("Failed writing IMU chunks: {}", writer.error());
        return EXIT_FAILURE;
    }

    spdlog::info(
        "Wrote {} chunk(s) to {}: {} point(s) from {} frame(s), {} IMU sample(s)",
        chunker.chunks(),
        session_dir.string(),
        writer.pointsWritten(),
        frames,
        writer.imuSamplesWritten());
    if (chunker.outOfOrder())
        spdlog::warn("Frames were not in ascending time order - chunk boundaries may not be monotonic");
    // lidar_odometry_step_1's load_data() needs more than two input files and then discards the first chunk's points
    // outright, so a session of one or two chunks contributes little or nothing there.
    if (chunker.chunks() < 3)
        spdlog::warn(
            "Only {} chunk(s): lidar_odometry_step_1 requires more than two files and discards the first chunk's points - "
            "use a smaller --chunk-seconds",
            chunker.chunks());

    // lidar_recorder's GNSS files (gnssNNNN.nmea, lidar-clock times) are already in the mandeye format.
    int nmea = 0;
    for (const auto& dir : recorder_dirs)
    {
        std::error_code ec;
        for (const auto& e : fs::directory_iterator(dir, ec))
        {
            const auto name = e.path().filename().string();
            if (e.is_regular_file() && name.rfind("gnss", 0) == 0 && e.path().extension() == ".nmea")
            {
                fs::copy_file(e.path(), session_dir / name, fs::copy_options::overwrite_existing, ec);
                if (ec)
                    spdlog::warn("Cannot copy {}: {}", e.path().string(), ec.message());
                else
                    ++nmea;
            }
        }
    }
    if (nmea > 0)
        spdlog::info("Copied {} gnssNNNN.nmea file(s) to {}", nmea, session_dir.string());
    return read_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
