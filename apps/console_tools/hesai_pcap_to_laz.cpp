// hesai_pcap_to_laz: Hesai pcap captures (.pcap, or zstd-compressed .pcap.zst as lidar_recorder writes them) ->
// mandeye session directory, as mcap_to_laz and ouster_pcap_to_laz write one.
//
// The packets are decoded by HesaiDecoder (the Hesai LiDAR SDK's parser) with the lidar's angle correction, as
// lidar_odometry_step_1 reads a capture directly; points and IMU samples are on the lidar clock. A lidar_recorder session directory stands
// for its lidarNNNN.pcap.zst chunks, and its gnssNNNN.nmea files (lidar-clock times, already in the mandeye format) are copied next to the
// result.

// See rosbags/McapWriter.h: the rosbags headers come before any standard header.
#include "MandeyeSessionChunker.h"
#include "MandeyeSessionWriter.h"
#include "hesai_decoder.h"

#include "console_progress.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
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
            const auto chunks = hesai_recorder_chunks(input);
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

    bool write_ok = true;
    uint64_t frames = 0;
    HesaiDecoder decoder(correction);
    decoder.onFrame = [&](std::vector<rosbags::McapPoint>&& points)
    {
        if (write_ok && !chunker.addPoints(points))
            write_ok = false;
        ++frames;
    };
    decoder.onImu = [&](const rosbags::McapImuSample& sample)
    {
        chunker.addImu(sample);
    };

    uint64_t input_bytes = 0;
    for (const auto& path : inputs)
    {
        std::error_code ec;
        input_bytes += fs::file_size(path, ec);
    }
    ConsoleProgress progress("Reading pcap (bytes)", input_bytes);
    uint64_t bytes_before = 0;
    bool read_ok = true;
    for (const auto& path : inputs)
    {
        // One decoder for all inputs: they are one continuous recording, so a frame spanning two files stays whole.
        std::string error;
        uint64_t bytes = 0;
        if (!decode_hesai_capture(
                path,
                static_cast<uint16_t>(udp_port),
                decoder,
                error,
                [&](uint64_t read)
                {
                    bytes = read;
                    progress.update(bytes_before + read);
                }))
        {
            spdlog::warn("{}: {} - kept what was read", path.string(), error);
            read_ok = false;
        }
        bytes_before += bytes;
    }
    decoder.finish();
    progress.finish(bytes_before);
    if (!decoder.error().empty())
    {
        spdlog::error("{}", decoder.error());
        return EXIT_FAILURE;
    }

    if (!write_ok)
    {
        spdlog::error("Failed writing lidar chunks: {}", writer.error());
        return EXIT_FAILURE;
    }
    if (chunker.chunks() == 0)
    {
        spdlog::error("No points decoded on UDP port {} - nothing to write", udp_port);
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
