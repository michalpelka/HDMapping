// hesai_pcap_to_laz: a recording -> mandeye session directory, read through the recording sources exactly as
// lidar_odometry_step_1 reads it. Meant for Hesai captures (.pcap / .pcap.zst files or a lidar_recorder session
// directory), but any registered recording kind converts the same way.
//
// Each chunk of the recording becomes one lidarNNNN.laz (all its points, nothing filtered); every IMU sample goes to the
// imuNNNN.csv of the chunk open at its time.

// See rosbags/McapWriter.h: the rosbags headers come before any standard header.
#include "MandeyeSessionWriter.h"

#include "recording_source_factory.h"

#include "console_progress.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
    void print_usage(const char* argv0)
    {
        spdlog::error("Usage: {} <input> [<input> ...] <output_session_dir>", argv0);
        spdlog::error("  input   a recording lidar_odometry_step_1 opens: Hesai .pcap / .pcap.zst files or a");
        spdlog::error("          lidar_recorder session directory (correction.dat next to them), ROS1 bags, MCAP files");
        spdlog::error("  output_session_dir  written as mandeye lidarNNNN.laz + imuNNNN.csv chunk pairs, one per");
        spdlog::error("                      chunk of the recording");
        spdlog::error("No calibration.json or .sn file is written, so lidar_odometry_step_1 applies no extrinsics.");
    }
} // namespace

int main(const int argc, const char** argv)
{
    std::vector<fs::path> paths;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg.starts_with("--"))
        {
            spdlog::error("Unrecognized argument '{}'", arg);
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
        paths.emplace_back(arg);
    }
    if (paths.size() < 2)
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    const fs::path session_dir = paths.back();
    paths.pop_back();

    const auto kind = RecordingSourceFactory::find(paths);
    if (!kind)
    {
        spdlog::error("No recording kind reads '{}'", paths.front().string());
        return EXIT_FAILURE;
    }
    std::string error;
    const auto source = kind->open(paths, error);
    if (!source)
    {
        spdlog::error("Cannot read '{}' as {}: {}", paths.front().string(), kind->name(), error);
        return EXIT_FAILURE;
    }
    spdlog::info("Reading '{}' as {}: {} chunk(s)", paths.front().string(), kind->name(), source->chunkCount());

    rosbags::MandeyeSessionWriter writer(session_dir);
    if (!writer.isOpen())
    {
        spdlog::error("Cannot write session directory: {}", writer.error());
        return EXIT_FAILURE;
    }

    Imu imu;
    std::vector<InputFileSpan> imu_spans;
    constexpr uint64_t kImuSteps = 1000;
    ConsoleProgress imu_progress("Reading IMU", kImuSteps);
    source->loadImu(
        imu,
        imu_spans,
        [&](double fraction)
        {
            imu_progress.update(static_cast<uint64_t>(fraction * kImuSteps));
        });
    imu_progress.finish(imu.size());
    std::sort(
        imu.begin(),
        imu.end(),
        [](const auto& a, const auto& b)
        {
            return std::get<0>(a).first < std::get<0>(b).first;
        });

    // Every point: no filter.
    PointFilter filter;
    filter.omit_timestamp_zero = false;
    filter.xy_inner = -1.0;
    filter.xy_outer = std::numeric_limits<double>::infinity();

    std::vector<double> chunk_start; // first point time of each written chunk
    ConsoleProgress chunk_progress("Writing chunks", source->chunkCount());
    for (size_t i = 0; i < source->chunkCount(); ++i)
    {
        const auto points = source->loadPoints(i, filter);
        chunk_progress.update(i + 1);
        if (points.empty())
            continue;
        std::vector<rosbags::McapPoint> out(points.size());
        for (size_t k = 0; k < points.size(); ++k)
        {
            out[k].x = static_cast<float>(points[k].point.x());
            out[k].y = static_cast<float>(points[k].point.y());
            out[k].z = static_cast<float>(points[k].point.z());
            out[k].intensity = points[k].intensity;
            out[k].timestamp = points[k].timestamp;
        }
        if (!writer.beginChunk(static_cast<int>(chunk_start.size())) || !writer.addPoints(out) || !writer.endChunk())
        {
            spdlog::error("Failed writing {}: {}", source->chunkName(i), writer.error());
            return EXIT_FAILURE;
        }
        chunk_start.push_back(points.front().timestamp);
    }
    chunk_progress.finish(source->chunkCount());
    if (chunk_start.empty())
    {
        spdlog::error("No points in '{}' - nothing to write", paths.front().string());
        return EXIT_FAILURE;
    }

    // Each IMU sample goes to the chunk open at its time; samples before all lidar data to the first chunk.
    std::vector<std::vector<rosbags::McapImuSample>> per_chunk(chunk_start.size());
    for (const auto& [timestamps, gyro, acc] : imu)
    {
        const double t = timestamps.first;
        const auto it = std::upper_bound(chunk_start.begin(), chunk_start.end(), t);
        const size_t index = it == chunk_start.begin() ? 0 : static_cast<size_t>(it - chunk_start.begin() - 1);
        rosbags::McapImuSample s;
        s.timestamp = t;
        s.gyro_x = gyro.x();
        s.gyro_y = gyro.y();
        s.gyro_z = gyro.z();
        s.acc_x = acc.x();
        s.acc_y = acc.y();
        s.acc_z = acc.z();
        per_chunk[index].push_back(s);
    }
    for (size_t k = 0; k < per_chunk.size(); ++k)
    {
        if (!writer.writeImuChunk(static_cast<int>(k), per_chunk[k]))
        {
            spdlog::error("Failed writing IMU chunk {}: {}", k, writer.error());
            return EXIT_FAILURE;
        }
    }

    spdlog::info(
        "Wrote {} chunk(s) to {}: {} point(s), {} IMU sample(s)",
        chunk_start.size(),
        session_dir.string(),
        writer.pointsWritten(),
        writer.imuSamplesWritten());
    return EXIT_SUCCESS;
}
