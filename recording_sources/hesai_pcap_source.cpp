// The rosbags headers come first: they re-enable std::byte on MSVC before any standard header (see rosbags/McapWriter.h).
#include "hesai_decoder.h"

#include "hesai_pcap_source.h"
#include "laz_csv_source.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace fs = std::filesystem;

namespace
{
    bool is_capture(const fs::path& file)
    {
        const std::string name = file.filename().string();
        return name.ends_with(".pcap") || name.ends_with(".pcap.zst");
    }

    //! The captures of paths in recording order: files as given, a folder's lidarNNNN chunks; empty when a path is neither.
    std::vector<fs::path> captures(const std::vector<fs::path>& paths)
    {
        std::vector<fs::path> files;
        for (const auto& path : paths)
        {
            std::error_code ec;
            if (fs::is_regular_file(path, ec) && is_capture(path))
                files.push_back(path);
            else if (fs::is_directory(path, ec))
            {
                const auto chunks = hesai_recorder_chunks(path);
                if (chunks.empty())
                    return {};
                files.insert(files.end(), chunks.begin(), chunks.end());
            }
            else
                return {};
        }
        return files;
    }

    //! The deepest folder that holds every file.
    fs::path common_folder(const std::vector<fs::path>& files)
    {
        if (files.empty())
            return {};
        fs::path common = files.front().parent_path();
        for (const auto& file : files)
        {
            const fs::path folder = file.parent_path();
            fs::path shared;
            for (auto a = common.begin(), b = folder.begin(); a != common.end() && b != folder.end() && *a == *b; ++a, ++b)
                shared /= *a;
            common = shared;
        }
        return common;
    }

    ChunkStamp file_stamp(const fs::path& file)
    {
        std::error_code ec;
        ChunkStamp s;
        s.size = fs::file_size(file, ec);
        if (!ec)
            s.time = fs::last_write_time(file, ec);
        s.valid = !ec;
        return s;
    }
} // namespace

HesaiPcapSource::HesaiPcapSource(std::vector<fs::path> files, fs::path correction, uint16_t udp_port)
    : files_(std::move(files))
    , correction_(std::move(correction))
    , udp_port_(udp_port)
{
}

fs::path HesaiPcapSource::workingDirectory() const
{
    return common_folder(files_);
}

size_t HesaiPcapSource::chunkCount() const
{
    return files_.size();
}

std::string HesaiPcapSource::chunkName(size_t i) const
{
    return files_[i].string();
}

std::vector<Point3Di> HesaiPcapSource::loadPoints(size_t i, const PointFilter& filter) const
{
    std::vector<Point3Di> points;
    size_t total = 0;
    HesaiDecoder decoder(correction_);
    decoder.onFrame = [&](std::vector<rosbags::McapPoint>&& frame)
    {
        total += frame.size();
        for (const auto& p : frame)
        {
            if (filter.omit_timestamp_zero && p.timestamp == 0.0)
                continue;
            const double r = std::sqrt(static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y);
            if (r <= filter.xy_inner || r >= filter.xy_outer)
                continue;
            Point3Di q{};
            q.point = Eigen::Vector3d(p.x, p.y, p.z);
            q.timestamp = p.timestamp;
            q.intensity = p.intensity;
            q.lidarid = 0;
            points.push_back(q);
        }
    };
    std::string error;
    if (!decode_hesai_capture(files_[i], udp_port_, decoder, error))
        std::cerr << "reading '" << chunkName(i) << "' failed: " << error << "\n";
    decoder.finish();
    if (!decoder.error().empty())
        std::cerr << "'" << chunkName(i) << "': " << decoder.error() << "\n";

    std::sort(
        points.begin(),
        points.end(),
        [](const Point3Di& a, const Point3Di& b)
        {
            return a.timestamp < b.timestamp;
        });
    std::cout << chunkName(i) << ": " << total << " - " << total - points.size() << " = " << points.size() << std::endl;
    return points;
}

ChunkStamp HesaiPcapSource::stamp(size_t i) const
{
    return file_stamp(files_[i]);
}

void HesaiPcapSource::loadImu(Imu& imu_data, std::vector<InputFileSpan>& imuSpans, const std::function<void(double)>& progress) const
{
    // The IMU samples travel in the point packets, so every packet is decoded once (without computing the points).
    uint64_t total_bytes = 0;
    for (const auto& file : files_)
    {
        std::error_code ec;
        total_bytes += fs::file_size(file, ec);
    }
    const size_t samples_before = imu_data.size();
    const size_t spans_before = imuSpans.size();
    uint64_t done_bytes = 0;
    // One decoder for every file: they are one continuous recording, and the decoder drops a sample repeated in the
    // next packet, which at a file boundary is the first packet of the next file.
    HesaiDecoder decoder(correction_, /*decode_points=*/false);
    size_t first = 0;
    decoder.onImu = [&](const rosbags::McapImuSample& s)
    {
        imu_data.emplace_back(
            std::pair(s.timestamp, 0.0), Eigen::Vector3f(s.gyro_x, s.gyro_y, s.gyro_z), Eigen::Vector3f(s.acc_x, s.acc_y, s.acc_z));
    };
    for (const auto& file : files_)
    {
        std::cout << "loading IMU data from '" << file.string() << "' ..\n";
        first = imu_data.size();
        std::string error;
        uint64_t file_bytes = 0;
        const bool ok = decode_hesai_capture(
            file,
            udp_port_,
            decoder,
            error,
            [&](uint64_t read)
            {
                file_bytes = read;
                if (progress && total_bytes > 0)
                    progress(static_cast<double>(done_bytes + read) / static_cast<double>(total_bytes));
            });
        if (!ok)
            std::cerr << "reading IMU from '" << file.string() << "' failed: " << error << "\n";
        done_bytes += file_bytes;

        InputFileSpan span{ file.string() };
        for (size_t k = first; k < imu_data.size(); ++k)
        {
            const double t = std::get<0>(imu_data[k]).first;
            span.first = span.count == 0 ? t : std::min(span.first, t);
            span.last = span.count == 0 ? t : std::max(span.last, t);
            span.count++;
        }
        imuSpans.push_back(span);

        // A lidar without an IMU (or one the Mandeye unit does not use): the unit logged the IMU itself, as for a Livox
        // recording, into imu****.csv or an external IMU's MURATA_IMU folder next to the captures.
        if (imu_data.size() == samples_before)
        {
            const fs::path folder = workingDirectory();
            std::vector<std::string> csv_files;
            std::error_code ec;
            for (const auto& e : fs::directory_iterator(folder, ec))
            {
                const std::string n = e.path().filename().string();
                if (e.is_regular_file() && n.rfind("imu", 0) == 0 && e.path().extension() == ".csv")
                    csv_files.push_back(e.path().string());
            }
            std::sort(csv_files.begin(), csv_files.end());
            std::cout << "no IMU data in the lidar packets of '" << file.string() << "', loading the IMU files of '" << folder.string()
                      << "' ..\n";
            imuSpans.resize(spans_before);
            load_mandeye_imu(csv_files, folder, 0, imu_data, imuSpans);
            if (progress)
                progress(1.0);
            return;
        }
    }
}

std::string HesaiPcapSource::name()
{
    return "Hesai pcap";
}

int HesaiPcapSource::priority()
{
    return 10;
}

bool HesaiPcapSource::accepts(const std::vector<fs::path>& paths)
{
    return !captures(paths).empty();
}

std::shared_ptr<RecordingSource> HesaiPcapSource::open(const std::vector<fs::path>& paths, std::string& error)
{
    error.clear();
    auto files = captures(paths);
    if (files.empty())
    {
        error = "no .pcap or .pcap.zst file";
        return nullptr;
    }
    std::error_code ec;
    const fs::path folder = !paths.empty() && fs::is_directory(paths.front(), ec) ? paths.front() : files.front().parent_path();
    const fs::path correction = folder / "correction.dat";
    if (!fs::is_regular_file(correction, ec))
    {
        error = "no angle correction (correction.dat) in " + folder.string();
        return nullptr;
    }
    return std::make_shared<HesaiPcapSource>(std::move(files), correction);
}

std::vector<std::string> HesaiPcapSource::fileFilter()
{
    return { "Hesai capture (*.pcap *.pcap.zst)", "*.pcap *.zst" };
}

HDMAPPING_REGISTER_RECORDING_SOURCE(HesaiPcapSource)
