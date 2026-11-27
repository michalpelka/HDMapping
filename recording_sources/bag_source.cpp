// The rosbags headers come first: they re-enable std::byte on MSVC before any standard header (see rosbags/McapWriter.h).
#include "ImuTransform.h"
#include "McapReader.h"
#include "Ros1Bag.h"

#include "bag_source.h"
#include "bag_source_ui.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string_view>

namespace fs = std::filesystem;

namespace
{
    std::string lower_extension(const fs::path& file)
    {
        std::string ext = file.extension().string();
        std::transform(
            ext.begin(),
            ext.end(),
            ext.begin(),
            [](unsigned char c)
            {
                return static_cast<char>(std::tolower(c));
            });
        return ext;
    }

    bool is_ros1_bag(const fs::path& file)
    {
        return lower_extension(file) == ".bag";
    }

    //! True for the files open_bag_source reads: .bag (ROS1) and .mcap.
    bool is_bag_file(const fs::path& file)
    {
        const std::string ext = lower_extension(file);
        return ext == ".bag" || ext == ".mcap";
    }

    //! True when folder holds Mandeye LAZ/LAS files at its top level: such a folder is a Mandeye recording even when an
    //! exported bag sits below it.
    bool has_mandeye_files(const fs::path& folder)
    {
        std::error_code ec;
        for (fs::directory_iterator it(folder, fs::directory_options::skip_permission_denied, ec), end; !ec && it != end; it.increment(ec))
        {
            const std::string ext = lower_extension(it->path());
            if ((ext == ".laz" || ext == ".las") && it->is_regular_file(ec))
                return true;
        }
        return false;
    }

    //! Files with this lower-case extension among paths: the files themselves and every one below a folder (a folder
    //! with Mandeye files at its top level holds none), sorted by path. Empty when a path holds none.
    std::vector<fs::path> files_with_extension(const std::vector<fs::path>& paths, std::string_view extension)
    {
        std::vector<fs::path> files;
        for (const auto& path : paths)
        {
            std::error_code ec;
            const size_t before = files.size();
            if (fs::is_regular_file(path, ec))
            {
                if (lower_extension(path) == extension)
                    files.push_back(path);
            }
            else if (fs::is_directory(path, ec) && !has_mandeye_files(path))
            {
                for (fs::recursive_directory_iterator it(path, fs::directory_options::skip_permission_denied, ec), end; !ec && it != end;
                     it.increment(ec))
                {
                    if (lower_extension(it->path()) == extension && it->is_regular_file(ec))
                        files.push_back(it->path());
                }
            }
            if (files.size() == before)
                return {};
        }
        std::sort(files.begin(), files.end());
        files.erase(std::unique(files.begin(), files.end()), files.end());
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

    //! Message type without its package, as both readers compare it: "sensor_msgs/msg/Imu" -> "Imu".
    std::string_view type_name(std::string_view type)
    {
        const auto slash = type.rfind('/');
        return slash == std::string_view::npos ? type : type.substr(slash + 1);
    }

    rosbags::McapReaderOptions reader_options(const BagSourceConfig& config)
    {
        rosbags::McapReaderOptions options;
        options.lidar_topic = config.lidar_topic;
        options.imu_topic = config.imu_topic;
        switch (config.lidar_layout)
        {
        case BagSourceConfig::LidarLayout::Auto:
            options.lidar_preset = rosbags::Pc2Preset::Auto;
            break;
        case BagSourceConfig::LidarLayout::Hesai:
            options.lidar_preset = rosbags::Pc2Preset::Hesai;
            break;
        case BagSourceConfig::LidarLayout::Ouster:
            options.lidar_preset = rosbags::Pc2Preset::Ouster;
            break;
        }
        return options;
    }

    rosbags::ImuTransform imu_transform(const BagSourceConfig& config)
    {
        rosbags::ImuTransform t;
        const auto& q = config.imu_to_lidar; // qx qy qz qw
        t.imu_to_lidar = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
        t.acc_scale = config.imu_acc_in_g ? 1.0 : 1.0 / rosbags::kStandardGravity;
        return t;
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

template<typename Reader>
BagSource<Reader>::BagSource(const BagSourceConfig& config, const std::vector<std::pair<uint64_t, uint64_t>>& spans)
    : config_(config)
{
    for (size_t f = 0; f < spans.size(); ++f)
    {
        const auto [start_ns, end_ns] = spans[f];
        const uint64_t length = end_ns - start_ns;
        uint64_t chunk_ns = config_.chunk_seconds > 0.0 ? static_cast<uint64_t>(config_.chunk_seconds * 1e9) : length;
        chunk_ns = std::max<uint64_t>(chunk_ns, 1);
        const size_t count = std::max<size_t>(1, static_cast<size_t>((length + chunk_ns - 1) / chunk_ns));
        for (size_t i = 0; i < count; ++i)
        {
            const uint64_t window_start = start_ns + i * chunk_ns;
            windows_.push_back({ f, i, window_start, i + 1 == count ? end_ns : window_start + chunk_ns });
        }
    }
}

template<typename Reader>
fs::path BagSource<Reader>::workingDirectory() const
{
    return common_folder(config_.files);
}

template<typename Reader>
size_t BagSource<Reader>::chunkCount() const
{
    return windows_.size();
}

template<typename Reader>
std::string BagSource<Reader>::chunkName(size_t i) const
{
    const Window& w = windows_[i];
    char index[16];
    std::snprintf(index, sizeof(index), "#%04zu", w.index);
    return config_.files[w.file].string() + index;
}

template<typename Reader>
std::vector<Point3Di> BagSource<Reader>::loadPoints(size_t i, const PointFilter& filter) const
{
    const Window& w = windows_[i];
    const fs::path& file = config_.files[w.file];
    std::vector<Point3Di> points;
    Reader reader(file, reader_options(config_));
    if (!reader.isOpen())
    {
        std::cerr << "cannot open '" << file.string() << "': " << reader.error() << "\n";
        return points;
    }

    size_t total = 0;
    typename Reader::Callbacks callbacks;
    callbacks.onPointCloud = [&](uint64_t, std::vector<rosbags::McapPoint>&& cloud)
    {
        total += cloud.size();
        for (const auto& p : cloud)
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
    rosbags::LogTimeRange range;
    range.start_ns = w.start_ns;
    range.end_ns = w.end_ns;
    if (!reader.read(callbacks, range))
        std::cerr << "reading '" << chunkName(i) << "' failed: " << reader.error() << "\n";
    if (!reader.lidarLayoutWarning().empty())
        std::cerr << "'" << file.string() << "': " << reader.lidarLayoutWarning() << "\n";

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

template<typename Reader>
ChunkStamp BagSource<Reader>::stamp(size_t i) const
{
    return file_stamp(config_.files[windows_[i].file]);
}

template<typename Reader>
void BagSource<Reader>::loadImu(Imu& imu_data, std::vector<InputFileSpan>& imuSpans) const
{
    const rosbags::ImuTransform transform = imu_transform(config_);
    for (const auto& file : config_.files)
    {
        std::cout << "loading IMU data from '" << file.string() << "' ..\n";
        Reader reader(file, reader_options(config_));
        if (!reader.isOpen())
        {
            std::cerr << "cannot open '" << file.string() << "': " << reader.error() << "\n";
            continue;
        }
        const size_t first = imu_data.size();
        typename Reader::Callbacks callbacks;
        callbacks.onImu = [&](rosbags::McapImuSample s)
        {
            rosbags::applyImuTransform(transform, s);
            imu_data.emplace_back(
                std::pair(s.timestamp, 0.0), Eigen::Vector3f(s.gyro_x, s.gyro_y, s.gyro_z), Eigen::Vector3f(s.acc_x, s.acc_y, s.acc_z));
        };
        if (!reader.read(callbacks))
            std::cerr << "reading IMU from '" << file.string() << "' failed: " << reader.error() << "\n";

        InputFileSpan span{ file.string() };
        for (size_t k = first; k < imu_data.size(); ++k)
        {
            const double t = std::get<0>(imu_data[k]).first;
            span.first = span.count == 0 ? t : std::min(span.first, t);
            span.last = span.count == 0 ? t : std::max(span.last, t);
            span.count++;
        }
        imuSpans.push_back(span);
    }
}

template<>
std::string BagSource<rosbags::Ros1BagFileReader>::name()
{
    return "ROS1 bag";
}

template<>
std::string BagSource<rosbags::McapFileReader>::name()
{
    return "MCAP";
}

template<>
std::vector<fs::path> BagSource<rosbags::Ros1BagFileReader>::files(const std::vector<fs::path>& paths)
{
    return files_with_extension(paths, ".bag");
}

template<>
std::vector<fs::path> BagSource<rosbags::McapFileReader>::files(const std::vector<fs::path>& paths)
{
    return files_with_extension(paths, ".mcap");
}

template<typename Reader>
bool BagSource<Reader>::accepts(const std::vector<fs::path>& paths)
{
    return !files(paths).empty();
}

template<>
std::vector<std::string> BagSource<rosbags::Ros1BagFileReader>::fileFilter()
{
    return { "ROS1 bag (*.bag)", "*.bag" };
}

template<>
std::vector<std::string> BagSource<rosbags::McapFileReader>::fileFilter()
{
    return { "MCAP (*.mcap)", "*.mcap" };
}

template<typename Reader>
int BagSource<Reader>::priority()
{
    return 10;
}

namespace
{

    template<typename Reader>
    std::shared_ptr<RecordingSource> open_with(BagSourceConfig config, std::string& error)
    {
        struct FileSpan
        {
            fs::path file;
            uint64_t start_ns = 0;
            uint64_t end_ns = 0;
        };
        std::vector<FileSpan> spans;
        for (const auto& file : config.files)
        {
            const std::string where = "'" + file.string() + "': ";
            Reader reader(file, reader_options(config));
            if (!reader.isOpen())
            {
                error = where + reader.error();
                return nullptr;
            }
            if (reader.topics().lidar.empty())
            {
                error = where + "no lidar topic (sensor_msgs/PointCloud2 or livox_ros_driver/CustomMsg) in the file";
                return nullptr;
            }
            if (reader.topics().imu.empty())
            {
                error = where + "no IMU topic (sensor_msgs/Imu) in the file";
                return nullptr;
            }
            const auto span = reader.logTimeSpan();
            if (!span)
            {
                error = where +
                    "the file has no index, so its time windows cannot be read on their own (reindex it: `rosbag "
                    "reindex` for a ROS1 bag, `mcap recover` for an MCAP file)";
                return nullptr;
            }
            spans.push_back({ file, span->start_ns, span->end_ns });
        }

        std::stable_sort(
            spans.begin(),
            spans.end(),
            [](const FileSpan& a, const FileSpan& b)
            {
                return a.start_ns < b.start_ns;
            });
        // end_ns is one past the last message: a file may start at the time the previous one ended
        for (size_t k = 1; k < spans.size(); ++k)
        {
            if (spans[k].start_ns + 1 < spans[k - 1].end_ns)
            {
                error = "'" + spans[k - 1].file.string() + "' and '" + spans[k].file.string() +
                    "' overlap in log time; only files recorded one after another form one recording";
                return nullptr;
            }
        }

        config.files.clear();
        std::vector<std::pair<uint64_t, uint64_t>> windows;
        for (const auto& span : spans)
        {
            config.files.push_back(span.file);
            windows.emplace_back(span.start_ns, span.end_ns);
        }
        return std::make_shared<BagSource<Reader>>(config, windows);
    }

    template<typename Reader>
    std::vector<BagChannel> channels_of(const fs::path& file, std::string& error)
    {
        Reader reader(file);
        std::vector<BagChannel> channels;
        for (const auto& c : reader.channels())
        {
            BagChannel channel{ c.topic, c.schema_name, c.message_count, BagChannel::Kind::Other };
            const auto type = type_name(c.schema_name);
            if (type == "PointCloud2" || type == "CustomMsg")
                channel.kind = BagChannel::Kind::Lidar;
            else if (type == "Imu")
                channel.kind = BagChannel::Kind::Imu;
            channels.push_back(std::move(channel));
        }
        // a reader that found channels but no unique lidar/IMU topic still lists them; only an unreadable file is an error
        if (channels.empty())
            error = reader.error().empty() ? "the file has no topics" : reader.error();
        return channels;
    }
} // namespace

std::vector<BagChannel> list_bag_channels(const fs::path& file, std::string& error)
{
    error.clear();
    return is_ros1_bag(file) ? channels_of<rosbags::Ros1BagFileReader>(file, error) : channels_of<rosbags::McapFileReader>(file, error);
}

std::shared_ptr<RecordingSource> open_bag_source(const BagSourceConfig& config, std::string& error)
{
    error.clear();
    if (config.files.empty())
    {
        error = "no .bag or .mcap file";
        return nullptr;
    }
    for (const auto& file : config.files)
    {
        if (!is_bag_file(file))
        {
            error = "not a .bag or .mcap file: " + file.string();
            return nullptr;
        }
        if (is_ros1_bag(file) != is_ros1_bag(config.files.front()))
        {
            error = "ROS1 bags and MCAP files cannot be mixed in one recording";
            return nullptr;
        }
    }
    if (config.chunk_seconds < 0.0)
    {
        error = "chunk length must not be negative";
        return nullptr;
    }
    return is_ros1_bag(config.files.front()) ? open_with<rosbags::Ros1BagFileReader>(config, error)
                                             : open_with<rosbags::McapFileReader>(config, error);
}

template<typename Reader>
std::shared_ptr<RecordingSource> BagSource<Reader>::open(const std::vector<fs::path>& paths, std::string& error)
{
    BagSourceConfig config;
    config.files = files(paths);
    return open_bag_source(config, error);
}

template<typename Reader>
std::unique_ptr<RecordingSourceConfigUi> BagSource<Reader>::configure(const std::vector<fs::path>& paths)
{
    return std::make_unique<BagSourceConfigUi>(files(paths));
}

template class BagSource<rosbags::Ros1BagFileReader>;
template class BagSource<rosbags::McapFileReader>;

HDMAPPING_REGISTER_RECORDING_SOURCE(Ros1BagSource)
HDMAPPING_REGISTER_RECORDING_SOURCE(McapSource)
