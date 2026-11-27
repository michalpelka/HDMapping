#pragma once

#include "recording_source_factory.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// No rosbags header here: they re-enable std::byte on MSVC and must come before any standard header (see
// rosbags/McapWriter.h), which a header included by GUI code cannot guarantee.

//! How ROS1 bags or MCAP files become a step 1 recording. Mirrors the options of bag_to_laz / mcap_to_laz. Several files
//! (a split recording) are one recording read in log time order; the topics and options apply to every file.
struct BagSourceConfig
{
    //! PointCloud2 record layout; Auto reads it from each message's fields.
    enum class LidarLayout
    {
        Auto,
        Hesai,
        Ouster
    };

    std::vector<std::filesystem::path> files; //!< all .bag or all .mcap, in any order; must not overlap in log time
    std::string lidar_topic; //!< sensor_msgs/PointCloud2 or livox_ros_driver/CustomMsg; empty: each file's only one
    std::string imu_topic; //!< sensor_msgs/Imu; empty: each file's only one
    LidarLayout lidar_layout = LidarLayout::Auto;
    bool imu_acc_in_g = true; //!< false: m/s^2 (the sensor_msgs/Imu convention), divided by standard gravity
    std::array<double, 4> imu_to_lidar{ 0.0, 0.0, 0.0, 1.0 }; //!< rotation IMU frame -> lidar frame, qx qy qz qw
    double chunk_seconds = 10.0; //!< length of one chunk of message log time
};

//! One topic of a bag file, for topic pickers.
struct BagChannel
{
    enum class Kind
    {
        Lidar,
        Imu,
        Other
    };

    std::string topic;
    std::string type; //!< message type as the file names it
    uint64_t message_count = 0; //!< 0 when the file carries no counts
    Kind kind = Kind::Other;
};

namespace rosbags
{
    class Ros1BagFileReader;
    class McapFileReader;
} // namespace rosbags

//! ROS1 bags or MCAP files split into windows of message log time, the files one after another. Every read opens its own
//! Reader, so chunks can be loaded from several threads at once. Defined and instantiated in bag_source.cpp for Ros1BagSource and
//! McapSource, which register themselves as recording kinds (see RecordingSourceFactoryFor).
template<typename Reader>
class BagSource final : public RecordingSource
{
public:
    //! Splits each file's log time span [start_ns, end_ns) into config.chunk_seconds windows. spans[k] belongs to
    //! config.files[k]; the files are in log time order.
    BagSource(const BagSourceConfig& config, const std::vector<std::pair<uint64_t, uint64_t>>& spans);

    std::filesystem::path workingDirectory() const override;
    size_t chunkCount() const override;
    std::string chunkName(size_t i) const override;
    std::vector<Point3Di> loadPoints(size_t i, const PointFilter& filter) const override;
    ChunkStamp stamp(size_t i) const override;
    void loadImu(Imu& imu_data, std::vector<InputFileSpan>& imuSpans, const std::function<void(double)>& progress) const override;

    //! "ROS1 bag" or "MCAP".
    static std::string name();
    //! Above the Mandeye LAZ/CSV kind, which accepts any folder.
    static int priority();
    //! True when every path is a file of this format (.bag or .mcap) or a folder with such files below it and no
    //! Mandeye LAZ/LAS files at its top level.
    static bool accepts(const std::vector<std::filesystem::path>& paths);
    //! Opens the files of paths (see files()) with the default BagSourceConfig: each file's only lidar and IMU topics.
    static std::shared_ptr<RecordingSource> open(const std::vector<std::filesystem::path>& paths, std::string& error);
    //! The files of this format among paths: the files themselves and every one below a folder, sorted by path; empty
    //! when a path is neither.
    static std::vector<std::filesystem::path> files(const std::vector<std::filesystem::path>& paths);
    //! The topic and option form for the files of paths.
    static std::unique_ptr<RecordingSourceConfigUi> configure(const std::vector<std::filesystem::path>& paths);
    //! File dialog filter of this format.
    static std::vector<std::string> fileFilter();

private:
    //! One chunk: a log time window [start_ns, end_ns) of one file.
    struct Window
    {
        size_t file = 0; //!< index into config_.files
        size_t index = 0; //!< the window's number within its file
        uint64_t start_ns = 0;
        uint64_t end_ns = 0;
    };

    BagSourceConfig config_;
    std::vector<Window> windows_;
};

using Ros1BagSource = BagSource<rosbags::Ros1BagFileReader>;
using McapSource = BagSource<rosbags::McapFileReader>;

//! Advice for ROS1 bags among files that use bz2 chunk compression, which loads tens of times slower than lz4 or none
//! because the IMU data is read in one pass over the whole bag; empty when no file does.
std::string bag_compression_hint(const std::vector<std::filesystem::path>& files);

//! The file's topics. Returns an empty list and sets error when the file cannot be read.
std::vector<BagChannel> list_bag_channels(const std::filesystem::path& file, std::string& error);

//! Opens ROS1 bags or MCAP files as one RecordingSource, the files ordered by log time. Chunks are chunk_seconds windows
//! of message log time within one file, read through the file's chunk index, so lazy loading reads each window on its
//! own. Points keep the lidar frame (lidar id 0, no calibration); IMU samples are rotated into the lidar frame and scaled
//! to g. The working directory is the deepest folder that holds every file. Requires indexed files with the lidar and IMU
//! topics, all of one format and not overlapping in log time.
//! Returns null and sets error when the files cannot be used.
std::shared_ptr<RecordingSource> open_bag_source(const BagSourceConfig& config, std::string& error);
