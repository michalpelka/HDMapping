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

//! How a ROS1 bag or an MCAP file becomes a step 1 recording. Mirrors the options of bag_to_laz / mcap_to_laz.
struct BagSourceConfig
{
    //! PointCloud2 record layout; Auto reads it from each message's fields.
    enum class LidarLayout
    {
        Auto,
        Hesai,
        Ouster
    };

    std::filesystem::path file;
    std::string lidar_topic; //!< sensor_msgs/PointCloud2 or livox_ros_driver/CustomMsg; empty: the file's only one
    std::string imu_topic; //!< sensor_msgs/Imu; empty: the file's only one
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

//! A ROS1 bag or MCAP file split into windows of message log time. Every read opens its own Reader, so chunks can be
//! loaded from several threads at once. Defined and instantiated in bag_source.cpp for Ros1BagSource and McapSource,
//! which register themselves as recording kinds (see RecordingSourceFactoryFor).
template<typename Reader>
class BagSource final : public RecordingSource
{
public:
    //! The recording [start_ns, end_ns) of message log time in config.file, split into config.chunk_seconds windows.
    BagSource(const BagSourceConfig& config, uint64_t start_ns, uint64_t end_ns);

    std::filesystem::path workingDirectory() const override;
    size_t chunkCount() const override;
    std::string chunkName(size_t i) const override;
    std::vector<Point3Di> loadPoints(size_t i, const PointFilter& filter) const override;
    ChunkStamp stamp(size_t i) const override;
    void loadImu(Imu& imu_data, std::vector<InputFileSpan>& imuSpans) const override;

    //! "ROS1 bag" or "MCAP".
    static std::string name();
    //! Above the Mandeye LAZ/CSV kind, which accepts any folder.
    static int priority();
    //! True for exactly one file of this format (.bag or .mcap).
    static bool accepts(const std::vector<std::filesystem::path>& paths);
    //! Opens paths.front() with the default BagSourceConfig: the file's only lidar and IMU topics.
    static std::shared_ptr<RecordingSource> open(const std::vector<std::filesystem::path>& paths, std::string& error);
    //! The topic and option form for paths.front().
    static std::unique_ptr<RecordingSourceConfigUi> configure(const std::vector<std::filesystem::path>& paths);
    //! File dialog filter of this format.
    static std::vector<std::string> fileFilter();

private:
    //! Log time window [start, end) of chunk i in nanoseconds.
    std::pair<uint64_t, uint64_t> window(size_t i) const;

    BagSourceConfig config_;
    uint64_t start_ns_ = 0;
    uint64_t end_ns_ = 0;
    uint64_t chunk_ns_ = 1;
    size_t chunk_count_ = 1;
};

using Ros1BagSource = BagSource<rosbags::Ros1BagFileReader>;
using McapSource = BagSource<rosbags::McapFileReader>;

//! The file's topics. Returns an empty list and sets error when the file cannot be read.
std::vector<BagChannel> list_bag_channels(const std::filesystem::path& file, std::string& error);

//! Opens a ROS1 bag or an MCAP file as a RecordingSource. Chunks are chunk_seconds windows of message log time, read
//! through the file's chunk index, so lazy loading reads each window on its own. Points keep the lidar frame (lidar id 0,
//! no calibration); IMU samples are rotated into the lidar frame and scaled to g. The working directory is the file's
//! folder. Requires an indexed file.
//! Returns null and sets error when the file cannot be used.
std::shared_ptr<RecordingSource> open_bag_source(const BagSourceConfig& config, std::string& error);
