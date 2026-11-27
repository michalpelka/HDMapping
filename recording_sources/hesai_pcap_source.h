#pragma once

#include "recording_source_factory.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

// No rosbags or Hesai SDK header here: the rosbags headers must come before any standard header on MSVC (see
// rosbags/McapWriter.h), which a header included by GUI code cannot guarantee.

/**
 * @brief Hesai pcap captures as a recording: a lidar_recorder session (its lidarNNNN.pcap.zst chunks) or a set of .pcap
 * / .pcap.zst files, each file one chunk, decoded with the lidar's angle correction (correction.dat next to the files).
 *
 * Every read decodes its file with its own HesaiDecoder, so chunks load from several threads at once. Points keep the
 * lidar frame (lidar id 0, no calibration) on the lidar clock; the IMU samples carried in the point packets come out in
 * g and rad/s. The working directory is the deepest folder holding every file.
 */
class HesaiPcapSource final : public RecordingSource
{
public:
    /**
     * @brief A recording of these files.
     * @param files Captures in recording order.
     * @param correction The lidar's angle correction.
     * @param udp_port The point cloud destination port.
     */
    HesaiPcapSource(std::vector<std::filesystem::path> files, std::filesystem::path correction, uint16_t udp_port = 2368);

    std::filesystem::path workingDirectory() const override;
    size_t chunkCount() const override;
    std::string chunkName(size_t i) const override;
    std::vector<Point3Di> loadPoints(size_t i, const PointFilter& filter) const override;
    ChunkStamp stamp(size_t i) const override;
    void loadImu(Imu& imu_data, std::vector<InputFileSpan>& imuSpans, const std::function<void(double)>& progress) const override;

    /** @return "Hesai pcap". */
    static std::string name();
    /** @return Above the Mandeye LAZ/CSV kind, which accepts any folder. */
    static int priority();
    /**
     * @brief True when every path is a .pcap / .pcap.zst file or a folder holding lidarNNNN.pcap[.zst] chunks.
     * @param paths Files and folders as picked or dropped.
     * @return Whether this kind reads them.
     */
    static bool accepts(const std::vector<std::filesystem::path>& paths);
    /**
     * @brief Opens the captures of paths, with correction.dat from the first folder or the first file's folder.
     * @param paths Files and folders as picked or dropped.
     * @param error Set when they cannot be opened.
     * @return The recording, or null.
     */
    static std::shared_ptr<RecordingSource> open(const std::vector<std::filesystem::path>& paths, std::string& error);
    /** @return File dialog filter of Hesai captures. */
    static std::vector<std::string> fileFilter();

private:
    std::vector<std::filesystem::path> files_;
    std::filesystem::path correction_;
    uint16_t udp_port_;
};
