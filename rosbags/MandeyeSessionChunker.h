#pragma once

#include "MandeyeSessionWriter.h" // McapPoint, McapImuSample (via McapWriter.h) -- first, see McapWriter.h

#include <cstdint>
#include <vector>

namespace rosbags
{

/**
 * @brief Splits a stream of point clouds and IMU samples into the chunks of a Mandeye session, as the bag and pcap
 * converters write them.
 *
 * Chunks follow an absolute time grid: a timestamp t belongs to bin floor(t / chunk_seconds), so lidar and IMU bin
 * identically without agreeing on a time origin. A cloud is never split: it lands in the bin of its first point. IMU
 * samples are kept until finish(), then each goes into the chunk that was open at its time (the last chunk whose bin is
 * not after the sample's, the first chunk for samples before all lidar data). No sample is dropped.
 */
class MandeyeSessionChunker
{
public:
    /**
     * @brief Writes through `writer`, which must outlive the chunker.
     * @param writer The session writer.
     * @param chunk_seconds Chunk length in seconds; 0 or less writes one chunk.
     */
    MandeyeSessionChunker(MandeyeSessionWriter& writer, double chunk_seconds);

    /**
     * @brief Writes one cloud, opening a new chunk when its first point starts a new time bin.
     * @param points Points with absolute timestamps in seconds.
     * @return False on a write error (see the writer's error()).
     */
    bool addPoints(const std::vector<McapPoint>& points);

    /**
     * @brief Keeps one IMU sample for finish().
     * @param sample Sample with an absolute timestamp in seconds.
     */
    void addImu(const McapImuSample& sample);

    /**
     * @brief Closes the last chunk and writes every chunk's imuNNNN.csv.
     * @return False on a write error (see the writer's error()).
     */
    bool finish();

    /** @return Chunks written so far. */
    size_t chunks() const;

    /** @return True when a cloud started earlier than the one before it. */
    bool outOfOrder() const;

private:
    int64_t timeBin(double timestamp_s) const;

    MandeyeSessionWriter& writer_;
    double chunk_seconds_;
    std::vector<int64_t> chunk_bins_;
    std::vector<McapImuSample> imu_;
    double last_stamp_;
    bool out_of_order_ = false;
};

} // namespace rosbags
