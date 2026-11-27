#pragma once

// McapPoint, McapImuSample. The rosbags headers come first (see rosbags/McapWriter.h); include this header only from
// .cpp files, never from a header GUI code includes.
#include "McapWriter.h"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Decodes Hesai point cloud UDP payloads with the Hesai LiDAR SDK's parser, synchronously on the caller's
 * thread. Every instance has its own parser, so several decode in parallel.
 *
 * Follows the SDK driver's packet loop: frames are split where the lidar starts a new rotation, the "no return" slots
 * the SDK emits at the origin are dropped, and the last (partial) frame comes out on finish(), so every point of the
 * input is kept. Points are in metres in the lidar frame with absolute timestamps in seconds on the lidar clock; IMU
 * samples carried in the point packets come out with acceleration in g and angular velocity in rad/s, as
 * mandeye_controller's Hesai client stores them.
 */
class HesaiDecoder
{
public:
    /**
     * @brief Prepares a decoder; the lidar model is detected from the first packet.
     * @param correction The lidar's angle correction file (correction.dat).
     * @param decode_points False decodes only the IMU samples, skipping the point computation.
     */
    explicit HesaiDecoder(const std::filesystem::path& correction, bool decode_points = true);
    ~HesaiDecoder();

    HesaiDecoder(const HesaiDecoder&) = delete;
    HesaiDecoder& operator=(const HesaiDecoder&) = delete;

    /** @brief Receives each decoded frame's points. */
    std::function<void(std::vector<rosbags::McapPoint>&&)> onFrame;
    /** @brief Receives each IMU sample. */
    std::function<void(const rosbags::McapImuSample&)> onImu;

    /**
     * @brief Decodes one UDP payload sent to the lidar's point cloud port.
     * @param payload The payload.
     * @param size Its length in bytes.
     */
    void push(const uint8_t* payload, size_t size);

    /** @brief Hands out the frame still being collected. */
    void finish();

    /** @return Empty, or why points cannot be decoded (no or unreadable angle correction). */
    std::string error() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * @brief The lidarNNNN.pcap.zst chunks of a lidar_recorder session directory (a .pcap where there is no .zst of the
 * same chunk), in order; incomplete .part chunks are skipped.
 * @param dir The session directory.
 * @return The chunk files; empty when the directory holds none.
 */
std::vector<std::filesystem::path> hesai_recorder_chunks(const std::filesystem::path& dir);

/**
 * @brief Decodes every point cloud UDP payload of one capture.
 * @param file A classic pcap, optionally zstd-compressed as a whole.
 * @param udp_port The point cloud destination port.
 * @param decoder Receives the payloads; finish() is not called.
 * @param error Set when the file cannot be read; what was read before the error was decoded.
 * @param on_bytes Optional: called with the bytes of the file read so far.
 * @return False on a read error.
 */
bool decode_hesai_capture(
    const std::filesystem::path& file,
    uint16_t udp_port,
    HesaiDecoder& decoder,
    std::string& error,
    const std::function<void(uint64_t)>& on_bytes = {});
