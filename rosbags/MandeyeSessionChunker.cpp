#include "MandeyeSessionChunker.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace rosbags
{

MandeyeSessionChunker::MandeyeSessionChunker(MandeyeSessionWriter& writer, double chunk_seconds)
    : writer_(writer)
    , chunk_seconds_(chunk_seconds)
    , last_stamp_(-std::numeric_limits<double>::infinity())
{
}

int64_t MandeyeSessionChunker::timeBin(double timestamp_s) const
{
    if (chunk_seconds_ <= 0.0)
        return 0; // splitting disabled: everything lands in one chunk
    return static_cast<int64_t>(std::floor(timestamp_s / chunk_seconds_));
}

bool MandeyeSessionChunker::addPoints(const std::vector<McapPoint>& points)
{
    if (points.empty())
        return true;
    const double stamp_s = points.front().timestamp;
    if (stamp_s < last_stamp_)
        out_of_order_ = true;
    last_stamp_ = stamp_s;

    const int64_t bin = timeBin(stamp_s);
    if (chunk_bins_.empty() || bin != chunk_bins_.back())
    {
        if (!writer_.beginChunk(static_cast<int>(chunk_bins_.size())))
            return false;
        chunk_bins_.push_back(bin);
    }
    return writer_.addPoints(points);
}

void MandeyeSessionChunker::addImu(const McapImuSample& sample)
{
    imu_.push_back(sample);
}

bool MandeyeSessionChunker::finish()
{
    if (!chunk_bins_.empty() && !writer_.endChunk())
        return false;
    if (chunk_bins_.empty())
        return true;

    // chunk_bins_ is ascending when clouds arrive in time order, but an out-of-order input (a file-order read, a clock
    // step) can break that, and upper_bound on an unsorted range would route samples arbitrarily: search a sorted
    // (bin, chunk index) view instead.
    std::vector<std::pair<int64_t, size_t>> bin_to_chunk;
    bin_to_chunk.reserve(chunk_bins_.size());
    for (size_t i = 0; i < chunk_bins_.size(); ++i)
        bin_to_chunk.emplace_back(chunk_bins_[i], i);
    std::sort(bin_to_chunk.begin(), bin_to_chunk.end());

    std::vector<std::vector<McapImuSample>> per_chunk(chunk_bins_.size());
    for (const auto& sample : imu_)
    {
        const int64_t bin = timeBin(sample.timestamp);
        const auto it = std::upper_bound(
            bin_to_chunk.begin(),
            bin_to_chunk.end(),
            bin,
            [](int64_t value, const std::pair<int64_t, size_t>& entry)
            {
                return value < entry.first;
            });
        const size_t index = (it == bin_to_chunk.begin()) ? bin_to_chunk.front().second : std::prev(it)->second;
        per_chunk[index].push_back(sample);
    }
    for (size_t i = 0; i < per_chunk.size(); ++i)
    {
        if (!writer_.writeImuChunk(static_cast<int>(i), per_chunk[i]))
            return false;
    }
    return true;
}

size_t MandeyeSessionChunker::chunks() const
{
    return chunk_bins_.size();
}

bool MandeyeSessionChunker::outOfOrder() const
{
    return out_of_order_;
}

} // namespace rosbags
