#include "lidar_point_map.h"

#include "voxel_key.h"

#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace colmap_sim3
{
    using namespace detail;

    namespace
    {
        using KeyedIndex = std::pair<uint64_t, uint32_t>;

        //! (cell key, point index) of every point in range, sorted, so each cell is one run.
        std::vector<KeyedIndex> sortedCellKeys(const std::vector<Eigen::Vector3d>& points, double voxel)
        {
            constexpr uint64_t kOutOfRange = std::numeric_limits<uint64_t>::max(); // above any packed key
            std::vector<KeyedIndex> keyed(points.size());
            tbb::parallel_for(
                size_t(0),
                points.size(),
                [&](size_t i)
                {
                    Eigen::Matrix<int64_t, 3, 1> c;
                    keyed[i] = { cellOf(points[i], voxel, c) ? packKey(c.x(), c.y(), c.z()) : kOutOfRange, static_cast<uint32_t>(i) };
                });
            tbb::parallel_sort(keyed.begin(), keyed.end());
            while (!keyed.empty() && keyed.back().first == kOutOfRange)
                keyed.pop_back();
            return keyed;
        }

        size_t runEnd(const std::vector<KeyedIndex>& keyed, size_t begin)
        {
            size_t end = begin + 1;
            while (end < keyed.size() && keyed[end].first == keyed[begin].first)
                end++;
            return end;
        }
    } // namespace

    void LidarPointMap::build(const std::vector<Eigen::Vector3d>& points, double cell_size, double spacing)
    {
        cell_size_ = cell_size;
        index_.clear();
        points_.clear();
        if (points.empty() || cell_size <= 0.0)
            return;

        std::vector<Eigen::Vector3d> thinned;
        if (spacing > 0.0)
        {
            const auto keyed = sortedCellKeys(points, spacing);
            for (size_t b = 0; b < keyed.size();)
            {
                const size_t e = runEnd(keyed, b);
                const Eigen::Vector3d& ref = points[keyed[b].second]; // sums relative to it, for precision
                Eigen::Vector3d sum = Eigen::Vector3d::Zero();
                for (size_t k = b; k < e; k++)
                    sum += points[keyed[k].second] - ref;
                thinned.push_back(ref + sum / double(e - b));
                b = e;
            }
        }
        const std::vector<Eigen::Vector3d>& source = spacing > 0.0 ? thinned : points;

        const auto keyed = sortedCellKeys(source, cell_size);
        points_.reserve(keyed.size());
        for (size_t b = 0; b < keyed.size();)
        {
            const size_t e = runEnd(keyed, b);
            index_[keyed[b].first] = { static_cast<uint32_t>(b), static_cast<uint32_t>(e) };
            for (size_t k = b; k < e; k++)
                points_.push_back(source[keyed[k].second]);
            b = e;
        }
    }

    bool LidarPointMap::nearestPoint(const Eigen::Vector3d& q, double max_dist, Eigen::Vector3d& out) const
    {
        if (points_.empty())
            return false;
        Eigen::Matrix<int64_t, 3, 1> c;
        if (!cellOf(q, cell_size_, c))
            return false;

        const int64_t rings = std::clamp<int64_t>(static_cast<int64_t>(std::ceil(max_dist / cell_size_)), 1, 3);
        double best_sq = max_dist * max_dist;
        bool found = false;
        for (int64_t dx = -rings; dx <= rings; dx++)
            for (int64_t dy = -rings; dy <= rings; dy++)
                for (int64_t dz = -rings; dz <= rings; dz++)
                {
                    // Skip cells whose box is further than the best point so far.
                    const Eigen::Vector3d lo = Eigen::Vector3d(double(c.x() + dx), double(c.y() + dy), double(c.z() + dz)) * cell_size_;
                    const Eigen::Vector3d gap = (lo - q).cwiseMax(q - lo - Eigen::Vector3d::Constant(cell_size_)).cwiseMax(0.0);
                    if (gap.squaredNorm() > best_sq)
                        continue;
                    const auto it = index_.find(packKey(c.x() + dx, c.y() + dy, c.z() + dz));
                    if (it == index_.end())
                        continue;
                    for (uint32_t k = it->second.first; k < it->second.second; k++)
                    {
                        const double d_sq = (points_[k] - q).squaredNorm();
                        if (d_sq <= best_sq)
                        {
                            best_sq = d_sq;
                            out = points_[k];
                            found = true;
                        }
                    }
                }
        return found;
    }
} // namespace colmap_sim3
