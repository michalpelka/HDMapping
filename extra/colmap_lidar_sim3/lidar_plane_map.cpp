#include "lidar_plane_map.h"

#include <Eigen/Eigenvalues>

#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>

namespace colmap_sim3
{
    namespace
    {
        // 21 bits per axis: +-2^20 cells, i.e. +-524 km at 0.5 m voxels.
        constexpr int64_t kHalfRange = int64_t(1) << 20;

        bool cellOf(const Eigen::Vector3d& p, double voxel, Eigen::Matrix<int64_t, 3, 1>& c)
        {
            for (int i = 0; i < 3; i++)
            {
                c[i] = static_cast<int64_t>(std::floor(p[i] / voxel));
                if (c[i] <= -kHalfRange || c[i] >= kHalfRange)
                    return false;
            }
            return true;
        }

        uint64_t packKey(int64_t x, int64_t y, int64_t z)
        {
            return (static_cast<uint64_t>(x + kHalfRange) << 42) | (static_cast<uint64_t>(y + kHalfRange) << 21) |
                static_cast<uint64_t>(z + kHalfRange);
        }

        //! Sums relative to the cell corner, so the covariance does not cancel out at large coordinates.
        struct Accum
        {
            Eigen::Vector3d corner = Eigen::Vector3d::Zero();
            int64_t n = 0;
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            Eigen::Matrix3d sum_sq = Eigen::Matrix3d::Zero();
        };

        using AccumMap = ankerl::unordered_dense::map<uint64_t, Accum>;
    } // namespace

    void LidarPlaneMap::build(const std::vector<Eigen::Vector3d>& points, double voxel_size, int min_points, double max_flatness)
    {
        voxel_size_ = voxel_size;
        index_.clear();
        planes_.clear();
        if (points.empty() || voxel_size <= 0.0)
            return;

        // Fixed chunks merged in order, so the result does not depend on the thread count.
        const size_t chunk = 1 << 20;
        const size_t num_chunks = (points.size() + chunk - 1) / chunk;
        std::vector<AccumMap> partial(num_chunks);
        tbb::parallel_for(
            size_t(0),
            num_chunks,
            [&](size_t k)
            {
                auto& map = partial[k];
                const size_t end = std::min(points.size(), (k + 1) * chunk);
                for (size_t i = k * chunk; i < end; i++)
                {
                    Eigen::Matrix<int64_t, 3, 1> c;
                    if (!cellOf(points[i], voxel_size, c))
                        continue;
                    auto [it, inserted] = map.try_emplace(packKey(c.x(), c.y(), c.z()));
                    Accum& a = it->second;
                    if (inserted)
                        a.corner = c.cast<double>() * voxel_size;
                    const Eigen::Vector3d d = points[i] - a.corner;
                    a.n++;
                    a.sum += d;
                    a.sum_sq += d * d.transpose();
                }
            });

        AccumMap merged = std::move(partial[0]);
        for (size_t k = 1; k < num_chunks; k++)
            for (const auto& [key, a] : partial[k])
            {
                auto [it, inserted] = merged.try_emplace(key, a);
                if (!inserted)
                {
                    it->second.n += a.n;
                    it->second.sum += a.sum;
                    it->second.sum_sq += a.sum_sq;
                }
            }
        partial.clear();

        const auto& cells = merged.values();
        std::vector<Plane> fitted(cells.size());
        std::vector<char> keep(cells.size(), 0);
        tbb::parallel_for(
            size_t(0),
            cells.size(),
            [&](size_t i)
            {
                const Accum& a = cells[i].second;
                if (a.n < min_points)
                    return;
                const Eigen::Vector3d mean = a.sum / double(a.n);
                const Eigen::Matrix3d cov = a.sum_sq / double(a.n) - mean * mean.transpose();
                const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
                const Eigen::Vector3d ev = es.eigenvalues(); // ascending
                if (!(ev[1] > 0.0) || ev[0] > max_flatness * ev[1])
                    return;
                fitted[i].mean = a.corner + mean;
                fitted[i].normal = es.eigenvectors().col(0).normalized();
                keep[i] = 1;
            });

        for (size_t i = 0; i < cells.size(); i++)
            if (keep[i])
            {
                index_[cells[i].first] = static_cast<uint32_t>(planes_.size());
                planes_.push_back(fitted[i]);
            }
    }

    bool LidarPlaneMap::nearestPlane(const Eigen::Vector3d& q, double max_dist, Plane& out) const
    {
        if (planes_.empty())
            return false;
        Eigen::Matrix<int64_t, 3, 1> c;
        if (!cellOf(q, voxel_size_, c))
            return false;

        const int64_t rings = std::clamp<int64_t>(static_cast<int64_t>(std::ceil(max_dist / voxel_size_)), 1, 3);
        const double max_center_dist = max_dist + voxel_size_;
        double best = max_dist;
        bool found = false;
        for (int64_t dx = -rings; dx <= rings; dx++)
            for (int64_t dy = -rings; dy <= rings; dy++)
                for (int64_t dz = -rings; dz <= rings; dz++)
                {
                    const auto it = index_.find(packKey(c.x() + dx, c.y() + dy, c.z() + dz));
                    if (it == index_.end())
                        continue;
                    const Plane& pl = planes_[it->second];
                    const Eigen::Vector3d d = q - pl.mean;
                    if (d.norm() > max_center_dist)
                        continue;
                    const double dist = std::abs(pl.normal.dot(d));
                    if (dist <= best)
                    {
                        best = dist;
                        out = pl;
                        found = true;
                    }
                }
        return found;
    }
} // namespace colmap_sim3
