#pragma once

// Voxel cell keys shared by the lidar maps. Internal: include only from their .cpp files.

#include <Eigen/Core>

#include <cmath>
#include <cstdint>

namespace colmap_sim3::detail
{
    // 21 bits per axis: +-2^20 cells, i.e. +-524 km at 0.5 m voxels.
    constexpr int64_t kHalfRange = int64_t(1) << 20;

    inline bool cellOf(const Eigen::Vector3d& p, double voxel, Eigen::Matrix<int64_t, 3, 1>& c)
    {
        for (int i = 0; i < 3; i++)
        {
            c[i] = static_cast<int64_t>(std::floor(p[i] / voxel));
            if (c[i] <= -kHalfRange || c[i] >= kHalfRange)
                return false;
        }
        return true;
    }

    inline uint64_t packKey(int64_t x, int64_t y, int64_t z)
    {
        return (static_cast<uint64_t>(x + kHalfRange) << 42) | (static_cast<uint64_t>(y + kHalfRange) << 21) |
            static_cast<uint64_t>(z + kHalfRange);
    }
} // namespace colmap_sim3::detail
