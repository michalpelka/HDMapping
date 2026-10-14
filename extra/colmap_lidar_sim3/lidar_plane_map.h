#pragma once

// Voxel hash of local planes fitted to a lidar map, for point-to-plane residuals: each cell keeps the
// mean and normal of its points if they are flat enough.

#include <Eigen/Core>

#include <ankerl/unordered_dense.h>

#include <cstdint>
#include <vector>

namespace colmap_sim3
{
    struct Plane
    {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    };

    class LidarPlaneMap
    {
    public:
        //! Fits one plane per voxel of @p points.
        //! @param points lidar points in a local frame (keep coordinates within about +-500 km)
        //! @param voxel_size cell edge in metres
        //! @param min_points cells with fewer points are dropped
        //! @param max_flatness a cell is kept when its smallest covariance eigenvalue is below this
        //!        fraction of the middle one (rejects blobs and lines alike)
        void build(const std::vector<Eigen::Vector3d>& points, double voxel_size, int min_points = 10, double max_flatness = 0.1);

        //! Finds the plane with the smallest point-to-plane distance among cells whose mean lies
        //! within @p max_dist + voxel size of @p q.
        //! @return false when no such plane is within @p max_dist of @p q
        bool nearestPlane(const Eigen::Vector3d& q, double max_dist, Plane& out) const;

        size_t size() const
        {
            return planes_.size();
        }

        double voxelSize() const
        {
            return voxel_size_;
        }

        const std::vector<Plane>& planes() const
        {
            return planes_;
        }

    private:
        double voxel_size_ = 0.5;
        ankerl::unordered_dense::map<uint64_t, uint32_t> index_;
        std::vector<Plane> planes_;
    };
} // namespace colmap_sim3
