#pragma once

// Voxel hash of a thinned lidar map, for point-to-point residuals: the map is thinned to one centroid
// per small cell and the centroids are bucketed in search cells.

#include <Eigen/Core>

#include <ankerl/unordered_dense.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace colmap_sim3
{
    class LidarPointMap
    {
    public:
        //! Thins @p points and buckets them for nearest-point search.
        //! @param points lidar points in a local frame (keep coordinates within about +-500 km)
        //! @param cell_size edge of the search cells in metres
        //! @param spacing the map keeps one centroid per cell of this edge; <= 0 keeps every point
        void build(const std::vector<Eigen::Vector3d>& points, double cell_size, double spacing);

        //! Finds the nearest map point within @p max_dist of @p q, searching at most 3 cells out.
        //! @return false when there is none
        bool nearestPoint(const Eigen::Vector3d& q, double max_dist, Eigen::Vector3d& out) const;

        size_t size() const
        {
            return points_.size();
        }

    private:
        double cell_size_ = 0.5;
        ankerl::unordered_dense::map<uint64_t, std::pair<uint32_t, uint32_t>> index_; //!< cell -> [begin, end) in points_
        std::vector<Eigen::Vector3d> points_; //!< grouped by cell
    };
} // namespace colmap_sim3
