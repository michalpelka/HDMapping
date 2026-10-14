#pragma once

// Ceres registration of an SfM (COLMAP) model to a lidar plane map.
//  - alignSimilarity(): one Sim(3) for the whole model (ICP with point-to-plane residuals).
//  - refinePerCamera(): one Sim(3) per camera, each point riding on one anchor camera, with
//    smoothness between consecutive cameras -- bends the model and lets its scale drift, to undo
//    the "banana" SfM drift.

#include "lidar_plane_map.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace colmap_sim3
{
    //! Similarity about a pivot: G(p) = s * R * (p - pivot) + t, so t is where the pivot lands.
    struct Similarity
    {
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        double s = 1.0;
        Eigen::Vector3d pivot = Eigen::Vector3d::Zero();

        Eigen::Vector3d apply(const Eigen::Vector3d& p) const
        {
            return s * (q * (p - pivot)) + t;
        }

        //! @return G as a 4x4 transform with s * R in its linear part.
        Eigen::Affine3d affine() const;
    };

    //! A camera's similarity, camera frame (COLMAP units) -> world: X = s * R * x + t.
    struct CameraSim3
    {
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        double s = 1.0;

        Eigen::Vector3d apply(const Eigen::Vector3d& x) const
        {
            return s * (q * x) + t;
        }
    };

    //! Re-association rounds: the correspondence distance shrinks geometrically from start to end.
    struct AssociationSchedule
    {
        double max_dist_start = 1.0; //!< [m]
        double max_dist_end = 0.2; //!< [m]
        int rounds = 5;
        int iterations = 20; //!< Ceres iterations per round
    };

    struct AlignParams
    {
        AssociationSchedule schedule;
        double sigma_point = 0.05; //!< point-to-plane noise [m]
        bool estimate_scale = true;
    };

    struct RefineParams
    {
        AssociationSchedule schedule{ 1.0, 0.1, 6, 20 };
        double sigma_point = 0.05; //!< point-to-plane noise [m]
        double sigma_translation = 0.02; //!< relative translation between consecutive cameras [m]
        double sigma_rotation_deg = 0.1; //!< relative rotation between consecutive cameras
        double sigma_scale = 0.001; //!< change of log(scale) between consecutive cameras
        bool use_prior = true; //!< weak pull of every camera towards its starting pose
        double prior_sigma_translation = 10.0; //!< [m]
        double prior_sigma_rotation_deg = 10.0;
    };

    struct SolveStats
    {
        bool ok = false;
        size_t inliers_before = 0;
        size_t inliers_after = 0;
        double rms_before = 0.0; //!< point-to-plane RMS over inliers [m]
        double rms_after = 0.0;
        std::string message;
    };

    //! Points expressed in the frame of one anchor camera each.
    struct AnchoredPoints
    {
        std::vector<Eigen::Vector3d> local; //!< anchor camera frame, COLMAP units
        std::vector<int> anchor; //!< index into the camera list
        std::vector<char> active; //!< used for residuals (the rest only follow their anchor)
    };

    //! Counts points within @p max_dist of a map plane and their point-to-plane RMS.
    void pointToPlaneStats(
        const std::vector<Eigen::Vector3d>& world_points,
        const std::vector<char>& active,
        const LidarPlaneMap& map,
        double max_dist,
        size_t& inliers,
        double& rms);

    //! Global Sim(3) ICP of @p points (COLMAP frame) against @p map.
    //! @param active points used for residuals (same size as points)
    //! @param G starting guess on input, result on success; its pivot is kept
    //! @return false when Ceres fails or no point finds a plane
    bool alignSimilarity(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<char>& active,
        const LidarPlaneMap& map,
        const AlignParams& params,
        Similarity& G,
        SolveStats& stats);

    //! Anchors every point to the camera of its track whose centre is closest (or to the closest
    //! camera overall when it has no track) and expresses it in that camera's frame.
    //! @param T_wc COLMAP camera -> world poses, in capture order
    //! @param track_cameras per point, indices into T_wc of the cameras observing it
    AnchoredPoints anchorPoints(
        const std::vector<Eigen::Affine3d>& T_wc,
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<std::vector<int>>& track_cameras,
        const std::vector<char>& active);

    //! @return Per-camera similarities equal to G applied to the COLMAP poses (the rigid start).
    std::vector<CameraSim3> camerasFromSimilarity(const Similarity& G, const std::vector<Eigen::Affine3d>& T_wc);

    //! Per-camera Sim(3) refinement against @p map.
    //! @param T_wc COLMAP camera -> world poses in capture order; consecutive ones are smoothed
    //! @param cameras starting values on input (e.g. camerasFromSimilarity), result on success
    bool refinePerCamera(
        const std::vector<Eigen::Affine3d>& T_wc,
        const AnchoredPoints& points,
        const LidarPlaneMap& map,
        const RefineParams& params,
        std::vector<CameraSim3>& cameras,
        SolveStats& stats);

    //! @return World positions of anchored points under @p cameras.
    std::vector<Eigen::Vector3d> anchoredWorldPoints(const AnchoredPoints& points, const std::vector<CameraSim3>& cameras);
} // namespace colmap_sim3
