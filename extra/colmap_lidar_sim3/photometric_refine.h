#pragma once

// Photometric pose refinement: nudges each camera's rotation and translation until the lidar's own
// colours, seen through the camera's model, match its photo. Runs after the geometric refinement.

#include "colmap_model.h"
#include "sim3_refine.h"

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace colmap_sim3
{
    //! An 8-bit grey photo, at any resolution with the camera's aspect ratio.
    struct GrayImage
    {
        int width = 0;
        int height = 0;
        std::vector<uint8_t> pixels; //!< row-major
        std::vector<uint8_t> mask; //!< empty, or one byte per pixel; 0 masks the pixel out
    };

    //! One photo used for photometric residuals.
    struct PhotometricView
    {
        int camera = -1; //!< index into the camera list
        Camera intrinsics; //!< full-resolution COLMAP camera
        GrayImage image;
    };

    struct PhotometricParams
    {
        double max_range = 20.0; //!< lidar points further from the camera are not used [m]
        double max_angle_deg = 85.0; //!< ... nor further off the optical axis
        int cell_px = 16; //!< at most one point per cell of the photo
        double min_gradient = 4.0; //!< photo gradient a point needs [grey levels / px]
        int levels = 3; //!< image pyramid levels, solved coarse to fine
        int iterations = 15; //!< Ceres iterations per level
        double sigma_intensity = 10.0; //!< photometric noise [grey levels]
        double sigma_translation = 0.02; //!< motion between consecutive cameras vs the starting trajectory [m]
        double sigma_rotation_deg = 0.1;
        double prior_sigma_translation = 0.10; //!< pull towards the starting pose [m]
        double prior_sigma_rotation_deg = 1.0;
    };

    struct PhotometricStats
    {
        bool ok = false;
        size_t views = 0; //!< photos that got residuals
        size_t residuals = 0;
        double rms_before = 0.0; //!< photometric RMS at full resolution [grey levels], after the per-photo gain/offset
        double rms_after = 0.0;
        double mean_rotation_change_deg = 0.0;
        double mean_translation_change = 0.0; //!< [m]
        std::string message;
    };

    struct LensRotationParams
    {
        double max_range = 20.0; //!< lidar points further from the camera are not used [m]
        double max_angle_deg = 85.0; //!< ... nor further off the optical axis
        int cell_px = 6; //!< at most one point per cell of the photo (denser than per camera: the signal is pooled)
        double min_gradient = 6.0; //!< photo gradient a point needs [grey levels / px]
        int max_photos_per_lens = 300; //!< evenly spaced along the sequence
        double search_deg = 3.0; //!< half-width of the coarse search in yaw, pitch and roll
    };

    //! A rotation correction for one lens (COLMAP camera), shared by all its photos and applied in the
    //! camera frame: corrected camera -> world rotation = R * delta.
    struct LensRotation
    {
        uint32_t camera_id = 0;
        Eigen::Quaterniond delta = Eigen::Quaterniond::Identity();
        double yaw_deg = 0.0; //!< the image turn that was found: about the camera's y (image vertical) axis
        double pitch_deg = 0.0; //!< about its x axis
        double roll_deg = 0.0; //!< about its optical axis
        size_t photos = 0;
        double ncc_before = 0.0; //!< mean photo/lidar normalised cross-correlation over the photos
        double ncc_after = 0.0;
    };

    //! Finds, per lens, the camera-frame rotation that best matches all its photos to the lidar grey
    //! values at once (mean NCC, coarse-to-fine search). A small error in a lens's fixed intrinsics or
    //! rig offset turns every one of its photos the same way; each photo alone hardly shows it, the
    //! pooled correlation does.
    //! @param cameras camera -> world poses the photos are judged against
    //! @param message why nothing was estimated, when the result is empty
    //! @return one entry per lens that had usable photos
    std::vector<LensRotation> estimateLensRotations(
        const std::vector<Eigen::Vector3d>& lidar_points,
        const std::vector<float>& lidar_gray,
        const std::vector<PhotometricView>& views,
        const std::vector<CameraSim3>& cameras,
        const LensRotationParams& params,
        std::string& message);

    //! Refines each camera's rotation and translation (not its scale) photometrically.
    //! @param lidar_points lidar points in the cameras' world frame
    //! @param lidar_gray their grey value 0..255; the photos may differ by a gain and offset per photo
    //! @param views photos; cameras without one only follow the smoothness and prior terms
    //! @param cameras starting values on input (e.g. the per-camera Sim(3) result), refined on success
    bool refinePhotometric(
        const std::vector<Eigen::Vector3d>& lidar_points,
        const std::vector<float>& lidar_gray,
        const std::vector<PhotometricView>& views,
        const PhotometricParams& params,
        std::vector<CameraSim3>& cameras,
        PhotometricStats& stats);
} // namespace colmap_sim3