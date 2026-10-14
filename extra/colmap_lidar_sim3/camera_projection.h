#pragma once

// Projection of camera-frame points into a COLMAP camera's image, with COLMAP's own camera models.

#include "colmap_model.h"

#include <Eigen/Core>

#include <string>

namespace colmap_sim3
{
    //! @return Whether CameraProjector handles this COLMAP camera model. FOV, THIN_PRISM_FISHEYE and
    //!         RAD_TAN_THIN_PRISM_FISHEYE are not supported.
    bool isProjectionSupported(const std::string& model);

    //! Projects camera-frame points (COLMAP: x right, y down, z forward) into one camera's image.
    //! A fitted distortion polynomial is only valid while the distorted radius still grows; past that
    //! it folds back and would draw points from outside the lens (for a fisheye, even from behind it)
    //! into the image, so those points are rejected.
    class CameraProjector
    {
    public:
        //! @param camera intrinsics and model
        //! @param max_angle_deg also reject points further than this from the optical axis
        explicit CameraProjector(const Camera& camera, double max_angle_deg = 180.0);

        bool supported() const
        {
            return supported_;
        }

        //! @return The largest angle from the optical axis that is imaged [deg].
        double maxAngleDeg() const;

        //! @param uv pixel coordinates (COLMAP convention: pixel centres at +0.5), set on success
        //! @return false for a point outside the imaged angles, or an unsupported model
        bool project(const Eigen::Vector3d& x_cam, Eigen::Vector2d& uv) const;

    private:
        //! Distorted radius for an undistorted one (angle for fisheye models, tan of it otherwise),
        //! without the tangential terms.
        double distortedRadius(double r) const;

        Camera camera_;
        bool supported_ = false;
        bool fisheye_ = false;
        double max_radius_ = 0.0; //!< largest undistorted radius that is imaged
    };

    //! Projects one point; see CameraProjector (which precomputes the per-camera limits).
    bool projectToImage(const Camera& camera, const Eigen::Vector3d& x_cam, Eigen::Vector2d& uv);
} // namespace colmap_sim3
