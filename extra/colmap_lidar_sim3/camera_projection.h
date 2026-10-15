#pragma once

// Projection of camera-frame points into a COLMAP camera's image, with COLMAP's own camera models.

#include "colmap_model.h"

#include <Eigen/Core>

#include <cmath>
#include <string>
#include <vector>

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
            return kind_ != Kind::Unsupported;
        }

        //! @return The largest angle from the optical axis that is imaged [deg].
        double maxAngleDeg() const;

        //! @return The full-resolution image width the intrinsics refer to [px].
        uint64_t imageWidth() const
        {
            return width_;
        }

        //! @param uv pixel coordinates (COLMAP convention: pixel centres at +0.5), set on success
        //! @return false for a point outside the imaged angles, or an unsupported model
        bool project(const Eigen::Vector3d& x_cam, Eigen::Vector2d& uv) const
        {
            return projectT<double>(x_cam, uv) && std::isfinite(uv.x()) && std::isfinite(uv.y());
        }

        //! project() for any scalar type, so Ceres can differentiate through it (T = ceres::Jet).
        template <typename T>
        bool projectT(const Eigen::Matrix<T, 3, 1>& x, Eigen::Matrix<T, 2, 1>& uv) const;

    private:
        enum class Kind
        {
            Unsupported,
            SimplePinhole,
            Pinhole,
            SimpleRadial,
            Radial,
            OpenCV,
            FullOpenCV,
            OpenCVFisheye,
            SimpleRadialFisheye,
            RadialFisheye,
        };

        bool fisheye() const
        {
            return kind_ == Kind::OpenCVFisheye || kind_ == Kind::SimpleRadialFisheye || kind_ == Kind::RadialFisheye;
        }

        //! Distorted radius for an undistorted one (angle for fisheye models, tan of it otherwise),
        //! without the tangential terms.
        template <typename T>
        T distortedRadius(const T& r) const;

        Kind kind_ = Kind::Unsupported;
        uint64_t width_ = 0;
        std::vector<double> p_; //!< COLMAP parameters
        double max_radius_ = 0.0; //!< largest undistorted radius that is imaged
    };

    //! Projects one point; see CameraProjector (which precomputes the per-camera limits).
    bool projectToImage(const Camera& camera, const Eigen::Vector3d& x_cam, Eigen::Vector2d& uv);

    // Same formulas as COLMAP's src/colmap/sensor/models.h.

    template <typename T>
    T CameraProjector::distortedRadius(const T& r) const
    {
        const T r2 = r * r;
        switch (kind_)
        {
        case Kind::SimpleRadial:
        case Kind::SimpleRadialFisheye:
            return r * (T(1) + T(p_[3]) * r2);
        case Kind::Radial:
        case Kind::RadialFisheye:
            return r * (T(1) + T(p_[3]) * r2 + T(p_[4]) * r2 * r2);
        case Kind::OpenCV:
            return r * (T(1) + T(p_[4]) * r2 + T(p_[5]) * r2 * r2);
        case Kind::FullOpenCV:
            return r * (T(1) + T(p_[4]) * r2 + T(p_[5]) * r2 * r2 + T(p_[8]) * r2 * r2 * r2) /
                (T(1) + T(p_[9]) * r2 + T(p_[10]) * r2 * r2 + T(p_[11]) * r2 * r2 * r2);
        case Kind::OpenCVFisheye:
            return r * (T(1) + T(p_[4]) * r2 + T(p_[5]) * r2 * r2 + T(p_[6]) * r2 * r2 * r2 + T(p_[7]) * r2 * r2 * r2 * r2);
        case Kind::SimplePinhole:
        case Kind::Pinhole:
        case Kind::Unsupported:
            break;
        }
        return r;
    }

    template <typename T>
    bool CameraProjector::projectT(const Eigen::Matrix<T, 3, 1>& x, Eigen::Matrix<T, 2, 1>& uv) const
    {
        using std::atan2;
        using std::sqrt;
        if (kind_ == Kind::Unsupported)
            return false;

        T u, v;
        if (fisheye())
        {
            // Angle from the axis via atan2, so a fisheye also images points more than 90 degrees off axis.
            const T rho2 = x[0] * x[0] + x[1] * x[1];
            if (rho2 <= T(1e-24) * x[2] * x[2])
            {
                // On the axis: theta_d / rho -> 1 / z, the perspective limit.
                if (x[2] <= T(0))
                    return false;
                u = x[0] / x[2];
                v = x[1] / x[2];
            }
            else
            {
                const T rho = sqrt(rho2);
                const T theta = atan2(rho, x[2]);
                if (theta > T(max_radius_))
                    return false;
                const T s = distortedRadius(theta) / rho;
                u = x[0] * s;
                v = x[1] * s;
            }
        }
        else
        {
            if (x[2] <= T(1e-9))
                return false;
            u = x[0] / x[2];
            v = x[1] / x[2];
            const T u2 = u * u, v2 = v * v, uv2 = u * v, r2 = u2 + v2;
            if (r2 > T(max_radius_ * max_radius_))
                return false;
            if (kind_ == Kind::SimpleRadial || kind_ == Kind::Radial)
            {
                const T radial = kind_ == Kind::SimpleRadial ? T(p_[3]) * r2 : T(p_[3]) * r2 + T(p_[4]) * r2 * r2;
                u = u + u * radial;
                v = v + v * radial;
            }
            else if (kind_ == Kind::OpenCV)
            {
                const T k1(p_[4]), k2(p_[5]), p1(p_[6]), p2(p_[7]);
                const T radial = k1 * r2 + k2 * r2 * r2;
                const T du = u * radial + T(2) * p1 * uv2 + p2 * (r2 + T(2) * u2);
                const T dv = v * radial + T(2) * p2 * uv2 + p1 * (r2 + T(2) * v2);
                u = u + du;
                v = v + dv;
            }
            else if (kind_ == Kind::FullOpenCV)
            {
                const T k1(p_[4]), k2(p_[5]), p1(p_[6]), p2(p_[7]), k3(p_[8]), k4(p_[9]), k5(p_[10]), k6(p_[11]);
                const T r4 = r2 * r2, r6 = r4 * r2;
                const T radial = (T(1) + k1 * r2 + k2 * r4 + k3 * r6) / (T(1) + k4 * r2 + k5 * r4 + k6 * r6);
                const T nu = u * radial + T(2) * p1 * uv2 + p2 * (r2 + T(2) * u2);
                const T nv = v * radial + T(2) * p2 * uv2 + p1 * (r2 + T(2) * v2);
                u = nu;
                v = nv;
            }
        }

        const bool single_focal = kind_ == Kind::SimplePinhole || kind_ == Kind::SimpleRadial || kind_ == Kind::Radial ||
            kind_ == Kind::SimpleRadialFisheye || kind_ == Kind::RadialFisheye;
        const double fx = p_[0];
        const double fy = single_focal ? p_[0] : p_[1];
        const double cx = single_focal ? p_[1] : p_[2];
        const double cy = single_focal ? p_[2] : p_[3];
        uv[0] = T(fx) * u + T(cx);
        uv[1] = T(fy) * v + T(cy);
        return true;
    }
} // namespace colmap_sim3