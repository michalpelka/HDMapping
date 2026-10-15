#include "camera_projection.h"

#include <algorithm>

namespace colmap_sim3
{
    bool isProjectionSupported(const std::string& model)
    {
        return model == "SIMPLE_PINHOLE" || model == "PINHOLE" || model == "SIMPLE_RADIAL" || model == "RADIAL" || model == "OPENCV" ||
            model == "FULL_OPENCV" || model == "OPENCV_FISHEYE" || model == "SIMPLE_RADIAL_FISHEYE" || model == "RADIAL_FISHEYE";
    }

    CameraProjector::CameraProjector(const Camera& camera, double max_angle_deg)
        : width_(camera.width)
        , p_(camera.params)
    {
        if (static_cast<int>(p_.size()) != cameraModelNumParams(camera.model))
            return;
        const std::string& m = camera.model;
        kind_ = m == "SIMPLE_PINHOLE"    ? Kind::SimplePinhole
            : m == "PINHOLE"             ? Kind::Pinhole
            : m == "SIMPLE_RADIAL"       ? Kind::SimpleRadial
            : m == "RADIAL"              ? Kind::Radial
            : m == "OPENCV"              ? Kind::OpenCV
            : m == "FULL_OPENCV"         ? Kind::FullOpenCV
            : m == "OPENCV_FISHEYE"      ? Kind::OpenCVFisheye
            : m == "SIMPLE_RADIAL_FISHEYE" ? Kind::SimpleRadialFisheye
            : m == "RADIAL_FISHEYE"      ? Kind::RadialFisheye
                                         : Kind::Unsupported;
        if (kind_ == Kind::Unsupported)
            return;

        // Walk outwards until the distorted radius stops growing (fisheye: up to 180 degrees,
        // perspective: up to about 89 degrees).
        const double limit = fisheye() ? M_PI : 50.0;
        const int steps = 20000;
        max_radius_ = limit;
        double previous = 0.0;
        for (int i = 1; i <= steps; i++)
        {
            const double r = limit * i / steps;
            const double d = distortedRadius(r);
            if (!(d > previous))
            {
                max_radius_ = limit * (i - 1) / steps;
                break;
            }
            previous = d;
        }

        const double max_angle = std::clamp(max_angle_deg, 0.0, 180.0) * M_PI / 180.0;
        if (fisheye())
            max_radius_ = std::min(max_radius_, max_angle);
        else if (max_angle < M_PI / 2)
            max_radius_ = std::min(max_radius_, std::tan(max_angle));
    }

    double CameraProjector::maxAngleDeg() const
    {
        return (fisheye() ? max_radius_ : std::atan(max_radius_)) * 180.0 / M_PI;
    }

    bool projectToImage(const Camera& camera, const Eigen::Vector3d& x_cam, Eigen::Vector2d& uv)
    {
        return CameraProjector(camera).project(x_cam, uv);
    }
} // namespace colmap_sim3