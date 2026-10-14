#include "camera_projection.h"

#include <algorithm>
#include <cmath>

namespace colmap_sim3
{
    namespace
    {
        // Same formulas as COLMAP's src/colmap/sensor/models.h.

        void opencvDistortion(double k1, double k2, double p1, double p2, double& u, double& v)
        {
            const double u2 = u * u, v2 = v * v, uv = u * v, r2 = u2 + v2;
            const double radial = k1 * r2 + k2 * r2 * r2;
            const double du = u * radial + 2.0 * p1 * uv + p2 * (r2 + 2.0 * u2);
            const double dv = v * radial + 2.0 * p2 * uv + p1 * (r2 + 2.0 * v2);
            u += du;
            v += dv;
        }

        bool isFisheye(const std::string& m)
        {
            return m == "OPENCV_FISHEYE" || m == "SIMPLE_RADIAL_FISHEYE" || m == "RADIAL_FISHEYE";
        }
    } // namespace

    bool isProjectionSupported(const std::string& model)
    {
        return model == "SIMPLE_PINHOLE" || model == "PINHOLE" || model == "SIMPLE_RADIAL" || model == "RADIAL" || model == "OPENCV" ||
            model == "FULL_OPENCV" || isFisheye(model);
    }

    CameraProjector::CameraProjector(const Camera& camera, double max_angle_deg)
        : camera_(camera)
    {
        supported_ = isProjectionSupported(camera.model) && static_cast<int>(camera.params.size()) == cameraModelNumParams(camera.model);
        fisheye_ = isFisheye(camera.model);
        if (!supported_)
            return;

        // Walk outwards until the distorted radius stops growing (fisheye: up to 180 degrees,
        // perspective: up to about 89 degrees).
        const double limit = fisheye_ ? M_PI : 50.0;
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
        if (fisheye_)
            max_radius_ = std::min(max_radius_, max_angle);
        else if (max_angle < M_PI / 2)
            max_radius_ = std::min(max_radius_, std::tan(max_angle));
    }

    double CameraProjector::maxAngleDeg() const
    {
        return (fisheye_ ? max_radius_ : std::atan(max_radius_)) * 180.0 / M_PI;
    }

    double CameraProjector::distortedRadius(double r) const
    {
        const std::vector<double>& p = camera_.params;
        const std::string& m = camera_.model;
        const double r2 = r * r;
        if (m == "SIMPLE_RADIAL")
            return r * (1.0 + p[3] * r2);
        if (m == "RADIAL")
            return r * (1.0 + p[3] * r2 + p[4] * r2 * r2);
        if (m == "OPENCV")
            return r * (1.0 + p[4] * r2 + p[5] * r2 * r2);
        if (m == "FULL_OPENCV")
            return r * (1.0 + p[4] * r2 + p[5] * r2 * r2 + p[8] * r2 * r2 * r2) / (1.0 + p[9] * r2 + p[10] * r2 * r2 + p[11] * r2 * r2 * r2);
        if (m == "OPENCV_FISHEYE")
            return r * (1.0 + p[4] * r2 + p[5] * r2 * r2 + p[6] * r2 * r2 * r2 + p[7] * r2 * r2 * r2 * r2);
        if (m == "SIMPLE_RADIAL_FISHEYE")
            return r * (1.0 + p[3] * r2);
        if (m == "RADIAL_FISHEYE")
            return r * (1.0 + p[3] * r2 + p[4] * r2 * r2);
        return r; // SIMPLE_PINHOLE, PINHOLE
    }

    bool CameraProjector::project(const Eigen::Vector3d& x, Eigen::Vector2d& uv) const
    {
        if (!supported_)
            return false;
        const std::vector<double>& p = camera_.params;
        const std::string& m = camera_.model;

        double u = 0.0, v = 0.0;
        if (fisheye_)
        {
            // Angle from the axis via atan2, so a fisheye also images points more than 90 degrees off axis.
            const double rho = std::hypot(x.x(), x.y());
            const double theta = std::atan2(rho, x.z());
            if (theta > max_radius_ || (rho <= 1e-12 && x.z() <= 0.0))
                return false;
            if (rho > 1e-12)
            {
                const double scale = distortedRadius(theta) / rho;
                u = x.x() * scale;
                v = x.y() * scale;
            }
        }
        else
        {
            if (x.z() <= 1e-9)
                return false;
            u = x.x() / x.z();
            v = x.y() / x.z();
            const double r = std::hypot(u, v);
            if (r > max_radius_)
                return false;
            if (m == "SIMPLE_RADIAL" || m == "RADIAL")
            {
                const double s = r > 0.0 ? distortedRadius(r) / r : 1.0;
                u *= s;
                v *= s;
            }
            else if (m == "OPENCV")
                opencvDistortion(p[4], p[5], p[6], p[7], u, v);
            else if (m == "FULL_OPENCV")
            {
                const double k1 = p[4], k2 = p[5], p1 = p[6], p2 = p[7], k3 = p[8], k4 = p[9], k5 = p[10], k6 = p[11];
                const double u2 = u * u, v2 = v * v, uv = u * v, r2 = u2 + v2, r4 = r2 * r2, r6 = r4 * r2;
                const double radial = (1.0 + k1 * r2 + k2 * r4 + k3 * r6) / (1.0 + k4 * r2 + k5 * r4 + k6 * r6);
                const double nu = u * radial + 2.0 * p1 * uv + p2 * (r2 + 2.0 * u2);
                const double nv = v * radial + 2.0 * p2 * uv + p1 * (r2 + 2.0 * v2);
                u = nu;
                v = nv;
            }
        }

        const bool single_focal = m == "SIMPLE_PINHOLE" || m == "SIMPLE_RADIAL" || m == "RADIAL" || m == "SIMPLE_RADIAL_FISHEYE" ||
            m == "RADIAL_FISHEYE";
        const double fx = p[0];
        const double fy = single_focal ? p[0] : p[1];
        const double cx = single_focal ? p[1] : p[2];
        const double cy = single_focal ? p[2] : p[3];
        uv = Eigen::Vector2d(fx * u + cx, fy * v + cy);
        return std::isfinite(uv.x()) && std::isfinite(uv.y());
    }

    bool projectToImage(const Camera& camera, const Eigen::Vector3d& x_cam, Eigen::Vector2d& uv)
    {
        return CameraProjector(camera).project(x_cam, uv);
    }
} // namespace colmap_sim3
