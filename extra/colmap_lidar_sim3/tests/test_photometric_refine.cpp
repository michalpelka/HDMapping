#include <doctest.h>

#include "photometric_refine.h"

#include <Eigen/Geometry>

#include <cmath>
#include <optional>

using namespace colmap_sim3;

namespace
{
    // A 40 m street (ground, two walls, an end wall) with a smooth texture on every surface.
    double texture(const Eigen::Vector3d& p)
    {
        return 128.0 + 40.0 * std::sin(3.1 * p.x() + 2.3 * p.y() + 1.7 * p.z()) +
            30.0 * std::cos(7.3 * p.x() - 5.1 * p.y() + 6.2 * p.z()) * std::sin(4.1 * p.z() + 2.9 * p.y() - 3.3 * p.x());
    }

    //! Nearest hit of a ray with the street's surfaces.
    std::optional<Eigen::Vector3d> castRay(const Eigen::Vector3d& o, const Eigen::Vector3d& d)
    {
        double best = std::numeric_limits<double>::max();
        auto plane = [&](int axis, double value, auto&& inside)
        {
            if (std::abs(d[axis]) < 1e-12)
                return;
            const double t = (value - o[axis]) / d[axis];
            if (t > 1e-6 && t < best && inside(o + t * d))
                best = t;
        };
        plane(2, 0.0, [](const Eigen::Vector3d& p) { return std::abs(p.y()) <= 6.0 && p.x() >= 0.0 && p.x() <= 40.0; });
        for (double y : { -6.0, 6.0 })
            plane(1, y, [](const Eigen::Vector3d& p) { return p.z() >= 0.0 && p.z() <= 5.0 && p.x() >= 0.0 && p.x() <= 40.0; });
        plane(0, 40.0, [](const Eigen::Vector3d& p) { return std::abs(p.y()) <= 6.0 && p.z() >= 0.0 && p.z() <= 5.0; });
        if (best == std::numeric_limits<double>::max())
            return std::nullopt;
        return o + best * d;
    }

    std::vector<Eigen::Vector3d> sampleStreet(double step)
    {
        std::vector<Eigen::Vector3d> pts;
        for (double x = 0; x <= 40.0; x += step)
        {
            for (double y = -6.0; y <= 6.0; y += step)
                pts.emplace_back(x, y, 0.0);
            for (double z = 0; z <= 5.0; z += step)
            {
                pts.emplace_back(x, -6.0, z);
                pts.emplace_back(x, 6.0, z);
            }
        }
        for (double y = -6.0; y <= 6.0; y += step)
            for (double z = 0; z <= 5.0; z += step)
                pts.emplace_back(40.0, y, z);
        return pts;
    }

    Camera pinhole()
    {
        Camera c;
        c.id = 1;
        c.model = "PINHOLE";
        c.width = 320;
        c.height = 240;
        c.params = { 220.0, 220.0, 160.0, 120.0 };
        return c;
    }

    //! The photo a camera at pose (camera -> world) takes, with a different gain/offset than the lidar.
    GrayImage render(const Camera& cam, const CameraSim3& pose)
    {
        GrayImage img;
        img.width = static_cast<int>(cam.width);
        img.height = static_cast<int>(cam.height);
        img.pixels.resize(size_t(img.width) * img.height, 0);
        for (int y = 0; y < img.height; y++)
            for (int x = 0; x < img.width; x++)
            {
                const Eigen::Vector3d ray((x + 0.5 - cam.params[2]) / cam.params[0], (y + 0.5 - cam.params[3]) / cam.params[1], 1.0);
                if (const auto hit = castRay(pose.t, pose.q * ray.normalized()))
                    img.pixels[size_t(y) * img.width + x] = static_cast<uint8_t>(std::clamp(0.8 * texture(*hit) + 20.0, 0.0, 255.0));
            }
        return img;
    }
} // namespace

TEST_CASE("photometric refinement recovers a smooth pose drift")
{
    const std::vector<Eigen::Vector3d> lidar = sampleStreet(0.04);
    std::vector<float> gray;
    for (const auto& p : lidar)
        gray.push_back(static_cast<float>(texture(p)));

    Eigen::Matrix3d R_look;
    R_look << 0, 0, 1, -1, 0, 0, 0, -1, 0; // camera z forward along +x, y down
    const Camera cam = pinhole();
    std::vector<CameraSim3> truth, cameras;
    std::vector<PhotometricView> views;
    for (int k = 0; k < 12; k++)
    {
        CameraSim3 c;
        c.q = Eigen::Quaterniond(R_look);
        c.t = Eigen::Vector3d(8.0 + 1.5 * k, 0.5 * std::sin(0.4 * k), 1.6);
        c.s = 1.0;
        truth.push_back(c);

        // Smooth, slowly varying error: up to about 0.6 deg and 4 cm.
        CameraSim3 start = c;
        const Eigen::Quaterniond dq = Eigen::AngleAxisd(0.6 * M_PI / 180.0 * std::sin(0.3 * k), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.4 * M_PI / 180.0 * std::cos(0.25 * k), Eigen::Vector3d::UnitY());
        start.q = (dq * c.q).normalized();
        start.t += Eigen::Vector3d(0.03 * std::cos(0.2 * k), 0.04 * std::sin(0.3 * k), 0.02);
        cameras.push_back(start);

        PhotometricView v;
        v.camera = k;
        v.intrinsics = cam;
        v.image = render(cam, c);
        views.push_back(std::move(v));
    }

    double rot_before = 0.0, trans_before = 0.0;
    for (int k = 0; k < 12; k++)
    {
        rot_before += truth[k].q.angularDistance(cameras[k].q) * 180.0 / M_PI;
        trans_before += (truth[k].t - cameras[k].t).norm();
    }

    PhotometricParams params;
    params.max_range = 25.0;
    params.cell_px = 8;
    params.iterations = 30;
    params.prior_sigma_rotation_deg = 2.0;
    params.prior_sigma_translation = 0.2;
    PhotometricStats stats;
    REQUIRE_MESSAGE(refinePhotometric(lidar, gray, views, params, cameras, stats), stats.message);

    double rot_after = 0.0, trans_after = 0.0;
    for (int k = 0; k < 12; k++)
    {
        rot_after += truth[k].q.angularDistance(cameras[k].q) * 180.0 / M_PI;
        trans_after += (truth[k].t - cameras[k].t).norm();
    }
    MESSAGE(
        "mean rotation error " << rot_before / 12 << " -> " << rot_after / 12 << " deg, translation " << trans_before / 12 << " -> "
                               << trans_after / 12 << " m, photometric RMS " << stats.rms_before << " -> " << stats.rms_after << ", "
                               << stats.residuals << " residuals");
    CHECK(stats.rms_after < stats.rms_before);
    CHECK(rot_after / 12 < 0.1);
    CHECK(trans_after / 12 < 0.02);
}

TEST_CASE("a rotation shared by one lens's photos is found and the other lens is left alone")
{
    const std::vector<Eigen::Vector3d> lidar = sampleStreet(0.04);
    std::vector<float> gray;
    for (const auto& p : lidar)
        gray.push_back(static_cast<float>(texture(p)));

    // A two-lens rig: lens 1 looks along +x, lens 2 along -x. Lens 2's photos are taken turned by a
    // fixed camera-frame rotation that its poses do not know about.
    Eigen::Matrix3d R_look;
    R_look << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    const Eigen::Matrix3d R_back = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * R_look;
    const double yaw = 1.0, pitch = -1.5, roll = 0.6;
    const Eigen::Matrix3d Rd = (Eigen::AngleAxisd(yaw * M_PI / 180, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(pitch * M_PI / 180, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(roll * M_PI / 180, Eigen::Vector3d::UnitZ()))
                                   .toRotationMatrix();
    const Eigen::Quaterniond true_delta(Rd.transpose());

    std::vector<CameraSim3> cameras;
    std::vector<PhotometricView> views;
    for (int k = 0; k < 10; k++)
        for (int lens : { 1, 2 })
        {
            CameraSim3 c;
            c.q = Eigen::Quaterniond(lens == 1 ? R_look : R_back);
            c.t = Eigen::Vector3d(lens == 1 ? 6.0 + 1.5 * k : 20.0 + 1.5 * k, 0.4 * std::sin(0.5 * k), 1.6);
            cameras.push_back(c);

            CameraSim3 taken = c;
            if (lens == 2)
                taken.q = (c.q * true_delta).normalized();
            PhotometricView v;
            v.camera = static_cast<int>(cameras.size()) - 1;
            v.intrinsics = pinhole();
            v.intrinsics.id = static_cast<uint32_t>(lens);
            v.image = render(v.intrinsics, taken);
            views.push_back(std::move(v));
        }

    LensRotationParams params;
    params.max_range = 25.0;
    std::string message;
    const std::vector<LensRotation> found = estimateLensRotations(lidar, gray, views, cameras, params, message);
    REQUIRE_MESSAGE(found.size() == 2, message);
    for (const LensRotation& lr : found)
    {
        MESSAGE(
            "lens " << lr.camera_id << ": yaw " << lr.yaw_deg << ", pitch " << lr.pitch_deg << ", roll " << lr.roll_deg << " deg, NCC "
                    << lr.ncc_before << " -> " << lr.ncc_after << " over " << lr.photos << " photos");
        if (lr.camera_id == 1)
            CHECK(lr.delta.angularDistance(Eigen::Quaterniond::Identity()) * 180 / M_PI < 0.1);
        else
        {
            CHECK(lr.delta.angularDistance(true_delta) * 180 / M_PI < 0.15);
            CHECK(lr.ncc_after > lr.ncc_before);
        }
    }
}

TEST_CASE("photometric refinement refuses when the photos see no lidar")
{
    const std::vector<Eigen::Vector3d> lidar{ { 100, 100, 100 }, { 101, 100, 100 } };
    const std::vector<float> gray{ 100.f, 200.f };
    std::vector<CameraSim3> cameras(1);
    PhotometricView v;
    v.camera = 0;
    v.intrinsics = pinhole();
    v.image.width = 320;
    v.image.height = 240;
    v.image.pixels.assign(320 * 240, 128);
    PhotometricStats stats;
    CHECK_FALSE(refinePhotometric(lidar, gray, { v }, PhotometricParams{}, cameras, stats));
    CHECK(!stats.message.empty());
}