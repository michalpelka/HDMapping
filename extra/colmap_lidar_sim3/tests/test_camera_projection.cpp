#include <doctest.h>

#include "camera_projection.h"

#include <cmath>

using namespace colmap_sim3;

namespace
{
    Camera make(const std::string& model, std::vector<double> params)
    {
        Camera c;
        c.model = model;
        c.width = 1000;
        c.height = 800;
        c.params = std::move(params);
        return c;
    }

    Eigen::Vector2d project(const Camera& c, const Eigen::Vector3d& x)
    {
        Eigen::Vector2d uv(-1, -1);
        REQUIRE(projectToImage(c, x, uv));
        return uv;
    }
} // namespace

TEST_CASE("every supported model has a known parameter count and the rest are refused")
{
    for (int id = 0; id < 12; id++)
    {
        const std::string name = cameraModelName(id);
        Camera c = make(name, std::vector<double>(cameraModelNumParams(name), 0.0));
        c.params[0] = 500.0;
        Eigen::Vector2d uv;
        CHECK(projectToImage(c, Eigen::Vector3d(0.1, 0.2, 1.0), uv) == isProjectionSupported(name));
    }
    Eigen::Vector2d uv;
    CHECK_FALSE(projectToImage(make("PINHOLE", { 500, 500, 320 }), Eigen::Vector3d(0, 0, 1), uv)); // wrong param count
}

TEST_CASE("pinhole models")
{
    const Eigen::Vector3d x(0.2, -0.1, 2.0);
    const Eigen::Vector2d pin = project(make("PINHOLE", { 500, 400, 320, 240 }), x);
    CHECK(pin.x() == doctest::Approx(500 * 0.1 + 320));
    CHECK(pin.y() == doctest::Approx(400 * -0.05 + 240));

    // No distortion: every perspective model agrees with PINHOLE.
    CHECK((project(make("OPENCV", { 500, 400, 320, 240, 0, 0, 0, 0 }), x) - pin).norm() < 1e-9);
    CHECK((project(make("FULL_OPENCV", { 500, 400, 320, 240, 0, 0, 0, 0, 0, 0, 0, 0 }), x) - pin).norm() < 1e-9);
    const Eigen::Vector2d simple = project(make("SIMPLE_PINHOLE", { 500, 320, 240 }), x);
    CHECK((project(make("SIMPLE_RADIAL", { 500, 320, 240, 0 }), x) - simple).norm() < 1e-9);
    CHECK((project(make("RADIAL", { 500, 320, 240, 0, 0 }), x) - simple).norm() < 1e-9);

    // Radial distortion pushes the point outwards: u * (1 + k r^2).
    const double u = 0.1, v = -0.05, r2 = u * u + v * v;
    const Eigen::Vector2d radial = project(make("SIMPLE_RADIAL", { 500, 320, 240, 0.3 }), x);
    CHECK(radial.x() == doctest::Approx(500 * u * (1 + 0.3 * r2) + 320));

    Eigen::Vector2d uv;
    CHECK_FALSE(projectToImage(make("PINHOLE", { 500, 400, 320, 240 }), Eigen::Vector3d(0.2, 0.1, -1.0), uv));
}

TEST_CASE("points where the lens polynomial folds back are not imaged")
{
    // colmap_full's back lens: theta_d peaks near 112 degrees, then shrinks through 0 by 180 degrees.
    const Camera c = make("OPENCV_FISHEYE", { 517.70, 517.70, 961.95, 960.30, 0.0871590, -0.0233734, 0.0016078, -0.00048032 });
    const CameraProjector projector(c);
    CHECK(projector.maxAngleDeg() == doctest::Approx(112.0).epsilon(0.01));

    auto at = [](double deg)
    {
        const double a = deg * M_PI / 180.0;
        return Eigen::Vector3d(std::sin(a), 0.0, std::cos(a));
    };
    Eigen::Vector2d uv;
    CHECK(projector.project(at(100.0), uv));
    CHECK_FALSE(projector.project(at(150.0), uv)); // would land 1900 px further in, on the wrong side
    CHECK_FALSE(projector.project(Eigen::Vector3d(0.0, 0.0, -1.0), uv));

    const CameraProjector narrow(c, 95.0);
    CHECK(narrow.maxAngleDeg() == doctest::Approx(95.0).epsilon(0.01));
    CHECK(narrow.project(at(90.0), uv));
    CHECK_FALSE(narrow.project(at(100.0), uv));

    // Barrel distortion (k1 < 0) in a perspective model folds too: r (1 + k1 r^2) peaks at r = 1/sqrt(-3 k1).
    const Camera barrel = make("SIMPLE_RADIAL", { 500, 320, 240, -0.5 });
    CHECK(std::tan(CameraProjector(barrel).maxAngleDeg() * M_PI / 180.0) == doctest::Approx(1.0 / std::sqrt(1.5)).epsilon(0.01));
    CHECK(projectToImage(barrel, Eigen::Vector3d(0.5, 0, 1), uv));
    CHECK_FALSE(projectToImage(barrel, Eigen::Vector3d(1.5, 0, 1), uv));
}

TEST_CASE("OpenCV fisheye matches COLMAP in front and keeps imaging past 90 degrees")
{
    const double k[4] = { 0.085, -0.02, -0.0016, 0.00009 };
    const Camera c = make("OPENCV_FISHEYE", { 533.4, 520.8, 961.9, 960.3, k[0], k[1], k[2], k[3] });

    // COLMAP: u = x/z, theta = atan(r), u_d = u * theta_d / r.
    const Eigen::Vector3d x(0.7, -0.4, 1.3);
    const double u = x.x() / x.z(), v = x.y() / x.z(), r = std::hypot(u, v), th = std::atan(r);
    const double th_d = th * (1 + k[0] * th * th + k[1] * std::pow(th, 4) + k[2] * std::pow(th, 6) + k[3] * std::pow(th, 8));
    const Eigen::Vector2d uv = project(c, x);
    CHECK(uv.x() == doctest::Approx(533.4 * u * th_d / r + 961.9));
    CHECK(uv.y() == doctest::Approx(520.8 * v * th_d / r + 960.3));

    // 100 degrees off axis, sideways: still in the image, further out than 90 degrees.
    const double a90 = M_PI / 2, a100 = 100.0 * M_PI / 180.0;
    const Eigen::Vector2d at90 = project(c, Eigen::Vector3d(std::sin(a90), 0, std::cos(a90)));
    const Eigen::Vector2d at100 = project(c, Eigen::Vector3d(std::sin(a100), 0, std::cos(a100)));
    CHECK(at100.x() > at90.x());
    CHECK(at90.x() > 961.9);

    Eigen::Vector2d none;
    CHECK_FALSE(projectToImage(c, Eigen::Vector3d(0, 0, -1), none)); // straight behind
    CHECK((project(c, Eigen::Vector3d(0, 0, 5)) - Eigen::Vector2d(961.9, 960.3)).norm() < 1e-9);
}
