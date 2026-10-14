#include <doctest.h>

#include "lidar_plane_map.h"

#include <random>

using namespace colmap_sim3;

namespace
{
    std::vector<Eigen::Vector3d> grid(const Eigen::Vector3d& origin, const Eigen::Vector3d& u, const Eigen::Vector3d& v, int nu, int nv)
    {
        std::vector<Eigen::Vector3d> pts;
        for (int i = 0; i < nu; i++)
            for (int j = 0; j < nv; j++)
                pts.push_back(origin + u * i + v * j);
        return pts;
    }
} // namespace

TEST_CASE("flat ground gives z normals and point-to-plane distances")
{
    std::mt19937 rng(1);
    std::normal_distribution<double> noise(0.0, 0.005);
    auto pts = grid(Eigen::Vector3d(-5, -5, 0.3), Eigen::Vector3d(0.05, 0, 0), Eigen::Vector3d(0, 0.05, 0), 200, 200);
    for (auto& p : pts)
        p.z() += noise(rng);

    LidarPlaneMap map;
    map.build(pts, 0.5);
    REQUIRE(map.size() > 300);
    for (const auto& pl : map.planes())
        CHECK(std::abs(pl.normal.z()) > 0.99);

    Plane pl;
    REQUIRE(map.nearestPlane(Eigen::Vector3d(1.1, 2.2, 0.5), 0.3, pl));
    CHECK(std::abs(pl.normal.dot(Eigen::Vector3d(1.1, 2.2, 0.5) - pl.mean)) == doctest::Approx(0.2).epsilon(0.05));
    CHECK_FALSE(map.nearestPlane(Eigen::Vector3d(1.1, 2.2, 0.5), 0.1, pl));
    CHECK_FALSE(map.nearestPlane(Eigen::Vector3d(50, 50, 0.3), 1.0, pl));
}

TEST_CASE("lines and blobs are not planes")
{
    std::vector<Eigen::Vector3d> line;
    for (int i = 0; i < 2000; i++)
        line.emplace_back(i * 0.005, 0.25, 0.25);

    std::mt19937 rng(2);
    std::uniform_real_distribution<double> u(0.0, 0.5);
    std::vector<Eigen::Vector3d> blob;
    for (int i = 0; i < 2000; i++)
        blob.emplace_back(u(rng), u(rng), u(rng));

    LidarPlaneMap map;
    map.build(line, 0.5);
    CHECK(map.size() == 0);
    map.build(blob, 0.5);
    CHECK(map.size() == 0);
}

TEST_CASE("large coordinates keep a clean normal")
{
    auto pts = grid(Eigen::Vector3d(400000.0, 5600000.0 / 20.0, 100.0), Eigen::Vector3d(0, 0.05, 0), Eigen::Vector3d(0, 0, 0.05), 20, 20);
    LidarPlaneMap map;
    map.build(pts, 0.5);
    REQUIRE(map.size() > 0);
    for (const auto& pl : map.planes())
        CHECK(std::abs(pl.normal.x()) > 0.999);
}
