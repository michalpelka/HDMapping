#include <doctest.h>

#include "lidar_point_map.h"

#include <limits>
#include <random>

using namespace colmap_sim3;

TEST_CASE("nearest point matches a brute-force search")
{
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> inside(0.0, 5.0), around(-1.0, 6.0);
    std::vector<Eigen::Vector3d> pts(5000);
    for (auto& p : pts)
        p = Eigen::Vector3d(inside(rng), inside(rng), inside(rng));

    LidarPointMap map;
    map.build(pts, 0.5, 0.0);
    REQUIRE(map.size() == pts.size());

    // Up to 3 cells out the search is exact.
    for (const double max_dist : { 0.2, 0.7, 1.5 })
        for (int k = 0; k < 500; k++)
        {
            const Eigen::Vector3d q(around(rng), around(rng), around(rng));
            double best = std::numeric_limits<double>::max();
            for (const auto& p : pts)
                best = std::min(best, (p - q).norm());

            Eigen::Vector3d nearest;
            const bool found = map.nearestPoint(q, max_dist, nearest);
            CHECK(found == (best <= max_dist));
            if (found)
                CHECK((nearest - q).norm() == doctest::Approx(best));
        }
}

TEST_CASE("thinning keeps one centroid per spacing cell")
{
    // 1 m x 1 m of ground at 1 cm, thinned to 10 cm: 100 cells, centroids at the cell centres.
    std::vector<Eigen::Vector3d> pts;
    for (int i = 0; i < 100; i++)
        for (int j = 0; j < 100; j++)
            pts.emplace_back(0.005 + 0.01 * i, 0.005 + 0.01 * j, 0.3);

    LidarPointMap map;
    map.build(pts, 0.5, 0.1);
    CHECK(map.size() == 100);

    Eigen::Vector3d nearest;
    REQUIRE(map.nearestPoint(Eigen::Vector3d(0.52, 0.33, 0.4), 0.2, nearest));
    CHECK((nearest - Eigen::Vector3d(0.55, 0.35, 0.3)).norm() < 1e-9);
    CHECK_FALSE(map.nearestPoint(Eigen::Vector3d(0.52, 0.33, 0.4), 0.05, nearest));
}
