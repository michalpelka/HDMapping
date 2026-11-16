#include <doctest.h>

#include "../lidar_odometry_utils.h"

namespace
{
    // Points every `step` seconds from `start`, `count` of them
    void appendPoints(std::vector<Point3Di>& points, double start, double step, int count)
    {
        for (int i = 0; i < count; i++)
        {
            Point3Di p{};
            p.timestamp = start + i * step;
            points.push_back(p);
        }
    }
} // namespace

TEST_CASE("find_lidar_gaps: continuous data has no gaps")
{
    std::vector<Point3Di> points;
    appendPoints(points, 100.0, 0.001, 1000);
    CHECK(find_lidar_gaps(points, 0.05).empty());
}

TEST_CASE("find_lidar_gaps: reports where a gap starts and how long it is")
{
    std::vector<Point3Di> points;
    appendPoints(points, 100.0, 0.001, 100); // last point at 100.099
    appendPoints(points, 100.3, 0.001, 100);

    const auto gaps = find_lidar_gaps(points, 0.05);
    REQUIRE(gaps.size() == 1);
    CHECK(gaps[0].start == doctest::Approx(100.099));
    CHECK(gaps[0].length == doctest::Approx(0.201));
}

TEST_CASE("find_lidar_gaps: only gaps longer than the threshold count")
{
    std::vector<Point3Di> points;
    appendPoints(points, 0.0, 0.001, 10); // last point at 0.009
    appendPoints(points, 0.04, 0.001, 10); // 31 ms gap: below the threshold
    appendPoints(points, 0.2, 0.001, 10); // 151 ms gap
    appendPoints(points, 0.5, 0.001, 10); // 291 ms gap

    const auto gaps = find_lidar_gaps(points, 0.05);
    REQUIRE(gaps.size() == 2);
    CHECK(gaps[0].length == doctest::Approx(0.151));
    CHECK(gaps[1].length == doctest::Approx(0.291));
}

TEST_CASE("find_lidar_gaps: empty or single-point input has no gaps")
{
    std::vector<Point3Di> points;
    CHECK(find_lidar_gaps(points, 0.05).empty());
    appendPoints(points, 1.0, 0.001, 1);
    CHECK(find_lidar_gaps(points, 0.05).empty());
}
