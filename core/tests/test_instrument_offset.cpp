#include <doctest.h>

#include <Core/instrument_offset.h>

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

using instrument_offset::TimedPose;

namespace
{
    // smooth rig motion: translating and turning about all three axes
    Eigen::Affine3d rig_pose(double t)
    {
        Eigen::Affine3d m = Eigen::Affine3d::Identity();
        m.translation() = Eigen::Vector3d(0.5 * std::sin(0.7 * t), 0.3 * std::cos(0.5 * t), 0.1 * t);
        m.linear() =
            (Eigen::AngleAxisd(0.5 * t, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.2 * std::sin(0.3 * t), Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0.3 * std::sin(0.4 * t), Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        return m;
    }

    Eigen::Affine3d make_pose(double x, double y, double z, double rz, double rx)
    {
        Eigen::Affine3d m = Eigen::Affine3d::Identity();
        m.translation() = Eigen::Vector3d(x, y, z);
        m.linear() = (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())).toRotationMatrix();
        return m;
    }

    std::vector<TimedPose> reference_path()
    {
        std::vector<TimedPose> r;
        for (int i = 0; i <= 2000; i++) // 200 Hz, 10 s
            r.push_back({ i * 0.005, rig_pose(i * 0.005) });
        return r;
    }

    // 50 Hz, off the reference sample times: frame * S(C) = L(t) * offset, the instrument clock reads clock_late_s late
    std::vector<TimedPose> instrument_path(const Eigen::Affine3d& offset, double clock_late_s, const Eigen::Affine3d& frame, double scale)
    {
        std::vector<TimedPose> c;
        for (int i = 10; i < 490; i++)
        {
            const double t = i * 0.02 + 0.003;
            Eigen::Affine3d pose = frame.inverse() * rig_pose(t) * offset;
            pose.translation() /= scale;
            c.push_back({ t + clock_late_s, pose });
        }
        return c;
    }

    double rotation_error_deg(const Eigen::Affine3d& a, const Eigen::Affine3d& b)
    {
        return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle() * 180.0 / M_PI;
    }

    const Eigen::Affine3d known_offset = make_pose(0.12, -0.05, 0.31, 1.2, -0.4);
} // namespace

TEST_CASE("instrument_offset::estimate recovers the offset of two registered paths")
{
    instrument_offset::Options options;
    options.estimate_time_shift = false;
    const auto r =
        instrument_offset::estimate(reference_path(), instrument_path(known_offset, 0.0, Eigen::Affine3d::Identity(), 1.0), 0.0, options);

    REQUIRE(r.valid);
    CHECK(r.pairs == 480);
    CHECK((r.offset.translation() - known_offset.translation()).norm() < 1e-4);
    CHECK(rotation_error_deg(r.offset, known_offset) < 0.01);
}

TEST_CASE("instrument_offset::estimate recovers the time shift")
{
    const auto r = instrument_offset::estimate(
        reference_path(), instrument_path(known_offset, 0.020, Eigen::Affine3d::Identity(), 1.0), 0.0, instrument_offset::Options{});

    REQUIRE(r.valid);
    CHECK(std::abs(r.time_shift_s - (-0.020)) < 0.0005);
    CHECK((r.offset.translation() - known_offset.translation()).norm() < 1e-3);
}

TEST_CASE("instrument_offset::estimate recovers the frame and scale of the instrument path")
{
    const Eigen::Affine3d frame = make_pose(0.05, -0.02, 0.03, 0.05, 0.02);
    instrument_offset::Options options;
    options.estimate_time_shift = false;
    options.estimate_frame = true;
    options.estimate_scale = true;
    const auto r = instrument_offset::estimate(reference_path(), instrument_path(known_offset, 0.0, frame, 1.02), 0.0, options);

    REQUIRE(r.valid);
    CHECK(std::abs(r.scale - 1.02) < 1e-4);
    CHECK((r.frame.translation() - frame.translation()).norm() < 1e-3);
    CHECK((r.offset.translation() - known_offset.translation()).norm() < 1e-3);
    CHECK(rotation_error_deg(r.offset, known_offset) < 0.01);
}

TEST_CASE("instrument_offset::estimate is invalid without time overlap")
{
    const auto r = instrument_offset::estimate(
        reference_path(), instrument_path(known_offset, 100.0, Eigen::Affine3d::Identity(), 1.0), 0.0, instrument_offset::Options{});
    CHECK_FALSE(r.valid);
}
