#include <doctest.h>

#include "sim3_refine.h"

#include <Eigen/Geometry>

#include <random>

using namespace colmap_sim3;

namespace
{
    // A 100 m street: ground, two walls 6 m off the centre line and thin plates across the walls
    // every 10 m (so travel along the street is observable). Cameras drive down the middle.
    constexpr double kLength = 100.0;

    struct Surface
    {
        Eigen::Vector3d origin, u, v; // points origin + a*u + b*v, a, b in [0, 1]
    };

    std::vector<Surface> streetSurfaces()
    {
        std::vector<Surface> s;
        s.push_back({ { 0, -5, 0 }, { kLength, 0, 0 }, { 0, 10, 0 } }); // ground
        s.push_back({ { 0, 6, 0 }, { kLength, 0, 0 }, { 0, 0, 5 } }); // left wall
        s.push_back({ { 0, -6, 0 }, { kLength, 0, 0 }, { 0, 0, 5 } }); // right wall
        for (double x = 5; x < kLength; x += 10)
        {
            s.push_back({ { x, 4, 0 }, { 0, 1.9, 0 }, { 0, 0, 4 } });
            s.push_back({ { x, -5.9, 0 }, { 0, 1.9, 0 }, { 0, 0, 4 } });
        }
        return s;
    }

    std::vector<Eigen::Vector3d> sampleLidarMap(const std::vector<Surface>& surfaces)
    {
        std::vector<Eigen::Vector3d> pts;
        for (const auto& s : surfaces)
        {
            const int nu = std::max(2, static_cast<int>(s.u.norm() / 0.1));
            const int nv = std::max(2, static_cast<int>(s.v.norm() / 0.1));
            for (int i = 0; i <= nu; i++)
                for (int j = 0; j <= nv; j++)
                    pts.push_back(s.origin + s.u * (double(i) / nu) + s.v * (double(j) / nv));
        }
        return pts;
    }

    //! Random points on the surfaces within +-3 m (along the street) of x.
    std::vector<Eigen::Vector3d> sampleNear(const std::vector<Surface>& surfaces, double x, std::mt19937& rng)
    {
        std::uniform_real_distribution<double> uni(0.0, 1.0);
        std::vector<Eigen::Vector3d> pts;
        for (const auto& s : surfaces)
        {
            const bool along = s.u.x() > 1.0;
            if (!along && std::abs(s.origin.x() - x) > 3.0)
                continue;
            const int count = along ? 15 : 10;
            for (int k = 0; k < count; k++)
            {
                double a = uni(rng);
                if (along)
                    a = std::clamp((x - 3.0 + 6.0 * a) / kLength, 0.0, 1.0);
                pts.push_back(s.origin + s.u * a + s.v * uni(rng));
            }
        }
        return pts;
    }

    struct Scene
    {
        std::vector<Eigen::Vector3d> lidar;
        std::vector<Eigen::Affine3d> true_T_wc; // world (lidar) frame
        std::vector<Eigen::Affine3d> sfm_T_wc; // distorted COLMAP frame
        std::vector<double> true_sfm_to_world_scale; // per camera
        std::vector<Eigen::Vector3d> sfm_points;
        std::vector<std::vector<int>> tracks;
    };

    //! Bends (yaw and pitch drifting along the street) and scales (drifting +-alpha) the SfM model
    //! camera by camera, then maps it into an arbitrary COLMAP frame.
    Scene makeBananaScene(double yaw_drift, double pitch_drift, double alpha)
    {
        Scene sc;
        const auto surfaces = streetSurfaces();
        sc.lidar = sampleLidarMap(surfaces);

        Eigen::Matrix3d R_look;
        R_look << 0, 0, 1, -1, 0, 0, 0, -1, 0; // camera z forward along +x, y down
        const int n = 61;
        const int mid = n / 2;
        std::vector<double> s(n);
        std::vector<Eigen::Matrix3d> R(n);
        for (int k = 0; k < n; k++)
        {
            Eigen::Affine3d T = Eigen::Affine3d::Identity();
            T.linear() = R_look;
            T.translation() = Eigen::Vector3d(5.0 + 1.5 * k, 0.0, 1.5);
            sc.true_T_wc.push_back(T);

            const double f = (T.translation().x() - kLength / 2) / (kLength / 2);
            s[k] = 1.0 + alpha * f;
            R[k] = (Eigen::AngleAxisd(yaw_drift * f, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch_drift * f, Eigen::Vector3d::UnitY()))
                       .toRotationMatrix();
        }

        // Distorted camera centres, integrated outwards from the undistorted middle camera.
        std::vector<Eigen::Vector3d> C(n);
        C[mid] = sc.true_T_wc[mid].translation();
        for (int k = mid + 1; k < n; k++)
            C[k] = C[k - 1] + s[k - 1] * R[k - 1] * (sc.true_T_wc[k].translation() - sc.true_T_wc[k - 1].translation());
        for (int k = mid - 1; k >= 0; k--)
            C[k] = C[k + 1] + s[k + 1] * R[k + 1] * (sc.true_T_wc[k].translation() - sc.true_T_wc[k + 1].translation());

        // Arbitrary COLMAP frame.
        const double s_q = 0.37;
        const Eigen::Matrix3d R_q = Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.3, 0.5, 0.8).normalized()).toRotationMatrix();
        const Eigen::Vector3d t_q(5, -3, 2);
        auto Q = [&](const Eigen::Vector3d& p) -> Eigen::Vector3d
        {
            return s_q * R_q * p + t_q;
        };

        std::mt19937 rng(7);
        std::normal_distribution<double> noise(0.0, 0.002);
        for (int k = 0; k < n; k++)
        {
            Eigen::Affine3d T = Eigen::Affine3d::Identity();
            T.linear() = R_q * R[k] * R_look;
            T.translation() = Q(C[k]);
            sc.sfm_T_wc.push_back(T);
            sc.true_sfm_to_world_scale.push_back(1.0 / (s_q * s[k]));

            for (const auto& p : sampleNear(surfaces, sc.true_T_wc[k].translation().x(), rng))
            {
                const Eigen::Vector3d distorted = s[k] * R[k] * (p - sc.true_T_wc[k].translation()) + C[k];
                sc.sfm_points.push_back(Q(distorted) + Eigen::Vector3d(noise(rng), noise(rng), noise(rng)));
                std::vector<int> track;
                for (int j = std::max(0, k - 2); j <= std::min(n - 1, k + 2); j++)
                    track.push_back(j);
                sc.tracks.push_back(track);
            }
        }
        return sc;
    }

    double centreRms(const std::vector<CameraSim3>& cams, const std::vector<Eigen::Affine3d>& truth)
    {
        double sum = 0.0;
        for (size_t k = 0; k < cams.size(); k++)
            sum += (cams[k].t - truth[k].translation()).squaredNorm();
        return std::sqrt(sum / double(cams.size()));
    }

    //! Umeyama fit of the SfM camera centres to the true ones, perturbed, in pivot form.
    Similarity roughInitialGuess(const Scene& sc)
    {
        Eigen::Matrix3Xd src(3, sc.sfm_T_wc.size()), dst(3, sc.true_T_wc.size());
        for (size_t k = 0; k < sc.sfm_T_wc.size(); k++)
        {
            src.col(k) = sc.sfm_T_wc[k].translation();
            dst.col(k) = sc.true_T_wc[k].translation();
        }
        const Eigen::Matrix4d m = Eigen::umeyama(src, dst, true);
        const double scale = m.block<3, 3>(0, 0).col(0).norm();

        Similarity G;
        for (const auto& p : sc.sfm_points)
            G.pivot += p;
        G.pivot /= double(sc.sfm_points.size());
        G.s = scale * 1.02;
        G.q = Eigen::Quaterniond(Eigen::AngleAxisd(1.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ())) *
            Eigen::Quaterniond(Eigen::Matrix3d(m.block<3, 3>(0, 0) / scale));
        G.t = m.block<3, 3>(0, 0) * G.pivot + m.block<3, 1>(0, 3) + Eigen::Vector3d(0.3, -0.2, 0.1);
        return G;
    }
} // namespace

TEST_CASE("similarity affine matches apply")
{
    Similarity G;
    G.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 1, 0).normalized()));
    G.t = Eigen::Vector3d(1, 2, 3);
    G.s = 2.5;
    G.pivot = Eigen::Vector3d(-4, 5, 6);
    const Eigen::Vector3d p(0.3, -0.7, 9);
    CHECK((G.affine() * p - G.apply(p)).norm() < 1e-12);
}

TEST_CASE("similarity from a scaled rotation keeps its pivot form")
{
    const Eigen::Matrix3d R = Eigen::AngleAxisd(1.2, Eigen::Vector3d(0.2, 0.9, -0.3).normalized()).toRotationMatrix();
    const Eigen::Vector3d t(15.0, -23.9, 1.7);
    const Eigen::Vector3d pivot(0.4, -0.2, 3.0);
    const Similarity G = Similarity::fromScaledRotation(2.95, R, t, pivot);
    CHECK(G.pivot == pivot);
    for (const Eigen::Vector3d& p : { Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, -2, 3), pivot })
        CHECK((G.apply(p) - (2.95 * R * p + t)).norm() < 1e-12);
}

TEST_CASE("cameras from a similarity reproduce the transformed points")
{
    Similarity G;
    G.q = Eigen::Quaterniond(Eigen::AngleAxisd(-0.9, Eigen::Vector3d(0, 1, 1).normalized()));
    G.t = Eigen::Vector3d(10, -2, 0.5);
    G.s = 0.8;
    G.pivot = Eigen::Vector3d(1, 1, 1);

    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.linear() = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()).toRotationMatrix();
    T.translation() = Eigen::Vector3d(2, 3, 4);
    const std::vector<Eigen::Affine3d> T_wc{ T };

    const std::vector<Eigen::Vector3d> pts{ { 5, 6, 7 }, { -1, 0, 2 } };
    const AnchoredPoints anchored = anchorPoints(T_wc, pts, { { 0 }, {} }, { 1, 1 });
    const auto world = anchoredWorldPoints(anchored, camerasFromSimilarity(G, T_wc));
    for (size_t i = 0; i < pts.size(); i++)
        CHECK((world[i] - G.apply(pts[i])).norm() < 1e-12);
}

TEST_CASE("global ICP recovers a similarity on an undistorted model")
{
    const Scene sc = makeBananaScene(0.0, 0.0, 0.0);
    LidarPlaneMap map;
    map.build(sc.lidar, 0.5);

    Similarity G = roughInitialGuess(sc);
    AlignParams params;
    params.schedule = { 1.0, 0.1, 6, 30 };
    SolveStats stats;
    REQUIRE_MESSAGE(alignSimilarity(sc.sfm_points, std::vector<char>(sc.sfm_points.size(), 1), map, params, G, stats), stats.message);

    CHECK(stats.rms_after < stats.rms_before);
    CHECK(G.s == doctest::Approx(sc.true_sfm_to_world_scale[0]).epsilon(0.002));
    CHECK(centreRms(camerasFromSimilarity(G, sc.sfm_T_wc), sc.true_T_wc) < 0.02);
}

TEST_CASE("per-camera refinement unbends a banana")
{
    const Scene sc = makeBananaScene(0.04, 0.02, 0.05);
    LidarPlaneMap map;
    map.build(sc.lidar, 0.5);
    const std::vector<char> active(sc.sfm_points.size(), 1);

    Similarity G = roughInitialGuess(sc);
    AlignParams align;
    align.schedule = { 1.0, 0.3, 6, 30 };
    SolveStats stats;
    REQUIRE_MESSAGE(alignSimilarity(sc.sfm_points, active, map, align, G, stats), stats.message);
    const double rms_global = centreRms(camerasFromSimilarity(G, sc.sfm_T_wc), sc.true_T_wc);

    std::vector<CameraSim3> cams = camerasFromSimilarity(G, sc.sfm_T_wc);
    const AnchoredPoints anchored = anchorPoints(sc.sfm_T_wc, sc.sfm_points, sc.tracks, active);
    RefineParams refine;
    refine.schedule = { 1.0, 0.1, 6, 30 };
    REQUIRE_MESSAGE(refinePerCamera(sc.sfm_T_wc, anchored, map, refine, cams, stats), stats.message);
    const double rms_refined = centreRms(cams, sc.true_T_wc);

    MESSAGE("camera centre RMS: global " << rms_global << " m, refined " << rms_refined << " m");
    CHECK(rms_global > 0.2);
    CHECK(rms_refined < 0.05);
    CHECK(stats.rms_after < stats.rms_before);

    // The recovered scale follows the injected drift (relative to the middle camera, which has none).
    const size_t mid = cams.size() / 2;
    for (size_t k = 0; k < cams.size(); k += 10)
    {
        const double expected = sc.true_sfm_to_world_scale[k] / sc.true_sfm_to_world_scale[mid];
        CHECK(cams[k].s / cams[mid].s == doctest::Approx(expected).epsilon(0.02));
    }
}

TEST_CASE("repeated refinement reaches a banana that starts out of reach")
{
    // Ends start several metres off, beyond the 1 m start distance; each pass pulls them closer.
    const Scene sc = makeBananaScene(0.2, 0.05, 0.15);
    LidarPlaneMap map;
    map.build(sc.lidar, 0.5);
    const std::vector<char> active(sc.sfm_points.size(), 1);

    Similarity G = roughInitialGuess(sc);
    AlignParams align;
    align.schedule = { 1.0, 0.3, 6, 30 };
    SolveStats stats;
    REQUIRE_MESSAGE(alignSimilarity(sc.sfm_points, active, map, align, G, stats), stats.message);

    std::vector<CameraSim3> cams = camerasFromSimilarity(G, sc.sfm_T_wc);
    const AnchoredPoints anchored = anchorPoints(sc.sfm_T_wc, sc.sfm_points, sc.tracks, active);
    const RefineParams refine;
    REQUIRE_MESSAGE(refinePerCamera(sc.sfm_T_wc, anchored, map, refine, cams, stats), stats.message);
    const double rms_one_pass = centreRms(cams, sc.true_T_wc);
    for (int pass = 1; pass < 3; pass++)
        REQUIRE_MESSAGE(refinePerCamera(sc.sfm_T_wc, anchored, map, refine, cams, stats), stats.message);
    const double rms_three_passes = centreRms(cams, sc.true_T_wc);

    MESSAGE("camera centre RMS: one pass " << rms_one_pass << " m, three passes " << rms_three_passes << " m");
    CHECK(rms_one_pass > 0.1);
    CHECK(rms_three_passes < 0.05);
}
