#include "sim3_refine.h"

#include "sim3_residuals.h"

#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace colmap_sim3
{
    using namespace detail;

    namespace
    {
        //! Point-to-plane distance of s * R * p + t to a fixed map plane.
        struct PointToPlaneResidual
        {
            PointToPlaneResidual(const Eigen::Vector3d& p, const Plane& plane, double weight)
                : p_(p)
                , mean_(plane.mean)
                , normal_(plane.normal)
                , weight_(weight)
            {
            }

            template <typename T>
            bool operator()(const T* const q, const T* const t, const T* const log_s, T* residual) const
            {
                using std::exp;
                const Eigen::Map<const Eigen::Quaternion<T>> Q(q);
                const Eigen::Map<const Vec3<T>> tt(t);
                const Vec3<T> X = exp(log_s[0]) * (Q * p_.cast<T>()) + tt;
                residual[0] = T(weight_) * normal_.cast<T>().dot(X - mean_.cast<T>());
                return true;
            }

            const Eigen::Vector3d p_;
            const Eigen::Vector3d mean_;
            const Eigen::Vector3d normal_;
            const double weight_;
        };

        //! Offset of s * R * p + t from a fixed map point.
        struct PointToPointResidual
        {
            PointToPointResidual(const Eigen::Vector3d& p, const Eigen::Vector3d& target, double weight)
                : p_(p)
                , target_(target)
                , weight_(weight)
            {
            }

            template <typename T>
            bool operator()(const T* const q, const T* const t, const T* const log_s, T* residual) const
            {
                using std::exp;
                const Eigen::Map<const Eigen::Quaternion<T>> Q(q);
                const Eigen::Map<const Vec3<T>> tt(t);
                const Vec3<T> X = exp(log_s[0]) * (Q * p_.cast<T>()) + tt;
                for (int k = 0; k < 3; k++)
                    residual[k] = T(weight_) * (X[k] - T(target_[k]));
                return true;
            }

            const Eigen::Vector3d p_;
            const Eigen::Vector3d target_;
            const double weight_;
        };

        //! Correspondence search, distance and residual for each kind of lidar map.
        template <typename Map>
        struct Matching;

        template <>
        struct Matching<LidarPlaneMap>
        {
            using Target = Plane;

            static bool nearest(const LidarPlaneMap& map, const Eigen::Vector3d& q, double max_dist, Plane& out)
            {
                return map.nearestPlane(q, max_dist, out);
            }

            static double distance(const Plane& plane, const Eigen::Vector3d& X)
            {
                return std::abs(plane.normal.dot(X - plane.mean));
            }

            static ceres::CostFunction* cost(const Eigen::Vector3d& p, const Plane& plane, double weight)
            {
                return new ceres::AutoDiffCostFunction<PointToPlaneResidual, 1, 4, 3, 1>(new PointToPlaneResidual(p, plane, weight));
            }
        };

        template <>
        struct Matching<LidarPointMap>
        {
            using Target = Eigen::Vector3d;

            static bool nearest(const LidarPointMap& map, const Eigen::Vector3d& q, double max_dist, Eigen::Vector3d& out)
            {
                return map.nearestPoint(q, max_dist, out);
            }

            static double distance(const Eigen::Vector3d& target, const Eigen::Vector3d& X)
            {
                return (X - target).norm();
            }

            static ceres::CostFunction* cost(const Eigen::Vector3d& p, const Eigen::Vector3d& target, double weight)
            {
                return new ceres::AutoDiffCostFunction<PointToPointResidual, 3, 4, 3, 1>(new PointToPointResidual(p, target, weight));
            }
        };

        double roundMaxDist(const AssociationSchedule& s, int round)
        {
            if (s.rounds <= 1 || s.max_dist_start <= 0.0 || s.max_dist_end <= 0.0)
                return s.max_dist_end;
            const double f = double(round) / double(s.rounds - 1);
            return s.max_dist_start * std::pow(s.max_dist_end / s.max_dist_start, f);
        }

        //! Nearest map target per active point (in parallel); found[i] is 0 where there is none.
        template <typename Map>
        void associate(
            const std::vector<Eigen::Vector3d>& world,
            const std::vector<char>& active,
            const Map& map,
            double max_dist,
            std::vector<typename Matching<Map>::Target>& targets,
            std::vector<char>& found)
        {
            targets.clear();
            targets.resize(world.size());
            found.assign(world.size(), 0);
            tbb::parallel_for(
                size_t(0),
                world.size(),
                [&](size_t i)
                {
                    if (active[i])
                        found[i] = Matching<Map>::nearest(map, world[i], max_dist, targets[i]) ? 1 : 0;
                });
        }

        //! Counts points within @p max_dist of a map target and their distance RMS.
        template <typename Map>
        void matchStats(
            const std::vector<Eigen::Vector3d>& world_points,
            const std::vector<char>& active,
            const Map& map,
            double max_dist,
            size_t& inliers,
            double& rms)
        {
            std::vector<typename Matching<Map>::Target> targets;
            std::vector<char> found;
            associate(world_points, active, map, max_dist, targets, found);
            inliers = 0;
            double sum_sq = 0.0;
            for (size_t i = 0; i < world_points.size(); i++)
                if (found[i])
                {
                    const double d = Matching<Map>::distance(targets[i], world_points[i]);
                    sum_sq += d * d;
                    inliers++;
                }
            rms = inliers > 0 ? std::sqrt(sum_sq / double(inliers)) : 0.0;
        }

        template <typename Map>
        bool alignSimilarityTo(
            const std::vector<Eigen::Vector3d>& points,
            const std::vector<char>& active,
            const Map& map,
            const AlignParams& params,
            Similarity& G,
            SolveStats& stats)
        {
            stats = SolveStats{};
            auto transformAll = [&points](const Similarity& g)
            {
                std::vector<Eigen::Vector3d> world(points.size());
                for (size_t i = 0; i < points.size(); i++)
                    world[i] = g.apply(points[i]);
                return world;
            };

            const double stats_dist = params.schedule.max_dist_end;
            matchStats(transformAll(G), active, map, stats_dist, stats.inliers_before, stats.rms_before);

            Sim3Block block = Sim3Block::from(G.q, G.t, G.s);
            ceres::Solver::Summary summary;
            for (int round = 0; round < std::max(1, params.schedule.rounds); round++)
            {
                Similarity current = G;
                current.q = block.quaternion();
                current.t = block.translation();
                current.s = std::exp(block.log_s);

                const double max_dist = roundMaxDist(params.schedule, round);
                std::vector<typename Matching<Map>::Target> targets;
                std::vector<char> found;
                associate(transformAll(current), active, map, max_dist, targets, found);

                ceres::Problem problem(problemOptions());
                problem.AddParameterBlock(block.q.data(), 4, new ceres::EigenQuaternionManifold);
                problem.AddParameterBlock(block.t.data(), 3);
                problem.AddParameterBlock(&block.log_s, 1);
                if (!params.estimate_scale)
                    problem.SetParameterBlockConstant(&block.log_s);

                const auto loss = std::make_unique<ceres::CauchyLoss>(std::max(1.0, 0.5 * max_dist / params.sigma_point));
                size_t used = 0;
                for (size_t i = 0; i < points.size(); i++)
                {
                    if (!found[i])
                        continue;
                    problem.AddResidualBlock(
                        Matching<Map>::cost(points[i] - G.pivot, targets[i], 1.0 / params.sigma_point),
                        loss.get(),
                        block.q.data(),
                        block.t.data(),
                        &block.log_s);
                    used++;
                }
                if (used < 10)
                {
                    stats.message = "Only " + std::to_string(used) + " points near the lidar map within " + std::to_string(max_dist) +
                        " m; move the model closer or raise the start distance.";
                    return false;
                }

                ceres::Solve(solverOptions(params.schedule.iterations, ceres::DENSE_NORMAL_CHOLESKY), &problem, &summary);
                if (!summary.IsSolutionUsable())
                {
                    stats.message = "Ceres failed: " + summary.BriefReport();
                    return false;
                }
            }

            G.q = block.quaternion();
            G.t = block.translation();
            G.s = std::exp(block.log_s);
            matchStats(transformAll(G), active, map, stats_dist, stats.inliers_after, stats.rms_after);
            stats.ok = true;
            stats.message = summary.BriefReport();
            return true;
        }

        template <typename Map>
        bool refinePerCameraTo(
            const std::vector<Eigen::Affine3d>& T_wc,
            const AnchoredPoints& points,
            const Map& map,
            const RefineParams& params,
            std::vector<CameraSim3>& cameras,
            SolveStats& stats)
        {
            stats = SolveStats{};
            const size_t n = cameras.size();
            if (n == 0 || T_wc.size() != n)
            {
                stats.message = "Camera count mismatch";
                return false;
            }

            const double stats_dist = params.schedule.max_dist_end;
            matchStats(anchoredWorldPoints(points, cameras), points.active, map, stats_dist, stats.inliers_before, stats.rms_before);

            std::vector<Sim3Block> blocks(n);
            for (size_t k = 0; k < n; k++)
                blocks[k] = Sim3Block::from(cameras[k].q, cameras[k].t, cameras[k].s);
            const std::vector<CameraSim3> start = cameras;

            // Relative motion between consecutive cameras as COLMAP reconstructed it (COLMAP units).
            std::vector<Eigen::Quaterniond> z_q(n);
            std::vector<Eigen::Vector3d> z_t(n);
            for (size_t k = 1; k < n; k++)
            {
                const Eigen::Quaterniond qi(T_wc[k - 1].linear()), qj(T_wc[k].linear());
                z_q[k] = (qi.conjugate() * qj).normalized();
                z_t[k] = T_wc[k - 1].linear().transpose() * (T_wc[k].translation() - T_wc[k - 1].translation());
            }

            auto currentCameras = [&blocks, n]()
            {
                std::vector<CameraSim3> out(n);
                for (size_t k = 0; k < n; k++)
                    out[k] = { blocks[k].quaternion(), blocks[k].translation(), std::exp(blocks[k].log_s) };
                return out;
            };

            ceres::Solver::Summary summary;
            for (int round = 0; round < std::max(1, params.schedule.rounds); round++)
            {
                const double max_dist = roundMaxDist(params.schedule, round);
                std::vector<typename Matching<Map>::Target> targets;
                std::vector<char> found;
                associate(anchoredWorldPoints(points, currentCameras()), points.active, map, max_dist, targets, found);

                ceres::Problem problem(problemOptions());
                for (auto& b : blocks)
                {
                    problem.AddParameterBlock(b.q.data(), 4, new ceres::EigenQuaternionManifold);
                    problem.AddParameterBlock(b.t.data(), 3);
                    problem.AddParameterBlock(&b.log_s, 1);
                }

                const double w_t = 1.0 / params.sigma_translation;
                const double w_r = 1.0 / (params.sigma_rotation_deg * kDegToRad);
                const double w_s = 1.0 / params.sigma_scale;
                for (size_t k = 1; k < n; k++)
                {
                    auto* cost = new ceres::AutoDiffCostFunction<SmoothnessResidual, 7, 4, 3, 1, 4, 3, 1>(
                        new SmoothnessResidual(z_q[k], z_t[k], w_t, w_r, w_s));
                    problem.AddResidualBlock(
                        cost,
                        nullptr,
                        blocks[k - 1].q.data(),
                        blocks[k - 1].t.data(),
                        &blocks[k - 1].log_s,
                        blocks[k].q.data(),
                        blocks[k].t.data(),
                        &blocks[k].log_s);
                }

                if (params.use_prior)
                    for (size_t k = 0; k < n; k++)
                    {
                        auto* cost = new ceres::AutoDiffCostFunction<PriorResidual, 6, 4, 3>(new PriorResidual(
                            start[k].q,
                            start[k].t,
                            1.0 / params.prior_sigma_translation,
                            1.0 / (params.prior_sigma_rotation_deg * kDegToRad)));
                        problem.AddResidualBlock(cost, nullptr, blocks[k].q.data(), blocks[k].t.data());
                    }

                const auto loss = std::make_unique<ceres::CauchyLoss>(std::max(1.0, 0.5 * max_dist / params.sigma_point));
                size_t used = 0;
                for (size_t i = 0; i < points.local.size(); i++)
                {
                    if (!found[i])
                        continue;
                    Sim3Block& b = blocks[points.anchor[i]];
                    problem.AddResidualBlock(
                        Matching<Map>::cost(points.local[i], targets[i], 1.0 / params.sigma_point),
                        loss.get(),
                        b.q.data(),
                        b.t.data(),
                        &b.log_s);
                    used++;
                }
                if (used < 10)
                {
                    stats.message = "Only " + std::to_string(used) + " points near the lidar map within " + std::to_string(max_dist) +
                        " m; run the global ICP first.";
                    return false;
                }

                ceres::Solve(solverOptions(params.schedule.iterations, ceres::SPARSE_NORMAL_CHOLESKY), &problem, &summary);
                if (!summary.IsSolutionUsable())
                {
                    stats.message = "Ceres failed: " + summary.BriefReport();
                    return false;
                }
            }

            cameras = currentCameras();
            matchStats(anchoredWorldPoints(points, cameras), points.active, map, stats_dist, stats.inliers_after, stats.rms_after);
            stats.ok = true;
            stats.message = summary.BriefReport();
            return true;
        }
    } // namespace

    Eigen::Affine3d Similarity::affine() const
    {
        Eigen::Affine3d m = Eigen::Affine3d::Identity();
        m.linear() = s * q.toRotationMatrix();
        m.translation() = t - m.linear() * pivot;
        return m;
    }

    Similarity Similarity::fromScaledRotation(double s, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& pivot)
    {
        Similarity G;
        G.q = Eigen::Quaterniond(R).normalized();
        G.s = s;
        G.pivot = pivot;
        G.t = s * (G.q * pivot) + t;
        return G;
    }

    void pointToPlaneStats(
        const std::vector<Eigen::Vector3d>& world_points,
        const std::vector<char>& active,
        const LidarPlaneMap& map,
        double max_dist,
        size_t& inliers,
        double& rms)
    {
        matchStats(world_points, active, map, max_dist, inliers, rms);
    }

    bool alignSimilarity(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<char>& active,
        const LidarPlaneMap& map,
        const AlignParams& params,
        Similarity& G,
        SolveStats& stats)
    {
        return alignSimilarityTo(points, active, map, params, G, stats);
    }

    bool alignSimilarity(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<char>& active,
        const LidarPointMap& map,
        const AlignParams& params,
        Similarity& G,
        SolveStats& stats)
    {
        return alignSimilarityTo(points, active, map, params, G, stats);
    }

    AnchoredPoints anchorPoints(
        const std::vector<Eigen::Affine3d>& T_wc,
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<std::vector<int>>& track_cameras,
        const std::vector<char>& active)
    {
        AnchoredPoints out;
        out.local.resize(points.size());
        out.anchor.assign(points.size(), -1);
        out.active = active;
        if (T_wc.empty())
            return out;

        tbb::parallel_for(
            size_t(0),
            points.size(),
            [&](size_t i)
            {
                double best = std::numeric_limits<double>::max();
                int anchor = -1;
                auto consider = [&](int k)
                {
                    const double d = (T_wc[k].translation() - points[i]).squaredNorm();
                    if (d < best)
                    {
                        best = d;
                        anchor = k;
                    }
                };
                for (int k : track_cameras[i])
                    if (k >= 0 && k < static_cast<int>(T_wc.size()))
                        consider(k);
                if (anchor < 0)
                    for (int k = 0; k < static_cast<int>(T_wc.size()); k++)
                        consider(k);
                out.anchor[i] = anchor;
                out.local[i] = T_wc[anchor].inverse(Eigen::Isometry) * points[i];
            });
        return out;
    }

    std::vector<CameraSim3> camerasFromSimilarity(const Similarity& G, const std::vector<Eigen::Affine3d>& T_wc)
    {
        std::vector<CameraSim3> cameras(T_wc.size());
        for (size_t k = 0; k < T_wc.size(); k++)
        {
            cameras[k].q = (G.q * Eigen::Quaterniond(T_wc[k].linear())).normalized();
            cameras[k].t = G.apply(T_wc[k].translation());
            cameras[k].s = G.s;
        }
        return cameras;
    }

    std::vector<Eigen::Vector3d> anchoredWorldPoints(const AnchoredPoints& points, const std::vector<CameraSim3>& cameras)
    {
        std::vector<Eigen::Vector3d> world(points.local.size());
        for (size_t i = 0; i < world.size(); i++)
            world[i] = cameras[points.anchor[i]].apply(points.local[i]);
        return world;
    }

    bool refinePerCamera(
        const std::vector<Eigen::Affine3d>& T_wc,
        const AnchoredPoints& points,
        const LidarPlaneMap& map,
        const RefineParams& params,
        std::vector<CameraSim3>& cameras,
        SolveStats& stats)
    {
        return refinePerCameraTo(T_wc, points, map, params, cameras, stats);
    }

    bool refinePerCamera(
        const std::vector<Eigen::Affine3d>& T_wc,
        const AnchoredPoints& points,
        const LidarPointMap& map,
        const RefineParams& params,
        std::vector<CameraSim3>& cameras,
        SolveStats& stats)
    {
        return refinePerCameraTo(T_wc, points, map, params, cameras, stats);
    }
} // namespace colmap_sim3
