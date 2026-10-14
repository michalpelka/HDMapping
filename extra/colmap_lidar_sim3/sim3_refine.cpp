#include "sim3_refine.h"

#include <ceres/ceres.h>

#include <tbb/parallel_for.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

namespace colmap_sim3
{
    namespace
    {
        constexpr double kDegToRad = M_PI / 180.0;

        template <typename T>
        using Vec3 = Eigen::Matrix<T, 3, 1>;

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

        //! Rotation vector of a near-identity quaternion (2 * vec, sign-fixed to the short way round).
        template <typename T>
        Vec3<T> smallRotationVector(const Eigen::Quaternion<T>& q)
        {
            const T sign = q.w() < T(0) ? T(-1) : T(1);
            return T(2) * sign * q.vec();
        }

        //! Keeps camera j where the SfM model put it relative to camera i, up to i's scale:
        //! translation R_i^T (t_j - t_i) vs s_i * z_t, rotation R_i^T R_j vs z_q, and log s_j vs log s_i.
        struct SmoothnessResidual
        {
            SmoothnessResidual(const Eigen::Quaterniond& z_q, const Eigen::Vector3d& z_t, double w_t, double w_r, double w_s)
                : z_q_(z_q)
                , z_t_(z_t)
                , w_t_(w_t)
                , w_r_(w_r)
                , w_s_(w_s)
            {
            }

            template <typename T>
            bool operator()(
                const T* const qi, const T* const ti, const T* const lsi, const T* const qj, const T* const tj, const T* const lsj, T* r)
                const
            {
                using std::exp;
                const Eigen::Map<const Eigen::Quaternion<T>> Qi(qi), Qj(qj);
                const Eigen::Map<const Vec3<T>> Ti(ti), Tj(tj);

                const Vec3<T> dt = Qi.conjugate() * (Tj - Ti) - exp(lsi[0]) * z_t_.cast<T>();
                const Vec3<T> dr = smallRotationVector(Eigen::Quaternion<T>(z_q_.conjugate().cast<T>() * (Qi.conjugate() * Qj)));
                for (int k = 0; k < 3; k++)
                {
                    r[k] = T(w_t_) * dt[k];
                    r[3 + k] = T(w_r_) * dr[k];
                }
                r[6] = T(w_s_) * (lsj[0] - lsi[0]);
                return true;
            }

            const Eigen::Quaterniond z_q_;
            const Eigen::Vector3d z_t_;
            const double w_t_, w_r_, w_s_;
        };

        //! Weak pull of a camera towards its starting pose.
        struct PriorResidual
        {
            PriorResidual(const Eigen::Quaterniond& q0, const Eigen::Vector3d& t0, double w_t, double w_r)
                : q0_(q0)
                , t0_(t0)
                , w_t_(w_t)
                , w_r_(w_r)
            {
            }

            template <typename T>
            bool operator()(const T* const q, const T* const t, T* r) const
            {
                const Eigen::Map<const Eigen::Quaternion<T>> Q(q);
                const Eigen::Map<const Vec3<T>> tt(t);
                const Vec3<T> dr = smallRotationVector(Eigen::Quaternion<T>(q0_.conjugate().cast<T>() * Q));
                for (int k = 0; k < 3; k++)
                {
                    r[k] = T(w_t_) * (tt[k] - T(t0_[k]));
                    r[3 + k] = T(w_r_) * dr[k];
                }
                return true;
            }

            const Eigen::Quaterniond q0_;
            const Eigen::Vector3d t0_;
            const double w_t_, w_r_;
        };

        //! Parameter blocks of one similarity, in the layout Ceres and EigenQuaternionManifold expect.
        struct Sim3Block
        {
            std::array<double, 4> q; // x, y, z, w (Eigen storage order)
            std::array<double, 3> t;
            double log_s;

            static Sim3Block from(const Eigen::Quaterniond& q, const Eigen::Vector3d& t, double s)
            {
                const Eigen::Quaterniond qn = q.normalized();
                return { { qn.x(), qn.y(), qn.z(), qn.w() }, { t.x(), t.y(), t.z() }, std::log(s) };
            }

            Eigen::Quaterniond quaternion() const
            {
                return Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized();
            }

            Eigen::Vector3d translation() const
            {
                return Eigen::Vector3d(t[0], t[1], t[2]);
            }
        };

        double roundMaxDist(const AssociationSchedule& s, int round)
        {
            if (s.rounds <= 1 || s.max_dist_start <= 0.0 || s.max_dist_end <= 0.0)
                return s.max_dist_end;
            const double f = double(round) / double(s.rounds - 1);
            return s.max_dist_start * std::pow(s.max_dist_end / s.max_dist_start, f);
        }

        //! Nearest map plane per active point (in parallel); found[i] is 0 where there is none.
        void associate(
            const std::vector<Eigen::Vector3d>& world,
            const std::vector<char>& active,
            const LidarPlaneMap& map,
            double max_dist,
            std::vector<Plane>& planes,
            std::vector<char>& found)
        {
            planes.assign(world.size(), Plane{});
            found.assign(world.size(), 0);
            tbb::parallel_for(
                size_t(0),
                world.size(),
                [&](size_t i)
                {
                    if (active[i])
                        found[i] = map.nearestPlane(world[i], max_dist, planes[i]) ? 1 : 0;
                });
        }

        ceres::Solver::Options solverOptions(int iterations, ceres::LinearSolverType type)
        {
            ceres::Solver::Options options;
            options.linear_solver_type = type;
            options.max_num_iterations = iterations;
            options.num_threads = std::max(1u, std::thread::hardware_concurrency());
            options.logging_type = ceres::SILENT;
            return options;
        }

        ceres::Problem::Options problemOptions()
        {
            // One loss function is shared by all point residuals of a round and owned by the caller.
            ceres::Problem::Options options;
            options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            return options;
        }
    } // namespace

    Eigen::Affine3d Similarity::affine() const
    {
        Eigen::Affine3d m = Eigen::Affine3d::Identity();
        m.linear() = s * q.toRotationMatrix();
        m.translation() = t - m.linear() * pivot;
        return m;
    }

    void pointToPlaneStats(
        const std::vector<Eigen::Vector3d>& world_points,
        const std::vector<char>& active,
        const LidarPlaneMap& map,
        double max_dist,
        size_t& inliers,
        double& rms)
    {
        std::vector<Plane> planes;
        std::vector<char> found;
        associate(world_points, active, map, max_dist, planes, found);
        inliers = 0;
        double sum_sq = 0.0;
        for (size_t i = 0; i < world_points.size(); i++)
            if (found[i])
            {
                const double d = planes[i].normal.dot(world_points[i] - planes[i].mean);
                sum_sq += d * d;
                inliers++;
            }
        rms = inliers > 0 ? std::sqrt(sum_sq / double(inliers)) : 0.0;
    }

    bool alignSimilarity(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<char>& active,
        const LidarPlaneMap& map,
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
        pointToPlaneStats(transformAll(G), active, map, stats_dist, stats.inliers_before, stats.rms_before);

        Sim3Block block = Sim3Block::from(G.q, G.t, G.s);
        ceres::Solver::Summary summary;
        for (int round = 0; round < std::max(1, params.schedule.rounds); round++)
        {
            Similarity current = G;
            current.q = block.quaternion();
            current.t = block.translation();
            current.s = std::exp(block.log_s);

            const double max_dist = roundMaxDist(params.schedule, round);
            std::vector<Plane> planes;
            std::vector<char> found;
            associate(transformAll(current), active, map, max_dist, planes, found);

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
                auto* cost = new ceres::AutoDiffCostFunction<PointToPlaneResidual, 1, 4, 3, 1>(
                    new PointToPlaneResidual(points[i] - G.pivot, planes[i], 1.0 / params.sigma_point));
                problem.AddResidualBlock(cost, loss.get(), block.q.data(), block.t.data(), &block.log_s);
                used++;
            }
            if (used < 10)
            {
                stats.message = "Only " + std::to_string(used) + " points near a lidar plane within " + std::to_string(max_dist) +
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
        pointToPlaneStats(transformAll(G), active, map, stats_dist, stats.inliers_after, stats.rms_after);
        stats.ok = true;
        stats.message = summary.BriefReport();
        return true;
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
        stats = SolveStats{};
        const size_t n = cameras.size();
        if (n == 0 || T_wc.size() != n)
        {
            stats.message = "Camera count mismatch";
            return false;
        }

        const double stats_dist = params.schedule.max_dist_end;
        pointToPlaneStats(anchoredWorldPoints(points, cameras), points.active, map, stats_dist, stats.inliers_before, stats.rms_before);

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
            std::vector<Plane> planes;
            std::vector<char> found;
            associate(anchoredWorldPoints(points, currentCameras()), points.active, map, max_dist, planes, found);

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
                        start[k].q, start[k].t, 1.0 / params.prior_sigma_translation, 1.0 / (params.prior_sigma_rotation_deg * kDegToRad)));
                    problem.AddResidualBlock(cost, nullptr, blocks[k].q.data(), blocks[k].t.data());
                }

            const auto loss = std::make_unique<ceres::CauchyLoss>(std::max(1.0, 0.5 * max_dist / params.sigma_point));
            size_t used = 0;
            for (size_t i = 0; i < points.local.size(); i++)
            {
                if (!found[i])
                    continue;
                Sim3Block& b = blocks[points.anchor[i]];
                auto* cost = new ceres::AutoDiffCostFunction<PointToPlaneResidual, 1, 4, 3, 1>(
                    new PointToPlaneResidual(points.local[i], planes[i], 1.0 / params.sigma_point));
                problem.AddResidualBlock(cost, loss.get(), b.q.data(), b.t.data(), &b.log_s);
                used++;
            }
            if (used < 10)
            {
                stats.message = "Only " + std::to_string(used) + " points near a lidar plane within " + std::to_string(max_dist) +
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
        pointToPlaneStats(anchoredWorldPoints(points, cameras), points.active, map, stats_dist, stats.inliers_after, stats.rms_after);
        stats.ok = true;
        stats.message = summary.BriefReport();
        return true;
    }
} // namespace colmap_sim3
