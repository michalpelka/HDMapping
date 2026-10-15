#pragma once

// Ceres pieces shared by the geometric (sim3_refine.cpp) and photometric (photometric_refine.cpp)
// per-camera refinements. Internal: include only from .cpp files that already use Ceres.

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <thread>

namespace colmap_sim3::detail
{
    constexpr double kDegToRad = M_PI / 180.0;

    template <typename T>
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    //! Rotation vector of a near-identity quaternion (2 * vec, sign-fixed to the short way round).
    template <typename T>
    Vec3<T> smallRotationVector(const Eigen::Quaternion<T>& q)
    {
        const T sign = q.w() < T(0) ? T(-1) : T(1);
        return T(2) * sign * q.vec();
    }

    //! Keeps camera j where the reference put it relative to camera i, up to i's scale:
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
            const T* const qi, const T* const ti, const T* const lsi, const T* const qj, const T* const tj, const T* const lsj, T* r) const
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

    inline ceres::Solver::Options solverOptions(int iterations, ceres::LinearSolverType type)
    {
        ceres::Solver::Options options;
        options.linear_solver_type = type;
        options.max_num_iterations = iterations;
        options.num_threads = std::max(1u, std::thread::hardware_concurrency());
        options.logging_type = ceres::SILENT;
        return options;
    }

    inline ceres::Problem::Options problemOptions()
    {
        // One loss function is shared by all point residuals of a round and owned by the caller.
        ceres::Problem::Options options;
        options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        return options;
    }
} // namespace colmap_sim3::detail