#include <Core/instrument_offset.h>

#include <Core/m_estimators.h>
#include <Core/session.h>
#include <Core/transformations.h>
#include <HDMapping/PoseInterpolation.h>

#include <python-scripts/constraints/path_to_path_tait_bryan_wc_jacobian.h>

#include <algorithm>
#include <map>

namespace instrument_offset
{
    namespace
    {
        // reference pose at t; false outside the path or across a hole in it
        bool reference_pose(const std::map<double, Eigen::Matrix4d>& reference, double t, Eigen::Affine3d& pose)
        {
            auto next = reference.lower_bound(t);
            if (next == reference.begin() || next == reference.end() || next->first - std::prev(next)->first > 0.1)
                return false;
            pose = Eigen::Affine3d(getInterpolatedPose(reference, t));
            return true;
        }
    } // namespace

    std::vector<TimedPose> session_trajectory(const Session& session)
    {
        std::vector<TimedPose> trajectory;
        for (const auto& pc : session.point_clouds_container.point_clouds)
            for (const auto& node : pc.local_trajectory)
                trajectory.push_back({ node.timestamps.first * 1e-9, pc.m_pose * node.m_pose });
        std::sort(
            trajectory.begin(),
            trajectory.end(),
            [](const TimedPose& a, const TimedPose& b)
            {
                return a.time_s < b.time_s;
            });
        return trajectory;
    }

    Result estimate(
        const std::vector<TimedPose>& reference_path, const std::vector<TimedPose>& instrument, double time_shift_s, const Options& options)
    {
        std::map<double, Eigen::Matrix4d> reference;
        for (const auto& node : reference_path)
            reference[node.time_s] = node.pose.matrix();

        // start: offset seen by the first pair, frame identity, scale 1
        TaitBryanPose offset;
        TaitBryanPose frame;
        double scale = 1.0;
        for (const auto& node : instrument)
        {
            Eigen::Affine3d l;
            if (reference_pose(reference, node.time_s + time_shift_s, l))
            {
                offset = pose_tait_bryan_from_affine_matrix(l.inverse() * node.pose);
                break;
            }
        }

        const double rate_dt = 0.01; // reference rate of change from poses 10 ms before and after
        const double w_t = 1.0 / (0.01 * 0.01); // 1 cm
        const double w_r = 1.0 / ((0.2 * M_PI / 180.0) * (0.2 * M_PI / 180.0)); // 0.2 deg
        const double lm_lambda = 1.0;

        for (int iter = 0; iter < options.iterations; iter++)
        {
            Eigen::Matrix<double, 14, 14> AtPA = Eigen::Matrix<double, 14, 14>::Zero();
            Eigen::Matrix<double, 14, 1> AtPB = Eigen::Matrix<double, 14, 1>::Zero();

            for (const auto& node : instrument)
            {
                const double t = node.time_s + time_shift_s;
                Eigen::Affine3d l, l_before, l_after;
                if (!reference_pose(reference, t, l) || !reference_pose(reference, t - rate_dt, l_before) ||
                    !reference_pose(reference, t + rate_dt, l_after))
                    continue;

                const TaitBryanPose r = pose_tait_bryan_from_affine_matrix(l);
                const TaitBryanPose a = pose_tait_bryan_from_affine_matrix(l_before);
                const TaitBryanPose b = pose_tait_bryan_from_affine_matrix(l_after);
                const TaitBryanPose m = pose_tait_bryan_from_affine_matrix(node.pose);
                TaitBryanPose v;
                v.px = (b.px - a.px) / (2 * rate_dt);
                v.py = (b.py - a.py) / (2 * rate_dt);
                v.pz = (b.pz - a.pz) / (2 * rate_dt);
                v.om = normalize_angle(b.om - a.om) / (2 * rate_dt);
                v.fi = normalize_angle(b.fi - a.fi) / (2 * rate_dt);
                v.ka = normalize_angle(b.ka - a.ka) / (2 * rate_dt);

                Eigen::Matrix<double, 6, 1> delta;
                Eigen::Matrix<double, 6, 14, Eigen::RowMajor> jacobian;
                path_to_path_obs_eq_tait_bryan_wc(
                    delta,
                    r.px,
                    r.py,
                    r.pz,
                    r.om,
                    r.fi,
                    r.ka,
                    v.px,
                    v.py,
                    v.pz,
                    v.om,
                    v.fi,
                    v.ka,
                    offset.px,
                    offset.py,
                    offset.pz,
                    offset.om,
                    offset.fi,
                    offset.ka,
                    frame.px,
                    frame.py,
                    frame.pz,
                    frame.om,
                    frame.fi,
                    frame.ka,
                    0.0,
                    scale,
                    m.px,
                    m.py,
                    m.pz,
                    m.om,
                    m.fi,
                    m.ka);
                path_to_path_obs_eq_tait_bryan_wc_jacobian(
                    jacobian,
                    r.px,
                    r.py,
                    r.pz,
                    r.om,
                    r.fi,
                    r.ka,
                    v.px,
                    v.py,
                    v.pz,
                    v.om,
                    v.fi,
                    v.ka,
                    offset.px,
                    offset.py,
                    offset.pz,
                    offset.om,
                    offset.fi,
                    offset.ka,
                    frame.px,
                    frame.py,
                    frame.pz,
                    frame.om,
                    frame.fi,
                    frame.ka,
                    0.0,
                    scale,
                    m.px,
                    m.py,
                    m.pz,
                    m.om,
                    m.fi,
                    m.ka);
                for (int k = 3; k < 6; k++)
                    delta(k) = normalize_angle(delta(k));

                Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
                for (int k = 0; k < 6; k++)
                    P(k, k) = (k < 3 ? w_t : w_r) * get_cauchy_w(delta(k), 1);

                const Eigen::Matrix<double, 6, 14> A = -jacobian;
                AtPA += A.transpose() * P * A;
                AtPB += A.transpose() * P * delta;
            }

            // unknowns that are not estimated stay where they are
            std::vector<int> fixed;
            if (!options.estimate_frame)
                fixed.insert(fixed.end(), { 6, 7, 8, 9, 10, 11 });
            if (!options.estimate_time_shift)
                fixed.push_back(12);
            if (!options.estimate_scale)
                fixed.push_back(13);
            for (int k : fixed)
            {
                AtPA.row(k).setZero();
                AtPA.col(k).setZero();
                AtPA(k, k) = 1.0;
                AtPB(k) = 0.0;
            }
            AtPA += lm_lambda * Eigen::Matrix<double, 14, 14>::Identity();

            const Eigen::Matrix<double, 14, 1> x = AtPA.ldlt().solve(AtPB);
            if (!x.allFinite())
                break;
            offset.px += x(0);
            offset.py += x(1);
            offset.pz += x(2);
            offset.om += x(3);
            offset.fi += x(4);
            offset.ka += x(5);
            frame.px += x(6);
            frame.py += x(7);
            frame.pz += x(8);
            frame.om += x(9);
            frame.fi += x(10);
            frame.ka += x(11);
            time_shift_s += x(12);
            scale += x(13);
        }

        Result result;
        result.offset = affine_matrix_from_pose_tait_bryan(offset);
        result.frame = affine_matrix_from_pose_tait_bryan(frame);
        result.time_shift_s = time_shift_s;
        result.scale = scale;

        double sum_t = 0.0, sum_r = 0.0;
        for (const auto& node : instrument)
        {
            Eigen::Affine3d l;
            if (!reference_pose(reference, node.time_s + time_shift_s, l))
                continue;
            Eigen::Affine3d c = node.pose;
            c.translation() *= scale;
            const Eigen::Affine3d e = (result.frame * c).inverse() * l * result.offset;
            sum_t += e.translation().squaredNorm();
            sum_r += std::pow(Eigen::AngleAxisd(e.linear()).angle() * 180.0 / M_PI, 2);
            result.pairs++;
        }
        if (result.pairs > 0)
        {
            result.rms_translation_m = std::sqrt(sum_t / result.pairs);
            result.rms_rotation_deg = std::sqrt(sum_r / result.pairs);
        }
        result.valid = result.pairs >= 3 && result.offset.matrix().allFinite();
        return result;
    }
} // namespace instrument_offset
