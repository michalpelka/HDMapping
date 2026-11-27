#pragma once

#include <Eigen/Geometry>
#include <vector>

class Session;

//! Rigid offset between two instruments carried on one rig (e.g. a camera and a lidar), from their paths:
//! frame * S(C_i) = L(t_i + time_shift) * offset for every instrument pose C_i, L = reference path, S = instrument
//! translation scaled by scale. Observation equation: path_to_path_tait_bryan_wc (observation_equations).
namespace instrument_offset
{
    struct TimedPose
    {
        double time_s = 0.0;
        Eigen::Affine3d pose = Eigen::Affine3d::Identity(); // world <- body
    };

    struct Options
    {
        bool estimate_time_shift = true;
        bool estimate_frame = false; //!< false: both sessions are registered in one world
        bool estimate_scale = false;
        int iterations = 30;
    };

    struct Result
    {
        bool valid = false;
        Eigen::Affine3d offset = Eigen::Affine3d::Identity(); //!< instrument pose in the reference body frame
        Eigen::Affine3d frame = Eigen::Affine3d::Identity(); //!< reference world <- instrument path world
        double time_shift_s = 0.0; //!< added to the instrument timestamps
        double scale = 1.0; //!< applied to the instrument path translations
        int pairs = 0;
        double rms_translation_m = 0.0;
        double rms_rotation_deg = 0.0;
    };

    //! World poses of every trajectory node of a session (scan pose * local node pose), sorted by time.
    //! Node timestamps are read as nanoseconds (the trajectory_lio_N.csv convention).
    std::vector<TimedPose> session_trajectory(const Session& session);

    //! Levenberg-Marquardt fit of the offset (and optionally the time shift, frame and scale).
    //! @param reference - sorted by time, e.g. the lidar session
    //! @param instrument - sorted by time, e.g. the camera session
    //! @param time_shift_s - initial time shift, added to the instrument timestamps
    Result estimate(
        const std::vector<TimedPose>& reference, const std::vector<TimedPose>& instrument, double time_shift_s, const Options& options);
} // namespace instrument_offset
