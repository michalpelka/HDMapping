#pragma once

#include <Core/structures.h>

#include <Eigen/Eigen>
#include <vector>

#include <limits>

//! Residuals and pose covariance behind an ICP uncertainty estimate.
struct IcpResidualStatistics
{
    double sum_squared_residuals = 0.0; //!< [m^2]
    int number_of_correspondences = 0;
    int number_of_observations = 0; //!< 3 per correspondence for point-to-point, 1 for point-to-plane
    //! Standard deviation of unit weight, sqrt(sum_squared_residuals / (number_of_observations - 6)) [m]; NaN when
    //! there are not more observations than the 6 pose parameters.
    double sigma0 = std::numeric_limits<double>::quiet_NaN();
    //! Pose covariance (I - J) Q_ini (I - J)^T, rows and columns px, py, pz, om, fi, ka ([m^2], [m*rad], [rad^2]); NaN
    //! when it is not computed (too few correspondences).
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Constant(std::numeric_limits<double>::quiet_NaN());
};

//! Covariance of a pose offset drawn uniformly from [-offset_m, offset_m] on px, py, pz and from
//! [-offset_rad, offset_rad] on om, fi, ka: diagonal, offset^2 / 3.
inline Eigen::Matrix<double, 6, 6> uniform_pose_offset_covariance(double offset_m, double offset_rad)
{
    Eigen::Matrix<double, 6, 1> variances;
    variances << offset_m * offset_m, offset_m * offset_m, offset_m * offset_m, offset_rad * offset_rad, offset_rad * offset_rad,
        offset_rad * offset_rad;
    return Eigen::Matrix<double, 6, 6>((variances / 3.0).asDiagonal());
}

class PairWiseICP
{
public:
    //! Damping mu of the Gauss-Newton step behind compute_icp_uncertainty(): along an eigenvector of the normal matrix
    //! A^T A (A = observation Jacobian) with eigenvalue lambda, the step keeps mu / (lambda + mu) of the initial error.
    static constexpr double uncertainty_damping = 100.0;

    PairWiseICP() = default;
    ~PairWiseICP() = default;

    bool compute(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        double search_radious,
        int number_of_iterations,
        Eigen::Affine3d& m_pose_result);
    //! Pose standard deviations (px, py, pz in metres; om, fi, ka in radians) of an aligned source from point-to-point
    //! residuals to the nearest target point within search_radious. Covariance = (I - J) Q_ini (I - J)^T, where
    //! J = (A^T A + mu I)^-1 A^T A is the 6 x 6 fraction of an initial pose error one damped Gauss-Newton step removes and
    //! A is the observation Jacobian.
    //! @param initial_covariance Q_ini, covariance of the initial pose error (see uniform_pose_offset_covariance())
    //! @param statistics if given, receives the residual statistics and the covariance
    TaitBryanPose compute_icp_uncertainty(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        const Eigen::Affine3d& m_pose_result,
        double search_radious,
        const Eigen::Matrix<double, 6, 6>& initial_covariance,
        IcpResidualStatistics* statistics = nullptr);

    //! As compute_icp_uncertainty(), from point-to-plane residuals: each source point is matched to the plane fitted to
    //! the target points within search_radious (non-planar neighbourhoods are skipped). Returns infinity for every
    //! parameter when there are too few correspondences.
    TaitBryanPose compute_icp_uncertainty_point_to_plane(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        const Eigen::Affine3d& m_pose_result,
        double search_radious,
        const Eigen::Matrix<double, 6, 6>& initial_covariance,
        IcpResidualStatistics* statistics = nullptr);
};
