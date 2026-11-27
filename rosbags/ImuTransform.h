#pragma once
// What an importer does to every IMU sample of a bag before HDMapping uses it:
// rotate it into the lidar frame (the frame HDMapping assumes the IMU is in) and
// bring acceleration to g. Shared by mcap_to_laz / bag_to_laz and the bag
// recording source of lidar_odometry_step_1.
#include "McapWriter.h" // McapImuSample

#include <Eigen/Geometry>

#include <vector>

namespace rosbags
{

// Standard gravity, the g HDMapping's IMU csv is scaled in.
inline constexpr double kStandardGravity = 9.80665;

struct ImuTransform
{
	Eigen::Quaterniond imu_to_lidar = Eigen::Quaterniond::Identity(); // IMU frame -> lidar frame
	double acc_scale = 1.0; // 1 for a bag already in g, 1/9.80665 for m/s^2
};

inline void applyImuTransform(const ImuTransform& t, McapImuSample& s)
{
	const Eigen::Vector3f gyro = (t.imu_to_lidar * Eigen::Vector3d(s.gyro_x, s.gyro_y, s.gyro_z)).cast<float>();
	const Eigen::Vector3f acc = (t.imu_to_lidar * Eigen::Vector3d(s.acc_x, s.acc_y, s.acc_z)).cast<float>();
	s.gyro_x = gyro.x();
	s.gyro_y = gyro.y();
	s.gyro_z = gyro.z();
	s.acc_x = static_cast<float>(acc.x() * t.acc_scale);
	s.acc_y = static_cast<float>(acc.y() * t.acc_scale);
	s.acc_z = static_cast<float>(acc.z() * t.acc_scale);
}

inline void applyImuTransform(const ImuTransform& t, std::vector<McapImuSample>& samples)
{
	for(auto& s : samples)
		applyImuTransform(t, s);
}

} // namespace rosbags
