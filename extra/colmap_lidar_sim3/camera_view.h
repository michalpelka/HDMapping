#pragma once

// What one COLMAP camera sees: the lidar map rendered through the camera's own model (colored by
// the LAZ RGB, intensity or range), and its photo, as images ready for ImGui.

#include "raylib.h"

#include "colmap_model.h"
#include "sim3_refine.h"

#include <Core/point_cloud.h>

#include <opencv2/core.hpp>

#include <string>

enum class ProjectionColor
{
    Rgb, //!< LAZ RGB
    Intensity, //!< LAZ intensity, grey
    Range, //!< distance from the camera, jet
};

struct ProjectionSettings
{
    int width = 960; //!< rendered image width [px]; height follows the camera's aspect ratio
    int stride = 1; //!< use every Nth lidar point
    double max_range = 80.0; //!< [m]
    double max_angle_deg = 95.0; //!< from the optical axis; beyond a fisheye's image circle points only land in its black corners
    int splat_radius = 1; //!< [px]
    ProjectionColor color = ProjectionColor::Intensity;

    bool operator==(const ProjectionSettings&) const = default;
};

//! How to turn the LAZ attributes of the lidar cloud into colors; set once per loaded LAZ.
struct LidarColoring
{
    bool has_rgb = false;
    double rgb_scale = 1.0; //!< PointCloud::colors * rgb_scale is 0..255
    float intensity_lo = 0.f, intensity_hi = 1.f; //!< 1st..99th percentile of the intensities

    //! Inspects the colors and intensities of @p pc.
    static LidarColoring from(const PointCloud& pc);
};

//! Renders @p lidar (points in the same frame as @p pose) through @p camera, nearest point per pixel.
//! @param pose camera -> lidar frame; only its rotation and translation matter
//! @return RGBA image, transparent where no lidar point lands; empty for an unsupported camera model
cv::Mat renderLidarProjection(
    const colmap_sim3::Camera& camera,
    const colmap_sim3::CameraSim3& pose,
    const PointCloud& lidar,
    const LidarColoring& coloring,
    const ProjectionSettings& settings);

//! Loads @p file as RGBA, resized to @p width x @p height. @return empty when it cannot be read.
cv::Mat loadPhoto(const std::string& file, int width, int height);

//! A GPU texture holding the latest uploaded image; must be released while the GL context exists.
struct GpuImage
{
    Texture2D texture{};
    bool valid = false;

    void upload(const cv::Mat& rgba);
    void release();
};
