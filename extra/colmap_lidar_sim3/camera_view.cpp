#include "camera_view.h"

#include "camera_projection.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    struct Hit
    {
        int32_t pixel;
        float range;
        cv::Vec4b rgba;
    };

    cv::Vec4b jet(float t)
    {
        t = std::clamp(t, 0.f, 1.f);
        const float r = std::clamp(1.5f - std::abs(4.f * t - 3.f), 0.f, 1.f);
        const float g = std::clamp(1.5f - std::abs(4.f * t - 2.f), 0.f, 1.f);
        const float b = std::clamp(1.5f - std::abs(4.f * t - 1.f), 0.f, 1.f);
        return cv::Vec4b(uchar(255 * r), uchar(255 * g), uchar(255 * b), 255);
    }
} // namespace

LidarColoring LidarColoring::from(const PointCloud& pc)
{
    LidarColoring c;
    const size_t n = pc.points_local.size();
    const size_t stride = std::max<size_t>(1, n / 100000);

    double max_rgb = 0.0;
    if (pc.colors.size() == n)
        for (size_t i = 0; i < n; i += stride)
            max_rgb = std::max(max_rgb, pc.colors[i].maxCoeff());
    c.has_rgb = max_rgb > 0.0;
    // load_pc divides LAS's 16-bit RGB by 256; files that store 8-bit values end up at most 1.
    c.rgb_scale = max_rgb <= 1.0 ? 255.0 : 1.0;

    std::vector<float> values;
    if (pc.intensities.size() == n)
        for (size_t i = 0; i < n; i += stride)
            values.push_back(pc.intensities[i]);
    if (!values.empty())
    {
        std::nth_element(values.begin(), values.begin() + values.size() / 100, values.end());
        c.intensity_lo = values[values.size() / 100];
        std::nth_element(values.begin(), values.begin() + values.size() * 99 / 100, values.end());
        c.intensity_hi = std::max(values[values.size() * 99 / 100], c.intensity_lo + 1.f);
    }
    return c;
}

cv::Mat renderLidarProjection(
    const colmap_sim3::Camera& camera,
    const colmap_sim3::CameraSim3& pose,
    const PointCloud& lidar,
    const LidarColoring& coloring,
    const ProjectionSettings& settings)
{
    const colmap_sim3::CameraProjector projector(camera, settings.max_angle_deg);
    if (!projector.supported() || camera.width == 0 || camera.height == 0)
        return {};
    const double scale = double(std::max(settings.width, 16)) / double(camera.width);
    const int W = static_cast<int>(std::lround(camera.width * scale));
    const int H = static_cast<int>(std::lround(camera.height * scale));

    const Eigen::Matrix3d R_cw = pose.q.conjugate().toRotationMatrix();
    const Eigen::Vector3d t = pose.t;
    const double max_range2 = settings.max_range * settings.max_range;
    const size_t stride = static_cast<size_t>(std::max(settings.stride, 1));
    const auto& pts = lidar.points_local;
    const bool use_rgb = settings.color == ProjectionColor::Rgb && coloring.has_rgb && lidar.colors.size() == pts.size();
    const bool use_intensity = settings.color == ProjectionColor::Intensity && lidar.intensities.size() == pts.size();

    // Project in parallel chunks, then splat in chunk order so the result does not depend on threads.
    const size_t chunk = 1 << 18;
    const size_t num_chunks = (pts.size() + chunk - 1) / chunk;
    std::vector<std::vector<Hit>> hits(num_chunks);
    tbb::parallel_for(
        size_t(0),
        num_chunks,
        [&](size_t c)
        {
            const size_t begin = c * chunk, end = std::min(pts.size(), begin + chunk);
            for (size_t i = begin - begin % stride + (begin % stride ? stride : 0); i < end; i += stride)
            {
                const Eigen::Vector3d d = pts[i] - t;
                const double r2 = d.squaredNorm();
                if (r2 > max_range2 || r2 < 1e-4)
                    continue;
                Eigen::Vector2d uv;
                if (!projector.project(R_cw * d, uv))
                    continue;
                // COLMAP pixel centres sit at +0.5.
                const int x = static_cast<int>(std::floor(uv.x() * scale));
                const int y = static_cast<int>(std::floor(uv.y() * scale));
                if (x < 0 || y < 0 || x >= W || y >= H)
                    continue;
                const float range = static_cast<float>(std::sqrt(r2));
                cv::Vec4b col;
                if (use_rgb)
                {
                    const Eigen::Vector3d rgb = (lidar.colors[i] * coloring.rgb_scale).cwiseMax(0.0).cwiseMin(255.0);
                    col = cv::Vec4b(uchar(rgb.x()), uchar(rgb.y()), uchar(rgb.z()), 255);
                }
                else if (use_intensity)
                {
                    const float v = (float(lidar.intensities[i]) - coloring.intensity_lo) / (coloring.intensity_hi - coloring.intensity_lo);
                    const uchar g = uchar(255.f * std::clamp(v, 0.f, 1.f));
                    col = cv::Vec4b(g, g, g, 255);
                }
                else
                    col = jet(range / float(settings.max_range));
                hits[c].push_back({ y * W + x, range, col });
            }
        });

    cv::Mat out(H, W, CV_8UC4, cv::Scalar(0, 0, 0, 0));
    std::vector<float> depth(size_t(W) * H, std::numeric_limits<float>::max());
    const int r = std::max(settings.splat_radius, 0);
    for (const auto& chunk_hits : hits)
        for (const Hit& h : chunk_hits)
        {
            const int x0 = h.pixel % W, y0 = h.pixel / W;
            for (int dy = -r; dy <= r; dy++)
                for (int dx = -r; dx <= r; dx++)
                {
                    const int x = x0 + dx, y = y0 + dy;
                    if (dx * dx + dy * dy > r * r || x < 0 || y < 0 || x >= W || y >= H)
                        continue;
                    float& z = depth[size_t(y) * W + x];
                    if (h.range < z)
                    {
                        z = h.range;
                        out.at<cv::Vec4b>(y, x) = h.rgba;
                    }
                }
        }
    return out;
}

cv::Mat loadPhoto(const std::string& file, int width, int height)
{
    const cv::Mat bgr = cv::imread(file, cv::IMREAD_COLOR);
    if (bgr.empty())
        return {};
    cv::Mat resized, rgba;
    cv::resize(bgr, resized, cv::Size(width, height), 0, 0, cv::INTER_AREA);
    cv::cvtColor(resized, rgba, cv::COLOR_BGR2RGBA);
    return rgba;
}

colmap_sim3::GrayImage loadGrayImage(const std::string& photo_file, const std::string& mask_file, int width, int height)
{
    colmap_sim3::GrayImage out;
    const cv::Mat gray = cv::imread(photo_file, cv::IMREAD_GRAYSCALE);
    if (gray.empty() || width <= 0 || height <= 0)
        return out;
    cv::Mat resized;
    cv::resize(gray, resized, cv::Size(width, height), 0, 0, cv::INTER_AREA);
    out.width = width;
    out.height = height;
    out.pixels.assign(resized.begin<uint8_t>(), resized.end<uint8_t>());

    if (!mask_file.empty())
    {
        const cv::Mat mask = cv::imread(mask_file, cv::IMREAD_GRAYSCALE);
        if (!mask.empty())
        {
            cv::Mat small;
            cv::resize(mask, small, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
            out.mask.assign(small.begin<uint8_t>(), small.end<uint8_t>());
        }
    }
    return out;
}

void GpuImage::upload(const cv::Mat& rgba)
{
    if (rgba.empty() || rgba.type() != CV_8UC4)
        return;
    const cv::Mat data = rgba.isContinuous() ? rgba : rgba.clone();
    if (valid && texture.width == data.cols && texture.height == data.rows)
    {
        UpdateTexture(texture, data.data);
        return;
    }
    release();
    Image img{ data.data, data.cols, data.rows, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
    texture = LoadTextureFromImage(img);
    SetTextureFilter(texture, TEXTURE_FILTER_BILINEAR);
    valid = texture.id != 0;
}

void GpuImage::release()
{
    if (valid)
        UnloadTexture(texture);
    texture = Texture2D{};
    valid = false;
}
