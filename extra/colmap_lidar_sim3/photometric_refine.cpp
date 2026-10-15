#include "photometric_refine.h"

#include "camera_projection.h"
#include "sim3_residuals.h"

#include <ceres/cubic_interpolation.h>

#include <ankerl/unordered_dense.h>

#include <tbb/parallel_for.h>

#include <limits>
#include <map>
#include <memory>

namespace colmap_sim3
{
    using namespace detail;

    namespace
    {
        using Grid = ceres::Grid2D<uint8_t, 1>;
        using Interpolator = ceres::BiCubicInterpolator<Grid>;

        //! One pyramid level of a photo; level 0 points at the caller's pixels.
        struct Level
        {
            int width = 0;
            int height = 0;
            std::vector<uint8_t> owned;
            std::unique_ptr<Grid> grid;
            std::unique_ptr<Interpolator> interpolator;
            double uv_scale = 1.0; //!< full-resolution pixel coordinates -> this level's
        };

        struct ViewData
        {
            int camera = -1;
            std::unique_ptr<CameraProjector> projector;
            std::vector<Level> levels;
            std::vector<uint32_t> points; //!< selected lidar points
            std::array<double, 2> affine{ 0.0, 0.0 }; //!< log gain, offset
        };

        //! Photo grey value at a lidar point minus the lidar's (after gain and offset).
        struct PhotometricResidual
        {
            PhotometricResidual(const CameraProjector* projector, const Level* level, const Eigen::Vector3d& X, double gray, double weight)
                : projector_(projector)
                , level_(level)
                , X_(X)
                , gray_(gray)
                , weight_(weight)
            {
            }

            template <typename T>
            bool operator()(const T* const q, const T* const t, const T* const affine, T* r) const
            {
                using std::exp;
                const Eigen::Map<const Eigen::Quaternion<T>> Q(q);
                const Eigen::Map<const Vec3<T>> tt(t);
                const Vec3<T> xc = Q.conjugate() * (X_.cast<T>() - tt);
                Eigen::Matrix<T, 2, 1> uv;
                // A point that leaves the photo (or its lens) stops contributing.
                if (!projector_->projectT(xc, uv))
                {
                    r[0] = T(0);
                    return true;
                }
                const T col = uv[0] * T(level_->uv_scale) - T(0.5);
                const T row = uv[1] * T(level_->uv_scale) - T(0.5);
                if (col < T(1) || row < T(1) || col > T(level_->width - 2) || row > T(level_->height - 2))
                {
                    r[0] = T(0);
                    return true;
                }
                T value;
                level_->interpolator->Evaluate(row, col, &value);
                r[0] = T(weight_) * (value - (exp(affine[0]) * T(gray_) + affine[1]));
                return true;
            }

            const CameraProjector* projector_;
            const Level* level_;
            const Eigen::Vector3d X_;
            const double gray_;
            const double weight_;
        };

        //! Lidar point indices bucketed by a coarse 3D grid, for "points near this camera".
        class PointBuckets
        {
        public:
            PointBuckets(const std::vector<Eigen::Vector3d>& points, double cell)
                : cell_(cell)
            {
                for (size_t i = 0; i < points.size(); i++)
                    buckets_[key(cellOf(points[i]))].push_back(static_cast<uint32_t>(i));
            }

            template <typename F>
            void forEachNear(const Eigen::Vector3d& c, double radius, F&& f) const
            {
                const Eigen::Vector3i lo = cellOf(c - Eigen::Vector3d::Constant(radius));
                const Eigen::Vector3i hi = cellOf(c + Eigen::Vector3d::Constant(radius));
                for (int x = lo.x(); x <= hi.x(); x++)
                    for (int y = lo.y(); y <= hi.y(); y++)
                        for (int z = lo.z(); z <= hi.z(); z++)
                            if (const auto it = buckets_.find(key(Eigen::Vector3i(x, y, z))); it != buckets_.end())
                                for (uint32_t i : it->second)
                                    f(i);
            }

        private:
            Eigen::Vector3i cellOf(const Eigen::Vector3d& p) const
            {
                return (p / cell_).array().floor().cast<int>();
            }

            static uint64_t key(const Eigen::Vector3i& c)
            {
                constexpr int64_t half = int64_t(1) << 20;
                return (uint64_t(c.x() + half) << 42) | (uint64_t(c.y() + half) << 21) | uint64_t(c.z() + half);
            }

            double cell_;
            ankerl::unordered_dense::map<uint64_t, std::vector<uint32_t>> buckets_;
        };

        //! Lidar points that the photo sees unoccluded, on strong photo gradients, away from depth edges and
        //! masks: per cell of the image, the one with the strongest gradient.
        //! @return (lidar point index, pixel index in the image) pairs
        std::vector<std::pair<uint32_t, size_t>> selectPoints(
            const GrayImage& img,
            const CameraProjector& projector,
            const CameraSim3& pose,
            const std::vector<Eigen::Vector3d>& lidar_points,
            const PointBuckets& buckets,
            double max_range,
            int cell_px,
            double min_gradient)
        {
            // Nearest lidar point per pixel.
            const int W = img.width, H = img.height;
            const double scale = double(W) / double(projector.imageWidth());
            std::vector<float> depth(size_t(W) * H, std::numeric_limits<float>::max());
            std::vector<int32_t> index(size_t(W) * H, -1);
            const Eigen::Matrix3d R_cw = pose.q.conjugate().toRotationMatrix();
            const double max_range2 = max_range * max_range;
            buckets.forEachNear(
                pose.t,
                max_range,
                [&](uint32_t i)
                {
                    const Eigen::Vector3d d = lidar_points[i] - pose.t;
                    const double r2 = d.squaredNorm();
                    if (r2 > max_range2 || r2 < 1e-4)
                        return;
                    Eigen::Vector2d uv;
                    if (!projector.project(R_cw * d, uv))
                        return;
                    const int x = static_cast<int>(std::floor(uv.x() * scale));
                    const int y = static_cast<int>(std::floor(uv.y() * scale));
                    if (x < 0 || y < 0 || x >= W || y >= H)
                        return;
                    const float r = static_cast<float>(std::sqrt(r2));
                    const size_t p = size_t(y) * W + x;
                    if (r < depth[p])
                    {
                        depth[p] = r;
                        index[p] = static_cast<int32_t>(i);
                    }
                });

            const auto& I = img.pixels;
            const int cell = std::max(2, cell_px);
            std::vector<std::pair<uint32_t, size_t>> out;
            for (int cy = 0; cy < H; cy += cell)
                for (int cx = 0; cx < W; cx += cell)
                {
                    double best = min_gradient;
                    int best_p = -1;
                    for (int y = std::max(cy, 2); y < std::min(cy + cell, H - 2); y++)
                        for (int x = std::max(cx, 2); x < std::min(cx + cell, W - 2); x++)
                        {
                            const size_t p = size_t(y) * W + x;
                            if (index[p] < 0 || (!img.mask.empty() && img.mask[p] == 0))
                                continue;
                            const double gx = 0.5 * (double(I[p + 1]) - double(I[p - 1]));
                            const double gy = 0.5 * (double(I[p + W]) - double(I[p - W]));
                            const double g = std::hypot(gx, gy);
                            if (g <= best)
                                continue;
                            int neighbours = 0;
                            bool edge = false;
                            for (const size_t n : { p - 1, p + 1, p - W, p + W })
                                if (index[n] >= 0)
                                {
                                    neighbours++;
                                    edge |= std::abs(depth[n] - depth[p]) > 0.05f * depth[p] + 0.05f;
                                }
                            if (edge || neighbours < 2)
                                continue;
                            best = g;
                            best_p = static_cast<int>(p);
                        }
                    if (best_p >= 0)
                        out.emplace_back(static_cast<uint32_t>(index[best_p]), static_cast<size_t>(best_p));
                }
            return out;
        }

        //! Builds the pyramid, picks lidar points that are visible, unoccluded and on photo gradients,
        //! and starts the gain/offset from them.
        void prepareView(
            const PhotometricView& view,
            const CameraSim3& pose,
            const std::vector<Eigen::Vector3d>& lidar_points,
            const std::vector<float>& lidar_gray,
            const PointBuckets& buckets,
            const PhotometricParams& params,
            ViewData& out)
        {
            const GrayImage& img = view.image;
            out.camera = view.camera;
            out.projector = std::make_unique<CameraProjector>(view.intrinsics, params.max_angle_deg);
            if (!out.projector->supported() || img.width < 8 || img.height < 8 || img.pixels.size() != size_t(img.width) * img.height)
                return;

            // Pyramid: level 0 is the caller's image, each next one a 2x2 box average.
            out.levels.resize(std::max(1, params.levels));
            for (size_t l = 0; l < out.levels.size(); l++)
            {
                Level& L = out.levels[l];
                const uint8_t* data = img.pixels.data();
                if (l == 0)
                {
                    L.width = img.width;
                    L.height = img.height;
                }
                else
                {
                    const Level& P = out.levels[l - 1];
                    L.width = std::max(4, P.width / 2);
                    L.height = std::max(4, P.height / 2);
                    const uint8_t* src = l == 1 ? img.pixels.data() : P.owned.data();
                    L.owned.resize(size_t(L.width) * L.height);
                    for (int y = 0; y < L.height; y++)
                        for (int x = 0; x < L.width; x++)
                        {
                            const int x0 = std::min(2 * x, P.width - 1), x1 = std::min(2 * x + 1, P.width - 1);
                            const int y0 = std::min(2 * y, P.height - 1), y1 = std::min(2 * y + 1, P.height - 1);
                            const int sum = src[y0 * P.width + x0] + src[y0 * P.width + x1] + src[y1 * P.width + x0] + src[y1 * P.width + x1];
                            L.owned[size_t(y) * L.width + x] = static_cast<uint8_t>((sum + 2) / 4);
                        }
                    data = L.owned.data();
                }
                L.uv_scale = double(L.width) / double(view.intrinsics.width);
                L.grid = std::make_unique<Grid>(data, 0, L.height, 0, L.width);
                L.interpolator = std::make_unique<Interpolator>(*L.grid);
            }

            std::vector<double> photo_values, lidar_values;
            for (const auto& [i, pixel] : selectPoints(view.image, *out.projector, pose, lidar_points, buckets, params.max_range, params.cell_px, params.min_gradient))
            {
                out.points.push_back(i);
                photo_values.push_back(img.pixels[pixel]);
                lidar_values.push_back(lidar_gray[i]);
            }

            // Gain and offset matching the means and spreads of photo and lidar grey at the picked points.
            if (out.points.size() >= 2)
            {
                auto meanStd = [](const std::vector<double>& v, double& mean, double& sd)
                {
                    mean = 0.0;
                    for (double x : v)
                        mean += x;
                    mean /= double(v.size());
                    double var = 0.0;
                    for (double x : v)
                        var += (x - mean) * (x - mean);
                    sd = std::sqrt(var / double(v.size()));
                };
                double mp, sp, ml, sl;
                meanStd(photo_values, mp, sp);
                meanStd(lidar_values, ml, sl);
                const double gain = std::clamp(sl > 1.0 ? sp / sl : 1.0, 0.2, 5.0);
                out.affine = { std::log(gain), mp - gain * ml };
            }
        }

        //! RMS photometric error at full resolution [grey levels], over points that land in their photo.
        double photometricRms(
            const std::vector<ViewData>& views,
            const std::vector<Sim3Block>& blocks,
            const std::vector<Eigen::Vector3d>& lidar_points,
            const std::vector<float>& lidar_gray)
        {
            double sum = 0.0;
            size_t n = 0;
            for (const ViewData& v : views)
            {
                if (v.points.empty())
                    continue;
                const Sim3Block& b = blocks[v.camera];
                for (uint32_t i : v.points)
                {
                    const PhotometricResidual f(v.projector.get(), &v.levels[0], lidar_points[i], lidar_gray[i], 1.0);
                    double r = 0.0;
                    f(b.q.data(), b.t.data(), v.affine.data(), &r);
                    if (r != 0.0)
                    {
                        sum += r * r;
                        n++;
                    }
                }
            }
            return n > 0 ? std::sqrt(sum / double(n)) : 0.0;
        }
    } // namespace

    bool refinePhotometric(
        const std::vector<Eigen::Vector3d>& lidar_points,
        const std::vector<float>& lidar_gray,
        const std::vector<PhotometricView>& views,
        const PhotometricParams& params,
        std::vector<CameraSim3>& cameras,
        PhotometricStats& stats)
    {
        stats = PhotometricStats{};
        const size_t n = cameras.size();
        if (n == 0 || lidar_points.size() != lidar_gray.size() || lidar_points.empty())
        {
            stats.message = "No cameras, or lidar points without grey values";
            return false;
        }

        const PointBuckets buckets(lidar_points, std::max(1.0, params.max_range / 4.0));
        std::vector<ViewData> data(views.size());
        tbb::parallel_for(
            size_t(0),
            views.size(),
            [&](size_t v)
            {
                if (views[v].camera >= 0 && size_t(views[v].camera) < n)
                    prepareView(views[v], cameras[views[v].camera], lidar_points, lidar_gray, buckets, params, data[v]);
            });
        for (auto& v : data)
            if (v.points.size() < 20)
                v.points.clear();
            else
            {
                stats.views++;
                stats.residuals += v.points.size();
            }
        if (stats.residuals < 100)
        {
            stats.message = "Only " + std::to_string(stats.residuals) +
                " lidar points visible on photo gradients; check the images folder, the alignment and the lidar colours.";
            return false;
        }

        std::vector<Sim3Block> blocks(n);
        for (size_t k = 0; k < n; k++)
            blocks[k] = Sim3Block::from(cameras[k].q, cameras[k].t, cameras[k].s);
        const std::vector<CameraSim3> start = cameras;
        stats.rms_before = photometricRms(data, blocks, lidar_points, lidar_gray);

        // Keep the trajectory's shape from the start (the geometric solution), allowing local corrections.
        std::vector<Eigen::Quaterniond> z_q(n);
        std::vector<Eigen::Vector3d> z_t(n);
        for (size_t k = 1; k < n; k++)
        {
            z_q[k] = (start[k - 1].q.conjugate() * start[k].q).normalized();
            z_t[k] = (start[k - 1].q.conjugate() * (start[k].t - start[k - 1].t)) / start[k - 1].s;
        }

        ceres::HuberLoss loss(1.5);
        ceres::Solver::Summary summary;
        const int levels = std::max(1, params.levels);
        for (int level = levels - 1; level >= 0; level--)
        {
            ceres::Problem problem(problemOptions());
            for (auto& b : blocks)
            {
                problem.AddParameterBlock(b.q.data(), 4, new ceres::EigenQuaternionManifold);
                problem.AddParameterBlock(b.t.data(), 3);
                problem.AddParameterBlock(&b.log_s, 1);
                problem.SetParameterBlockConstant(&b.log_s);
            }
            const double w_t = 1.0 / params.sigma_translation, w_r = 1.0 / (params.sigma_rotation_deg * kDegToRad);
            for (size_t k = 1; k < n; k++)
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<SmoothnessResidual, 7, 4, 3, 1, 4, 3, 1>(
                        new SmoothnessResidual(z_q[k], z_t[k], w_t, w_r, 0.0)),
                    nullptr,
                    blocks[k - 1].q.data(),
                    blocks[k - 1].t.data(),
                    &blocks[k - 1].log_s,
                    blocks[k].q.data(),
                    blocks[k].t.data(),
                    &blocks[k].log_s);
            for (size_t k = 0; k < n; k++)
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<PriorResidual, 6, 4, 3>(new PriorResidual(
                        start[k].q, start[k].t, 1.0 / params.prior_sigma_translation, 1.0 / (params.prior_sigma_rotation_deg * kDegToRad))),
                    nullptr,
                    blocks[k].q.data(),
                    blocks[k].t.data());

            const double weight = 1.0 / params.sigma_intensity;
            for (ViewData& v : data)
            {
                if (v.points.empty())
                    continue;
                Sim3Block& b = blocks[v.camera];
                const Level& L = v.levels[std::min<size_t>(level, v.levels.size() - 1)];
                for (uint32_t i : v.points)
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<PhotometricResidual, 1, 4, 3, 2>(
                            new PhotometricResidual(v.projector.get(), &L, lidar_points[i], lidar_gray[i], weight)),
                        &loss,
                        b.q.data(),
                        b.t.data(),
                        v.affine.data());
            }

            ceres::Solve(solverOptions(params.iterations, ceres::SPARSE_NORMAL_CHOLESKY), &problem, &summary);
            if (!summary.IsSolutionUsable())
            {
                stats.message = "Ceres failed: " + summary.BriefReport();
                return false;
            }
        }

        stats.rms_after = photometricRms(data, blocks, lidar_points, lidar_gray);
        double rot = 0.0, trans = 0.0;
        for (size_t k = 0; k < n; k++)
        {
            cameras[k].q = blocks[k].quaternion();
            cameras[k].t = blocks[k].translation();
            rot += start[k].q.angularDistance(cameras[k].q) / kDegToRad;
            trans += (cameras[k].t - start[k].t).norm();
        }
        stats.mean_rotation_change_deg = rot / double(n);
        stats.mean_translation_change = trans / double(n);
        stats.ok = true;
        stats.message = summary.BriefReport();
        return true;
    }
    std::vector<LensRotation> estimateLensRotations(
        const std::vector<Eigen::Vector3d>& lidar_points,
        const std::vector<float>& lidar_gray,
        const std::vector<PhotometricView>& views,
        const std::vector<CameraSim3>& cameras,
        const LensRotationParams& params,
        std::string& message)
    {
        message.clear();
        std::vector<LensRotation> result;
        if (lidar_points.empty() || lidar_points.size() != lidar_gray.size())
        {
            message = "No lidar points with grey values";
            return result;
        }

        std::map<uint32_t, std::vector<size_t>> by_lens;
        for (size_t v = 0; v < views.size(); v++)
            if (views[v].camera >= 0 && size_t(views[v].camera) < cameras.size() && !views[v].image.pixels.empty())
                by_lens[views[v].intrinsics.id].push_back(v);
        if (by_lens.empty())
        {
            message = "No photos";
            return result;
        }
        const PointBuckets buckets(lidar_points, std::max(1.0, params.max_range / 4.0));

        //! One photo's points, in its camera frame at the starting pose, with their lidar grey.
        struct Sample
        {
            std::unique_ptr<CameraProjector> projector;
            const GrayImage* image = nullptr;
            double scale = 1.0;
            std::vector<Eigen::Vector3d> x;
            std::vector<double> gray;
        };

        auto rotation = [](double yaw, double pitch, double roll)
        {
            return (Eigen::AngleAxisd(yaw * kDegToRad, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(pitch * kDegToRad, Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(roll * kDegToRad, Eigen::Vector3d::UnitZ()))
                .toRotationMatrix();
        };

        for (const auto& [lens, list] : by_lens)
        {
            const size_t step = std::max<size_t>(1, list.size() / size_t(std::max(1, params.max_photos_per_lens)));
            std::vector<size_t> chosen;
            for (size_t a = 0; a < list.size(); a += step)
                chosen.push_back(list[a]);

            std::vector<Sample> samples(chosen.size());
            tbb::parallel_for(
                size_t(0),
                chosen.size(),
                [&](size_t a)
                {
                    const PhotometricView& view = views[chosen[a]];
                    const GrayImage& img = view.image;
                    Sample& smp = samples[a];
                    smp.projector = std::make_unique<CameraProjector>(view.intrinsics, params.max_angle_deg);
                    if (!smp.projector->supported() || img.width < 8 || img.height < 8 || img.pixels.size() != size_t(img.width) * img.height)
                        return;
                    smp.image = &img;
                    smp.scale = double(img.width) / double(view.intrinsics.width);
                    const CameraSim3& pose = cameras[view.camera];
                    const Eigen::Matrix3d R_cw = pose.q.conjugate().toRotationMatrix();
                    for (const auto& [i, pixel] :
                         selectPoints(img, *smp.projector, pose, lidar_points, buckets, params.max_range, params.cell_px, params.min_gradient))
                    {
                        smp.x.push_back(R_cw * (lidar_points[i] - pose.t));
                        smp.gray.push_back(lidar_gray[i]);
                    }
                });
            std::erase_if(
                samples,
                [](const Sample& smp)
                {
                    return smp.x.size() < 50;
                });
            if (samples.empty())
                continue;

            // Mean over photos of the NCC between the photo (bilinear) and the lidar grey, with the
            // camera-frame points turned by Rd.
            auto meanNcc = [&](const Eigen::Matrix3d& Rd)
            {
                std::vector<double> ncc(samples.size(), std::numeric_limits<double>::quiet_NaN());
                tbb::parallel_for(
                    size_t(0),
                    samples.size(),
                    [&](size_t a)
                    {
                        const Sample& smp = samples[a];
                        const GrayImage& img = *smp.image;
                        double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
                        size_t n = 0;
                        for (size_t q = 0; q < smp.x.size(); q++)
                        {
                            Eigen::Vector2d uv;
                            if (!smp.projector->project(Rd * smp.x[q], uv))
                                continue;
                            const double x = uv.x() * smp.scale - 0.5, y = uv.y() * smp.scale - 0.5;
                            if (x < 0.0 || y < 0.0 || x >= img.width - 1 || y >= img.height - 1)
                                continue;
                            const int x0 = static_cast<int>(x), y0 = static_cast<int>(y);
                            const double fx = x - x0, fy = y - y0;
                            const uint8_t* p = &img.pixels[size_t(y0) * img.width + x0];
                            const double v = (1 - fx) * (1 - fy) * p[0] + fx * (1 - fy) * p[1] + (1 - fx) * fy * p[img.width] + fx * fy * p[img.width + 1];
                            const double g = smp.gray[q];
                            sa += v;
                            sb += g;
                            saa += v * v;
                            sbb += g * g;
                            sab += v * g;
                            n++;
                        }
                        if (n < 30)
                            return;
                        const double cov = sab / n - (sa / n) * (sb / n);
                        const double var = (saa / n - (sa / n) * (sa / n)) * (sbb / n - (sb / n) * (sb / n));
                        if (var > 1e-9)
                            ncc[a] = cov / std::sqrt(var);
                    });
                double sum = 0.0;
                size_t valid = 0;
                for (double v : ncc)
                    if (std::isfinite(v))
                    {
                        sum += v;
                        valid++;
                    }
                return valid > 0 ? sum / double(valid) : -1.0;
            };

            LensRotation lr;
            lr.camera_id = lens;
            lr.photos = samples.size();
            lr.ncc_before = meanNcc(Eigen::Matrix3d::Identity());
            double best = lr.ncc_before, by = 0.0, bp = 0.0, br = 0.0;
            auto consider = [&](double y, double p, double r)
            {
                const double v = meanNcc(rotation(y, p, r));
                if (v > best)
                {
                    best = v;
                    by = y;
                    bp = p;
                    br = r;
                }
            };

            // Coarse: yaw x pitch (the image shifts), then roll; then all three around the best, finer and finer.
            const double S = std::max(0.5, params.search_deg);
            for (double y = -S; y <= S + 1e-9; y += 0.5)
                for (double p = -S; p <= S + 1e-9; p += 0.5)
                    consider(y, p, 0.0);
            const double y0 = by, p0 = bp;
            for (double r = -S; r <= S + 1e-9; r += 0.5)
                consider(y0, p0, r);
            for (double fine : { 0.25, 0.1, 0.05 })
            {
                const double cy = by, cp = bp, cr = br;
                for (int i = -2; i <= 2; i++)
                    for (int j = -2; j <= 2; j++)
                        for (int k = -2; k <= 2; k++)
                            consider(cy + i * fine, cp + j * fine, cr + k * fine);
            }

            lr.yaw_deg = by;
            lr.pitch_deg = bp;
            lr.roll_deg = br;
            lr.ncc_after = best;
            lr.delta = Eigen::Quaterniond(rotation(by, bp, br).transpose()).normalized();
            result.push_back(lr);
        }
        if (result.empty())
            message = "No lens has photos with at least 50 lidar points on photo gradients";
        return result;
    }
} // namespace colmap_sim3
