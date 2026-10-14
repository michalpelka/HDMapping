// COLMAP -> lidar Sim(3) registration. Load a COLMAP model and a lidar LAZ map, move the model
// roughly into place with the gizmo and the Scale field, run a global Sim(3) ICP, then a per-camera
// Sim(3) refinement that removes the SfM "banana" (bending and scale drift). Saves the corrected
// model and the camera trajectory (TUM) in the lidar frame.

#include <algorithm>
#include <cfloat>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>

#include <imgui.h>
#include <rlImGui.h>

#include <ImGuizmo.h>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <Eigen/Eigen>

#include <Core/pfd_wrapper.hpp>
#include <Core/point_cloud.h>
#include <Core/raylib_render.hpp>

#include <RaylibWidgets/RayPlaneD.h>

#ifdef _WIN32
// windows.h (pulled in by portable-file-dialogs.h) declares CloseWindow/ShowCursor like raylib.h does.
#define CloseWindow CloseWindow_win32
#define ShowCursor ShowCursor_win32
#endif
#include <portable-file-dialogs.h>
#ifdef _WIN32
#undef CloseWindow
#undef ShowCursor
#endif

#include <HDMapping/HDMAPPING_ConfigureInfo.hpp>

#include "app_shell.h"
#include "camera_projection.h"
#include "camera_view.h"
#include "colmap_model.h"
#include "lidar_plane_map.h"
#include "sim3_refine.h"

namespace fs = std::filesystem;
namespace cs = colmap_sim3;

namespace
{
    const std::string winTitle = std::string("COLMAP to lidar Sim(3) ") + HDMAPPING_CONFIGURE_PROJECT_VERSION;

    const std::vector<std::string> infoLines = {
        "Registers a COLMAP model to a lidar map and removes its bending and scale drift",
        "",
        "1. File > Open COLMAP model (the sparse/0 folder) and File > Open lidar LAZ",
        "2. Move, rotate and scale the model roughly onto the lidar with the gizmo (scale: drag the ring",
        "   around its centre), or load a saved alignment (align.json)",
        "3. ICP (Sim3): one similarity for the whole model",
        "4. Refine per camera: one similarity per camera, smoothed along the trajectory",
        "5. File > Save COLMAP model / Save TUM trajectory (both in the lidar frame)",
    };

    const std::vector<ShortcutEntry> appShortcuts = {
        { "Normal keys", "1-9", "lidar point size" },
        { "", "F / B / L / R / T / U / I", "camera presets" },
        { "", "Z", "reset camera" },
        { "", "O", "orthographic / perspective" },
        { "", "X", "show axes" },
        { "", "C", "compass" },
        { "Mouse", "Ctrl + middle / right click", "rotation center on the z = 0 plane" },
        { "", "Shift + right click", "pick the nearest camera and open the camera view" },
    };

    //! The COLMAP model, with cameras in capture (image name) order.
    struct SfmData
    {
        std::string dir;
        cs::Model model;
        std::vector<uint32_t> image_ids; //!< capture order
        std::vector<Eigen::Affine3d> T_wc; //!< COLMAP camera -> world, capture order
        std::vector<uint64_t> point_ids;
        std::vector<Eigen::Vector3d> points; //!< COLMAP frame
        std::vector<std::vector<int>> tracks; //!< per point, indices into T_wc
        std::vector<char> active;
        cs::AnchoredPoints anchored;
    };

    //! The lidar map, shifted by origin so it renders and hashes in small coordinates.
    struct LidarData
    {
        std::string file;
        Eigen::Vector3d origin = Eigen::Vector3d::Zero(); //!< added back on export
        std::vector<PointCloud> clouds; //!< one cloud, points relative to origin
        double z_min = 0.0, z_max = 1.0, extent = 1.0;
        cs::LidarPlaneMap planes;
        double planes_voxel = -1.0; //!< voxel the plane map was built with, -1 when not built
        double planes_flatness = -1.0; //!< flatness limit the plane map was built with
        LidarColoring coloring;
    };

    SfmData sfm;
    LidarData lidar;

    cs::Similarity G; //!< COLMAP frame -> lidar local frame
    std::vector<cs::CameraSim3> refined; //!< empty until refined; cleared whenever G changes
    std::vector<float> scale_plot; //!< refined s_k / G.s along the trajectory

    // [0] raw COLMAP points (drawn under G), [1] refined points in the lidar local frame,
    // [2] raw camera centres (drawn under G), [3] refined camera centres.
    std::vector<PointCloud> sfm_clouds(4);
    std::unique_ptr<ScanRenderer> lidar_renderer;
    std::unique_ptr<ScanRenderer> sfm_renderer;

    int min_track_length = 3;
    float max_reprojection_error = 2.0f;
    double voxel_size = 0.5;
    double max_flatness = 0.1;
    cs::AlignParams align_params;
    cs::RefineParams refine_params;

    bool gizmo_enabled = true;
    float gizmo_matrix[16];
    bool show_lidar = true;
    bool show_before = true;
    bool show_after = true;
    bool show_trajectory = true;
    int trajectory_point_size = 6;
    float trajectory_line_width = 2.0f;
    bool trajectory_on_top = true;
    int sfm_point_size = 3;
    ScanColorMode lidar_color_mode = ScanColorMode::Elevation;
    std::string status;
    //! COLMAP camera whose images go into the TUM file, 0 for all; a rig's lenses share timestamps.
    uint32_t tum_camera_id = 0;

    // Camera view: the lidar seen through one COLMAP camera, next to its photo.
    enum class CameraViewMode
    {
        Lidar,
        Photo,
        SideBySide,
        Blend,
    };
    bool camera_view_open = false;
    int selected_camera = 0;
    std::string images_dir;
    ProjectionSettings projection;
    CameraViewMode camera_view_mode = CameraViewMode::SideBySide;
    float blend_alpha = 0.5f;
    bool flicker = false;
    GpuImage projection_image;
    GpuImage photo_image;
    bool photo_missing = false;
    //! What the current projection_image / photo_image were made from; re-render when it changes.
    struct ProjectionKey
    {
        int camera = -1;
        Eigen::Vector4d q = Eigen::Vector4d::Zero();
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        ProjectionSettings settings;
        const PointCloud* lidar = nullptr;
        bool operator==(const ProjectionKey&) const = default;
    } projection_key;
    struct PhotoKey
    {
        int camera = -1;
        std::string file;
        int width = 0;
        bool operator==(const PhotoKey&) const = default;
    } photo_key;

    const Color kBeforeColor{ 255, 140, 0, 255 };
    const Color kAfterColor{ 30, 200, 60, 255 };
    const Color kSelectedColor{ 0, 230, 255, 255 };

    void setStatus(const std::string& text)
    {
        status = text;
        spdlog::info("{}", text);
    }

    void releaseGpu()
    {
        lidar_renderer.reset();
        sfm_renderer.reset();
        projection_image.release();
        photo_image.release();
    }

    void clearRefinement()
    {
        refined.clear();
        scale_plot.clear();
    }

    //! Points with a short track or a large reprojection error only follow their anchor camera.
    void updateActive()
    {
        sfm.active.assign(sfm.points.size(), 0);
        size_t used = 0;
        for (size_t i = 0; i < sfm.points.size(); i++)
        {
            const cs::Point3D& pt = sfm.model.points.at(sfm.point_ids[i]);
            const bool ok = static_cast<int>(sfm.tracks[i].size()) >= min_track_length &&
                (pt.error < 0.0 || pt.error <= static_cast<double>(max_reprojection_error));
            sfm.active[i] = ok ? 1 : 0;
            used += ok ? 1 : 0;
        }
        sfm.anchored.active = sfm.active;
        spdlog::info("{} of {} COLMAP points used for registration", used, sfm.points.size());
    }

    void resetAlignment()
    {
        G = cs::Similarity{};
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& p : sfm.points)
            centroid += p;
        if (!sfm.points.empty())
            centroid /= double(sfm.points.size());
        G.pivot = centroid;
        // The lidar origin is its bounding-box centre, so this drops the model onto the map.
        G.t = Eigen::Vector3d::Zero();
        clearRefinement();
    }

    bool loadColmap(const std::string& dir)
    {
        cs::Model model;
        std::string error;
        if (!cs::readModel(dir, model, error))
        {
            setStatus("Cannot read COLMAP model: " + error);
            return false;
        }
        if (model.images.empty() || model.points.empty())
        {
            setStatus("COLMAP model in " + dir + " has no images or no points");
            return false;
        }

        sfm = SfmData{};
        sfm.dir = dir;
        sfm.model = std::move(model);
        sfm.image_ids = cs::imagesSortedByName(sfm.model);
        std::map<uint32_t, int> index_of_image;
        for (size_t k = 0; k < sfm.image_ids.size(); k++)
        {
            index_of_image[sfm.image_ids[k]] = static_cast<int>(k);
            sfm.T_wc.push_back(sfm.model.images.at(sfm.image_ids[k]).worldFromCamera());
        }

        for (const auto& [id, pt] : sfm.model.points)
        {
            std::vector<int> track;
            for (const auto& el : pt.track)
                if (const auto it = index_of_image.find(el.image_id); it != index_of_image.end())
                    track.push_back(it->second);
            std::sort(track.begin(), track.end());
            track.erase(std::unique(track.begin(), track.end()), track.end());
            sfm.point_ids.push_back(id);
            sfm.points.push_back(pt.xyz);
            sfm.tracks.push_back(std::move(track));
        }

        updateActive();
        sfm.anchored = cs::anchorPoints(sfm.T_wc, sfm.points, sfm.tracks, sfm.active);

        for (auto& c : sfm_clouds)
            c = PointCloud{};
        sfm_clouds[0].points_local = sfm.points;
        for (const auto& T : sfm.T_wc)
            sfm_clouds[2].points_local.push_back(T.translation());
        if (sfm_renderer)
            sfm_renderer->rebuildAll(sfm_clouds);

        // The photos usually sit in an "images" folder next to the model's sparse/ or txt/ folder.
        images_dir.clear();
        const std::string first_name = sfm.model.images.at(sfm.image_ids.front()).name;
        for (fs::path d = fs::path(dir); !d.empty() && d != d.parent_path(); d = d.parent_path())
            if (fs::exists(d / "images" / first_name))
            {
                images_dir = (d / "images").string();
                break;
            }
        selected_camera = 0;

        // One lens of a rig by default, so the TUM file has one pose per timestamp.
        tum_camera_id = sfm.model.cameras.size() > 1 ? sfm.model.cameras.begin()->first : 0;

        resetAlignment();
        setNewRotationCenter(G.t);
        setStatus(
            "Loaded COLMAP model " + dir + ": " + std::to_string(sfm.T_wc.size()) + " images, " + std::to_string(sfm.points.size()) +
            " points");
        return true;
    }

    bool loadLidar(const std::string& file)
    {
        PointCloud pc;
        if (!pc.load_pc(file, false) || pc.points_local.empty())
        {
            setStatus("Cannot read lidar file " + file);
            return false;
        }

        Eigen::Vector3d lo = pc.points_local.front(), hi = lo;
        for (const auto& p : pc.points_local)
        {
            lo = lo.cwiseMin(p);
            hi = hi.cwiseMax(p);
        }
        lidar = LidarData{};
        lidar.file = file;
        lidar.origin = ((lo + hi) / 2.0).array().round();
        for (auto& p : pc.points_local)
            p -= lidar.origin;

        // Height colors span the 1st..99th percentile, so a few outliers don't flatten them to one color.
        std::vector<double> z;
        const size_t stride = std::max<size_t>(1, pc.points_local.size() / 100000);
        for (size_t i = 0; i < pc.points_local.size(); i += stride)
            z.push_back(pc.points_local[i].z());
        std::nth_element(z.begin(), z.begin() + z.size() / 100, z.end());
        lidar.z_min = z[z.size() / 100];
        std::nth_element(z.begin(), z.begin() + z.size() * 99 / 100, z.end());
        lidar.z_max = std::max(z[z.size() * 99 / 100], lidar.z_min + 1e-3);
        lidar.extent = std::max((hi - lo).maxCoeff(), 1.0);
        lidar.coloring = LidarColoring::from(pc);
        if (!lidar.coloring.has_rgb && projection.color == ProjectionColor::Rgb)
            projection.color = ProjectionColor::Intensity;
        pc.render_color[0] = pc.render_color[1] = pc.render_color[2] = 0.85f;
        lidar.clouds.push_back(std::move(pc));
        if (lidar_renderer)
            lidar_renderer->rebuildAll(lidar.clouds);

        clearRefinement();
        setStatus(
            "Loaded lidar " + file + ": " + std::to_string(lidar.clouds[0].points_local.size()) + " points, local origin (" +
            std::to_string(lidar.origin.x()) + ", " + std::to_string(lidar.origin.y()) + ", " + std::to_string(lidar.origin.z()) + ")");
        return true;
    }

    bool ensurePlaneMap()
    {
        if (lidar.clouds.empty())
            return false;
        if (lidar.planes_voxel != voxel_size || lidar.planes_flatness != max_flatness)
        {
            lidar.planes.build(lidar.clouds[0].points_local, voxel_size, 10, max_flatness);
            lidar.planes_voxel = voxel_size;
            lidar.planes_flatness = max_flatness;
            spdlog::info("{} planar cells at {} m voxels", lidar.planes.size(), voxel_size);
        }
        return lidar.planes.size() > 0;
    }

    bool readyToRegister()
    {
        if (sfm.points.empty() || lidar.clouds.empty())
        {
            setStatus("Open a COLMAP model and a lidar LAZ first");
            return false;
        }
        if (!ensurePlaneMap())
        {
            setStatus("No planar cells in the lidar map; try a larger voxel size");
            return false;
        }
        return true;
    }

    void runIcp()
    {
        if (!readyToRegister())
            return;
        cs::Similarity g = G;
        cs::SolveStats st;
        if (!cs::alignSimilarity(sfm.points, sfm.active, lidar.planes, align_params, g, st))
        {
            setStatus("ICP failed: " + st.message);
            return;
        }
        G = g;
        clearRefinement();
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4) << "ICP (Sim3): inliers " << st.inliers_before << " -> " << st.inliers_after << ", RMS "
           << st.rms_before << " -> " << st.rms_after << " m, scale " << std::setprecision(6) << G.s;
        setStatus(ss.str());
    }

    void runRefine()
    {
        if (!readyToRegister())
            return;
        // Continues from the last refinement, so strong bends that start out of reach converge over repeated runs.
        std::vector<cs::CameraSim3> cams = refined.empty() ? cs::camerasFromSimilarity(G, sfm.T_wc) : refined;
        cs::SolveStats st;
        if (!cs::refinePerCamera(sfm.T_wc, sfm.anchored, lidar.planes, refine_params, cams, st))
        {
            setStatus("Refinement failed: " + st.message);
            return;
        }
        refined = std::move(cams);
        sfm_clouds[1].points_local = cs::anchoredWorldPoints(sfm.anchored, refined);
        sfm_clouds[3].points_local.clear();
        for (const auto& c : refined)
            sfm_clouds[3].points_local.push_back(c.t);
        sfm_renderer->rebuildAll(sfm_clouds);

        scale_plot.clear();
        for (const auto& c : refined)
            scale_plot.push_back(static_cast<float>(c.s / G.s));
        const auto [lo, hi] = std::minmax_element(scale_plot.begin(), scale_plot.end());
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4) << "Refined per camera: inliers " << st.inliers_before << " -> " << st.inliers_after
           << ", RMS " << st.rms_before << " -> " << st.rms_after << " m, scale / global " << *lo << " .. " << *hi;
        setStatus(ss.str());
    }

    //! Camera @p k (capture order) -> lidar local frame: the refinement if there is one, else G on its COLMAP pose.
    cs::CameraSim3 currentCamera(size_t k)
    {
        if (!refined.empty())
            return refined[k];
        return { (G.q * Eigen::Quaterniond(sfm.T_wc[k].linear())).normalized(), G.apply(sfm.T_wc[k].translation()), G.s };
    }

    //! Camera similarities to export: the refinement if there is one, else G on the COLMAP poses.
    std::vector<cs::CameraSim3> outputCameras()
    {
        return refined.empty() ? cs::camerasFromSimilarity(G, sfm.T_wc) : refined;
    }

    bool confirmOverwrite(const fs::path& dir)
    {
        bool exists = false;
        for (const char* f : { "cameras.txt", "images.txt", "points3D.txt", "cameras.bin", "images.bin", "points3D.bin" })
            exists |= fs::exists(dir / f);
        if (!exists)
            return true;
        return pfd::message("Overwrite COLMAP model?", dir.string() + " already holds a COLMAP model. Overwrite it?", pfd::choice::yes_no,
                            pfd::icon::warning)
                   .result() == pfd::button::yes;
    }

    void saveColmap()
    {
        if (sfm.points.empty() || lidar.clouds.empty())
        {
            setStatus("Nothing to save: open a COLMAP model and a lidar LAZ first");
            return;
        }
        const std::string dir = mandeye::fd::SelectFolder("Folder for the corrected COLMAP model");
        if (dir.empty())
            return;
        std::error_code ec;
        if (fs::equivalent(dir, sfm.dir, ec))
        {
            setStatus("Choose a folder other than the input model's");
            return;
        }
        if (!confirmOverwrite(dir))
            return;

        cs::Model out = sfm.model;
        const auto cams = outputCameras();
        for (size_t k = 0; k < cams.size(); k++)
        {
            Eigen::Affine3d T = Eigen::Affine3d::Identity();
            T.linear() = cams[k].q.toRotationMatrix();
            T.translation() = cams[k].t + lidar.origin;
            out.images.at(sfm.image_ids[k]).setWorldFromCamera(T);
        }
        const auto world = cs::anchoredWorldPoints(sfm.anchored, cams);
        for (size_t i = 0; i < world.size(); i++)
            out.points.at(sfm.point_ids[i]).xyz = world[i] + lidar.origin;

        std::string error;
        if (!cs::writeModelText(dir, out, error))
        {
            setStatus("Saving the COLMAP model failed: " + error);
            return;
        }
        setStatus(std::string("Saved ") + (refined.empty() ? "globally aligned" : "refined") + " COLMAP model to " + dir);
    }

    void saveTum()
    {
        if (sfm.points.empty() || lidar.clouds.empty())
        {
            setStatus("Nothing to save: open a COLMAP model and a lidar LAZ first");
            return;
        }
        const std::string file = mandeye::fd::SaveFileDialog("Save TUM trajectory", mandeye::fd::Tum_filter, ".tum");
        if (file.empty())
            return;

        std::vector<int64_t> ts;
        bool all_timed = true;
        for (uint32_t id : sfm.image_ids)
        {
            ts.push_back(cs::timestampNsFromImageName(sfm.model.images.at(id).name));
            all_timed &= ts.back() >= 0;
        }

        std::ofstream out(file);
        if (!out)
        {
            setStatus("Cannot write " + file);
            return;
        }
        out << std::fixed;
        const auto cams = outputCameras();
        size_t written = 0;
        for (size_t k = 0; k < cams.size(); k++)
        {
            if (tum_camera_id != 0 && sfm.model.images.at(sfm.image_ids[k]).camera_id != tum_camera_id)
                continue;
            const Eigen::Vector3d p = cams[k].t + lidar.origin;
            const Eigen::Quaterniond& q = cams[k].q;
            out << std::setprecision(9) << (all_timed ? double(ts[k]) * 1e-9 : double(k)) << " " << std::setprecision(6) << p.x() << " "
                << p.y() << " " << p.z() << " " << std::setprecision(9) << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
            written++;
        }
        setStatus(
            "Saved " + std::to_string(written) + " camera poses to " + file +
            (all_timed ? "" : " (image names carry no timestamps; time = image index)"));
    }

    void drawScene()
    {
        const Vector3 rc = camera.euler.rotationCenter;
        if (show_lidar && lidar_renderer && !lidar.clouds.empty())
            lidar_renderer->draw(
                lidar.clouds,
                static_cast<float>(point_size),
                lidar_color_mode,
                static_cast<float>(lidar.z_min),
                static_cast<float>(lidar.z_max),
                Eigen::Vector3d(rc.x, rc.y, rc.z),
                static_cast<float>(lidar.extent));

        if (sfm.points.empty() || !sfm_renderer)
            return;
        if (show_before)
            sfm_renderer->drawCachedWithTransform(0, G.affine(), kBeforeColor, static_cast<float>(sfm_point_size));
        if (show_after && !refined.empty())
            sfm_renderer->drawCachedWithTransform(1, Eigen::Affine3d::Identity(), kAfterColor, static_cast<float>(sfm_point_size));

        if (!show_trajectory && !camera_view_open)
            return;
        // On top of the lidar by default: inside a dense map the trajectory is otherwise mostly hidden.
        rlDrawRenderBatchActive();
        if (trajectory_on_top)
            rlDisableDepthTest();
        auto line = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b)
        {
            rlVertex3f(static_cast<float>(a.x()), static_cast<float>(a.y()), static_cast<float>(a.z()));
            rlVertex3f(static_cast<float>(b.x()), static_cast<float>(b.y()), static_cast<float>(b.z()));
        };
        if (show_trajectory)
        {
            auto polyline = [&line](const std::vector<Eigen::Vector3d>& pts, Color c)
            {
                rlBegin(RL_LINES);
                rlColor4ub(c.r, c.g, c.b, c.a);
                for (size_t k = 1; k < pts.size(); k++)
                    line(pts[k - 1], pts[k]);
                rlEnd();
            };
            rlSetLineWidth(trajectory_line_width);
            if (show_before)
            {
                std::vector<Eigen::Vector3d> centres;
                for (const auto& T : sfm.T_wc)
                    centres.push_back(G.apply(T.translation()));
                polyline(centres, kBeforeColor);
            }
            if (show_after && !refined.empty())
            {
                std::vector<Eigen::Vector3d> centres;
                for (const auto& c : refined)
                    centres.push_back(c.t);
                polyline(centres, kAfterColor);
            }
            rlDrawRenderBatchActive();
            rlSetLineWidth(1.0f);

            const float size = static_cast<float>(trajectory_point_size);
            if (show_before)
                sfm_renderer->drawCachedWithTransform(2, G.affine(), kBeforeColor, size);
            if (show_after && !refined.empty())
                sfm_renderer->drawCachedWithTransform(3, Eigen::Affine3d::Identity(), kAfterColor, size);
        }

        // The camera shown in the camera view: its axes, the viewing direction (z) longest.
        if (camera_view_open && selected_camera >= 0 && selected_camera < static_cast<int>(sfm.T_wc.size()))
        {
            const cs::CameraSim3 c = currentCamera(selected_camera);
            const Eigen::Matrix3d R = c.q.toRotationMatrix();
            const double len = 1.0;
            rlSetLineWidth(3.0f);
            rlBegin(RL_LINES);
            rlColor4ub(255, 60, 60, 255);
            line(c.t, c.t + R.col(0) * len);
            rlColor4ub(60, 255, 60, 255);
            line(c.t, c.t + R.col(1) * len);
            rlColor4ub(kSelectedColor.r, kSelectedColor.g, kSelectedColor.b, 255);
            line(c.t, c.t + R.col(2) * 3.0 * len);
            rlEnd();
            rlDrawRenderBatchActive();
            rlSetLineWidth(1.0f);
        }

        rlDrawRenderBatchActive();
        if (trajectory_on_top)
            rlEnableDepthTest();
    }

    //! Moves, rotates and scales the whole COLMAP model (G, about its pivot). The scale ring around the
    //! centre and the axis-tip boxes all scale uniformly.
    void gizmo()
    {
        if (!gizmo_enabled || sfm.points.empty())
            return;
        const ImGuiIO& io = ImGui::GetIO();
        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        if (!ImGuizmo::IsUsing())
        {
            Eigen::Affine3f m = Eigen::Affine3f::Identity();
            m.linear() = static_cast<float>(G.s) * G.q.toRotationMatrix().cast<float>();
            m.translation() = G.t.cast<float>();
            std::memcpy(gizmo_matrix, m.data(), sizeof(gizmo_matrix));
        }

        if (!camera.isOrtho)
        {
            float projection[16], modelview[16];
            getProjectionMatrix(projection);
            getModelviewMatrix(modelview);
            ImGuizmo::Manipulate(
                modelview, projection, ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y | ImGuizmo::ROTATE_Z | ImGuizmo::SCALEU,
                ImGuizmo::WORLD, gizmo_matrix, nullptr);
        }
        else
            ImGuizmo::Manipulate(
                camera.orthoGizmoView, camera.orthoProjection,
                ImGuizmo::TRANSLATE_X | ImGuizmo::TRANSLATE_Y | ImGuizmo::ROTATE_Z | ImGuizmo::SCALEU, ImGuizmo::WORLD, gizmo_matrix,
                nullptr);

        if (ImGuizmo::IsUsing())
        {
            const Eigen::Matrix4d m = Eigen::Map<const Eigen::Matrix4f>(gizmo_matrix).cast<double>();
            Eigen::Matrix3d M = m.topLeftCorner<3, 3>();
            // An axis-tip box scales one axis only; the axis that moved most sets the uniform scale.
            double s = G.s;
            for (int i = 0; i < 3; i++)
            {
                const double n = M.col(i).norm();
                if (n > 0.0 && std::abs(std::log(n / G.s)) > std::abs(std::log(s / G.s)))
                    s = n;
            }
            for (int i = 0; i < 3; i++)
                M.col(i).normalize();
            G.q = Eigen::Quaterniond(M).normalized();
            G.s = s;
            G.t = m.block<3, 1>(0, 3);
            clearRefinement();
        }
    }

    //! Starts from a saved COLMAP -> lidar similarity, {"scale": s, "R": [[..], [..], [..]], "t": [x, y, z]}
    //! with lidar = s * R * colmap + t (the map_aligned/align.json layout).
    void loadAlignment(const std::string& file)
    {
        if (sfm.points.empty() || lidar.clouds.empty())
        {
            setStatus("Open the COLMAP model and the lidar LAZ before loading an alignment");
            return;
        }
        double s = 0.0;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        try
        {
            std::ifstream in(file);
            if (!in)
            {
                setStatus("Cannot open " + file);
                return;
            }
            const nlohmann::json j = nlohmann::json::parse(in);
            s = j.at("scale").get<double>();
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                    R(r, c) = j.at("R").at(r).at(c).get<double>();
                t[r] = j.at("t").at(r).get<double>();
            }
        } catch (const std::exception& e)
        {
            setStatus("Cannot read alignment " + file + ": " + e.what());
            return;
        }
        if (!(s > 0.0) || (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() > 1e-3 || R.determinant() < 0.0)
        {
            setStatus("Alignment " + file + " needs scale > 0 and R a rotation");
            return;
        }

        G = cs::Similarity::fromScaledRotation(s, R, t - lidar.origin, G.pivot);
        clearRefinement();
        setNewRotationCenter(G.t);
        std::ostringstream ss;
        ss << "Loaded alignment " << file << ", scale " << std::setprecision(6) << s;
        setStatus(ss.str());
    }

    void openAlignment()
    {
        const std::string file =
            mandeye::fd::OpenFileDialogOneFile("COLMAP to lidar alignment", { "Alignment (*.json)", "*.json", "All files", "*" });
        if (!file.empty())
            loadAlignment(file);
    }

    void menuBar()
    {
        if (!ImGui::BeginMainMenuBar())
            return;
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open COLMAP model..."))
            {
                const std::string dir = mandeye::fd::SelectFolder("COLMAP model folder (cameras/images/points3D)");
                if (!dir.empty())
                    loadColmap(dir);
            }
            if (ImGui::MenuItem("Open lidar LAZ..."))
            {
                const std::string file = mandeye::fd::OpenFileDialogOneFile("Lidar map", mandeye::fd::LAS_LAZ_filter);
                if (!file.empty())
                    loadLidar(file);
            }
            if (ImGui::MenuItem("Load alignment (align.json)..."))
                openAlignment();
            ImGui::Separator();
            if (ImGui::MenuItem("Save COLMAP model..."))
                saveColmap();
            if (ImGui::MenuItem("Save TUM trajectory..."))
                saveTum();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View"))
        {
            camMenu();
            if (ImGui::BeginMenu("Lidar color"))
            {
                const std::pair<const char*, ScanColorMode> items[] = { { "Height", ScanColorMode::Elevation },
                                                                        { "Intensity", ScanColorMode::Intensity },
                                                                        { "Flat", ScanColorMode::Flat } };
                for (const auto& [label, mode] : items)
                    if (ImGui::MenuItem(label, nullptr, lidar_color_mode == mode))
                        lidar_color_mode = mode;
                ImGui::EndMenu();
            }
            ImGui::MenuItem("Compass", "key C", &compass_ruler);
            ImGui::MenuItem("Axes", "key X", &show_axes);
            ImGui::MenuItem("Orthographic", "key O", &camera.isOrtho);
            ImGui::MenuItem("Center of rotation", "Shift+R", &cor_gui);
            ImGui::MenuItem("Camera view", "Shift+right click", &camera_view_open);
            ImGui::EndMenu();
        }
        ImGui::Text("(%.1f FPS)", ImGui::GetIO().Framerate);

        ImGui::SameLine(
            ImGui::GetWindowWidth() - ImGui::CalcTextSize("Info").x - ImGui::GetStyle().ItemSpacing.x * 2 -
            ImGui::GetStyle().FramePadding.x * 2);
        if (ImGui::SmallButton("Info"))
            info_gui = !info_gui;
        ImGui::EndMainMenuBar();
    }

    void scheduleInputs(const char* id, cs::AssociationSchedule& s)
    {
        ImGui::PushID(id);
        ImGui::SetNextItemWidth(120);
        ImGui::InputDouble("Start distance [m]", &s.max_dist_start, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Correspondence distance of the first round; shrinks to the end distance");
        ImGui::SetNextItemWidth(120);
        ImGui::InputDouble("End distance [m]", &s.max_dist_end, 0.0, 0.0, "%.3f");
        ImGui::SetNextItemWidth(120);
        ImGui::InputInt("Rounds", &s.rounds);
        ImGui::SetNextItemWidth(120);
        ImGui::InputInt("Iterations / round", &s.iterations);
        s.max_dist_start = std::max(s.max_dist_start, 1e-3);
        s.max_dist_end = std::max(s.max_dist_end, 1e-3);
        s.rounds = std::max(s.rounds, 1);
        s.iterations = std::max(s.iterations, 1);
        ImGui::PopID();
    }

    void positiveInput(const char* label, double& v, const char* format, const char* tooltip)
    {
        ImGui::SetNextItemWidth(120);
        ImGui::InputDouble(label, &v, 0.0, 0.0, format);
        v = std::max(v, 1e-9);
        if (tooltip && ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", tooltip);
    }

    void panel()
    {
        ImGui::SetNextWindowPos(ImVec2(20, 40), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(430, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("COLMAP -> lidar");

        size_t used = std::count(sfm.active.begin(), sfm.active.end(), 1);
        ImGui::TextWrapped("COLMAP: %s", sfm.dir.empty() ? "(none)" : sfm.dir.c_str());
        ImGui::Text("  %zu images, %zu points (%zu used)", sfm.T_wc.size(), sfm.points.size(), used);
        ImGui::TextWrapped("Lidar: %s", lidar.file.empty() ? "(none)" : lidar.file.c_str());
        ImGui::Text(
            "  %zu points, %s",
            lidar.clouds.empty() ? size_t(0) : lidar.clouds[0].points_local.size(),
            lidar.planes_voxel > 0 ? (std::to_string(lidar.planes.size()) + " planar cells").c_str() : "planes not built yet");

        if (ImGui::CollapsingHeader("Points used"))
        {
            bool changed = false;
            ImGui::SetNextItemWidth(120);
            changed |= ImGui::InputInt("Min track length", &min_track_length);
            ImGui::SetNextItemWidth(120);
            changed |= ImGui::InputFloat("Max reprojection error [px]", &max_reprojection_error, 0.0f, 0.0f, "%.2f");
            min_track_length = std::max(min_track_length, 1);
            if (changed && !sfm.points.empty())
                updateActive();
        }

        if (ImGui::CollapsingHeader("Lidar planes"))
        {
            positiveInput("Voxel size [m]", voxel_size, "%.3f", "Edge of the cells a plane is fitted in; rebuilt on the next ICP/refine");
            positiveInput(
                "Max flatness", max_flatness, "%.3f",
                "A cell counts as a plane when its smallest spread is below this fraction of the middle one; raise it for rough scenes");
            if (ImGui::Button("Rebuild planes now") && !lidar.clouds.empty())
            {
                lidar.planes_voxel = -1.0;
                ensurePlaneMap();
            }
        }

        if (ImGui::CollapsingHeader("Global alignment", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Checkbox("Gizmo", &gizmo_enabled);
            ImGui::SameLine();
            if (ImGui::Button("Center view on model"))
                setNewRotationCenter(G.t);
            ImGui::SameLine();
            if (ImGui::Button("Reset"))
                resetAlignment();
            ImGui::SameLine();
            if (ImGui::Button("Load align.json..."))
                openAlignment();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Start from a saved similarity: {\"scale\", \"R\", \"t\"} with lidar = scale * R * colmap + t");

            double scale = G.s;
            ImGui::SetNextItemWidth(120);
            if (ImGui::InputDouble("Scale", &scale, 0.0, 0.0, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue) && scale > 0.0)
            {
                G.s = scale;
                clearRefinement();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "COLMAP units to metres, about the model's centroid (Enter to apply).\n"
                    "Or drag the gizmo's scale ring (around its centre) left/right, or any axis-tip box.");

            ImGui::Separator();
            scheduleInputs("icp", align_params.schedule);
            positiveInput("Point sigma [m]##icp", align_params.sigma_point, "%.3f", "Expected point-to-plane noise");
            ImGui::Checkbox("Estimate scale", &align_params.estimate_scale);
            if (ImGui::Button("ICP (Sim3)"))
                runIcp();
        }

        if (ImGui::CollapsingHeader("Per-camera refinement", ImGuiTreeNodeFlags_DefaultOpen))
        {
            scheduleInputs("refine", refine_params.schedule);
            positiveInput("Point sigma [m]##refine", refine_params.sigma_point, "%.3f", "Expected point-to-plane noise");
            positiveInput(
                "Smooth translation [m]", refine_params.sigma_translation, "%.4f",
                "How far the motion between consecutive cameras may deviate from COLMAP's");
            positiveInput(
                "Smooth rotation [deg]", refine_params.sigma_rotation_deg, "%.4f",
                "How far the rotation between consecutive cameras may deviate from COLMAP's");
            positiveInput("Smooth scale", refine_params.sigma_scale, "%.5f", "Allowed change of log(scale) between consecutive cameras");
            ImGui::Checkbox("Prior to the global alignment", &refine_params.use_prior);
            if (refine_params.use_prior)
            {
                positiveInput("Prior translation [m]", refine_params.prior_sigma_translation, "%.2f", nullptr);
                positiveInput("Prior rotation [deg]", refine_params.prior_sigma_rotation_deg, "%.2f", nullptr);
            }
            if (ImGui::Button(refined.empty() ? "Refine per camera" : "Refine again"))
                runRefine();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Runs from the current refinement if there is one; repeat when the ends start beyond the start distance");
            ImGui::SameLine();
            ImGui::BeginDisabled(refined.empty());
            if (ImGui::Button("Discard"))
                clearRefinement();
            ImGui::EndDisabled();

            if (!scale_plot.empty())
            {
                ImGui::PlotLines(
                    "##scale", scale_plot.data(), static_cast<int>(scale_plot.size()), 0, "scale / global along the trajectory", FLT_MAX,
                    FLT_MAX, ImVec2(ImGui::GetContentRegionAvail().x, 90));
            }
        }

        if (ImGui::CollapsingHeader("Output"))
        {
            auto cameraLabel = [](uint32_t id)
            {
                if (id == 0)
                    return std::string("All cameras");
                const auto it = sfm.model.cameras.find(id);
                return "Camera " + std::to_string(id) + (it != sfm.model.cameras.end() ? " (" + it->second.model + ")" : "");
            };
            ImGui::SetNextItemWidth(200);
            if (ImGui::BeginCombo("TUM camera", cameraLabel(tum_camera_id).c_str()))
            {
                if (ImGui::Selectable(cameraLabel(0).c_str(), tum_camera_id == 0))
                    tum_camera_id = 0;
                for (const auto& [id, cam] : sfm.model.cameras)
                    if (ImGui::Selectable(cameraLabel(id).c_str(), tum_camera_id == id))
                        tum_camera_id = id;
                ImGui::EndCombo();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Which camera's images go into the TUM file; a rig's lenses share timestamps");
            if (ImGui::Button("Save COLMAP model..."))
                saveColmap();
            ImGui::SameLine();
            if (ImGui::Button("Save TUM trajectory..."))
                saveTum();
        }

        if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Checkbox("Lidar", &show_lidar);
            ImGui::SameLine();
            ImGui::Checkbox("Before (orange)", &show_before);
            ImGui::SameLine();
            ImGui::Checkbox("After (green)", &show_after);
            ImGui::SetNextItemWidth(120);
            ImGui::SliderInt("COLMAP point size", &sfm_point_size, 1, 10);
            ImGui::Checkbox("Camera trajectory", &show_trajectory);
            ImGui::SameLine();
            ImGui::Checkbox("On top of the lidar", &trajectory_on_top);
            ImGui::SetNextItemWidth(120);
            ImGui::SliderInt("Camera dot size", &trajectory_point_size, 1, 20);
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("Trajectory line width", &trajectory_line_width, 1.0f, 8.0f, "%.0f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Some OpenGL drivers draw core-profile lines 1 px wide whatever this says");
            if (ImGui::Button(camera_view_open ? "Close camera view" : "Camera view..."))
                camera_view_open = !camera_view_open;
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("The lidar seen through one COLMAP camera, next to its photo (or Shift + right click a camera)");
        }

        if (!status.empty())
        {
            ImGui::Separator();
            ImGui::TextWrapped("%s", status.c_str());
        }
        ImGui::End();
    }

    void imageOrPlaceholder(const GpuImage& image, float width, float height, const char* placeholder)
    {
        if (image.valid)
            rlImGuiImageSize(&image.texture, static_cast<int>(width), static_cast<int>(height));
        else
        {
            const ImVec2 p = ImGui::GetCursorScreenPos();
            ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + width, p.y + height), IM_COL32(25, 25, 25, 255));
            ImGui::GetWindowDrawList()->AddText(ImVec2(p.x + 8, p.y + 8), IM_COL32(200, 200, 200, 255), placeholder);
            ImGui::Dummy(ImVec2(width, height));
        }
    }

    //! The lidar seen through the selected COLMAP camera (its own model, current pose) next to its photo.
    void cameraViewWindow()
    {
        if (!camera_view_open)
            return;
        ImGui::SetNextWindowSize(ImVec2(900, 640), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(1.0f);
        if (!ImGui::Begin("Camera view", &camera_view_open))
        {
            ImGui::End();
            return;
        }
        if (sfm.T_wc.empty() || lidar.clouds.empty())
        {
            ImGui::TextWrapped("Open a COLMAP model and a lidar LAZ first.");
            ImGui::End();
            return;
        }

        const int n = static_cast<int>(sfm.T_wc.size());
        selected_camera = std::clamp(selected_camera, 0, n - 1);
        if (ImGui::ArrowButton("##prev", ImGuiDir_Left))
            selected_camera = std::max(0, selected_camera - 1);
        ImGui::SameLine();
        if (ImGui::ArrowButton("##next", ImGuiDir_Right))
            selected_camera = std::min(n - 1, selected_camera + 1);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        ImGui::SliderInt("##camera", &selected_camera, 0, n - 1, "image %d (capture order)");
        ImGui::SameLine();
        if (ImGui::Button("Center 3D view"))
            setNewRotationCenter(currentCamera(selected_camera).t);

        const cs::Image& image = sfm.model.images.at(sfm.image_ids[selected_camera]);
        const cs::Camera& cam = sfm.model.cameras.at(image.camera_id);
        ImGui::Text(
            "%s   camera %u, %s %llux%llu%s", image.name.c_str(), cam.id, cam.model.c_str(), static_cast<unsigned long long>(cam.width),
            static_cast<unsigned long long>(cam.height), refined.empty() ? "" : ", refined pose");
        if (!cs::isProjectionSupported(cam.model))
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Projecting into %s cameras is not supported.", cam.model.c_str());

        ImGui::BeginDisabled(!lidar.coloring.has_rgb);
        if (ImGui::RadioButton("LAZ RGB", projection.color == ProjectionColor::Rgb))
            projection.color = ProjectionColor::Rgb;
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::RadioButton("Intensity", projection.color == ProjectionColor::Intensity))
            projection.color = ProjectionColor::Intensity;
        ImGui::SameLine();
        if (ImGui::RadioButton("Range", projection.color == ProjectionColor::Range))
            projection.color = ProjectionColor::Range;
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        const std::pair<const char*, CameraViewMode> modes[] = { { "Lidar", CameraViewMode::Lidar },
                                                                 { "Photo", CameraViewMode::Photo },
                                                                 { "Side by side", CameraViewMode::SideBySide },
                                                                 { "Blend", CameraViewMode::Blend } };
        for (const auto& [label, mode] : modes)
        {
            ImGui::SameLine();
            if (ImGui::RadioButton(label, camera_view_mode == mode))
                camera_view_mode = mode;
        }
        ImGui::SameLine();
        ImGui::Checkbox("Flicker", &flicker);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Alternate lidar and photo twice a second");
        if (camera_view_mode == CameraViewMode::Blend && !flicker)
        {
            ImGui::SetNextItemWidth(200);
            ImGui::SliderFloat("Lidar opacity", &blend_alpha, 0.0f, 1.0f, "%.2f");
        }

        if (ImGui::TreeNode("Projection settings"))
        {
            ImGui::SetNextItemWidth(120);
            ImGui::InputInt("Image width [px]", &projection.width, 64, 256);
            projection.width = std::clamp(projection.width, 128, 4096);
            ImGui::SetNextItemWidth(120);
            ImGui::InputInt("Lidar point stride", &projection.stride);
            projection.stride = std::clamp(projection.stride, 1, 1000);
            ImGui::SetNextItemWidth(120);
            ImGui::InputDouble("Max range [m]", &projection.max_range, 0.0, 0.0, "%.1f");
            projection.max_range = std::max(projection.max_range, 0.5);
            ImGui::SetNextItemWidth(120);
            ImGui::InputDouble("Max angle from axis [deg]", &projection.max_angle_deg, 0.0, 0.0, "%.0f");
            projection.max_angle_deg = std::clamp(projection.max_angle_deg, 1.0, 180.0);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Points further off axis are skipped. The lens model's own limit (where its distortion\n"
                    "polynomial stops growing) always applies: %.0f deg for this camera.",
                    cs::CameraProjector(cam).maxAngleDeg());
            ImGui::SetNextItemWidth(120);
            ImGui::SliderInt("Splat radius [px]", &projection.splat_radius, 0, 5);

            char buf[1024];
            std::snprintf(buf, sizeof(buf), "%s", images_dir.c_str());
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 160);
            if (ImGui::InputText("##images", buf, sizeof(buf), ImGuiInputTextFlags_EnterReturnsTrue))
                images_dir = buf;
            ImGui::SameLine();
            if (ImGui::Button("Images folder..."))
            {
                const std::string dir = mandeye::fd::SelectFolder("Folder the COLMAP image names are relative to");
                if (!dir.empty())
                    images_dir = dir;
            }
            ImGui::TreePop();
        }

        // Re-render when the camera, its pose or the settings change, but not on every frame of a gizmo drag.
        const double scale = double(projection.width) / double(std::max<uint64_t>(cam.width, 1));
        const int W = static_cast<int>(std::lround(cam.width * scale));
        const int H = static_cast<int>(std::lround(cam.height * scale));
        const cs::CameraSim3 pose = currentCamera(selected_camera);
        const ProjectionKey key{ selected_camera, pose.q.coeffs(), pose.t, projection, &lidar.clouds[0] };
        if (!(key == projection_key) && !ImGuizmo::IsUsing())
        {
            const cv::Mat img = renderLidarProjection(cam, pose, lidar.clouds[0], lidar.coloring, projection);
            if (img.empty())
                projection_image.release();
            else
                projection_image.upload(img);
            projection_key = key;
        }
        const PhotoKey photo{ selected_camera, images_dir.empty() ? std::string() : (fs::path(images_dir) / image.name).string(), W };
        if (!(photo == photo_key))
        {
            const cv::Mat img = photo.file.empty() ? cv::Mat() : loadPhoto(photo.file, W, H);
            photo_missing = img.empty();
            if (img.empty())
                photo_image.release();
            else
                photo_image.upload(img);
            photo_key = photo;
        }

        CameraViewMode mode = camera_view_mode;
        if (flicker)
            mode = static_cast<int>(GetTime() * 2.0) % 2 ? CameraViewMode::Photo : CameraViewMode::Lidar;
        const float avail = ImGui::GetContentRegionAvail().x;
        const float aspect = float(H) / float(W);
        const std::string no_photo = "No photo at " + (photo.file.empty() ? std::string("(set the images folder)") : photo.file);
        switch (mode)
        {
        case CameraViewMode::Lidar:
            imageOrPlaceholder(projection_image, avail, avail * aspect, "No projection");
            break;
        case CameraViewMode::Photo:
            imageOrPlaceholder(photo_image, avail, avail * aspect, no_photo.c_str());
            break;
        case CameraViewMode::SideBySide:
        {
            const float w = (avail - ImGui::GetStyle().ItemSpacing.x) / 2.0f;
            imageOrPlaceholder(projection_image, w, w * aspect, "No projection");
            ImGui::SameLine();
            imageOrPlaceholder(photo_image, w, w * aspect, no_photo.c_str());
            break;
        }
        case CameraViewMode::Blend:
        {
            const ImVec2 p = ImGui::GetCursorScreenPos();
            const ImVec2 q(p.x + avail, p.y + avail * aspect);
            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddRectFilled(p, q, IM_COL32(25, 25, 25, 255));
            if (photo_image.valid)
                dl->AddImage(ImTextureID(photo_image.texture.id), p, q);
            if (projection_image.valid)
                dl->AddImage(
                    ImTextureID(projection_image.texture.id), p, q, ImVec2(0, 0), ImVec2(1, 1),
                    IM_COL32(255, 255, 255, static_cast<int>(255 * blend_alpha)));
            ImGui::Dummy(ImVec2(avail, avail * aspect));
            break;
        }
        }
        if (photo_missing && mode != CameraViewMode::Lidar)
            ImGui::TextWrapped("%s", no_photo.c_str());
        ImGui::End();
    }

    void display()
    {
        beginScene3D();
        showAxes();
        drawScene();

        rlImGuiBegin();
        ShowMainDockSpace();
        gizmo();
        view_kbd_shortcuts();
        menuBar();
        panel();
        cameraViewWindow();
        cor_window();
        info_window(infoLines, appShortcuts);
        endScene3D();
        if (compass_ruler)
            drawMiniCompassWithRuler();
        rlImGuiEnd();
    }

    void mouse(int glut_button, int state, int x, int y)
    {
        const ImGuiIO& io = ImGui::GetIO();
        if (io.WantCaptureMouse)
            return;
        if ((glut_button == GLUT_MIDDLE_BUTTON || glut_button == GLUT_RIGHT_BUTTON) && state == GLUT_DOWN && io.KeyCtrl)
            setNewRotationCenter(x, y, 0.0);
        if (glut_button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN && io.KeyShift && !io.KeyCtrl && !sfm.T_wc.empty())
        {
            // The camera whose centre passes closest to the mouse ray.
            const Ray ray = camera.eulerScreenRay(x, y, GetScreenWidth(), GetScreenHeight());
            const Eigen::Vector3d origin(ray.position.x, ray.position.y, ray.position.z);
            const Eigen::Vector3d dir(ray.direction.x, ray.direction.y, ray.direction.z);
            double best = std::numeric_limits<double>::max();
            for (size_t k = 0; k < sfm.T_wc.size(); k++)
            {
                const double d = raylib_widgets::distancePointToLine(currentCamera(k).t, origin, dir);
                if (d < best)
                {
                    best = d;
                    selected_camera = static_cast<int>(k);
                }
            }
            camera_view_open = true;
        }
        if (state == GLUT_DOWN)
            mouse_buttons |= 1 << glut_button;
        else if (state == GLUT_UP)
            mouse_buttons = 0;
        mouse_old_x = x;
        mouse_old_y = y;
    }
} // namespace

int main(int argc, char* argv[])
{
    try
    {
        if (checkClHelp(argc, argv))
        {
            std::cout << winTitle << "\n\n"
                      << "USAGE:\n"
                      << fs::path(argv[0]).stem().string() << " [<colmap_model_dir>] [<lidar.laz>] [<align.json>] [--camera <n>]\n\n"
                      << "   <colmap_model_dir>   folder with cameras/images/points3D (.bin or .txt)\n"
                      << "   <lidar.laz>          lidar map (.laz / .las)\n"
                      << "   <align.json>         starting alignment {scale, R, t}: lidar = scale * R * colmap + t\n"
                      << "   --camera <n>         open the camera view at image n (capture order)\n"
                      << "   -h, /h, --help, /?   show this help and exit\n\n";
            return 0;
        }

        initGL(winTitle, display, mouse);
        lidar_renderer = std::make_unique<ScanRenderer>();
        lidar_renderer->init();
        sfm_renderer = std::make_unique<ScanRenderer>();
        sfm_renderer->init();

        // Lidar first, so the model lands on its local origin.
        std::string colmap_dir, lidar_file, align_file;
        int start_camera = -1;
        for (int i = 1; i < argc; i++)
        {
            if (std::string(argv[i]) == "--camera" && i + 1 < argc)
            {
                start_camera = std::atoi(argv[++i]);
                continue;
            }
            const fs::path arg(argv[i]);
            std::string ext = arg.extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (fs::is_directory(arg))
                colmap_dir = arg.string();
            else if (ext == ".laz" || ext == ".las")
                lidar_file = arg.string();
            else if (ext == ".json")
                align_file = arg.string();
            else
                std::cerr << "Ignoring argument: '" << arg.string() << "'" << std::endl;
        }
        if (!lidar_file.empty())
            loadLidar(lidar_file);
        if (!colmap_dir.empty())
            loadColmap(colmap_dir);
        if (!align_file.empty())
            loadAlignment(align_file);
        if (start_camera >= 0)
        {
            // After loading: a new model resets the selection.
            selected_camera = std::clamp(start_camera, 0, std::max(0, static_cast<int>(sfm.T_wc.size()) - 1));
            camera_view_open = true;
        }

        mainLoop();
        shutdownGL(releaseGpu);
    } catch (const std::bad_alloc& e)
    {
        std::cerr << "System is out of memory : " << e.what() << std::endl;
        mandeye::fd::OutOfMemMessage();
    }
    return 0;
}
