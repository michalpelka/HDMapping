#include "colmap_model.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

namespace colmap_sim3
{
    namespace
    {
        struct CameraModelInfo
        {
            int id;
            const char* name;
            int num_params;
        };

        // Same ids and parameter counts as COLMAP's src/colmap/sensor/models.h.
        constexpr CameraModelInfo kCameraModels[] = {
            { 0, "SIMPLE_PINHOLE", 3 },
            { 1, "PINHOLE", 4 },
            { 2, "SIMPLE_RADIAL", 4 },
            { 3, "RADIAL", 5 },
            { 4, "OPENCV", 8 },
            { 5, "OPENCV_FISHEYE", 8 },
            { 6, "FULL_OPENCV", 12 },
            { 7, "FOV", 5 },
            { 8, "SIMPLE_RADIAL_FISHEYE", 4 },
            { 9, "RADIAL_FISHEYE", 5 },
            { 10, "THIN_PRISM_FISHEYE", 12 },
            { 11, "RAD_TAN_THIN_PRISM_FISHEYE", 16 },
        };

        std::string trim(const std::string& s)
        {
            const auto b = s.find_first_not_of(" \t\r\n");
            if (b == std::string::npos)
                return "";
            const auto e = s.find_last_not_of(" \t\r\n");
            return s.substr(b, e - b + 1);
        }

        //! Next non-empty, non-comment line; false at end of file.
        bool nextDataLine(std::istream& in, std::string& line)
        {
            while (std::getline(in, line))
            {
                line = trim(line);
                if (!line.empty() && line[0] != '#')
                    return true;
            }
            return false;
        }

        template <typename T>
        bool readBin(std::istream& in, T& value)
        {
            in.read(reinterpret_cast<char*>(&value), sizeof(T));
            return static_cast<bool>(in);
        }

        template <typename T>
        void writeBin(std::ostream& out, const T& value)
        {
            out.write(reinterpret_cast<const char*>(&value), sizeof(T));
        }

        bool readCamerasText(const fs::path& file, Model& model, std::string& error)
        {
            std::ifstream in(file);
            if (!in)
            {
                error = "cannot open " + file.string();
                return false;
            }
            std::string line;
            while (nextDataLine(in, line))
            {
                std::istringstream ss(line);
                Camera cam;
                ss >> cam.id >> cam.model >> cam.width >> cam.height;
                double p;
                while (ss >> p)
                    cam.params.push_back(p);
                const int n = cameraModelNumParams(cam.model);
                if (n < 0 || static_cast<int>(cam.params.size()) != n)
                {
                    error = file.string() + ": bad camera line '" + line + "'";
                    return false;
                }
                model.cameras[cam.id] = cam;
            }
            return true;
        }

        bool readImagesText(const fs::path& file, Model& model, std::string& error)
        {
            std::ifstream in(file);
            if (!in)
            {
                error = "cannot open " + file.string();
                return false;
            }
            std::string line;
            while (nextDataLine(in, line))
            {
                std::istringstream ss(line);
                Image img;
                double qw, qx, qy, qz;
                ss >> img.id >> qw >> qx >> qy >> qz >> img.t_cw.x() >> img.t_cw.y() >> img.t_cw.z() >> img.camera_id;
                if (!ss)
                {
                    error = file.string() + ": bad image line '" + line + "'";
                    return false;
                }
                std::string name;
                std::getline(ss, name);
                img.name = trim(name);
                img.q_cw = Eigen::Quaterniond(qw, qx, qy, qz).normalized();

                // The POINTS2D line follows directly and may be empty, so it is not skipped like a blank line.
                std::string points_line;
                if (!std::getline(in, points_line))
                {
                    error = file.string() + ": missing POINTS2D line for image " + std::to_string(img.id);
                    return false;
                }
                std::istringstream ps(points_line);
                double x, y;
                int64_t id;
                while (ps >> x >> y >> id)
                {
                    Observation2D obs;
                    obs.xy = Eigen::Vector2d(x, y);
                    obs.point3D_id = id < 0 ? kInvalidPoint3DId : static_cast<uint64_t>(id);
                    img.points2D.push_back(obs);
                }
                model.images[img.id] = std::move(img);
            }
            return true;
        }

        bool readPointsText(const fs::path& file, Model& model, std::string& error)
        {
            std::ifstream in(file);
            if (!in)
            {
                error = "cannot open " + file.string();
                return false;
            }
            std::string line;
            while (nextDataLine(in, line))
            {
                std::istringstream ss(line);
                Point3D pt;
                int r, g, b;
                ss >> pt.id >> pt.xyz.x() >> pt.xyz.y() >> pt.xyz.z() >> r >> g >> b >> pt.error;
                if (!ss)
                {
                    error = file.string() + ": bad point line '" + line + "'";
                    return false;
                }
                pt.rgb = { static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b) };
                TrackElement el;
                while (ss >> el.image_id >> el.point2D_idx)
                    pt.track.push_back(el);
                model.points[pt.id] = std::move(pt);
            }
            return true;
        }

        bool readCamerasBinary(const fs::path& file, Model& model, std::string& error)
        {
            std::ifstream in(file, std::ios::binary);
            if (!in)
            {
                error = "cannot open " + file.string();
                return false;
            }
            uint64_t count = 0;
            if (!readBin(in, count))
            {
                error = file.string() + ": truncated header";
                return false;
            }
            for (uint64_t i = 0; i < count; i++)
            {
                Camera cam;
                int32_t model_id = 0;
                if (!readBin(in, cam.id) || !readBin(in, model_id) || !readBin(in, cam.width) || !readBin(in, cam.height))
                {
                    error = file.string() + ": truncated camera record";
                    return false;
                }
                cam.model = cameraModelName(model_id);
                const int n = cameraModelNumParams(cam.model);
                if (n < 0)
                {
                    error = file.string() + ": unknown camera model id " + std::to_string(model_id);
                    return false;
                }
                cam.params.resize(n);
                for (auto& p : cam.params)
                    if (!readBin(in, p))
                    {
                        error = file.string() + ": truncated camera params";
                        return false;
                    }
                model.cameras[cam.id] = cam;
            }
            return true;
        }

        bool readImagesBinary(const fs::path& file, Model& model, std::string& error)
        {
            std::ifstream in(file, std::ios::binary);
            if (!in)
            {
                error = "cannot open " + file.string();
                return false;
            }
            uint64_t count = 0;
            if (!readBin(in, count))
            {
                error = file.string() + ": truncated header";
                return false;
            }
            for (uint64_t i = 0; i < count; i++)
            {
                Image img;
                double q[4], t[3];
                bool ok = readBin(in, img.id);
                for (double& v : q)
                    ok = ok && readBin(in, v);
                for (double& v : t)
                    ok = ok && readBin(in, v);
                ok = ok && readBin(in, img.camera_id);
                if (!ok)
                {
                    error = file.string() + ": truncated image record";
                    return false;
                }
                img.q_cw = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized();
                img.t_cw = Eigen::Vector3d(t[0], t[1], t[2]);
                char c;
                while (readBin(in, c) && c != '\0')
                    img.name += c;
                uint64_t num_points2D = 0;
                if (!readBin(in, num_points2D))
                {
                    error = file.string() + ": truncated image record";
                    return false;
                }
                img.points2D.resize(num_points2D);
                for (auto& obs : img.points2D)
                    if (!readBin(in, obs.xy.x()) || !readBin(in, obs.xy.y()) || !readBin(in, obs.point3D_id))
                    {
                        error = file.string() + ": truncated POINTS2D";
                        return false;
                    }
                model.images[img.id] = std::move(img);
            }
            return true;
        }

        bool readPointsBinary(const fs::path& file, Model& model, std::string& error)
        {
            std::ifstream in(file, std::ios::binary);
            if (!in)
            {
                error = "cannot open " + file.string();
                return false;
            }
            uint64_t count = 0;
            if (!readBin(in, count))
            {
                error = file.string() + ": truncated header";
                return false;
            }
            for (uint64_t i = 0; i < count; i++)
            {
                Point3D pt;
                uint64_t track_length = 0;
                if (!readBin(in, pt.id) || !readBin(in, pt.xyz.x()) || !readBin(in, pt.xyz.y()) || !readBin(in, pt.xyz.z()) ||
                    !readBin(in, pt.rgb[0]) || !readBin(in, pt.rgb[1]) || !readBin(in, pt.rgb[2]) || !readBin(in, pt.error) ||
                    !readBin(in, track_length))
                {
                    error = file.string() + ": truncated point record";
                    return false;
                }
                pt.track.resize(track_length);
                for (auto& el : pt.track)
                    if (!readBin(in, el.image_id) || !readBin(in, el.point2D_idx))
                    {
                        error = file.string() + ": truncated track";
                        return false;
                    }
                model.points[pt.id] = std::move(pt);
            }
            return true;
        }
    } // namespace

    Eigen::Affine3d Image::worldFromCamera() const
    {
        Eigen::Affine3d T_cw = Eigen::Affine3d::Identity();
        T_cw.linear() = q_cw.toRotationMatrix();
        T_cw.translation() = t_cw;
        return T_cw.inverse(Eigen::Isometry);
    }

    void Image::setWorldFromCamera(const Eigen::Affine3d& T_wc)
    {
        const Eigen::Affine3d T_cw = T_wc.inverse(Eigen::Isometry);
        q_cw = Eigen::Quaterniond(T_cw.linear()).normalized();
        t_cw = T_cw.translation();
    }

    int cameraModelId(const std::string& model)
    {
        for (const auto& m : kCameraModels)
            if (model == m.name)
                return m.id;
        return -1;
    }

    std::string cameraModelName(int id)
    {
        for (const auto& m : kCameraModels)
            if (id == m.id)
                return m.name;
        return "";
    }

    int cameraModelNumParams(const std::string& model)
    {
        for (const auto& m : kCameraModels)
            if (model == m.name)
                return m.num_params;
        return -1;
    }

    bool readModelText(const std::string& dir, Model& model, std::string& error)
    {
        model = Model{};
        const fs::path d(dir);
        return readCamerasText(d / "cameras.txt", model, error) && readImagesText(d / "images.txt", model, error) &&
            readPointsText(d / "points3D.txt", model, error);
    }

    bool readModelBinary(const std::string& dir, Model& model, std::string& error)
    {
        model = Model{};
        const fs::path d(dir);
        return readCamerasBinary(d / "cameras.bin", model, error) && readImagesBinary(d / "images.bin", model, error) &&
            readPointsBinary(d / "points3D.bin", model, error);
    }

    bool readModel(const std::string& dir, Model& model, std::string& error)
    {
        const fs::path d(dir);
        if (fs::exists(d / "cameras.bin") && fs::exists(d / "images.bin") && fs::exists(d / "points3D.bin"))
            return readModelBinary(dir, model, error);
        if (fs::exists(d / "cameras.txt") && fs::exists(d / "images.txt") && fs::exists(d / "points3D.txt"))
            return readModelText(dir, model, error);
        error = "no cameras/images/points3D (.bin or .txt) in " + dir;
        return false;
    }

    bool writeModelText(const std::string& dir, const Model& model, std::string& error)
    {
        const fs::path d(dir);
        std::ofstream cams(d / "cameras.txt");
        std::ofstream imgs(d / "images.txt");
        std::ofstream pts(d / "points3D.txt");
        if (!cams || !imgs || !pts)
        {
            error = "cannot write the model into " + dir;
            return false;
        }
        for (auto* f : { &cams, &imgs, &pts })
            *f << std::setprecision(17);

        cams << "# Camera list with one line of data per camera:\n"
             << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n"
             << "# Number of cameras: " << model.cameras.size() << "\n";
        for (const auto& [id, cam] : model.cameras)
        {
            cams << cam.id << " " << cam.model << " " << cam.width << " " << cam.height;
            for (double p : cam.params)
                cams << " " << p;
            cams << "\n";
        }

        imgs << "# Image list with two lines of data per image:\n"
             << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n"
             << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n"
             << "# Number of images: " << model.images.size() << "\n";
        for (const auto& [id, img] : model.images)
        {
            imgs << img.id << " " << img.q_cw.w() << " " << img.q_cw.x() << " " << img.q_cw.y() << " " << img.q_cw.z() << " "
                 << img.t_cw.x() << " " << img.t_cw.y() << " " << img.t_cw.z() << " " << img.camera_id << " " << img.name << "\n";
            for (size_t k = 0; k < img.points2D.size(); k++)
            {
                const auto& obs = img.points2D[k];
                if (k > 0)
                    imgs << " ";
                imgs << obs.xy.x() << " " << obs.xy.y() << " ";
                if (obs.point3D_id == kInvalidPoint3DId)
                    imgs << -1;
                else
                    imgs << obs.point3D_id;
            }
            imgs << "\n";
        }

        pts << "# 3D point list with one line of data per point:\n"
            << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n"
            << "# Number of points: " << model.points.size() << "\n";
        for (const auto& [id, pt] : model.points)
        {
            pts << pt.id << " " << pt.xyz.x() << " " << pt.xyz.y() << " " << pt.xyz.z() << " " << int(pt.rgb[0]) << " " << int(pt.rgb[1])
                << " " << int(pt.rgb[2]) << " " << pt.error;
            for (const auto& el : pt.track)
                pts << " " << el.image_id << " " << el.point2D_idx;
            pts << "\n";
        }

        if (!cams || !imgs || !pts)
        {
            error = "write error in " + dir;
            return false;
        }
        return true;
    }

    bool writeModelBinary(const std::string& dir, const Model& model, std::string& error)
    {
        const fs::path d(dir);
        std::ofstream cams(d / "cameras.bin", std::ios::binary);
        std::ofstream imgs(d / "images.bin", std::ios::binary);
        std::ofstream pts(d / "points3D.bin", std::ios::binary);
        if (!cams || !imgs || !pts)
        {
            error = "cannot write the model into " + dir;
            return false;
        }

        writeBin(cams, static_cast<uint64_t>(model.cameras.size()));
        for (const auto& [id, cam] : model.cameras)
        {
            const int32_t model_id = cameraModelId(cam.model);
            if (model_id < 0)
            {
                error = "unknown camera model '" + cam.model + "'";
                return false;
            }
            writeBin(cams, cam.id);
            writeBin(cams, model_id);
            writeBin(cams, cam.width);
            writeBin(cams, cam.height);
            for (double p : cam.params)
                writeBin(cams, p);
        }

        writeBin(imgs, static_cast<uint64_t>(model.images.size()));
        for (const auto& [id, img] : model.images)
        {
            writeBin(imgs, img.id);
            for (double v : { img.q_cw.w(), img.q_cw.x(), img.q_cw.y(), img.q_cw.z(), img.t_cw.x(), img.t_cw.y(), img.t_cw.z() })
                writeBin(imgs, v);
            writeBin(imgs, img.camera_id);
            imgs.write(img.name.c_str(), static_cast<std::streamsize>(img.name.size() + 1));
            writeBin(imgs, static_cast<uint64_t>(img.points2D.size()));
            for (const auto& obs : img.points2D)
            {
                writeBin(imgs, obs.xy.x());
                writeBin(imgs, obs.xy.y());
                writeBin(imgs, obs.point3D_id);
            }
        }

        writeBin(pts, static_cast<uint64_t>(model.points.size()));
        for (const auto& [id, pt] : model.points)
        {
            writeBin(pts, pt.id);
            writeBin(pts, pt.xyz.x());
            writeBin(pts, pt.xyz.y());
            writeBin(pts, pt.xyz.z());
            for (uint8_t c : pt.rgb)
                writeBin(pts, c);
            writeBin(pts, pt.error);
            writeBin(pts, static_cast<uint64_t>(pt.track.size()));
            for (const auto& el : pt.track)
            {
                writeBin(pts, el.image_id);
                writeBin(pts, el.point2D_idx);
            }
        }

        if (!cams || !imgs || !pts)
        {
            error = "write error in " + dir;
            return false;
        }
        return true;
    }

    std::vector<uint32_t> imagesSortedByName(const Model& model)
    {
        std::vector<uint32_t> ids;
        ids.reserve(model.images.size());
        for (const auto& [id, img] : model.images)
            ids.push_back(id);
        // File name first: a rig's front/<t>.jpg and back/<t>.jpg sit next to each other, not one folder after the other.
        auto key = [&model](uint32_t id)
        {
            const std::string& name = model.images.at(id).name;
            return std::make_pair(fs::path(name).filename().string(), name);
        };
        std::sort(
            ids.begin(),
            ids.end(),
            [&key](uint32_t a, uint32_t b)
            {
                return key(a) < key(b);
            });
        return ids;
    }

    int64_t timestampNsFromImageName(const std::string& name)
    {
        std::string stem = fs::path(name).stem().string();
        if (const auto us = stem.rfind('_'); us != std::string::npos)
            stem = stem.substr(us + 1);
        // Under 10 digits it is a frame counter (frame_000123.jpg), not nanoseconds.
        if (stem.size() < 10 || stem.find_first_not_of("0123456789") != std::string::npos)
            return -1;
        try
        {
            return std::stoll(stem);
        } catch (...)
        {
            return -1;
        }
    }
} // namespace colmap_sim3
