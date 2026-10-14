#pragma once

// COLMAP sparse model (cameras / images / points3D) in memory, with readers for COLMAP's .bin and
// .txt layouts and writers for both. Poses are kept the way COLMAP stores them: world -> camera.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace colmap_sim3
{
    //! Marks a 2D observation with no 3D point (COLMAP writes it as -1 in text).
    constexpr uint64_t kInvalidPoint3DId = std::numeric_limits<uint64_t>::max();

    struct Camera
    {
        uint32_t id = 0;
        std::string model; //!< COLMAP model name, e.g. "PINHOLE"
        uint64_t width = 0;
        uint64_t height = 0;
        std::vector<double> params;
    };

    struct Observation2D
    {
        Eigen::Vector2d xy = Eigen::Vector2d::Zero();
        uint64_t point3D_id = kInvalidPoint3DId;
    };

    struct Image
    {
        uint32_t id = 0;
        Eigen::Quaterniond q_cw = Eigen::Quaterniond::Identity(); //!< world -> camera rotation
        Eigen::Vector3d t_cw = Eigen::Vector3d::Zero(); //!< world -> camera translation
        uint32_t camera_id = 0;
        std::string name;
        std::vector<Observation2D> points2D;

        //! @return The camera -> world pose (camera centre in the translation).
        Eigen::Affine3d worldFromCamera() const;
        //! Sets q_cw / t_cw from a camera -> world pose; the rotation part must be orthonormal.
        void setWorldFromCamera(const Eigen::Affine3d& T_wc);
    };

    struct TrackElement
    {
        uint32_t image_id = 0;
        uint32_t point2D_idx = 0;
    };

    struct Point3D
    {
        uint64_t id = 0;
        Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
        std::array<uint8_t, 3> rgb{};
        double error = -1.0;
        std::vector<TrackElement> track;
    };

    struct Model
    {
        std::map<uint32_t, Camera> cameras;
        std::map<uint32_t, Image> images;
        std::map<uint64_t, Point3D> points;
    };

    //! @return COLMAP's numeric id of a camera model name, -1 when unknown.
    int cameraModelId(const std::string& model);
    //! @return The camera model name for a COLMAP numeric id, "" when unknown.
    std::string cameraModelName(int id);
    //! @return The number of intrinsic parameters of a camera model, -1 when unknown.
    int cameraModelNumParams(const std::string& model);

    //! Reads a model from @p dir, preferring cameras/images/points3D.bin and falling back to .txt.
    //! @param dir directory holding the three files (e.g. sparse/0)
    //! @param model filled on success
    //! @param error why it failed
    //! @return false on a missing or malformed file
    bool readModel(const std::string& dir, Model& model, std::string& error);
    bool readModelText(const std::string& dir, Model& model, std::string& error);
    bool readModelBinary(const std::string& dir, Model& model, std::string& error);

    //! Writes cameras.txt, images.txt and points3D.txt into @p dir, which must exist.
    bool writeModelText(const std::string& dir, const Model& model, std::string& error);
    //! Writes cameras.bin, images.bin and points3D.bin into @p dir, which must exist.
    bool writeModelBinary(const std::string& dir, const Model& model, std::string& error);

    //! @return Image ids ordered by file name, then by full name: the capture order for frame-numbered or
    //!         time-stamped names, with the images of one rig frame (front/<t>.jpg, back/<t>.jpg) adjacent.
    std::vector<uint32_t> imagesSortedByName(const Model& model);

    //! Parses a capture time from an image name, "<prefix>_<ns>.<ext>" or "<ns>.<ext>".
    //! @return The timestamp in nanoseconds, -1 when the name does not match or the number has
    //!         fewer than 10 digits (a frame counter rather than a time).
    int64_t timestampNsFromImageName(const std::string& name);
} // namespace colmap_sim3
