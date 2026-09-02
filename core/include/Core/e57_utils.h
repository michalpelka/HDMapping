#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

// NOTE: namespace is `e57io`, not `e57`, to avoid colliding with libE57Format's
// own top-level `::e57` namespace inside the implementation.
namespace mandeye::e57io
{
    // One E57 `Data3D` block (a single scan) decoded into HDMapping-friendly buffers.
    // Points are kept in the scan-local coordinate frame; `pose` places that frame in
    // the file-level frame (from the Data3D `pose` element, identity when absent).
    struct E57Scan
    {
        std::string name;
        std::vector<Eigen::Vector3d> points;      // cartesian, scan-local, meters
        std::vector<unsigned short> intensities;  // empty when the scan has no intensity
        std::vector<Eigen::Vector3d> colors;      // 0..1 RGB, empty when the scan has no color
        std::vector<double> timestamps;           // seconds, empty when the scan has no timestamps
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    };

    // Reads every Data3D block of `path`. Returns false and fills `error` on failure
    // (file missing, not an E57, corrupt, unreadable). Scans with zero valid points are
    // skipped. Spherical-only scans are converted to cartesian.
    bool load_e57(const std::string& path, std::vector<E57Scan>& scans, std::string& error);

    // Copies `src_path` to `dst_path` verbatim (all scans, images, point fields
    // and attributes preserved) except that the `pose` of each Data3D block whose
    // index appears in `new_poses` is replaced with the given file-level transform.
    // `src_path` and `dst_path` must differ. Returns false and fills `error` on
    // failure, leaving `dst_path` in an unspecified state (callers write to a
    // temp file and only swap it in on success).
    bool rewrite_e57_poses(
        const std::string& src_path,
        const std::string& dst_path,
        const std::map<int, Eigen::Affine3d>& new_poses,
        std::string& error);
} // namespace mandeye::e57io
