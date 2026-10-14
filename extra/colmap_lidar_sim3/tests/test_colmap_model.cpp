#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include "colmap_model.h"

#include <filesystem>
#include <random>

namespace fs = std::filesystem;
using namespace colmap_sim3;

namespace
{
    const char* kAllModels[] = { "SIMPLE_PINHOLE",
                                 "PINHOLE",
                                 "SIMPLE_RADIAL",
                                 "RADIAL",
                                 "OPENCV",
                                 "OPENCV_FISHEYE",
                                 "FULL_OPENCV",
                                 "FOV",
                                 "SIMPLE_RADIAL_FISHEYE",
                                 "RADIAL_FISHEYE",
                                 "THIN_PRISM_FISHEYE",
                                 "RAD_TAN_THIN_PRISM_FISHEYE" };

    Model makeModel()
    {
        Model m;
        uint32_t id = 1;
        for (const char* name : kAllModels)
        {
            Camera cam;
            cam.id = id++;
            cam.model = name;
            cam.width = 1920;
            cam.height = 1080;
            for (int k = 0; k < cameraModelNumParams(name); k++)
                cam.params.push_back(100.0 + k + 0.123456789012345);
            m.cameras[cam.id] = cam;
        }

        Image a;
        a.id = 3;
        a.q_cw = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized()));
        a.t_cw = Eigen::Vector3d(1.5, -2.25, 1e5 + 0.125);
        a.camera_id = 1;
        a.name = "cam0/frame_1700000000123456789.jpg";
        a.points2D = { { Eigen::Vector2d(10.5, 20.25), 7 }, { Eigen::Vector2d(1, 2), kInvalidPoint3DId } };
        m.images[a.id] = a;

        Image b;
        b.id = 5;
        b.camera_id = 2;
        b.name = "frame_000002.jpg"; // no observations: empty POINTS2D line
        m.images[b.id] = b;

        Point3D p;
        p.id = 7;
        p.xyz = Eigen::Vector3d(1.0 / 3.0, -2.0, 123456.789);
        p.rgb = { 255, 0, 17 };
        p.error = 0.75;
        p.track = { { 3, 0 }, { 5, 9 } };
        m.points[p.id] = p;
        return m;
    }

    void checkEqual(const Model& a, const Model& b)
    {
        REQUIRE(a.cameras.size() == b.cameras.size());
        for (const auto& [id, ca] : a.cameras)
        {
            const Camera& cb = b.cameras.at(id);
            CHECK(ca.model == cb.model);
            CHECK(ca.width == cb.width);
            CHECK(ca.height == cb.height);
            REQUIRE(ca.params.size() == cb.params.size());
            for (size_t k = 0; k < ca.params.size(); k++)
                CHECK(ca.params[k] == cb.params[k]);
        }

        REQUIRE(a.images.size() == b.images.size());
        for (const auto& [id, ia] : a.images)
        {
            const Image& ib = b.images.at(id);
            CHECK(ia.name == ib.name);
            CHECK(ia.camera_id == ib.camera_id);
            CHECK(ia.q_cw.angularDistance(ib.q_cw) < 1e-12);
            CHECK((ia.t_cw - ib.t_cw).norm() < 1e-9);
            REQUIRE(ia.points2D.size() == ib.points2D.size());
            for (size_t k = 0; k < ia.points2D.size(); k++)
            {
                CHECK(ia.points2D[k].xy == ib.points2D[k].xy);
                CHECK(ia.points2D[k].point3D_id == ib.points2D[k].point3D_id);
            }
        }

        REQUIRE(a.points.size() == b.points.size());
        for (const auto& [id, pa] : a.points)
        {
            const Point3D& pb = b.points.at(id);
            CHECK(pa.xyz == pb.xyz);
            CHECK(pa.rgb == pb.rgb);
            CHECK(pa.error == pb.error);
            REQUIRE(pa.track.size() == pb.track.size());
            for (size_t k = 0; k < pa.track.size(); k++)
            {
                CHECK(pa.track[k].image_id == pb.track[k].image_id);
                CHECK(pa.track[k].point2D_idx == pb.track[k].point2D_idx);
            }
        }
    }

    fs::path freshDir(const std::string& name)
    {
        const fs::path dir = fs::temp_directory_path() / ("colmap_lidar_sim3_" + name + "_" + std::to_string(std::random_device{}()));
        fs::create_directories(dir);
        return dir;
    }
} // namespace

TEST_CASE("camera model table agrees in both directions")
{
    for (int id = 0; id < 12; id++)
    {
        const std::string name = cameraModelName(id);
        REQUIRE(!name.empty());
        CHECK(cameraModelId(name) == id);
        CHECK(cameraModelNumParams(name) > 0);
    }
    CHECK(cameraModelName(12).empty());
    CHECK(cameraModelId("NOT_A_MODEL") == -1);
    CHECK(cameraModelNumParams("NOT_A_MODEL") == -1);
}

TEST_CASE("text model round-trips")
{
    const fs::path dir = freshDir("txt");
    const Model m = makeModel();
    std::string error;
    REQUIRE_MESSAGE(writeModelText(dir.string(), m, error), error);
    Model read;
    REQUIRE_MESSAGE(readModel(dir.string(), read, error), error);
    checkEqual(m, read);
    fs::remove_all(dir);
}

TEST_CASE("binary model round-trips and wins over text")
{
    const fs::path dir = freshDir("bin");
    const Model m = makeModel();
    std::string error;
    REQUIRE_MESSAGE(writeModelBinary(dir.string(), m, error), error);

    Model other;
    other.cameras[1] = m.cameras.at(1);
    REQUIRE(writeModelText(dir.string(), other, error));

    Model read;
    REQUIRE_MESSAGE(readModel(dir.string(), read, error), error);
    checkEqual(m, read);
    fs::remove_all(dir);
}

TEST_CASE("missing model reports an error")
{
    const fs::path dir = freshDir("empty");
    Model read;
    std::string error;
    CHECK_FALSE(readModel(dir.string(), read, error));
    CHECK(!error.empty());
    fs::remove_all(dir);
}

TEST_CASE("image pose conversion")
{
    Image img;
    Eigen::Affine3d T_wc = Eigen::Affine3d::Identity();
    T_wc.linear() = Eigen::AngleAxisd(1.1, Eigen::Vector3d(0.2, -1, 0.4).normalized()).toRotationMatrix();
    T_wc.translation() = Eigen::Vector3d(3, -4, 5);
    img.setWorldFromCamera(T_wc);
    CHECK((img.worldFromCamera().matrix() - T_wc.matrix()).norm() < 1e-12);
    // Camera centre is -R^T t in COLMAP's convention.
    CHECK((-(img.q_cw.conjugate() * img.t_cw) - T_wc.translation()).norm() < 1e-12);
}

TEST_CASE("images sort by name and timestamps parse")
{
    Model m;
    for (uint32_t id : { 1u, 2u, 3u })
    {
        Image img;
        img.id = id;
        img.name = id == 1 ? "c.jpg" : (id == 2 ? "a.jpg" : "b.jpg");
        m.images[id] = img;
    }
    CHECK(imagesSortedByName(m) == std::vector<uint32_t>{ 2, 3, 1 });

    Model rig;
    const char* names[] = { "front/200.jpg", "back/100.jpg", "back/200.jpg", "front/100.jpg" };
    for (uint32_t id = 0; id < 4; id++)
    {
        Image img;
        img.id = id;
        img.name = names[id];
        rig.images[id] = img;
    }
    CHECK(imagesSortedByName(rig) == std::vector<uint32_t>{ 1, 3, 2, 0 });

    CHECK(timestampNsFromImageName("dir/frame_1700000000123456789.jpg") == 1700000000123456789LL);
    CHECK(timestampNsFromImageName("1700000000123456789.png") == 1700000000123456789LL);
    CHECK(timestampNsFromImageName("frame_000123.jpg") == -1);
    CHECK(timestampNsFromImageName("frame_abc.jpg") == -1);
    CHECK(timestampNsFromImageName("99999999999999999999999.jpg") == -1);
}
