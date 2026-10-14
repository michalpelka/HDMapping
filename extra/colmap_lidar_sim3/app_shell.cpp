#include "app_shell.h"

#include "raymath.h"
#include "rlImGui.h"

#include <RaylibWidgets/CenterOfRotationWindow.h>
#include <RaylibWidgets/CompassRuler.h>
#include <RaylibWidgets/RayPlaneD.h>
#include <RaylibWidgets/WindowFit.h>

#include <HDMapping/HDMAPPING_ConfigureInfo.hpp>

#include <algorithm>
#include <iostream>

raylib_widgets::OrbitCamera camera;

int mouse_old_x = 0, mouse_old_y = 0;
int mouse_buttons = 0;
bool show_axes = true;
ImVec4 bg_color = ImVec4(0.65f, 0.65f, 0.65f, 1.00f);
int point_size = 1;
bool info_gui = false;
bool compass_ruler = true;
bool cor_gui = false;
Eigen::Affine3f viewLocal = Eigen::Affine3f::Identity();

namespace
{
    constexpr float kDegToRad = M_PI / 180.0f;

    void (*display_cb)() = nullptr;
    void (*mouse_cb)(int, int, int, int) = nullptr;

    void copyMatrix(const Matrix& m, float out[16])
    {
        const float v[16] = { m.m0, m.m1, m.m2, m.m3, m.m4, m.m5, m.m6, m.m7, m.m8, m.m9, m.m10, m.m11, m.m12, m.m13, m.m14, m.m15 };
        std::copy(v, v + 16, out);
    }

    void wheel()
    {
        if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
            camera.zoom(GetMouseWheelMove(), ImGui::GetIO().KeyShift);
    }

    void motion(int x, int y)
    {
        ImGuiIO& io = ImGui::GetIO();
        if (io.WantCaptureMouse)
            return;

        const float dx = static_cast<float>(x - mouse_old_x);
        const float dy = static_cast<float>(y - mouse_old_y);

        // Ctrl/Shift+click picks a rotation center; don't let a stray drag break that transition.
        if (!io.KeyCtrl && !io.KeyShift)
        {
            if (mouse_buttons & 1)
                camera.dragOrbit(dx, dy);
            if (mouse_buttons & 4)
            {
                if (camera.isOrtho)
                    camera.dragPanOrtho(dx, dy, io.DisplaySize.x, io.DisplaySize.y);
                else
                    camera.dragPanPerspective(dx, dy);
            }
        }

        mouse_old_x = x;
        mouse_old_y = y;
    }

    Vector3 toVec3(const Eigen::Vector3d& v)
    {
        return Vector3{ static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()) };
    }
} // namespace

bool initGL(const std::string& winTitle, void (*display)(), void (*mouse)(int, int, int, int))
{
    display_cb = display;
    mouse_cb = mouse;

    // HiDPI breaks ImGui scaling on Windows, so only macOS/Linux enable it (as in step 2 and 3).
    unsigned int flags = FLAG_WINDOW_RESIZABLE;
#if defined(__APPLE__) || defined(__linux__)
    flags |= FLAG_WINDOW_HIGHDPI;
#endif
    SetConfigFlags(flags);
    InitWindow(1600, 900, winTitle.c_str());
    SetExitKey(KEY_NULL); // Esc must not close the window
    SetTargetFPS(60);
    raylib_widgets::fitWindowToScreen(100, 100, true);

    rlImGuiSetup(true);
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;
    io.ConfigDockingWithShift = true;
    return true;
}

void mainLoop()
{
    const int buttons[][2] = { { MOUSE_BUTTON_LEFT, GLUT_LEFT_BUTTON },
                               { MOUSE_BUTTON_MIDDLE, GLUT_MIDDLE_BUTTON },
                               { MOUSE_BUTTON_RIGHT, GLUT_RIGHT_BUTTON } };
    while (!WindowShouldClose())
    {
        const int x = GetMouseX();
        const int y = GetMouseY();
        for (const auto& b : buttons)
        {
            if (IsMouseButtonPressed(b[0]))
                mouse_cb(b[1], GLUT_DOWN, x, y);
            if (IsMouseButtonReleased(b[0]))
                mouse_cb(b[1], GLUT_UP, x, y);
        }
        motion(x, y);
        if (GetMouseWheelMove() != 0.0f)
            wheel();

        BeginDrawing();
        display_cb();
        EndDrawing();
    }
}

void shutdownGL(void (*releaseGpu)())
{
    if (releaseGpu)
        releaseGpu();
    rlImGuiShutdown();
    CloseWindow();
}

void beginScene3D()
{
    const ImGuiIO& io = ImGui::GetIO();
    rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
    ClearBackground(ColorFromNormalized(Vector4{ bg_color.x * bg_color.w, bg_color.y * bg_color.w, bg_color.z * bg_color.w, bg_color.w }));
    rlEnableDepthTest();

    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    camera.updateEulerTransition(GetFrameTime());

    viewLocal = Eigen::Affine3f::Identity();
    if (!camera.isOrtho)
    {
        camera.applyPerspectiveProjection(static_cast<int>(io.DisplaySize.x), static_cast<int>(io.DisplaySize.y));

        const Eigen::Map<const Eigen::Vector3f> rotation_center(&camera.euler.rotationCenter.x);
        viewLocal.translate(rotation_center);
        viewLocal.translate(Eigen::Vector3f(camera.euler.translate.x, camera.euler.translate.y, camera.euler.translate.z));
        if (!camera.lockZ)
            viewLocal.rotate(Eigen::AngleAxisf(camera.euler.rotateX * kDegToRad, Eigen::Vector3f::UnitX()));
        else
            viewLocal.rotate(Eigen::AngleAxisf(-90.0f * kDegToRad, Eigen::Vector3f::UnitX()));
        viewLocal.rotate(Eigen::AngleAxisf(camera.euler.rotateY * kDegToRad, Eigen::Vector3f::UnitZ()));
        viewLocal.translate(-rotation_center);

        rlMultMatrixf(viewLocal.matrix().data());
    }
    else
    {
        viewLocal.rotate(Eigen::AngleAxisf((camera.euler.rotateX + camera.euler.rotateY) * kDegToRad, Eigen::Vector3f::UnitZ()));
        camera.updateOrtho(io.DisplaySize.x / io.DisplaySize.y);
    }
    camera.captureFrameMatrices();
}

void showAxes()
{
    if (!show_axes && !ImGui::GetIO().KeyCtrl)
        return;

    const Vector3 rc = camera.euler.rotationCenter;
    rlBegin(RL_LINES);
    rlColor3f(1.f, 1.f, 1.f);
    for (int axis = 0; axis < 3; axis++)
    {
        const Vector3 d{ axis == 0 ? 1.f : 0.f, axis == 1 ? 1.f : 0.f, axis == 2 ? 1.f : 0.f };
        rlVertex3f(rc.x - d.x, rc.y - d.y, rc.z - d.z);
        rlVertex3f(rc.x + d.x, rc.y + d.y, rc.z + d.z);
    }
    rlColor3f(1.f, 0.f, 0.f);
    rlVertex3f(0, 0, 0);
    rlVertex3f(100, 0, 0);
    rlColor3f(0.f, 1.f, 0.f);
    rlVertex3f(0, 0, 0);
    rlVertex3f(0, 100, 0);
    rlColor3f(0.f, 0.f, 1.f);
    rlVertex3f(0, 0, 0);
    rlVertex3f(0, 0, 100);
    rlEnd();
}

void camMenu()
{
    using P = raylib_widgets::OrbitCamera::EulerPreset;
    if (ImGui::BeginMenu("Camera"))
    {
        const std::pair<const char*, P> items[] = { { "Front (yz view)", P::Front }, { "Back", P::Back },
                                                    { "Left (xz view)", P::Left },   { "Right", P::Right },
                                                    { "Top (xy view)", P::Top },     { "Bottom", P::Bottom },
                                                    { "Isometric", P::Iso } };
        const char* keys[] = { "key F", "key B", "key L", "key R", "key T", "key U", "key I" };
        for (size_t i = 0; i < 7; i++)
            if (ImGui::MenuItem(items[i].first, keys[i]))
                camera.setEulerPreset(items[i].second);
        ImGui::Separator();
        if (ImGui::MenuItem("Reset", "key Z"))
            camera.setEulerPreset(P::Reset);
        ImGui::EndMenu();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Change camera view to fixed positions");
}

void view_kbd_shortcuts()
{
    using P = raylib_widgets::OrbitCamera::EulerPreset;
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard)
        return;

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_R, false))
        cor_gui = true;
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false) && !camera.isOrtho)
        camera.lockZ = !camera.lockZ;

    if (io.KeyCtrl || io.KeyAlt || io.KeyShift)
        return;

    const std::pair<ImGuiKey, P> presets[] = { { ImGuiKey_B, P::Back },  { ImGuiKey_F, P::Front }, { ImGuiKey_I, P::Iso },
                                               { ImGuiKey_L, P::Left },  { ImGuiKey_R, P::Right }, { ImGuiKey_T, P::Top },
                                               { ImGuiKey_U, P::Bottom } };
    for (const auto& [key, preset] : presets)
        if (ImGui::IsKeyPressed(key))
            camera.setEulerPreset(preset);
    if (ImGui::IsKeyPressed(ImGuiKey_Z))
        camera.setEulerPreset(P::Reset);

    if (ImGui::IsKeyPressed(ImGuiKey_C, false))
        compass_ruler = !compass_ruler;
    if (ImGui::IsKeyPressed(ImGuiKey_O, false))
        camera.isOrtho = !camera.isOrtho;
    if (ImGui::IsKeyPressed(ImGuiKey_X, false))
        show_axes = !show_axes;
    for (int k = 1; k <= 9; k++)
        if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_0 + k)))
            point_size = k;
}

void cor_window()
{
    raylib_widgets::showEulerCenterOfRotationWindow(
        cor_gui, camera, "Longitudinal (forward/backward)", "Lateral (left/right)", "Vertical (up/down)");
}

void info_window(const std::vector<std::string>& infoLines, const std::vector<ShortcutEntry>& appShortcuts)
{
    raylib_widgets::ShowInfoWindow(info_gui, infoLines, appShortcuts, HDMAPPING_CONFIGURE_PROJECT_VERSION, __DATE__);
}

void endScene3D()
{
    const ImGuiIO& io = ImGui::GetIO();
    raylib_widgets::end3DMatrixStack(io.DisplaySize.x, io.DisplaySize.y);
}

void drawMiniCompassWithRuler()
{
    // Rows 0/1 of the world-to-eye rotation are the world directions of screen right/up.
    const Eigen::Matrix3f& R = viewLocal.rotation();
    const Color rulerColor = ColorFromNormalized(Vector4{ 1.0f - bg_color.x, 1.0f - bg_color.y, 1.0f - bg_color.z, 1.0f });
    raylib_widgets::drawCompassRuler(
        Vector3{ R(0, 0), R(0, 1), R(0, 2) },
        Vector3{ R(1, 0), R(1, 1), R(1, 2) },
        camera.euler.translate.z,
        rulerColor,
        raylib_widgets::CompassAxisLabels{ "X", "Y", "Z" });
}

void setNewRotationCenter(int x, int y, double z)
{
    const Ray ray = camera.eulerScreenRay(x, y, GetScreenWidth(), GetScreenHeight());
    Eigen::Vector3d hit(ray.position.x, ray.position.y, ray.position.z);
    raylib_widgets::intersectPlane(
        Eigen::Vector3d(ray.position.x, ray.position.y, ray.position.z),
        Eigen::Vector3d(ray.direction.x, ray.direction.y, ray.direction.z),
        0,
        0,
        1,
        -z,
        hit);
    setNewRotationCenter(hit);
}

void setNewRotationCenter(const Eigen::Vector3d& center)
{
    camera.moveEulerRotationCenterTo(toVec3(center));
}

bool checkClHelp(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "-h" || arg == "/h" || arg == "--help" || arg == "/?")
            return true;
    }
    return false;
}

void getProjectionMatrix(float m[16])
{
    copyMatrix(rlGetMatrixProjection(), m);
}

void getModelviewMatrix(float m[16])
{
    copyMatrix(rlGetMatrixModelview(), m);
}
