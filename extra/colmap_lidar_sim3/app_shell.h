#pragma once

// Window, camera and input plumbing, trimmed from step 3's raylib_utils (apps/multi_session_registration):
// no sessions, picking or labels, just what a single-view registration tool needs.

#include "raylib.h"
#include "rlgl.h"

#include <imgui.h>

#include <Eigen/Eigen>

#include <RaylibWidgets/AppShell.h>
#include <RaylibWidgets/OrbitCamera.h>
#include <RaylibWidgets/ShortcutsTable.h>

#include <string>
#include <vector>

using raylib_widgets::ShortcutEntry;
using raylib_widgets::ShowMainDockSpace;

// GLUT mouse codes, so mouse() keeps step 3's GLUT-callback shape.
constexpr int GLUT_LEFT_BUTTON = 0;
constexpr int GLUT_MIDDLE_BUTTON = 1;
constexpr int GLUT_RIGHT_BUTTON = 2;
constexpr int GLUT_DOWN = 0;
constexpr int GLUT_UP = 1;

extern raylib_widgets::OrbitCamera camera;
extern int mouse_old_x, mouse_old_y;
extern int mouse_buttons;
extern bool show_axes;
extern ImVec4 bg_color;
extern int point_size;
extern bool info_gui;
extern bool compass_ruler;
extern bool cor_gui;
extern Eigen::Affine3f viewLocal;

bool initGL(const std::string& winTitle, void (*display)(), void (*mouse)(int, int, int, int));
//! Polls raylib input into the mouse()/motion()/wheel() callbacks and calls display() per frame.
void mainLoop();
//! Runs @p releaseGpu (GPU buffers must go while the GL context exists), then closes the window.
void shutdownGL(void (*releaseGpu)());

//! Sets the projection and view for this frame (perspective or ortho) and captures the matrices.
void beginScene3D();
void showAxes();
void camMenu();
void view_kbd_shortcuts();
void cor_window();
void info_window(const std::vector<std::string>& infoLines, const std::vector<ShortcutEntry>& appShortcuts);
void endScene3D();
void drawMiniCompassWithRuler();

//! Moves the rotation center to where the mouse ray hits the plane z = @p z.
void setNewRotationCenter(int x, int y, double z);
void setNewRotationCenter(const Eigen::Vector3d& center);
bool checkClHelp(int argc, char** argv);

//! Copies the current rlgl projection/modelview (column-major, for ImGuizmo).
void getProjectionMatrix(float m[16]);
void getModelviewMatrix(float m[16]);
