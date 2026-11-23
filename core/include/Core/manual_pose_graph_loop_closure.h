#pragma once

#include <Core/gnss.h>
#include <Core/point_clouds.h>
#include <Core/pose_graph_loop_closure.h>
#include <Core/tum.h>

#include <chrono>

#if WITH_GUI == 1
#include <Core/observation_picking.h>
#endif

// Defined in every build (not only WITH_GUI) so Session has one layout; only the GUI methods are conditional.
class ManualPoseGraphLoopClosure : public PoseGraphLoopClosure
{
public:
    int index_active_edge = 0;
    bool manipulate_active_edge = false;
    bool gizmo = false;
    //! N / P step the active edge in Gui(); the app turns this off while another list takes these keys.
    bool edge_hotkeys = true;
    double search_radious = 0.1;
    //! "ICP all edges": first search radius of its coarse-to-fine steps, see PoseGraphLoopClosure::icp_search_radii.
    double icp_all_edges_start_radius = 1.0;
    //! Edges the last "ICP all edges" could not refine.
    std::vector<int> icp_failed_edges;
    //! Number of edges when "ICP all edges" last finished; 0 when it hasn't, or the edges changed since.
    size_t icp_all_edges_count = 0;
    //! "ICP all edges" in progress: it runs one ICP step per frame, so the window keeps drawing its progress.
    struct IcpAllEdgesRun
    {
        bool running = false;
        int edge = 0; //!< edge being refined
        size_t step = 0; //!< index into radii
        std::vector<float> radii;
        std::vector<int> failed;
        std::chrono::steady_clock::time_point start;
        bool progress_drawn = false; //!< the progress bar has been shown at least once
    };
    IcpAllEdgesRun icp_all_edges_run;
    //! Last "Calculate ICP uncertainty" result: the edge, relative pose and start offsets it was computed for, so it is
    //! shown only while they are unchanged.
    struct IcpUncertainty
    {
        int edge = -1;
        TaitBryanPose relative_pose;
        //! Start offsets (Q_ini) it was computed with.
        double start_offset_m = std::numeric_limits<double>::quiet_NaN();
        double start_offset_deg = std::numeric_limits<double>::quiet_NaN();
        TaitBryanPose point_to_point;
        TaitBryanPose point_to_plane;
        IcpResidualStatistics point_to_point_residuals;
        IcpResidualStatistics point_to_plane_residuals;
    };
    IcpUncertainty icp_uncertainty;
    //! "Calculate ICP spread": its settings and last result, shown only while the edge and the relative pose it was
    //! computed for are unchanged.
    struct IcpSpreadGui
    {
        int number_of_runs = 20;
        double start_offset_m = 0.05;
        double start_offset_deg = 0.5;
        //! Size of the drawn translation ellipsoids (spread and point-to-plane uncertainty), in standard deviations.
        double ellipsoid_sigma = 10.0;
        int edge = -1;
        TaitBryanPose relative_pose;
        IcpSpread result;
    };
    IcpSpreadGui icp_spread;
    //! True when edge is the active edge and its relative pose still equals relative_pose.
    bool is_active_edge_at(int edge, const TaitBryanPose& relative_pose) const
    {
        if (edge != index_active_edge || edge < 0 || edge >= static_cast<int>(edges.size()))
        {
            return false;
        }
        const TaitBryanPose& a = edges[edge].relative_pose_tb;
        const TaitBryanPose& b = relative_pose;
        return a.px == b.px && a.py == b.py && a.pz == b.pz && a.om == b.om && a.fi == b.fi && a.ka == b.ka;
    }
    //! True when icp_uncertainty was computed for the active edge at its current relative pose and start offsets.
    bool icp_uncertainty_is_current() const
    {
        return is_active_edge_at(icp_uncertainty.edge, icp_uncertainty.relative_pose) &&
            icp_uncertainty.start_offset_m == icp_spread.start_offset_m && icp_uncertainty.start_offset_deg == icp_spread.start_offset_deg;
    }
    //! True when icp_spread.result was computed for the active edge at its current relative pose.
    bool icp_spread_is_current() const
    {
        return is_active_edge_at(icp_spread.edge, icp_spread.relative_pose);
    }
    //! "Keep initial trajectory curvature": compute_pose_graph_slam() uses the initial poses as the motion model, else the current ones.
    bool keep_initial_trajectory_curvature = true;

    ManualPoseGraphLoopClosure() = default;
    ~ManualPoseGraphLoopClosure() = default;

#if WITH_GUI == 1
    //! "Compute Pose Graph SLAM": sets the motion model, then graph_slam() over the edges, GNSS, GCPs and CPs.
    void compute_pose_graph_slam(PointClouds& point_clouds_container, GNSS& gnss, GroundControlPoints& gcps, ControlPoints& cps);
    void Gui(
        PointClouds& point_clouds_container,
        int& index_loop_closure_source,
        int& index_loop_closure_target,
        float* m_gizmo,
        GNSS& gnss,
        TUM& tum,
        GroundControlPoints& gcps,
        ControlPoints& cps,
        int num_edge_extended_before,
        int num_edge_extended_after);
    void Render(
        PointClouds& point_clouds_container,
        int index_loop_closure_source,
        int index_loop_closure_target,
        int num_edge_extended_before,
        int num_edge_extended_after);

private:
    //! Runs the next "ICP all edges" step; finishes the run after the last one.
    void icp_all_edges_step(PointClouds& point_clouds_container, int num_edge_extended_before, int num_edge_extended_after);
    //! The "ICP all edges" button, start radius and progress.
    void icp_all_edges_gui();
#endif
};
