#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <chrono>
#include <vector>
#include <random>
#include <iostream>

namespace py = pybind11;

class TracIKSolverWrapper {
public:
    std::unique_ptr<TRAC_IK::TRAC_IK> solver;
    KDL::Chain chain;
    KDL::JntArray q_min;
    KDL::JntArray q_max;
    int num_joints;
    double maxtime;
    double eps;

    TracIKSolverWrapper(double maxtime_sec = 0.005, double tolerance = 1e-4)
        : maxtime(maxtime_sec), eps(tolerance) {
        // Construct KDL::Chain matching ARIA 5-DOF DH parameters:
        // Base height offset 0.105m
        chain.addSegment(KDL::Segment("base_seg", KDL::Joint("base_fixed", KDL::Joint::Fixed), KDL::Frame(KDL::Vector(0, 0, 0.105))));
        // Joint 1: Waist (RotZ, alpha=pi/2)
        chain.addSegment(KDL::Segment("waist_seg", KDL::Joint("waist", KDL::Joint::RotZ), KDL::Frame(KDL::Rotation::RotX(M_PI/2), KDL::Vector(0, 0, 0))));
        // Joint 2: Shoulder (RotZ, a=0.145m)
        chain.addSegment(KDL::Segment("shoulder_seg", KDL::Joint("shoulder", KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.145, 0, 0))));
        // Joint 3: Elbow (RotZ, a=0.115m)
        chain.addSegment(KDL::Segment("elbow_seg", KDL::Joint("elbow", KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.115, 0, 0))));
        // Joint 4: Wrist pitch (RotZ, a=0.095m)
        chain.addSegment(KDL::Segment("wrist_pitch_seg", KDL::Joint("wrist_pitch", KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.095, 0, 0))));
        // Joint 5: Wrist roll (RotZ matching DH table)
        chain.addSegment(KDL::Segment("wrist_roll_seg", KDL::Joint("wrist_roll", KDL::Joint::RotZ), KDL::Frame::Identity()));

        num_joints = 5;
        q_min.resize(num_joints);
        q_max.resize(num_joints);

        double limits[5][2] = {
            {-3.1416, 3.1416},
            {-1.5708, 1.5708},
            {-1.5708, 1.5708},
            {-1.5708, 1.5708},
            {-1.5708, 1.5708}
        };

        for (int i = 0; i < num_joints; ++i) {
            q_min(i) = limits[i][0];
            q_max(i) = limits[i][1];
        }

        solver = std::make_unique<TRAC_IK::TRAC_IK>(chain, q_min, q_max, maxtime, eps, TRAC_IK::Speed);
    }

    py::tuple solve(const std::vector<double>& target_pos,
                    const std::vector<double>& target_rot_mat,
                    const std::vector<double>& q_init,
                    int max_restarts = 1) {
        auto t0 = std::chrono::high_resolution_clock::now();

        KDL::Vector p(target_pos[0], target_pos[1], target_pos[2]);
        KDL::Rotation R(
            target_rot_mat[0], target_rot_mat[1], target_rot_mat[2],
            target_rot_mat[3], target_rot_mat[4], target_rot_mat[5],
            target_rot_mat[6], target_rot_mat[7], target_rot_mat[8]
        );
        KDL::Frame target_frame(R, p);

        KDL::JntArray q_in(num_joints);
        for (int i = 0; i < num_joints; ++i) {
            q_in(i) = (i < (int)q_init.size()) ? q_init[i] : 0.0;
        }

        KDL::JntArray q_out(num_joints);
        KDL::Twist bounds = KDL::Twist::Zero();

        int restarts_used = 0;
        bool success = false;
        int rc = -1;

        std::mt19937 rng(42);

        for (int attempt = 0; attempt < max_restarts; ++attempt) {
            restarts_used++;
            rc = solver->CartToJnt(q_in, target_frame, q_out, bounds);
            if (rc >= 0) {
                success = true;
                break;
            }

            // Check if timeout exceeded
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed_s = std::chrono::duration<double>(now - t0).count();
            if (elapsed_s >= maxtime) {
                break;
            }

            // Sample random restart within limits
            for (int j = 0; j < num_joints; ++j) {
                std::uniform_real_distribution<double> dist(q_min(j), q_max(j));
                q_in(j) = dist(rng);
            }
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        double solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        std::vector<double> sol(num_joints, 0.0);
        if (success) {
            for (int i = 0; i < num_joints; ++i) sol[i] = q_out(i);
        }

        return py::make_tuple(success, sol, restarts_used, solve_time_ms, rc);
    }
};

PYBIND11_MODULE(trac_ik_native, m) {
    m.doc() = "Native pybind11 interface for TRACLabs TRAC-IK solver";
    py::class_<TracIKSolverWrapper>(m, "TracIKSolver")
        .def(py::init<double, double>(),
             py::arg("maxtime") = 0.005,
             py::arg("eps") = 1e-4)
        .def("solve", &TracIKSolverWrapper::solve,
             py::arg("target_pos"),
             py::arg("target_rot_mat"),
             py::arg("q_init") = std::vector<double>{0,0,0,0,0},
             py::arg("max_restarts") = 1);
}
