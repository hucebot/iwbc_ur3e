#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <signal.h>

/// simulation files
#include <dart/dynamics/BodyNode.hpp>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robots/ur3e.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/force_torque.hpp>
#include <robot_dart/sensor/imu.hpp>
#include <robot_dart/sensor/torque.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

/// controller
#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"
#include "inria_wbc/robot_dart/utils.hpp"
#include "tsid/tasks/task-self-collision.hpp"



int main(int argc, char* argv[])
{
    // a few parameters for this test
    bool verbose = true;
    int sim_freq = 500;
    int control_freq = 500;
    double sim_duration = 15.0; // 15 s
    bool show_collisions = true;
    auto controller_path = "../etc/controller.yaml";
    auto collision_task = "ee_collisions";

    try {
        ////////////// CONTROLLER
        auto controller_config = IWBC_CHECK(YAML::LoadFile(controller_path));
        auto controller = std::make_shared<inria_wbc::controllers::PosTracker>(controller_config);

        ////////////// SIMULATOR
        // robot
        auto robot = std::make_shared<robot_dart::robots::Ur3e>();
        robot->set_position_enforced(true);
        robot->set_actuator_types("servo"); // torque or velocity
        // add the robot to the simulator
        auto simu = std::make_shared<robot_dart::RobotDARTSimu>(1.0 / sim_freq);
        simu->set_collision_detector("fcl");
        simu->set_control_freq(control_freq);
        simu->add_robot(robot);
        auto floor = simu->add_checkerboard_floor();

#ifdef GRAPHIC
        // add some graphical display 
        robot_dart::gui::magnum::GraphicsConfiguration configuration;
        configuration.width = 1280;
        configuration.height = 960;
        auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(configuration);
        simu->set_graphics(graphics);
        graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
        graphics->record_video("video.mp4");
#endif

        // self-collision shapes : for visual debugging
        std::vector<std::shared_ptr<robot_dart::Robot>> self_collision_spheres;
        auto task_self_collision = controller->task<tsid::tasks::TaskSelfCollision>(collision_task);
        for (size_t i = 0; i < task_self_collision->avoided_frames_positions().size(); ++i) {
            auto pos = task_self_collision->avoided_frames_positions()[i];
            auto tf = Eigen::Isometry3d(Eigen::Translation3d(pos[0], pos[1], pos[2]));
            double r0 = task_self_collision->avoided_frames_r0s()[i];
            auto sphere = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(r0 * 2, r0 * 2, r0 * 2), tf, "fixed", 1, Eigen::Vector4d(0, 1, 0, 0.5), "self-collision-" + std::to_string(i));
            sphere->set_color_mode("aspect");
            self_collision_spheres.push_back(sphere);
            simu->add_visual_robot(self_collision_spheres.back());
        }


        //////////////////// START SIMULATION //////////////////////////////////////
        // initialize the positions of the simulation
        robot->set_positions(controller->q0(), controller->all_dofs());

        // start the loop
        while (simu->scheduler().next_time() < sim_duration) {

            // step the command
            if (simu->schedule(simu->control_freq())) {
                controller->update();
                // positions are in: controller->q(false); 
            }
            // show the collisions spheres
            if (simu->schedule(simu->graphics_freq()) && show_collisions) {
                auto task_self_collision = controller->task<tsid::tasks::TaskSelfCollision>(collision_task);
                for (size_t i = 0; i < task_self_collision->avoided_frames_positions().size(); ++i) {
                    auto cp = self_collision_spheres[i]->base_pose();
                    cp.translation() = task_self_collision->avoided_frames_positions()[i];
                    //cp.translation()[0] -= 1; // move to the ghost
                    self_collision_spheres[i]->set_base_pose(cp);
                    auto bd = self_collision_spheres[i]->skeleton()->getBodyNodes()[0];
                    auto visual = bd->getShapeNodesWith<dart::dynamics::VisualAspect>()[0];
                    visual->getShape()->setDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);
                    bool c = task_self_collision->collision(i);
                    if (c) {
                        visual->getVisualAspect()->setRGBA(dart::Color::Red(1.0));
                    }
                    else {
                        visual->getVisualAspect()->setRGBA(dart::Color::Green(1.0));
                    }
                }
            }

            // step the simulation
            {
                auto q = controller->q(false); // get the computed joint positions 
                auto cmd = inria_wbc::robot_dart::compute_velocities(robot, q, 1. / control_freq, controller->all_dofs());
                robot->set_commands(cmd, controller->all_dofs());
                simu->step_world();
            }
        } // end sim loop
    } // end try-catch
    catch (YAML::RepresentationException& e) {
        std::cout << "YAML Parse error (missing key in YAML file?): " << e.what() << std::endl;
    }
    catch (YAML::ParserException& e) {
        std::cout << "YAML Parse error: " << e.what() << std::endl;
    }
    catch (std::exception& e) {
        std::cout << "Error (exception): " << e.what() << std::endl;
    }
    return 0;
}
