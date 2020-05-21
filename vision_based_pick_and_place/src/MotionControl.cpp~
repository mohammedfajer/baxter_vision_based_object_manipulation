

/* Author: Mohammed Fajer */

// Standard C++ libraries
#include <iostream>
#include <string>

// ROS libraries
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// Moveit libraries
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/planning_scene/planning_scene.h>


class MotionControl
{

    private:

        std::string planning_group;
        ros::NodeHandle node_handle;
        moveit::planning_interface::MoveGroup group;

        // Interface to deal with the world directly
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // RVIZ publisher
        ros::Publisher display_publisher;
        moveit_msgs::DisplayTrajectory display_trajectory;

    public:
        MotionControl(): planning_group("left_arm"), group(this->planning_group) {
            this->Initialize();
        }

        void SetUpPlanningGroup() {
            // The joint group to plan for and control
            group.allowReplanning(true);
            group.setPoseReferenceFrame("base");
            group.setPlannerId("RRTConnectKConfigDefault");
            group.setGoalTolerance(0.1);
            group.setNumPlanningAttempts(10);
            group.setPlanningTime(5.0);
            group.setMaxVelocityScalingFactor(1.0);
            group.setMaxAccelerationScalingFactor(1.0);
        }

        void VisualizeWithRviz() {
            display_publisher = this->node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        }

        void PrintUsefulInformation() {
            // ------ Information ------
            ROS_INFO("The Robot Reference frame: %s",               group.getPlanningFrame().c_str());
            ROS_INFO("The End-Effector Link for this group: %s",    group.getEndEffectorLink().c_str());
        }

        void PlanningToPose() {
            geometry_msgs::Pose target_pose;

            geometry_msgs::Point displacement_vector;
            displacement_vector.x = 0.0;
            displacement_vector.y = 0.0;
            displacement_vector.z = 0.0;

            geometry_msgs::Quaternion rotation_matrix;
            rotation_matrix.w = 1.0;
            rotation_matrix.x = 0.0;
            rotation_matrix.y = 0.0;
            rotation_matrix.z = 0.0;

            target_pose.position    = displacement_vector;
            target_pose.orientation = rotation_matrix;

            group.setPoseTarget( target_pose );

            moveit::planning_interface::MoveGroup::Plan my_plan;
            moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

            ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "":"FAILED");
            sleep(5.0);

            bool visualizePlan = false;
            if( visualizePlan )
            {
                ROS_INFO("Visualizing plan 1 (Again)");
                display_trajectory.trajectory_start = my_plan.start_state_;
                display_trajectory.trajectory.push_back(my_plan.trajectory_);
                display_publisher.publish(display_trajectory);
                sleep(5.0);
            }

            group.move();
        }

        void Initialize() {
            this->planning_group = "left_arm";

            this->SetUpPlanningGroup();
            this->VisualizeWithRviz();
            this->PrintUsefulInformation();
        }
};

int main( int argc, char** argv )
{
    ros::init(  argc, argv, "MotionControl" );

    ros::AsyncSpinner spinner(1);
    spinner.start();

    return 0;
}
