#ifndef ARM_H
#define ARM_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <group_04_a2/ArmAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_srvs/Empty.h>
#include <gazebo_ros_link_attacher/Attach.h>

class Arm
{
public:
    Arm(std::string name):
    as_(nh_, name, boost::bind(&Arm::goalCB, this, _1), false),
    action_name_(name)
    {
        as_.start();
    }

    ~Arm(){}

    /// @brief Callback function for the action server
    /// @param goal from the client
    void goalCB(const group_04_a2::ArmGoalConstPtr &goal);

    /// @brief Pick the object from the table
    /// @param goal 
    void pickObject(const group_04_a2::ArmGoalConstPtr &goal);

    /// @brief Place the object on the table
    /// @param goal 
    void placeObject(const group_04_a2::ArmGoalConstPtr &goal);


protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<group_04_a2::ArmAction> as_;
    std::string action_name_;
    group_04_a2::ArmFeedback feedback_;
    group_04_a2::ArmResult result_;

private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    ros::ServiceClient attachClient_;
    ros::ServiceClient detachClient_;
    
    /// @brief Add the objects to the collision objects
    /// @param objects
    /// @param ids
    /// @param pick, true if pick, false if place
    void addCollisionObjects(std::vector<geometry_msgs::Pose>& objects, std::vector<int>& ids, bool pick);

    /// @brief Subroutine to pick the object using intermediate poses
    /// @param object, pose of the object to pick
    /// @param id, id of the object to pick
    void pickObj(const geometry_msgs::Pose& object, int id);

    /// @brief Given a path of poses, move the arm
    /// @param path, path of poses
    void moveArmPath(const std::vector<geometry_msgs::Pose>& path);

    /// @brief Given an object id, return the dimensions of the object
    /// @param id of the object
    std::vector<double> returnDimesions(int id);

    /// @brief Open or close the gripper
    /// @param open, true if open, false if close
    void gripper(bool open, int id);

    /// @brief Attach the object to the gripper and close the gripper
    /// @param id of the object
    void attachObjectToGripper(int id);

    /// @brief Detach the object from the gripper and open the gripper
    /// @param id of the object
    void detachObjectFromGripper(int id);

    /// @brief Move the arm to a safe pose or tuck it
    /// @param tuck, true if tuck, false if safe pose
    void safePose(bool tuck);
};




#endif // ARM_H