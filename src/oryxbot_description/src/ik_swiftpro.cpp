#include <string>  
#include "ros/ros.h"
#include "oryxbot_description/IK.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "global_constants.hh"
#include "forward_kinematics.hh"
#include "inverse_kinematics.hh"
#include "arm_controller/move.h"

double joint_angle[num_of_joints] = {0.0};
double current_joint_angle[num_of_joints] = {0.0};  // Actual angles from Gazebo
double target_joint_angle[num_of_joints] = {0.0};   // Target angles we commanded
bool joint_state_received = false;

// Global joint publisher variables
ros::Publisher joints_pubs[num_of_joints];
ros::Subscriber joint_state_sub;

// Joint state callback - track actual joint angles from Gazebo
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // Joint names in Gazebo URDF are "Joint1"..."Joint9" (capital J, no prefix)
    // joint_state_controller publishes under /swiftpro/ namespace
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++) {
        const std::string& name = msg->name[i];
        for (int j = 0; j < num_of_joints; j++) {
            // Match "JointN" (URDF name), "swiftpro/arm_jointN", "arm_jointN", "jointN"
            std::string name_urdf = "Joint" + std::to_string(j+1);
            std::string name_with_prefix = "swiftpro/arm_joint" + std::to_string(j+1);
            std::string name_no_prefix = "arm_joint" + std::to_string(j+1);
            std::string name_simple = "joint" + std::to_string(j+1);
            if (name == name_urdf || name == name_with_prefix || name == name_no_prefix || name == name_simple) {
                current_joint_angle[j] = msg->position[i] * deg2rad;  // Convert rad to degrees (deg2rad=57.2958)
                break;
            }
        }
    }
    joint_state_received = true;
}

// Wait for arm joints to reach target positions (timeout in seconds)
bool waitForArmArrival(double timeout_sec, double tolerance_deg) {
    if (!joint_state_received) {
        // Fallback: if no joint state available, use fixed delay
        ROS_WARN("No joint state received, falling back to fixed delay");
        ros::Duration(delay_time).sleep();
        return true;
    }
    
    ros::Rate wait_rate(20);  // Check at 20Hz
    ros::Time start = ros::Time::now();
    // current_joint_angle and target_joint_angle are both in DEGREES
    // so tolerance is directly in degrees (no radian conversion needed)
    // OLD CODE was: tolerance_deg * M_PI / 180.0 = 2 * 0.0175 = 0.035 degrees
    // which was way too small, causing every movement to timeout!
    
    while (ros::ok()) {
        // Check timeout
        if ((ros::Time::now() - start).toSec() > timeout_sec) {
            ROS_WARN("Arm movement timeout after %.1f seconds", timeout_sec);
            return false;
        }
        
        // Check if first 3 arm joints have reached target positions
        bool all_reached = true;
        for (int i = 0; i < 3; i++) {
            double err = fabs(current_joint_angle[i] - target_joint_angle[i]);
            if (err > tolerance_deg) {  // Both values in degrees, compare directly
                all_reached = false;
                break;
            }
        }
        
        if (all_reached) {
            return true;
        }
        
        ros::spinOnce();  // Process joint state callbacks
        wait_rate.sleep();
    }
    return false;
}

bool check_requested_position (arm_controller::move::Request& req)
{
    if ((req.pose.position.x>= min_x && req.pose.position.x<= max_x)&&
    (req.pose.position.y>= min_y && req.pose.position.y<= max_y)&&
    (req.pose.position.z>= min_z && req.pose.position.z<= max_z))
    {return true;}

    else
    {
        ROS_ERROR("Infeasible requested position!, feasible range is x: [%d,%d], y: [%d:%d], z: [%d,%d]",min_x,max_x,min_y,max_y,min_z,max_z);
        return false;
    }
    
}

// This callback function executes whenever a inverse_kinematics service is requested
bool handle_inverse_kinematics_request(arm_controller::move::Request& req,
    arm_controller::move::Response& res)
{
    res.success = false;
    res.message = "";

    if (!check_requested_position(req))
    {
        res.message = "Position out of workspace range";
        return true;
    }

    double position[3];
    double angle[3] = {0};
    // in mm 
    position[0] = req.pose.position.x;
    position[1] = req.pose.position.y;
    position[2] = req.pose.position.z;
    
    try {
        if (!inverse_kinematics(position,angle))
        {
            res.message = "Inverse kinematics failed";
            return true;
        }
        if (!forward_kinematics(joint_angle,angle))
        {
            res.message = "Forward kinematics failed";
            return true;
        }

        // Set the arm joint angles
        std_msgs::Float64 joint_angles[num_of_joints];
        for (int i = 0;i<num_of_joints;i++)
        {
            joint_angles[i].data = joint_angle[i]/deg2rad;
        }
        
        // Publish the arm joint angles
        for (int i = 0;i<num_of_joints;i++)
        {
            joints_pubs[i].publish(joint_angles[i]);
            target_joint_angle[i] = joint_angle[i];
        }

        // Wait for arm to reach target position
        ros::Duration(2.0).sleep();
        res.success = true;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in IK service: %s", e.what());
        res.message = std::string("Exception: ") + e.what();
    } catch (...) {
        ROS_ERROR("Unknown exception in IK service");
        res.message = "Unknown exception";
    }
    return true;
}
int main(int argc, char** argv)
{
    // Initialize the ik_swiftpro node
    ros::init(argc, argv, "ik_swiftpro");
    // Create a handle to the  node (to communicate with ros master) 
    ros::NodeHandle n;
    std::string topic_name; 

    // Publish the arm joint angles
    for (int i = 0;i<num_of_joints;i++)
        {
        topic_name = "/swiftpro/joint" + std::to_string(i+1) + "_position_controller/command";
        joints_pubs[i] = n.advertise<std_msgs::Float64>(topic_name,10);
        }

    // Subscribe to joint states for arrival detection
    // Gazebo remaps to /swiftpro/joint_states, so subscribe to both
    joint_state_sub = n.subscribe("/joint_states", 10, jointStateCallback);
    ros::Subscriber joint_state_sub2 = n.subscribe("/swiftpro/joint_states", 10, jointStateCallback);

    // Define a inverse_kinematics service with a handle_inverse_kinematics_request callback function
    ros::ServiceServer service = n.advertiseService("goto_position", handle_inverse_kinematics_request);
    ROS_INFO("Ready to send x y z coordinates");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
