#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "arm_controller/move.h"
#include "arm_controller/control.h"
#include "arm_controller/PickPlace.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>

using namespace std;

#define CAMREA_X_MAX 200
#define CAMREA_Y_MAX 0
#define CAMREA_Z_MAX 170

// Z-offset: descend this much BELOW the AR marker center to ensure suction contact.
// Positive value = descend further below marker center (closer to block surface)
#define GRAB_Z_EXTRA_MM 0

// Maximum number of grab attempts within this service call
#define MAX_GRAB_ATTEMPTS 3

class PickAr
{
public:
    PickAr();

    ros::NodeHandle n;
	ros::ServiceServer pick_server,place_server;
    ros::ServiceClient client_pick,client_goto,client_place;
	ros::Subscriber sub;		
    tf::TransformListener listener;
    tf::TransformBroadcaster robot_broadcaster;
    ros::Time current_time, last_time;

    float position[4];
    bool grasping_status;  // 吸盘是否吸住物块
    ros::Subscriber grasping_sub;  // 吸盘状态订阅
    void position_callback(const arm_controller::control& msg);
    void grasping_callback(const std_msgs::Bool::ConstPtr& msg);
    bool pick_callback( arm_controller::PickPlace::Request &req,
                        arm_controller::PickPlace::Response &res);
    bool place_callback( arm_controller::move::Request &req,
                         arm_controller::move::Response &res);

    // Helper: try to grab at a specific position. Returns true if goto succeeded.
    bool try_grab(float gx, float gy, float gz);

    // Helper: wait for grasping confirmation after pump ON
    bool wait_for_grasp(double timeout_sec);

private:
    /* data */
};

//pos_info回调函数
void PickAr::position_callback(const arm_controller::control& msg)
{
    //info当前位置
	position[0] = msg.position.x * 1000;
	position[1] = msg.position.y * 1000;
	position[2] = msg.position.z * 1000;
}

// 吸盘状态回调
void PickAr::grasping_callback(const std_msgs::Bool::ConstPtr& msg)
{
    grasping_status = msg->data;
}

// 等待吸盘确认吸附
bool PickAr::wait_for_grasp(double timeout_sec)
{
    ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < timeout_sec) {
        if (grasping_status) {
            ROS_INFO("Grasp confirmed by vacuum gripper");
            return true;
        }
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("Grasp NOT confirmed within %.1f seconds", timeout_sec);
    return false;
}

// Helper: move to position and return success status
bool PickAr::try_grab(float gx, float gy, float gz)
{
    arm_controller::move pick;
    pick.request.pose.position.x = gx;
    pick.request.pose.position.y = gy;
    pick.request.pose.position.z = gz;
    return client_goto.call(pick) && pick.response.success;
}

//pick_ar回调函数
bool PickAr::pick_callback(arm_controller::PickPlace::Request &req,
                        arm_controller::PickPlace::Response &res)
{
    //获取字符串"ar_marker_number"
    int a=req.number;
    stringstream ss;
    string stra="";
    ss<<"/ar_marker_";
    ss<<a;
    ss>>stra;
    const string &strb=stra;
    
    arm_controller::move pos;
    if(req.mode==0)//mode=0，则机械臂跑到盒子上面抓取物体
    {
        //走到摄像头固定位置-盒子的上方
        pos.request.pose.position.x=CAMREA_X_MAX;
        pos.request.pose.position.y=CAMREA_Y_MAX;
        pos.request.pose.position.z=CAMREA_Z_MAX;
        if (!client_goto.call(pos))
        {
            res.message=" goto camera pos error !";
            res.success=false;
            return true;
        }
    }else if(req.mode==1){

    }else{
        res.message="  Pattern error  !";
        res.success=false;
        return true;
    }
    sleep(1);

    bool grab_ok = false;
    float x_=0, y_=0, z_=0;

    for (int attempt = 0; attempt < MAX_GRAB_ATTEMPTS; attempt++)
    {
        if (attempt > 0) {
            ROS_WARN("=== Grab attempt %d/%d: re-detecting AR marker ===", attempt+1, MAX_GRAB_ATTEMPTS);
            if (req.mode == 0) {
                pos.request.pose.position.x=CAMREA_X_MAX;
                pos.request.pose.position.y=CAMREA_Y_MAX;
                pos.request.pose.position.z=CAMREA_Z_MAX;
                client_goto.call(pos);
            }
            sleep(1);
        }

        //监听tf
        tf::StampedTransform transform;
        try
        {
            if (!listener.canTransform("Base", strb, ros::Time(0))) {
                ROS_ERROR("TF frame '%s' not found in TF tree (attempt %d)", strb.c_str(), attempt+1);
                std::vector<std::string> frames;
                listener.getFrameStrings(frames);
                std::string frame_list;
                for (size_t f = 0; f < frames.size() && f < 20; f++) {
                    frame_list += frames[f] + " ";
                }
                ROS_WARN("Available TF frames (%zu): %s", frames.size(), frame_list.c_str());
            }
            listener.waitForTransform("Base",strb, ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("Base",strb, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("TF lookup failed (attempt %d): %s", attempt+1, ex.what());
            if (attempt == MAX_GRAB_ATTEMPTS - 1) {
                res.message = "TF lookup failed after all attempts";
                res.success = false;
                return true;
            }
            ros::Duration(1.0).sleep();
            continue;
        }

        x_=transform.getOrigin().x()*1000.0;
        y_=transform.getOrigin().y()*1000.0;
        z_=transform.getOrigin().z()*1000.0;
        ROS_INFO("AR marker pos (attempt %d): X=%.3f Y=%.3f Z=%.3f", attempt+1, x_, y_, z_);

        // Clamp coordinates to arm workspace
        float hover_x = std::max(0.0f, std::min(280.0f, x_));
        float hover_y = std::max(-278.0f, std::min(278.0f, y_));
        float hover_z = std::max(0.0f, std::min(280.0f, z_ + 50.0f));
        float mid_z   = std::max(0.0f, std::min(280.0f, z_ + 20.0f));
        float grab_z  = std::max(0.0f, std::min(130.0f, (float)(z_ - GRAB_Z_EXTRA_MM)));

        if (fabs(hover_x - x_) > 10 || fabs(hover_y - y_) > 10) {
            ROS_WARN("AR position (%.1f, %.1f) significantly outside workspace, clamping to (%.1f, %.1f)", 
                     x_, y_, hover_x, hover_y);
        }

        // Step 1: 走到ar码上方
        ROS_INFO("Step1: Move above AR marker hover_z=%.1f", hover_z);
        pos.request.pose.position.x=hover_x;
        pos.request.pose.position.y=hover_y;
        pos.request.pose.position.z=hover_z;
        if (!client_goto.call(pos) || !pos.response.success) {
            ROS_WARN("Failed to move above AR marker (attempt %d)", attempt+1);
            continue;
        }
        sleep(0.5);

        // Step 2: 缓慢下降到中间高度
        ROS_INFO("Step2: Descend to mid_z=%.1f", mid_z);
        pos.request.pose.position.z = mid_z;
        if (!client_goto.call(pos) || !pos.response.success) {
            ROS_WARN("Failed to descend to mid position (attempt %d)", attempt+1);
            continue;
        }
        sleep(0.5);

        // Step 3: 缓慢下降到抓取高度
        ROS_INFO("Step3: Descend to grab_z=%.1f", grab_z);
        if (!try_grab(hover_x, hover_y, grab_z)) {
            ROS_WARN("Failed to descend to grab position (attempt %d)", attempt+1);
            continue;
        }

        // Step 4: 等待机械臂稳定后开气泵
        ROS_INFO("Step4: Waiting for arm to stabilize...");
        sleep(2);

        // Step 5: 开启气泵（吸盘插件自动吸附物块）
        std_srvs::Empty pump;
        client_pick.call(pump);
        ROS_INFO("Pump ON - vacuum gripper active");

        // 等待吸盘确认吸附（最多2秒）
        bool grasp_confirmed = wait_for_grasp(2.0);
        if (!grasp_confirmed) {
            // 没吸住，关气泵重试
            ROS_WARN("Grasp not confirmed on attempt %d, retrying...", attempt+1);
            client_place.call(pump);  // 关气泵
            sleep(0.5);
            continue;
        }

        sleep(0.5);

        // Lift up
        pos.request.pose.position.x=hover_x;
        pos.request.pose.position.y=hover_y;
        pos.request.pose.position.z=hover_z;
        client_goto.call(pos);
        sleep(0.5);

        grab_ok = true;
        ROS_INFO("Grab attempt %d completed successfully", attempt+1);
        break;
    }

    if (!grab_ok) {
        res.message = "Failed to grab after all attempts";
        res.success = false;
        // Turn off pump since we failed
        std_srvs::Empty pump;
        client_place.call(pump);
        return true;
    }

    sleep(0.5);

    if(!((abs(req.pose.position.x)<0.001)&&(abs(req.pose.position.y)<0.001)&&(abs(req.pose.position.z)<0.001))){
        //走到放置位置上方
        ROS_INFO("Moving to place position");
        pos.request.pose.position.x=req.pose.position.x;
        pos.request.pose.position.y=req.pose.position.y;
        pos.request.pose.position.z=req.pose.position.z+50;
        client_goto.call(pos);
        sleep(1.0);

        //下去放下
        arm_controller::move place;
        place.request.pose.position.x=req.pose.position.x;
        place.request.pose.position.y=req.pose.position.y;
        place.request.pose.position.z=req.pose.position.z+10;
        client_goto.call(place);
        sleep(1.0);

        // 关闭气泵，吸盘释放物块
        std_srvs::Empty pump;
        client_place.call(pump);
        ROS_INFO("Pump OFF - block released");
	    ROS_INFO("Place end");
    }else{
        if(req.mode==0)
        {
            ROS_INFO("Returning to camera position");
            pos.request.pose.position.x=CAMREA_X_MAX;
            pos.request.pose.position.y=CAMREA_Y_MAX;
            pos.request.pose.position.z=CAMREA_Z_MAX;
            client_goto.call(pos);
        }else if(req.mode==1){
            pos.request.pose.position.x=position[0];
            pos.request.pose.position.y=position[1];
            pos.request.pose.position.z=position[2];
            client_goto.call(pos);
            ROS_INFO("mode == 1");
        }
    }

    res.success = true;
    return true;	
}

//place_ar回调函数
bool PickAr::place_callback(arm_controller::move::Request &req,
                                 arm_controller::move::Response &res)
{
    //走到放置位置上方
    arm_controller::move pos;
    pos.request.pose.position.x=req.pose.position.x;
    pos.request.pose.position.y=req.pose.position.y;
    pos.request.pose.position.z=req.pose.position.z+50;
    client_goto.call(pos);    
    //下去放下
    arm_controller::move place;
    place.request.pose.position.x=req.pose.position.x;
    place.request.pose.position.y=req.pose.position.y;
    place.request.pose.position.z=req.pose.position.z;
    client_goto.call(place);

    // 关闭气泵
    std_srvs::Empty pump;
    client_place.call(pump);

	res.success = true;
    return true;
}

PickAr::PickAr()
{
    pick_server=n.advertiseService("pick_ar",&PickAr::pick_callback,this);
    place_server=n.advertiseService("place_ar",&PickAr::place_callback,this);

    client_pick=n.serviceClient<std_srvs::Empty>("swiftpro/on");
    client_place=n.serviceClient<std_srvs::Empty>("swiftpro/off");
    client_goto=n.serviceClient<arm_controller::move>("goto_position");

    sub = n.subscribe("arm_controller/position_info",100,&PickAr::position_callback,this);
    grasping_status = false;
    grasping_sub = n.subscribe("swiftpro/grasping", 10, &PickAr::grasping_callback, this);
    current_time = ros::Time::now();
    ROS_INFO("INIT OK - Grab Z offset: %.1fmm, Max attempts: %d", GRAB_Z_EXTRA_MM, MAX_GRAB_ATTEMPTS);
}

int main(int argc,char** argv)
{
    //节点初始化
    ros::init(argc,argv,"robot_pick");
    //实例化类PickAr
    PickAr PickAr;
    //多线程接收
    ros::AsyncSpinner spinner(2); 
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
