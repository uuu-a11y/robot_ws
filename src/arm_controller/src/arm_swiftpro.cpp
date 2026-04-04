/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2020, reinovo, Inc.
 * All rights reserved.
 * Author: LN  <825255961@qq.com>	   
 */

#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>//string转化为double
#include <iomanip>//保留有效小数
#include "unistd.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

#include "arm_controller/move.h"
#include "arm_controller/rotate.h"
#include "arm_controller/control.h"
#include "std_srvs/SetBool.h"
#include "arm_controller/relative_pos.h"

using namespace std;

std::string read_data;
serial::Serial _serial;				// serial object
arm_controller::control pos_;

//获取位姿
bool get_pos()
{
	//向串口发送P2220获取位姿
	std::string Gcode = "",	data="";
	Gcode = (std::string)"P2220" + "\r\n";
	_serial.write(Gcode.c_str());
	//读取串口返回消息
	data = _serial.read(_serial.available());
	//std::cout << data << endl;

	//分割返回数据
    std::vector<std::string> v;
    std::string::size_type pos1, pos2, pos3;
    pos1 = data.find("X");
    pos2 = data.find("Y");
    pos3 = data.find("Z");

	v.push_back(data.substr(pos1+1,pos2-pos1));	//X
	v.push_back(data.substr(pos2+1,pos3-pos2));	//Y
	v.push_back(data.substr(pos3+1,data.length()-pos3-1));	//Z

	//赋值
	pos_.position.x	=	std::atof(v[0].c_str());
	pos_.position.y	=	std::atof(v[1].c_str());
	pos_.position.z	=	std::atof(v[2].c_str());
}

//goto pos
bool goto_pos(float gx,float gy,float gz)
{
	std::string Gcode = "";
	string result;
	ros::Rate loop_rate(20);

	_serial.read(_serial.available());

	//float转字符
	char x[10];
	char y[10];
	char z[10];

	sprintf(x, "%.2f", gx);
	sprintf(y, "%.2f", gy);
	sprintf(z, "%.2f", gz);
	Gcode = (std::string)"M2222 X" + x + " Y" + y + " Z" + z + "\n";
	ROS_INFO("%s", Gcode.c_str());
	//将移动数据发送到串口
	_serial.write(Gcode.c_str());
	//延时
	usleep(200*1000);
	//读取返回
	result = _serial.read(_serial.available());

	if (result.find("V0") < 100){				//如果该点不在工作范围内
		return false;
	}else if (result.find("V1") < 100){			//如果该点在工作范围内
		Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F10000" + "\n";
		ROS_INFO("%s", Gcode.c_str());
		_serial.write(Gcode.c_str());
		usleep(200*1000);
		result = _serial.read(_serial.available());
		if (result.find("E26") < 100)			//如果目标点可以达到而直线插补不到
		{
			//先将其运动到中间点200,0,100
			char x1[10];
			char y1[10];
			char z1[10];
			sprintf(x1, "%.2f", 200.0);
			sprintf(y1, "%.2f", 0.0);
			sprintf(z1, "%.2f", 100.0);
			Gcode = (std::string)"G0 X" + x1 + " Y" + y1 + " Z" + z1 + " F10000" + "\r\n";
			ROS_INFO("%s", Gcode.c_str());
			_serial.write(Gcode.c_str());
			result = _serial.read(_serial.available());
			int i=0;
			while (!(abs(pos_.position.x-200.0)<1&&abs(pos_.position.y-0.0)<1&&abs(pos_.position.z-100.0)<1))
			{
				get_pos();
				loop_rate.sleep();
				i++;
				if(i>200)return false;
			}
			//然后再运动到目标点
			Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F10000" + "\n";
			_serial.write(Gcode.c_str());
			result = _serial.read(_serial.available());
		}
		//等待移动完成
		int i=0;
		while (!(abs(pos_.position.x-gx)<1&&abs(pos_.position.y-gy)<1&&abs(pos_.position.z-gz)<1))
		{
			/* code for loop body */
			get_pos();
			loop_rate.sleep();
			i++;
			// result = _serial.read(_serial.available());
			// if (result.find("@6") < 100&& result.find("V1")){
			// 	return true;
			// }
			if(i>200)
				return false;
		}
		return true;		
	}
}
bool goto_angle(arm_controller::rotate::Request &req,
						arm_controller::rotate::Response &res)
{   if (req.angle<0 ||req.angle>180){
        res.success=false;
		res.message="Angular range 0~180";
		return true;
      }
	std::string Gcode = "";
	string result;
	ros::Rate loop_rate(20);

	_serial.read(_serial.available());
    Gcode = (std::string)"M2201 N" + "3" + " \n";
	ROS_INFO("发送数据 : %s", Gcode.c_str());
	//将移动数据发送到串口
	_serial.write(Gcode.c_str());
	//延时100ms
	usleep(50*1000);
	//读取返回
	result = _serial.read(_serial.available());
    cout << result << endl;



	//float转字符
	char x[10];
	std::int16_t gx=req.angle;

	sprintf(x, "%d", gx);
	
	Gcode = (std::string)"G2202 N" + "3" + " V" + x + " \n";
	ROS_INFO("发送数据 : %s", Gcode.c_str());
	//将移动数据发送到串口
	_serial.write(Gcode.c_str());
	//延时100ms
	usleep(50*1000);
	//读取返回

	result = _serial.read(_serial.available());

	cout << result << endl;
    res.success=(result.find("ok") != std::string::npos);
	return true;
		
}
//goto pos 回调函数
bool goto_position_deal(arm_controller::move::Request &req,
						arm_controller::move::Response &res)
{
	//调用goto 函数
	res.success=goto_pos(req.pose.position.x,req.pose.position.y,req.pose.position.z);
	return true;
}
//放置
bool place_deal(arm_controller::move::Request &req, 
				arm_controller::move::Response &res)
{
	//调用goto 函数
	res.success=goto_pos(req.pose.position.x,req.pose.position.y,req.pose.position.z);
	//函数返回false 则退出
	if(!res.success)
		return true;
	//发送关闭气泵指令M2231 V0
	std::string Gcode = "";
	Gcode = (std::string)"M2231 V0" + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	read_data = _serial.read(_serial.available());
	return true;
}

//抓取回调函数
bool pick_deal(arm_controller::move::Request &req, 
				arm_controller::move::Response &res)
{
	//调用goto 函数
	res.success=goto_pos(req.pose.position.x,req.pose.position.y,req.pose.position.z);
	//函数返回false 则退出
	if(!res.success){
		res.success=false;
		return true;
	}
	//发送打开气泵指令M2231 V1
	std::string Gcode = "";
	Gcode = (std::string)"M2231 V1" + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	int i = _serial.write(Gcode.c_str());
	res.success=1;
	read_data = _serial.read(_serial.available());
	return true;
}
bool relativeMotion_deal(arm_controller::relative_pos::Request& req,
														arm_controller::relative_pos::Response& res){
	float x,y,z;
	x = pos_.position.x + req.dx;
	y = pos_.position.y + req.dy;
	z = pos_.position.z + req.dz;

	res.success=goto_pos(x,y,z);
	return true;
}
bool pump(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	if(req.data==true)
	{
		//发送打开气泵指令M2231 V1
		std::string Gcode = "";
		Gcode = (std::string)"M2231 V1" + "\r\n";
		ROS_INFO("%s", Gcode.c_str());
		int i = _serial.write(Gcode.c_str());
		res.success=true;
		read_data = _serial.read(_serial.available());
	}
	else if(req.data==false){
		//发送关闭气泵指令M2231 V1
		std::string Gcode = "";
		Gcode = (std::string)"M2231 V0" + "\r\n";
		ROS_INFO("%s", Gcode.c_str());
		int i = _serial.write(Gcode.c_str());
		res.success=true;
		read_data = _serial.read(_serial.available());
	}
	return true;
}
/* 
 * Node name:
 *	 swiftpro_write_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *	 swiftpro_state_topic
 *
 * Topic subscribe: (queue size = 1)
 *	 position_write_topic
 *	 swiftpro_status_topic
 *	 angle4th_topic
 *	 gripper_topic
 *	 pump_topic
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "swiftpro_write_node");
	ros::NodeHandle nh;

	//服务器 pick
	ros::ServiceServer pick_server = nh.advertiseService("pick",pick_deal);
	//服务器 place
	ros::ServiceServer place_server = nh.advertiseService("place",place_deal);
	//服务器 goto_position
	ros::ServiceServer goto_position_server = nh.advertiseService("goto_position",goto_position_deal);
    //服务器 goto_angle
	ros::ServiceServer goto_angle_server = nh.advertiseService("goto_angle",goto_angle);
	//服务器 relative_position
	ros::ServiceServer relativeMotion_server = nh.advertiseService("relative_position",relativeMotion_deal);
	//服务器pump
	ros::ServiceServer pump_server = nh.advertiseService("pump",pump);
	//发布者 arm_controller/position_info 发布机械臂位置信息
	ros::Publisher 	 pub = nh.advertise<arm_controller::control>("arm_controller/position_info", 1);
	//10hz
	ros::Rate loop_rate(10);

	//串口连接
	try
	{
		_serial.setPort("/dev/ttyACM0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		ROS_INFO_STREAM("Port has been open successfully");
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}
	if (_serial.isOpen())
	{
		ros::Duration(3.5).sleep();				// wait 3.5s
		_serial.write("M2120 V0\r\n");			// stop report position
		ros::Duration(0.1).sleep();				// wait 0.1s
		_serial.write("M17\r\n");				// attach
		ros::Duration(0.1).sleep();				// wait 0.1s
		ROS_INFO_STREAM("Attach and wait for commands");
	}

	//读取初始化信息
	while (ros::ok())
	{
		string read_line;
		_serial.readline(read_line,65535,"\n");
		cout << read_line;
		if (read_line.find("@5 V1") < 1000) break;
		ros::Duration(0.05).sleep();				// wait 0.1s
	}
	//开始主循环
	while (ros::ok())
	{
		//获取机械臂位姿函数
		get_pos();
		//如果机械臂位姿未获取到则不发布消息
		if(abs(pos_.position.x)<0.1&&abs(pos_.position.y)<0.1&&abs(pos_.position.z)<0.1)
		{
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}
		//发布话题 arm_controller/position_info
		pub.publish(pos_);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
