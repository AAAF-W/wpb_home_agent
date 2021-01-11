/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/BatteryState.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <waterplus_map_tools/GetChargerByName.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include "RobotInfoSend.h"
#include "WPRAgent.h"

static int Robot_ID = 1;                //机器人ID
static string ServerIP = "127.0.0.1";   //服务器IP地址

static int Robot_Dev = DEV_WPB_HOME;       //机器人设备类型

static std::string name_space = "";
static bool sim_flag = false;

static int path_index = 0;
static ros::Publisher path_pub;
static geometry_msgs::PoseArray path_msg;
static ros::Publisher move_pub;

static CWPRAgent wpr_agent;
static CRobotInfoSend robot_info_send;
static int nRobotState = RBT_ST_STOP;
static stCommandMsg cmd_recv;

static geometry_msgs::Pose navi_msg;
static tf::Quaternion quat;
static ros::Publisher behaviors_pub;
static std_msgs::String behavior_msg;

static bool bNaviDone = false;

static std_msgs::String odom_ctrl_msg;
static geometry_msgs::Pose2D pose_diff;
static ros::Publisher vel_pub;
static ros::Publisher navi_cancel_pub;
static actionlib_msgs::GoalID cancel_msg;
static ros::Publisher relocation_pub;
static geometry_msgs::PoseWithCovarianceStamped relocation_msg; 
static tf::TransformListener* m_tf_listener = NULL; 
static std::string m_robot_base_frame_id = "base_footprint";
static ros::Publisher marker_pub;
static visualization_msgs::Marker text_marker;
static ros::Publisher team_mate_pub;

static float robot_battery = 0;
static float move_x = 0;
static float move_y = 0;
static float move_yaw = 0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void NaviResultCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = msg->data.find("done");
    if( nFindIndex >= 0 )
    {
        bNaviDone = true;
    }
}

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void SetRobotPose(float inX, float inY, float inYaw)
{
    relocation_msg.header.stamp = ros::Time::now();
    relocation_msg.header.frame_id = "map";
    //position
     relocation_msg.pose.pose.position.x = inX;
     relocation_msg.pose.pose.position.y = inY;
     relocation_msg.pose.pose.position.z = 0;
    //yaw
    tf::Quaternion quat_pose = tf::createQuaternionFromYaw(inYaw);
    //quat_pose.setRPY(0.0, 0.0, inYaw);
    relocation_msg.pose.pose.orientation.x = quat_pose.getX();
    relocation_msg.pose.pose.orientation.y = quat_pose.getY();
    relocation_msg.pose.pose.orientation.z = quat_pose.getZ();
    relocation_msg.pose.pose.orientation.w = quat_pose.getW();
    //publish msg
    relocation_pub.publish(relocation_msg);
}

void getTransformedPosition(geometry_msgs::PoseStamped& pose, std::string& frame_id, double& x, double& y, double& theta)
{
    geometry_msgs::PoseStamped tf_pose;
    pose.header.stamp = ros::Time(0);
    m_tf_listener->transformPose(frame_id, pose, tf_pose);
    x = tf_pose.pose.position.x;
    y = tf_pose.pose.position.y,
    theta = tf::getYaw(tf_pose.pose.orientation);
}

void SendTeamMatePose(stCommandMsg* inMsg)
{
    geometry_msgs::PoseArray msg;
    msg.header.frame_id = m_robot_base_frame_id;
    msg.header.stamp = ros::Time(0);
    geometry_msgs::Pose pose;
    for(int i=0;i<2;i++)
    {
        geometry_msgs::PoseStamped tm_pose;
        tm_pose.header.frame_id="map";
        tm_pose.pose.position.x = inMsg->team_mate_x[i];
        tm_pose.pose.position.y = inMsg->team_mate_y[i];
        tm_pose.pose.position.z = 0;
        tm_pose.pose.orientation.w = 1;
        double tm_x ,tm_y,tm_yaw;
        getTransformedPosition(tm_pose, m_robot_base_frame_id,tm_x,tm_y,tm_yaw);
        pose.position.x = tm_x;
        pose.position.y = tm_y;
        msg.poses.push_back(pose);
    }
    team_mate_pub.publish(msg);
}

void MoveTo(float inX , float inY , float inYaw)
{
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id="map";
    target_pose.pose.position.x = inX;
    target_pose.pose.position.y = inY;
    target_pose.pose.position.z = 0;
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, inYaw);
    quaternionTFToMsg(quat, target_pose.pose.orientation);
    double local_x ,local_y,local_yaw;
    getTransformedPosition(target_pose, m_robot_base_frame_id, local_x ,local_y,local_yaw);
    //ROS_WARN("[MoveTo] = ( %.2f , %.2f ) %.2f",local_x ,local_y,local_yaw);

    geometry_msgs::PoseStamped move_msg;
    move_msg.pose.position.x = local_x;
    move_msg.pose.position.y = local_y;
    move_msg.pose.position.z = 0;
    tf::Quaternion local_quat;
    local_quat.setRPY(0.0, 0.0, local_yaw);
    quaternionTFToMsg(local_quat, move_msg.pose.orientation);
    move_pub.publish(move_msg);
}

void StopAll()
{
    navi_cancel_pub.publish(cancel_msg);
}

static float arRanges[1081];
void LidarCB(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(int i=0;i<1081;i++)
    {
        arRanges[i] = scan->ranges[i] ;
    }
}

void DrawRobotID(int inRobotID, std::string inBase)
{
    text_marker.header.frame_id = inBase;
    text_marker.ns = "id";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = inRobotID;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 0.2;
    text_marker.color.r = 1.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = 0;
    text_marker.pose.position.y = 0;
    text_marker.pose.position.z = 1.6;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    std::ostringstream stringStream;
    stringStream <<"ID = "<< inRobotID;
    std::string retStr = stringStream.str();
    text_marker.text = retStr;

    marker_pub.publish(text_marker);
}

void AgentTestCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[AgentTestCB] recv %s",msg->data.c_str());
    int nFindIndex = 0;
    nFindIndex = msg->data.find("goto");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("goto test!");
        wpr_agent.cmd_new.command = RBT_ST_GOTO;
        wpr_agent.cmd_new.map_x = 0.09;
        wpr_agent.cmd_new.map_y = 0.06;
        wpr_agent.cmd_new.map_yaw = -0.03;
        wpr_agent.bNewCmd = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_agent_node");

    ros::NodeHandle n_param("~");
    n_param.getParam("sim_flag", sim_flag);
    if(n_param.getParam("robot_id", Robot_ID))
    {
        if(sim_flag == true)
        {
            std::ostringstream stringStream;
            stringStream << "/wpb_" << Robot_ID;
            name_space = stringStream.str();
            stringStream  << "/base_footprint";
            m_robot_base_frame_id = stringStream.str();;
        }
        else
        {
            name_space = "";
            m_robot_base_frame_id = "/base_footprint";
        }
        robot_info_send.m_robot_base_frame_id = m_robot_base_frame_id;
    }
    n_param.getParam("server_ip", ServerIP);
    ROS_WARN("[wpb_home_agent_node] robot_id = %d server_ip = %s",Robot_ID,ServerIP.c_str());
    m_tf_listener = new tf::TransformListener;
    
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        if(!ros::ok())
            break;
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::NodeHandle n;
    ros::Publisher navi_pub = n.advertise<geometry_msgs::Pose>(name_space + "/navi_pose", 10);
    ros::Subscriber navi_result_sub = n.subscribe(name_space + "/navi_result", 10, NaviResultCB);
    vel_pub = n.advertise<geometry_msgs::Twist>(name_space + "/cmd_vel", 10);
    relocation_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(name_space + "/initialpose", 5, true);
    ros::Subscriber test_sub = n.subscribe(name_space + "/wpb_home/agent_test", 30, AgentTestCB);
    navi_cancel_pub = n.advertise<actionlib_msgs::GoalID>(name_space + "move_base/cancel",1);
    ros::Subscriber scan_sub = n.subscribe(name_space + "/scan",30,LidarCB);
    move_pub = n.advertise<geometry_msgs::PoseStamped>(name_space + "/wpb_home/move", 10);
    marker_pub = n.advertise<visualization_msgs::Marker>(name_space + "/robot_id", 10);
    team_mate_pub = n.advertise<geometry_msgs::PoseArray>(name_space + "/team_mate_pose", 10);
    behaviors_pub = n.advertise<std_msgs::String>(name_space + "/wpb_home/behaviors", 30);

    int nPort = 20200 + Robot_ID;
    wpr_agent.InitUDPServer(nPort);
    robot_info_send.SetID(Robot_ID);
    robot_info_send.SetDevType(Robot_Dev);
    robot_info_send.Initial();
    robot_info_send.InitUDPClient(ServerIP.c_str(),21202);

    ros::Rate r(30.0);
    while(ros::ok())
    {
        /*******************
         * 获取服务器发来的指令
         *******************/
        if(wpr_agent.bNewCmd == true)
        {
            StopAll();
            memcpy(&cmd_recv,&(wpr_agent.cmd_new),sizeof(stCommandMsg));
            switch (cmd_recv.command)
            {
            case CMD_STOP:
                behavior_msg.data = "move stop";
                behaviors_pub.publish(behavior_msg);
                nRobotState = RBT_ST_STOP;
                break;
            case CMD_ROBOT_MOVE:
                nRobotState = RBT_ST_MOVE;
                move_x = cmd_recv.map_x;
                move_y = cmd_recv.map_y;
                move_yaw = cmd_recv.map_yaw;
                MoveTo(move_x ,move_y , move_yaw);
                // ROS_WARN("[wpr_agent - CMD_ROBOT_MOVE] = ( %.2f , %.2f ) %.2f",move_x ,move_y , move_yaw);
                SendTeamMatePose(&cmd_recv);
                break;
            case CMD_FOLLOWER:
                nRobotState = RBT_ST_FOLLOWER;
                move_x = cmd_recv.map_x;
                move_y = cmd_recv.map_y;
                move_yaw = cmd_recv.map_yaw;
                MoveTo(move_x ,move_y , move_yaw);
                //ROS_WARN("[wpr_agent - CMD_FOLLOWER] = ( %.2f , %.2f ) %.2f",move_x ,move_y , move_yaw);
                // 发送队友信息给follower_demo
                //ROS_WARN("[SendTeamMatePose] = tm( %.2f , %.2f )  ( %.2f , %.2f ) ",cmd_recv.team_mate_x[0] ,cmd_recv.team_mate_y[0] , cmd_recv.team_mate_x[1] ,cmd_recv.team_mate_y[1]);
                SendTeamMatePose(&cmd_recv);
                break;
            case CMD_ROBOT_GOTO:
                nRobotState = RBT_ST_GOTO;
                navi_msg.position.x = cmd_recv.map_x;
                navi_msg.position.y = cmd_recv.map_y;
                quat.setRPY(0.0, 0.0, cmd_recv.map_yaw);
                navi_msg.orientation.x = quat.getX();
                navi_msg.orientation.y = quat.getY();
                navi_msg.orientation.z = quat.getZ();
                navi_msg.orientation.w = quat.getW();
                navi_pub.publish(navi_msg);
                bNaviDone = false;
                //ROS_WARN("GOTO wpr_agent.cmd_new ( %.2f , %.2f )",cmd_recv.map_x,cmd_recv.map_y);
                ROS_WARN("GOTO navi_msg ( %.2f , %.2f )",navi_msg.position.x,navi_msg.position.y);
                break;
            case CMD_ROBOT_POSE:
                SetRobotPose(cmd_recv.map_x,cmd_recv.map_y,cmd_recv.map_yaw);
                //ROS_WARN("CMD_ROBOT_POSE ( %.2f , %.2f )  %.2f",cmd_recv.map_x,cmd_recv.map_y,cmd_recv.map_yaw);
                break;
            case CMD_ROBOT_TELEOP:
                nRobotState = RBT_ST_TELEOP;
                break;
            default:
                break;
            }
            
            wpr_agent.bNewCmd = false;
        }
        
         /*******************
         * 机器人行为状态
         *******************/
        if(nRobotState == RBT_ST_STOP)
        {
            VelCmd(0,0,0);
        }

        // [1] 导航
        if(nRobotState == RBT_ST_GOTO)
        {
            if(bNaviDone == true)
            {
                nRobotState = RBT_ST_ARRIVED;
            }
        }

        if(nRobotState == RBT_ST_ARRIVED)
        {
        }

        // 遥控
        if(nRobotState == RBT_ST_TELEOP)
        {
            VelCmd(cmd_recv.vel_x,cmd_recv.vel_y,cmd_recv.vel_angular);
        }

        // 直线移动
        if(nRobotState == RBT_ST_MOVE)
        {
            MoveTo(move_x ,move_y , move_yaw);
        }
        // 跟随者
        if(nRobotState == RBT_ST_FOLLOWER)
        {
            MoveTo(move_x ,move_y , move_yaw);
        }

        /*******************
         * 发送机器人新状态 
         *******************/
        robot_info_send.GetMapPosition();
        robot_info_send.info_msg.state = nRobotState;
        robot_info_send.info_msg.cmd_recv = cmd_recv.command;
        robot_info_send.info_msg.battery = robot_battery;
        robot_info_send.SendInfo();

        DrawRobotID(Robot_ID, name_space + "/base_footprint");

        ros::spinOnce();
        r.sleep();
    }
    delete m_tf_listener;
    return 0;
}