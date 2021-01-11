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

#ifndef _PATH_FOLLOWER_H
#define _PATH_FOLLOWER_H

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#define PF_STEP_WAIT       0
#define PF_STEP_GOTO     1
#define PF_STEP_NEAR     2
#define PF_STEP_ARRIVED  3

class CPathFollower
{
public:
    CPathFollower();
    virtual ~CPathFollower();
    void Initialize();
    bool ComputeVelocity();
    void getTransformedPosition(geometry_msgs::PoseStamped& pose, std::string& frame_id, double& x, double& y, double& theta);
    void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void PathCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    std::vector<geometry_msgs::PoseStamped> m_global_plan;
    ros::Subscriber m_path_sub;
    ros::Subscriber m_lidar_sub;
    ros::Publisher m_vel_pub;
    ros::Publisher m_path_pub;
    ros::Publisher result_pub;
    std_msgs::String result_msg;
    geometry_msgs::Twist m_cmd_vel;
    geometry_msgs::Twist m_last_cmd;

    std::string m_global_frame_id; 
    std::string m_robot_base_frame_id;
    tf::TransformListener* m_tf_listener;
    
    double m_max_vel_trans;
    double m_max_vel_rot;
    double m_acc_scale_trans;
    double m_acc_scale_rot;
    double m_goal_dist_tolerance;
    double m_goal_yaw_tolerance;

    bool m_bInitialized;
    int m_nStep;
    int m_nPathIndex;
    bool m_bFirstStep;
    bool m_goal_reached;
};
#endif //_PATH_FOLLOWER_H
