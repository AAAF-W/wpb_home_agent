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
#ifndef _EXS_STRUCT_H
#define _EXS_STRUCT_H
#include <vector>
#include <string>

#include "ros/ros.h"

using namespace std;

/***************************************************************************
 * COMMAND 调度系统给机器人发送的指令
 **************************************************************************/
typedef struct stCommandMsg
{
    unsigned char header[2];
    unsigned char msg_type;
    unsigned char id;
    int command;
    float map_x;
    float map_y;
    float map_yaw;
    float vel_x;
    float vel_y;
    float vel_angular;
    int path_index;
    int box_color;
    float follow_angle;
    float follow_dist;
    float team_mate_x[2];
    float team_mate_y[2];
    float data[10]; //扩展用
}stCommandMsg;

// command 指令内容
#define CMD_STOP                        0   // 停止移动
#define CMD_ROBOT_GOTO      1   // 移动到指定坐标
#define CMD_GRAB_BOX             2   // 抓取物料盒子
#define CMD_GRAB_MOBILE     3   // 抓取启程3移动货架的物品
#define CMD_PLACE_MOBILE   4   // 将物料盒子放置到启程3移动货架上
#define CMD_GRAB_PALLET      5   // 抓取固定货架的物品
#define CMD_PLACE_PALLET    6   // 将物料盒子放置到固定货架上
#define CMD_LEAVE_DOCK        7   // 离开充电坞
#define CMD_DOCKING                8   // 进入充电坞
#define CMD_CHARGING             9   // 充电
#define CMD_GRAB_STORAGE      10   // 抓取储存货架的物品
#define CMD_PLACE_STORAGE    11   // 将物料盒子放置到储存货架上

#define CMD_GRAB_COLOR           12   //抓取指定颜色的料盒
#define CMD_PLACE_COLOR         13   //将料盒放置在特定颜色区域

#define CMD_ROBOT_MOVE          20  // 直线移动

#define CMD_FOLLOW_PATH         30   // 手动指定轨迹移动

#define CMD_CONVEY_BOX            50   // 传送料盒

#define CMD_LT_POSE_INIT          100  // 借助LinkTrack系统标定初始位置
#define CMD_LT_POSE_SET           101  // 使用LinkTrack系统直接设置位置

#define CMD_ROBOT_POSE           200  // 给机器人设置当前位置
#define CMD_ROBOT_TELEOP      210  // 设置机器人遥控速度
#define CMD_FOLLOWER                 220   // 跟随一个Leader

/***************************************************************************
 * ROBOT_INFO_MESSAGE 机器人反馈给调度系统的信息
 **************************************************************************/
typedef struct stRobotInfoMsg
{
    unsigned char header[2];
    unsigned char msg_type;
    unsigned char dev_type;
    unsigned char id;
    unsigned char battery;
    float map_x;
    float map_y;
    float map_yaw;
    int cmd_recv;
    int state;
    float joint_pos[10];
    int box_color;
}stRobotInfoMsg;

// msg_type 消息类型
#define MSG_T_ROBOT_STATE   1   // 机器人状态消息
#define MSG_T_PATH          2   // 路径消息
#define MSG_T_SERVER_CMD    10  // 服务器发给机器人的指令

// dev_type 设备型号
#define DEV_WPR_1               1
#define DEV_WPB_HOME     2
#define DEV_WPV_3                3
#define DEV_EXT_ARM           4
#define DEV_WPB_MANI       5

typedef struct stPathMsg
{
    unsigned char header[2];
    unsigned char msg_type;
    unsigned char dev_type;
    unsigned char id;
    unsigned char len;
    float path_x[100];
    float path_y[100];
}stPathMsg;

// state 机器人状态
#define RBT_ST_STOP         0   // 闲置待命状态
#define RBT_ST_GOTO         1   // 导航移动状态
#define RBT_ST_ARRIVED      2   // 到达导航目标
#define RBT_ST_MOVE          3   // 直线移动

#define RBT_ST_FOLLOW_PATH                  30   // 沿路径移动中
#define RBT_ST_FOLLOW_PATH_END      31   // 移动到达路径重点

#define RBT_ST_LT_INIT      80  // 借助LinkTrack系统标定初始位置

#define RBT_ST_DOCK_FACETO  90  // 对准充电坞
#define RBT_ST_DOCK_ENTER   91  // 进入充电坞
#define RBT_ST_DOCK_DONE    92  // 进入完成

#define RBT_ST_DOCK_LEAVE   95  // 离开充电坞
#define RBT_ST_LEAVE_DONE   96  // 离开完成
  
#define RBT_ST_CHARGE_DONE  99  // 充电完成

#define RBT_ST_BOX_DETECT   101   // 检测物料盒子
#define RBT_ST_BOX_F_MOVE   102   // 靠近物料盒子
#define RBT_ST_BOX_F_ROT    103   // 对准物料盒子
#define RBT_ST_BOX_GRAB     104   // 抓取物料盒子
#define RBT_ST_BOX_DONE     105   // 拿到物料盒子

#define RBT_ST_PLACE_COLOR                    110   // 抓取物料盒子
#define RBT_ST_PLACE_COLOR_DONE     111   // 拿到物料盒子

#define RBT_ST_MOBILE_DETECT   201   // 检测移动货架
#define RBT_ST_MOBILE_F_MOVE   202   // 靠近移动货架
#define RBT_ST_MOBILE_F_ROT    203   // 对准移动货架
#define RBT_ST_MOBILE_PLACE    204   // 放置移动货架
#define RBT_ST_MOBILE_DONE     205   // 放置完毕

#define RBT_ST_PALLET_DETECT   301   // 检测固定货架
#define RBT_ST_PALLET_F_MOVE   302   // 靠近固定货架
#define RBT_ST_PALLET_F_ROT    303   // 对准固定货架
#define RBT_ST_PALLET_MEASURE  304   // 定位放置空位
#define RBT_ST_PALLET_PLACE    305   // 放置固定货架
#define RBT_ST_PALLET_DONE     306   // 放置完毕

#define RBT_ST_CONVEY_BOX      401   // 传送带传送料盒
#define RBT_ST_EXIT_BOX        402   // 料盒到达转移口

#define RBT_ST_GM_DETECT   501   // 检测移动货架
#define RBT_ST_GM_F_MOVE   502   // 靠近移动货架
#define RBT_ST_GM_F_ROT    503   // 对准移动货架
#define RBT_ST_GM_OBJECT   504   // 搜索移动货架上的物品
#define RBT_ST_GM_GRAB     505   // 抓取移动货架上的料盒
#define RBT_ST_GM_DONE     506   // 抓取完毕

#define RBT_ST_GS_DETECT   601   // 检测存储货架
#define RBT_ST_GS_F_MOVE   602   // 靠近存储货架
#define RBT_ST_GS_F_ROT    603      // 对准存储货架
#define RBT_ST_GS_OBJECT   604   // 搜索存储货架上的物品
#define RBT_ST_GS_GRAB     605   // 抓取存储货架上的料盒
#define RBT_ST_GS_DONE     606   // 抓取完毕

#define RBT_ST_PS_DETECT   621   // 检测存储货架
#define RBT_ST_PS_F_MOVE   622   // 靠近存储货架
#define RBT_ST_PS_F_ROT    623     // 对准存储货架
#define RBT_ST_PS_MEASURE   624   // 搜索存储货架上的放置空位
#define RBT_ST_PS_PLACE   625    // 将物品放置到存储货架上
#define RBT_ST_PS_DONE     626   // 抓取完毕

#define RBT_ST_TELEOP           1000   //机器人处于遥控状态
#define RBT_ST_FOLLOWER     1010   // 跟随一个Leader

/***************************************************************************
 * MISSION_STRUCT 任务调度数据结构
 **************************************************************************/
typedef struct stTask
{
    stCommandMsg command;
    int state;
}stTask;

// state 单体任务状态
#define T_ST_OFFLINE        0   // 任务执行者离线
#define T_ST_WAIT           1   // 单体任务等待执行
#define T_ST_IN_PROGRESS    2   // 单体任务正在执行
#define T_ST_DONE           3   // 单体任务完成

typedef struct stMission
{
    string mission;
    vector<stTask> arTask;
    int state;
}stMission;

// state 群体任务状态
#define M_ST_WAIT        0   // 群体任务等待执行
#define M_ST_IN_PROGRESS 1   // 群体任务正在执行
#define M_ST_COMPLETE    2   // 群体任务完成

typedef struct stRobotState
{
    stRobotInfoMsg info;
    int package_recv;
    int package_recv_last;
    int state;  //T_ST_XX
}stRobotState;

#endif