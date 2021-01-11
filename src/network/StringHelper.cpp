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
#include "StringHelper.h"

CStringHelper::CStringHelper()
{
    
}

CStringHelper::~CStringHelper()
{
}

string CStringHelper::GetCommandStr(int inCmd)
{
    string res;
    switch (inCmd)
    {
    case CMD_STOP:
        res = "CMD_STOP";
        break;
    case CMD_ROBOT_GOTO:
        res = "CMD_ROBOT_GOTO";
        break;
    case CMD_GRAB_BOX:
        res = "CMD_GRAB_BOX";
        break;
    case CMD_GRAB_MOBILE:
        res = "CMD_GRAB_MOBILE";
        break;
    case CMD_PLACE_MOBILE:
        res = "CMD_PLACE_MOBILE";
        break;
    case CMD_GRAB_PALLET:
        res = "CMD_GRAB_PALLET";
        break;
    case CMD_PLACE_PALLET:
        res = "CMD_PLACE_PALLET";
        break;
    case CMD_LEAVE_DOCK:
        res = "CMD_LEAVE_DOCK";
        break;
    case CMD_DOCKING:
        res = "CMD_DOCKING";
        break;
    case CMD_CHARGING:
        res = "CMD_CHARGING";
        break;
    case CMD_GRAB_STORAGE:
        res = "CMD_GRAB_STORAGE";
        break;
    case CMD_PLACE_STORAGE:
        res = "CMD_PLACE_STORAGE";
        break;
    case CMD_CONVEY_BOX:
        res = "CMD_CONVEY_BOX";
        break;
    case CMD_LT_POSE_INIT:
        res = "CMD_LT_POSE_INIT";
        break;
    case CMD_LT_POSE_SET:
        res = "CMD_LT_POSE_SET";
        break;
    default:
        std::ostringstream ss;
        ss << "("<< inCmd << ")Unknown";
        res = ss.str();
        break;
    }
    return res;
}

string CStringHelper::GetMsgTypeStr(int inMsgType)
{
    return "";
}

string CStringHelper::GetDevTypeStr(int inDevType)
{
    string res;
    switch (inDevType)
    {
    case DEV_WPR_1:
        res = "DEV_WPR_1";
        break;
    case DEV_WPB_HOME:
        res = "DEV_WPB_HOME";
        break;
    case DEV_WPV_3:
        res = "DEV_WPV_3";
        break;
    case DEV_EXT_ARM:
        res = "DEV_EXT_ARM";
        break;
    default:
        std::ostringstream ss;
        ss << "("<< inDevType << ")Unknown";
        res = ss.str();
        break;
    }
    return res;
}

string CStringHelper::GetRobotStateStr(int inRobotState)
{
    string res;
    switch (inRobotState)
    {
    case RBT_ST_STOP:
        res = "RBT_STOP";
        break;
    case RBT_ST_GOTO:
        res = "RBT_GOTO";
        break;
    case RBT_ST_ARRIVED:
        res = "RBT_ARRIVED";
        break;
    case RBT_ST_DOCK_FACETO:
        res = "RBT_DOCK_FACETO";
        break;
    case RBT_ST_DOCK_ENTER:
        res = "RBT_DOCK_ENTER";
        break;
    case RBT_ST_DOCK_DONE:
        res = "RBT_DOCK_DONE";
        break;
    case RBT_ST_DOCK_LEAVE:
        res = "RBT_DOCK_LEAVE";
        break;
    case RBT_ST_LEAVE_DONE:
        res = "RBT_LEAVE_DONE";
        break;
    case RBT_ST_CHARGE_DONE:
        res = "RBT_CHARGE_DONE";
        break;
    case RBT_ST_BOX_DETECT :
        res = "BOX_DETECT";
        break;
    case RBT_ST_BOX_F_MOVE:
        res = "BOX_F_MOVE";
        break;
    case RBT_ST_BOX_F_ROT:
        res = "BOX_F_ROT";
        break;
    case RBT_ST_BOX_GRAB:
        res = "BOX_GRAB";
        break;
    case RBT_ST_BOX_DONE:
        res = "BOX_DONE";
        break;
    case RBT_ST_MOBILE_DETECT:
        res = "MOBILE_DETECT";
        break;
    case RBT_ST_MOBILE_F_MOVE:
        res = "MOBILE_F_MOVE";
        break;
    case RBT_ST_MOBILE_F_ROT:
        res = "MOBILE_F_ROT";
        break;
    case RBT_ST_MOBILE_PLACE:
        res = "MOBILE_PLACE";
        break;
    case RBT_ST_MOBILE_DONE:
        res = "MOBILE_DONE";
        break;
    case RBT_ST_PALLET_DETECT:
        res = "PALLET_DETECT";
        break;
    case RBT_ST_PALLET_F_MOVE:
        res = "PALLET_F_MOVE";
        break;
    case RBT_ST_PALLET_F_ROT:
        res = "PALLET_F_ROT";
        break;
    case RBT_ST_PALLET_PLACE:
        res = "PALLET_PLACE";
        break;
    case RBT_ST_PALLET_DONE:
        res = "PALLET_DONE";
        break;
    case RBT_ST_CONVEY_BOX:
        res = "RBT_CONVEY_BOX";
        break;
    case RBT_ST_EXIT_BOX:
        res = "RBT_EXIT_BOX";
        break;
    case RBT_ST_GM_DETECT: 
        res = "ST_GM_DETECT";
        break;
    case RBT_ST_GM_F_MOVE: 
        res = "ST_GM_F_MOVE";
        break;
    case RBT_ST_GM_F_ROT: 
        res = "ST_GM_F_ROT";
        break;
    case RBT_ST_GM_OBJECT: 
        res = "ST_GM_OBJECT";
        break;
    case RBT_ST_GM_GRAB: 
        res = "ST_GM_PLACE";
        break;
    case RBT_ST_GM_DONE: 
        res = "ST_GM_DONE";
        break;
    case RBT_ST_GS_DETECT: 
        res = "ST_GS_DETECT";
        break;
    case RBT_ST_GS_F_MOVE: 
        res = "ST_GS_F_MOVE";
        break;
    case RBT_ST_GS_F_ROT: 
        res = "ST_GS_F_ROT";
        break;
    case RBT_ST_GS_OBJECT: 
        res = "ST_GS_OBJECT";
        break;
    case RBT_ST_GS_GRAB: 
        res = "ST_GS_GRAB";
        break;
    case RBT_ST_GS_DONE: 
        res = "ST_GS_DONE";
        break;
    case RBT_ST_PS_DETECT: 
        res = "ST_PS_DETECT";
        break;
    case RBT_ST_PS_F_MOVE: 
        res = "ST_PS_F_MOVE";
        break;
    case RBT_ST_PS_F_ROT: 
        res = "ST_PS_F_ROT";
        break;
    case RBT_ST_PS_MEASURE: 
        res = "ST_PS_MEASURE";
        break;
    case RBT_ST_PS_PLACE: 
        res = "ST_PS_PLACE";
        break;
    case RBT_ST_PS_DONE: 
        res = "ST_PS_DONE";
        break;

    default:
        std::ostringstream ss;
        ss << "("<< inRobotState << ")Unknown";
        res = ss.str();
        break;
    }
    return res;
}

string CStringHelper::GetTaskStateStr(int inTaskState)
{
    string res;
    switch (inTaskState)
    {
    case T_ST_OFFLINE:
        res = "T_ST_OFFLINE";
        break;
    case T_ST_WAIT:
        res = "T_ST_WAIT";
        break;
    case T_ST_IN_PROGRESS:
        res = "T_IN_PROGRESS";
        break;
    case T_ST_DONE:
        res = "T_ST_DONE";
        break;
    default:
        std::ostringstream ss;
        ss << "("<< inTaskState << ")Unknown";
        res = ss.str();
        break;
    }
    return res;
}

string CStringHelper::GetMissionStateStr(int inMissionState)
{
    string res;
    switch (inMissionState)
    {
    case M_ST_WAIT:
        res = "M_ST_WAIT";
        break;
    case M_ST_IN_PROGRESS:
        res = "M_IN_PROGRESS";
        break;
    case M_ST_COMPLETE:
        res = "M_ST_COMPLETE";
        break;
    default:
        std::ostringstream ss;
        ss << "("<< inMissionState << ")Unknown";
        res = ss.str();
        break;
    }
    return res;
}

