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
#include "WPRAgent.h"

CWPRAgent::CWPRAgent()
{
    memset(&cmd_new,0,sizeof(stCommandMsg));
    lastCmdRecv = 0;
    lastMapXRecv = 0;
    lastMapYRecv = 0;
    lastMapYawRecv = 0;
    bNewCmd = false;
}

CWPRAgent::~CWPRAgent()
{
}

void CWPRAgent::RecvNewPackage(stCommandMsg* inCmd)
{
    memcpy(&cmd_new,inCmd,sizeof(stCommandMsg));

    if(inCmd->command == CMD_ROBOT_GOTO || inCmd->command == CMD_ROBOT_POSE )
    {
        if(inCmd->map_x != lastMapXRecv || inCmd->map_y != lastMapYRecv || inCmd->map_yaw != lastMapYawRecv)
        {
            bNewCmd = true;
        }
        else
        {
            //bNewCmd = false;
        }
        //printf("recv (%.2f , %.2f)  %.2f\n",inCmd->map_x,inCmd->map_y,inCmd->map_yaw);
    }else if(inCmd->command == CMD_ROBOT_TELEOP || inCmd->command == CMD_FOLLOWER || inCmd->command == CMD_ROBOT_MOVE)
    {
        bNewCmd = true;
    }
    else
    {
        if(inCmd->command != lastCmdRecv)
        {
            bNewCmd = true;
        }
        else
        {
            //bNewCmd = false;
        }
    }

    
    lastCmdRecv = inCmd->command;
    lastMapXRecv = inCmd->map_x;
    lastMapYRecv = inCmd->map_y;
    lastMapYawRecv = inCmd->map_yaw;
}
