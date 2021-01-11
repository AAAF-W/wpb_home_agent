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
#include "UDPClient.h"

CUDPClient::CUDPClient()
{
    bInited = 0;
}

CUDPClient::~CUDPClient()
{
}

void CUDPClient::InitUDPClient(const char* inServerIP, int inPort)
{
	printf("[CUDPClient]InitUDPClient(IP = %s port = %d )...\n",inServerIP,inPort);
	memcpy(arRemoteIP,inServerIP,16);
	nRemotePort = inPort;

    addr_len =sizeof(struct sockaddr_in);
    /* 建立socket*/
    if((s = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP))<0)
    {
        perror("[CUDPClient::InitUDPClient] Creating socket failed!");
        return;
    }
    /* 填写sockaddr_in*/
    bzero(&addr,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(nRemotePort);
    addr.sin_addr.s_addr = inet_addr(arRemoteIP);

	bInited = 1;
}

void CUDPClient::WordToBytes(int inVal,unsigned char* inDest)
{
    inDest[0] = (unsigned char )(inVal >> 8);
    inDest[1] = (unsigned char )(inVal);
}

void CUDPClient::Int32ToBytes(int inVal,unsigned char* inDest)
{
    inDest[0] = (unsigned char )(inVal >> 24);
    inDest[1] = (unsigned char )(inVal >> 16);
    inDest[2] = (unsigned char )(inVal >> 8);
    inDest[3] = (unsigned char )(inVal);
}

void CUDPClient::FloatToBytes(float inVal,unsigned char* inDest)
{
    memcpy(inDest,(unsigned char*)&inVal,sizeof(float));
}

void CUDPClient::Send(unsigned char* inData,int inLen)
{
	if(bInited == 0)
	    return;
    //printf("UDPClientSend( len = %d )...\n",inLen);
    // printf("[UDP_Send] ");
    // int i =0;
    // for(i=0;i<inLen;i++)
    // {
    //     printf("%.2X ",inData[i]);
    // }
    // printf("\n");
    /* 将字符串传送给server端*/
    sendto(s,inData,inLen,0,(struct sockaddr *)&addr,addr_len);
}
