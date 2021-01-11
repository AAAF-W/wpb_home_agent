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
#include "UDPServer.h"

CUDPServer::CUDPServer()
{
}

CUDPServer::~CUDPServer()
{
}


int CUDPServer::BytesToWord(unsigned char* inBuf)
{
    int res = 0;
    res = inBuf[0];
    res <<= 8;
    res |= inBuf[1];
    return res;
}

float CUDPServer::BytesToFloat(unsigned char* inBuf)
{
    float res = 0;
    memcpy((char*)&res,inBuf,sizeof(float));
    return res;
}

void *threadUDPServer_func(void *arg)
{
    //printf ("[CUDPServer]threadUDPServer_func start...\n");
    CUDPServer* pServer = (CUDPServer*)arg;
    socklen_t addr_len = sizeof(struct sockaddr_in);
    char* buffer = new char[1024];
    while(1)
	{
        int len = recvfrom(pServer->sockfd,buffer,1024, 0 , (struct sockaddr *)&(pServer->addr) ,&addr_len);
        pServer->Receive(buffer, len);
    }
    delete []buffer;
    //printf("[CUDPServer]threadUDPServer_func exit\n");
    pthread_exit(0);
}

void CUDPServer::InitUDPServer(int inPort)
{
	//printf("[CUDPServer]InitUDPServer( port = %d )...\n",inPort);

    /*建立socket*/
    if((sockfd=socket(AF_INET,SOCK_DGRAM,0))<0)
    {
        perror ("[CUDPServer::InitUDPServer] Creating socket failed!");
        return;
    }
    /*填写sockaddr_in 结构*/
    bzero ( &addr, sizeof(addr) );
    addr.sin_family=AF_INET;
    addr.sin_port=htons(inPort);
    addr.sin_addr.s_addr=htonl(INADDR_ANY) ;
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr))<0)
    {
        perror("[CUDPServer::InitUDPServer] Binding socket failed!");
        return;
    }

    int res;
    res = pthread_create(&hThread, NULL, threadUDPServer_func, (void*)this);
}