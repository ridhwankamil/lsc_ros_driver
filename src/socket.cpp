/*
* Copyright (c) 2022, Autonics Co.,Ltd.
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met:
* 
*   * Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*
*   * Redistributions in binary form must reproduce the above 
*     copyright notice, this list of conditions and the following 
*     disclaimer in the documentation and/or other materials provided 
*     with the distribution.
*
*   * Neither the name of the Autonics Co.,Ltd nor the names of its 
*     contributors may be used to endorse or promote products derived 
*     from this software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE.
*/


#include <sys/poll.h>
#include <stdio.h>

#include "socket.hpp"

Socket::Socket()
{
    m_server_sock_ = 0;
    m_connected_ = 0;

    rcv_timeout_ = false;
    rcv_error_ = false;

    pthread_running_ = false;
    pthrd_id_ = 0;
}

Socket::~Socket()
{
    pthread_running_ = false;

    if(pthrd_id_)
    {
        pthread_join(pthrd_id_, NULL);
    }

    if(m_connected_ == true)
    {
        close(m_server_sock_);

        m_connected_ = false;

        std::cout << "disconnect server" << std::endl;
    }
}

int Socket::getServerSocket(void)
{
    return m_server_sock_;
}

bool Socket::getConnected(void)
{
    return m_connected_;
}

bool Socket::getPthreadRunning(void)
{
    return pthread_running_;
}


bool Socket::getRcvTimeout(void)
{
    return rcv_timeout_;
}

bool Socket::getRcvError(void)
{
    return rcv_error_;
}

void Socket::setRcvTimeout(bool flag)
{
    rcv_timeout_ = flag;
}

void Socket::setRcvError(bool flag)
{
    rcv_error_ = flag;
}



int Socket::tryReconnection(void)
{
    int ret = 0;
    pthread_running_ = false;

    if(pthrd_id_)
    {
        pthread_join(pthrd_id_, NULL);
        pthrd_id_ = 0;
    }
    
    m_connected_ = false;
    close(m_server_sock_);

    usleep(1000*1000);

    ret = clientOpen(addr_st_, port_num_st_);

    if(ret < 0)
    {
        std::cout << "reconnection failed" << std::endl;
        return ret;
    }
    
    return ret;
}

void Socket::putBufToMsg(unsigned char* buf, uint16_t size)
{
    std::vector<unsigned char> vec;

    for(int i = 0; i < size; i++)
    {
        vec.push_back(*(buf+i));
    }

    recvQueue.push(vec);   
}


void* readCallback(void* arg)
{
    int32_t read_cnt = 0;
    uint16_t packet_size = 0, total_byte = 0, offset = 0, recv_byte = 0, \
             buf_size = 16384;
    int ret = 0;
    unsigned char r_buffer[buf_size] = {0, }, \
                  buffer[buf_size] = {0, }, \
                  cp_temp[16] = {0, };
    Socket* sock = (Socket*) arg;
    struct pollfd fd;
    ssize_t chk_connection = 0;

    fd.fd = sock->getServerSocket();
    fd.events = POLLIN;

    while(sock->getPthreadRunning())
    {
        read_cnt = 0;
        offset = 0;

        ret = poll(&fd, 1, 1000);

        if(ret < 0)
        {
            sock->setRcvError(true);
            std::cout << "polling error" << std::endl;
        }
        else if(ret == 0)
        {
            read_cnt = recv(sock->m_server_sock_, r_buffer, sizeof(r_buffer), 0);
            if(read_cnt < 0)
            {
                *r_buffer = 'a';
                chk_connection = write(sock->m_server_sock_, r_buffer, sizeof(r_buffer));

                if(chk_connection == -1)
                {
                    sock->setRcvTimeout(true);
                    std::cout << "server disconnected" << std::endl;
                }
            }
        }
        else // ret > 0
        {
            sock->setRcvError(false);

            read_cnt = sock->clientRead(r_buffer, sizeof(r_buffer));            
            if(read_cnt > 0)
            {      
                sock->setRcvTimeout(false);
                // Search packet size
                if(packet_size == 0)
                {
                    for(uint16_t i = 0; i < read_cnt; i++)
                    {
                        if(r_buffer[i] == 0x02)
                        {
                            for(int j = 0; j < 4; j++)
                            {
                                cp_temp[j] = r_buffer[i+1+j];
                            }
                            packet_size = strtoul((const char*) cp_temp, NULL, 16);
                            break;
                        }
                    }
                }

                // Search ETX
                for(uint16_t i = 0; i < read_cnt - 1; i++)
                {
                    if(r_buffer[read_cnt - 1 - i] == 0x03)
                    {
                        read_cnt -= i;
                        break;
                    }
                }
                
                if(packet_size > total_byte)
                {
                    memcpy(buffer + total_byte, r_buffer + offset, read_cnt - offset);
                    total_byte += read_cnt - offset;
                }   
                else if(packet_size < total_byte)
                {
                    if(buffer[0] == 0x02 && buffer[packet_size] == 0x03)
                    {
                        total_byte = packet_size; 
                    }
                    else
                    {
                        memset(buffer, 0x00, sizeof(buffer));
                        packet_size = 0;
                        total_byte = 0;
                        offset = 0;
                    }
                }
                else
                {

                }

                if(packet_size !=0 && total_byte != 0 && packet_size == total_byte)
                {
                    if(buffer[total_byte - 1] == 0x03)
                    {
                        // push in the queue
                        sock->putBufToMsg(buffer, total_byte);
                        
                        memset(buffer, 0x00, sizeof(buffer));
                        packet_size = 0;
                        total_byte = 0;
                    }
                }
                else
                {

                }                
                memset(r_buffer, 0x00, sizeof(r_buffer));
            }
            else
            {
                sock->setRcvTimeout(true);
                std::cout << "server response timeout" << std::endl;
            }
        }
    }

    return 0;
}


int Socket::clientOpen(std::string addr, std::string port)
{
    int m_server_addr_size = 0;
    uint16_t port_num = 0;

    timeval tv;
    
    
    addr_st_ = addr;
    port_num_st_ = port;
    
    m_server_sock_ = socket(PF_INET, SOCK_STREAM, 0);    

    if(m_server_sock_ < 0)
    {
        std::cout << "faild to create client socket" << std::endl;
        return -1;
    }

    tv.tv_sec = 0;;
    tv.tv_usec = 500*1000;

    if(setsockopt(m_server_sock_, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv)) < 0)
    {
        std::cout << "failed rcvtimeo setsockopt" << std::endl;
    }
    if(setsockopt(m_server_sock_, SOL_SOCKET, SO_SNDTIMEO, (char*)&tv, sizeof(tv)) < 0)
    {
        std::cout << "failed sndtimeo setsockopt" << std::endl;
    }

    sscanf(port.c_str(), "%u", (unsigned int*)&port_num);

    memset(&m_server_addr_, 0x00, sizeof(m_server_addr_));

    m_server_addr_.sin_family = AF_INET;
    m_server_addr_.sin_addr.s_addr = inet_addr(addr.begin().base());
    m_server_addr_.sin_port = htons(port_num);

    m_server_addr_size = sizeof(m_server_addr_);
    
    if(connect(m_server_sock_, (const sockaddr*)&m_server_addr_, m_server_addr_size) < 0)
    {
        std::cout << "failed client connect, " << strerror(errno) << std::endl;
        return -1;
    }
    else
    {
        m_connected_ = true;
    }

    if(pthread_create(&pthrd_id_, NULL, readCallback, this) < 0)
    {
        std::cout << "pthread create failed" << std::endl;
        return -1;
    }
    else
    {
        pthread_running_ = true;
    }    

    return 0;
}

int32_t Socket::clientRead(unsigned char* buffer, uint16_t buf_size)
{
    int32_t read_size = 0;

    if(m_connected_ == true)
    {
       read_size = recv(m_server_sock_, buffer, buf_size, 0);

       if(read_size < 0)
        {
            std::cout << "client read failed" << std::endl;
        }
        
        return read_size;
    }
    else
    {
        std::cout << "server is disconnected" << std::endl;
        return -1;
    }
}

int32_t Socket::clientWrite(unsigned char* buffer, uint16_t buf_size)
{
    int32_t write_size = 0;

    if(m_connected_ == true)
    {
        write_size = send(m_server_sock_, buffer, buf_size, 0);
        if(write_size < 0)
        {
            std::cout << "client write failed" << std::endl;

            try
            {

            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
        }
        return write_size;
    }
    else
    {
        std::cout << "server is disconnected" << std::endl;
        return -1;
    }

    
}

