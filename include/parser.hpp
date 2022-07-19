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


#ifndef PARSER_H
#define PARSER_H

#include <sensor_msgs/LaserScan.h>

//#define PRINT_PARSE_DATA

#define PI  3.14159265358979323846
#define DEG2RAD PI / 180

struct ScanInfo
{
    uint16_t fw_ver;            
    std::string model_name;
};

struct ScanMea
{
    uint16_t scan_counter;
    uint16_t scan_freq;
    uint16_t meas_freq;
    int32_t angle_begin;
    uint16_t angle_resol;
    uint16_t amnt_of_data;
    uint16_t active_field_num;
};

struct Lsc_t
{
    ScanMea scan_mea;
    ScanInfo scan_info;
};

class Parser
{
public:
    Parser();

    void parsingMsg(std::vector<unsigned char> raw_msg, sensor_msgs::LaserScan::Ptr msg, Lsc_t* lsc);
    int makeCommand(unsigned char* buf, std::string cmd);
};

#endif