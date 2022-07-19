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


#include <queue>
#include "parser.hpp"

#define NUM_OF_CMD_TYPE 4
#define NUM_OF_CMD 7

#define NUMBER_OF_PARAM 13
#define MAX_NUM_OF_PACKET_PARAM 16

#define INTENSITY_MAX 50000.0


const char* cmd_type_list[NUM_OF_CMD_TYPE] = 
{"sMC",
"sMA",
"sRC",
"sRA",
};

const char* cmd_list[NUM_OF_CMD] = 
{"None",
"SetAccessLevel",
"SensorScanInfo",
"SensorStart",
"SensorStop",
"ScanData",
"FirstConnectDummySend"
};

enum CmdListNum 
{
    NONE = 0,
    SET_ACCESS_LEVEL,
    SENSOR_SCAN_INFO,
    SENSOR_START,
    SENSOR_STOP,
    SCAN_DATA,
    FIRST_CONNECT_DUMMY_SEND
};

Parser::Parser()
{

}

int Parser::makeCommand(unsigned char* buf, std::string cmd)
{
    std::vector<unsigned char> v_cmd_frame;
    int cnt = 0, cmd_num = 0;
    std::string target = " ";
    std::string rep = ",";
    std::string temp_str;

    // replacing " " to ","
    for(int i = 0; cmd.find(target) != std::string::npos && i < MAX_NUM_OF_PACKET_PARAM; i++)
    {
        cmd.replace(cmd.find(target), target.length(), rep);
    }

    temp_str = cmd;
    char* ptr_tok_cmd = strtok(&temp_str[0], ",");

    // searching matched cmd
    for(int i = 0; i < NUM_OF_CMD; i++)
    {
        if(strcmp(cmd_list[i], ptr_tok_cmd) == 0)
        {
            cmd_num = i;
            break;  
        }
    }

    v_cmd_frame.push_back(',');

    switch(cmd_num)
    {
        case SET_ACCESS_LEVEL :
        case SENSOR_START :
        case SENSOR_STOP :
            v_cmd_frame.push_back('s');
            v_cmd_frame.push_back('M');
            v_cmd_frame.push_back('C');
            break;
        
        case SENSOR_SCAN_INFO :
            v_cmd_frame.push_back('s');
            v_cmd_frame.push_back('R');
            v_cmd_frame.push_back('C');
            break;

        case NONE :
        default :
            std::cout << "invalid command, type correct command " << cmd_num << std::endl;
            return -1; 
            break;
    }
    v_cmd_frame.push_back(',');

    for(int i = 0; i < cmd.size() + 1; i++)
    {
        v_cmd_frame.push_back(cmd[i]);
    }

    memcpy(buf, &v_cmd_frame[0], v_cmd_frame.size());

    return v_cmd_frame.size() - 1;
}

void Parser::parsingMsg(std::vector<unsigned char> raw_msg, sensor_msgs::LaserScan::Ptr msg, Lsc_t* lsc)
{
    uint16_t msg_size = 0, index= 0, field_size = 0, index_base = 0;
    int cmd_type = 0, cmd = 0, version = 0, status = 0;
    float chk_intensity = 0;
    std::vector<char *> field;
    char* ptr;
    
    if(raw_msg[0] == 0x02)
    {
        ptr = strtok((char*) &raw_msg[1], ",");

        if(raw_msg[strtoul(ptr,NULL, 16) - 1] == 0x03)
        {
            raw_msg[strtoul(ptr, NULL, 16) - 1] = '\0';
        }
        else
        {
            for(int i = 0; i < raw_msg.size(); i++)
            {
                std::cout << raw_msg[i] <<std::endl;
            }
            return;
        }

        while(ptr != NULL)
        {
            field.push_back(ptr);
            ptr = strtok(NULL, ",");
        }
    
        // get packet size
        msg_size = strtoul(field[index++], NULL, 16);
        field_size = field.size();

        // get command type
        for(int i = 0; i < NUM_OF_CMD_TYPE; i++)
        {
            int temp_ret = strcmp(field[index], cmd_type_list[i]);

            if(temp_ret == 0)
            {
                cmd_type = i;
                break;
            }
        }
        index++;

        // get command
        for(int i = 0; i < NUM_OF_CMD; i++)
        {
            int temp_ret = strcmp(field[index], cmd_list[i]);

            if(temp_ret == 0)
            {
                cmd = i;
                break;
            }
        }
        index++;

        switch(cmd)
        {
            case SENSOR_SCAN_INFO :
                lsc->scan_info.fw_ver = strtoul(field[8], NULL, 16);         // fw_ver
                lsc->scan_info.model_name = field[13];                        // model name
                
                #ifdef PRINT_PARSE_DATA                
                std::cout << "Command : " << cmd_list[cmd] << std::endl;
                std::cout << "fw_ver : " << lsc->scan_info.fw_ver << std::endl;
                std::cout << "Model_name : " << lsc->scan_info.model_name << std::endl;
                #endif

                break;

            case SCAN_DATA :
                for(int i = 0; i < NUMBER_OF_PARAM; i++)
                {
                    ptr = strtok(NULL, ",");
                    field.push_back(ptr);
                }
                lsc->scan_mea.scan_counter = strtoul(field[7], NULL, 16);       // scan counter
                lsc->scan_mea.scan_freq = strtoul(field[10], NULL, 16);          // scan freq
                lsc->scan_mea.meas_freq = strtoul(field[11], NULL, 16);          // meas freq
                lsc->scan_mea.angle_begin = strtoul(field[12], NULL, 16);        // angle_begin
                lsc->scan_mea.angle_resol = strtoul(field[13], NULL, 16);        // angle_resol
                lsc->scan_mea.amnt_of_data = strtoul(field[14], NULL, 16);       // amount of data
                index = 16;

                #ifdef PRINT_PARSE_DATA                
                std::cout << "Cmd type : " << cmd_type_list[cmd_type] << std::endl;
                std::cout << "Command : " << cmd_list[cmd] << std::endl;
                std::cout << "Scan counter : " << lsc->scan_mea.scan_counter << std::endl;
                std::cout << "Scan frequency : " << std::dec << lsc->scan_mea.scan_freq << std::endl;
                std::cout << "Measuring frequency : " << lsc->scan_mea.meas_freq << std::endl;
                std::cout << "Angle begin : " << lsc->scan_mea.angle_begin << std::endl;
                std::cout << "Angle resolution : " << lsc->scan_mea.angle_resol << std::endl;
                std::cout << "Amount of data : " << lsc->scan_mea.amnt_of_data << std::endl;
                #endif

                msg->angle_min = lsc->scan_mea.angle_begin / 10000.0 * DEG2RAD;
                msg->angle_max = (lsc->scan_mea.angle_begin / 10000.0 + (lsc->scan_mea.angle_resol / 10000.0 * lsc->scan_mea.amnt_of_data)) * DEG2RAD;
                msg->angle_increment = lsc->scan_mea.angle_resol / 10000.0 * DEG2RAD;
                msg->time_increment = (60.0 / lsc->scan_mea.meas_freq) / lsc->scan_mea.scan_counter;
                msg->scan_time = (60.0 / lsc->scan_mea.meas_freq); 

                #ifdef PRINT_PARSE_DATA 
                std::cout << "" << std::endl;
                std::cout << "msg->angle_min = " << msg->angle_min << std::endl;
                std::cout << "msg->angle_max = " << msg->angle_max << std::endl;
                std::cout << "msg->angle_increment = " << msg->angle_increment << std::endl;
                std::cout << "msg->range_max = " << msg->range_max << std::endl;
                std::cout << "msg->range_min = " << msg->range_min << std::endl;
                std::cout << "msg->scan_time = " << msg->scan_time << std::endl;
                std::cout << "msg->time_increment = " << msg->time_increment << std::endl;
                #endif
                
                if(strcmp(field[index++], "DIST1") == 0)
                {
                    index_base = index;

                    msg->ranges.clear();
                    for(uint16_t i = 0; i < lsc->scan_mea.amnt_of_data; i++)
                    {
                        msg->ranges.push_back(strtoul(field[index++], NULL, 16) / 1000.0);
                        
                    #ifdef PRINT_PARSE_DATA
                        std::cout << msg->ranges[i] << ", ";
                    }
                    std::cout << "" << std::endl;
                    #else
                    }                   
                    #endif
                }

                msg->intensities.clear();
                if(field[index] == field.back())
                {   
                }
                else
                {
                    if(strcmp(field[index++], "RSSI1") == 0 )
                    {
                        index_base = index;

                        for(uint16_t i = 0; i < lsc->scan_mea.amnt_of_data; i++)
                        { 
                            chk_intensity = strtoul(field[index++], NULL, 16);                            

                            msg->intensities.push_back(chk_intensity);

                        #ifdef PRINT_PARSE_DATA
                            std::cout << std::hex << std::uppercase << msg->intensities[i] << ",";
                        }
                        std::cout << "" << std::endl;
                        #else
                        }
                        #endif
                    }
                } 
                break;

            case SENSOR_START :
            case SENSOR_STOP :
            case SET_ACCESS_LEVEL :
            case FIRST_CONNECT_DUMMY_SEND :
                for(int i = 2; i < field.size(); i++)
                {
                    std::cout << field[i] << " ";
                }
                std::cout << "" << std::endl;
                break;
                
            default :
                for(int i = 2; i < field.size(); i++)
                {
                    std::cout << field[i] << " ";
                }
                std::cout << "" << std::endl;
                break;
        }
    }
}