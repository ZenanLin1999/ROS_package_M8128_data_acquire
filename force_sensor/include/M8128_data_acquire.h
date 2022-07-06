#ifndef _M8128_DATA_AC_
#define _M8128_DATA_AC_

// 头文件
#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include "std_msgs/UInt16.h"
#include "force_sensor/Six_Axis_Force_sensor.h"

using namespace std;

#define FRAME_HEADER      0X55  //帧头，和下位机一致
#define FRAME_TAIL        0XAA  //帧尾
#define RECEIVE_DATA_SIZE	31  //下位机发过来的数据的长度(2+2+24+2+1=31)

/*
要点提示:
1. float和unsigned long具有相同的数据结构长度
2. union据类型里的数据存放在相同的物理空间
*/
typedef union
{
	float fdata;
	unsigned long ldata;
}FloatLongType;


/*
将浮点数f转化为4个字节数据存放在byte[4]中
*/
void Float_to_Byte(float f,unsigned char byte[])
{
	FloatLongType fl;
	fl.fdata=f;
	byte[0]=(unsigned char)fl.ldata;
	byte[1]=(unsigned char)(fl.ldata>>8);
	byte[2]=(unsigned char)(fl.ldata>>16);
	byte[3]=(unsigned char)(fl.ldata>>24);
}


/*
将4个字节数据byte[4]转化为浮点数存放在*f中
*/
void Byte_to_Float(float *f,unsigned char byte[])
{
	FloatLongType fl;
	fl.ldata=0;
	fl.ldata=byte[3];
	fl.ldata=(fl.ldata<<8)|byte[2];
	fl.ldata=(fl.ldata<<8)|byte[1];
	fl.ldata=(fl.ldata<<8)|byte[0];
	*f=fl.fdata;
}


// 6轴力传感器结构体
typedef struct __Force_Data_
{
    unsigned int frame_number;
    float X_force;
    float Y_force;
    float Z_force;
    float X_torque;
    float Y_torque;
    float Z_torque;
}Force_Data;


// 接收的原始数据
typedef struct _M8128_Receive_Data_
{
    uint8_t rx[RECEIVE_DATA_SIZE];
    // uint8_t Flag_Stop;
    unsigned char Frame_Header; //1个字节 帧头
    
    uint16_t frame_number;
    float X_force;
    float Y_force;
    float Z_force;
    float X_torque;
    float Y_torque;
    float Z_torque;
    
    unsigned char Frame_Tail; //1个字节  帧尾 校验位

}M8128_Receive_Data;


// 使用构造函数初始化数据和发布话题等
class turn_on_sensor
{
	public:
		turn_on_sensor(); //构造函数
		~turn_on_sensor(); //析构函数
		void sensor_run();//循环控制代码
		serial::Serial M8128_Serial; //声明串口对象 
	
    private:
		/* Read/Write data from ttyUSB 串口和控制函数声明 */
        std::string start_cmd = "AT+GSD\r\n";
        std::string sample_freq = "AT+SMPF=100\r\n";
        std::string end_cmd = "AT+GOD\r\n";
        bool Get_Sensor_Data();
        unsigned char Check_Sum();//校验函数
        Force_Data M8128_data;// M8128采集卡数据
        void M8128_data_publisher_fun(); // 发布M8128采集卡解析数据topic
        int serial_baud_rate;//波特率
		string usart_port_name, robot_frame_id, smoother_cmd_vel;
		ros::NodeHandle n;//创建句柄
		ros::Time _Now, _Last_Time;//时间相关
		float Sampling_Time; //采样时间
        // ros::Publisher M8128_data_publisher_1, M8128_data_publisher_2, M8128_data_publisher_3, \    
        //                M8128_data_publisher_4, M8128_data_publisher_5, M8128_data_publisher_6, \
        //                M8128_data_publisher_7;//初始化发布者
        ros::Publisher M8128_data_publisher;
        M8128_Receive_Data Receive_Data;  //接收结构体  
};


#endif
