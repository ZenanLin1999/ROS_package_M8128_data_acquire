#include "M8128_data_acquire.h"

/**************************************
Function: 主函数，ROS初始化
***************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "M8128_data_acquire");//ROS初始化 并设置节点名称，可修改
  ROS_INFO("M8128_data_acquire node has turned on ");//显示状态
  turn_on_sensor M4341A_force_sensor; //实例化一个对象
  M4341A_force_sensor.sensor_run();  //循环执行数据采集和发布topic等操作
  return 0;
} 


/**************************************
Function: 代码循环执行
***************************************/
void turn_on_sensor::sensor_run()
{
  _Last_Time = ros::Time::now();
  while(ros::ok())
  {
    _Now = ros::Time::now();
    //Sampling_time是采样时间，虽然下位机发送的数据频率是固定的，这里以ROS系统的时间更加可靠精确。
    if (true == Get_Sensor_Data())  //从串口读取下位机法过来的全部数据
    {
      Sampling_Time = (_Now - _Last_Time).toSec();
      M8128_data_publisher_fun(); // 发布M8128采集卡解析数据topic
      _Last_Time = _Now;//记录时间
    }
    ros::spinOnce();//循环等待回调函数
  }
}

/**************************************
Function: 从串口读取数据
读取6轴力传感器数据并解析（一个数据包有2+2+2+24+1=31个byte)
example: 55 00 1B D1 EB 41 A3 A4 3E CA D1 B4 3F 36 06 12 BE 6A 6F 83 3B 24 02 56 BB 00 EA 3C 37 8B AA 
帧头-1：0x55 
数量-2：0x00 0x1B(27-byte不含帧头和数量) 
数据包index-2: 0xD1 0xEB 
数据位-24：0x41 0xA3 0xA4 0x3E 0xCA 0xD1 0xB4 0x3F 0x36 0x06 0x12 0xBE 0x6A 0x6F 0x83 0x3B 0x24 0x02 0x56 0xBB 0x00 0xEA 0x3C 0x37 
sum校验(仅仅对数据位求和校验)-1： 0x8B
帧尾-1：0xAA 
***************************************/
bool turn_on_sensor::Get_Sensor_Data()
{ 
    short transition_16=0,j=0,Header_Pos=0,Tail_Pos=0;  //中间变量
    uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE]={0};
    unsigned char byte[4]={0};
    M8128_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr));//读串口数据

    // step1: 找到帧头和尾
    for(j=0;j<31;j++)
    {
        if(Receive_Data_Pr[j]==FRAME_HEADER)
            Header_Pos=j;
        else if(Receive_Data_Pr[j]==FRAME_TAIL)
            Tail_Pos=j;    
    }
    //ROS_INFO("%x-%x",Header_Pos,Tail_Pos);

    // step2: 根据不同情况写入数据帧原始数据
    if(Tail_Pos==(Header_Pos+30))// 情况1-帧头和尾的间隔30
    {
        //ROS_INFO("1----");
        memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }
    else if(Header_Pos==(1+Tail_Pos))// 情况2-帧头和尾相邻
    {
        //ROS_INFO("2----");
        for(j=0;j<31;j++)
          Receive_Data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%31];
    }
    else // 情况3-读出出错
    {
        //ROS_INFO("3----");
        return false;
    }    
    
    // 取出数据帧原始数据的头和尾
    Receive_Data.Frame_Header = Receive_Data.rx[0]; //数据的第一位是帧头（固定值）
    Receive_Data.Frame_Tail = Receive_Data.rx[30];  //数据的最后一位是帧尾（数据校验位）

    // 判断数据可靠性
    if (Receive_Data.Frame_Header == FRAME_HEADER ) //判断帧头
    {
        if (Receive_Data.Frame_Tail == FRAME_TAIL) //判断帧尾
        { 
            if (Receive_Data.rx[29] == Check_Sum())//校验位检测
            { 
                // 数据无误可以保存
                M8128_data.frame_number =  Receive_Data.rx[3]*256 + Receive_Data.rx[4]; // frame number

                // 六轴数据
                byte[0] = Receive_Data.rx[5];
                byte[1] = Receive_Data.rx[6];
                byte[2] = Receive_Data.rx[7];
                byte[3] = Receive_Data.rx[8];
                Byte_to_Float(&(M8128_data.X_force), byte);

                byte[0] = Receive_Data.rx[9];
                byte[1] = Receive_Data.rx[10];
                byte[2] = Receive_Data.rx[11];
                byte[3] = Receive_Data.rx[12];
                Byte_to_Float(&(M8128_data.Y_force), byte);

                byte[0] = Receive_Data.rx[13];
                byte[1] = Receive_Data.rx[14];
                byte[2] = Receive_Data.rx[15];
                byte[3] = Receive_Data.rx[16];
                Byte_to_Float(&(M8128_data.Z_force), byte);

                byte[0] = Receive_Data.rx[17];
                byte[1] = Receive_Data.rx[18];
                byte[2] = Receive_Data.rx[19];
                byte[3] = Receive_Data.rx[20];
                Byte_to_Float(&(M8128_data.X_torque), byte);

                byte[0] = Receive_Data.rx[21];
                byte[1] = Receive_Data.rx[22];
                byte[2] = Receive_Data.rx[23];
                byte[3] = Receive_Data.rx[24];
                Byte_to_Float(&(M8128_data.Y_torque), byte);

                byte[0] = Receive_Data.rx[25];
                byte[1] = Receive_Data.rx[26];
                byte[2] = Receive_Data.rx[27];
                byte[3] = Receive_Data.rx[28];
                Byte_to_Float(&(M8128_data.Z_torque), byte);
                
                return true;
            }
        }
    } 
    return false;
}

/**************************************
Function: 串口通讯求和校验函数
求和范围：[5, 28]
***************************************/
unsigned char turn_on_sensor::Check_Sum()
{
    unsigned char check_sum = 0, k;
    unsigned char Count_Number = 24;
  
    for(k=0;k<Count_Number;k++) // Count_Number
    {
        check_sum = check_sum + Receive_Data.rx[k + 5];
    }
    // debug
    ROS_INFO("check_sum = %d", check_sum);
    return check_sum;//返回结果
}


/**************************************
Function: 发布六轴传感器相关信息
***************************************/
void turn_on_sensor::M8128_data_publisher_fun()
{
    // // old-version
    // std_msgs::Float32 sensor_msgs;//定义发布topic的数据类型std_msgs::Float32
    // std_msgs::UInt16 sensor_frame_num;//定义发布topic的数据类型std_msgs::UInt16

    // // 发布传感器数据
    // sensor_msgs.data = M8128_data.X_force;//获取值
    // M8128_data_publisher_1.publish(sensor_msgs);//发布话题
    // sensor_msgs.data = M8128_data.Y_force;//获取值
    // M8128_data_publisher_2.publish(sensor_msgs);//发布话题
    // sensor_msgs.data = M8128_data.Z_force;//获取值
    // M8128_data_publisher_3.publish(sensor_msgs);//发布话题
    // sensor_msgs.data = M8128_data.X_torque;//获取值
    // M8128_data_publisher_4.publish(sensor_msgs);//发布话题
    // sensor_msgs.data = M8128_data.X_torque;//获取值
    // M8128_data_publisher_5.publish(sensor_msgs);//发布话题
    // sensor_msgs.data = M8128_data.X_torque;//获取值
    // M8128_data_publisher_6.publish(sensor_msgs);//发布话题
    // sensor_frame_num.data = M8128_data.frame_number;//获取值
    // M8128_data_publisher_7.publish(sensor_frame_num);//发布话题

    // new version
    force_sensor::Six_Axis_Force_sensor sensor_msgs;//定义发布topic的数据类型
    sensor_msgs.X_force = M8128_data.X_force;
    sensor_msgs.Y_force = M8128_data.Y_force;
    sensor_msgs.Z_force = M8128_data.Z_force;
    sensor_msgs.X_torque = M8128_data.X_torque;
    sensor_msgs.Y_torque = M8128_data.Y_torque;
    sensor_msgs.Z_torque = M8128_data.Z_torque;
    sensor_msgs.Sampling_Time = Sampling_Time;
    sensor_msgs.frame_number = M8128_data.frame_number;

    M8128_data_publisher.publish(sensor_msgs);//发布话题

}

/**************************************
Function: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_sensor::turn_on_sensor():Sampling_Time(0)
{
  memset(&Receive_Data, 0, sizeof(Receive_Data)); //构造函数初始化
  ros::NodeHandle private_nh("~");
  //把以上的类成员参数注册到参数服务器，这样在launch文件里面即可修改
  //3个入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  private_nh.param<std::string>("usart_port_name", usart_port_name, "/dev/ttyUSB0"); //固定串口
  private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200); //和下位机底层波特率115200
  
  // //发布7个话题
  // M8128_data_publisher_1 = n.advertise<std_msgs::Float32>("/Force_X", 10);//传感器数据发布
  // M8128_data_publisher_2 = n.advertise<std_msgs::Float32>("/Force_Y", 10);//传感器数据发布
  // M8128_data_publisher_3 = n.advertise<std_msgs::Float32>("/Force_Z", 10);//传感器数据发布
  // M8128_data_publisher_4 = n.advertise<std_msgs::Float32>("/Torque_X", 10);//传感器数据发布
  // M8128_data_publisher_5 = n.advertise<std_msgs::Float32>("/Torque_Y", 10);//传感器数据发布
  // M8128_data_publisher_6 = n.advertise<std_msgs::Float32>("/Torque_Z", 10);//传感器数据发布
  // M8128_data_publisher_7 = n.advertise<std_msgs::UInt16>("/Frame_num", 10);//传感器数据发布

  // 发布话题
  M8128_data_publisher = n.advertise<force_sensor::Six_Axis_Force_sensor>("/M4341A_Force_Sensor", 10);//传感器数据发布

  ROS_INFO_STREAM("Data ready");//ready显示状态
  
  //初始化串口
  try{
         M8128_Serial.setPort(usart_port_name);//选择哪个口，如果选择的口没有接串口外设初始化会失败
         M8128_Serial.setBaudrate(serial_baud_rate);//设置波特率
         serial::Timeout _time = serial::Timeout::simpleTimeout(2000);//超时等待
         M8128_Serial.setTimeout(_time);
         M8128_Serial.open();//串口开启
    }
  catch (serial::IOException& e){
     ROS_ERROR_STREAM("Force sensor can not open serial port,Please check the serial port cable! ");//如果try失败，打印错误信息
  }
  if(M8128_Serial.isOpen()){
    ROS_INFO_STREAM("Force sensor serial port opened");//开启成功

    // 发送指令"AT+GSD\r\n"开启数据传输过程 
    M8128_Serial.write(sample_freq); // 发送采样率设置（可在.h文件修改）
    M8128_Serial.write(start_cmd); //向串口发数据

  }else{
  }
}


/**************************************
Function: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_sensor::~turn_on_sensor()
{
  // 发送指令"AT+GOD\r\n"开启数据传输过程 
  M8128_Serial.write(end_cmd); //向串口发数据
  ROS_INFO_STREAM("turn_on_sensor deconstruction fun!");//close
}


