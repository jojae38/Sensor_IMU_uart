#ifndef SENSOR_IMU_UART
#define SENSOR_IMU_UART

#include <iostream>
#include <mraa.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>
#include <regex>
using namespace std;

enum IMU_BAUD_RATE
{
    sb1 = 9600,
    sb2 = 19200,
    sb3 = 38400,
    sb4 = 57600,
    sb5 = 115200,
    sb6 = 230400,
    sb7 = 460800,
    sb8 = 921600,
};
class SensorIMU_uart
{
    private:

    ros::NodeHandle _nh;
    ros::Publisher _pub_imu_cmd;
    ros::Publisher _pub_imu_data;
    ros::Subscriber _sub;
    ros::Publisher _pub_cmd;

    int _ok_count=0;
    int _uart_num,_baud_rate, _output_rate,_output_format;
    int _output_gyro,_output_accelero;

    std::string _regex_pattern;
    mraa::Uart *uart;
    int _ros_rate;

    std::string _output_ok="<ok>";

    std_msgs::String _msg_start_bit;
    std_msgs::String _msg_error;

    sensor_msgs::Imu _imu_msg;

    void init_param();
    bool init_uart(mraa::Uart &uart);
    void callback_writeToSerial(const std_msgs::String::ConstPtr &msg);
    void imu_data_publisher(std::string serial_data);


    public:
        SensorIMU_uart();
        ~SensorIMU_uart();

};

SensorIMU_uart::SensorIMU_uart()
{
    init_param();
    
    mraa::Uart temp(_uart_num);
    ROS_INFO("dd");
    *uart=temp;
    init_uart(*uart);
    _pub_imu_cmd = _nh.advertise<std_msgs::String>("/imu_cmd_read", 1);
    _pub_imu_data = _nh.advertise<sensor_msgs::Imu>("/imu_read", 1);
    // _sub = _nh.subscribe("/imu_write", 1, &SensorIMU_uart::callback_writeToSerial, this);
    _pub_cmd = _nh.advertise<std_msgs::String>("/imu_write", 1);
    _msg_start_bit.data = "*";
}
SensorIMU_uart::~SensorIMU_uart()
{

}
void SensorIMU_uart::init_param()
{
    ros::NodeHandle _nh("~");
    std::string regex_pattern = "(([-+]?\\d*\\.\\d+)|\\d+)";
    _nh.param("UART_NUM", _uart_num, 0);
    _nh.param("SET_BAUDRATE", _baud_rate, 5);
    _nh.param("SET_OUTPUT_RATE", _output_rate, 10);
    _nh.param("SET_OUTPUT_FORMAT", _output_format, 1);
    _nh.param("SET_OUTPUT_GYRO", _output_gyro, 0);
    _nh.param("SET_OUTPUT_ACCELERO", _output_accelero, 0);
    // _nh.param("SET_OUTPUT_DISTANCE", _output_distance, 0);
    // _nh.param("SET_SENS_GYRO", _sens_gyro, 5);
    // _nh.param("SET_SENS_ACCELERO", _sens_accelero, 3);
    // _nh.param("SET_Low_Pass_Filter_Gyroscope", _low_pass_filter_gyro, 3);
    // _nh.param("SET_Low_Pass_Filter_Accelerometer", _low_pass_filter_accelero, 3);
    // _nh.param("POWER_ON_START", _power_on_start, 1);
    _nh.param("ros_rate", _ros_rate, 10);
    _nh.param("regex_pattern", _regex_pattern, regex_pattern);
    _msg_error.data = "::: <err> occurred ::: check sensor_imu.launch & parameters";
}
bool SensorIMU_uart::init_uart(mraa::Uart &uart)
{
    uart.setMode(8,mraa::UART_PARITY_NONE,1);
    uart.setBaudRate(115200);
    uart.setFlowcontrol(false,false);
    uart.setTimeout(10,10,10);

    char data[10];
    memset(data,0,sizeof(data));

    std::string input1="<sb"+std::to_string(_baud_rate)+">";
    uart.write(input1.c_str(),input1.size());
    uart.read(data,sizeof(data));
    if(strcmp(_output_ok.c_str(),data)!=0)
        return false;
    std::cout << data<< std::endl;

    std::string input2="<sor"+std::to_string(_output_rate)+">";
    uart.write(input2.c_str(),input2.size());
    uart.read(data,sizeof(data));
    if(strcmp(_output_ok.c_str(),data)!=0)
        return false;
    std::cout << data<<std::endl;

    std::string input3="<sof"+std::to_string(_output_format)+">";
    uart.write(input3.c_str(),input3.size());
    uart.read(data,sizeof(data));
    if(strcmp(_output_ok.c_str(),data)!=0)
        return false;
    std::cout << data<<std::endl;

    std::string input4="<sog"+std::to_string(_output_gyro)+">";
    uart.write(input4.c_str(),input4.size());
    uart.read(data,sizeof(data));
    if(strcmp(_output_ok.c_str(),data)!=0)
        return false;
    std::cout << data<<std::endl;

    std::string input5="<soa"+std::to_string(_output_accelero)+">";
    uart.write(input5.c_str(),input5.size());
    uart.read(data,sizeof(data));
    if(strcmp(_output_ok.c_str(),data)!=0)
        return false;
    std::cout << data<<std::endl;

    return true;
}

void SensorIMU_uart::callback_writeToSerial(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    uart->write(msg->data.c_str(),sizeof(msg->data));
}

void SensorIMU_uart::imu_data_publisher(std::string serial_data)
{
    std::regex reg{ _regex_pattern };
    std::sregex_token_iterator iter(serial_data.begin(), serial_data.end(), reg), end;
    std::vector<std::string> result = std::vector<std::string>(iter, end);
    if (!result.empty())
    {
        _imu_msg.header.frame_id = "/imu_read";
        _imu_msg.header.stamp = ros::Time::now();

        _imu_msg.orientation.z = std::stof(result[0]);
        _imu_msg.orientation.y = std::stof(result[1]);
        _imu_msg.orientation.x = std::stof(result[2]);
        _imu_msg.orientation.w = std::stof(result[3]);

        _imu_msg.angular_velocity.x = std::stof(result[4]);
        _imu_msg.angular_velocity.y = std::stof(result[5]);
        _imu_msg.angular_velocity.z = std::stof(result[6]);

        _imu_msg.linear_acceleration.x = std::stof(result[7]);
        _imu_msg.linear_acceleration.y = std::stof(result[8]);
        _imu_msg.linear_acceleration.z = std::stof(result[9]);

        _pub_imu_data.publish(_imu_msg);
    }
}

#endif