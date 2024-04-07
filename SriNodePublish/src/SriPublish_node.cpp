#include <cstring>
#include <iostream>
#include <sri/ftsensor.hpp>
#include <sri/commethernet.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
using namespace SRI;
ros::Publisher wrench_pub;
ros::Time last_publish_time = ros::Time(0);      // 上一次发布的时间
double global_publish_frequency = 0.01;          // 全局发布频率，默认为0.1秒
std::string global_ip_address = "192.168.2.109"; // 全局IP地址，默认为"192.168.2.109"
int global_port_number = 4008;                   // 全局端口号，默认为4008
/**
 * 处理实时数据的函数
 *
 * @param rtData 引用传递，一个包含RTData类型的浮点数向量。每个RTData元素代表一个时间点的6个通道的数据。
 *
 * 该函数主要功能是遍历输入的实时数据向量，并对每个数据点的6个通道进行处理。
 * 但实际对数据的处理逻辑（如赋值或计算）未实现。
 */
void rtDataHandler(std::vector<RTData<float>> &rtData)
{
    geometry_msgs::Wrench wrench_msg;
    ros::Duration threshold_duration(global_publish_frequency); // 将发布频率转换为ros::Duration对象

    ros::Time current_time = ros::Time::now();
    if (current_time - last_publish_time >= threshold_duration) // 限制发布频率
    {

        // 下面的循环用于遍历输入的实时数据，每个数据包含6个通道的数据
        for (int i = 0; i < rtData.size(); i++)
        {
            for (int j = 0; j < 6; j++)
            {

                for (size_t i = 0; i < rtData.size(); ++i)
                {
                    for (int j = 0; j < 6; ++j)
                    {
                        switch (j)
                        {
                        case 0: // 将rtData[i][j]赋值给wrench_msg的力分量
                            wrench_msg.force.x = rtData[i][j];
                            break;
                        case 1:
                            wrench_msg.force.y = rtData[i][j];
                            break;
                        case 2:
                            wrench_msg.force.z = rtData[i][j];
                            break;
                        case 3: // 将rtData[i][j]赋值给wrench_msg的扭矩分量
                            wrench_msg.torque.x = rtData[i][j];
                            break;
                        case 4:
                            wrench_msg.torque.y = rtData[i][j];
                            break;
                        case 5:
                            wrench_msg.torque.z = rtData[i][j];
                            break;
                        default:
                            ROS_WARN_STREAM("Invalid channel index: " << j);
                            break;
                        }
                    }

                    // 发布wrench_msg
                    wrench_pub.publish(wrench_msg);
                }
            }
        }
        last_publish_time = current_time;
    }
}
int main(int argc, char  *argv[])
{

    ros::init(argc, argv, "Sri_data_publisher");                                         // 初始化ROS节点
    ros::NodeHandle nh;                                                                  // 创建NodeHandle对象以管理节点
    ros::Publisher wrench_pub = nh.advertise<geometry_msgs::Wrench>("wrench_topic", 10); // 创建publisher，发布至"wrench_topic"，队列大小为10
    // 从参数服务器获取参数
    if (ros::param::has("~ip_address"))
    {
        if (!ros::param::get("~ip_address", global_ip_address))
        {
            ROS_WARN_STREAM("Failed to parse ip_address parameter, using default value: " << global_ip_address);
        }
    }
    else
    {
        ROS_WARN_STREAM("No ip_address parameter found, using default value: " << global_ip_address);
    }
    if (ros::param::has("~port"))
    {
        if (!ros::param::get("~port", global_port_number))
        {
            ROS_WARN_STREAM("Failed to parse port parameter, using default value: " << global_port_number);
        }
    }
    else
    {
        ROS_WARN_STREAM("No port parameter found, using default value: " << global_port_number);
    }
    if (ros::param::has("~publish_frequency"))
    {
        if (!ros::param::get("~publish_frequency", global_publish_frequency))
        {
            ROS_WARN_STREAM("Failed to parse publish_frequency parameter, using default value: " << global_publish_frequency);
        }
    }
    else
    {
        ROS_WARN_STREAM("No publish_frequency parameter found, using default value: " << global_publish_frequency);
    }

    // 传感器初始化
    SRI::CommEthernet *ce = new SRI::CommEthernet(global_ip_address, global_port_number);
    SRI::FTSensor sensor(ce);
    double x, y, z, mx, my, mz;
    auto rtDataValid = sensor.getRealTimeDataValid();
    auto rtMode = sensor.getRealTimeDataMode();

    sensor.startRealTimeDataRepeatedly<float>(&rtDataHandler, rtMode, rtDataValid);
    std::cout << "Hello, World!" << std::endl;
    ros::spin(); // 进入循环，等待回调函数
    return 0;
}
