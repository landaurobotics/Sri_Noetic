#include <cstring>
#include <iostream>
#include <sri/ftsensor.hpp>
#include <sri/commethernet.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
using namespace SRI;
ros::Publisher wrench_pub;
ros::Time last_publish_time = ros::Time(0);      // Global variable to control publishing frequency
double global_publish_frequency = 0.01;          // Global variable to record the time of the last publication

/**
 * @brief Handles the real-time data and publishes the wrench message.
 * 
 * This function takes a vector of RTData objects and converts them into a geometry_msgs::Wrench message.
 * It limits the publishing frequency based on the global_publish_frequency parameter.
 * The wrench message is published only if the time difference between the current time and the last publish time
 * is greater than or equal to the threshold duration.
 * 
 * @param rtData The vector of RTData objects containing real-time data.
 */
void rtDataHandler(std::vector<RTData<float>> &rtData)
{
    geometry_msgs::Wrench wrench_msg;
    ros::Duration threshold_duration(global_publish_frequency); // Convert publishing frequency to ros::duration object

    ros::Time current_time = ros::Time::now();
    if (current_time - last_publish_time >= threshold_duration) // Limit publishing frequency
    {

        // The following loop is used to traverse the input real-time data, each data contains 6 channels of data
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
                        case 0: // Assign rtData[i][j] to the force component of wrench msg
                            wrench_msg.force.x = rtData[i][j];
                            break;
                        case 1:
                            wrench_msg.force.y = rtData[i][j];
                            break;
                        case 2:
                            wrench_msg.force.z = rtData[i][j];
                            break;
                        case 3: // Assign rtData[i][j] to the torque component of wrench msg
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

                    // Publish wrench msg
                    std::cout<<"force_x:"<<wrench_msg.force.x<<"force_y:"<<wrench_msg.force.y<<"force_z:"<<wrench_msg.force.z<<std::endl;
                    std::cout<<"torque_x:"<<wrench_msg.torque.x<<"torque_y:"<<wrench_msg.torque.y<<"torque_z:"<<wrench_msg.torque.z<<std::endl;
                    wrench_pub.publish(wrench_msg);
                }
            }
        }
        last_publish_time = current_time;
    }
}
int main(int argc, char  *argv[])
{
    std::string ip_address = "192.168.2.109"; // Global IP address ， default value is "192.168.2.109".
    int port_number = 4008;                   // Global port number, default is 4008

    ros::init(argc, argv, "Sri_data_publisher");                                         
    ros::NodeHandle nh;                                                                  
    wrench_pub = nh.advertise<geometry_msgs::Wrench>("wrench_topic", 10); 
    // Get parameters from ros parameter master
    if (ros::param::has("~ip_address"))
    {
        if (!ros::param::get("~ip_address", ip_address))
        {
            ROS_WARN_STREAM("Failed to parse ip_address parameter, using default value: " << ip_address);
        }
    }
    else
    {
        ROS_WARN_STREAM("No ip_address parameter found, using default value: " << ip_address);
    }
    if (ros::param::has("~port"))
    {
        if (!ros::param::get("~port", port_number))
        {
            ROS_WARN_STREAM("Failed to parse port parameter, using default value: " << port_number);
        }
    }
    else
    {
        ROS_WARN_STREAM("No port parameter found, using default value: " << port_number);
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

    // FTsensor initialization
    SRI::CommEthernet *ce = new SRI::CommEthernet(ip_address, port_number);
    SRI::FTSensor sensor(ce);
    double x, y, z, mx, my, mz;
    auto rtDataValid = sensor.getRealTimeDataValid();
    auto rtMode = sensor.getRealTimeDataMode();

    sensor.startRealTimeDataRepeatedly<float>(&rtDataHandler, rtMode, rtDataValid);
    ros::spin(); 
    return 0;
}
