#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/WrenchStamped.h"
#include <std_msgs/Float64.h>
#include <string>
#include "aidin_ros/aidin_ec_wrapper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aidin_ft_sensor");
    ros::NodeHandle nh_p("~");
    ros::NodeHandle nh;

    float sample_rate;
    if (!nh_p.getParam("sample_rate", sample_rate))
    {
        sample_rate = 1000.0;  // Default value if not provided
        ROS_INFO("Using default sample rate[Hz]: %.2f", sample_rate);
    } else {
    ROS_INFO("Sample rate [Hz]: %.2f", sample_rate);
    }

    float cutoff_frequency;
    if (!nh_p.getParam("cutoff_frequency", cutoff_frequency))
    {
        cutoff_frequency = 35.0;  // Default value if not provided
        ROS_INFO("Using default cutoff frequency [Hz]: %.2f", cutoff_frequency);
    } else {
    ROS_INFO("Cutoff frequency [Hz]: %.2f", cutoff_frequency);
    }

    std::string arm_id;
    if(!nh_p.getParam("arm_id", arm_id))
    {
        arm_id = "panda";  // Default value if not provided
        ROS_INFO("Using default arm_id: %s", arm_id.c_str());
    } else {
    ROS_INFO("arm_id: %s", arm_id.c_str());
    }

    bool use_imu;
    if(!nh_p.getParam("use_imu", use_imu))
    {
        use_imu = false;  // Default value if not provided
        ROS_INFO("Using default setting for use of IMU: %s", use_imu ? "true" : "false");
    } else {
        ROS_INFO("use_imu: %s", use_imu ? "true" : "false");
    }

    std::string interface;
    if(!nh_p.getParam("interface", interface))
    {
        interface = "enp2s0";  // Default value if not provided
        ROS_INFO("Using default ethercat interface: %s", interface.c_str());
    } else {
        ROS_INFO("Using interface: %s", interface.c_str());
    }

    // ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/ft_sensor/ft_raw", 1000);
    std::string wrench_topic = "/ft_sensor/" + interface + "/ft_raw";
    std::string imu_topic = "/ft_sensor/" + interface + "/imu";
    std::string temperature_topic = "/ft_sensor/" + interface + "/temperature";
    ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>(wrench_topic, 1000);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
    ros::Publisher temperature_pub = nh.advertise<std_msgs::Float64>(temperature_topic, 10);

    ros::Rate loop_rate(sample_rate);

    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = arm_id + "_sensor";
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = arm_id + "_sensor";
    std_msgs::Float64 temperature_msg;

    // EtherCAT
    ROS_INFO("Configuring sensor.");
    aidin_ec_wrapper sensor(interface);
    sensor.init(sample_rate, cutoff_frequency);  
    if(use_imu) {
        sensor.configureSensor( false,       // bias
                                true,       // accel
                                true);      // gyro

    } else {
        sensor.configureSensor( false,      // bias
                                false,       // accel
                                false);      // gyro
    }

    float Fx, Fy, Fz, Tx, Ty, Tz;
    float Ax, Ay, Az, Gx, Gy, Gz;
    float temperature;
    ROS_INFO("Starting continuous mode.");
    while (ros::ok()) {
        
        sensor.getForceTorque(Fx, Fy, Fz, Tx, Ty, Tz);
        // Publish force and torque data
        wrench_msg.header.stamp = ros::Time::now();
        wrench_msg.wrench.force.x = Fx;
        wrench_msg.wrench.force.y = Fy;
        wrench_msg.wrench.force.z = Fz;
        wrench_msg.wrench.torque.x = Tx;
        wrench_msg.wrench.torque.y = Ty;
        wrench_msg.wrench.torque.z = Tz;
        wrench_pub.publish(wrench_msg);

        if (use_imu) {
            sensor.getIMUData(Ax, Ay, Az, Gx, Gy, Gz);
            // Publish IMU data if enabled
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.linear_acceleration.x = Ax;
            imu_msg.linear_acceleration.y = Ay;
            imu_msg.linear_acceleration.z = Az;
            imu_msg.angular_velocity.x = Gx;
            imu_msg.angular_velocity.y = Gy;
            imu_msg.angular_velocity.z = Gz;
            imu_pub.publish(imu_msg);
        }

        sensor.getTempData(temperature);
        temperature_msg.data = temperature;
        temperature_pub.publish(temperature_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Stop.");
    return 0;
}
