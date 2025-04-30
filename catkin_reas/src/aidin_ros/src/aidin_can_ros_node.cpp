#include "ros/ros.h"
#include "aidin_ros/aidin_can_wrapper.h"
#include "geometry_msgs/WrenchStamped.h"
#include <string>

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

    ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/ft_sensor/ft_raw", 1000);

    ECI_RESULT        hResult         = ECI_OK;
    aidin_can_wrapper sensor(true);

    sensor.init_adapter();
    sensor.init_sensor();
    sensor.setup(sample_rate, cutoff_frequency);
    sensor.start();

    geometry_msgs::WrenchStamped msg;
    msg.header.frame_id = arm_id + "_sensor";

    while(ros::ok()){
        float force[3];
        float torque[3];
        sensor.get_force_torque(force, torque);
        
        msg.header.stamp = ros::Time::now();
        msg.wrench.force.x = force[0];
        msg.wrench.force.y = force[1];
        msg.wrench.force.z = force[2];

        msg.wrench.torque.x = torque[0];
        msg.wrench.torque.y = torque[1];
        msg.wrench.torque.z = torque[2];

        wrench_pub.publish(msg);
    }

    sensor.stop();
    sensor.release();

    return 0;
}