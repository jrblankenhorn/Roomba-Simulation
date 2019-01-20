#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

class Navigator
{
public:
    Navigator(ros::NodeHandle &nh);
    ~Navigator();

    void spinOnce();

private:

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void bumperCallback(const std_msgs::Bool::ConstPtr &msg);
    void maxVelCallback(const std_msgs::Float32::ConstPtr &msg);
    void maxRangeCallback(const std_msgs::Float32::ConstPtr &msg);

    void checkScan();
    void maneuver(const double &range, const double &angle);
    const bool checkBumper() const;
    void recover();

    void pubVel() const;

    ros::Subscriber m_scan_sub;
    ros::Subscriber m_bumper_sub;
    ros::Subscriber m_max_vel_sub;
    ros::Subscriber m_max_range_sub;
    ros::Publisher m_vel_pub;

    sensor_msgs::LaserScan::ConstPtr m_scan;
    std_msgs::Bool::ConstPtr m_bumper;
    geometry_msgs::Twist m_vel;
    double m_max_vel;

    bool m_have_scan = false;
    bool m_have_bumper = false;


    //avoidance
    double m_max_obstacle_range;


};
