#include <navigator.hpp>

Navigator::Navigator(ros::NodeHandle &nh)
{
    m_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Navigator::laserScanCallback, this);
    m_bumper_sub = nh.subscribe<std_msgs::Bool>("/bumper", 10, &Navigator::bumperCallback, this);
    m_max_vel_sub = nh.subscribe<std_msgs::Float32>("/max_vel", 10, &Navigator::maxVelCallback, this);
    m_max_range_sub = nh.subscribe<std_msgs::Float32>("/max_obs_range", 10, &Navigator::maxRangeCallback, this);
    m_vel_pub = nh.advertise<geometry_msgs::Twist>("/velocity", 10);    
    m_vel.linear.x = 0;
    m_vel.angular.z = 0;
}

Navigator::~Navigator(){}

void Navigator::spinOnce()
{
    if(m_have_scan)
    {
        checkScan();
    }
    if(m_have_bumper)
    {
        if(checkBumper())
        {
            recover();
        }
    }
    pubVel();
}

void Navigator::checkScan()
{
    double min_range = std::numeric_limits<double>::infinity();
    double angle;
    for(int i = 0; i < m_scan->ranges.size(); i++)
    {
        if(m_scan->ranges[i] < min_range)
        {
            min_range = m_scan->ranges[i];
            angle = m_scan->angle_min + m_scan->angle_increment * i;
        }
    }
    maneuver(min_range, angle);
}

void Navigator::maneuver(const double &range, const double &angle)
{
    if(range <= m_max_obstacle_range)
    {
        if(angle < M_PI && angle > 0)
        {
            m_vel.linear.x = m_max_vel;
            m_vel.angular.z = m_max_vel / 5;
        }
        else if(angle > M_PI)
        {
            m_vel.linear.x = m_max_vel;
            m_vel.angular.z = -m_max_vel / 5;
        }
        else
        {
            m_vel.linear.x = 0;
            m_vel.angular.z = m_max_vel / 5;
        }
    }
    else
    {
        m_vel.linear.x = m_max_vel;
        m_vel.angular.z = 0;        
    }
}

const bool Navigator::checkBumper() const
{
    return m_bumper->data;
}

void Navigator::recover()
{
    m_vel.angular.z = m_max_vel / 5;
    m_vel.linear.x = -m_max_vel;
}

void Navigator::pubVel() const
{
    m_vel_pub.publish(m_vel);
}

void Navigator::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    m_have_scan = true;
    m_scan = msg;
}

void Navigator::bumperCallback(const std_msgs::Bool::ConstPtr &msg)
{
    m_have_bumper = true;
    m_bumper = msg;
}

void Navigator::maxVelCallback(const std_msgs::Float32::ConstPtr &msg)
{
    m_max_vel = msg->data;
}

void Navigator::maxRangeCallback(const std_msgs::Float32::ConstPtr &msg)
{
    m_max_obstacle_range = msg->data;
}
