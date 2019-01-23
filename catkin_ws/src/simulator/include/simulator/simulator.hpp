#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <simulator/SimulatorConfig.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>

class Simulator
{
public:
    Simulator(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::vector<std::string> &files);
    ~Simulator();

    void updateSim();

private:

    //callbacks
    void velCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void dynReconfigureCallback(simulator::SimulatorConfig &config, uint32_t level);

    //methods
    void initImg(const std::vector<std::string> &files);
    void initSim();
    void resetImgs();
    void calcNewPose();
    const bool checkForCollision(const double &x, const double &y);
    void drawRoomba();
    void trimWhiteSpace(const double &x, const double &y);
    void drawPath();
    void calcScan();
    const double getRange(const double &angle);
    void pubImg() const;
    void updatePose(const double &x, const double &y, const double &heading);
    void updatePath(const geometry_msgs::Pose &pose);
    void recordVideo();
    void updateSensorViz();
    void drawUltrasonicViz();
    void drawRecordViz();
    void drawBumperViz();
    void drawRoombaViz();
    void pubPose();
    void pubScan() const;
    void pubMaxRange() const;
    void pubBumper() const;
    void pubMaxVel() const;
    void pubAggressiveness() const;
    void pubSensorViz() const;

    //pubsub
    ros::Subscriber m_path_sub;
    ros::Subscriber m_vel_sub;
    ros::Publisher m_image_pub;
    ros::Publisher m_pose_pub;
    ros::Publisher m_scan_pub;
    ros::Publisher m_max_range_pub;
    ros::Publisher m_bumper_pub;
    ros::Publisher m_max_vel_pub;
    ros::Publisher m_aggressiveness_pub;
    ros::Publisher m_viz_pub;

    //images
    cv::Mat m_sim_img;
    cv::Mat *m_world_img;
    cv::Mat *m_roomba_img;
    cv::Mat *m_roomba_viz_img;
    cv::Mat *m_ultrasonic_img;
    cv::Mat *m_recording_img;
    cv::Mat m_sensor_img;
    cv::Mat m_roomba_viz_img_;

    //pose and sensor data
    geometry_msgs::Pose m_roomba_pose;
    geometry_msgs::Pose m_roomba_pose_corrected;
    sensor_msgs::LaserScan m_scan;
    bool m_bumper = false;
    ros::Time m_prev_time;
    geometry_msgs::Twist::ConstPtr m_vel;
    std::vector<geometry_msgs::Pose> m_path;

    //dynamic reconfigure
    dynamic_reconfigure::Server<simulator::SimulatorConfig> server;
    dynamic_reconfigure::Server<simulator::SimulatorConfig>::CallbackType f;    
    double m_max_vel = 0;
    double m_aggressiveness = 0;
    double m_max_range = 0;
    bool m_trace_path = false;
    bool m_show_scan = false;
    std::string m_vid_path;
    bool m_record_vid = false;
    bool m_reset_sim = false;

    //video recording
    cv::VideoWriter *m_writer;
    ros::Time m_prev_rec_time;
    bool m_show_rec = false;

    //flags
    bool m_have_vel = false;    

    //params
    double m_robot_rad;


};
