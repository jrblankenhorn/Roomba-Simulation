#include <simulator.hpp>

Simulator::Simulator(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::vector<std::string> &files)
{
    m_vel_sub = nh.subscribe<geometry_msgs::Twist>("/velocity", 10, &Simulator::velCallback, this);
    m_image_pub = nh.advertise<sensor_msgs::Image>("/sim_image", 100);
    m_pose_pub = nh.advertise<geometry_msgs::Pose>("/pose", 100);
    m_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 100);
    m_max_range_pub = nh.advertise<std_msgs::Float32>("/max_obs_range", 100);
    m_bumper_pub = nh.advertise<std_msgs::Bool>("/bumper", 100);
    m_max_vel_pub = nh.advertise<std_msgs::Float32>("/max_vel", 100);
    m_viz_pub = nh.advertise<sensor_msgs::Image>("/sensor_viz", 100);
    f = boost::bind(&Simulator::dynReconfigureCallback, this, _1, _2);
    server.setCallback(f);
    pnh.getParam("robot_radius", m_robot_rad);
    initImg(files);
    initSim();
}

Simulator::~Simulator()
{
    delete m_world_img;
    delete m_roomba_img;
    delete m_roomba_viz_img;
    delete m_recording_img;
    delete m_writer;
}

void Simulator::updateSim()
{
    resetImgs();
    if(m_have_vel)
    {
        calcNewPose();
    }
    if(m_trace_path)
    {
        drawPath();
    }
    drawRoomba();
    calcScan();
    pubImg();
    if(m_record_vid)
    {
        recordVideo();
    }
    pubPose();
    pubScan();
    pubMaxRange();
    pubBumper();
    pubMaxVel();
    updateSensorViz();
    pubSensorViz();
}

void Simulator::initImg(const std::vector<std::string> &files)
{
    m_world_img = new cv::Mat(cv::imread(files[0]));
    m_roomba_img = new cv::Mat(cv::imread(files[1]));
    m_roomba_viz_img = new cv::Mat(cv::imread(files[2]));
    m_ultrasonic_img = new cv::Mat(cv::imread(files[3]));
    m_recording_img = new cv::Mat(cv::imread(files[4]));
}

void Simulator::initSim()
{
    updatePose(50, 50, -M_PI / 2);
}

void Simulator::resetImgs()
{
    m_world_img->copyTo(m_sim_img);
    m_sensor_img = cv::Mat(500, 400, CV_8UC3, cv::Scalar(255, 255, 255));
    m_roomba_viz_img->copyTo(m_roomba_viz_img_);
}

void Simulator::calcNewPose()
{
    ros::Time current_time = ros::Time::now();
    ros::Duration dt = current_time - m_prev_time;
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_roomba_pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw += m_vel->angular.z * dt.toSec();
    double x = m_roomba_pose.position.x - m_vel->linear.x * cos(yaw) * dt.toSec();
    double y = m_roomba_pose.position.y - m_vel->linear.x * sin(yaw) * dt.toSec();
    if(!checkForCollision(x, y))
    {
        updatePose(x, y, yaw);
    }
    m_prev_time = current_time;
}

const bool Simulator::checkForCollision(const double &x, const double &y)
{
    const int &roomba_rad = m_robot_rad;
    const double ang_res = 0.05;
    const int &num_pts = 2 * M_PI / ang_res;
    for(int i = 0; i < num_pts; i++)
    {
        const int &x_ = x + roomba_rad * cos(i * ang_res);
        const int &y_ = y + roomba_rad * sin(i * ang_res);
        cv::Vec3b pixel = m_sim_img.at<cv::Vec3b>(x_, y_);
        if(pixel == cv::Vec3b(0, 0, 0))
        {
            m_bumper = true;
            return true;
        }
        if(x_ > m_world_img->cols - 1 || x_ < 0 || y_ > m_world_img->rows - 1 || y_ < 0)
        {
            return true;
        }
    }
    m_bumper = false;
    return false;
}

void Simulator::drawRoomba()
{
    cv::Mat roomba_img = *m_roomba_img;
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_roomba_pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw = yaw * 180 / M_PI;
    const cv::Point2f center((roomba_img.cols - 1) / 2.0F, (roomba_img.rows - 1) / 2.0F);
    const cv::Mat &rot = cv::getRotationMatrix2D(center, yaw, 1.0);
    cv::Mat rotated_roomba;
    cv::warpAffine(roomba_img, rotated_roomba, rot, roomba_img.size());
    double x = m_roomba_pose.position.x - roomba_img.cols / 2 - 1;
    double y = m_roomba_pose.position.y - roomba_img.rows / 2 - 1;
    if(x > m_world_img->cols - m_roomba_img->cols / 2 - 1)
    {
        x = m_world_img->cols - m_roomba_img->cols / 2 - 1;
    }
    if(x < 0)
    {
        x = 0;
    }
    if(y > m_world_img->rows - m_roomba_img->rows / 2 - 1)
    {
        y = m_world_img->rows - m_roomba_img->rows / 2 - 1;
    }
    if(y < 0)
    {
        y = 0;
    }
    for(int y_ = 0; y_ < rotated_roomba.rows; y_++)
    {
        for(int x_ = 0; x_ < rotated_roomba.cols; x_++)
        {
            cv::Vec3b &pixel = rotated_roomba.at<cv::Vec3b>(y_, x_);
            if(pixel[0] < 230 && pixel[1] < 230 && pixel[2] < 230)
            {
                pixel[0] = 255;
                pixel[1] = 255;
                pixel[2] = 255;
            }
        }
    }
    rotated_roomba.copyTo(m_sim_img(cv::Rect(y, x, roomba_img.cols, roomba_img.rows)));
    trimWhiteSpace(x, y);
}

void Simulator::trimWhiteSpace(const double &x, const double &y)
{
    for(int y_ = x; y_ < x + m_roomba_img->rows; y_++)
    {
        for(int x_ = y; x_ < y + m_roomba_img->cols; x_++)
        {
            cv::Vec3b &original_pixel = m_world_img->at<cv::Vec3b>(y_, x_);
            cv::Vec3b &new_pixel = m_sim_img.at<cv::Vec3b>(y_, x_);
            if(original_pixel[0] == 0 && original_pixel[1] == 0 && original_pixel[2] == 0)
            {
                new_pixel[0] = 0;
                new_pixel[1] = 0;
                new_pixel[2] = 0;
            }
        }
    }
}

void Simulator::drawPath()
{
    for(auto pose : m_path)
    {
        m_sim_img.at<cv::Vec3b>(pose.position.x, pose.position.y) = cv::Vec3b(0, 255, 255);
    }
}

void Simulator::calcScan()
{
    m_scan.header.stamp = ros::Time::now();
    m_scan.angle_increment = 2 * M_PI / 8;
    m_scan.angle_min = 0;
    m_scan.angle_max = 7 / 4 * M_PI;
    m_scan.ranges.clear();
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_roomba_pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    for(int i = 0; i < 8; i++)
    {
        double range = getRange(-yaw - M_PI / 2 + i * m_scan.angle_increment);
        m_scan.ranges.push_back(range);
    }
}

const double Simulator::getRange(const double &angle)
{

    cv::Vec3b pixel(255, 255, 255);
    double x = m_roomba_pose.position.y + m_robot_rad * cos(angle);
    double y = m_roomba_pose.position.x + m_robot_rad * sin(angle);
    while(pixel[0] != 0 && pixel[1] != 0 && pixel[2] != 0)
    {
        x += cos(angle);
        y += sin(angle);
        if(x > m_world_img->cols - 1 || x < 0 || y > m_world_img->rows - 1 || y < 0)
        {
            return 0;
        }
        pixel = m_world_img->at<cv::Vec3b>(int(y), int(x));
        if(m_show_scan)
        {
            cv::Vec3b &new_pixel = m_sim_img.at<cv::Vec3b>(int(y), int(x));
            new_pixel[0] = 0;
            new_pixel[1] = 0;
            new_pixel[2] = 0;
        }
    }
    return sqrt(pow(x - m_roomba_pose.position.y, 2) + pow(y - m_roomba_pose.position.x, 2)) - m_robot_rad;
}

void Simulator::pubImg() const
{
    cv_bridge::CvImage img;
    img.header.stamp = ros::Time::now();
    img.encoding = sensor_msgs::image_encodings::BGR8;
    img.image = m_sim_img;
    m_image_pub.publish(img.toImageMsg());
}

void Simulator::updatePose(const double &x, const double &y, const double &heading)
{
    m_roomba_pose.position.x = x;
    m_roomba_pose.position.y = y;
    const tf::Quaternion &q = tf::createQuaternionFromYaw(heading);
    m_roomba_pose.orientation.w = q.getW();
    m_roomba_pose.orientation.x = q.getX();
    m_roomba_pose.orientation.y = q.getY();
    m_roomba_pose.orientation.z = q.getZ();
    updatePath(m_roomba_pose);
}

void Simulator::updatePath(const geometry_msgs::Pose &pose)
{
     m_path.push_back(pose);
}

void Simulator::recordVideo()
{
    m_writer->write(m_sim_img);
}

void Simulator::updateSensorViz()
{
    if(m_record_vid)
    {
        drawRecordViz();
    }
    if(m_bumper)
    {
        drawBumperViz();
    }
    drawRoombaViz();
    drawUltrasonicViz();
}

void Simulator::drawUltrasonicViz()
{
    for(int i = 0; i < m_scan.ranges.size(); i++)
    {
        if(m_scan.ranges[i] <= m_max_range)
        {
            const double &angle = M_PI - i * m_scan.angle_increment;
            const cv::Point2f center((m_ultrasonic_img->cols - 1) / 2.0F, (m_ultrasonic_img->rows - 1) / 2.0F);
            const cv::Mat &rot = cv::getRotationMatrix2D(center, angle / M_PI * 180 + 180, 1.0);
            cv::Mat rotated_ultrasonic;
            cv::Mat ultrasonic_img = *m_ultrasonic_img;
            cv::warpAffine(ultrasonic_img, rotated_ultrasonic, rot, ultrasonic_img.size());
            const int &x = m_sensor_img.rows / 2 - 1 - rotated_ultrasonic.rows / 2 - 1 + 149 * cos(angle);
            const int &y = m_sensor_img.cols / 2 - 1 - rotated_ultrasonic.cols / 2 - 1 + 149 * sin(angle);
            for(int y_ = 0; y_ < rotated_ultrasonic.rows; y_++)
            {
                for(int x_ = 0; x_ < rotated_ultrasonic.cols; x_++)
                {
                    cv::Vec3b &pixel = rotated_ultrasonic.at<cv::Vec3b>(y_, x_);
                    if(pixel[0] < 100 && pixel[1] < 100 && pixel[2] < 100)
                    {
                        pixel[0] = 255;
                        pixel[1] = 255;
                        pixel[2] = 255;
                    }
                }
            }
            rotated_ultrasonic.copyTo(m_sensor_img(cv::Rect(y, x, rotated_ultrasonic.cols, rotated_ultrasonic.rows)));
        }
    }
}

void Simulator::drawRecordViz()
{
    ros::Duration dt = ros::Time::now() - m_prev_rec_time;
    if(dt.toSec() > 0.75)
    {
        m_show_rec = !m_show_rec;
        m_prev_rec_time = ros::Time::now();
    }
    if(m_show_rec)
    {
        m_recording_img->copyTo(m_sensor_img(cv::Rect(m_sensor_img.cols - m_recording_img->cols - 1, 0,
                                                      m_recording_img->cols,
                                                      m_recording_img->rows)));
    }
}

void Simulator::drawBumperViz()
{
    cv::circle(m_roomba_viz_img_, cv::Point(m_roomba_viz_img_.cols / 2 - 1, m_roomba_viz_img_.rows / 2 - 1),
                                       85, cv::Scalar(0, 0, 255), 3, 8, 0);
}

void Simulator::drawRoombaViz()
{
    const int &x = (m_sensor_img.rows - m_roomba_viz_img_.rows) / 2 - 1;
    const int &y = (m_sensor_img.cols - m_roomba_viz_img_.cols) / 2 - 1;
    m_roomba_viz_img_.copyTo(m_sensor_img(cv::Rect(y, x, m_roomba_viz_img_.cols, m_roomba_viz_img_.rows)));
}

void Simulator::pubPose() const
{
    m_pose_pub.publish(m_roomba_pose);
}

void Simulator::pubScan() const
{
    m_scan_pub.publish(m_scan);
}

void Simulator::pubMaxRange() const
{
    std_msgs::Float32 max_range;
    max_range.data = m_max_range;
    m_max_range_pub.publish(max_range);
}

void Simulator::pubBumper() const
{
    std_msgs::Bool bumper;
    bumper.data = m_bumper;
    m_bumper_pub.publish(bumper);
}

void Simulator::pubMaxVel() const
{
    std_msgs::Float32 vel;
    vel.data = m_max_vel;
    m_max_vel_pub.publish(vel);
}

void Simulator::pubSensorViz() const
{
    cv_bridge::CvImage img;
    img.header.stamp = ros::Time::now();
    img.encoding = sensor_msgs::image_encodings::BGR8;
    img.image = m_sensor_img;
    m_viz_pub.publish(img.toImageMsg());
}

void Simulator::velCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if(!m_have_vel)
    {
        m_prev_time = ros::Time::now();
    }
    m_have_vel = true;
    m_vel = msg;
}

void Simulator::dynReconfigureCallback(simulator::SimulatorConfig &config, uint32_t level)
{
    m_max_vel = config.velocity;
    m_max_range = config.max_obstacle_range;
    m_trace_path = config.show_path;
    m_show_scan = config.show_scan;
    m_vid_path = config.video_filepath;
    if(m_record_vid != config.record_video)
    {
        m_record_vid = config.record_video;
        if(m_record_vid)
        {
            if(m_vid_path.find(".avi") == std::string::npos)
            {
                ROS_ERROR_STREAM("Video Location Must Include Filename Ending in '.avi'");
                m_record_vid = false;
                config.record_video = false;
                return;
            }
            ROS_INFO_STREAM("Recording Video");
            m_writer = new cv::VideoWriter(m_vid_path, CV_FOURCC('M', 'J', 'P', 'G'), 400, cv::Size(m_sim_img.cols, m_sim_img.rows));
            m_prev_rec_time = ros::Time::now();
            m_show_rec = true;
        }
        else
        {
            m_writer->release();
            ROS_INFO_STREAM("Video Saved to " << m_vid_path);
        }
    }
    m_reset_sim = config.reset_sim;
    if(m_reset_sim)
    {
        m_path.clear();
        initSim();
        m_reset_sim = false;
        config.reset_sim = false;
        m_max_vel = 0;
        config.velocity = 0;
        m_max_range = 25;
        config.max_obstacle_range = 25;
        ROS_INFO_STREAM("Simulation Reset!");
    }
}
