#include <navigator.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigator_node");

    ros::NodeHandle nh;

    ros::Rate r(1000);

    Navigator nav(nh);

    while(ros::ok)
    {
        ros::spinOnce();

        nav.spinOnce();

        r.sleep();
    }

    return 0;

}
