#include <simulator.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    ros::Rate r(250);

    Simulator sim(nh, pnh, std::vector<std::string>{argv[1], argv[2], argv[3], argv[4], argv[5]});

    while(ros::ok)
    {
        ros::spinOnce();

        sim.updateSim();

        r.sleep();
    }

    return 0;

}
