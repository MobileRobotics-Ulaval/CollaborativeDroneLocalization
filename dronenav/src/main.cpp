#include "dronenav/dronenav_node.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv,  "dronenav_node", ros::init_options::AnonymousName);
    ros::NodeHandle nodeHandle;
    DroneNav droneNav(nodeHandle);
    ros::spin();
}
