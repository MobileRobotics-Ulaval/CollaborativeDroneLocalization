#include "drone_nav/drone_nav_node.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv,  "drone_nav_node", ros::init_options::AnonymousName);
    ros::NodeHandle nodeHandle;
    DroneNav droneNav(nodeHandle);
    ros::spin();
}
