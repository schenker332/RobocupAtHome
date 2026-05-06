#include <open_door/open_door.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "open_door_node");

    ros::NodeHandle nh;

    try
    {
        OpenDoor open_door;
        
        if (!open_door.initialize(nh))
        {
            ROS_ERROR("Failed to initialize OpenDoor node");
            return -1;
        }

        open_door.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception occurred: %s", e.what());
        return 1;
    }

    OutputHelper::printHeader("Open Door Node Terminated");
    return 0;
}
