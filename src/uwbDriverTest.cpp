#include "hans_bringup/uwbDriver.h"

int main(int argc,char** argv)
{
    ROS_INFO("UWB Kit ros node is initializing...");
    ros::init(argc,argv,"UWB_driver_node");
    UWBDriver driver;
    driver.loop();
    return 0;
}
