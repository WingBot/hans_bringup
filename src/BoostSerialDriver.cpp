#include <string>
#include <ros/ros.h>                          
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>                  

#include <boost/bind.hpp>
#include <math.h>
#include "std_msgs/String.h"              

#include "stdio.h"


using namespace std;
using namespace boost::asio;          
typedef boost::shared_ptr<boost::asio::serial_port> serialp_ptr;
unsigned char buf[100];                 

int i=0;

int data[]={0};

int main(int argc, char** argv) {

    ros::init(argc, argv, "HexSerialDriver");   
    ros::NodeHandle n;
    
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Feedback", 1000);    
    ros::Rate loop_rate(10);


    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");         
    sp.set_option(serial_port::baud_rate(115200));   
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));

    while (ros::ok()) {
      // write(sp, buffer(buf1, 6));  //write the speed for cmd_val    
     //write(sp, buffer("Hello world", 12));  
     read (sp,buffer(buf));
     for(i=0;i<100;i++)
     ROS_INFO("%d",buf[i]);

    ros::spinOnce();

    loop_rate.sleep();
  //  }
    }

    iosev.run(); 
    return 0;
}
