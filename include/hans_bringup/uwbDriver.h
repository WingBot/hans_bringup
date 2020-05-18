/*
 *      Author: szr@giimagv
 */

#ifndef INCLUDE_HANS_BRINGUP_UWBDRIVER_H_
#define INCLUDE_HANS_BRINGUP_UWBDRIVER_H_

#include <ros/ros.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
// #include <sensor_msgs/Imu.h>
// #include <lino_msgs/Imu.h>
// #include <sensor_msgs/Range.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <tf/tf.h>
// #include <dynamic_reconfigure/server.h>
// #include <xtark_driver/PID_reconfigConfig.h>

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

//#define G     9.8
#define head1    0x6D
#define head2    0x72
#define head3    0x02 //version
#define endchar1 0x0A
#define endchar2 0x0D
//#define TagNumInFrame 4
#define Tagpayload_size 11
//#define EndcharIndex 14

#define TagMasterID 0x0F
#define TagSlaveID1 0x00
#define TagSlaveID2 0x01
#define TagSlaveID3 0x02
//#define sendType_wheel_speed 0x01
// #define sendType_pid         0x11
// 
// #define foundType_wheel_count 0x01
// #define foundType_battery     0x02
// #define foundType_imu         0x03
// #define foundType_sonar       0x04
// #define foundType_loop_test   0x31

//#define M_PI 3.1415926535

enum packetFinderState
{
    waitingForHead1,
    waitingForHead2,
    waitingForHead3,
    waitingForTagID,
    waitingForFrameID,
    waitingForPayload,
    waitingForEndChar1,
    waitingForEndChar2,
    handlePayload
};

// typedef struct vec3d	vec3d;
// struct vec3d {
// 	double	x;
// 	double	y;
// 	double	z;
// };
// struct imu_data
// {
//     float angle_x;
//     float angle_y;
//     float angle_z;
//     float gyro_x;
//     float gyro_y;
//     float gyro_z;
//     float accel_x;
//     float accel_y;
//     float accel_z;
//     float q0;
//     float q1;
//     float q2;
//     float q3;
// };

typedef boost::shared_ptr<boost::asio::serial_port> serialp_ptr;

class UWBDriver
{
    public:
        UWBDriver();
        ~UWBDriver();
        void loop();
    
    private:
        bool initKit();

        void recv_msg();

//         void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
//         void send_speed_callback(const ros::TimerEvent&);
//         void dynamic_reconfig_callback(xtark_driver::PID_reconfigConfig &config, uint32_t level);
//         void calib_imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void handle_data(uint8_t msg_type,uint8_t* buffer_data);
        void handle_and_trilateration(uint8_t msg_type,uint8_t* buffer_data);
        
//         void handle_speed_data(uint8_t* buffer_data);
//         void handle_battery_data(uint8_t* buffer_data);
//         void handle_imu_data(uint8_t* buffer_data);
//         void handle_sonar_data(uint8_t* buffer_data);

//         void calc_encoder(int& cur, int& recv, int &delta);
        void check_sum(uint8_t* data, size_t len, uint8_t& dest);
//         std::String StringToHex(const std::string& data);
        void distribute_data(uint8_t msg_type, uint8_t* buffer_data);
//         void upload_pid_param();


           packetFinderState state_;
//         int imu_msg_count_=0;
// 
//         struct imu_data imu_data_;

//         sensor_msgs::Imu imu_pub_data_;
//         sensor_msgs::Imu imu_calib_data_;
//         std_msgs::Float32  battery_pub_data_;

//         boost::mutex cmd_vel_mutex_;

        boost::system::error_code ec_;
        boost::asio::io_service io_service_;
        boost::mutex mutex_;
        serialp_ptr sp_;

        bool recv_flag_;
//      bool publish_odom_transform_;
        bool start_flag_;
        uint8_t msg_seq_;

        double rev_dis_anchor_1;
        double rev_dis_anchor_2;
        double rev_dis_anchor_3;
        double rev_dis_anchor_4;
        
//         struct uwb_locat anchor_locat;
        double anchor0_locat_x;
        double anchor0_locat_y;
        double anchor0_locat_z;
        
        double anchor1_locat_x;
        double anchor1_locat_y;
        double anchor1_locat_z;
        
        double anchor2_locat_x;
        double anchor2_locat_y;
        double anchor2_locat_z;
        
        int master_tag_id_;
        int slave1_tag_id_;
        int slave2_tag_id_;
        int slave3_tag_id_;
// 	double ax_roll_acc_;
// 	double ax_roll_;
// 	double ax_pitch_acc_;
// 	double ax_pitch_;
// 	double ax_yaw_;
// 	double ax_cm_k_;
	

//         geometry_msgs::Twist current_twist_;
//         nav_msgs::Odometry odom_;
//         geometry_msgs::TransformStamped transformStamped_;
//         geometry_msgs::TransformStamped bl2lidar_transformStamped_;
//         tf2_ros::TransformBroadcaster br_;
//         tf2_ros::TransformBroadcaster bl2lidar_br_;

//         ros::Time last_twist_time_;
//         ros::Time last_time_;
//         ros::Time now_;
//         ros::Time check_time_;

//         ros::Publisher odom_pub_;
//         ros::Publisher battery_pub_;
//         ros::Publisher imu_pub_;
//         ros::Publisher sonar_pub_;

//         ros::Publisher lvel_pub_;
//         ros::Publisher rvel_pub_;
//         ros::Publisher lset_pub_;
//         ros::Publisher rset_pub_;
// 
//         ros::Publisher avel_pub_;
//         ros::Publisher bvel_pub_;
//         ros::Publisher cvel_pub_;
//         ros::Publisher dvel_pub_;
//         
//         ros::Publisher aset_pub_;
//         ros::Publisher bset_pub_;
//         ros::Publisher cset_pub_;
//         ros::Publisher dset_pub_;
//         
//         std_msgs::Int32 lvel_pub_data_;
//         std_msgs::Int32 rvel_pub_data_;
//         std_msgs::Int32 rset_pub_data_;
//         std_msgs::Int32 lset_pub_data_;
// 
//         std_msgs::Int32 avel_pub_data_;
//         std_msgs::Int32 bvel_pub_data_;
//         std_msgs::Int32 cvel_pub_data_;
//         std_msgs::Int32 dvel_pub_data_;
// 
//         std_msgs::Int32 aset_pub_data_;
//         std_msgs::Int32 bset_pub_data_;
//         std_msgs::Int32 cset_pub_data_;
//         std_msgs::Int32 dset_pub_data_;
// 
// 
//         ros::Subscriber cmd_sub_;
//         ros::Subscriber imu_calib_sub_;

        std::string port_name_;
        
        int baud_rate_;


};

#endif /* INCLUDE_HANS_BRINGUP_UWBDRIVER_H_ */
