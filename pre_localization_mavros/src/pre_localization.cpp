#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
//#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robot_localization/navsat_conversions.h>

using namespace std;

class preLocalization
{
public:
    preLocalization(ros::NodeHandle &nh):
    navsat_sub(nh.subscribe("/ublox/fix", 100, &preLocalization::gps_CB, this)),
    imu_sub(nh.subscribe("/iahrs/imu/data", 100, &preLocalization::imu_CB, this)),
    mavros_imu_sub(nh.subscribe("/mavros/imu/data", 100, &preLocalization::mavros_imu_CB, this)),
    mavros_pos_sub(nh.subscribe("/mavros/local_position/pose", 100, &preLocalization::mavros_pose_CB, this)),
    imu_gps_pub(nh.advertise<nav_msgs::Odometry>("/imu_gps", 10)),
    mavros_pub(nh.advertise<nav_msgs::Odometry>("/mavros", 10)),
    loop_rate(20)
    {
        initImuGpsOdom();
        initMavrosOdom();
        //GPS2base();
    }

    void spin()
    {
        while(ros::ok())
        {
            ros::spinOnce();
            initMap2Odom();
            initVisualOdomTF();
            imu_gps_pub.publish(imu_gps_msg);
            loop_rate.sleep();
        }
    }

    //initializing imu_gps_msg
    void initImuGpsOdom(void) 
    {
        imu_gps_msg.header.frame_id = "odom";
        imu_gps_msg.child_frame_id = "base_footprint";
        imu_gps_msg.pose.pose.position.x = 0.0;
        imu_gps_msg.pose.pose.position.y = 0.0;
        imu_gps_msg.pose.pose.position.z = 0.0;

        imu_gps_msg.pose.pose.orientation.x = 0.0;
        imu_gps_msg.pose.pose.orientation.y = 0.0;
        imu_gps_msg.pose.pose.orientation.z = 0.0;
        imu_gps_msg.pose.pose.orientation.w = 1.0;

        imu_gps_msg.twist.twist.linear.x = 0.0;
        imu_gps_msg.twist.twist.angular.z = 0.0;
    }

    //initializing mavros_msg
    void initMavrosOdom(void)
    {
        mavros_msg.header.frame_id = "odom";
        mavros_msg.child_frame_id = "base_footprint";
        mavros_msg.pose.pose.position.x = 0.0;
        mavros_msg.pose.pose.position.y = 0.0;
        mavros_msg.pose.pose.position.z = 0.0;

        mavros_msg.pose.pose.orientation.x = 0.0;
        mavros_msg.pose.pose.orientation.y = 0.0;
        mavros_msg.pose.pose.orientation.z = 0.0;
        mavros_msg.pose.pose.orientation.w = 1.0;

        mavros_msg.twist.twist.linear.x = 0.0;
        mavros_msg.twist.twist.angular.z = 0.0;
    }
    
    //initializing map-odom tf
    void initMap2Odom()
    {
        if(navSat_msg_received && !map_odom_tf_initialized){
            geometry_msgs::TransformStamped map_odom_tf;

            map_odom_tf.header.stamp = ros::Time::now();
            map_odom_tf.header.frame_id = "map";
            map_odom_tf.child_frame_id = "odom";
            map_odom_tf.transform.translation.x = odom_origin_x_ - gps_origin_x_;
            map_odom_tf.transform.translation.y = odom_origin_y_ - gps_origin_y_;
            map_odom_tf.transform.translation.z = 0;
            map_odom_tf.transform.rotation.x = 0;
            map_odom_tf.transform.rotation.y = 0;
            map_odom_tf.transform.rotation.z = 0;
            map_odom_tf.transform.rotation.w = 1;
            static_broadcaster_.sendTransform(map_odom_tf);

            map_odom_tf_initialized = true;
        }
    }


    void gps_CB(const sensor_msgs::NavSatFix &navsat_msg)
    {
        double utm_x = 0, utm_y = 0;
        std::string utm_zone;
        //convert lat/long to utm
        RobotLocalization::NavsatConversions::LLtoUTM(navsat_msg.latitude, navsat_msg.longitude, utm_x, utm_y, utm_zone);

        if(!navSat_msg_received){
            if(gps_origin_x_ == 0 && gps_origin_y_ == 0)
            {
            gps_origin_x_ = utm_x;
            gps_origin_y_ = utm_y;
            }
            odom_origin_x_ = utm_x;
            odom_origin_y_ = utm_y;
            navSat_msg_received = true;
        }

        imu_gps_msg.header.stamp = ros::Time::now();
        imu_gps_msg.pose.pose.position.x = utm_x - odom_origin_x_;
        imu_gps_msg.pose.pose.position.y = -(utm_y - odom_origin_y_);
        imu_gps_msg.pose.pose.position.z = 0;
    }

    void imu_CB(const sensor_msgs::Imu &imu_msg)
    {
        if(!imu_msg_received){
            initial_orientation = imu_msg.orientation;
            imu_msg_received = true;
        }

        imu_gps_msg.pose.pose.orientation.x = imu_msg.orientation.x;
        imu_gps_msg.pose.pose.orientation.y = imu_msg.orientation.y;
        imu_gps_msg.pose.pose.orientation.z = imu_msg.orientation.z;
        imu_gps_msg.pose.pose.orientation.w = imu_msg.orientation.w;
    }

    void mavros_pose_CB(const geometry_msgs::PoseStamped &pos_msg)
    {   
        if(!mavros_pos_received){
            initial_position = pos_msg.pose
            mavros_pos_received = true
        }

        mavros_msg.pose.pose.position.x = pos_msg.pose.position.x;
        mavros_msg.pose.pose.position.y = pos_msg.pose.position.y;
        mavros_msg.pose.pose.position.z = pos_msg.pose.position.z;
    }

    void mavros_imu_CB(const sensor_msgs::Imu &imu_msg)
    {
        if(!mavros_imu_received){
            initial_orientation = imu_msg.orientation;
            mavros_imu_received = true;
        }

        mavros_msg.pose.pose.orientation.x = imu_msg.orientation.x;
        mavros_msg.pose.pose.orientation.y = imu_msg.orientation.y;
        mavros_msg.pose.pose.orientation.z = imu_msg.orientation.z;
        mavros_msg.pose.pose.orientation.w = imu_msg.orientation.w;
    }


private:
    /*Subscriber */
    ros::Subscriber navsat_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber realsense_sub;
    
    /*Publisher */
    ros::Publisher imu_gps_pub;
    ros::Publisher vodom_pub;

    /*Transform Broadcaster */
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    tf2_ros::Buffer buffer_;

    /*Parameter*/
    bool navSat_msg_received = false;
    bool imu_msg_received = false;
    bool mavros_imu_received = false;
    bool mavros_pos_received = false;
    bool map_odom_tf_initialized = false;
    double gps_origin_x_ = 0;
    double gps_origin_y_ = 0;
    double odom_origin_x_ = 0;
    double odom_origin_y_ = 0;

    /*ROS Variable*/
    ros::NodeHandle nh;
    ros::Rate loop_rate;  

    /*Message Types*/
    nav_msgs::Odometry imu_gps_msg;
    nav_msgs::Odometry mavros_msg;
    geometry_msgs::Quaternion initial_orientation;
    geometry_msgs::Pose initial_position
	
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pre_localization");
    ros::NodeHandle nh;
    preLocalization core(nh);
    core.spin();
    return 0;
}
