#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
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
    //realsense_sub(nh.subscribe("/realsense_odom", 100, &preLocalization::realsense_CB, this)),
    realsense_sub(nh.subscribe("/camera/imu", 100, &preLocalization::camera_CB, this)),

    imu_gps_pub(nh.advertise<nav_msgs::Odometry>("/imu_gps", 10)), // position, orientation
    camera_imu_pub(nh.advertise<nav_msgs::Odometry>("/camera_imu", 10))
    //vodom_pub(nh.advertise<nav_msgs::Odometry>("/visual_odom", 10)),
    loop_rate(20)
    {
        initImuGpsOdom();
        // initVisualOdom();
        intiCameraOdom();
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
            camera_imu_pub.publish(camera_imu_pub)
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

    //initializing camera_odom_msg NEW
    void intiCameraOdom(void)
    {
        camera_odom_msg.header.frame_id = "odom";
        camera_odom_msg.child_frame_id = "base_footprint";
        camera_odom_msg.pose.pose.position.x = 0.0;
        camera_odom_msg.pose.pose.position.y = 0.0;
        camera_odom_msg.pose.pose.position.z = 0.0;

        camera_odom_msg.pose.pose.orientation.x = 0.0;
        camera_odom_msg.pose.pose.orientation.y = 0.0;
        camera_odom_msg.pose.pose.orientation.z = 0.0;
        camera_odom_msg.pose.pose.orientation.w = 1.0;

        camera_odom_msg.twist.twist.linear.x = 0.0;
        camera_odom_msg.twist.twist.linear.y = 0.0;
        camera_odom_msg.twist.twist.linear.z = 0.0;
        camera_odom_msg.twist.twist.angular.x = 0.0;
        camera_odom_msg.twist.twist.angular.y = 0.0;
        camera_odom_msg.twist.twist.angular.z = 0.0;
    }

    //initializing tf for odometry of "realsense" and "vodom"
    void initVisualOdomTF()
    {
        if(imu_msg_received)
        {
            geometry_msgs::TransformStamped parent_tf, child_tf;

            parent_tf.header.stamp = ros::Time::now();
            parent_tf.header.frame_id = "realsense_parent"; //realsense_parent
            parent_tf.child_frame_id = "vodom_parent";  //parent_new
            parent_tf.transform.translation.x = 0;
            parent_tf.transform.translation.y = 0;
            parent_tf.transform.translation.z = 0;
            parent_tf.transform.rotation = initial_orientation;
            static_broadcaster_.sendTransform(parent_tf);

            child_tf.header.stamp = ros::Time::now();
            child_tf.header.frame_id = "realsense_child";  //realsense_child
            child_tf.child_frame_id = "vodom_child";   //child_new
            child_tf.transform.translation.x = 0;
            child_tf.transform.translation.y = 0;
            child_tf.transform.translation.z = 0;
            child_tf.transform.rotation = initial_orientation;
            static_broadcaster_.sendTransform(child_tf);

            vodom_tf_initialized = true;
        }
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

    void camera_CB(const sensor_msgs::Imu &imu_msg)
    {
        if(!cam_imu_msg_received){
            initial_linear_acceleration = imu_msg.linear_acceleration;
            initial_angular_velocity = imu_msg.angular_velocity;
            cam_imu_msg_received = true;
        }
        camera_odom_msg.twist.twist.linear.x = imu_msg.linear_acceleration.x
        camera_odom_msg.twist.twist.linear.y = imu_msg.linear_acceleration.y
        camera_odom_msg.twist.twist.linear.z = imu_msg.linear_acceleration.z
        camera_odom_msg.twist.twist.angular.x = imu_msg.angular_velocity.x 
        camera_odom_msg.twist.twist.angular.y = imu_msg.angular_velocity.y
        camera_odom_msg.twist.twist.angular.z = imu_msg.angular_velocity.z
    }

    /*
    // need to change this callback
    void realsense_CB(const nav_msgs::OdometryConstPtr &realsense_msg)
    {
        if(vodom_tf_initialized)
        {
            tf2::Transform transform;
            tf2::fromMsg(
            buffer_.lookupTransform(
                "vodom_parent", "vodom_child", realsense_msg->header.stamp, ros::Duration(1.0)
            ).transform,
            transform
            );
            tf2::toMsg(transform, vodom_msg.pose.pose);
                vodom_msg.pose.covariance = tf2::transformCovariance(
            realsense_msg->pose.covariance, transform
            );

            tf2::Transform inverse;
            tf2::fromMsg(
            buffer_.lookupTransform(
                "vodom_child", "realsense_child", realsense_msg->header.stamp, ros::Duration(1.0)
            ).transform,
            inverse
            );
            tf2::Vector3 linear_old, angular_old;
            tf2::fromMsg(realsense_msg->twist.twist.linear, linear_old);
            tf2::fromMsg(realsense_msg->twist.twist.angular, angular_old);
            tf2::Vector3 angular_new = inverse.getBasis() * angular_old;
            tf2::Vector3 linear_new  = inverse.getBasis() * linear_old + inverse.getOrigin().cross(angular_new);
            vodom_msg.twist.twist.angular = tf2::toMsg(angular_new);
            vodom_msg.twist.twist.linear = tf2::toMsg(linear_new);
            vodom_msg.twist.covariance = realsense_msg->twist.covariance;

            vodom_msg.header.stamp = realsense_msg->header.stamp;

            // Publish transformed odometry.
            vodom_pub.publish(vodom_msg);
        }
    }
    */


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
    bool cam_imu_msg_received = false;
    bool vodom_tf_initialized = false;
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
    nav_msgs::Odometry vodom_msg;
    geometry_msgs::Quaternion initial_orientation;
	
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pre_localization");
    ros::NodeHandle nh;
    preLocalization core(nh);
    core.spin();
    return 0;
}
