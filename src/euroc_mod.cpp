#include "ros/ros.h"
#include "Eigen/Dense"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
// #include "math.h"

using namespace Eigen;

ros::Publisher gndtruth_pub;

void gndtruth_cb(const geometry_msgs::TransformStamped::ConstPtr &msg) {

    nav_msgs::Odometry odom_msg;

    odom_msg.header = msg->header;
    odom_msg.header.frame_id = "world";
    
    Vector3d pos_VR(msg->transform.translation.x,
                    msg->transform.translation.y,
                    msg->transform.translation.z);

    Quaterniond quat_VR(msg->transform.rotation.w,
                        msg->transform.rotation.x,
                        msg->transform.rotation.y,
                        msg->transform.rotation.z);

    Vector3d    trans_VR_W(0.787, 2.177, 1.062);
    Quaterniond quat_VR_W(cos(0.2272/2), 0, 0, sin(0.2272/2));

    Vector3d pos_W     = quat_VR_W.inverse()*pos_VR - quat_VR_W.inverse()*trans_VR_W;
    Quaterniond quat_W = quat_VR_W.inverse()*quat_VR;

    odom_msg.pose.pose.position.x = pos_W.x();
    odom_msg.pose.pose.position.y = pos_W.y();
    odom_msg.pose.pose.position.z = pos_W.z();

    odom_msg.pose.pose.orientation.x = quat_W.x();
    odom_msg.pose.pose.orientation.y = quat_W.y();
    odom_msg.pose.pose.orientation.z = quat_W.z();
    odom_msg.pose.pose.orientation.w = quat_W.w();

    // odom_msg.pose.pose.position.x = msg->transform.translation.x;
    // odom_msg.pose.pose.position.y = msg->transform.translation.y;
    // odom_msg.pose.pose.position.z = msg->transform.translation.z;
    
    // odom_msg.pose.pose.orientation.x = msg->transform.rotation.x;
    // odom_msg.pose.pose.orientation.y = msg->transform.rotation.y;
    // odom_msg.pose.pose.orientation.z = msg->transform.rotation.z;
    // odom_msg.pose.pose.orientation.w = msg->transform.rotation.w;

    // Leave the velocity as zeros
    
    gndtruth_pub.publish(odom_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "euroc_mod");
    ros::NodeHandle nh("~");

    ros::Subscriber gndtruth_sub = nh.subscribe("/vicon/firefly_sbx/firefly_sbx", 1, gndtruth_cb);
    gndtruth_pub = nh.advertise<nav_msgs::Odometry>("/vicon/firefly_sbx/firefly_sbx_mod", 1);

    // ros::Rate rate(10);

    // while(ros::ok())
    // {
    //     rate.sleep();
    //     ros::spinOnce();
    // }
    ros::spin();
}