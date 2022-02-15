#include <iostream>

#include "red_zone_paper/drone_class.hpp"




droneClass::droneClass(ros::NodeHandle& nh, ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private)
{
    initializeParameters();
    initializePublishers();
    initializeSubscribers();
}


droneClass::~droneClass()
{

}

void droneClass::initializeParameters()
{
    nh_private_.getParam("/droneType", droneType);
}


void droneClass::initializeSubscribers()
{
    subPosition_ = nh_.subscribe("/" + droneType + "/ground_truth/pose", 1, &droneClass::positionCallback, this);
}

void droneClass::initializePublishers()
{
    pubCommand_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/" + droneType + "/command/trajectory", 1);
}


void droneClass::positionCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    ROS_INFO_STREAM("Position x:" <<  msg->position.x << " y:" << msg->position.y << " z:" << msg->position.z);
    ROS_INFO_STREAM("Position x:" <<  msg->orientation.x << " y:" << msg->orientation.y << " z:" << msg->orientation.z << " w:" << msg->orientation.w);
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.header.frame_id = "/world";
    trajectory_msg.points.resize(1);
    geometry_msgs::Vector3 pose;
    pose.x = -1.0;
    pose.y = -1.0;
    pose.z = 5.0;

    tf2::Quaternion q;
    q.setRPY(0.0,0.0,0.0);
    q.normalize();
    geometry_msgs::Transform transform;
    transform.translation = pose;
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.rotation.w = q.w();
    trajectory_msg.points[0].transforms.push_back(transform);

    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    point.transforms.push_back(transform);
    point.velocities.push_back(twist);
    point.accelerations.push_back(twist);
    trajectory_msg.points[0] = point;
    trajectory_msg.points[0].time_from_start = ros::Duration(1.0);
    pubCommand_.publish(trajectory_msg);

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_class");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    droneClass drone(nh, nh_private);

    ros::spin();
    return 0;


}