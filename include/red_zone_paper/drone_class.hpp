#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <tf2/LinearMath/Quaternion.h>



using namespace std;

class droneClass
{
    public:
    droneClass(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~droneClass();


    void initializeParameters();
    void initializeSubscribers();
    void initializePublishers();

    private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    //Parameters
    string droneType;

    //msg
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

    // subscribers
    ros::Subscriber subPosition_;

    // publishers
    ros::Publisher pubCommand_;

    // Callbacks
    void positionCallback(const geometry_msgs::Pose::ConstPtr& msg);

};