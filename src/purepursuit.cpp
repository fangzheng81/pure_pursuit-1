#include <cmath> // where to get this?
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

class PurePursuit
{
public:
    // constructor
    PurePursuit();

    void getOdometry(nav_msgs::Odometry odometry_msg);

    void getPath(nav_msgs::Path path_msg);

    void getGoalPoint();

    void computeCurvature();

    void RunUpdate();

    // some helper functions

    double dist2D(double x1, double y1, double x2, double y2);


private:

    nav_msgs::Path path;

    nav_msgs::Odometry odometry;

    geometry_msgs::Pose goal_pose;

    ros::NodeHandle nh, nh_private; // what is this nh_private used for
    ros::Subscriber odometry_sub, path_sub;
    ros::Publisher vel_pub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // frame id
    std::string map_id, robot_id;

    // index of the path node
    int path_index;

    // look ahead distance
    double D;

    // constant linear velocity
    double lin_vel;
}

PurePursuit::PurePursuit() : path_index(0), tf_listener(tf_buffer) // put initialized value of private variable here
{
    // get some parameters from parameter server and set them

    path_sub = nh.subscribe("path", 1, &PurePursuit::pathCallback, this);
    odometry_sub = nh.subscribe("odometry", 1, &PurePursuit::odomCallback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("twist_test", 1);
}


void PurePursuit::getGoalPoint()
{
    geometry_msgs::TransformStamped tfstamped;
    std::vector<geometry_msgs::Pose> path_temp;

    try
    {
        // current position of robot in global frame
        double xr;
        double yr;
        double hr;
        // closest point x y
        double xc;
        double yc;

        int min_index;
        double min_dist = 99999999.0;

        tfstamped = tf_listener.lookupTransform(map_id, robot_id, ros::Time(0));

        geometry_msgs::Pose pose_temp = new geometry_msgs::Pose;

        double dist_temp;
        // closest point
        for(int i = path_index; i < path.poses.size(); i++)
        {
            // get position from point x,y and get heading from point z
            double x1 = path.poses[i].pose.position.x;
            double y1 = path.poses[i].pose.position.y;
            // double a1 = path.poses[i].pose.point.z;

            // find the closest point to the robot
            dist_temp = dist2D(x1, y1, xr, yr);
            if (dist_temp < min_dist)
            {
                min_dist = dist_temp;
                min_index = i;
                xc = x1;
                yc = y1;
            }

        }

        // goal pose
        int goal_index;

        double di = path.poses[min_index].pose.orientation.x;
        for(int i = min_index; i < path.poses.size(); i++)
        {
            if (path.poses[i].pose.orientation.x <= di )
            {
                ROS_INFO("already passed this point!");
                continue;
            }

            if (path.poses[i].pose.orientation.x - di <= D)
            {
                goal_index = i;
            }
            else
            { break; }
        }

        double xg = path.poses[goal_index].pose.position.x;
        double yg = path.poses[goal_index].pose.position.y;

        goal_pose.position.x = (xg - xr) * cos(hr) + (yg - yr) * sin(hr);
        goal_pose.position.y = -(xg - xr) * sin(hr) + (yg - yr) * cos(hr);
    }
}

void PurePursuit::computeCurvature()
{
    double xg = goal_pose.position.x;
    double yg = goal_pose.position.y;

    double d_true = sqrt(xg*xg + yg*yg);

    double radius = d_true*d_true/(2*fabs(x));

    double ang_vel = lin_vel / radius;

    // do you remember which direction is
    if (xg > 0)
    {
        ang_vel = -ang_vel;
    }

    geometry_msgs::Twist twist;

    twist.linear.x = lin_vel;
    twist.angular.z = ang_vel;

    vel_pub.publish(twist);
}

double PurePursuit::dist2D(double x1, double y1, double x2, double y2)
{
    double result;
    double d1 = x1 - x2;
    double d2 = y1 - y2;

    result = sqrt(d1*d1 + d2*d2);

}

void PurePursuit::getPath(nav_msgs::Path path_msg)
{
    ROS_INFO_STREAM("path i got from frame " << path_msg.header.frame_id << " and size of it " << path_msg.poses.size() );
    path = path_msg;
}

void PurePursuit::getOdometry(nav_msgs::Odometry odometry_msg)
{
    ROS_INFO_STREAM("recieved an odometry msg");
    odometry = odometry_msg;
}

void PurePursuit::RunUpdate()
{
    ros::spin();
}

