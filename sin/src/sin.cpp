#include "sin/sin.h"

SinPathCreator::SinPathCreator():private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("resolution", resolution_);
    private_nh_.getParam("path_length", path_length_);
    private_nh_.getParam("init_x", init_x_);
    private_nh_.getParam("init_y", init_y_);
    private_nh_.getParam("init_yaw", init_yaw_);
    private_nh_.param("frame_id", frame_id_, {"map"});

    path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);

    init_yaw_ = init_yaw_ * M_PI / 180.0;
}

double SinPathCreator::phi(float phi_0, float phi_v, float phi_u, float s)
{
    return phi_0 + phi_v * s + phi_u * s * s;
}

double SinPathCreator::current_y(float t)
{
    return sin(t);
}


void SinPathCreator::createPath()
{
    path_.header.frame_id = frame_id_;
    path_.header.stamp = ros::Time::now();

    for(double x=0.0; x<path_length_; x+=resolution_)
    {
        // ROS_INFO_STREAM("t: " << t);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id_;
        pose.header.stamp = ros::Time::now();
        double y = current_y(x);

        // ROS_INFO_STREAM("x: " << x << ", y: " << y);

        pose.pose.position.x = init_x_ + x;
        pose.pose.position.y = init_y_ + y;

        // double r = (x * x) + (y * y);
        // double theta = atan2(y, x);
        // pose.pose.position.x = init_x_ + r * cos(theta + init_yaw_);
        // pose.pose.position.y = init_y_ + r * sin(theta + init_yaw_);
        pose.pose.orientation.w = 1.0;
        path_.poses.push_back(pose);
    }
}

void SinPathCreator::publishPath()
{
        createPath();
        path_pub_.publish(path_);
}

void SinPathCreator::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        publishPath();
        ros::spinOnce();
        loop_rate.sleep();
    }
}