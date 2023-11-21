#include "clothoid/clothoid.h"

ClothoidPathCreator::ClothoidPathCreator():private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("resolution", resolution_);
    private_nh_.getParam("path_length", path_length_);
    private_nh_.getParam("init_x", init_x_);
    private_nh_.getParam("init_y", init_y_);
    private_nh_.getParam("init_yaw", init_yaw_);
    private_nh_.param("frame_id", frame_id_, std::string("map"));

    path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 1);

    init_yaw_ = init_yaw_ * M_PI / 180.0;
}

double ClothoidPathCreator::phi(float phi_0, float phi_v, float phi_u, float s)
{
    return phi_0 + phi_v * s + phi_u * s * s;
}

double ClothoidPathCreator::current_x(float t)
{
    return t * sin(t * t / 2.0);
}

double ClothoidPathCreator::current_y(float t)
{
    return - t * cos(t * t / 2.0);
}


void ClothoidPathCreator::createPath()
{
    path_.header.frame_id = frame_id_;
    path_.header.stamp = ros::Time::now();

    for(double t=0.0; t<path_length_; t+=resolution_)
    {
        // ROS_INFO_STREAM("t: " << t);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id_;
        pose.header.stamp = ros::Time::now();
        double x = current_x(t);
        double y = current_y(t);

        // ROS_INFO_STREAM("x: " << x << ", y: " << y);
        
        double r = (x * x) + (y * y);
        double theta = atan2(y, x);
        pose.pose.position.x = init_x_ + r * cos(theta + init_yaw_);
        pose.pose.position.y = init_y_ + r * sin(theta + init_yaw_);
        pose.pose.orientation.w = 1.0;
        path_.poses.push_back(pose);
    }
}

void ClothoidPathCreator::publishPath()
{
        createPath();
        path_pub_.publish(path_);
}

void ClothoidPathCreator::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        publishPath();
        ros::spinOnce();
        loop_rate.sleep();
    }
}