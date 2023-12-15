#ifndef CLOTHOID_H
#define CLOTHOID_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
// #include <kv/defint.hpp>

class ClothoidPathCreator
{
public:
    ClothoidPathCreator();
    void process();

private:
    void createPath();
    void publishPath();

    double phi(float phi_0, float phi_v, float phi_u, float s);
    double current_x(float t);
    double current_y(float t);

    int hz_;
    double resolution_;
    double path_length_;
    double init_x_;
    double init_y_;
    double init_yaw_;

    nav_msgs::Path path_;
    std::string frame_id_;

    ros::Publisher path_pub_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
};

#endif