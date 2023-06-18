
//
// ██████╗ ██████╗ ███████╗
// ██╔══██╗██╔══██╗██╔════╝
// ██████╔╝██████╔╝███████╗
// ██╔══██╗██╔══██╗╚════██║
// ██║  ██║██████╔╝███████║
// ╚═╝  ╚═╝╚═════╝ ╚══════
//  RB5 Ground Navigation ROS package, pose publisher source code
//
//  Supplementary material to the paper "A low-cost energy-efficient approach for long-term autonomous exploration": https://adamseewald.cc/short/rb52023
//
//  Copyright (c) Adam Seewald, GRAB Lab at Yale University, Department of Mechanical Engineering and Materials Science 
//  Distributed under CC BY-NC-SA licence, details: https://creativecommons.org/licenses/by-nc-sa/4.0/
//


#include "../include/pose_pub.hpp"

#include <functional>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <cstdlib>

using namespace ytcg;

PosePub::PosePub(void) {
    subscription_ = handler_.subscribe<geometry_msgs::Pose>(ORBSLAM_FRAMES_TOPIC, 1000, std::bind(&PosePub::topic_callback, this, std::placeholders::_1));

    ROS_INFO_STREAM("pose publisher node subscribed to topic " << ORBSLAM_FRAMES_TOPIC);
}

PosePub::~PosePub(void) { } 

void PosePub::topic_callback(const geometry_msgs::Pose::ConstPtr& _pose) {
	
    static size_t _count = 0;
    char buffer[100];
    
    if (_count++ == 0) {
#ifdef LOG_POSE
	__LOG_POSE::file().open(LOG_POSE_FILE, std::ios::app);
	ROS_INFO_STREAM("logging enabled, check file " << LOG_POSE_FILE);
#endif
        ROS_INFO_STREAM("callback " << _pose << " from topic " ORBSLAM_FRAMES_TOPIC);
        ROS_INFO("position (x, y, z)");
    }
    
    ROS_INFO_STREAM("(" << _pose->position.x << ", " <<
                           _pose->position.y << ", " << 
                           _pose->position.z << ")"
                   );
#ifdef LOG_POSE
    snprintf(buffer, sizeof(buffer), "%f,%f,%f", _pose->position.x, _pose->position.y, _pose->position.z);
    __LOG_POSE::file() << buffer << std::endl;
#endif
}

void signal_callback_handler(int signum) {
#ifdef LOG_POSE
    __LOG_POSE::file().close();
    ROS_INFO_STREAM(LOG_POSE_FILE << " closed");
#endif
    ROS_INFO("pose publisher node terminating");
    ros::shutdown();
}

int main(int argc, char ** argv) {
    
    // making sure the log file is closed w/ abrupt termiantion
    signal(SIGINT, signal_callback_handler);

    ROS_INFO("started pose publisher node");
    ros::init(argc, argv, NODE_POSE_PUB, ros::init_options::NoSigintHandler);
    ROS_INFO("initialized pose publisher node");

    auto pp = new PosePub();
    ros::spin();
    
    return 0;
}


