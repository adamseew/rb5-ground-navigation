
#include "../include/pose_pub.hpp"

#include <iostream>
#include <fstream>

using namespace ytcg;

PosePub::PosePub(void) {
    subscription_ = handler_.subscribe(ORBSLAM_FRAMES_TOPIC, 1000, topic_callback);

    ROS_INFO_STREAM("pose publisher node subscribed to topic " << ORBSLAM_FRAMES_TOPIC);
}

PosePub::~PosePub(void) { } 

#ifdef LOG_POSE
    static std::ofstream file;
#endif

void PosePub::topic_callback(const geometry_msgs::Pose::ConstPtr& _pose) {
	
    static size_t _count = 0;
    char buffer[100];
    
    if (_count++ == 0) {
#ifdef LOG_POSE
        file.open(LOG_POSE_FILE, std::ios::app);
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
    file << buffer << std::endl;
#endif
}

int main(int argc, char ** argv) {

    ROS_INFO("started pose publisher node");
    ros::init(argc, argv, NODE_POSE_PUB);
    ROS_INFO("initialized pose publisher node");

    auto pp = new PosePub();
    ros::spin();
#ifdef LOG_POSE
    file.close();
    ROS_INFO_STREAM(LOG_POSE_FILE << " ready");
#endif

    ROS_INFO("pose publisher node terminated");
    return 0;
}


