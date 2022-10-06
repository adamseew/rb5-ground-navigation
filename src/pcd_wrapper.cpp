
#include "../include/pcd_wrapper.hpp"

#include <string>

using namespace ytcg;

PCDWrapper::PCDWrapper(void) {
    timer_ = handler_.createTimer(ros::Duration(PCD_FREQ), timer_callback);
    ROS_INFO_STREAM("pointcloud depth wrapper subscribed to timer, triggering each " << std::to_string(PCD_FREQ) << " secs");
}

PCDWrapper::~PCDWrapper(void) { } 

void PCDWrapper::timer_callback(const ros::TimerEvent& _event) {
	
    static size_t _count = 0;
    
    if (_count++ == 0) {
        ROS_INFO("callback from timer");
    }
    
    // todo   
}

int main(int argc, char ** argv) {
    
    ROS_INFO("started pointcloud wrapper node");
    ros::init(argc, argv, NODE_PCD_WRAPPER);

    ROS_INFO("initialized pose publisher node");

    auto pc = new PCDWrapper();
    ros::spin();
    
    return 0;
}


