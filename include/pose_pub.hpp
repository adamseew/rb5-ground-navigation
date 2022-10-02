
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#ifndef YTCG_POSE_PUB_HPP
#define YTCG_POSE_PUB_HPP

#define NODE_POSE_PUB        "pose_publisher_node"
#define ORBSLAM_FRAMES_TOPIC "/orb_slam3_ros_wrapper/pose"

#define LOG_POSE             1
#define LOG_POSE_FILE        "log_pose.dat"

namespace ytcg {

    class PosePub {

    public:
        PosePub(void);
	~PosePub(void);

    private:
	static void topic_callback(const geometry_msgs::Pose::ConstPtr&);

	ros::Subscriber subscription_;
	ros::NodeHandle handler_;
    };
}

#endif


