
#include <ros/ros.h>

#ifndef YTCG_PCD_WRAPPER_HPP
#define YTCG_PCD_WRAPPER_HPP

#define NODE_PCD_WRAPPER "pointcloud_depth_wrapper"
#define PCD_FOLDER       "/home/user"
#define PCD_FREQ         1

namespace ytcg {

    struct PointCloud {
        float x, y, z;
	PointCloud(float _x, float _y, float _z) : x(_x), y(_y), z(_z) { }
    };

    class PCDWrapper {

    public:
        PCDWrapper(void);
	~PCDWrapper(void);

    private:
	static void timer_callback(const ros::TimerEvent&);

	ros::Timer timer_;
	ros::NodeHandle handler_;
    };
}

#endif


