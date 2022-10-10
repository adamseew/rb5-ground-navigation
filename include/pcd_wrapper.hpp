
#include <ros/ros.h>
#include <vector>

#ifndef YTCG_PCD_WRAPPER_HPP
#define YTCG_PCD_WRAPPER_HPP

#define NODE_PCD_WRAPPER        "pointcloud_depth_wrapper"
#define PCD_FOLDER              "/home/user"
#define PCD_FREQ                1

#define ROCKER_BOGIE_MAX_HEIGHT 1.3
#define ROCKER_BOGIE_MIN_WIDTH  0.8
#define POINT_MAX_DISTANCE      1.5

namespace ytcg {

    struct Point3D {
        double x, y, z;
	Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) { }
	Point3D() : x(.0), y(.0), z(.0) { }
        bool operator==(const Point3D& point) const {
            return (x == point.x && y == point.y && z == point.z); 
	}
        bool operator<(const Point3D& point) const {
            return x < point.x;
        }
    };

    enum Filter { 
        height =        1, // filters datapoints that are higher than the robot
        voxel =         2, // voxel filtering algorithm
	distance =      3, // removes points that are further than a given distance
	duplicates =    4,  // removes duplicates
    };

    class PCDWrapper {

    public:
        PCDWrapper(void);
	~PCDWrapper(void);

    private:
	static void timer_callback(const ros::TimerEvent&);
	static void _filter(const Filter, std::vector<Point3D>&);
	static auto _longest_distance(std::vector<Point3D>&);

	ros::Timer timer_;
	ros::NodeHandle handler_;
    };
}

#endif


