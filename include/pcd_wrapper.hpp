
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#ifndef YTCG_PCD_WRAPPER_HPP
#define YTCG_PCD_WRAPPER_HPP

#define NODE_PCD_WRAPPER                "pointcloud_depth_wrapper"
#define PCD_FOLDER                      "/home/user"
#define PCD_FREQ                        .1

#define ROCKER_BOGIE_MAX_HEIGHT         1.3
#define ROCKER_BOGIE_MIN_WIDTH          .8
#define ROCKER_BOGIE_OBSTACLE_CLEARANCE .14
#define POINT_MAX_DISTANCE              1.2

#define LONGEST_DISTANCE_POINT1_TOPIC   "/pointcloud_depth_wrapper/ld_point1"
#define LONGEST_DISTANCE_POINT2_TOPIC   "/pointcloud_depth_wrapper/ld_point2"

#define MAX_FOV_REALSENSE_X             .55
#define MAX_FOV_REALSENSE_Z             1.06653645


namespace ytcg {

    struct Point3D {
        double x, y, z;
        Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) { }
        Point3D() : x(.0), y(.0), z(.0) { }
        bool operator==(const Point3D& point) const {
            return (x == point.x && y == point.y && z == point.z); 
        }
        bool operator<(const Point3D& point) const {
            return point.x < x;
        }
        Point3D& operator*(const Eigen::Matrix<double, 3, 3>& _matrix) {
            x = x*_matrix(0, 0)+y*_matrix(1, 0)+z*_matrix(2, 0);
            y = x*_matrix(0, 1)+y*_matrix(1, 1)+z*_matrix(2, 1);
            z = z*_matrix(0, 2)+y*_matrix(1, 2)+z*_matrix(2, 2);
            return *this;
        }
    };

    enum Filter { 
        distance =   0b00001, // removes points that are further than a given distance
        voxel =      0b00010, // voxel filtering algorithm
        height =     0b00100, // filters datapoints that are higher than the robot
        duplicates = 0b01000, // removes duplicates
        origin =     0b10000, // filter origin point
    };

    class PCDWrapper {

    public:
        PCDWrapper(void);
        ~PCDWrapper(void);

    private:
        void timer_callback(const ros::TimerEvent&);
        void filter(const Filter, std::vector<Point3D>&);
        auto longest_distance(std::vector<Point3D>&);

        ros::Timer timer_;
        ros::NodeHandle handler_;
        ros::Publisher _publisher;
        ros::Publisher __publisher;
        Eigen::Matrix<double, 3, 3> rotx, roty, rot;
    };
}

#endif


