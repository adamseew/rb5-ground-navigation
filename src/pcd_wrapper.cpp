
#include "../include/pcd_wrapper.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/filesystem.hpp>

#include <functional>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <iterator>
#include <cstdlib>
#include <fstream>
#include <string>
#include <cmath>

#define BOOST_RANGE_ENABLE_CONCEPT_ASSERT 0

using namespace boost::filesystem;
using namespace boost::adaptors;
using namespace ytcg;

using std::string;
using std::vector;
using std::time_t;

constexpr double pi = std::acos(-1);

PCDWrapper::PCDWrapper(void) {
    timer_ = handler_.createTimer(ros::Duration(PCD_FREQ), std::bind(&PCDWrapper::timer_callback, this, std::placeholders::_1));
    ROS_INFO_STREAM("pointcloud depth wrapper node subscribed to timer, triggering each " << std::to_string(PCD_FREQ) << " secs");
    _publisher  = handler_.advertise<geometry_msgs::Point>(LONGEST_DISTANCE_POINT1_TOPIC, 1);
    __publisher = handler_.advertise<geometry_msgs::Point>(LONGEST_DISTANCE_POINT2_TOPIC, 1);
    ROS_INFO_STREAM("initialized topics " << LONGEST_DISTANCE_POINT1_TOPIC << ", " << LONGEST_DISTANCE_POINT2_TOPIC);

    rotx << cos(pi),    0,               sin(pi),
            0,          1,               0,
            -1*sin(pi), 0,               cos(pi);
    roty << 1,          0,               0,
            0,          cos(193*pi/180), -1*sin(193*pi/180),
            0,          sin(193*pi/180), cos(193*pi/180);
    rot = roty*rotx;
}

PCDWrapper::~PCDWrapper(void) { } 

auto PCDWrapper::longest_distance(vector<Point3D>& _pcd) {

    int     i         = 1;
    double  distance,
            __distance__;
    
    _pcd.insert(_pcd.begin(), Point3D(MAX_FOV_REALSENSE_X, 0, MAX_FOV_REALSENSE_Z));
    _pcd.push_back(Point3D(-1*MAX_FOV_REALSENSE_X, 0, MAX_FOV_REALSENSE_Z));

    Point3D ld_point1 = _pcd.at(0),
            ld_point2 = _pcd.at(1);
    
    struct __return {         
        double __1;
        Point3D __2, __3;
    };

    std::sort(_pcd.begin(), _pcd.end());
    distance = std::abs(_pcd.at(0).x-_pcd.at(1).x);

    for ( ; i+1 < _pcd.size(); i++) {
        __distance__ = std::abs(_pcd.at(i).x-_pcd.at(i+1).x);
        if (__distance__ > distance) {
            ld_point1 = _pcd.at(i);
            ld_point2 = _pcd.at(i+1);
            distance = __distance__;
        }
    }
    return __return{distance, ld_point1, ld_point2};
}

void PCDWrapper::timer_callback(const ros::TimerEvent& _event) {

    static vector<Point3D> pcd;
    static size_t          _count   = 0;
    static size_t          _hash    = 0;
    size_t                 __hash__ = 0,
                           _size;
    int                    i;
    std::string            line,
                           raw_data;

    if (_count++ == 0) {
        ROS_INFO("callback from timer");
    }

    const std::string target_path(PCD_FOLDER);
    vector<string>    entries, __raw_data__;
    
    for (auto &entry : boost::make_iterator_range(directory_iterator(target_path), {})
         | filtered(static_cast<bool (*)(const path &)>(&is_regular_file))
         | filtered([&](const path &_path){
                        return _path.filename().string().rfind("locale_pcd", 0) == 0; 
                      })
        ) {
        entries.push_back(entry.path().filename().string());
    }

    sort(entries.begin(), entries.end());

    if ((__hash__ = std::hash<std::string>{}(entries.back())) != _hash) {
        ROS_INFO_STREAM("detected new pcd " << entries.back() << ", " << __hash__);
	_hash = __hash__;
    }

    std::ifstream pcd_file(std::string(PCD_FOLDER) + "/" + entries.back());
    std::getline(pcd_file, raw_data); // all the data are just in the first line
    pcd_file.close();

    boost::split(__raw_data__, raw_data, boost::is_any_of(","));

    pcd.clear();
    // get the pcd data point by point and perform basic rotations
    for (i = 0; i+2 < __raw_data__.size(); i += 3) // {
        pcd.push_back(Point3D(std::atof(__raw_data__.at(i).c_str()),
                              std::atof(__raw_data__.at(i+1).c_str()),
                              std::atof(__raw_data__.at(i+2).c_str()))*rot);
        // ROS_INFO_STREAM("pcd data " << i/3 << " is " << pcd.back().x << ", " << pcd.back().y << ", " << pcd.back().z);
        // }    

    // filtering the restults, i.e., removing points that are above rocker bogie...
    _size = pcd.size();
    filter(Filter::height|Filter::distance|Filter::duplicates|Filter::origin, pcd);
    ROS_INFO_STREAM("pcd size before filtering " << _size << ", after " << pcd.size());

    // finding the two points with the longest possible distance
    // if (pcd.size() < 2) {
    //     ROS_WARN("not enough points after filtering to find the logest distance");
    //     return;
    // }
    auto [distance, ld_point1, ld_point2] = longest_distance(pcd);
    ROS_INFO_STREAM("pcd points with the highest distance (" << distance << "), are " 
                    << ld_point1.x << ", " << ld_point1.y << ", " << ld_point1.z <<  " and "
                    << ld_point2.x << ", " << ld_point2.y << ", " << ld_point2.z
                   );
    auto ros_point = [](Point3D& point) -> geometry_msgs::Point {
        geometry_msgs::Point _ros_point;
        _ros_point.x = point.x;
        _ros_point.y = point.y;
        _ros_point.z = point.z;
        return _ros_point;
    };

     _publisher.publish(ros_point(ld_point1));
     __publisher.publish(ros_point(ld_point2));
    ROS_INFO("points with highest distance are now published");

}

void PCDWrapper::filter(const Filter _filter, vector<Point3D>& _pcd) {

    auto lowest_height = [](const auto &point, const auto &point_) {
        return point.y < point_.y;
    };

    double min_height = (*std::min_element(std::begin(_pcd), std::end(_pcd), lowest_height)).y;
    ROS_INFO_STREAM("min height for filtering: " << min_height);

    auto _0_2_4_filter = [&min_height](Point3D& point, bool _0_filter, bool _2_filter, bool _4_filter) -> 
        bool { 
            
            return _0_filter*(
                              point.x == 0 && point.y == 0 && point.z == 0
                             ) ||
                   _2_filter*(
                              point.y > ROCKER_BOGIE_MAX_HEIGHT ||
                              point.y < min_height+ROCKER_BOGIE_MAX_HEIGHT
                             ) ||
                   _4_filter*(sqrt(pow(point.x, 2)+pow(point.y, 2)+pow(point.z, 2)) > POINT_MAX_DISTANCE);
        };
    string string_filter = std::bitset<5>(_filter).to_string();
    
    if (string_filter[4] == '1' || string_filter[2] == '1' || string_filter[0] == '0') {
        _pcd.erase(std::remove_if(_pcd.begin(), _pcd.end(), [&_0_2_4_filter, &string_filter](Point3D& point) { 
                                      return _0_2_4_filter(point, string_filter[0] == '1', string_filter[2] == '1', string_filter[4] == '1');
                                  }
                                 ), _pcd.end()
                  );
        string_filter[0] = '0';
        string_filter[2] = '0';
        string_filter[4] = '0';
    } 

    if (string_filter[1] == '1') {
        _pcd.erase(std::unique(_pcd.begin(), _pcd.end()), _pcd.end());
        string_filter[1] = '0';
    }

    if (string_filter != "00000") {
        throw std::logic_error("filter not yet implemented");   
    }
}

int main(int argc, char ** argv) {
    
    ROS_INFO("started pointcloud depth wrapper node");
    ros::init(argc, argv, NODE_PCD_WRAPPER);

    ROS_INFO("initialized pointcloud depth wrapper node");

    auto pc = new PCDWrapper();
    ros::spin();
    
    return 0;
}


