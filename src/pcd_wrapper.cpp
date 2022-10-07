
#include "../include/pcd_wrapper.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#define BOOST_RANGE_ENABLE_CONCEPT_ASSERT 0

using namespace boost::filesystem;
using namespace boost::adaptors;
using namespace ytcg;

using std::string;
using std::vector;
using std::time_t;

PCDWrapper::PCDWrapper(void) {
    timer_ = handler_.createTimer(ros::Duration(PCD_FREQ), timer_callback);
    ROS_INFO_STREAM("pointcloud depth wrapper subscribed to timer, triggering each " << std::to_string(PCD_FREQ) << " secs");
}

PCDWrapper::~PCDWrapper(void) { } 

void PCDWrapper::timer_callback(const ros::TimerEvent& _event) {
	
    static std::vector<PointCloud> pcd;
    static size_t                  _count   = 0;
    static size_t                  _hash    = 0;
    size_t                         __hash__ = 0;
    int                            i;
    std::string                    line, raw_data;

    if (_count++ == 0) {
        ROS_INFO("callback from timer");
    }

    const std::string target_path(PCD_FOLDER);
    vector<string>    entries, __raw_data__;
    
    for (auto &entry : boost::make_iterator_range(directory_iterator(target_path), {})
         | filtered(static_cast<bool (*)(const path &)>(&is_regular_file))
         | filtered([&](const path &_path){
                        return _path.filename().string().rfind("isdf_pcd_", 0) == 0; 
                      })
        ) {
	entries.push_back(entry.path().filename().string());
    }

    sort(entries.begin(), entries.end());

    if ((__hash__ = std::hash<std::string>{}(entries.back())) != _hash) {
        ROS_INFO_STREAM("detected new pcd " << entries.back() << ", " << __hash__);
        std::ifstream pcd_file(std::string(PCD_FOLDER) + "/" + entries.back());
	std::getline(pcd_file, raw_data); // all the data are just in the first line
	pcd_file.close();

	// ROS_INFO_STREAM("debug 1 " << raw_data);

	boost::split(__raw_data__, raw_data, boost::is_any_of(","));

	pcd.clear();
	for (i = 0; i+2 < __raw_data__.size(); i += 3) {
            pcd.push_back(PointCloud(std::atof(__raw_data__.at(i).c_str()),
                                     std::atof(__raw_data__.at(i+1).c_str()),
                                     std::atof(__raw_data__.at(i+2).c_str())));
	    ROS_INFO_STREAM("pcd data " << i/3 << " is " << pcd.back().z << ", " << pcd.back().y << ", " << pcd.back().z);
	}

	_hash = __hash__;
    }
}

int main(int argc, char ** argv) {
    
    ROS_INFO("started pointcloud wrapper node");
    ros::init(argc, argv, NODE_PCD_WRAPPER);

    ROS_INFO("initialized pose publisher node");

    auto pc = new PCDWrapper();
    ros::spin();
    
    return 0;
}


