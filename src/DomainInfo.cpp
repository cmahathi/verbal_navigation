#include "verbal_navigation/DomainInfo.h"

DomainInfo::DomainInfo() {
    read_domain_info();
}

void DomainInfo::read_domain_info () {
    ROS_INFO(boost::filesystem::current_path().string().c_str());
    std::string fullPath = boost::filesystem::current_path().string() + filename;

    if (!boost::filesystem::exists(fullPath)) {
      ROS_INFO("DOMAIN FILE NOT FOUND\nRun from verbal_nav directory");
      return;
    }

    std::ifstream fin(fullPath.c_str());

    YAML::Node doc;
    doc = YAML::Load(fin);
    for (int i = 0; i < doc.size(); i++) {
        std::string robot_id = doc[i]["id"].as<std::string>();
        std::vector<std::string> regions;
        const YAML::Node region_node = doc[i]["regions"];
        for (int j = 0; j < region_node.size(); j++) {
            std::string region = (region_node[j].as<std::string>());
            regions.push_back(region);
            regionToRobotMap.emplace(std::make_pair(region, robot_id));
        }
        robotToDomainMap.emplace(std::make_pair(robot_id, regions));

    }

    ROS_INFO("Map size: %d", robotToDomainMap.size());
}

int DomainInfo::getNumRobots() {
    return robotToDomainMap.size();
}

std::vector<std::string> DomainInfo::getDomainByRobot (std::string robot) {
    return robotToDomainMap[robot];
}

std::vector<std::string> DomainInfo::getRobotList () {
    std::vector<std::string> robots;
    for (auto it = robotToDomainMap.begin(); it != robotToDomainMap.end(); ++it) {
        robots.push_back(it->first);
    }
    return robots;
}

std::string DomainInfo::getRobotByRegion (std::string region) {
    return regionToRobotMap[region];
}

bool DomainInfo::isDomainTransition (std::string region1, std::string region2) {
    
    if (regionToRobotMap[region1].compare(regionToRobotMap[region2]) == 0) {
        //ROS_INFO("Regions %s and %s are in the same domain\n",region1.c_str(), region2.c_str());
        return false;
    }
    else {
        //ROS_INFO("Regions %s and %s are in different domains\n",region1.c_str(), region2.c_str());
        return true;
    }
}
