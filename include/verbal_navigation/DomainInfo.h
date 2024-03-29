#ifndef DOMAIN_INFO
#define DOMAIN_INFO

#include "verbal_navigation/Region.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <libgen.h>
#include <stdexcept>
#include "ros/ros.h"
#include "ros/package.h"
#include <boost/filesystem.hpp>

class DomainInfo {
    private:
        std::map<std::string, std::vector<std::string>> robotToDomainMap;
        std::map<std::string, std::string> regionToRobotMap;
        const std::string filename = "/src/multimap/RobotDomains.yaml";
        void read_domain_info();
        
    public:
        DomainInfo ();
        int getNumRobots();
        std::vector<std::string> getDomainByRobot (std::string robot);
        std::vector<std::string> getRobotList ();
        std::string getRobotByRegion (std::string region);
        bool isDomainTransition (std::string region1, std::string region2);
};

#endif
