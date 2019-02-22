#ifndef REGION_PATH_H
#define REGION_PATH_H

#include <string>
#include <vector>
#include "verbal_navigation/Region.h"
#include "verbal_navigation/DomainInfo.h"

class RegionPath{
public:
	std::vector<Region> path;
	std::string floor_id;
	// Domains go here
};

#endif
