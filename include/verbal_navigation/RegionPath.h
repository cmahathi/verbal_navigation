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

	bool hasRegion(const std::string& regionName) {
		return getRegion(regionName) != NULL;
	}

	Region* getRegion(const std::string& regionName) {
		for(int i = 0; i < path.size(); ++i) {
			if(path.at(i).getName() == regionName) {
				return &path.at(i);
			}
		}
		return NULL;
	}
};

#endif
