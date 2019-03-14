#include "verbal_navigation/GuidanceAction.h"

GuidanceAction::GuidanceAction(std::vector<Region> regions, GuidanceActionTypes t) : regions(regions), type(t){}

std::vector<Region> GuidanceAction::getRegions() {
    return regions;
}

GuidanceActionTypes GuidanceAction::getType() {
    return type;
}