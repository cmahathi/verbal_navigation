#include <verbal_navigation/Region.h>

Region::Region (std::string n) : name(n), has_door(false), common_name(n) { }

void Region::setDoor (bool door) {
    has_door = door;
}

void Region::setName (std::string n) {
    name = n;
}

void Region::setCommonName (std::string cn) {
    common_name = cn;
}

void Region::setLength (double l) {
    length = l;
}

void Region::setType (int t) {
    type = t;
}

void Region::appendPoseToPath(geometry_msgs::PoseStamped pose) {
    path.push_back(pose);
}

std::vector<geometry_msgs::PoseStamped> Region::getPath() {
    return path;
}


bool Region::getDoor () {
    return has_door;
}

std::string Region::getName () {
    return name;
}

std::string Region::getCommonName () {
    return common_name;
}

double Region::getLength () {
    return length;
}

int Region::getType () {
    return type;
}
