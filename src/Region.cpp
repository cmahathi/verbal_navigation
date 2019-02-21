#include <verbal_navigation/Region.h>

Region::Region (std::string n) : name(n) {
    has_door = false;
    common_name = name;
}
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

void Region::setFloor (int f) {
    floor = f;
}

void Region::setType (int t) {
    switch (t) {
        case 0: type = RegionType::ROOM;
                break;
        case 1: type = RegionType::HALLWAY;
                break;
        case 2: type = RegionType::OPEN_SPACE;
                break;
        case 3: type = RegionType::ELEVATOR;
                break;
    }
}

void Region::setNumNeighbors (int n) {
    num_neighbors = n;
}

void Region::setTraversibility(double n){
    traversibility = n;
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

RegionType Region::getType () {
    return type;
}

int Region::getFloor () {
    return floor;
}

double Region::getTraversibility(){
    return traversibility;
}

int Region::getNumNeighbors () {
    return num_neighbors;
}
