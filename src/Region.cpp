#include <verbal_navigation/Region.h>

Region::Region (std::string n) : name(n), has_door(false), common_name(n),
                        robot_time(0), human_time(0), base_human_time(0), 
                        length(0), floor(0), num_neighbors(0), traversibility(0), instruction(nullptr) { }

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

void Region::setFloor(int f) {
    floor = f;
}

void Region::setInstruction(std::shared_ptr<Instruction> i) {
    instruction = i;
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

void Region::appendPoseToPath(geometry_msgs::PoseStamped pose) {
    path.push_back(pose);
}

std::vector<geometry_msgs::PoseStamped> Region::getPath() {
    return path;
}

int Region::getFloor() {
    return floor;
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

double Region::getTraversibility(){
    return traversibility;
}

int Region::getNumNeighbors () {
    return num_neighbors;
}

std::shared_ptr<Instruction> Region::getInstruction() {
    return instruction;
}

geometry_msgs::Pose Region::getInitialPose() {
    return path.at(0).pose;
}
geometry_msgs::Pose Region::getEndPose() {
    return path.at((int)(path.size()*.9)).pose;
}