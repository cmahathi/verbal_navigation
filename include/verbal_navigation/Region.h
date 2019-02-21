#ifndef REGION
#define REGION

#include <vector>
#include <string>

enum class RegionType {ROOM, HALLWAY, OPEN_SPACE, ELEVATOR};

class Region {

// Region Type:
// 0: Room
// 1: Hallway
// 2: Open Space
// 3: Elevator

public:
    std::string name;
    std::string common_name;
    bool has_door;
    double length;
    int floor;
    RegionType type;
    int num_neighbors;
    double traversibility;
    double robot_time;
    double base_human_time;
    double human_time;

    Region (std::string name);
    void setDoor (bool door);
    void setName (std::string n);
    void setCommonName (std::string cn);
    void setLength (double l);
    void setFloor (int f);
    void setType (int t);
    void setNumNeighbors(int n);
    void setTraversibility(double n);

    bool getDoor ();
    std::string getName ();
    std::string getCommonName ();
    double getLength ();
    RegionType getType ();
    int getFloor();
    int getNumNeighbors();
    double getTraversibility();

};
#endif
