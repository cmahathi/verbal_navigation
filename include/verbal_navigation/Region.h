#ifndef REGION
#define REGION

#include <vector>
#include <string>

#include <geometry_msgs/PoseStamped.h>

class Region {

// Region Type:
// 0: Room
// 1: Hallway
// 2: Open Space
// 3: Elevator

protected:
  std::string name;
  std::string common_name;
  bool has_door; 
  double length;
  int type;
  std::vector<geometry_msgs::PoseStamped> path;

public:
  Region (std::string name);
  void setDoor (bool door);
  void setName (std::string n);
  void setCommonName (std::string cn);
  void setLength (double l);
  void setType (int t);

  void appendPoseToPath(geometry_msgs::PoseStamped pose);

  std::vector<geometry_msgs::PoseStamped> getPath();
  bool getDoor ();
  std::string getName ();
  std::string getCommonName ();
  double getLength ();
  int getType ();
  
};
#endif