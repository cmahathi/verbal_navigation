#ifndef REGION_H
#define REGION_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include "verbal_navigation/Predicates.h"

enum class RegionType {ROOM, HALLWAY, OPEN_SPACE, ELEVATOR};

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
  int floor;
  RegionType type;
  int num_neighbors;
  double traversibility;
  std::vector<geometry_msgs::PoseStamped> path;
  std::shared_ptr<Instruction> instruction;

public:
  double robot_time;
  double base_human_time;
  double human_time;

  Region (std::string name);

  void setDoor (bool door);
  void setName (std::string n);
  void setCommonName (std::string cn);
  void setLength (double l);
  void setType (int t);
  void setNumNeighbors(int n);
  void setTraversibility(double n);
  void setFloor(int f);
  void setInstruction(std::shared_ptr<Instruction> i);

  void appendPoseToPath(geometry_msgs::PoseStamped pose);

  std::vector<geometry_msgs::PoseStamped> getPath();
  bool getDoor ();
  std::string getName ();
  std::string getCommonName ();
  double getLength ();
  RegionType getType ();
  int getNumNeighbors();
  double getTraversibility();
  int getFloor();
  std::shared_ptr<Instruction> getInstruction();
};
#endif
