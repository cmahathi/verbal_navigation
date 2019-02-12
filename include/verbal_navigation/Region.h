#ifndef REGION
#define REGION

#include <vector>
#include <string>

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
  int type;

public:
  Region (std::string name);
  void setDoor (bool door);
  void setName (std::string n);
  void setCommonName (std::string cn);
  void setLength (double l);
  void setFloor (int f);
  void setType (int t);

  bool getDoor ();
  std::string getName ();
  std::string getCommonName ();
  double getLength ();
  int getType ();
  
};
#endif