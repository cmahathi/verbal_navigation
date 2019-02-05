#ifndef REGION
#define REGION

#include <vector>
#include <string>

class Region {

protected:
  std::string name;
  std::string common_name;
  bool has_door;  

public:
  Region (std::string name);
  void setDoor (bool door);
  void setName (std::string n);
  void setCommonName (std::string cn);
  
};
#endif