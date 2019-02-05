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
