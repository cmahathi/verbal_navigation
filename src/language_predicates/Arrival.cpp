#include <verbal_navigation/language_predicates/Arrival.h>

Arrival::Arrival(std::string name, bool rt, bool e, int f)  : Instruction(name), isRobotTransition(rt), isElevator(e), floorNum(f) { }

std::string Arrival::toNaturalLanguage() {
	std::string directionString;
	
	switch (direction)
	{
    	case Directions::NONE : directionString = "";
                            break;
    	case Directions::STRAIGHT : directionString = "straight ahead";
                                break;
   		case Directions::LEFT : directionString = "on your left";
                            break;
    	case Directions::RIGHT : directionString = "on your right";
                             break;
	}

	if (isRobotTransition) {
		return "The next robot will be waiting in the " + name + " " + directionString + ".";
	}
	if (isElevator) {
		return "Take the elevator " + directionString + " to floor " + std::to_string(floorNum) + "."; 
	}
	return name + " will be " + directionString + "!";
}

void Arrival::addChild (Preposition p) {
  children.push_back(p);
}

void Arrival::addDirection(Directions dir) {
  direction = dir;
}
