#include <verbal_navigation/language_predicates/Arrival.h>

Arrival::Arrival(std::string name, bool robotTransition, bool elevator, int floor)  : Instruction(name), isRobotTransition(robotTransition), isElevator(elevator), floorNum(floor) { }

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

	std::string message = "";
	if (isElevator) {
		message += "Take the elevator " + directionString + " to floor " + std::to_string(floorNum) + "."; 
	}
	if (isRobotTransition) {
		message += "The next robot will be waiting in the " + name + " " + directionString + ".";
	}
	if(message.empty()) {
		message = name + " will be " + directionString + "!";
	}
	return message;
}

void Arrival::addChild (Preposition p) {
  children.push_back(p);
}

void Arrival::addDirection(Directions dir) {
  direction = dir;
}
