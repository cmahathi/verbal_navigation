#include <verbal_navigation/language_predicates/Arrival.h>

Arrival::Arrival(std::string name) : Instruction(name) { }

std::string Arrival::toNaturalLanguage() {
	std::string directionString;
	//TODO WOO Hard coding.
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

	return name + " is " + directionString + "! ;)";
}

void Arrival::addChild (Preposition p) {
  children.push_back(p);
}

void Arrival::addDirection(Directions dir) {
  direction = dir;
}
