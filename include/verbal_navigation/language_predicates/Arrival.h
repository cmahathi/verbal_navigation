#ifndef ARRIVAL
#define ARRIVAL

#include <vector>
#include <string>
#include "verbal_navigation/language_predicates/Instruction.h"
#include "verbal_navigation/language_predicates/Preposition.h"

class Arrival : public Instruction {

	Directions direction;
	bool isRobotTransition;
	bool isElevator;
	int floorNum;
	std::vector<Preposition> children;

	public:
		Arrival(std::string name, bool robotTransition, bool elevator, int floor);
		std::string toNaturalLanguage();
		void addChild(Preposition p);
		void addDirection(Directions dir);

};

#endif
