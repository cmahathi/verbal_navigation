#ifndef ARRIVAL
#define ARRIVAL

#include <vector>
#include <string>
#include "verbal_navigation/Instruction.h"
#include "verbal_navigation/Preposition.h"

class Arrival : public Instruction {

	Directions direction;
	std::vector<Preposition> children;
	
	public:
	Arrival(std::string name);
	virtual std::string toNaturalLanguage();
	void addChild(Preposition p);
    void addDirection(Directions dir);
	
};

#endif
