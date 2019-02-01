#ifndef PREDICATES
#define PREDICATES

#include <verbal_navigation/language_predicates/Arrival.h>
#include <verbal_navigation/language_predicates/Instruction.h>
#include <verbal_navigation/language_predicates/Preposition.h>
#include <verbal_navigation/language_predicates/VerbPhrase.h>

// #include <string>
// #include "verbal_navigation/MapItem.h"

// enum class Directions {NONE, STRAIGHT, LEFT, RIGHT};

// class Instruction {

// protected:
//   std::string name;
//   Instruction(std::string name);

// public:
//   virtual std::string toNaturalLanguage() = 0;

//   bool operator==(const Instruction& rhs) { return this->name == rhs.name; }
// };

// class Preposition : public Instruction {

// public:
//   Preposition(std::string name, MapItem landmark);
//   MapItem landmark;
//   std::string toNaturalLanguage();

// };

// class VerbPhrase : public Instruction {

//   Directions direction;
//   std::vector<Preposition> children;
//   std::string startRegion;
//   std::string endRegion;
  
// public:
//   VerbPhrase(std::string name);
//   std::string toNaturalLanguage();
//   std::string getDirectionString();
//   void addDirection(Directions dir);
//   void setStartRegion(std::string region);
//   void setEndRegion(std::string region);
//   void addPreposition(Preposition prep);
// };

// class Arrival : public Instruction {

// 	Directions direction;
// 	std::vector<Preposition> children;

// 	public:
// 	Arrival(std::string name);
// 	std::string toNaturalLanguage();
// 	void addChild(Preposition p);
//     void addDirection(Directions dir);

// };

#endif