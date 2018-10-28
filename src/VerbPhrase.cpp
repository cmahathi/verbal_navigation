#include "verbal_navigation/VerbPhrase.h"

VerbPhrase::VerbPhrase(std::string name) : Instruction(name) { }

// converts VerbPhrase to natural language by concatenating name, direciton,
// and list of predicates
std::string VerbPhrase::toNaturalLanguage() {
  std::string result = name;
  std::string directionString;
  switch (direction)
  {
    case Directions::NONE : directionString = "";
                            break;
    case Directions::STRAIGHT : directionString = "straight";
                                break;
    case Directions::LEFT : directionString = "left";
                            break;
    case Directions::RIGHT : directionString = "right";
                             break;
  }
  result += " " + directionString;
  for (Preposition &child : children) {
    result += " " + child.toNaturalLanguage();
  }
  result += ".\n";
  return result;
}


// aguments this VerbPhrase with a Direction enum (STRAIGHT, RIGHT, or LEFT)
void VerbPhrase::addDirection(Directions dir) {
  direction = dir;
}


// augments this VerbPhrase with a Preposition instruction
void VerbPhrase::addPreposition(Preposition prep){
  children.push_back(prep);
}


void VerbPhrase::setStartRegion(std::string region){
  startRegion = region;
}


void VerbPhrase::setEndRegion(std::string region){
  endRegion = region;
}
