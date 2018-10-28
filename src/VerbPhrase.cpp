#include "verbal_navigation/VerbPhrase.h"

VerbPhrase::VerbPhrase(std::string name) : Instruction(name) { }

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

void VerbPhrase::addChild (Preposition p) {
  children.push_back(p);
}

void VerbPhrase::addDirection(Directions dir) {
  direction = dir;
}

void VerbPhrase::addPreposition(Preposition prep){
  children.push_back(prep);
}


void VerbPhrase::setStartRegion(std::string region){
  startRegion = region;

}

void VerbPhrase::setEndRegion(std::string region){
  endRegion = region;
}
