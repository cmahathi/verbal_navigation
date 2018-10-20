#include "verbal_navigation/VerbPhrase.h"

VerbPhrase::VerbPhrase(std::string name) : Instruction(name) {

}

std::string VerbPhrase::toNaturalLanguage() {
  std::string result = name;
  std::string directionString;
  switch (direction)
  {
    case Directions::NONE : directionString = "";
    case Directions::STRAIGHT : directionString = "straight";
    case Directions::LEFT : directionString = "left";
    case Directions::RIGHT : directionString = "right";
  }
  result += " " + directionString;
  for (Preposition &child : children) {
    result += " " + child.toNaturalLanguage();
  }
  result += ".\n";
  return result;
}
