#include "verbal_navigation/Preposition.h"


Preposition::Preposition(std::string name, Landmark landmark) :
  Instruction(name), landmark(landmark) {
}


std::string Preposition::toNaturalLanguage() {
  return name + " the " + landmark.getName();
}
