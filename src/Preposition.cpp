#include "verbal_navigation/Preposition.h"


Preposition::Preposition(std::string name, MapItem landmark) :
  Instruction(name), landmark(landmark) {
}


std::string Preposition::toNaturalLanguage() {
  return name + " the " + landmark.getCommonName();
}