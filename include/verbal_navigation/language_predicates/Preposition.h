#ifndef PREPOSITION
#define PREPOSITION

#include <vector>
#include <string>
#include "verbal_navigation/language_predicates/Instruction.h"
#include "verbal_navigation/MapItem.h"

class Preposition : public Instruction {

public:
  Preposition(std::string name, MapItem landmark);
  MapItem landmark;
  std::string toNaturalLanguage();

};
#endif
