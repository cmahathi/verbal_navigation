#ifndef PREPOSITION
#define PREPOSITION

#include <vector>
#include <string>
#include "verbal_navigation/Instruction.h"
#include "verbal_navigation/Landmark.h"

class Preposition : public Instruction {

public:
  Preposition(std::string name, Landmark landmark);
  Landmark landmark;
  std::string toNaturalLanguage();

};
#endif
