#ifndef PREPOSITION
#define PREPOSITION

#include <vector>
#include <string>
#include "verbal_navigation/Instruction.h"

class Preposition : public Instruction {

public:
  Preposition(std::string name, std::string landmark);
  std::string landmark;
  virtual std::string toNaturalLanguage();
};
#endif
