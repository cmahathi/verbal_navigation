#ifndef VERB_PHRASE
#define VERB_PHRASE

#include <vector>
#include <string>


class Preposition:Instruction {

public:
  Preposition(std::string name, std::string landmark);
  std::string landmark;
  std::string toNaturalLanguage();
};
#endif
