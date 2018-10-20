#ifndef VERB_PHRASE
#define VERB_PHRASE

#include <vector>
#include <string>
#include "verbal_navigation/Instruction.h"
#include "verbal_navigation/Preposition.h"

enum class Directions {NONE, STRAIGHT, LEFT, RIGHT};

class VerbPhrase : Instruction {

public:
  VerbPhrase(std::string name, Directions dir);
  Directions direction;
  std::vector<Preposition> children;
  virtual std::string toNaturalLanguage();
  void addChild(Preposition p);
};
#endif
