#ifndef VERB_PHRASE
#define VERB_PHRASE

#include <vector>
#include <string>

class VerbPhrase:Instruction {

public:
  enum class Directions = {NONE = NULL, STRAIGHT = "Straight", LEFT = "Left", RIGHT = "Right"};
  VerbPhrase(std::string name);
  Directions direction;
  std::vector<Preposition> children;
  std::string toNaturalLanguage();
};
#endif
