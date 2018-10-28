#ifndef VERB_PHRASE
#define VERB_PHRASE

#include <vector>
#include <string>
#include "verbal_navigation/Instruction.h"
#include "verbal_navigation/Preposition.h"

enum class Directions {NONE, STRAIGHT, LEFT, RIGHT};

class VerbPhrase : public Instruction {

  Directions direction;
  std::vector<Preposition> children;
  std::string startRegion;
  std::string endRegion;

public:
  VerbPhrase(std::string name);

  virtual std::string toNaturalLanguage();
  void addDirection(Directions dir);
  void setStartRegion(std::string region);
  void setEndRegion(std::string region);
  void addPreposition(Preposition prep);
};
#endif
