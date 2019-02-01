#ifndef VERB_PHRASE
#define VERB_PHRASE

#include <vector>
#include <string>
#include "verbal_navigation/language_predicates/Instruction.h"
#include "verbal_navigation/language_predicates/Preposition.h"

class VerbPhrase : public Instruction {

  Directions direction;
  std::vector<Preposition> children;
  std::string startRegion;
  std::string endRegion;
  
public:
  VerbPhrase(std::string name);
  std::string toNaturalLanguage();
  std::string getDirectionString();
  void addDirection(Directions dir);
  void setStartRegion(std::string region);
  void setEndRegion(std::string region);
  void addPreposition(Preposition prep);
};
#endif
