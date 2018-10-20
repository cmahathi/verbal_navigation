#include "verbal_navigation/VerbPhrase.h"



VerbPhrase::VerbPhrase(std::string name) (Instruction(name)) {

}

std::string VerbPhrase::toNaturalLanguage() {
  std::string result = name;
  result += " " + direction;
  for (Preposition &child : children) {
    result += " " + child.toNaturalLanguage();
  }
  result += ".\n";
  return result;
}
