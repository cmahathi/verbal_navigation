#ifndef INSTRUCTION
#define INSTRUCTION

#include <vector>
#include <string>

enum class Directions {NONE, STRAIGHT, LEFT, RIGHT};

class Instruction {

protected:
  std::string name;

public:
  Instruction(std::string name);
  virtual std::string toNaturalLanguage() = 0;
};
#endif
