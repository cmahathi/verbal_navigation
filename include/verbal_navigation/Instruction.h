#ifndef INSTRUCTION
#define INSTRUCTION

#include <vector>
#include <string>

enum class Directions {NONE, STRAIGHT, LEFT, RIGHT};

class Instruction {

protected:
  std::string name;
  Instruction(std::string name);

public:
  virtual std::string toNaturalLanguage() = 0;

  bool operator==(const Instruction& rhs) { return this->name == rhs.name; }
};
#endif
