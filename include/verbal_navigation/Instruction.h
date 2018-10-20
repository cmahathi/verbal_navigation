#ifndef INSTRUCTION
#define INSTRUCTION

#include <vector>
#include <string>


class Instruction {

protected:
  std::string name;
public:
  Instruction(std::string name);
  virtual std::string toNaturalLanguage() = 0;
};
#endif
