#include <vector>
#include <string>

enum class Directions {NONE, STRAIGHT, LEFT, RIGHT};

class Instruction {
  protected:
    std::string name;

  public:
    Instruction(std::string name) : name(name) {}
    virtual std::string toNaturalLanguage() = 0;
};

class VerbPhrase : public Instruction {
  Directions direction;
  std::vector<Preposition> children;
  std::string startRegion;
  std::string endRegion;

  public:
    VerbPhrase(std::string name) : Instruction(name) {}
    std::string toNaturalLanguage() {
        std::string result = name;
        std::string directionString;
        switch (direction)
        {
          case Directions::NONE : directionString = "";
                                  break;
          case Directions::STRAIGHT : directionString = "straight";
                                      break;
          case Directions::LEFT : directionString = "left";
                                  break;
          case Directions::RIGHT : directionString = "right";
                                   break;
        }
        result += " " + directionString;
        for (Preposition &child : children) {
          result += " " + child.toNaturalLanguage();
        }
        result += ".\n";
        return result;
    }
    void addChild(Preposition p) {
      children.push_back(p);
    }
    void addDirection(Directions dir) {
      direction = dir;
    }
    void setStartRegion(std::string region) {
      startRegion = region;
    }
    void setEndRegion(std::string region) {
      endRegion = region;
    }
};

class Preposition : public Instruction {
  std::string landmark;

  public:
    Preposition(std::string name, std::string landmark) :
      Instruction(name), landmark(landmark) {
    }

    std::string toNaturalLanguage() {
      return name + " the " + landmark;
    }
}

class Arrival : public Instruction {
  Directions direction;
  std::vector<Preposition> children;

  public:
    Arrival(std::string name) : Instruction(name) { }

    std::string toNaturalLanguage() {
    	std::string directionString;
    	switch (direction)
    	{
        	case Directions::NONE : directionString = "";
                                break;
        	case Directions::STRAIGHT : directionString = "straight ahead";
                                    break;
       		case Directions::LEFT : directionString = "on your left";
                                break;
        	case Directions::RIGHT : directionString = "on your right";
                                 break;
    	}

    	return name + " is " + directionString;
    }

    void addChild (Preposition p) {
      children.push_back(p);
    }

    void addDirection(Directions dir) {
      direction = dir;
    }

}
