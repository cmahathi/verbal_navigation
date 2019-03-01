#include "verbal_navigation/MapInfo.h"
#include "verbal_navigation/DomainInfo.h"
#include <limits>

class Optimizer {

    private:
        static constexpr double ROBOT_VELOCITY = 0.5;
        static constexpr double HUMAN_VELOCITY = 1.4;
        static constexpr double SPEECH_TIME = 5.0;
        std::vector<Region>& segmentedPath;
        MapInfo floor2;
        MapInfo floor3;
        DomainInfo domains;
        double currentMinTime;
        std::vector<char> currentPath;
        std::vector<char> currentMinPath;
        bool debug;

        void preprocess();
        double getLengthOfRegion(Region r, MapInfo mapinfo);
        double calculateTraversibility (Region r);
        void printPathInfo();
        void calculateRobotTimes();
        void calculateBaseHumanTimes();
        void calculateRegionTime(double accumulatedTime, int numInstructedRegions, int regionCounter, char action, bool transition);
        void updatePath(char action);
        void backtrackPath();
        double calculateAccumulatedTime(double accumulatedTime, int numInstructedRegions, int regionCounter, char action);
        void updateMin(double accumulatedTime);
        bool domainTransition(int regionCount);
        std::string pathToString (std::vector<char> path);




    public:
        Optimizer(RegionPath& regionPath, MapInfo f2, MapInfo f3);
        void optimize();
};
