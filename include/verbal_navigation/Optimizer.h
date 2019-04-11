#include "verbal_navigation/MapInfo.h"
#include "verbal_navigation/DomainInfo.h"
#include "verbal_navigation/Actions.h"
#include <limits>

class Optimizer {

    private:
        static constexpr double ROBOT_VELOCITY = 0.5;
        static constexpr double HUMAN_VELOCITY = 1.4;
        static constexpr double SPEECH_TIME = 3.0;
        std::vector<Region>& segmentedPath;
        MapInfo floor2;
        MapInfo floor3;
        DomainInfo domains;
        double currentMinTime;
        std::vector<GuidanceActionTypes> currentPath;
        std::vector<GuidanceActionTypes> currentMinPath;
        bool optimized;
        bool debug;

        void preprocess();
        double getLengthOfRegion(Region r, MapInfo mapinfo);
        double calculateTraversibility (Region r);
        void printPathInfo();
        void calculateRobotTimes();
        void calculateBaseHumanTimes();
        void calculateRegionTime(double accumulatedTime, int numInstructedRegions, int regionCounter, GuidanceActionTypes action, bool transition);
        void updatePath(GuidanceActionTypes action);
        void backtrackPath();
        double calculateAccumulatedTime(double accumulatedTime, int numInstructedRegions, int regionCounter, GuidanceActionTypes action);
        void updateMin(double accumulatedTime);
        bool domainTransition(int regionCount);
        std::string pathToString (std::vector<GuidanceActionTypes> path);




    public:
        Optimizer(RegionPath& regionPath, MapInfo f2, MapInfo f3);
        void optimize();

        std::vector<GuidanceActionTypes> getOptimalGuidanceSequence();
};
