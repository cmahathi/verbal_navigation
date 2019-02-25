#include "verbal_navigation/MapInfo.h"
#include "verbal_navigation/DomainInfo.h"

class Optimizer {

    private:
        std::vector<Region>& segmentedPath;
        MapInfo floor2;
        MapInfo floor3;
        DomainInfo domains;
        static constexpr double ROBOT_VELOCITY = 0.5;
        static constexpr double HUMAN_VELOCITY = 1.4;

        void preprocess();
        double getLengthOfRegion(Region r, MapInfo mapinfo);
        double calculateTraversibility (Region r);
        void printPathInfo();
        void calculateRobotTimes();
        void calculateBaseHumanTimes();

    public:
        Optimizer(RegionPath& regionPath, MapInfo f2, MapInfo f3);
        void optimize();
};
