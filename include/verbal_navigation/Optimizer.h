#include "verbal_navigation/MapInfo.h"

class Optimizer {

    private:
        std::vector<Region> regionPath;
        MapInfo floor2;
        MapInfo floor3;

        void preprocess();
        double getLengthOfRegion(Region r, MapInfo mapinfo);
        double distanceBetween(geometry_msgs::Pose firstPose, geometry_msgs::Pose lastPose);


    public:
        Optimizer(std::vector<Region> path, MapInfo f2, MapInfo f3);
        void optimize();
};
