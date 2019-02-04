#include "verbal_navigation/bwi_directions_generator.h"

#include <bwi_logical_translator/bwi_logical_translator.h>
namespace bwi_directions_generator {
    BwiDirectionsGenerator::BwiDirectionsGenerator() {
    }

    MapInfo BwiDirectionsGenerator::GenerateDirectionsForPathOnMap(std::vector<geometry_msgs::PoseStamped> path, fs::path mapFile, std::string destinationName) {
        bwi_logical_translator::BwiLogicalTranslator translator;
        ros::param::set("~map_file", mapFile.string());
	    boost::filesystem::path dataPath = mapFile.remove_filename();
        ros::param::set("~data_directory", dataPath.string());
        translator.initialize();

		return MapInfo(translator, path, destinationName);
    }
}