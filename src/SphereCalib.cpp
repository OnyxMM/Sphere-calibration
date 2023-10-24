#include <iostream>

#include "pointCloudProcessor.h"

int main() {

    // Provide the directories and filenames as input vectors
    std::vector<std::string> imgPaths = { "input/C1_S5.bmp" };
    std::vector<std::string> plyPaths = { "input/sphereCalibScan5.ply" };

    PointCloudProcessor pointCloudProcessor(plyPaths);

    pointCloudProcessor.displayPointClouds();
    pointCloudProcessor.writePlys();

    return 0;
}
