#include <iostream>

#include "pointCloudProcessor.h"
#include <Eigen/Dense>

int main() {

    // Provide the directories and filenames as input vectors
    std::vector<std::string> imgPaths = { "input/C1_S5.bmp" };
    std::vector<std::string> plyPaths = { "input/sphereCalibScan5_reduced.ply" };

    PointCloudProcessor pointCloudProcessor(plyPaths);

    pointCloudProcessor.displayPointClouds();

    int iterations = 10000;
    int adjacencyThreshold = 30;
    float ransacThreshold = 0.05;
    float rMin = 0.1;
    float rMax = 0.5;
    std::vector<Eigen::Vector3f> inliers;
    Eigen::Vector3f S0;
    float r;
    std::vector<int> inlierIndices;
    
    // Rand4p method
    //pointCloudProcessor.detectSphereRand4p(iterations, ransacThreshold, rMin, rMax, inliers, S0, r, inlierIndices);
    
    // Adjacency method
    pointCloudProcessor.detectSphereWithAdjacency(iterations, adjacencyThreshold, ransacThreshold, rMin, rMax, inliers, S0, r, inlierIndices);
    pointCloudProcessor.colorInlierIndicesRed(inlierIndices);
    pointCloudProcessor.writePlys();

    return 0;
}
