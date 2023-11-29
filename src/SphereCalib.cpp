#include <iostream>

#include "pointCloudProcessor.h"
#include "ImageProcessor.h"
#include <Eigen/Dense>

int main() {
    // Detect sphere centers from 3D data
    std::vector<std::string> plyPaths = { "input/sphereCalibScan5_reduced.ply" };

    PointCloudProcessor pointCloudProcessor(plyPaths);

    pointCloudProcessor.displayPointClouds();

    int iterations = 10000;
    int adjacencyThreshold = 30;
    float ransacThreshold = 0.05;
    float distanceThreshold = 0.15;
    float rMin = 0.1;
    float rMax = 0.5;
    std::vector<Eigen::Vector3f> inliersLid;
    Eigen::Vector3f S0Lid;
    float rLid;
    std::vector<int> inlierIndices;
    
    // Rand4p method
    //pointCloudProcessor.detectSphereRand4p(iterations, ransacThreshold, rMin, rMax, inliersLid, S0Lid, r, inlierIndices);
    
    // Adjacency method
    //pointCloudProcessor.detectSphereWithAdjacency(iterations, adjacencyThreshold, ransacThreshold, rMin, rMax, inliersLid, S0Lid, r, inlierIndices);

    // Distance method
    pointCloudProcessor.detectSphereWithDistance(iterations, distanceThreshold, ransacThreshold, rMin, rMax, inliersLid, S0Lid, rLid, inlierIndices);

    // Output result
    pointCloudProcessor.colorInlierIndicesRed(inlierIndices);
    pointCloudProcessor.writePlys();


    
    
    // Detect sphere centers from 2D data
    std::vector<std::string> imgPaths = { "input/C1_S5.bmp" };

    int edgeDetect = 1;
    float edgeThreshold = 0.25;
    int iterationsRCam = 10000;
    float fu = 1262.620252;
    float fv = 1267.36535;
    float u0 = 934.611657;
    float v0 = 659.520995;
    double ransacThresholdCam = 4 / fu; // n / fu means n pixel max error
    double rMinCam = 10 / fu;
    double rMaxCam = u0 / fu;
    int camRansac = 2;

    ImageProcessor imageProcessor(imgPaths, edgeDetect, edgeThreshold, iterationsRCam, ransacThresholdCam, rMinCam, rMaxCam, fu, fv, u0, v0, rLid, camRansac);

    Eigen::MatrixXf inliersCam;
    Eigen::Vector3f S0Cam;

    imageProcessor.detectSphereCenters(/* TODO RANSAC inliersCam, S0Cam */);

    return 0;
}
