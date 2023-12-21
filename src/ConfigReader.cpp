#include "ConfigReader.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

ConfigReader::ConfigReader(const std::string& configFile) {
    std::ifstream ifs(configFile);
    if (!ifs.is_open()) {
        throw std::runtime_error("Failed to open config file.");
    }

    nlohmann::json configJson;
    ifs >> configJson;

    // Extract parameters from JSON
    xyzPaths = configJson.value("xyzPaths", xyzPaths);
    imgPaths = configJson.value("imgPaths", imgPaths);
    ransacTypeLid = configJson.value("ransacTypeLid", ransacTypeLid);
    ransacThresholdLid = configJson.value("ransacThresholdLid", ransacThresholdLid);
    rMinLid = configJson.value("rMinLid", rMinLid);
    rMaxLid = configJson.value("rMaxLid", rMaxLid);
    iterationsRand4p = configJson.value("iterationsRand4p", iterationsRand4p);
    iterationsAdjacency = configJson.value("iterationsAdjacency", iterationsAdjacency);
    adjacencyThreshold = configJson.value("adjacencyThreshold", adjacencyThreshold);
    iterationsDistance = configJson.value("iterationsDistance", iterationsDistance);
    distanceThreshold = configJson.value("distanceThreshold", distanceThreshold);
    iterationsCam = configJson.value("iterationsCam", iterationsCam);
    ransacTypeCam = configJson.value("ransacTypeCam", ransacTypeCam);
    edgeDetect = configJson.value("edgeDetect", edgeDetect);
    edgeThreshold = configJson.value("edgeThreshold", edgeThreshold);
    fu = configJson.value("fu", fu);
    fv = configJson.value("fv", fv);
    u0 = configJson.value("u0", u0);
    v0 = configJson.value("v0", v0);

    // Check if S0CamGT is present
    if (configJson.contains("S0CamGT")) {
        auto S0CamGTOriginal = configJson["S0CamGT"];
        for (const auto& item : S0CamGTOriginal) {
            if (item.is_array() && item.size() == 3) {
                Eigen::Vector3f sphereCenters(item[0], item[1], item[2]);
                S0CamGT.push_back(sphereCenters);
            }
            else {
                std::cerr << "Error: Invalid JSON array for Eigen::Vector3f in S0CamGT\n";
            }
        }
    }

    // Check if S0LidGT is present
    if (configJson.contains("S0LidGT")) {
        auto S0LidGTOriginal = configJson["S0LidGT"];
        for (const auto& item : S0LidGTOriginal) {
            if (item.is_array() && item.size() == 3) {
                Eigen::Vector3f sphereCenters(item[0], item[1], item[2]);
                S0LidGT.push_back(sphereCenters);
            }
            else {
                std::cerr << "Error: Invalid JSON array for Eigen::Vector3f in S0LidGT\n";
            }
        }
    }

    // Check if rGT is present
    if (configJson.contains("rGT")) {
        rGT = configJson["rGT"];
    }

    // Check if rotGT is present
    if (configJson.contains("rotGT")) {
        auto rotGTFloat = configJson["rotGT"];
        for (int i = 0; i < 3; ++i) {
            // Check that the vector has enough elements
            if (i < rotGTFloat.size() && rotGTFloat[i].is_array() && rotGTFloat[i].size() == 3) {
                rotGT.row(i) << rotGTFloat[i][0], rotGTFloat[i][1], rotGTFloat[i][2];
            }
            else {
                std::cerr << "Error: Invalid JSON array for Eigen::MatrixXf at row " << i << " in rotGT\n";
            }
        }
    }

    // Check if transGT is present
    if (configJson.contains("transGT")) {
        auto transGTOriginal = configJson["transGT"];
        if (transGTOriginal.is_array() && transGTOriginal.size() == 3) {
            transGT << transGTOriginal[0], transGTOriginal[1], transGTOriginal[2];
        }
        else {
            std::cerr << "Error: Invalid JSON array for Eigen::Vector3f in transGT\n";
        }
    }

    ifs.close();

    // Calculate parameters based on config
    if (rMaxLid == 0) { rMaxLid = 1; }
    ransacThresholdCam = 4 / fu;
    rMinCam = 10 / fu;
    rMaxCam = u0 / fu;
}
