#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <string>
#include <vector>
#include <Eigen/Dense>

class ConfigReader {
public:
    ConfigReader(const std::string& configFile);

    // Define getter methods for configuration parameters
    const std::vector<std::string>& getXYZPaths() const { return xyzPaths; }
    const std::vector<std::string>& getImgPaths() const { return imgPaths; }
    int getRansacTypeLid() const { return ransacTypeLid; }
    float getRansacThresholdLid() const { return ransacThresholdLid; }
    float getRMinLid() const { return rMinLid; }
    float getRMaxLid() const { return rMaxLid; }
    int getIterationsRand4p() const { return iterationsRand4p; }
    int getIterationsAdjacency() const { return iterationsAdjacency; }
    int getAdjacencyThreshold() const { return adjacencyThreshold; }
    int getIterationsDistance() const { return iterationsDistance; }
    float getDistanceThreshold() const { return distanceThreshold; }
    int getIterationsCam() const { return iterationsCam; }
    int getRansacTypeCam() const { return ransacTypeCam; }
    float getRansacThresholdCam() const { return ransacThresholdCam; }
    int getEdgeDetect() const { return edgeDetect; }
    float getEdgeThreshold() const { return edgeThreshold; }
    float getFu() const { return fu; }
    float getFv() const { return fv; }
    float getU0() const { return u0; }
    float getV0() const { return v0; }
    double getRMinCam() const { return rMinCam; }
    double getRMaxCam() const { return rMaxCam; }
    const std::vector<Eigen::Vector3f>& getS0CamGT() const { return S0CamGT; }
    const std::vector<Eigen::Vector3f>& getS0LidGT() const { return S0LidGT; }
    float getRGT() const { return rGT; }
    const Eigen::Matrix<float, 3, 3>& getRotGT() const { return rotGT; }
    const Eigen::Vector3f& getTransGT() const { return transGT; }


private:
    std::vector<std::string> xyzPaths;
    std::vector<std::string> imgPaths;
    int ransacTypeLid;
    float ransacThresholdLid;
    float rMinLid;
    float rMaxLid;
    int iterationsRand4p;
    int iterationsAdjacency;
    int adjacencyThreshold;
    int iterationsDistance;
    float distanceThreshold;
    int iterationsCam;
    int ransacTypeCam;
    float ransacThresholdCam;
    int edgeDetect;
    float edgeThreshold;
    float fu;
    float fv;
    float u0;
    float v0;
    double rMinCam;
    double rMaxCam;
    std::vector<Eigen::Vector3f> S0CamGT;
    std::vector<Eigen::Vector3f> S0LidGT;
    float rGT;
    Eigen::Matrix<float, 3, 3> rotGT;
    Eigen::Vector3f transGT;
};

#endif // CONFIG_READER_H
