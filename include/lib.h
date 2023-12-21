#pragma once
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <Eigen/Dense>

namespace lib {
    void readXYZFile(const std::string& plyFile, std::vector<Eigen::Vector3f>& pointCloud);
    void readImage(const std::string& filePath, cv::Mat& img);
    void writePly(const std::string& fileName, const std::vector<Eigen::Vector3f>& pointCloud, const std::vector<cv::Vec3i>& colorCloud);

    void displayPointCloud(const std::vector<Eigen::Vector3f>& pointCloud);
    void visualizePoints(const cv::Mat& originalImage, const Eigen::MatrixXf& inliersNormalized, const Eigen::Vector2f& S0Normalized, const std::string& windowName, const std::string& outputPath);
    std::vector<int> generateRandomIndices(int numPts, int numRandomIndices);
}  // namespace lib