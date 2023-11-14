#pragma once
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <Eigen/Dense>

namespace lib {
    void readPlyFiles(const std::vector<std::string>& filePaths, std::vector<std::vector<Eigen::Vector3f>>& pointClouds, std::vector<std::vector<cv::Vec3i>>& colorClouds);
    void readPlyFile(const std::string& plyFile, std::vector<Eigen::Vector3f>& pointCloud, std::vector<cv::Vec3i>& colorCloud);
    void readImages(const std::vector<std::string>& filePaths, std::vector<cv::Mat>& imgs);

    void writePlys(const std::vector<std::vector<Eigen::Vector3f>>& pointClouds, const std::vector<std::vector<cv::Vec3i>>& colorClouds);
    void writePly(const std::string& fileName, const std::vector<Eigen::Vector3f>& pointCloud, const std::vector<cv::Vec3i>& colorCloud);

    void displayImages(const std::vector<cv::Mat>& imgs);
    void displayPointClouds(const std::vector<std::vector<Eigen::Vector3f>>& pointClouds, const std::vector<std::vector<cv::Vec3i>>& colorClouds);
}  // namespace lib