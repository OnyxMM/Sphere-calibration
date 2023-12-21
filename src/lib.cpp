#include "lib.h"

#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <algorithm>
#include <random>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

namespace lib {

    void readXYZFile(const std::string& xyzFile, std::vector<Eigen::Vector3f>& pointCloud) {
        std::ifstream file(xyzFile);

        if (!file.is_open()) {
            throw std::runtime_error("Failed to open XYZ file.");
        }

        std::string line;
        int numVertices = 0;
        bool readData = false;

        while (std::getline(file, line)) {
            numVertices++;
        }

        // Reset file position to the beginning
        file.clear();
        file.seekg(0, std::ios::beg);

        pointCloud.resize(numVertices);

        for (int i = 0; i < numVertices; i++) {
            Eigen::Vector3f& p = pointCloud[i];
            file >> p.x() >> p.y() >> p.z();
        }

        file.close();
    }

    void readImage(const std::string& filePath, cv::Mat& img) {
        img = cv::imread(filePath, cv::IMREAD_COLOR);
    }

    void writePly(const std::string& path, const std::vector<Eigen::Vector3f>& pointCloud, const std::vector<cv::Vec3i>& colorCloud) {
        if (pointCloud.size() != colorCloud.size()) {
            std::cerr << "Error: Number of points and colors do not match." << std::endl;
            return;
        }

        // Replace "input" with "output" in the path
        std::string updatedPath = path;
        size_t pos = updatedPath.find("input");
        if (pos != std::string::npos) {
            updatedPath.replace(pos, 5, "output");
        }

        // Create directories if they don't exist
        std::filesystem::create_directories(std::filesystem::path(updatedPath).parent_path());

        std::ofstream file(updatedPath + ".ply");
        if (!file.is_open()) {
            std::cerr << "Error: Failed to open PLY file." << std::endl;
            return;
        }

        file << "ply" << std::endl;
        file << "format ascii 1.0" << std::endl;
        file << "element vertex " << pointCloud.size() << std::endl;
        file << "property float x" << std::endl;
        file << "property float y" << std::endl;
        file << "property float z" << std::endl;
        file << "property uchar red" << std::endl;
        file << "property uchar green" << std::endl;
        file << "property uchar blue" << std::endl;
        file << "end_header" << std::endl;

        for (size_t i = 0; i < pointCloud.size(); i++) {
            file << std::fixed << std::setprecision(7)
                << pointCloud[i](0) << " " << pointCloud[i](1) << " " << pointCloud[i](2) << " "
                << colorCloud[i][2] << " " << colorCloud[i][1] << " " << colorCloud[i][0]
                << std::endl;
        }

        file.close();
    }

    void displayPointCloud(const std::vector<Eigen::Vector3f>& pointCloud) {
        // Display the first few points
        for (int i = 0; i < std::min(5, static_cast<int>(pointCloud.size())); i++) {
            Eigen::Vector3f p = pointCloud[i];

            std::cout << "Point " << i + 1 << ": (" << p(0) << ", " << p(1) << ", " << p(2)
                << ")\n";
        }
    }

    void lib::visualizePoints(const cv::Mat& originalImage, const Eigen::MatrixXf& inliersNormalized, const Eigen::Vector2f& S0Normalized, const std::string& windowName, const std::string& outputPath) {
        cv::Mat imageWithCircles = originalImage.clone();

        // Ensure that points are in integer type (assuming pixel coordinates)
        Eigen::MatrixXi scaledPoints = inliersNormalized.cast<int>();

        // Draw circles at the point locations
        for (int i = 0; i < scaledPoints.rows(); ++i) {
            cv::Point center(scaledPoints(i, 0), scaledPoints(i, 1));
            cv::circle(imageWithCircles, center, 5, cv::Scalar(0, 255, 0), -1); // Green circle
        }

        // Draw a point for S0Normalized if its not set to default
        if (S0Normalized(0) != 0 && S0Normalized(1) != 0) {
            cv::Point s0Point(S0Normalized(0), S0Normalized(1));
            cv::circle(imageWithCircles, s0Point, 5, cv::Scalar(255, 0, 0), -1); // Blue circle for S0Normalized
        }

        // Resize the image
        cv::resize(imageWithCircles, imageWithCircles, cv::Size(), 0.5, 0.5);

        // Display the resized image with circles
        cv::imshow(windowName, imageWithCircles);

        // Replace "input" with "output" in the path
        std::string updatedPath = outputPath;
        size_t pos = updatedPath.find("input");
        if (pos != std::string::npos) {
            updatedPath.replace(pos, 5, "output");
        }

        // Save the image to the specified output path
        cv::imwrite(updatedPath, imageWithCircles);

        // Wait for a key press and then close the window
        cv::waitKey(0);
        cv::destroyWindow(outputPath);
    }

    std::vector<int> generateRandomIndices(int numPts, int numRandomIndices) {
        std::vector<int> indices(numPts);
        for (int i = 0; i < numPts; ++i) {
            indices[i] = i;
        }

        // Use a random device and a random engine to shuffle the indices
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(indices.begin(), indices.end(), g);

        // Take the first numRandomIndices elements as the result
        indices.resize(numRandomIndices);

        return indices;
    }
} // namespace lib
