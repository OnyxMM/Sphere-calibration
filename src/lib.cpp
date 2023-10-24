#include "lib.h"

#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace lib {

    void readPlyFiles(const std::vector<std::string>& filePaths, std::vector<std::vector<cv::Point3f>>& pointClouds, std::vector<std::vector<cv::Vec3i>>& colorClouds) {
        int numScan = filePaths.size();

        if (numScan < 1) {
            throw std::runtime_error("Not enough ply input file names.");
        }

        pointClouds.resize(numScan);
        colorClouds.resize(numScan);

        for (int i = 0; i < numScan; i++) {
            std::string scan3dFile = filePaths[i];
            readPlyFile(scan3dFile, pointClouds[i], colorClouds[i]);
        }
    }

    void readPlyFile(const std::string& plyFile, std::vector<cv::Point3f>& pointCloud, std::vector<cv::Vec3i>& colorCloud) {
        std::ifstream file(plyFile);

        if (!file.is_open()) {
            throw std::runtime_error("Failed to open PLY file.");
        }

        std::string line;
        int numVertices = 0;
        bool readData = false;

        while (std::getline(file, line)) {
            if (line.find("element vertex") != std::string::npos) {
                sscanf_s(line.c_str(), "element vertex %d", &numVertices);
            }
            if (line == "end_header") {
                readData = true;
                break;
            }
        }

        if (!readData) {
            throw std::runtime_error("Invalid PLY file format.");
        }

        pointCloud.resize(numVertices);
        colorCloud.resize(numVertices);

        for (int i = 0; i < numVertices; i++) {
            cv::Point3f& p = pointCloud[i];
            cv::Vec3i& color = colorCloud[i];
            file >> p.x >> p.y >> p.z >> color[2] >> color[1] >> color[0]; // Read color in reverse order (BGR).
        }

        file.close();
    }

    void readImages(const std::vector<std::string>& filePaths, std::vector<cv::Mat>& imgs) {
        int numImgs = filePaths.size();

        if (numImgs < 1) {
            throw std::runtime_error("Not enough image input file names.");
        }

        imgs.resize(numImgs);

        for (int i = 0; i < numImgs; i++) {
            std::string imgFile = filePaths[i];
            imgs[i] = cv::imread(imgFile, cv::IMREAD_COLOR);
        }
    }

    void writePlys(const std::vector<std::vector<cv::Point3f>>& pointClouds, const std::vector<std::vector<cv::Vec3i>>& colorClouds) {
        std::filesystem::create_directory("output"); // Create "output" folder if it doesn't exist

        for (int i = 0; i < pointClouds.size(); i++) {
            std::string outputFileName = "output/pointCloud" + std::to_string(i + 1);
            writePly(outputFileName, pointClouds[i], colorClouds[i]);
            std::cout << "Point cloud written to " << outputFileName << ".ply" << std::endl;
        }
    }

    void writePly(const std::string& fileName, const std::vector<cv::Point3f>& pointCloud, const std::vector<cv::Vec3i>& colorCloud) {
        if (pointCloud.size() != colorCloud.size()) {
            std::cerr << "Error: Number of points and colors do not match." << std::endl;
            return;
        }

        std::ofstream file(fileName + ".ply");
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
                << pointCloud[i].x << " " << pointCloud[i].y << " " << pointCloud[i].z << " "
                << colorCloud[i][2] << " " << colorCloud[i][1] << " " << colorCloud[i][0]
                << std::endl;
        }

        file.close();
    }

    void displayImages(const std::vector<cv::Mat>& imgs) {
        for (int i = 0; i < imgs.size(); i++) {
            cv::imshow("Image " + std::to_string(i + 1), imgs[i]);
            cv::waitKey(0);  // Wait for a key press (you may need to close the image window manually)
        }
    }

    void displayPointClouds(const std::vector<std::vector<cv::Point3f>>& pointClouds, const std::vector<std::vector<cv::Vec3i>>& colorClouds) {
        for (int i = 0; i < pointClouds.size(); i++) {
            // Display the first few points from each point cloud
            std::cout << "Point Cloud " << i + 1 << ":\n";
            for (int j = 0; j < std::min(5, static_cast<int>(pointClouds[i].size())); j++) {
                cv::Point3f p = pointClouds[i][j];
                cv::Vec3i color = colorClouds[i][j];

                std::cout << "Point " << j + 1 << ": (" << p.x << ", " << p.y << ", " << p.z
                    << "), Color: (" << color[2] << ", " << color[1] << ", " << color[0] << ")\n";
            }
        }
    }
} // namespace lib
