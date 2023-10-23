#include "helpers/WritePly.h"

#include <iostream>
#include <iomanip>

void writePly(const std::string& fileName, const std::vector<cv::Point3f>& points, const std::vector<cv::Vec3i>& colors) {
    if (points.size() != colors.size()) {
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
    file << "element vertex " << points.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "end_header" << std::endl;

    for (size_t i = 0; i < points.size(); i++) {
        file << std::fixed << std::setprecision(7)
            << points[i].x << " "    << points[i].y  << " "   << points[i].z << " "
            << colors[i][2] << " " << colors[i][1] << " " << colors[i][0]
            << std::endl;
    }

    file.close();
}