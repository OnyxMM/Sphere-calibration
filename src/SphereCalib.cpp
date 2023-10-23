#include "helpers/readFiles.h"
#include "helpers/writePly.h"
#include <iostream>

int main() {

    // Provide the directories and filenames as input vectors
    std::string imgDir = "input/";
    std::vector<std::string> imgNames = { "C1_S5.bmp" };
    std::string scan3dDir = "input/";
    std::vector<std::string> scan3dNames = { "sphereCalibScan5.ply" };

    std::vector<cv::Mat> imgs;
    std::vector<std::vector<cv::Point3f>> points;
    std::vector<std::vector<cv::Vec3i>> colors;

    try {
        readFiles(imgDir, imgNames, scan3dDir, scan3dNames, imgs, points, colors);
    }
    catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // Display the images
    for (int i = 0; i < imgs.size(); i++) {
        cv::imshow("Image " + std::to_string(i + 1), imgs[i]);
        cv::waitKey(0);  // Wait for a key press (you may need to close the image window manually)
    }

    for (int i = 0; i < points.size(); i++) {
        // Display the first few points from each point cloud
        std::cout << "Point Cloud " << i + 1 << ":\n";
        for (int j = 0; j < std::min(5, static_cast<int>(points[i].size())); j++) {
            cv::Point3f p = points[i][j];
            cv::Vec3i color = colors[i][j];

            std::cout << "Point " << j + 1 << ": (" << p.x << ", " << p.y << ", " << p.z
                << "), Color: (" << color[2] << ", " << color[1] << ", " << color[0] << ")\n";
        }

        // Write the point cloud to a PLY file
        std::string outputFileName = "pointCloud" + std::to_string(i + 1);
        writePly(outputFileName, points[i], colors[i]);
        std::cout << "Point cloud written to " << outputFileName << ".ply" << std::endl;
    }

    return 0;
}
