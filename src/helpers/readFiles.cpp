#include "helpers/readFiles.h"

void readPlyFile(const std::string& plyFile, std::vector<cv::Point3f>& points, std::vector<cv::Vec3i>& colors) {
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

    points.resize(numVertices);
    colors.resize(numVertices);

    for (int i = 0; i < numVertices; i++) {
        cv::Point3f& p = points[i];
        cv::Vec3i& color = colors[i];
        file >> p.x >> p.y >> p.z >> color[2] >> color[1] >> color[0]; // Read color in reverse order (BGR).
    }

    file.close();
}


void readFiles(const std::string& imgDir, const std::vector<std::string>& imgNames, const std::string& scan3dDir, const std::vector<std::string>& scan3dNames, std::vector<cv::Mat>& imgs, std::vector<std::vector<cv::Point3f>>& points, std::vector<std::vector<cv::Vec3i>>& colors) {
    int numImgs = imgNames.size();
    int numScan = scan3dNames.size();

    if (numImgs < 1) {
        throw std::runtime_error("Not enough input image file names.");
    }

    if (numScan < 1) {
        throw std::runtime_error("Not enough input 3D scan file names.");
    }

    if (numImgs != numScan) {
        throw std::runtime_error("Number of input images and 3D scans are different.");
    }

    imgs.resize(numImgs);
    points.resize(numScan);
    colors.resize(numScan);

    for (int i = 0; i < numScan; i++) {
        std::string scan3dFile = scan3dDir + scan3dNames[i];
        readPlyFile(scan3dFile, points[i], colors[i]);
    }

    for (int i = 0; i < numImgs; i++) {
        std::string imgFile = imgDir + imgNames[i];
        imgs[i] = cv::imread(imgFile, cv::IMREAD_COLOR);
    }
}