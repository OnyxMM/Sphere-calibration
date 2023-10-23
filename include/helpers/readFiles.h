#pragma once

#include <string>
#include <vector>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void readPlyFile(const std::string& plyFile, std::vector<cv::Point3f>& points, std::vector<cv::Vec3i>& colors);
void readFiles(const std::string& imgDir, const std::vector<std::string>& imgNames, const std::string& scan3dDir, const std::vector<std::string>& scan3dNames, std::vector<cv::Mat>& imgs, std::vector<std::vector<cv::Point3f>>& points, std::vector<std::vector<cv::Vec3i>>& colors);