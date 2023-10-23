#pragma once

#include <string>
#include <vector>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void writePly(const std::string& fileName, const std::vector<cv::Point3f>& points, const std::vector<cv::Vec3i>& colors);