#pragma once
#include <vector>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "lib.h"

class PointCloudProcessor
{
	std::vector<std::vector<cv::Point3f>> pointClouds;
	std::vector<std::vector<cv::Vec3i>> colorClouds;

public:
	PointCloudProcessor(const std::vector<std::string> filePaths)
	{
		try {
			lib::readPlyFiles(filePaths, pointClouds, colorClouds);
		}
		catch (const std::runtime_error& e) {
			std::cerr << "Error: " << e.what() << std::endl;
		}
	}

	void displayPointClouds() {
		lib::displayPointClouds(pointClouds, colorClouds);
	}

	void writePlys() {
		lib::writePlys(pointClouds, colorClouds);
	}
};