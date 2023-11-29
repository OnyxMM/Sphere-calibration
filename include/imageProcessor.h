#pragma once

#include <vector>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "lib.h"

class ImageProcessor
{
private:
    int edgeDetect;
    double edgeThreshold;
    int iterationsRCam;
    double ransacThresholdCam;
    double rMinCam;
    double rMaxCam;
    float fu;
    float fv;
    float u0;
    float v0;
    double rCam;
    int camRansac;

public:
    std::vector<cv::Mat> images;

    ImageProcessor(const std::vector<std::string>& imgPaths,
        int edgeDetect,
        double edgeThreshold,
        int iterationsRCam,
        double ransacThresholdCam,
        double rMinCam,
        double rMaxCam,
        float fu,
        float fv,
        float u0,
        float v0,
        double rCam,
        int camRansac);

    void detectSphereCenters();

private:
    cv::Mat rgb2gray(const cv::Mat& img);
    cv::Mat getEdgePoints(const cv::Mat& grayImg);
    Eigen::MatrixXi getIdxList(const cv::Mat& edgeImg);
    Eigen::MatrixXf pixel2meter(const Eigen::MatrixXf& points_pix);
    // TODO RANSAC
};
