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
    const std::string& imgPath;

public:
    cv::Mat img;

    ImageProcessor(const std::string& imgPath,
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

    void detectSphereCenters(Eigen::MatrixXf& inliers, Eigen::Vector3f& S0);

private:
    cv::Mat rgb2gray(const cv::Mat& img);
    cv::Mat getEdgePoints(const cv::Mat& grayImg);
    Eigen::MatrixXi getIdxList(const cv::Mat& edgeImg);
    Eigen::MatrixXf pixel2meter(const Eigen::MatrixXf& points_pix);
    Eigen::MatrixXf meter2pixel(const Eigen::MatrixXf& points_m);
    Eigen::Vector2f meter2pixel(const Eigen::Vector3f& point_m);
    void detectSphereRand3p(const Eigen::MatrixXf& points_m, Eigen::MatrixXf& inliers, Eigen::Vector3f& S0);
    Eigen::VectorXf fitEllipse3p(const Eigen::MatrixXf& XY);
    Eigen::ArrayXf pointEllipseDistance(const Eigen::ArrayXf& px, const Eigen::ArrayXf& py, float a, float b, float ex, float ey, float theta);
    std::vector<int> classifyEllipsePoints(const Eigen::MatrixXf& points, const Eigen::VectorXf& ellipseParam);
    Eigen::Vector3f fitSphere3p(const Eigen::MatrixXf& inliersMat);
};
