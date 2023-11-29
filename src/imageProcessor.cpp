#include "ImageProcessor.h"

#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <random>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

ImageProcessor::ImageProcessor(const std::vector<std::string>& imgPaths,
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
    int camRansac)
    : edgeDetect(edgeDetect),
    edgeThreshold(edgeThreshold),
    iterationsRCam(iterationsRCam),
    ransacThresholdCam(ransacThresholdCam),
    rMinCam(rMinCam),
    rMaxCam(rMaxCam),
    fu(fu),
    fv(fv),
    u0(u0),
    v0(v0),
    rCam(rCam),
    camRansac(camRansac)
{
    lib::readImages(imgPaths, images);
}

void ImageProcessor::detectSphereCenters()
{
    for (const auto& img : images) {
        // Perform processing on each image
        cv::Mat grayImg = rgb2gray(img);

        // Get edge points from the current image
        cv::Mat edgeImg = getEdgePoints(grayImg);

        // Convert edge points to Eigen MatrixXf
        Eigen::MatrixXi edgeIndices = getIdxList(edgeImg);

        // Swap x and y to have x as horizontal and y as vertical axis
        Eigen::MatrixXf points_pix(edgeIndices.rows(), 2);
        points_pix << edgeIndices.col(1).cast<float>(), edgeIndices.col(0).cast<float>();

        // Normalize coordinates
        Eigen::MatrixXf points_m = pixel2meter(points_pix);

        // TODO RANSAC
    }
}

cv::Mat ImageProcessor::rgb2gray(const cv::Mat& img)
{
    cv::Mat grayImg;

    if (img.channels() == 1) {
        // If the image is already grayscale, return a copy
        grayImg = img.clone();
    }
    else if (img.channels() == 3) {
        // If the image is in color, convert it to grayscale
        cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);
    }
    else {
        // Handle other cases as needed
        std::cerr << "Unsupported image format." << std::endl;
    }

    return grayImg;
}

cv::Mat ImageProcessor::getEdgePoints(const cv::Mat& grayImg)
{
    cv::Mat edgeImg;
    cv::Mat gradientX, gradientY;
    switch (edgeDetect)
    {
    case 1:
        cv::Canny(grayImg, edgeImg, edgeThreshold, 2 * edgeThreshold);
        break;
    case 2:
        cv::Sobel(grayImg, gradientX, CV_64F, 1, 0);
        cv::Sobel(grayImg, gradientY, CV_64F, 0, 1);
        cv::magnitude(gradientX, gradientY, edgeImg);
        break;
    default:
        std::cerr << "Unsupported edge detection method." << std::endl;
        break;
    }

    return edgeImg;
}

Eigen::MatrixXi ImageProcessor::getIdxList(const cv::Mat& edgeImg)
{
    int N = edgeImg.rows;
    int M = edgeImg.cols;

    Eigen::MatrixXi edgeIdxs(edgeImg.rows * edgeImg.cols, 2);
    int numIdxs = 0;

    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            if (edgeImg.at<uchar>(i, j) > 0)
            {
                ++numIdxs;
                edgeIdxs(numIdxs - 1, 0) = i;
                edgeIdxs(numIdxs - 1, 1) = j;
            }
        }
    }

    return edgeIdxs.topRows(numIdxs);
}

Eigen::MatrixXf ImageProcessor::pixel2meter(const Eigen::MatrixXf& points_pix)
{
    // Convert image pixel to meters using camera intrinsic parameters
    Eigen::MatrixXf points_m = (points_pix.array() - Eigen::RowVector2f(u0, v0).replicate(points_pix.rows(), 1).array())
        .array() / Eigen::RowVector2f(fu, fv).replicate(points_pix.rows(), 1).array();

    return points_m;
}