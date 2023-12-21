#include "ImageProcessor.h"

#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include <tuple>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

ImageProcessor::ImageProcessor(const std::string& imgPath,
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
    camRansac(camRansac),
    imgPath(imgPath)
{
    lib::readImage(imgPath, img);
}

void ImageProcessor::detectSphereCenters(Eigen::MatrixXf& inliers, Eigen::Vector3f& S0)
{
        // Perform processing on each image
        cv::Mat grayImg = rgb2gray(img);

        // Get edge points from the current image
        cv::Mat edgeImg = getEdgePoints(grayImg);

        // Convert edge points to Eigen MatrixXf
        Eigen::MatrixXi edgeIndices = getIdxList(edgeImg);

        // Swap x and y to have x as horizontal and y as vertical axis
        Eigen::MatrixXf points_pix(edgeIndices.rows(), 2);
        points_pix << edgeIndices.col(1).cast<float>(), edgeIndices.col(0).cast<float>();

    // Visualization of the selected edges
    lib::visualizePoints(img, points_pix, Eigen::Vector2f(0,0), "Selected edges: " + imgPath, imgPath);

        // Normalize coordinates
        Eigen::MatrixXf points_m = pixel2meter(points_pix);

        switch (camRansac)
        {
        case 1:
        // TODO
            break;
        case 2:
            detectSphereRand3p(points_m, inliers, S0);
            break;
        default:
            break;
        }

    Eigen::MatrixXf inliersNormalized = meter2pixel(inliers);
    Eigen::MatrixXf S0Normalized = meter2pixel(S0);

    // Visualization of inliers and sphere center
    lib::visualizePoints(img, inliersNormalized, S0Normalized, "Inliers and sphere center: " + imgPath, imgPath);
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
    cv::Mat magnitude;
    switch (edgeDetect)
    {
    case 1:
        cv::Canny(grayImg, edgeImg, edgeThreshold, 2 * edgeThreshold);
        break;
    case 2:
        cv::Sobel(grayImg, gradientX, CV_64F, 1, 0);
        cv::Sobel(grayImg, gradientY, CV_64F, 0, 1);

        cv::magnitude(gradientX, gradientY, magnitude);
        cv::convertScaleAbs(magnitude, edgeImg);
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

Eigen::MatrixXf ImageProcessor::meter2pixel(const Eigen::MatrixXf& points_m)
{
    // Convert meters to image pixels using camera intrinsic parameters
    Eigen::MatrixXf points_pix = (points_m.array() * Eigen::RowVector2f(fu, fv).replicate(points_m.rows(), 1).array())
        .array() + Eigen::RowVector2f(u0, v0).replicate(points_m.rows(), 1).array();

    return points_pix;
}

Eigen::Vector2f ImageProcessor::meter2pixel(const Eigen::Vector3f& point_m)
{
    // Extract x, y, z coordinates from the Vector3f
    float x_m = point_m(0);
    float y_m = point_m(1);
    float z_m = point_m(2);

    // Convert meters to image pixels using camera intrinsic parameters
    float x_pix = fu * x_m / z_m + u0;
    float y_pix = fv * y_m / z_m + v0;

    return Eigen::Vector2f(x_pix, y_pix);
}


void ImageProcessor::detectSphereRand3p(const Eigen::MatrixXf& points_m, Eigen::MatrixXf& inliers, Eigen::Vector3f& S0)
{
    // Assuming the necessary functions for ellipse fitting, classification, and sphere fitting are available
    int numPts = points_m.rows();
    std::vector<int> inlierIdxs;

    for (int j = 0; j < iterationsRCam; ++j) {
        // Generate 3 random indices
        std::vector<int> inlierIdxsTmp = lib::generateRandomIndices(numPts-2, 3);

        // Approximate sphere projection parameters with 3 points
        Eigen::MatrixXf selectedPoints = points_m.block(inlierIdxsTmp[0], 0, 3, 2);
        std::cout << "\rIndex" << j << "/" << iterationsRCam << std::flush;

        Eigen::VectorXf ellipseParam = fitEllipse3p(selectedPoints);

        // Optional speed-up: Check if the longer ellipse semi-axis is within the specified range
        if (ellipseParam(0) > rMinCam && ellipseParam(0) < rMaxCam) {
            // Label points
            std::vector<int> inlierIdxsTmpUpdated = classifyEllipsePoints(points_m, ellipseParam);

            // Save the best model
            if (inlierIdxsTmpUpdated.size() > inlierIdxs.size()) {
                inlierIdxs = std::move(inlierIdxsTmpUpdated);
            }
        }
    }

    std::cout << std::endl;

    // Re-fit a sphere
    inliers.resize(inlierIdxs.size(), 2);
    for (size_t i = 0; i < inlierIdxs.size(); ++i) {
        inliers.row(i) = points_m.row(inlierIdxs[i]);
    }

    S0 = fitSphere3p(inliers);
}

Eigen::VectorXf ImageProcessor::fitEllipse3p(const Eigen::MatrixXf& XY)
{
    // Estimate ellipse parameters with at least 3 points

    // Add third coordinate z=1 and normalize points
    Eigen::MatrixXf XY_h(XY.rows(), 3);
    XY_h << XY, Eigen::MatrixXf::Ones(XY.rows(), 1);
    XY_h = (XY_h.array().colwise() / XY_h.rowwise().norm().array()).matrix();

    // Find the axis direction and angle of the cone
    Eigen::VectorXf wPerCosAlpha = XY_h.colPivHouseholderQr().solve(Eigen::VectorXf::Ones(XY.rows()));
    float cosAlpha = 1.0 / wPerCosAlpha.norm();
    Eigen::VectorXf w = cosAlpha * wPerCosAlpha;

    // Rotation around the orthogonal vector o with alpha
    float sinAlpha = sqrt(1.0 - cosAlpha * cosAlpha);
    float versinAlpha = 1.0 - cosAlpha;
    Eigen::Matrix3f R;

    if (w.isApprox(Eigen::Vector3f(0, 0, 1))) {
        R << cosAlpha, 0, sinAlpha,
            0, 1, 0,
            -sinAlpha, 0, cosAlpha;
    }
    else {
        Eigen::Vector2f o(-w(1), w(0));
        o /= o.norm();
        R << o(0) * o(0) * versinAlpha + cosAlpha, o(0)* o(1)* versinAlpha, o(1)* sinAlpha,
            o(0)* o(1)* versinAlpha, o(1)* o(1)* versinAlpha + cosAlpha, -o(0) * sinAlpha,
            -o(1) * sinAlpha, o(0)* sinAlpha, cosAlpha;
    }

    // Vector from the origin to the direction of the major ellipse axis ending points
    Eigen::Vector3f q_a1 = R * w;
    Eigen::Vector3f q_a2 = R.transpose() * w;

    // Ending points of the major ellipse axes in the image
    Eigen::Vector2f a1(q_a1(0) / q_a1(2), q_a1(1) / q_a1(2));
    Eigen::Vector2f a2(q_a2(0) / q_a2(2), q_a2(1) / q_a2(2));

    // Ellipse center
    Eigen::Vector2f E0 = 0.5 * (a1 + a2);
    float ex = E0(0), ey = E0(1);

    // Length of the semi-major and semi-minor axis
    float a = (a2 - a1).norm() / 2.0;
    float omega = ex * ex + ey * ey + 1 - a * a;
    float b = sqrt((-omega + sqrt(omega * omega + 4 * a * a)) / 2);

    // Rotation angle of the ellipse
    float theta;
    if (ey == 0) {
        theta = 0;
    }
    else if (ex == 0) {
        theta = 0.5 * M_PI;
    }
    else {
        theta = atan2(ey, ex);
    }

    Eigen::VectorXf ellipseParam(5);
    ellipseParam << a, b, ex, ey, theta;

    return ellipseParam;
}

Eigen::ArrayXf ImageProcessor::pointEllipseDistance(const Eigen::ArrayXf& px, const Eigen::ArrayXf& py, float a, float b, float ex, float ey, float theta)
{
    Eigen::ArrayXf ab(2);
    ab << a, b;

    Eigen::ArrayXf px0 = cos(theta) * (px - ex) + sin(theta) * (py - ey);
    Eigen::ArrayXf py0 = -sin(theta) * (px - ex) + cos(theta) * (py - ey);

    Eigen::ArrayXXf p(px0.size(), 2);
    p.col(0) = px0.abs();
    p.col(1) = py0.abs();

    // Perform element-wise operations without reshaping
    Eigen::ArrayXf k0 = (p.col(0) / ab(0)).square() + (p.col(1) / ab(1)).square();
    k0 = k0.sqrt();

    Eigen::ArrayXf k1 = (p.col(0) / (ab(0) * ab(0))).square() + (p.col(1) / (ab(1) * ab(1))).square();
    k1 = k1.sqrt();

    return (k0 * (k0 - 1) / k1).abs();
}



std::vector<int> ImageProcessor::classifyEllipsePoints(const Eigen::MatrixXf& points, const Eigen::VectorXf& ellipseParam)
{
    int numPts = points.rows();
    std::vector<int> inlierIdxs;

    // Distance of the points from the ellipse with numerical approximation
    Eigen::ArrayXf distances = pointEllipseDistance(points.col(0), points.col(1), ellipseParam(0),
        ellipseParam(1), ellipseParam(2), ellipseParam(3), ellipseParam(4));

    // If the point fits, save the index
    for (int i = 0; i < numPts; ++i) {
        if (distances(i) < ransacThresholdCam) {
            inlierIdxs.push_back(i);
        }
    }

    return inlierIdxs;
}

Eigen::Vector3f ImageProcessor::fitSphere3p(const Eigen::MatrixXf& inliersMat)
{
    int numPts = inliersMat.rows();

    // Add third coordinate z=1 and normalize points
    Eigen::MatrixXf XY_h(numPts, 3);
    XY_h << inliersMat, Eigen::MatrixXf::Ones(numPts, 1);
    Eigen::MatrixXf XY_h_norm = XY_h.rowwise().normalized();

    // Element-wise multiplication for the first three columns
    Eigen::MatrixXf result(numPts, 3);
    result.col(0) = XY_h_norm.col(0).cwiseProduct(XY_h.col(0));
    result.col(1) = XY_h_norm.col(1).cwiseProduct(XY_h.col(1));
    result.col(2) = XY_h_norm.col(2).cwiseProduct(XY_h.col(2));

    // Find the axis direction and angle of the cone
    Eigen::VectorXf wPerCosAlpha = result.col(0).householderQr().solve(Eigen::VectorXf::Ones(numPts));
    float cosAlpha = 1 / wPerCosAlpha.norm();
    float wScalar = cosAlpha * wPerCosAlpha(0);

    // Direction vector of the cone axis
    float d = rCam / sqrt(1 - cosAlpha * cosAlpha);

    Eigen::Vector3f S0 = d * Eigen::Vector3f(wScalar, wScalar, wScalar);
    return S0;
}