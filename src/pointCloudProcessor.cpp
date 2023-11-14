#include "pointCloudProcessor.h"

#include <random>
#include <cmath>
#include <Eigen/Dense>

void PointCloudProcessor::detectSphereRand4p(int iterations, float ransacThreshold, float rMin, float rMax, std::vector<Eigen::Vector3f>& inliers, Eigen::Vector3f& S0, float& r, std::vector<int>& inlierIndices) {
    // Iterate through each point cloud
    for (const auto& pointCloud : pointClouds) {
        int numPts = static_cast<int>(pointCloud.size());
        std::vector<int> inlierIdxs;

        for (int i = 1; i <= iterations; ++i) {
            // generate 4 random indices
            std::vector<int> inlierIdxsTmp = generateRandomIndices(numPts, 4);

            // find sphere parameters
            Eigen::Vector3f S0_tmp;
            float r_tmp;
            fitSphereNonit(pointCloud, inlierIdxsTmp, S0_tmp, r_tmp);

            // optional speed-up
            if (r_tmp > rMin && r_tmp < rMax) {
                // label points
                std::vector<int> inlierIdxsTmpUpdated;
                classifySpherePoints(pointCloud, S0_tmp, r_tmp, ransacThreshold, inlierIdxsTmpUpdated);

                // save the best model
                if (inlierIdxsTmpUpdated.size() > inlierIdxs.size()) {
                    inlierIdxs = std::move(inlierIdxsTmpUpdated);
                }
            }
        }

        // re-fit
        inliers.insert(inliers.end(), pointCloud.begin(), pointCloud.end());

        // Save the inlier indices
        inlierIndices.insert(inlierIndices.end(), inlierIdxs.begin(), inlierIdxs.end());
    }

    // Fit the sphere to all inliers
    fitSphereLsq(inliers, S0, r);
}

void PointCloudProcessor::detectSphereWithAdjacency(int iterations, int adjacencyThreshold, float ransacThreshold, float rMin, float rMax, std::vector<Eigen::Vector3f>& inliers, Eigen::Vector3f& S0, float& r, std::vector<int>& inlierIndices) {
    for (std::vector<Eigen::Vector3f> pointCloud : pointClouds) {
        std::vector<Eigen::Vector3f> bestInlierPoints;

        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(0, pointCloud.size() - 1);

        for (int i = 0; i < iterations; i++) {
            // Generate 1 random point
            int randIdx = distribution(generator);

            // Select initial subset
            std::vector<int> initialInlierIndices;
            findKNearestNeighbors(pointCloud, pointCloud[randIdx], initialInlierIndices, adjacencyThreshold);

            // Find sphere parameters
            Eigen::Vector3f S0Tmp;
            float rTmp;
            fitSphereNonit(pointCloud, initialInlierIndices, S0Tmp, rTmp);

            // Optional speed-up: check if the radius is within the valid range
            if (rTmp > rMin && rTmp < rMax) {
                // Label points
                std::vector<int> currentInlierIndices;
                classifySpherePoints(pointCloud, S0Tmp, rTmp, ransacThreshold, currentInlierIndices);

                // Save the best model
                if (currentInlierIndices.size() > bestInlierPoints.size()) {
                    pointsFromIndices(pointCloud, currentInlierIndices, bestInlierPoints);

                    // Append the current inlier indices to the inlierIndices
                    inlierIndices.clear();
                    inlierIndices.insert(inlierIndices.end(), currentInlierIndices.begin(), currentInlierIndices.end());
                }
            }
        }

        // Re-fit the best inliers
        inliers = bestInlierPoints;
        fitSphereLsq(inliers, S0, r);
    }
}

// We could use a already existing optimal solution for finding the K nearest neighbours. (for example nanoflann)
void PointCloudProcessor::findKNearestNeighbors(const std::vector<Eigen::Vector3f>& pointCloud, const Eigen::Vector3f& queryPoint, std::vector<int>& indices, int k) {
    indices.clear();

    // Calculate distances and store indices
    std::vector<std::pair<float, int>> distances;

    for (int i = 0; i < pointCloud.size(); ++i) {
        float distance = (pointCloud[i] - queryPoint).norm();
        distances.emplace_back(distance, i);
    }

    // Sort distances and select the first k indices
    std::sort(distances.begin(), distances.end());

    for (int i = 0; i < k; ++i) {
        indices.push_back(distances[i].second);
    }
}

void PointCloudProcessor::fitSphereNonit(const std::vector<Eigen::Vector3f>& pointCloud, const std::vector<int>& indices, Eigen::Vector3f& S0, float& r) {
    int N = indices.size();

    // Compute the sums and products needed for the algorithm
    float Sx = 0.0f, Sy = 0.0f, Sz = 0.0f;
    float Sxx = 0.0f, Syy = 0.0f, Szz = 0.0f;
    float Sxy = 0.0f, Sxz = 0.0f, Syz = 0.0f;
    float Sxxx = 0.0f, Syyy = 0.0f, Szzz = 0.0f;
    float Sxyy = 0.0f, Sxzz = 0.0f, Sxxy = 0.0f;
    float Sxxz = 0.0f, Syyz = 0.0f, Syzz = 0.0f;

    for (int i : indices) {
        float x = pointCloud[i].x();
        float y = pointCloud[i].y();
        float z = pointCloud[i].z();

        Sx += x;
        Sy += y;
        Sz += z;
        Sxx += x * x;
        Syy += y * y;
        Szz += z * z;
        Sxy += x * y;
        Sxz += x * z;
        Syz += y * z;
        Sxxx += x * x * x;
        Syyy += y * y * y;
        Szzz += z * z * z;
        Sxyy += x * y * y;
        Sxzz += x * z * z;
        Sxxy += x * x * y;
        Sxxz += x * x * z;
        Syyz += y * y * z;
        Syzz += y * z * z;
    }

    float A1 = Sxx + Syy + Szz;
    float a = 2 * (Sx * Sx) - 2 * N * Sxx;
    float b = 2 * (Sx * Sy) - 2 * N * Sxy;
    float c = 2 * (Sx * Sz) - 2 * N * Sxz;
    float d = -N * (Sxxx + Sxyy + Sxzz) + A1 * Sx;
    float e = 2 * (Sx * Sy) - 2 * N * Sxy;
    float f = 2 * (Sy * Sy) - 2 * N * Syy;
    float g = 2 * (Sy * Sz) - 2 * N * Syz;
    float h = -N * (Sxxy + Syyy + Syzz) + A1 * Sy;
    float j = 2 * (Sx * Sz) - 2 * N * Sxz;
    float k = 2 * (Sy * Sz) - 2 * N * Syz;
    float l = 2 * (Sz * Sz) - 2 * N * Szz;
    float m = -N * (Sxxz + Syyz + Szzz) + A1 * Sz;

    float delta = a * (f * l - g * k) - e * (b * l - c * k) + j * (b * g - c * f);
    float x0 = (d * (f * l - g * k) - h * (b * l - c * k) + m * (b * g - c * f)) / delta;
    float y0 = (a * (h * l - m * g) - e * (d * l - m * c) + j * (d * g - h * c)) / delta;
    float z0 = (a * (f * m - h * k) - e * (b * m - d * k) + j * (b * h - d * f)) / delta;
    S0 = Eigen::Vector3f(x0, y0, z0);
    r = std::sqrt(x0 * x0 + y0 * y0 + z0 * z0 + (A1 - 2 * (x0 * Sx + y0 * Sy + z0 * Sz)) / N);
}

void PointCloudProcessor::classifySpherePoints(const std::vector<Eigen::Vector3f>& pointCloud, const Eigen::Vector3f& S0, float r, float ransacThreshold, std::vector<int>& inlierIndices) {
    inlierIndices.clear();

    // Iterate through the point cloud and classify points as inliers or outliers
    for (int i = 0; i < pointCloud.size(); i++) {
        // Calculate the Euclidean distance between the point and the sphere center
        Eigen::Vector3f radii = S0 - pointCloud[i];
        float radius = radii.norm();

        // Calculate the radius error
        float distance = std::abs(r - radius);

        // Check if the point is an inlier based on the RANSAC threshold
        if (distance < ransacThreshold) {
            // Point is an inlier
            inlierIndices.push_back(i);
        }
    }
}

void PointCloudProcessor::pointsFromIndices(const std::vector<Eigen::Vector3f>& pointCloud, const std::vector<int>& indices, std::vector<Eigen::Vector3f>& selectedPoints) {
    selectedPoints.clear();

    for (int i : indices) {
        selectedPoints.push_back(pointCloud[i]);
    }
}

void PointCloudProcessor::fitSphereLsq(const std::vector<Eigen::Vector3f>& pointCloud, Eigen::Vector3f& S0, float& r) {
    int iterations = 1000;
    int numPts = static_cast<int>(pointCloud.size());
    Eigen::Vector3f meanPts(0.0f, 0.0f, 0.0f);
    std::vector<Eigen::Vector3f> normalizedPoints = pointCloud;

    // Calculate the mean of the points
    for (const Eigen::Vector3f& point : normalizedPoints) {
        meanPts += point;
    }
    meanPts /= static_cast<float>(numPts);

    for (int i = 0; i < numPts; i++) {
        normalizedPoints[i] -= meanPts;
    }

    int numS0Ini = numPts / 4;
    std::vector<Eigen::Vector3f> S0Ini(numS0Ini);
    std::vector<int> idxs(numPts);
    for (int i = 0; i < numPts; i++) {
        idxs[i] = i;
    }
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(idxs.begin(), idxs.end(), g);

    for (int i = 0; i < numS0Ini; i++) {
        int j = i * 4;
        std::vector<Eigen::Vector3f> rnd4;
        for (int k = j; k < j + 4; k++) {
            rnd4.push_back(normalizedPoints[idxs[k]]);
        }
        fitSphere4p(rnd4); //S0Ini[i], r
    }

    std::sort(S0Ini.begin(), S0Ini.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
        return a.norm() < b.norm();
        });

    S0 = S0Ini[numS0Ini / 2]; // Use the median as the initial sphere center

    for (int j = 0; j < iterations; j++) {
        float r_avg = 0.0f;
        Eigen::Vector3f dir_avg(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f S0_tmp = S0;

        for (int i = 0; i < numPts; i++) {
            Eigen::Vector3f dir_i = S0 - normalizedPoints[i];
            float r_i = dir_i.norm();
            r_avg += r_i;
            dir_avg += dir_i / r_i;
        }

        S0 = (r_avg * dir_avg) / (numPts * numPts);
        r = r_avg / numPts;
        Eigen::Vector3f diff_new = S0 - S0_tmp;

        if (diff_new.isZero()) {
            break;
        }
    }

    S0 += meanPts; // Add back the mean of the points
}

Eigen::Vector3f PointCloudProcessor::fitSphere4p(const std::vector<Eigen::Vector3f>& points) {
    if (points.size() != 4) {
        // Handle the case where there are not exactly 4 points
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }

    Eigen::Vector3f midP[4];
    Eigen::Vector3f n[4];
    float d[4] = { 0.0f, 0.0f, 0.0f };

    for (int i = 0; i < 4; i++) {
        midP[i] = (points[i] + points[(i + 1) % 4]) / 2;
        n[i] = points[i] - midP[i];
        n[i].normalize();
        d[i] = n[i].dot(midP[i]);
    }

    Eigen::Matrix3f n_matx;
    n_matx << n[0], n[1], n[2];

    Eigen::Vector3f d_vec(d[0], d[1], d[2]);

    // Check for NaN and Inf values
    if (n_matx.array().isNaN().any() || d_vec.array().isNaN().any() ||
        n_matx.array().isInf().any() || d_vec.array().isInf().any()) {
        // Handle the case of NaN or Inf in the calculations
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }

    Eigen::Vector3f S0 = n_matx.colPivHouseholderQr().solve(d_vec);

    return S0;
}

void PointCloudProcessor::colorInlierIndicesRed(std::vector<int>& inlierIndices) {
    for (std::vector<cv::Vec3i>& colorCloud : colorClouds) {
        for (int index : inlierIndices) {
            if (index >= 0 && index < colorCloud.size()) {
                colorCloud[index] = cv::Vec3i(0, 0, 255);  // Set the color to red (BGR format)
            }
        }
    }
}

std::vector<int> PointCloudProcessor::generateRandomIndices(int numPts, int numRandomIndices) {
    std::vector<int> indices(numPts);
    for (int i = 0; i < numPts; ++i) {
        indices[i] = i;
    }

    // Use a random device and a random engine to shuffle the indices
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);

    // Take the first numRandomIndices elements as the result
    indices.resize(numRandomIndices);

    return indices;
}