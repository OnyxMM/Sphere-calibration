#include <iostream>
#include "ConfigReader.h"
#include "PointCloudProcessor.h"
#include "ImageProcessor.h"
#include "PointsetRegistration.h"
#include <Eigen/Dense>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


class SphereCalib {
public:
    void run(const std::string& configFile) {
        ConfigReader config(configFile);

        // Lidar Calibration
        std::vector<std::tuple<std::vector<Eigen::Vector3f>, Eigen::Vector3f, float>> lidarResults;

        for (const auto& xyzPath : config.getXYZPaths()) {
            // Initialize Point Cloud Processor for each Lidar scan
            PointCloudProcessor pointCloudProcessor({ xyzPath });

            std::vector<Eigen::Vector3f> inliersLid;
            Eigen::Vector3f S0Lid;
            float rLid;
            std::vector<int> inlierIndices;

            switch (config.getRansacTypeLid()) {
            case 1:
                // Adjacency method
                pointCloudProcessor.detectSphereWithAdjacency(config.getIterationsAdjacency(),
                    config.getAdjacencyThreshold(),
                    config.getRansacThresholdLid(),
                    config.getRMinLid(),
                    config.getRMaxLid(),
                    inliersLid,
                    S0Lid,
                    rLid,
                    inlierIndices);
                break;
            case 2:
                // Distance method
                pointCloudProcessor.detectSphereWithDistance(config.getIterationsDistance(),
                    config.getDistanceThreshold(),
                    config.getRansacThresholdLid(),
                    config.getRMinLid(),
                    config.getRMaxLid(),
                    inliersLid,
                    S0Lid,
                    rLid,
                    inlierIndices);
                break;
            case 3:
                // Rand4p method
                pointCloudProcessor.detectSphereRand4p(config.getIterationsRand4p(),
                    config.getRansacThresholdLid(),
                    config.getRMinLid(),
                    config.getRMaxLid(),
                    inliersLid,
                    S0Lid,
                    rLid,
                    inlierIndices);
                break;
            }

            pointCloudProcessor.colorInlierIndicesRed(inlierIndices, S0Lid);
            pointCloudProcessor.writeFile(xyzPath);

            // Store results for each Lidar scan
            lidarResults.push_back(std::make_tuple(inliersLid, S0Lid, rLid));
        }

        // Calculate rCam as the mean of rLid
        float rCam = 0.0f;
        for (const auto& lidarResult : lidarResults) {
            rCam += std::get<2>(lidarResult);
        }
        rCam /= static_cast<float>(lidarResults.size());

        // Camera Calibration
        std::vector<std::tuple<Eigen::MatrixXf, Eigen::Vector3f>> cameraResults;

        for (const auto& imgPath : config.getImgPaths()) {
            // Initialize Image Processor for each Camera image
            ImageProcessor imageProcessor({ imgPath },
                config.getEdgeDetect(),
                config.getEdgeThreshold(),
                config.getIterationsCam(),
                config.getRansacThresholdCam(),
                config.getRMinCam(),
                config.getRMaxCam(),
                config.getFu(),
                config.getFv(),
                config.getU0(),
                config.getV0(),
                rCam,
                config.getRansacTypeCam());

            Eigen::MatrixXf inliersCam;
            Eigen::Vector3f S0Cam;

            imageProcessor.detectSphereCenters(inliersCam, S0Cam);

            // Store results for each Camera image
            cameraResults.push_back(std::make_tuple(inliersCam, S0Cam));
        }

        checkOutput(config, lidarResults, cameraResults, rCam);
    }

    void estimateSphereCenterError(const Eigen::MatrixXf& S0Est, const Eigen::MatrixXf& S0GT, float& S0MeanErr, float& S0Std) {
        // Calculate the norm of the vector differences
        Eigen::VectorXf S0Norm = (S0GT - S0Est).rowwise().norm();

        // Calculate mean
        S0MeanErr = S0Norm.mean();

        // Calculate standard deviation
        float sumSquaredDiffs = 0.0f;
        for (int i = 0; i < S0Norm.size(); ++i) {
            float diff = S0Norm[i] - S0MeanErr;
            sumSquaredDiffs += diff * diff;
        }
        S0Std = std::sqrt(sumSquaredDiffs / S0Norm.size());
    }

    void estimateTransformError(const Eigen::Vector3f& transEst, const Eigen::Matrix3f& rotEst, const Eigen::Vector3f& transGT, const Eigen::Matrix3f& rotGT, float& transError, float& rotError) {
        // Estimate translation error - norm of the Euclidean distance of the vectors in meters
        transError = (transGT - transEst).norm();

        // Estimate rotation error - angular error in degrees
        float traceValue = (rotGT.transpose() * rotEst).trace();
        rotError = acos((traceValue - 1) / 2) * 180.0 / M_PI;
    }

    void checkOutput(const ConfigReader& config, const std::vector<std::tuple<std::vector<Eigen::Vector3f>, Eigen::Vector3f, float>>& lidarResults, const std::vector<std::tuple<Eigen::MatrixXf, Eigen::Vector3f>>& cameraResults, float rCam) {
        // Initialize PointsetRegistration
        PointsetRegistration registration;

        // Combine S0Cam and S0Lid into Eigen matrices row by row
        Eigen::MatrixXf S0LidMatrix(lidarResults.size(), 3);
        Eigen::MatrixXf S0CamMatrix(cameraResults.size(), 3);
        
        // Pointset registration between sphere centers
        for (size_t i = 0; i < std::min(lidarResults.size(), cameraResults.size()); ++i) {
            const auto& lidarResult = lidarResults[i];
            const auto& cameraResult = cameraResults[i];

            // Extract relevant information for pointset registration
            const auto& S0Lid = std::get<1>(lidarResult);
            const auto& S0Cam = std::get<1>(cameraResult);

            // Check the output values against ground truth
            const auto& S0CamGT = config.getS0CamGT()[i];
            const auto& S0LidGT = config.getS0LidGT()[i];


            float S0LidMeanErr, S0LidStd, S0CamMeanErr, S0CamStd, rErr, transErr, rotErr;

            // Check S0Cam against S0CamGT
            estimateSphereCenterError(S0Cam, S0CamGT, S0CamMeanErr, S0CamStd);
            std::cout << "S0Cam Error - Mean: " << S0CamMeanErr << ", Std: " << S0CamStd << "\n";

            // Check S0Lid against S0LidGT
            estimateSphereCenterError(S0Lid, S0LidGT, S0LidMeanErr, S0LidStd);
            std::cout << "S0Lid Error - Mean: " << S0LidMeanErr << ", Std: " << S0LidStd << "\n";

            // Check rCam against rGT
            rErr = std::abs(rCam - config.getRGT());
            std::cout << "r Error: " << rErr << "\n";

            S0LidMatrix.row(i) = S0Lid.transpose();
            S0CamMatrix.row(i) = S0Cam.transpose();
        }

        Eigen::MatrixXf rot;
        Eigen::VectorXf trans;
        float transErr, rotErr;

        // Perform pointset registration for each corresponding Lidar scan and Camera image
        registration.pointRegistration(S0CamMatrix, S0LidMatrix, rot, trans);

        const auto& rotGT = config.getRotGT();
        const auto& transGT = config.getTransGT();

        // Note: the stored GT transformation (rotate then translate) order is
        // opposite like in the pointset registration method (translate then rotate).
        // Hence, translation has to be re-estimated.
        Eigen::MatrixXf rotS0CamMatrix = rot * S0CamMatrix.transpose();
        Eigen::Vector3f transEst = S0LidMatrix.colwise().mean() - rotS0CamMatrix.topLeftCorner(3, 4).transpose().colwise().mean();

        estimateTransformError(trans, rot, transGT, rotGT, transErr, rotErr);

        std::cout << "Translation Error: " << transErr << "\n";
        std::cout << "Rotation Error:\n" << rotErr << "\n";
    }
};

int main() {
    SphereCalib sphereCalib;
    sphereCalib.run("input/example1/default_Config.json");

    return 0;
}
