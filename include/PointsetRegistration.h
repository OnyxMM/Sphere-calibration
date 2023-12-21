#pragma once
#include <Eigen/Dense>

class PointsetRegistration {
public:
    PointsetRegistration();
    ~PointsetRegistration();

    void pointRegistration(const Eigen::MatrixXf& pts1, const Eigen::MatrixXf& pts2, Eigen::MatrixXf& rot, Eigen::VectorXf& trans);
};