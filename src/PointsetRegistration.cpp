#include "PointsetRegistration.h"

PointsetRegistration::PointsetRegistration() {}

PointsetRegistration::~PointsetRegistration() {}

void PointsetRegistration::pointRegistration(const Eigen::MatrixXf& pts1, const Eigen::MatrixXf& pts2, Eigen::MatrixXf& rot, Eigen::VectorXf& trans) {
    Eigen::VectorXf offset1 = pts1.colwise().mean();
    Eigen::VectorXf offset2 = pts2.colwise().mean();

    Eigen::MatrixXf centered_pts1 = pts1.rowwise() - offset1.transpose();
    Eigen::MatrixXf centered_pts2 = pts2.rowwise() - offset2.transpose();

    Eigen::MatrixXf H = centered_pts1.transpose() * centered_pts2;

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    rot = svd.matrixV() * svd.matrixU().transpose();

    trans = offset2 - rot * offset1;
}