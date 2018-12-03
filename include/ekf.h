#ifndef EKF
#define EKF

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class Ekf {
public:
 Ekf();
 Eigen::VectorXd x, z;
 Eigen::MatrixXd P, F, Q, H, R, G;

 Eigen::VectorXd predict(Eigen::VectorXd x, Eigen::VectorXd u);
 Eigen::VectorXd update(Eigen::VectorXd x, Eigen::VectorXd z);



};

#endif // EKF

