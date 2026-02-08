#pragma once

#include <Eigen/Dense>

namespace DallE {
/**
 * @brief Inverse kinematics functions.
 */
class IK {
 public:
  /**
   * @brief Compute inverse kinematics in the body frame.
   * @param Blist Eigen::MatrixXd 6xn screw axes in the body frame.
   * @param M Eigen::MatrixXd home configuration of the end-effector.
   * @param T Eigen::MatrixXd desired end-effector configuration.
   * @param thetalist Eigen::VectorXd initial guess; updated on success.
   * @param eomg double orientation error tolerance.
   * @param ev double position error tolerance.
   * @return bool true if solution converged within tolerances.
   */
  static bool IKinBody(const Eigen::MatrixXd& Blist,
                       const Eigen::MatrixXd& M,
                       const Eigen::MatrixXd& T,
                       Eigen::VectorXd& thetalist,
                       double eomg,
                       double ev);

  /**
   * @brief Compute inverse kinematics in the space frame.
   * @param Slist Eigen::MatrixXd 6xn screw axes in the space frame.
   * @param M Eigen::MatrixXd home configuration of the end-effector.
   * @param T Eigen::MatrixXd desired end-effector configuration.
   * @param thetalist Eigen::VectorXd initial guess; updated on success.
   * @param eomg double orientation error tolerance.
   * @param ev double position error tolerance.
   * @return bool true if solution converged within tolerances.
   */
  static bool IKinSpace(const Eigen::MatrixXd& Slist,
                        const Eigen::MatrixXd& M,
                        const Eigen::MatrixXd& T,
                        Eigen::VectorXd& thetalist,
                        double eomg,
                        double ev);
};
}  // namespace DallE
