#include "my_modern_robotics/robot_control.h"

#include "my_modern_robotics/dynamics.h"
#include "my_modern_robotics/inverse_dynamics.h"

namespace mymr {
Eigen::VectorXd RobotControl::ComputedTorque(
    const Eigen::VectorXd& thetalist,
    const Eigen::VectorXd& dthetalist,
    const Eigen::VectorXd& eint,
    const Eigen::VectorXd& thetalistd,
    const Eigen::VectorXd& dthetalistd,
    const Eigen::VectorXd& ddthetalistd,
    const Eigen::Vector3d& g,
    const std::vector<Eigen::MatrixXd>& Mlist,
    const std::vector<Eigen::MatrixXd>& Glist,
    const Eigen::MatrixXd& Slist,
    const Eigen::VectorXd& Kp,
    const Eigen::VectorXd& Ki,
    const Eigen::VectorXd& Kd) {
  Eigen::VectorXd e = thetalistd - thetalist;
  Eigen::VectorXd edot = dthetalistd - dthetalist;
  Eigen::VectorXd u = ddthetalistd + Kp.asDiagonal() * e +
                      Ki.asDiagonal() * eint + Kd.asDiagonal() * edot;
  Eigen::VectorXd Ftip = Eigen::VectorXd::Zero(6);
  return InverseDynamics::Compute(thetalist, dthetalist, u, g, Ftip, Mlist,
                                  Glist, Slist);
}

std::vector<Eigen::MatrixXd> RobotControl::SimulateControl(
    const Eigen::VectorXd& thetalist,
    const Eigen::VectorXd& dthetalist,
    const Eigen::Vector3d& g,
    const Eigen::MatrixXd& Ftipmat,
    const std::vector<Eigen::MatrixXd>& Mlist,
    const std::vector<Eigen::MatrixXd>& Glist,
    const Eigen::MatrixXd& Slist,
    const Eigen::MatrixXd& thetamatd,
    const Eigen::MatrixXd& dthetamatd,
    const Eigen::MatrixXd& ddthetamatd,
    const Eigen::VectorXd& Kp,
    const Eigen::VectorXd& Ki,
    const Eigen::VectorXd& Kd,
    double dt,
    int intRes) {
  int steps = static_cast<int>(thetamatd.rows());
  int n = static_cast<int>(thetamatd.cols());
  Eigen::MatrixXd thetamat(steps, n);
  Eigen::MatrixXd dthetamat(steps, n);
  Eigen::VectorXd theta = thetalist;
  Eigen::VectorXd dtheta = dthetalist;
  Eigen::VectorXd eint = Eigen::VectorXd::Zero(n);

  if (steps == 0) {
    return {thetamat, dthetamat};
  }

  thetamat.row(0) = theta.transpose();
  dthetamat.row(0) = dtheta.transpose();

  for (int i = 0; i < steps - 1; ++i) {
    Eigen::VectorXd thetalistd = thetamatd.row(i).transpose();
    Eigen::VectorXd dthetalistd = dthetamatd.row(i).transpose();
    Eigen::VectorXd ddthetalistd = ddthetamatd.row(i).transpose();
    Eigen::VectorXd Ftip = Eigen::VectorXd::Zero(6);
    if (Ftipmat.size() != 0) {
      Ftip = Ftipmat.row(i).transpose();
    }

    Eigen::VectorXd e = thetalistd - theta;
    eint += e * dt;
    Eigen::VectorXd u = ddthetalistd + Kp.asDiagonal() * e +
                        Ki.asDiagonal() * eint +
                        Kd.asDiagonal() * (dthetalistd - dtheta);
    Eigen::VectorXd tau = InverseDynamics::Compute(
        theta, dtheta, u, g, Ftip, Mlist, Glist, Slist);

    for (int j = 0; j < intRes; ++j) {
      Eigen::VectorXd ddthetalist = Dynamics::ForwardDynamics(
          theta, dtheta, tau, g, Ftip, Mlist, Glist, Slist);
      auto next = Dynamics::EulerStep(theta, dtheta, ddthetalist,
                                      dt / static_cast<double>(intRes));
      theta = next.at(0);
      dtheta = next.at(1);
    }

    thetamat.row(i + 1) = theta.transpose();
    dthetamat.row(i + 1) = dtheta.transpose();
  }

  return {thetamat, dthetamat};
}
}  // namespace mymr
