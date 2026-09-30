#include <iostream>
#include <stdexcept>
#include <plan_manage/plan_container.hpp>

static void expect(const Eigen::Vector3d &actual, const Eigen::Vector3d &wanted)
{
  if (!actual.allFinite() || (actual - wanted).norm() > 1e-8)
    throw std::runtime_error("Unexpected trajectory sample");
}

static PolynomialTraj reference(double duration)
{
  PolynomialTraj traj;
  // x(t) = t^2, y(t) = z(t) = 0.
  traj.addSegment({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                  {0.0, 0.0, 0.0}, duration);
  traj.init();
  return traj;
}

int main()
{
  scan_planner::GlobalTrajData data;
  data.setGlobalTraj(reference(10.0), legbot::Time(0));
  expect(data.getPosition(8.0), {64.0, 0.0, 0.0});

  // Reproduce a stale progress query after replacing a long reference
  // with a shorter one, with no local spline present.
  data.setGlobalTraj(reference(2.0), legbot::Time(0));
  for (double t : {2.0, 2.0005, 8.0})
  {
    expect(data.getPosition(t), {4.0, 0.0, 0.0});
    expect(data.getVelocity(t), {4.0, 0.0, 0.0});
    expect(data.getAcceleration(t), {2.0, 0.0, 0.0});
  }
  expect(data.getPosition(-3.0), {0.0, 0.0, 0.0});
  expect(data.getVelocity(-3.0), {0.0, 0.0, 0.0});
  expect(data.getAcceleration(-3.0), {2.0, 0.0, 0.0});
  expect(data.getPosition(1.0), {1.0, 0.0, 0.0});

  // Retain the local-spline branch and polynomial sections on either side.
  Eigen::MatrixXd controls = Eigen::MatrixXd::Zero(3, 6);
  controls.row(0).setConstant(7.0);
  scan_planner::UniformBspline spline(controls, 3, 0.2);
  data.setLocalTraj(spline, 0.5, 1.1, 0.0);
  expect(data.getPosition(0.8), {7.0, 0.0, 0.0});
  expect(data.getVelocity(0.8), {0.0, 0.0, 0.0});
  expect(data.getAcceleration(0.8), {0.0, 0.0, 0.0});
  expect(data.getPosition(0.25), {0.0625, 0.0, 0.0});
  expect(data.getPosition(1.5), {2.25, 0.0, 0.0});
  expect(data.getPosition(8.0), {4.0, 0.0, 0.0});
  data.setGlobalTraj(reference(1.0), legbot::Time(0));
  expect(data.getPosition(8.0), {1.0, 0.0, 0.0});
  std::cout << "Global trajectory bounds regression passed\n";
}
