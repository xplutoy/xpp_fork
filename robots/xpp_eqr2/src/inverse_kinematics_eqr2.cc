/******************************************************************************
BSD-3-Clause
******************************************************************************/

#include <xpp_eqr2/inverse_kinematics_eqr2.h>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Cholesky>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

namespace xpp {

namespace {
template <typename T>
T Clamp(const T& value, const T& low, const T& high)
{
  return value.cwiseMax(low).cwiseMin(high);
}
} // namespace

constexpr double InverseKinematicsEqr2::kTolerance;
constexpr double InverseKinematicsEqr2::kFiniteDiffStep;
constexpr int InverseKinematicsEqr2::kMaxIterations;

InverseKinematicsEqr2::InverseKinematicsEqr2()
{
  using quad::LF;
  using quad::RF;
  using quad::LH;
  using quad::RH;

  legs_[LF] = {
    Vector3d(0.2375, 0.0625, -0.0225),   // 左前腿：髋部安装位偏移（机身坐标系下）
    Vector3d(0.075, 0.00975, 0.0),       // 关节2相对上一连杆的位移（连杆长度/偏移）
    Vector3d(-0.25, 0.12675, 0.0),       // 关节3相对上一连杆的位移（连杆长度/偏移）
    Vector3d(0.25045, 0.0, -0.11113),    // 脚端（末端执行器）相对关节3的固定偏移
    Vector3d(0.0, 1.0, 0.0),             // 关节2的转轴单位向量（在关节坐标系）
    Vector3d(0.0, 1.0, 0.0),             // 关节3的转轴单位向量（在关节坐标系）
    Vector3d(-0.838, -3.49, 0.0),        // 关节角最小值 q_min（弧度）：[髋滚转, 髋俯仰, 膝关节]
    Vector3d(0.838, 1.84, 1.86)          // 关节角最大值 q_max（弧度）：[髋滚转, 髋俯仰, 膝关节]
  };

  legs_[RF] = {
    Vector3d(0.2375, -0.0625, -0.0225),
    Vector3d(0.075, -0.00975, 0.0),
    Vector3d(-0.25, -0.12675, 0.0),
    Vector3d(0.25045, 0.0, -0.11113),
    Vector3d(0.0, 1.0, 0.0),
    Vector3d(0.0, 1.0, 0.0),
    Vector3d(-0.838, -3.49, 0.0),
    Vector3d(0.838, 1.84, 1.86)
  };

  legs_[LH] = {
    Vector3d(-0.2385, 0.0625, -0.0225),
    Vector3d(-0.075, 0.00975, 0.0),
    Vector3d(-0.25, 0.12675, 0.0),
    Vector3d(0.25045, 0.0, -0.11113),
    Vector3d(0.0, 1.0, 0.0),
    Vector3d(0.0, 1.0, 0.0),
    Vector3d(-0.838, -3.49, 0.0),
    Vector3d(0.838, 1.84, 1.86)
  };

  legs_[RH] = {
    Vector3d(-0.2385, -0.0625, -0.0225),
    Vector3d(-0.075, -0.00975, 0.0),
    Vector3d(-0.25, -0.12675, 0.0),
    Vector3d(0.25045, 0.0, -0.11113),
    Vector3d(0.0, 1.0, 0.0),
    Vector3d(0.0, 1.0, 0.0),
    Vector3d(-0.838, -3.49, 0.0),
    Vector3d(0.838, 1.84, 1.86)
  };

  last_solution_.fill(Vector3d::Zero());
  initialized_.fill(false);
}

Joints
InverseKinematicsEqr2::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  auto pos_B = x_B.ToImpl();
  pos_B.resize(kNumLegs, pos_B.front());

  std::vector<Eigen::VectorXd> q_legs;
  q_legs.reserve(kNumLegs);

  using quad::LF;
  using quad::RF;
  using quad::LH;
  using quad::RH;

  for (int leg_id = 0; leg_id < kNumLegs; ++leg_id) {
    Vector3d target = pos_B.at(leg_id);
    Vector3d q;

    bool solved = SolveLegIK(leg_id, target, q);
    if (!solved && initialized_.at(leg_id)) {
      q = last_solution_.at(leg_id);
    }

    Eigen::VectorXd q_vec(3);
    q_vec = q;

    q_legs.push_back(q_vec);
  }

  return Joints(q_legs);
}

Vector3d
InverseKinematicsEqr2::ForwardKinematics(int leg_id, const Vector3d& q) const
{
  const auto& leg = legs_.at(leg_id);

  Matrix4d T = Matrix4d::Identity();
  T *= MakeTranslation(leg.hip_offset);
  T *= MakeRotation(Vector3d(1.0, 0.0, 0.0), q(0));
  T *= MakeTranslation(leg.joint2_offset);
  T *= MakeRotation(leg.axis2, q(1));
  T *= MakeTranslation(leg.joint3_offset);
  T *= MakeRotation(leg.axis3, q(2));
  T *= MakeTranslation(leg.foot_offset);

  return T.block<3,1>(0,3);
}

bool
InverseKinematicsEqr2::SolveLegIK(int leg_id, const Vector3d& target, Vector3d& q) const
{
  const auto& leg = legs_.at(leg_id);
  const auto& q_min = leg.q_min;
  const auto& q_max = leg.q_max;

  Vector3d initial = initialized_.at(leg_id) ? last_solution_.at(leg_id) : InitialGuess(leg_id);
  q = Clamp(initial, q_min, q_max);

  for (int iter = 0; iter < kMaxIterations; ++iter) {
    Vector3d pos = ForwardKinematics(leg_id, q);
    Vector3d err = target - pos;

    if (err.norm() < kTolerance) {
      last_solution_[leg_id] = q;
      initialized_[leg_id] = true;
      return true;
    }

    Matrix3d J;
    Vector3d pos_eps;
    for (int i = 0; i < 3; ++i) {
      Vector3d dq = q;
      dq(i) += kFiniteDiffStep;
      pos_eps = ForwardKinematics(leg_id, dq);
      J.col(i) = (pos_eps - pos) / kFiniteDiffStep;
    }

    Matrix3d damping = 1e-4 * Matrix3d::Identity();
    Vector3d delta = (J.transpose()*J + damping).ldlt().solve(J.transpose()*err);

    if (!delta.array().isFinite().all()) {
      break;
    }

    q += delta;
    q = Clamp(q, q_min, q_max);
  }

  Vector3d final_err = target - ForwardKinematics(leg_id, q);
  bool success = final_err.norm() < 5e-3;
  if (success) {
    last_solution_[leg_id] = q;
    initialized_[leg_id] = true;
  }

  return success;
}

Vector3d
InverseKinematicsEqr2::InitialGuess(int leg_id) const
{
  (void)leg_id;
  return Vector3d(0.0, -0.90, 1.40);
}

Matrix4d
InverseKinematicsEqr2::MakeTranslation(const Vector3d& t)
{
  Matrix4d T = Matrix4d::Identity();
  T.block<3,1>(0,3) = t;
  return T;
}

Matrix4d
InverseKinematicsEqr2::MakeRotation(const Vector3d& axis, double angle)
{
  Matrix4d T = Matrix4d::Identity();
  if (axis.isZero(1e-9) || std::abs(angle) < 1e-12) {
    return T;
  }

  Eigen::AngleAxisd aa(angle, axis.normalized());
  T.block<3,3>(0,0) = aa.toRotationMatrix();
  return T;
}

} /* namespace xpp */


