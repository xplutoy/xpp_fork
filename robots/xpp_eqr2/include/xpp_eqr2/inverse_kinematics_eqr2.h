/******************************************************************************
BSD-3-Clause
******************************************************************************/

#ifndef XPP_EQR2_INVERSE_KINEMATICS_EQR2_H_
#define XPP_EQR2_INVERSE_KINEMATICS_EQR2_H_

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <xpp_vis/inverse_kinematics.h>

namespace xpp {

class InverseKinematicsEqr2 : public InverseKinematics {
public:
  InverseKinematicsEqr2();
  virtual ~InverseKinematicsEqr2() = default;

  Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;
  int GetEECount() const override { return 4; };

private:
  struct LegKinematics {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector3d hip_offset;
    Vector3d joint2_offset;
    Vector3d joint3_offset;
    Vector3d foot_offset;
    Vector3d axis2;
    Vector3d axis3;
    Vector3d q_min;
    Vector3d q_max;
  };

  static constexpr int kNumLegs = 4;

  Vector3d ForwardKinematics(int leg_id, const Vector3d& q) const;
  bool SolveLegIK(int leg_id, const Vector3d& target, Vector3d& q) const;
  Vector3d InitialGuess(int leg_id) const;

  static Eigen::Matrix4d MakeTranslation(const Vector3d& t);
  static Eigen::Matrix4d MakeRotation(const Vector3d& axis, double angle);

  std::array<LegKinematics, kNumLegs> legs_;
  mutable std::array<Vector3d, kNumLegs> last_solution_;
  mutable std::array<bool, kNumLegs> initialized_;

  static constexpr double kTolerance = 5e-7;
  static constexpr double kFiniteDiffStep = 1e-6;
  static constexpr int kMaxIterations = 120;
};

} /* namespace xpp */

#endif /* XPP_EQR2_INVERSE_KINEMATICS_EQR2_H_ */


