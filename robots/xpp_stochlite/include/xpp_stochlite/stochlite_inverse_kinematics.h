#ifndef XPP_VIS_STOCHLITE_INVERSE_KINEMATICS_H_
#define XPP_VIS_STOCHLITE_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>
#include <xpp_vis/inverse_kinematics.h>

#include "stochlite_kinematics.h"

namespace xpp {

enum StochJointID {ABD=0, HIP, KNEE, legJointCount};

/**
 * @brief Converts a hyq foot position to joint angles.
 */
class StochliteInverseKinematics: public InverseKinematics {
public:
  using Vector3d = Eigen::Vector3d;
  enum KneeBend { Forward, Backward };

  /**
   * @brief Default c'tor initializing leg lengths with standard values.
   */
  StochliteInverseKinematics ();
  virtual ~StochliteInverseKinematics () = default;

  /**
   * @brief Returns the joint angles to reach a Cartesian foot position.
   * @param ee_pos_H  Foot position xyz expressed in the frame attached
   * at the hip-aa (H).
   */
  Vector3d GetJointAngles(const Vector3d& ee_pos_H, KneeBend bend=Forward) const;

  /**
   * @brief Restricts the joint angles to lie inside the feasible range
   * @param q[in/out]  Current joint angle that is adapted if it exceeds
   * the specified range.
   * @param joint  Which joint (HAA, HFE, KFE) this value represents.
   */
  void EnforceLimits(double& q, StochJointID joint) const;

  /**
   * @brief Returns joint angles to reach for a specific foot position.
   * @param pos_B  3D-position of the foot expressed in the base frame (B).
   */
  Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  int GetEECount() const { return 4; };

private:
  //Needs to be corrected
  Vector3d hfe_to_haa_z = Vector3d(0.0, 0.0, 0.08); //distance of HFE to HAA in z direction
  double length_thigh = 0.35; // length of upper leg
  double length_shank = 0.33; // length of lower leg
  Vector3d leg_joint_shift = Vector3d(0.16695, 0.0964, 0.0);
  stochlite_control::StochliteKinematics kinematics_; // An object for doing kinematics on stochlite robot

};

StochliteInverseKinematics::StochliteInverseKinematics(): kinematics_(char('>'), char('>'))
{

}

StochliteInverseKinematics::Vector3d StochliteInverseKinematics::GetJointAngles(const Vector3d& ee_pos_B, KneeBend bend ) const
{
    // Implement 3R

    return Vector3d(1.0, 1.0, 1.0);
}

void StochliteInverseKinematics::EnforceLimits (double& val, StochJointID joint) const 
{
    // implement safety checks

    return ;
}

Joints StochliteInverseKinematics::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
    Vector3d ee_pos_H;
    auto pos_B = x_B.ToImpl();
    pos_B.resize(4, pos_B.front());

    std::vector<Eigen::VectorXd> q_vec;
    std::vector<double> foot_pos;
    for(int ee=0; ee<GetEECount(); ee++){
        // StochliteInverseKinematics::KneeBend bend = StochliteInverseKinematics::Forward;
        // q_vec.push_back(GetJointAngles(ee_pos_H, bend));
        using namespace quad;
        switch (ee) {
          case LF:
            ee_pos_H = pos_B.at(ee);
            foot_pos.push_back(ee_pos_H[0] - leg_joint_shift[0] );
            foot_pos.push_back(ee_pos_H[1] - leg_joint_shift[1] );
            foot_pos.push_back(ee_pos_H[2] - leg_joint_shift[2] );
            break;
          case RF:
            ee_pos_H = pos_B.at(ee);
            foot_pos.push_back(ee_pos_H[0] - leg_joint_shift[0] );
            foot_pos.push_back(ee_pos_H[1] + leg_joint_shift[1] );
            foot_pos.push_back(ee_pos_H[2] - leg_joint_shift[2] );
            break;
          case LH:
            ee_pos_H = pos_B.at(ee);
            foot_pos.push_back(ee_pos_H[0] + leg_joint_shift[0] );
            foot_pos.push_back(ee_pos_H[1] - leg_joint_shift[1] );
            foot_pos.push_back(ee_pos_H[2] - leg_joint_shift[2] );
            break;
          case RH:
            ee_pos_H = pos_B.at(ee);
            foot_pos.push_back(ee_pos_H[0] + leg_joint_shift[0] );
            foot_pos.push_back(ee_pos_H[1] + leg_joint_shift[1] );
            foot_pos.push_back(ee_pos_H[2] - leg_joint_shift[2] );
            break;
          default: // joint angles for this foot do not exist
            break;
        }
    }
    std::vector<double> target_joint_positions (12,0);
    kinematics_.inverseKinematics(foot_pos, target_joint_positions);

    for(int ee=0;ee<GetEECount();ee++){
      q_vec.push_back(Vector3d(target_joint_positions[ 3*ee + 0 ],
                     target_joint_positions[ 3*ee + 1 ],
                     target_joint_positions[ 3*ee + 2 ]
                    ));
    }

    return Joints(q_vec);
}

} /* namespace xpp */


#endif /* XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_ */