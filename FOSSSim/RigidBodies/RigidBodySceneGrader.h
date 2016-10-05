#ifndef __RIGID_BODY_SCENE_GRADER_H__
#define __RIGID_BODY_SCENE_GRADER_H__

#include "RigidBodyScene.h"
#include "FOSSSim/StringUtilities.h"

#include <iomanip>

class RigidBodySceneGrader
{
public:
  
  RigidBodySceneGrader();

  void addDifferencesToResiduals( const RigidBodyScene& oracle_scene, const RigidBodyScene& testing_scene );

  void checkMassComputation( int rbidx, const scalar& Musr, const scalar& Morcl );
  void checkCenterOfMassComputation( int rbidx, const Vector2s& Xusr, const Vector2s& Xorcl );
  void checkMomentOfInertiaComputation( int rbidx, const scalar& Iusr, const scalar& Iorcl );

  void checkMomentumComputation( int rbidx, const Vector2s& Pusr, const Vector2s& Porcl );
  void checkCenterOfMassAngularMomentumComputation( int rbidx, const scalar& Lusr, const scalar& Lorcl );
  void checkSpinAngularMomentumComputation( int rbidx, const scalar& Lusr, const scalar& Lorcl );
  void checkCenterOfMassKineticEnergyComputation( int rbidx, const scalar& Tusr, const scalar& Torcl );
  void checkSpinKineticEnergyComputation( int rbidx, const scalar& Tusr, const scalar& Torcl );
  void checkTotalPotentialEnergyComputation( const scalar& Uusr, const scalar& Uorcl );

  void printSummary( bool print_pass ) const;

private:
  // Total residual in center of mass positions
  scalar m_accm_X_resid;
  // Total residual in orientations
  scalar m_accm_theta_resid;
  // Total residual in center of mass' velocity
  scalar m_accm_V_resid;
  // Total residual in angular velocity about the center of mass
  scalar m_accm_omega_resid;

  // Max individual residual in center of mass positions
  scalar m_max_X_resid;
  // Max individual residual in orientations
  scalar m_max_theta_resid;
  // Max individual residual in center of mass' velocity
  scalar m_max_V_resid;
  // Max individual residual in angular velocity about the center of mass
  scalar m_max_omega_resid;
  
  // Acceptable residuals for the accumulated quantities
  scalar m_pass_accm_X_resid;
  scalar m_pass_accm_theta_resid;
  scalar m_pass_accm_V_resid;
  scalar m_pass_accm_omega_resid;
  
  // Acceptable residuals for the maximum quantities
  scalar m_pass_max_X_resid;
  scalar m_pass_max_theta_resid;
  scalar m_pass_max_V_resid;
  scalar m_pass_max_omega_resid;

  // Acceptable residuals for 'precomputed' quantities
  scalar m_pass_M_resid;
  scalar m_pass_X_resid;
  scalar m_pass_I_resid;

  // Track wheher the 'precomputed' quantities passed
  bool m_M_passed;
  bool m_X_passed;
  bool m_I_passed;
  
  // Acceptable residuals for 'per-step' quantities
  scalar m_pass_P_resid;
  scalar m_pass_Lcm_resid;
  scalar m_pass_Lspin_resid;
  scalar m_pass_Tcm_resid;
  scalar m_pass_Tspin_resid;
  scalar m_pass_U_resid;
  
  // Track wheher the 'per-step' quantities passed
  bool m_P_passed;
  bool m_Lcm_passed;
  bool m_Lspin_passed;
  bool m_Tcm_passed;
  bool m_Tspin_passed;
  bool m_U_passed;

};

#endif
