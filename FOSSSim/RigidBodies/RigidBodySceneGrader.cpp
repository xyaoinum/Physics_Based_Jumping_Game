#include "RigidBodySceneGrader.h"

RigidBodySceneGrader::RigidBodySceneGrader()
: m_accm_X_resid(0.0)
, m_accm_theta_resid(0.0)
, m_accm_V_resid(0.0)
, m_accm_omega_resid(0.0)
, m_max_X_resid(0.0)
, m_max_theta_resid(0.0)
, m_max_V_resid(0.0)
, m_max_omega_resid(0.0)
, m_pass_accm_X_resid(1.0e-6)
, m_pass_accm_theta_resid(1.0e-6)
, m_pass_accm_V_resid(1.0e-6)
, m_pass_accm_omega_resid(1.0e-6)
, m_pass_max_X_resid(1.0e-8)
, m_pass_max_theta_resid(1.0e-8)
, m_pass_max_V_resid(1.0e-8)
, m_pass_max_omega_resid(1.0e-8)
, m_pass_M_resid(1.0e-8)
, m_pass_X_resid(1.0e-8)
, m_pass_I_resid(1.0e-8)
, m_M_passed(true)
, m_X_passed(true)
, m_I_passed(true)
, m_pass_P_resid(1.0e-8)
, m_pass_Lcm_resid(1.0e-8)
, m_pass_Lspin_resid(1.0e-8)
, m_pass_Tcm_resid(1.0e-8)
, m_pass_Tspin_resid(1.0e-8)
, m_pass_U_resid(1.0e-8)
, m_P_passed(true)
, m_Lcm_passed(true)
, m_Lspin_passed(true)
, m_Tcm_passed(true)
, m_Tspin_passed(true)
, m_U_passed(true)
{}

void RigidBodySceneGrader::addDifferencesToResiduals( const RigidBodyScene& oracle_scene, const RigidBodyScene& testing_scene )
{
  // Get the rigid bodies from each scene
  const std::vector<RigidBody>& oraclebodies = oracle_scene.getRigidBodies();
  const std::vector<RigidBody>& testbodies = testing_scene.getRigidBodies();
  assert( oraclebodies.size() == testbodies.size() );

  // Compute the squared norm of the difference between all centers of mass and the maximum difference
  scalar x_cm_diff = 0.0;
  for( std::vector<RigidBody>::size_type i = 0; i < oraclebodies.size(); ++i )
  {
    scalar resid = (oraclebodies[i].getX()-testbodies[i].getX()).squaredNorm();
    x_cm_diff += resid;
    // Also update the maximum residual tracker
    m_max_X_resid = std::max(m_max_X_resid,sqrt(resid));
  }
  x_cm_diff = sqrt(x_cm_diff);
  m_accm_X_resid += x_cm_diff;

  // Compute the squared norm of the difference between all orientations and the maximum difference
  scalar theta_diff = 0.0;
  for( std::vector<RigidBody>::size_type i = 0; i < oraclebodies.size(); ++i )
  {
    scalar resid = (oraclebodies[i].getTheta()-testbodies[i].getTheta())*(oraclebodies[i].getTheta()-testbodies[i].getTheta());
    theta_diff += resid;
    // Also update the maximum residual tracker
    m_max_theta_resid = std::max(m_max_theta_resid,sqrt(resid));
  }
  theta_diff = sqrt(theta_diff);
  m_accm_theta_resid += theta_diff;

  // Compute the squared norm of the difference between all centers of mass' velcoities and the maximum difference
  scalar v_cm_diff = 0.0;
  for( std::vector<RigidBody>::size_type i = 0; i < oraclebodies.size(); ++i )
  {
    scalar resid = (oraclebodies[i].getV()-testbodies[i].getV()).squaredNorm();
    v_cm_diff += resid;
    m_max_V_resid = std::max(m_max_V_resid,sqrt(resid));
  }
  v_cm_diff = sqrt(v_cm_diff);
  m_accm_V_resid += v_cm_diff;

  // Compute the squared norm of the difference between all angular velocities and the maximum difference
  scalar omega_diff = 0.0;
  for( std::vector<RigidBody>::size_type i = 0; i < oraclebodies.size(); ++i )
  {
    scalar resid = (oraclebodies[i].getOmega()-testbodies[i].getOmega())*(oraclebodies[i].getOmega()-testbodies[i].getOmega());
    omega_diff += resid;
    m_max_omega_resid = std::max(m_max_omega_resid,sqrt(resid));
  }
  omega_diff = sqrt(omega_diff);
  m_accm_omega_resid += omega_diff;
}

void RigidBodySceneGrader::checkMassComputation( int rbidx, const scalar& Musr, const scalar& Morcl )
{
  bool passed = sqrt((Musr-Morcl)*(Musr-Morcl)) < m_pass_M_resid;
  m_M_passed = m_M_passed && passed;
  
  if( !passed )
  {
    std::cout << outputmod::startgreen << "Grading message: " << outputmod::endgreen;
    std::cout << "Mass not computed correctly for rigid body " << rbidx;
    std::cout << "   Provided M = " << Musr << "   Expected M = " << Morcl << std::endl;
  }
}

void RigidBodySceneGrader::checkCenterOfMassComputation( int rbidx, const Vector2s& Xusr, const Vector2s& Xorcl )
{
  bool passed = (Xusr-Xorcl).norm() < m_pass_X_resid;
  m_X_passed = m_X_passed && passed;
  
  if( !passed )
  {
    std::cout << outputmod::startgreen << "Grading message: " << outputmod::endgreen;
    std::cout << "Center of mass not computed correctly for rigid body " << rbidx;
    std::cout << "   Provided X = " << Xusr.transpose() << "   Expected X = " << Xorcl.transpose() << std::endl;
  }  
}

void RigidBodySceneGrader::checkMomentOfInertiaComputation( int rbidx, const scalar& Iusr, const scalar& Iorcl )
{
  bool passed = sqrt((Iusr-Iorcl)*(Iusr-Iorcl)) < m_pass_I_resid;
  m_I_passed = m_I_passed && passed;
  
  if( !passed )
  {
    std::cout << outputmod::startgreen << "Grading message: " << outputmod::endgreen;
    std::cout << "Mass not computed correctly for rigid body " << rbidx;
    std::cout << "   Provided I = " << Iusr << "   Expected I = " << Iorcl << std::endl;
  }  
}

void RigidBodySceneGrader::checkMomentumComputation( int rbidx, const Vector2s& Pusr, const Vector2s& Porcl )
{
  bool passed = (Pusr-Porcl).norm() < m_pass_P_resid;
  m_P_passed = m_P_passed && passed;
  
  if( !passed )
  {
    std::cout << outputmod::startgreen << "Grading message: " << outputmod::endgreen;
    std::cout << "Momentum not computed correctly for rigid body " << rbidx;
    std::cout << "   Provided P = " << Pusr.transpose() << "   Expected P = " << Porcl.transpose() << std::endl;
  }  
}

void RigidBodySceneGrader::checkCenterOfMassAngularMomentumComputation( int rbidx, const scalar& Lusr, const scalar& Lorcl )
{
  bool passed = sqrt((Lusr-Lorcl)*(Lusr-Lorcl)) < m_pass_I_resid;
  m_Lcm_passed = m_Lcm_passed && passed;
  
  if( !passed )
  {
    std::cout << outputmod::startgreen << "Grading message: " << outputmod::endgreen;
    std::cout << "Center of mass angular momentum not computed correctly for rigid body " << rbidx;
    std::cout << "   Provided Lcm = " << Lusr << "   Expected Lcm = " << Lorcl << std::endl;
  }
}

void RigidBodySceneGrader::checkSpinAngularMomentumComputation( int rbidx, const scalar& Lusr, const scalar& Lorcl )
{
  bool passed = sqrt((Lusr-Lorcl)*(Lusr-Lorcl)) < m_pass_I_resid;
  m_Lspin_passed = m_Lspin_passed && passed;
  
  if( !passed )
  {
    std::cout << outputmod::startgreen << "Grading message: " << outputmod::endgreen;
    std::cout << "Spin angular momentum not computed correctly for rigid body " << rbidx;
    std::cout << "   Provided Lspin = " << Lusr << "   Expected Lspin = " << Lorcl << std::endl;
  }  
}

void RigidBodySceneGrader::checkCenterOfMassKineticEnergyComputation( int rbidx, const scalar& Tusr, const scalar& Torcl )
{
  bool passed = sqrt((Tusr-Torcl)*(Tusr-Torcl)) < m_pass_Tcm_resid;
  m_Tcm_passed = m_Tcm_passed && passed;
  
  if( !passed )
  {
    std::cout << outputmod::startgreen << "Grading message: " << outputmod::endgreen;
    std::cout << "Center of mass kinetic energy not computed correctly for rigid body " << rbidx;
    std::cout << "   Provided Tcm = " << Tusr << "   Expected Tcm = " << Torcl << std::endl;
  } 
}

void RigidBodySceneGrader::checkSpinKineticEnergyComputation( int rbidx, const scalar& Tusr, const scalar& Torcl )
{
  bool passed = sqrt((Tusr-Torcl)*(Tusr-Torcl)) < m_pass_Tspin_resid;
  m_Tspin_passed = m_Tspin_passed && passed;
  
  if( !passed )
  {
    std::cout << outputmod::startgreen << "Grading message: " << outputmod::endgreen;
    std::cout << "Spin kinetic energy not computed correctly for rigid body " << rbidx;
    std::cout << "   Provided Tspin = " << Tusr << "   Expected Tspin = " << Torcl << std::endl;
  }
}

void RigidBodySceneGrader::checkTotalPotentialEnergyComputation( const scalar& Uusr, const scalar& Uorcl )
{
  bool passed = sqrt((Uusr-Uorcl)*(Uusr-Uorcl)) < m_pass_U_resid;
  m_U_passed = m_U_passed && passed;
  
  if( !passed )
  {
    std::cout << outputmod::startgreen << "Grading message: " << outputmod::endgreen;
    std::cout << "Total potential energy not computed correctly. ";
    std::cout << "   Provided U = " << Uusr << "   Expected U = " << Uorcl << std::endl;
  }
}

void RigidBodySceneGrader::printSummary( bool print_pass ) const
{
  std::cout << outputmod::startgreen << "Grading message: " << outputmod::endgreen;
  std::cout << (print_pass ? "Simulation ran to completion." : "Simulation did not complete, no pass/fail status will be reported") << std::endl;

  std::cout << outputmod::startgreen << "M     Computation(s):        " << outputmod::endgreen << std::setw(20) << (m_M_passed ? "correct" : "incorrect") << std::endl;
  std::cout << outputmod::startgreen << "X(0)  Computation(s):        " << outputmod::endgreen << std::setw(20) << (m_X_passed ? "correct" : "incorrect") << std::endl;
  std::cout << outputmod::startgreen << "I     Computation(s):        " << outputmod::endgreen << std::setw(20) << (m_I_passed ? "correct" : "incorrect") << std::endl;
  
  std::cout << outputmod::startgreen << "P     Computation(s):        " << outputmod::endgreen << std::setw(20) << (m_P_passed ? "correct" : "incorrect") << std::endl;
  std::cout << outputmod::startgreen << "Lcm   Computation(s):        " << outputmod::endgreen << std::setw(20) << (m_Lcm_passed ? "correct" : "incorrect") << std::endl;
  std::cout << outputmod::startgreen << "Lspin Computation(s):        " << outputmod::endgreen << std::setw(20) << (m_Lspin_passed ? "correct" : "incorrect") << std::endl;
  std::cout << outputmod::startgreen << "Tcm   Computation(s):        " << outputmod::endgreen << std::setw(20) << (m_Tcm_passed ? "correct" : "incorrect") << std::endl;
  std::cout << outputmod::startgreen << "Tspin Computation(s):        " << outputmod::endgreen << std::setw(20) << (m_Tspin_passed ? "correct" : "incorrect") << std::endl;
  std::cout << outputmod::startgreen << "U     Computation:           " << outputmod::endgreen << std::setw(20) << (m_U_passed ? "correct" : "incorrect") << std::endl;
  
  // If the simulation didn't run to completion, don't attempt to declare it a success or a failure
  if( !print_pass )
  {
    std::cout << outputmod::startgreen << "Accumulated X Residual:      " << outputmod::endgreen << std::setw(20) << m_accm_X_resid << std::endl;
    std::cout << outputmod::startgreen << "Accumulated Theta Residual:  " << outputmod::endgreen << std::setw(20) << m_accm_theta_resid << std::endl;
    std::cout << outputmod::startgreen << "Accumulated V Residual:      " << outputmod::endgreen << std::setw(20) << m_accm_V_resid << std::endl;
    std::cout << outputmod::startgreen << "Accumulated Omega Residual:  " << outputmod::endgreen << std::setw(20) << m_accm_omega_resid << std::endl;

    std::cout << outputmod::startgreen << "Maximum X Residual:          " << outputmod::endgreen << std::setw(20) << m_max_X_resid << std::endl;
    std::cout << outputmod::startgreen << "Maximum Theta Residual:      " << outputmod::endgreen << std::setw(20) << m_max_theta_resid << std::endl;
    std::cout << outputmod::startgreen << "Maximum V Residual:          " << outputmod::endgreen << std::setw(20) << m_max_V_resid << std::endl;
    std::cout << outputmod::startgreen << "Maximum Omega Residual:      " << outputmod::endgreen << std::setw(20) << m_max_omega_resid << std::endl;
    
    // TODO: Add checks that residual.txt was actually written!
    std::ofstream residfile;
    residfile.open("residual.txt");
    residfile << 0 << "   " << 999 << std::endl;
    residfile << 0 << "   " << 999 << std::endl;
    residfile << 0   << "   " << 999 << std::endl;
    residfile << 0   << "   " << 999 << std::endl;
    residfile << 0 << std::endl;
    residfile.close();      
    
    std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Wrote dummy values to residual.txt to avoid grading confusion." << std::endl;    
  }
  // If the simulation did run to completion, declare it as a success or a failure
  else
  {
    bool accm_X_pass = m_accm_X_resid < m_pass_accm_X_resid;
    bool accm_theta_pass = m_accm_theta_resid < m_pass_accm_theta_resid;
    bool accm_V_pass = m_accm_V_resid < m_pass_accm_V_resid;
    bool accm_omega_pass = m_accm_omega_resid < m_pass_accm_omega_resid;

    std::cout << outputmod::startgreen << "Accumulated X Residual:     " << outputmod::endgreen << std::setw(20) << m_accm_X_resid;
    std::cout << "\t\t" << (accm_X_pass ? "pass" : "fail") << std::endl;
    std::cout << outputmod::startgreen << "Accumulated Theta Residual: " << outputmod::endgreen << std::setw(20) << m_accm_theta_resid;
    std::cout << "\t\t" << (accm_theta_pass ? "pass" : "fail") << std::endl;
    std::cout << outputmod::startgreen << "Accumulated V Residual:     " << outputmod::endgreen << std::setw(20) << m_accm_V_resid;
    std::cout << "\t\t" << (accm_V_pass ? "pass" : "fail") << std::endl;
    std::cout << outputmod::startgreen << "Accumulated Omega Residual: " << outputmod::endgreen << std::setw(20) << m_accm_omega_resid;
    std::cout << "\t\t" << (accm_omega_pass ? "pass" : "fail") << std::endl;

    bool max_X_pass = m_max_X_resid < m_pass_max_X_resid;
    bool max_theta_pass = m_max_theta_resid < m_pass_max_theta_resid;
    bool max_V_pass = m_max_V_resid < m_pass_max_V_resid;
    bool max_omega_pass = m_max_omega_resid < m_pass_max_omega_resid;

    std::cout << outputmod::startgreen << "Maximum X Residual:         " << outputmod::endgreen << std::setw(20) << m_max_X_resid;
    std::cout << "\t\t" << (max_X_pass ? "pass" : "fail") << std::endl;
    std::cout << outputmod::startgreen << "Maximum Theta Residual:     " << outputmod::endgreen << std::setw(20) << m_max_theta_resid;
    std::cout << "\t\t" << (max_theta_pass ? "pass" : "fail") << std::endl;
    std::cout << outputmod::startgreen << "Maximum V Residual:         " << outputmod::endgreen << std::setw(20) << m_max_V_resid;
    std::cout << "\t\t" << (max_V_pass ? "pass" : "fail") << std::endl;
    std::cout << outputmod::startgreen << "Maximum Omega Residual:     " << outputmod::endgreen << std::setw(20) << m_max_omega_resid;
    std::cout << "\t\t" << (max_omega_pass ? "pass" : "fail") << std::endl;

    bool quantity_success = m_M_passed && m_X_passed && m_I_passed && m_P_passed && m_Lcm_passed && m_Lspin_passed && m_Tcm_passed && m_Tspin_passed && m_U_passed;
    bool resid_success = accm_X_pass && accm_theta_pass && accm_V_pass && accm_omega_pass && max_X_pass && max_theta_pass && max_V_pass && max_omega_pass;
    std::cout << "Overall success: " << ((quantity_success&&resid_success)?"Passed.":"Failed.") << std::endl;

    // TODO: Add checks that residual.txt was actually written!
    std::ofstream residfile;
    residfile.open("residual.txt");
    residfile << (accm_X_pass && accm_theta_pass) << "   " << (m_accm_X_resid+m_accm_theta_resid) << std::endl;
    residfile << (accm_V_pass && accm_omega_pass) << "   " << (m_accm_V_resid+m_accm_omega_resid) << std::endl;
    residfile << (max_X_pass && max_theta_pass)   << "   " << (m_max_X_resid+m_max_theta_resid) << std::endl;
    residfile << (max_V_pass && max_omega_pass)   << "   " << (m_max_V_resid+m_max_omega_resid) << std::endl;
    residfile << quantity_success << std::endl;
    residfile.close();      
    
    std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Saved residual to residual.txt." << std::endl;
  }
}

