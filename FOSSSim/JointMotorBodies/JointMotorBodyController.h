#ifndef __JOINT_MOTOR_BODY_CONTROLLER_H__
#define __JOINT_MOTOR_BODY_CONTROLLER_H__

#include <Eigen/Core>
#include "FOSSSim/MathDefs.h"
#include "JointMotorBody.h"
#include <iostream>
#include "../RigidBodies/RigidBodyForce.h"

class JointMotorBodyController : public RigidBodyForce
{
public:
  JointMotorBodyController(JointMotorBodyScene * scene, JointMotorBody * body);
  
  // If this force is conservative, computes the potential energy given all rigid bodies in the scene. 
  virtual scalar computePotentialEnergy( const std::vector<RigidBody>& rbs ) { return 0; /* non conservative */ }
  
  // Adds this force's contribution to the total force and torque acting on each rigid body.
  virtual void computeForceAndTorque( std::vector<RigidBody>& rbs );
  
  // True if this force is conservative.
  virtual bool isConservative() const { return false; }
  
  // Creates a new copy of this force.
  virtual RigidBodyForce * createNewCopy() { return new JointMotorBodyController(m_scene, m_body); }
  
protected:
  JointMotorBodyScene * m_scene;
  JointMotorBody * m_body;

};

#endif
