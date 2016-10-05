#ifndef __MONOPOD_ROBOT_CONTROLLER_H__
#define __MONOPOD_ROBOT_CONTROLLER_H__

#include "JointMotorBodyController.h"
#include "MonopodRobot.h"
#include "MonopodRobotControllerStrategy.h"

class MonopodRobotController : public JointMotorBodyController
{
public:
  MonopodRobotController(JointMotorBodyScene * scene, MonopodRobot * robot);
  
  MonopodRobot * robot() { return m_robot; }
  
  // If this force is conservative, computes the potential energy given all rigid bodies in the scene. 
  virtual scalar computePotentialEnergy( const std::vector<RigidBody>& rbs ) { return 0; /* non conservative */ }
  
  // Adds this force's contribution to the total force and torque acting on each rigid body.
  virtual void computeForceAndTorque( std::vector<RigidBody>& rbs );
  
  // True if this force is conservative.
  virtual bool isConservative() const { return false; }
  
  // Creates a new copy of this force.
  virtual RigidBodyForce * createNewCopy() { return new MonopodRobotController(m_scene, m_robot); }
  
protected:
  MonopodRobotControllerStrategy* m_strategy;
  MonopodRobot * m_robot;

};

#endif
