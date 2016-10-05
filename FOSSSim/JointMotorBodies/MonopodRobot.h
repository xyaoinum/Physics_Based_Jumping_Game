#ifndef __MONOPOD_ROBOT_H__
#define __MONOPOD_ROBOT_H__

#include "JointMotorBody.h"
#include "../RigidBodies/RigidBodySpringForce.h"

class MonopodRobot : public JointMotorBody
{
  friend class JointMotorBodySimulationJudge;
  
public:
  MonopodRobot(JointMotorBodyScene * scene, Link * head, Link * foot, scalar k, scalar l0, scalar l0min, scalar l0max, scalar b);
  
  // control
  void setLegSpringRestLength(scalar l0);
  scalar getLegSpringRestLength() const;
  
  const RigidBodySpringForce * legSpringForce() const { return &m_leg_spring; }
        RigidBodySpringForce * legSpringForce()       { return &m_leg_spring; }
  
  const Link * head() const { return m_head; }
        Link * head()       { return m_head; }
  const Link * foot() const { return m_foot; }
        Link * foot()       { return m_foot; }

  scalar legSpringLength() const { return (rigidBody(head())->computeWorldSpacePosition(foot()->axis_in_parent_body_space) - rigidBody(foot())->getX()).norm(); }
  scalar legSpringRestLength() const { return m_leg_spring.l0(); }
  scalar legSpringRestLengthMin() const { return m_l0min; }
  scalar legSpringRestLengthMax() const { return m_l0max; }
  
protected:
  RigidBodySpringForce m_leg_spring;
  
  Link * m_head;
  Link * m_foot;
  
  scalar m_l0;
  scalar m_l0min;
  scalar m_l0max;
  
};

#endif
