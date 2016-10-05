#ifndef __JOINT_MOTOR_BODY_SCENE_H__
#define __JOINT_MOTOR_BODY_SCENE_H__

#include "FOSSSim/MathDefs.h"
#include <iostream>
#include "JointMotorBody.h"
#include "FOSSSim/RigidBodies/RigidBodyScene.h"
#include "../RigidBodies/RigidBodyGravityForce.h"

class JointMotorBodyScene : public RigidBodyScene
{
public:
  JointMotorBodyScene();
  JointMotorBodyScene(const JointMotorBodyScene & scene);
  
  // body access
  
        JointMotorBody * getJointMotorBody(int i)       { assert(i >= 0 && i < (int)m_jmbs.size()); return (m_jmbs[i]); }
  const JointMotorBody * getJointMotorBody(int i) const { assert(i >= 0 && i < (int)m_jmbs.size()); return (m_jmbs[i]); }

        std::vector<JointMotorBody *> & getJointMotorBodies()       { return m_jmbs; }
  const std::vector<JointMotorBody *> & getJointMotorBodies() const { return m_jmbs; }
  
  // common parameters
  
  const Vector2s & gravity() const { return m_gravity->g(); }
  
  void set_key(char c) {
      cur_key = c;
  }
  
  const char get_key() {
      return cur_key;
  }
  
  void set_buf(int i) {
      buf = i;
  }
  
  const char get_buf() {
      return buf;
  }
  
  // construction
  void addJointMotorBody(JointMotorBody * jmb)  { m_jmbs.push_back(jmb); }
  

  void parseScene();
  
  scalar dt() const { return m_dt; }
  scalar & dt() { return m_dt; }
  
protected:
  
  char cur_key;
  int buf;
  std::vector<JointMotorBody *> m_jmbs;
  
  RigidBodyGravityForce * m_gravity;
  
  scalar m_dt;
  
};

#endif
