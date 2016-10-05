#include "JointMotorBodyScene.h"

JointMotorBodyScene::JointMotorBodyScene():
RigidBodyScene(),
m_gravity(NULL)
{
  
}

JointMotorBodyScene::JointMotorBodyScene(const JointMotorBodyScene & scene) :
RigidBodyScene(scene),
m_jmbs(scene.m_jmbs),
m_gravity(scene.m_gravity)
{

}

void JointMotorBodyScene::parseScene()
{
  for (size_t i = 0; i < getForces().size(); i++)
    if (dynamic_cast<RigidBodyGravityForce *>(getForces()[i]))
      m_gravity = dynamic_cast<RigidBodyGravityForce *>(getForces()[i]);
  

}
