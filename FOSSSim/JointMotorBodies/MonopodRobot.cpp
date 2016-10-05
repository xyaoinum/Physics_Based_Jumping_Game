#include "MonopodRobot.h"
#include "JointMotorBodyScene.h"

MonopodRobot::MonopodRobot(JointMotorBodyScene * scene, Link * head, Link * foot, scalar k, scalar l0, scalar l0min, scalar l0max, scalar b) :
  JointMotorBody(scene),
  m_leg_spring(k, l0, head->rbindex, foot->axis - rigidBody(head)->getX(), foot->rbindex, Vector2s(0, 0), b),
  m_head(head),
  m_foot(foot),
  m_l0(l0),
  m_l0min(l0min),
  m_l0max(l0max)
{
  m_links.clear();
  m_links.push_back(head);
  head->children.clear();
  head->parent = NULL;
  
  // due to the spring between the head and the foot, foot is not treated as a link (i.e. no joint coupling)

  foot->axis_in_self_body_space = foot->axis - rigidBody(foot)->getX();
  foot->axis_in_parent_body_space = foot->axis - rigidBody(head)->getX();
  
}

void MonopodRobot::setLegSpringRestLength(scalar l0)
{
  if (l0 < m_l0min)
    l0 = m_l0min;
  if (l0 > m_l0max)
    l0 = m_l0max;
  m_l0 = l0;
  m_leg_spring.m_l0 = l0;
}

scalar MonopodRobot::getLegSpringRestLength() const
{
  return m_l0;
}
