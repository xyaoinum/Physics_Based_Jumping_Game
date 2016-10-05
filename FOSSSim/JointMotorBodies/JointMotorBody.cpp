#include "JointMotorBody.h"
#include "JointMotorBodyScene.h"
#include <queue>

JointMotorBody::JointMotorBody(JointMotorBodyScene * scene) :
m_scene(scene),
m_links(),
m_theta()
{
  
}

void JointMotorBody::addLink(Link * l, Link * parent)
{
  if (m_links.size() == 0)
    assert(!parent);

  m_links.push_back(l);
  l->parent = parent;
  if (parent)
    parent->children.push_back(l);
  
}

void JointMotorBody::buildLinkSerialization()
{
  // this method assumes it is called at simulation initialization
  assert(m_links.size() > 0);
  std::queue<Link *> open;
  open.push(m_links.front());
  m_links.clear();
  while (open.size() > 0)
  {
    Link * front = open.front();
    m_links.push_back(front);
    open.pop();
    for (size_t i = 0; i < front->children.size(); i++)
      open.push(front->children[i]);
  }
}

void JointMotorBody::computeAxisVectors()
{
  // this method assumes it is called at simulation initialization (all rigid body at original position and zero rotation)
  assert(m_links.size() > 0);
  for (size_t i = 1; i < m_links.size(); i++)
  {
    Link * self_link = m_links[i];                                        assert(self_link);
    RigidBody * self_rb = m_scene->getRigidBody(self_link->rbindex);      assert(self_rb);
    Link * parent_link = self_link->parent;                               assert(parent_link);
    RigidBody * parent_rb = m_scene->getRigidBody(parent_link->rbindex);  assert(parent_rb);
    
    Vector2s self_cm = self_rb->getX();
    Vector2s parent_cm = parent_rb->getX();
    
    self_link->axis_in_self_body_space = self_link->axis - self_cm;
    self_link->axis_in_parent_body_space = self_link->axis - parent_cm;
    
  }
}

RigidBody * JointMotorBody::rigidBody(const Link * l)
{
  return m_scene->getRigidBody(l->rbindex); 
}

const RigidBody * JointMotorBody::rigidBody(const Link * l) const
{
  return m_scene->getRigidBody(l->rbindex); 
}
