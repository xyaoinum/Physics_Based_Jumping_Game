#ifndef __JOINT_MOTOR_BODY_BODY_H__
#define __JOINT_MOTOR_BODY_BODY_H__

#include <Eigen/Core>
#include "FOSSSim/MathDefs.h"
#include <iostream>
#include "../RigidBodies/RigidBodyScene.h"

class JointMotorBodyScene;

class JointMotorBody
{
public:
  struct Link
  {
    // joint axis position in world space (unused for root)
    Vector2s axis;

    // topology
    Link * parent;
    std::vector<Link *> children;
    
    // the corresponding rigid body
    int rbindex;
    
    // axis in body spaces
    Vector2s axis_in_parent_body_space; // unused for root
    Vector2s axis_in_self_body_space;   // unused for root
    
  };
  
public:
  JointMotorBody(JointMotorBodyScene * scene);
  
  // link access
  int nlinks() const { return m_links.size(); }    
  Link * link(int i) { return m_links[i]; }
  const std::vector<Link *> getLinks() const { return m_links; }
  
  const RigidBody * rigidBody(const Link * l) const;
        RigidBody * rigidBody(const Link * l);
  
  // root access
  const Link * root() const { return m_links[0]; }
        Link * root()       { return m_links[0]; }
  
  // construction
  void addLink(Link * l, Link * parent);
  virtual void buildLinkSerialization();
  virtual void computeAxisVectors();
  
  // state access
  VectorXs & theta() { return m_theta; }
  const VectorXs & theta() const { return m_theta; }
    
protected:
  // scene reference for looking up the rigid bodies
  JointMotorBodyScene * m_scene;
  
  // serialized list of links (breadth first, element 0 is always the root)
  std::vector<Link *> m_links;
  
  // thetas for the joints (each component corresponds to the theta of a link in m_links)
  VectorXs m_theta;    // size: nlinks - 1, because root link has no motor
  
};

#endif
