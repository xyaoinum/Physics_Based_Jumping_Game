#include "RigidBodyGravityForce.h"

RigidBodyGravityForce::RigidBodyGravityForce( const Vector2s& g )
: m_g(g)
{}

RigidBodyGravityForce::~RigidBodyGravityForce()
{}

bool RigidBodyGravityForce::isConservative() const
{
  return true;
}

scalar RigidBodyGravityForce::computePotentialEnergy( const std::vector<RigidBody>& rbs )
{
  scalar U = 0.0;
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i ) U += -rbs[i].getM()*m_g.dot(rbs[i].getX());
  return U;
}

void RigidBodyGravityForce::computeForceAndTorque( std::vector<RigidBody>& rbs )
{
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i ) if (rbs[i].tag() != "treat_of_trick_or_treat") rbs[i].getForce() += rbs[i].getM()*m_g;
}

RigidBodyForce* RigidBodyGravityForce::createNewCopy()
{
  return new RigidBodyGravityForce(m_g);
}
