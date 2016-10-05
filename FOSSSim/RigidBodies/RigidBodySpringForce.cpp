#include "RigidBodySpringForce.h"

RigidBodySpringForce::RigidBodySpringForce( const scalar& k, const scalar& l0, int rb0, const Vector2s& vc0, int rb1, const Vector2s& vc1, scalar b )
: m_k(k)
, m_l0(l0)
, m_rb0(rb0)
, m_rb1(rb1)
, m_vc0(vc0)
, m_vc1(vc1)
, m_b(b)
{
  assert( m_k >= 0.0 );
  assert( m_l0 >= 0.0 );
  assert( m_rb0 >= -1 );
  assert( m_rb1 >= -1 );
  assert( m_b >= 0 );
}

RigidBodySpringForce::~RigidBodySpringForce()
{
}

bool RigidBodySpringForce::isConservative() const
{
  return true;
}

int RigidBodySpringForce::getFirstRigidBody() const
{
  return m_rb0;
}

int RigidBodySpringForce::getSecondRigidBody() const
{
  return m_rb1;
}

const Vector2s& RigidBodySpringForce::getFirstEndpoint() const
{
  return m_vc0;
}

const Vector2s& RigidBodySpringForce::getSecondEndpoint() const
{
  return m_vc1;
}


bool RigidBodySpringForce::firstEndpointIsFixed() const
{
  return m_rb0 == -1;
}

bool RigidBodySpringForce::secondEndpointIsFixed() const
{
  return m_rb1 == -1;
}  

Vector2s RigidBodySpringForce::computeFirstEndpoint( const std::vector<RigidBody>& rbs ) const
{
  assert( m_rb0 >= -1 ); assert( m_rb0 < (int) rbs.size() );
  // If the endpoint is fixed, we store its world space position. Otherwise we must compute the world space position.
  return firstEndpointIsFixed() ? m_vc0 : rbs[m_rb0].computeWorldSpacePosition(m_vc0);
}

Vector2s RigidBodySpringForce::computeSecondEndpoint( const std::vector<RigidBody>& rbs ) const
{
  assert( m_rb1 >= -1 ); assert( m_rb1 < (int) rbs.size() );
  // If the endpoint is fixed, we store its world space position. Otherwise we must compute the world space position.
  return secondEndpointIsFixed() ? m_vc1 : rbs[m_rb1].computeWorldSpacePosition(m_vc1);
}

scalar RigidBodySpringForce::computePotentialEnergy( const std::vector<RigidBody>& rbs )
{
  assert( m_rb0 >= -1 ); assert( m_rb0 < (int) rbs.size() );
  assert( m_rb1 >= -1 ); assert( m_rb1 < (int) rbs.size() );
  
  // The endpoint positions depend on whether the endpoint is fixed in space or attached to a rigid body
  Vector2s x0 = firstEndpointIsFixed()  ? m_vc0 : rbs[m_rb0].computeWorldSpacePosition(m_vc0);
  Vector2s x1 = secondEndpointIsFixed() ? m_vc1 : rbs[m_rb1].computeWorldSpacePosition(m_vc1);

  scalar l = (x1-x0).norm();

  return 0.5*m_k*(l-m_l0)*(l-m_l0);
}

void RigidBodySpringForce::computeForceAndTorque( std::vector<RigidBody>& rbs )
{
  assert( m_rb0 >= -1 ); assert( m_rb0 < (int) rbs.size() );
  assert( m_rb1 >= -1 ); assert( m_rb1 < (int) rbs.size() );

  // Compute the direction the spring acts in
  Vector2s r0 = firstEndpointIsFixed()  ? m_vc0 : rbs[m_rb0].rotateIntoWorldSpace(m_vc0);
  Vector2s r1 = secondEndpointIsFixed() ? m_vc1 : rbs[m_rb1].rotateIntoWorldSpace(m_vc1);
  Vector2s v0 = rbs[m_rb0].computeWorldSpaceVelocity(rbs[m_rb0].computeWorldSpacePosition(m_vc0));
  Vector2s v1 = rbs[m_rb1].computeWorldSpaceVelocity(rbs[m_rb1].computeWorldSpacePosition(m_vc1));
  Vector2s nhat = r1-r0;
  if( !firstEndpointIsFixed() )  nhat -= rbs[m_rb0].getX();
  if( !secondEndpointIsFixed() ) nhat += rbs[m_rb1].getX();
  
  // Compute the length of the spring
  scalar l = nhat.norm();
  
  // If this is a 0 rest length spring and the length is 0, there is no force
  if( m_l0 == 0.0 && l == 0.0 ) return;
  
  // TODO: Some sort of workaround (assume direction is in rel-vel's direction, for example)
  // There is still the ambiguity when a non-zero reset length spring is compressed to zero rest length.
  assert( l != 0.0 );
  
  // Normalize the direction of the spring's action
  nhat /= l;

  // Compute the magnitude of the force
  Vector2s f = m_k * (l - m_l0) * nhat + m_b * (v1 - v0).dot(nhat) * nhat;
  
  if( !firstEndpointIsFixed() )  rbs[m_rb0].getForce() += f;
  if( !secondEndpointIsFixed() ) rbs[m_rb1].getForce() += -f;

  if( !firstEndpointIsFixed() )  rbs[m_rb0].getTorque() += mathutils::crossTwoD(r0,f);
  if( !secondEndpointIsFixed() ) rbs[m_rb1].getTorque() += mathutils::crossTwoD(r1,-f);
}

RigidBodyForce* RigidBodySpringForce::createNewCopy()
{
  return new RigidBodySpringForce( m_k, m_l0, m_rb0, m_vc0, m_rb1, m_vc1 );
}

