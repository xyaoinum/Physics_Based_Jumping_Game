#include "RigidBodyScene.h"

RigidBodyScene::RigidBodyScene()
: m_rbs()
, m_forces()
{}

RigidBodyScene::RigidBodyScene( const RigidBodyScene& otherscene )
: m_rbs(otherscene.m_rbs)
, m_forces()
{
  // Copy the forces from the other scene
  for( std::vector<RigidBodyForce*>::size_type i = 0; i < otherscene.m_forces.size(); ++i )
  {
    assert( otherscene.m_forces[i] != NULL );
    this->m_forces.push_back(otherscene.m_forces[i]->createNewCopy());
  }
}

RigidBodyScene::~RigidBodyScene()
{
  for( std::vector<RigidBodyForce*>::size_type i = 0; i < m_forces.size(); ++i ) 
  {
    if( m_forces[i] != NULL )
    {
      delete m_forces[i];
      m_forces[i] = NULL;
    }
  }
}

RigidBodyScene& RigidBodyScene::operator=( const RigidBodyScene& rhs )
{
  if( this == &rhs ) return *this;
  
  // Copy the rigid bodies from the other scene
  this->m_rbs = rhs.m_rbs;

  // Deallocate current forces
  for( std::vector<RigidBodyForce*>::size_type i = 0; i < this->m_forces.size(); ++i )
  {
    assert( this->m_forces[i] != NULL );
    delete this->m_forces[i];
    this->m_forces[i] = NULL;
  }
  m_forces.clear();

  // Copy the forces from the other scene
  for( std::vector<RigidBodyForce*>::size_type i = 0; i < rhs.m_forces.size(); ++i )
  {
    assert( rhs.m_forces[i] != NULL );
    this->m_forces.push_back(rhs.m_forces[i]->createNewCopy());
  }
  
  return *this;
}

void RigidBodyScene::insertForce( RigidBodyForce* force )
{
  assert( force != NULL );
  m_forces.push_back(force);
}

const std::vector<RigidBody>& RigidBodyScene::getRigidBodies() const
{
  return m_rbs;
}

std::vector<RigidBody>& RigidBodyScene::getRigidBodies()
{
    //std::cout <<"num of rbs: " <<  m_rbs.size() << std::endl;
  return m_rbs;
}

const RigidBody * RigidBodyScene::getRigidBody(int i) const
{
  assert(i >= 0 && i < (int)m_rbs.size());
  return &(m_rbs[i]);
}

RigidBody * RigidBodyScene::getRigidBody(int i)
{
  assert(i >= 0 && i < (int)m_rbs.size());
  return &(m_rbs[i]);
}

std::vector<RigidBodyForce*>& RigidBodyScene::getForces()
{
  return m_forces;
}

void RigidBodyScene::addRigidBody( const RigidBody& rb )
{
  m_rbs.push_back(rb);
}

Vector2s RigidBodyScene::computeTotalMomentum() const
{
  Vector2s p(0.0,0.0);
  for( std::vector<RigidBody>::size_type i = 0; i < m_rbs.size(); ++i ) p += m_rbs[i].computeTotalMomentum();
  return p;
}

scalar RigidBodyScene::computeTotalAngularMomentum() const
{
  scalar L = 0.0;
  for( std::vector<RigidBody>::size_type i = 0; i < m_rbs.size(); ++i ) L += m_rbs[i].computeTotalAngularMomentum();
  return L;  
}

scalar RigidBodyScene::computeKineticEnergy() const
{
  scalar K = 0.0;
  for( std::vector<RigidBody>::size_type i = 0; i < m_rbs.size(); ++i ) K += m_rbs[i].computeKineticEnergy();
  return K;
}

scalar RigidBodyScene::computeTotalPotentialEnergy()
{
  scalar U = 0.0;
  for( std::vector<RigidBodyForce*>::size_type i = 0; i < m_forces.size(); ++i ) 
    if( m_forces[i]->isConservative() ) U += m_forces[i]->computePotentialEnergy(m_rbs);
  return U;
}

void RigidBodyScene::serialize( std::ofstream& outputstream ) const
{
  assert( outputstream.is_open() );

  // Save the state of each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < m_rbs.size(); ++i ) m_rbs[i].serialize( outputstream );
}

void RigidBodyScene::deserialize( std::ifstream& inputstream )
{
  assert( inputstream.is_open() );

  // Load the state of each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < m_rbs.size(); ++i ) m_rbs[i].deserialize( inputstream );
}

