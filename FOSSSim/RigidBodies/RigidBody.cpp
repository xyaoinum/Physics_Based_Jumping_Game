#include "RigidBody.h"

RigidBody::RigidBody( const Vector2s& v, const scalar& omega, const VectorXs& vertices, const VectorXs& masses, const scalar& radius, const bool& fixed )
: m_fixed(fixed)
, m_M(computeTotalMass(masses))
, m_masses(masses)
, m_I(computeMomentOfInertia(vertices,masses))
, m_vertices(vertices)
, m_r(radius)
, m_X(computeCenterOfMass(vertices,masses))
, m_theta(0.0)
, m_V(v)
, m_omega(omega)
// theta == 0 => identity matrix
, m_R(Matrix2s::Identity())
, m_F(0.0,0.0)
, m_tau(0.0)
, collision_enable(true)
{
  assert( (masses.array()>0.0).all() );
  assert( m_M > 0.0 );
//  assert( m_I > 0.0 );
//  assert( m_vertices.size() >= 2 );
  assert( m_r >= 0.0 );

  // Translate the rigid body so the body space center of mass is the origin
  for( int i = 0; i < m_vertices.size()/2; ++i ) m_vertices.segment<2>(2*i) -= m_X;
}

void RigidBody::updateDerivedQuantities()
{
  // Update the rotation matrix representation of orientation
  m_R << cos(m_theta), -sin(m_theta), sin(m_theta), cos(m_theta);
}

// TODO: Add some sanity checks in here about applying forces and stuff
bool RigidBody::isFixed() const
{
  return m_fixed;
}


Vector2s& RigidBody::getX()
{
  return m_X;
}

const Vector2s& RigidBody::getX() const
{
  return m_X;
}

scalar& RigidBody::getTheta()
{
  return m_theta;
}

const scalar& RigidBody::getTheta() const
{
  return m_theta;
}

Vector2s& RigidBody::getV()
{
  return m_V;
}

const Vector2s& RigidBody::getV() const
{
  return m_V;
}

scalar& RigidBody::getOmega()
{
  return m_omega;
}

const scalar& RigidBody::getOmega() const
{
  return m_omega;
}

const scalar& RigidBody::getM() const
{
  return m_M;
}

const scalar& RigidBody::getI() const
{
  return m_I;
}

const scalar& RigidBody::getRadius() const
{
  return m_r;
}

int RigidBody::getNumVertices() const
{
  assert( m_vertices.size()%2 == 0 );
  return m_vertices.size()/2;
}

int RigidBody::getNumEdges() const
{
  return getNumVertices();
}

Vector2s RigidBody::getWorldSpaceVertex( int i ) const
{
  assert( i >= 0 ); assert( i < getNumVertices() );
  // Rotate body-space coordinate about center of mass, then translate by center of mass
  return m_R*m_vertices.segment<2>(2*i)+m_X;
}

Vector2s RigidBody::rotateIntoWorldSpace( const Vector2s& bodyvec ) const
{
  return m_R*bodyvec;
}

Vector2s RigidBody::computeWorldSpacePosition( const Vector2s& bodyvec ) const
{
  return m_R*bodyvec+m_X;
}

Vector2s RigidBody::computeBodySpacePosition( const Vector2s& worldvec ) const
{
	return m_R.transpose() * (worldvec - m_X);
}

Vector2s RigidBody::computeWorldSpaceVelocity( const Vector2s& worldposition ) const
{
  return m_V + m_omega*mathutils::rotateCounterClockwise90degrees(worldposition-m_X);
}

Vector2s RigidBody::computeWorldSpaceVelocityGivenPositionRelativeToCM( const Vector2s& posnreltocm ) const
{
  return m_V + m_omega*mathutils::rotateCounterClockwise90degrees(posnreltocm);
}

Vector2s RigidBody::computeWorldSpaceEdge( int i ) const
{
  assert( i >= 0 );
  assert( i < getNumEdges() );
  
  int i0 = i;
  int i1 = (i+1)%getNumVertices();
  
  return m_R*(m_vertices.segment<2>(2*i1)-m_vertices.segment<2>(2*i0));
}

Vector2s& RigidBody::getForce()
{
  return m_F;
}

scalar& RigidBody::getTorque()
{
  return m_tau;
}

Vector2s RigidBody::computeTotalMomentum() const
{
  return m_M*m_V;
}

scalar RigidBody::computeCenterOfMassAngularMomentum() const
{
  return m_M*mathutils::crossTwoD(m_X,m_V);
}

scalar RigidBody::computeSpinAngularMomentum() const
{
  return m_I*m_omega;
}

scalar RigidBody::computeTotalAngularMomentum() const
{
  // Linear and rotational components
  return computeCenterOfMassAngularMomentum() + computeSpinAngularMomentum();
}

scalar RigidBody::computeCenterOfMassKineticEnergy() const
{
  return 0.5*m_M*m_V.dot(m_V);
}

scalar RigidBody::computeSpinKineticEnergy() const
{
  return 0.5*m_I*m_omega*m_omega;
}

scalar RigidBody::computeKineticEnergy() const
{
  // Linear and rotational component
  return computeCenterOfMassKineticEnergy() + computeSpinKineticEnergy();
}



// !!!WARNING!!! DO NOT MODIFY THIS METHOD OR YOUR CODE WILL FAIL
void RigidBody::serialize( std::ofstream& outputstream ) const
{
  assert( outputstream.is_open() );

  // Serialize the center of mass
  outputstream.write((char*)m_X.data(),m_X.size()*sizeof(scalar));
  // Serialize the orientation
  outputstream.write((char*)&m_theta,sizeof(scalar));
  // Serialize the center of mass' velocity
  outputstream.write((char*)m_V.data(),m_V.size()*sizeof(scalar));
  // Serialize the angular velocity
  outputstream.write((char*)&m_omega,sizeof(scalar));
}

// !!!WARNING!!! DO NOT MODIFY THIS METHOD OR YOUR CODE WILL FAIL
void RigidBody::deserialize( std::ifstream& inputstream )
{
  assert( inputstream.is_open() );

  // Deserialize the center of mass
  inputstream.read((char*)m_X.data(),m_X.size()*sizeof(scalar));
  // Deserialize the orientation
  inputstream.read((char*)&m_theta,sizeof(scalar));
  // Deserialize the center of mass' velocity
  inputstream.read((char*)m_V.data(),m_V.size()*sizeof(scalar));
  // Deserialize the angular velocity
  inputstream.read((char*)&m_omega,sizeof(scalar));
  
  updateDerivedQuantities();
}



scalar RigidBody::computeTotalMass( const VectorXs& masses ) const
{
  return masses.sum();
}

Vector2s RigidBody::computeCenterOfMass( const VectorXs& vertices, const VectorXs& masses ) const
{
  assert( vertices.size()%2 == 0 );
  assert( 2*masses.size() == vertices.size() );

  // TODO: By using a partial reduction from Eigen, we could make this a single line of vectorized code :). 
  
  Vector2s cm(0.0,0.0);
  for( int i = 0; i < vertices.size()/2; ++i ){
       //std::cout << "masses(i): " << masses(i) << std::endl;
       
       
      cm += masses(i)*vertices.segment<2>(2*i);
  }
  
  scalar M = masses.sum();  
  
  //std::cout << "M: " << M << std::endl;
  
  assert( M > 0.0 );
  cm /= M;

  return cm;
}

scalar RigidBody::computeMomentOfInertia( const VectorXs& vertices, const VectorXs& masses ) const
{
  assert( vertices.size()%2 == 0 );
  assert( 2*masses.size() == vertices.size() );

  // TODO: By using a partial reduction from Eigen, we could make this a single line of vectorized code :). 
  
  Vector2s cm = computeCenterOfMass(vertices,masses);
  scalar I = 0.0;
  for( int i = 0; i < masses.size(); ++i ) I += masses(i)*(vertices.segment<2>(2*i)-cm).squaredNorm();
  if (masses.size() == 1) // single vertex rigid body, no rotation dof, zero moment of inertia
    I = 0;
  return I;
}
