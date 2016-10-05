#include "RigidBodyWindForce.h"


RigidBodyWindForce::RigidBodyWindForce( int num_quad_points, const scalar& beta, const Vector2s& wind )
: m_num_quadrature_points(num_quad_points)
, m_beta(beta)
, m_wind(wind)
{
  assert( m_num_quadrature_points >= 2 );
  assert( beta >= 0.0 );
}

RigidBodyWindForce::~RigidBodyWindForce()
{
}

bool RigidBodyWindForce::isConservative() const
{
  return false;
}

scalar RigidBodyWindForce::computePotentialEnergy( const std::vector<RigidBody>& rbs )
{
  // TODO: Clean up this warning
  std::cout << "WARNING: TRYING TO COMPUTE POTENTIAL ENERGY FOR A NON-CONSERVATIVE FORCE." << std::endl;
  
  return 0.0;
}
  
void RigidBodyWindForce::computeForceAndTorque( std::vector<RigidBody>& rbs )
{
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    const Vector2s& cm = rbs[i].getX();
    
    // For each 'edge' of the rigid body
    for( int j = 0; j < rbs[i].getNumEdges(); ++j )
    {
      // Starting point of the edge
      Vector2s p0 = rbs[i].getWorldSpaceVertex(j);
      // Edge itself
      Vector2s edge = rbs[i].computeWorldSpaceEdge(j);

      // Compute the outward facing edge normal
      Vector2s nhat = mathutils::rotateCounterClockwise90degrees( edge );
      
      // If the edge is not facing the wind force, exert no force
      //if( nhat.dot(m_wind) >= 0.0 ) continue;
      
      // Normalize the outward facing edge normal
      scalar l = nhat.norm();
      assert( l != 0.0 );
      nhat /= l;

      // Compute the points at which to evaluate velocities, forces, torques
      int num_quad_points = m_num_quadrature_points;
      scalar length_frac = 1.0/((scalar)num_quad_points);
      for( int k = 0; k < num_quad_points; ++k )
      {
        // Compute the world-space location of the point to exert the wind force on
        scalar bccoord = (0.5+((scalar)k))*length_frac;
        Vector2s x_i = p0 + bccoord*edge;
        // Compute the world-space velocity of the point to exert the wind force on
        Vector2s v_i = rbs[i].computeWorldSpaceVelocity( x_i );
        // Compute the velocity relative to the wind in the direction normal to the edge
        scalar vnorm = nhat.dot(m_wind-v_i);
        // If the normal relative velocity is not 'pushing', do not exert a force
        //if( vnorm >= 0.0 ) continue;
        // Compute the force on the point
        Vector2s f = m_beta*length_frac*l*vnorm*nhat;
        
        // Add the force's effect on center of mass acceleration
        rbs[i].getForce() += f;
        // Add the force's effect on angular acceleration
        rbs[i].getTorque() += mathutils::crossTwoD(x_i-cm,f);
      }
    }
  }
}

RigidBodyForce* RigidBodyWindForce::createNewCopy()
{
  return new RigidBodyWindForce(m_num_quadrature_points,m_beta,m_wind);
}

