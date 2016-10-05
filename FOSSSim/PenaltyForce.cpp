#include "PenaltyForce.h"
#include "TwoDScene.h"
#include "CollisionDetector.h"

PenaltyForce::PenaltyForce( const TwoDScene &scene, CollisionDetector &detector, const scalar stiffness, const scalar thickness )
: Force()
, m_scene(scene)
, m_detector(detector)
, m_k(stiffness)
, m_thickness(thickness)
{
}

PenaltyForce::~PenaltyForce()
{}
  
void PenaltyForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
  // Feel free to implement if you feel like doing so.
}

void PenaltyForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
  class PenaltyCallback : public DetectionCallback
  {
  public:
    PenaltyCallback(PenaltyForce &f, const VectorXs &x, VectorXs &gradE) : f(f), x(x), gradE(gradE) {}
    
    virtual void ParticleParticleCallback(int idx1, int idx2)
    {
      f.addParticleParticleGradEToTotal(x, idx1, idx2, gradE);
    }
    
    virtual void ParticleEdgeCallback(int vidx, int eidx)
    {
      f.addParticleEdgeGradEToTotal(x, vidx, eidx, gradE);
    }

    virtual void ParticleHalfplaneCallback(int vidx, int hidx)
    {
      f.addParticleHalfplaneGradEToTotal(x, vidx, hidx, gradE);
    }
  private:
    PenaltyForce &f;
    const VectorXs &x;
    VectorXs &gradE;
    
  };
  
  PenaltyCallback callback(*this, x, gradE);
  
  m_detector.performCollisionDetection(m_scene, x, x, callback);
}

void PenaltyForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert(!"Implicit integration of penalty forces is not supported in this milestone.");
  // Nothing to do.
}

void PenaltyForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert(!"Implicit integration of penalty forces is not supported in this milestone.");
  // Nothing to do.
}

Force* PenaltyForce::createNewCopy()
{
  return new PenaltyForce(*this);
}




// BEGIN STUDENT CODE


// Adds the gradient of the penalty potential (-1 * force) for a pair of 
// particles to the total.
// Read the positions of the particles from the input variable x. Radii can
// be obtained from the member variable m_scene, the penalty force stiffness 
// from member variable m_k, and penalty force thickness from member variable
// m_thickness.
// Inputs:
//   x:    The positions of the particles in the scene. 
//   idx1: The index of the first particle, i.e. the position of this particle
//         is ( x[2*idx1], x[2*idx1+1] ).
//   idx2: The index of the second particle.
// Outputs:
//   gradE: The total gradient of penalty force. *ADD* the particle-particle
//          gradient to this total gradient.
void PenaltyForce::addParticleParticleGradEToTotal(const VectorXs &x, int idx1, int idx2, VectorXs &gradE)
{

  double r1 = m_scene.getRadius(idx1);
  double r2 = m_scene.getRadius(idx2);

  if((x.segment<2>(2*idx1)-x.segment<2>(2*idx2)).squaredNorm() < (r1 + r2 + m_thickness)*(r1+r2+m_thickness))
    {
      VectorXs x1 = x.segment<2>(2*idx1);
      VectorXs x2 = x.segment<2>(2*idx2);
      
      VectorXs n = x2-x1;
      VectorXs nhat = n;
      nhat.normalize();
      if(n.norm() < 1e-10)
	return;
      nhat.normalize();
      gradE.segment<2>(2*idx1) -= m_k * (n.norm() - r1 - r2 - m_thickness) * nhat;
      gradE.segment<2>(2*idx2) += m_k * (n.norm() - r1 - r2 - m_thickness) * nhat;
    }
}

// Adds the gradient of the penalty potential (-1 * force) for a particle-edge
// pair to the total.
// Read the positions of the particle and edge endpoints from the input
// variable x.
// Inputs:
//   x:    The positions of the particles in the scene.
//   vidx: The index of the particle.
//   eidx: The index of the edge, i.e. the indices of the particle making up the
//         endpoints of the edge are given by m_scene.getEdge(eidx).first and 
//         m_scene.getEdges(eidx).second.
// Outputs:
//   gradE: The total gradient of penalty force. *ADD* the particle-edge
//          gradient to this total gradient.
void PenaltyForce::addParticleEdgeGradEToTotal(const VectorXs &x, int vidx, int eidx, VectorXs &gradE)
{
  VectorXs x1 = x.segment<2>(2*vidx);
  VectorXs x2 = x.segment<2>(2*m_scene.getEdge(eidx).first);
  VectorXs x3 = x.segment<2>(2*m_scene.getEdge(eidx).second);

  double r1 = m_scene.getRadius(vidx);
  double r2 = m_scene.getEdgeRadii()[eidx];

  double alpha = (x1-x2).dot(x3-x2)/(x3-x2).dot(x3-x2);
  if(alpha < 0)
    alpha = 0;
  if(alpha > 1)
    alpha = 1;

  VectorXs n = x2 + alpha*(x3-x2) - x1;
  VectorXs nhat=n;

  if(n.norm() < 1e-10)
    return;

  nhat.normalize();

  if(n.norm() < r1+r2 + m_thickness)
    {
      gradE.segment<2>(2*vidx) -= m_k * (n.norm() - r1 - r2 - m_thickness) * nhat;

      gradE.segment<2>(2*m_scene.getEdge(eidx).first) += m_k*(1-alpha)*(n.norm()-r1-r2 - m_thickness)*nhat;
      gradE.segment<2>(2*m_scene.getEdge(eidx).second) += m_k*alpha*(n.norm()-r1-r2 - m_thickness)*nhat;
    }
}

// Adds the gradient of the penalty potential (-1 * force) for a particle-
// half-plane pair to the total.
// Read the positions of the particle from the input variable x.
// Inputs:
//   x:    The positions of the particles in the scene.
//   vidx: The index of the particle.
//   pidx: The index of the half-plane, i.e. the position and normal vectors
//         for the half-plane can be retrieved by calling
//         m_scene.getHalfplane(pidx).
// Outputs:
//   gradE: The total gradient of the penalty force. *ADD* the particle-
//          half-plane gradient to this total gradient.
void PenaltyForce::addParticleHalfplaneGradEToTotal(const VectorXs &x, int vidx, int pidx, VectorXs &gradE)
{
  VectorXs x1 = x.segment<2>(2*vidx);
  VectorXs nh = m_scene.getHalfplane(pidx).second;
  VectorXs n = (m_scene.getHalfplane(pidx).first - x1).dot(nh)/(nh.dot(nh))*nh;
  VectorXs nhat = n;
  if(n.norm() < 1e-10)
    return;
  nhat.normalize();

  double r = m_scene.getRadius(vidx);

  if(n.norm() < r + m_thickness)
    {
      gradE.segment<2>(2*vidx) -= m_k*(n.norm()-r - m_thickness)*nhat.dot(nh)/(nh.dot(nh))*nh;
    }
}
