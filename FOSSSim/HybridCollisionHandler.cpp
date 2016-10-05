#include "HybridCollisionHandler.h"
#include "ContinuousTimeUtilities.h"
#include <iostream>
#include <set>
#include <algorithm>
#include "HybridCollisionComparison.h"
#include "CollisionDetector.h"

// Impact zone utility methods.
//
// Does not need to be modified by students.

bool intersects(const ImpactZone &z1, const ImpactZone &z2)
{
  std::set<int> temp;
  set_intersection(z1.m_verts.begin(), z1.m_verts.end(), z2.m_verts.begin(), z2.m_verts.end(), inserter(temp, temp.end()));
  return temp.size() > 0;
}

ImpactZone mergeZones(const ImpactZone &z1, const ImpactZone &z2)
{
  std::set<int> combinedverts;
  set_union(z1.m_verts.begin(), z1.m_verts.end(), z2.m_verts.begin(), z2.m_verts.end(), inserter(combinedverts, combinedverts.end()));
  return ImpactZone(combinedverts, z1.m_halfplane || z2.m_halfplane);
}


void mergeAllZones(ImpactZones &zones)
{
  ImpactZones result;

  ImpactZones *src = &zones;
  ImpactZones *dst = &result;
  do
    {
      dst->clear();
      for(int i=0; i<(int)src->size(); i++)
	{
	  bool merged = false;
	  for(int j=0; j<(int)dst->size(); j++)
	    {
	      if(intersects((*dst)[j], (*src)[i]))
		{
		  ImpactZone newzone = mergeZones((*dst)[j], (*src)[i]);
		  (*dst)[j] = newzone;
		  merged = true;

		  break;
		}
	    }
	  if(!merged)
	    {
	      dst->push_back((*src)[i]);
	    }
	}
      std::swap(src, dst);
    }
  while(src->size() < dst->size());

  zones = *dst;
}

void growImpactZones(const TwoDScene &scene, ImpactZones &zones, const std::vector<CollisionInfo> &impulses)
{
  for(int i=0; i<(int)impulses.size(); i++)
    {
      switch(impulses[i].m_type)
	{
	case CollisionInfo::PP:
	  {
	    std::set<int> verts;
	    verts.insert(impulses[i].m_idx1);
	    verts.insert(impulses[i].m_idx2);
	    zones.push_back(ImpactZone(verts, false));
	    break;
	  }
	case CollisionInfo::PE:
	  {
	    std::set<int> verts;
	    verts.insert(impulses[i].m_idx1);
	    verts.insert(scene.getEdge(impulses[i].m_idx2).first);
	    verts.insert(scene.getEdge(impulses[i].m_idx2).second);
	    zones.push_back(ImpactZone(verts, false));
	    break;
	  }
	case CollisionInfo::PH:
	  {
	    std::set<int> verts;
	    verts.insert(impulses[i].m_idx1);
	    zones.push_back(ImpactZone(verts, true));
	    break;
	  }
	}
    }
  mergeAllZones(zones);
}

bool zonesEqual(const ImpactZones &zones1, const ImpactZones &zones2)
{
  if(zones1.size() != zones2.size())
    return false;

  for(int i=0; i<(int)zones1.size(); i++)
    {
      bool found = false;
      for(int j=0; j<(int)zones2.size(); j++)
	{
	  if(zones1[i] == zones2[j])
	    {
	      found = true;
	      
	      break;
	    }
	}
      if(!found)
	return false;
    }
  return true;
}










HybridCollisionHandler::HybridCollisionHandler(int maxiters, double COR) : CollisionHandler(COR), m_maxiters(maxiters), m_in(NULL)
{
  m_out = new HybridCollisionComparison();
}

HybridCollisionHandler::~HybridCollisionHandler()
{
  delete m_out;
  if(m_in)
    delete m_in;
}

void HybridCollisionHandler::loadHybridData(std::ifstream &ifs)
{
  if(m_in)
    delete m_in;
  m_in = new HybridCollisionComparison();
  m_in->load(ifs);
}

bool HybridCollisionHandler::failed() {assert(m_in); return m_in->failed();}

void HybridCollisionHandler::serializeHybridData(std::ofstream &ofs)
{
  m_out->serialize(ofs);
}

// Applies hybrid collision response to the simulation.
// For up to m_maxiters iterations:
//  - Find all pairs of colliding primitives using continuous-time collision detection
//  - Simultaneously apply an impulse for each detected collision
// After m_maxiters iterations, resolves remaining collisions using the failsafe, as described in the assignment instructions
//
// Does not need to be modified by students
void HybridCollisionHandler::handleCollisions(TwoDScene &scene, CollisionDetector &detector, const VectorXs &oldpos, VectorXs &oldvel, scalar dt)
{
  m_out->reset();

  VectorXs xend(oldpos.size());
  VectorXs vend(oldvel.size());

  xend = scene.getX();
  vend = scene.getV();

  bool done = applyIterativeImpulses(scene, detector, oldpos, scene.getX(), scene.getV(), dt, xend, vend);

  m_out->serializePostImpulses(xend, vend);
  if(m_in) m_in->comparePI(xend, vend);

  scene.getX() = xend;
  scene.getV() = vend;

  if(!done)
    {
      applyGeometricCollisionHandling(scene, detector, oldpos, xend, vend, dt, scene.getX(), scene.getV());
    }
}

// Takes start of time step and end of time step positions, and performs continuous-time collision detection.
// Inputs:
//   scene:   The simulation scene, needed for radii, etc.
//   qs:      Start of time step positions.
//   qe:      End of time step positions.
// Output:
//   Returns a list of all collisions found after performing continuous-time collision detection using positions qs and qe.
std::vector<CollisionInfo> HybridCollisionHandler::detectCollisions(const TwoDScene &scene, CollisionDetector &detector, const VectorXs &qs, const VectorXs &qe)
{
  class HybridDetectionCallback : public DetectionCallback
  {
  public:

    HybridDetectionCallback(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, std::vector<CollisionInfo> &result, HybridCollisionHandler &hch) : scene(scene), qs(qs), qe(qe), result(result), hch(hch) {}

    virtual void ParticleParticleCallback(int idx1, int idx2)
    {
      VectorXs n(2);
      double time;
      if(hch.detectParticleParticle(scene, qs, qe, idx1, idx2, n, time))
      {
        result.push_back(CollisionInfo(CollisionInfo::PP, idx1, idx2, n, time));
      }
    }

    virtual void ParticleEdgeCallback(int vidx, int eidx)
    {
      VectorXs n(2);
      double time;
      if(hch.detectParticleEdge(scene, qs, qe, vidx, eidx, n, time))
      {
        result.push_back(CollisionInfo(CollisionInfo::PE, vidx, eidx, n, time));
      }
    }

    virtual void ParticleHalfplaneCallback(int vidx, int hidx)
    {
      VectorXs n(2);
      double time;
      if(hch.detectParticleHalfplane(scene, qs, qe, vidx, hidx, n, time))
      {
        result.push_back(CollisionInfo(CollisionInfo::PH, vidx, hidx, n, time));
      }
    }
    const TwoDScene &scene;
    const VectorXs &qs;
    const VectorXs &qe;
    std::vector<CollisionInfo> &result;
    HybridCollisionHandler &hch;
  };

  std::vector<CollisionInfo> result;
  HybridDetectionCallback callback(scene, qs, qe, result, *this);

  detector.performCollisionDetection(scene, qs, qe, callback);

  return result;
}

// Takes a list of detected collisions, positions at the start of time step, and predicted end of time step positions and velocities, and updates the
// predicted quantities by applying impulses.
// Inputs:
//   scene:      The simulation scene, needed for radii, masses, etc.
//   collisions: The list, generated by detectCollisions, of collisions that need to be responded to.
//   qs:         Start of time step positions.
//   qe:         Predicted end of time step positions.
//   qdote:      Predicted enf of time step velocities.
//   dt          The time step length
// Outputs:
//   qm:         Predicted end of time step positions after impulses have been applied. Do *NOT* pass in the same vector as for qe!
//   qdotm:      Predicted end of time step velocities after impulses have been applied.
void HybridCollisionHandler::applyImpulses(const TwoDScene &scene, const std::vector<CollisionInfo> &collisions, const VectorXs &qs, const VectorXs &qe, const VectorXs &qdote, double dt, VectorXs &qm, VectorXs &qdotm)
{
  qm = qe;
  qdotm = qdote;
  for(int i=0; i<(int)collisions.size(); i++)
    {
      switch(collisions[i].m_type)
	{
	case CollisionInfo::PP:
	  {
	    respondParticleParticle(scene, qs, qe, collisions[i].m_idx1, collisions[i].m_idx2, collisions[i].m_n, collisions[i].m_time, dt, qm, qdotm);
	    break;
	  }
	case CollisionInfo::PE:
	  {
	    respondParticleEdge(scene, qs, qe, collisions[i].m_idx1, collisions[i].m_idx2, collisions[i].m_n, collisions[i].m_time, dt, qm, qdotm);
	    break;
	  }
	case CollisionInfo::PH:
	  {
	    respondParticleHalfplane(scene, qs, qe, collisions[i].m_idx1, collisions[i].m_idx2, collisions[i].m_n, collisions[i].m_time, dt, qm, qdotm);
	    break;
	  }
	}
    }
}
      


std::string HybridCollisionHandler::getName() const
{
  return "Hybrid Collision Handling";
}

// Responds to a collision detected between two particles by applying an impulse
// to the velocities of each one.
void HybridCollisionHandler::respondParticleParticle(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int idx1, int idx2, const VectorXs &n, double time, double dt, VectorXs &qm, VectorXs &qdotm)
{
  const VectorXs &M = scene.getM();
  VectorXs dx = (qe-qs)/dt;

  VectorXs nhat = n;
  nhat.normalize();

  double cfactor = (1.0 + getCOR())/2.0;
  double m1 = scene.isFixed(idx1) ? std::numeric_limits<double>::infinity() : M[2*idx1];
  double m2 = scene.isFixed(idx2) ? std::numeric_limits<double>::infinity() : M[2*idx2];

  double numerator = 2*cfactor * (dx.segment<2>(2*idx2) - dx.segment<2>(2*idx1) ).dot(nhat);
  double denom1 = 1+m1/m2;
  double denom2 = m2/m1 + 1;

  if(!scene.isFixed(idx1))
    {
      qdotm.segment<2>(2*idx1) += numerator/denom1 * nhat;
      qm.segment<2>(2*idx1) += dt* numerator/denom1 * nhat;
    }
  if(!scene.isFixed(idx2))
    {
      qdotm.segment<2>(2*idx2) -= numerator/denom2 * nhat;
      qm.segment<2>(2*idx2) -= dt * numerator/denom2 * nhat;
    }
}

// Responds to a collision detected between a particle and an edge by applying
// an impulse to the velocities of each one.
void HybridCollisionHandler::respondParticleEdge(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int vidx, int eidx, const VectorXs &n, double time, double dt, VectorXs &qm, VectorXs &qdotm)
{
  const VectorXs &M = scene.getM();
  VectorXs dx = (qe-qs)/dt;

  int eidx1 = scene.getEdges()[eidx].first;
  int eidx2 = scene.getEdges()[eidx].second;
  

  VectorXs dx1 = dx.segment<2>(2*vidx);
  VectorXs dx2 = dx.segment<2>(2*eidx1);
  VectorXs dx3 = dx.segment<2>(2*eidx2);

  VectorXs x1 = qs.segment<2>(2*vidx) + time*dt*dx1;
  VectorXs x2 = qs.segment<2>(2*eidx1) + time*dt*dx2;
  VectorXs x3 = qs.segment<2>(2*eidx2) + time*dt*dx3;
  
  VectorXs nhat = n;
  nhat.normalize();

  double alpha = (x1-x2).dot(x3-x2)/(x3-x2).dot(x3-x2);
  alpha = std::min(1.0, std::max(0.0, alpha) );
  VectorXs vedge = dx2 + alpha*(dx3-dx2);
  double cfactor = (1.0 + getCOR())/2.0;

  double m1 = scene.isFixed(vidx) ? std::numeric_limits<double>::infinity() : M[2*vidx];
  double m2 = scene.isFixed(eidx1) ? std::numeric_limits<double>::infinity() : M[2*eidx1];
  double m3 = scene.isFixed(eidx2) ? std::numeric_limits<double>::infinity() : M[2*eidx2];

  double numerator = 2*cfactor*(vedge-dx1).dot(nhat);
  double denom1 = 1.0 + (1-alpha)*(1-alpha)*m1/m2 + alpha*alpha*m1/m3;
  double denom2 = m2/m1 + (1-alpha)*(1-alpha) + alpha*alpha*m2/m3;
  double denom3 = m3/m1 + (1-alpha)*(1-alpha)*m3/m2 + alpha*alpha;

  if(!scene.isFixed(vidx))
    {
      qdotm.segment<2>(2*vidx) += numerator/denom1 * nhat;
      qm.segment<2>(2*vidx) += dt*numerator/denom1 * nhat;
    }
  if(!scene.isFixed(eidx1))
    {
      qdotm.segment<2>(2*eidx1) -= (1.0-alpha)*numerator/denom2 * nhat;
      qm.segment<2>(2*eidx1) -= dt*(1.0-alpha)*numerator/denom2 * nhat;
    }
  if(!scene.isFixed(eidx2))
    {
      qdotm.segment<2>(2*eidx2) -= alpha * numerator/denom3 * nhat;
      qm.segment<2>(2*eidx2) -= dt*alpha*numerator/denom3*nhat;
    }

}


// Responds to a collision detected between a particle and a half-plane by 
// applying an impulse to the velocity of the particle.
void HybridCollisionHandler::respondParticleHalfplane(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int vidx, int pidx, const VectorXs &n, double time, double dt, VectorXs &qm, VectorXs &qdotm)
{
  VectorXs nhat = n;
  nhat.normalize();
  double cfactor = (1.0+getCOR())/2.0;
  VectorXs dx1 = (qe-qs)/dt;
  VectorXs oldqe = qe;
  qdotm.segment<2>(2*vidx) -= 2*cfactor*dx1.segment<2>(2*vidx).dot(nhat)*nhat;
  qm.segment<2>(2*vidx) -= dt*2*cfactor*dx1.segment<2>(2*vidx).dot(nhat)*nhat;
}






// Given the start position (oldpos) and end position (scene.getX) of two
// particles, and assuming the particles moved in a straight line between the
// two positions, determines whether the two particles were overlapping and
// approaching at any point during that motion.
bool HybridCollisionHandler::detectParticleParticle(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int idx1, int idx2, VectorXs &n, double &time)
{
  VectorXs dx = qe-qs;

  VectorXs x1 = qs.segment<2>(2*idx1);
  VectorXs x2 = qs.segment<2>(2*idx2);

  VectorXs dx1 = dx.segment<2>(2*idx1);
  VectorXs dx2 = dx.segment<2>(2*idx2);

  double r1 = scene.getRadius(idx1);
  double r2 = scene.getRadius(idx2);
  
  std::vector<double> pospoly;
  pospoly.push_back( -(dx2-dx1).dot(dx2-dx1) );
  pospoly.push_back( -2*(dx2-dx1).dot(x2-x1) );
  pospoly.push_back( (r1+r2)*(r1+r2) - (x2-x1).dot(x2-x1) );

  std::vector<double> velpoly;
  velpoly.push_back( -(dx1-dx2).dot(dx1-dx2) );
  velpoly.push_back( (dx1-dx2).dot(x2-x1) );

  std::vector<Polynomial> polys;
  polys.push_back(Polynomial(pospoly));
  polys.push_back(Polynomial(velpoly));

  time = PolynomialIntervalSolver::findFirstIntersectionTime(polys);
  if(time <= 1.0)
  {
    n = (x2+ time*dx2) - (x1+time*dx1);
    return true;
  }
  return false;
}

// Given start positions (oldpos) and end positions (scene.getX) of a
// particle and an edge, and assuming the particle and edge endpoints moved in 
// a straight line between the two positions, determines whether the two 
// objects were overlapping and approaching at any point during that motion.
bool HybridCollisionHandler::detectParticleEdge(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int vidx, int eidx, VectorXs &n, double &time)
{
  VectorXs dx = qe - qs;

  VectorXs x1 = qs.segment<2>(2*vidx);
  VectorXs x2 = qs.segment<2>(2*scene.getEdge(eidx).first);
  VectorXs x3 = qs.segment<2>(2*scene.getEdge(eidx).second);

  VectorXs dx1 = dx.segment<2>(2*vidx);
  VectorXs dx2 = dx.segment<2>(2*scene.getEdge(eidx).first);
  VectorXs dx3 = dx.segment<2>(2*scene.getEdge(eidx).second);

  double r1 = scene.getRadius(vidx);
  double r2 = scene.getEdgeRadii()[eidx];

  // poly = (a+bt + ct^2) - (d+2et+ft^2)(g+2ht+it^2) + rs(g+2ht+it^2)

  double a = (x1-x2).dot(x3-x2);
  double b = (x1-x2).dot(dx3-dx2) + (dx1-dx2).dot(x3-x2);
  double c = (dx1-dx2).dot(dx3-dx2);

  double d = (x2-x1).dot(x2-x1);
  double e = (dx2-dx1).dot(x2-x1);
  double f = (dx2-dx1).dot(dx2-dx1);

  double g = (x3-x2).dot(x3-x2);
  double h = (dx3-dx2).dot(x3-x2);
  double i = (dx3-dx2).dot(dx3-dx2);

  double rs = (r1+r2)*(r1+r2);

  std::vector<double> distpoly;
  distpoly.push_back( c*c - f*i );
  distpoly.push_back( 2*b*c - 2*e*i - 2*f*h );
  distpoly.push_back( 2*c*a + b*b - d*i - f*g -4*e*h + rs * i );
  distpoly.push_back( 2*a*b - 2*d*h - 2*e*g + 2*rs*h );
  distpoly.push_back( a*a - d*g + rs*g );

  std::vector<double> rightpoly;
  rightpoly.push_back( (dx1-dx2).dot(dx3-dx2) );
  rightpoly.push_back( (dx1-dx2).dot(x3-x2) + (x1-x2).dot(dx3-dx2) );
  rightpoly.push_back( (x1-x2).dot(x3-x2) );

  std::vector<double> leftpoly;
  leftpoly.push_back( (dx1-dx3).dot(dx2-dx3) );
  leftpoly.push_back( (dx1-dx3).dot(x2-x3) + (x1-x3).dot(dx2-dx3) );
  leftpoly.push_back( (x1-x3).dot(x2-x3) );


  // To think we almost put this bad boy in Milestone II
  std::vector<double> velpoly;
  {
    double a = (x3-x2).dot(x3-x2);
    double b = (x3-x2).dot(dx3-dx2);
    double c = (dx3-dx2).dot(dx3-dx2);
    double d = (dx2-dx1).dot(dx2-dx1);
    double e = (dx2-dx1).dot(x2-x1);
    double f = (x1-x2).dot(x3-x2);
    double g = (x1-x2).dot(dx3-dx2) + (dx1-dx2).dot(x3-x2);
    double h = (dx1-dx2).dot(dx3-dx2);
    double i = (dx3-dx2).dot(x2-x1) + (dx2-dx1).dot(x3-x2);
    double j = (dx3-dx2).dot(dx2-dx1);
    double k = a*f;
    double l = a*g+2*b*f;
    double m = a*h+2*b*g+c*f;
    double n = c*g+2*b*h;
    double o = c*h;
    double p = (dx3-dx2).dot(x3-x2);
    double q = (dx3-dx2).dot(dx3-dx2);
    
    velpoly.push_back( -h*h*q - c*c*d - 2*o*j );
    velpoly.push_back( -h*h*p - 2*g*h*q - 4*b*c*d - c*c*e - o*i - 2*n*j );
    velpoly.push_back( -2*g*h*p - 2*f*g*q - g*g*q - 2*a*c*d - 4*b*b*d - 4*b*c*e - n*i - 2*m*j );
    velpoly.push_back( -2*f*h*p - g*g*p - 2*f*g*q - 4*a*b*d - 2*a*c*e - 4*b*b*e - m*i - 2*l*j );
    velpoly.push_back( -2*f*g*p - f*f*q - a*a*d - 4*a*b*e - l*i - 2*k*j );
    velpoly.push_back( -f*f*p - a*a*e - k*i );
  }

  std::vector<Polynomial> polys;
  polys.push_back(distpoly);
  polys.push_back(rightpoly);
  polys.push_back(leftpoly);
  polys.push_back(velpoly);
 

  time = PolynomialIntervalSolver::findFirstIntersectionTime(polys);

  if(time <= 1.0)
  {
    double alpha = (x1 + time*dx1 - x2 - time*dx2).dot(x3+time*dx3-x2-time*dx2)/(x3+time*dx3-x2-time*dx2).dot(x3+time*dx3-x2-time*dx2);

    n = (x2+time*dx2) + alpha*(x3+time*dx3 -x2 - time*dx2) - x1 -time*dx1;

    if(n.norm()==0)
      return false;

    return true;
  }
  return false;
}

// Given start positions (oldpos) and end positions (scene.getX) of a
// particle and a half-plane, and assuming the particle endpoints moved in 
// a straight line between the two positions, determines whether the two 
// objects were overlapping and approaching at any point during that motion.
bool HybridCollisionHandler::detectParticleHalfplane(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int vidx, int pidx, VectorXs &n, double &time)
{
  VectorXs dx = qe - qs;

  VectorXs x1 = qs.segment<2>(2*vidx);
  VectorXs dx1 = dx.segment<2>(2*vidx);

  VectorXs xp = scene.getHalfplane(pidx).first;
  VectorXs np = scene.getHalfplane(pidx).second;
  
  double r = scene.getRadius(vidx);

  std::vector<double> pospoly;
  pospoly.push_back( -dx1.dot(np) * dx1.dot(np) );
  pospoly.push_back( 2*(xp-x1).dot(np) * dx1.dot(np) );
  pospoly.push_back( -(xp-x1).dot(np) * (xp-x1).dot(np) + r*r*np.dot(np));

  std::vector<double> velpoly;
  velpoly.push_back(- dx1.dot(np) * dx1.dot(np) );
  velpoly.push_back((xp-x1).dot(np) * dx1.dot(np) );

  std::vector<Polynomial> polys;
  polys.push_back(pospoly);
  polys.push_back(velpoly);

  time = PolynomialIntervalSolver::findFirstIntersectionTime(polys);

  if(time <= 1.0)
    {
      n = (xp - x1 - time*dx1).dot(np)/np.dot(np) * np;

      return true;
    }
  return false;
}



// BEGIN STUDENT CODE




// Iteratively performs collision detection and interative impulse response until either there are no more detected collisions, or the maximum number of
// iterations has been reached. See the assignment instructions for more details.
// The maximum number of iterations is stored in the member variable m_maxiters.
// Inputs:
//   scene:   The simulation scene. Get masses, radii, edge endpoint indices, etc. from here. Do *NOT* get any positions or velocities from here.
//   qs:      The positions of the particles at the start of the time step.
//   qe:      The predicted end-of-time-step positions.
//   qdote:   The predicted end-of-time-step velocities.
//   dt:      The time step size.
// Outputs:
//   qefinal:    The collision-free end-of-time-step positions (if no new collisions are detected), or the last set of predicted end-of-time-step positions
//               (if maximum number of iterations reached).
//   qdotefinal: Same as qefinal, but for velocities.
//   Returns true if the algorithm found a collision-free state. Returns false if the maximum number of iterations was reached without finding a collision-
//   free state.
// Possibly useful functions: detectCollisions, applyImpulses.
bool HybridCollisionHandler::applyIterativeImpulses(const TwoDScene &scene, CollisionDetector &detector, const VectorXs &qs, const VectorXs &qe, const VectorXs &qdote, double dt, VectorXs &qefinal, VectorXs &qdotefinal)
{
  std::vector<CollisionInfo> collisions = detectCollisions(scene, detector, qs, qe);
  qefinal = qe;
  qdotefinal = qdote;

  for(int iter=0; iter<m_maxiters && collisions.size() > 0; iter++)
    {
      VectorXs qenew = qefinal;
      VectorXs qdotenew = qdotefinal;
      applyImpulses(scene, collisions, qs, qefinal, qdotefinal, dt, qenew, qdotenew);
      qefinal = qenew;
      qdotefinal = qdotenew;
      collisions = detectCollisions(scene, detector, qs, qefinal);
    }
  return collisions.size() == 0;
}


// Resolves any remaining collisions in a simulation time step by setting the velocities of all particles involved in a way that guarantees
// that the distance between particles does not change.
// Inputs:
//   scene:   The simulation scene, from which the masses of the particles, current (colliding) positions, and whether or not a given particle is fixed,
//             can be retrieved.
//   qs:      The positions of the particles at the start of the time step.
//   qe:      The predicted end-of-timestep positions of the particles.
//   qdote:   The precicted end-of-timestep velocities of the particles.
//   zone:    Information about the impact zone of colliding particles. zone.m_verts is an std::set of particle indices; each particle in this set
//            is part of the impact zone and needs to have its position and velocity changed. Whether or not a half-plane is part of the impact zone
//            can be checked by looking at zone.m_halfplane.
//   dt:      The time step.
// Outputs:
//   qe:      For each particle in the impact zone, its position should be changed as described in the assignment instructions.
//   qdote:   For each particle in the impact zone, its velocity should be changed as described in the assignment instructions.
void HybridCollisionHandler::performFailsafe(const TwoDScene &scene, const VectorXs &qs, const ImpactZone &zone, double dt, VectorXs &qe, VectorXs &qdote)
{
  VectorXs dx = qe - qs;
  double totalmass = 0;
  VectorXs cm(2);
  cm.setZero();
  VectorXs p(2);
  p.setZero();
  double angularp = 0;

  double I = 0;

  bool hasfixed = false;

  for(std::set<int>::iterator it = zone.m_verts.begin(); it != zone.m_verts.end(); ++it)
    {
      cm += scene.getM()[2* *it] * qs.segment<2>(2* *it);
      p += scene.getM()[2* *it] * dx.segment<2>(2* *it);
      totalmass += scene.getM()[2* *it];
      if(scene.isFixed(*it))
	hasfixed = true;
    }
  
  cm /= totalmass;
  p /= totalmass;

  for(std::set<int>::iterator it = zone.m_verts.begin(); it != zone.m_verts.end(); ++it)
    {
      VectorXs v = dx.segment<2>(2* *it) - p;
      VectorXs r = qs.segment<2>(2* *it) - cm;
      double L = r[0]*v[1] - r[1]*v[0];
      L *= scene.getM()[2* *it];
      angularp += L;

      I += scene.getM()[2* *it] * r.dot(r);
    }

  if(hasfixed || zone.m_halfplane)
    {
      for(std::set<int>::iterator it = zone.m_verts.begin(); it != zone.m_verts.end(); ++it)
	{
	  qe.segment<2>(2* *it) = qs.segment<2>(2* *it);
	  qdote.segment<2>(2* *it)*=0;
	}
      return;
    }
  
  double w = angularp/I;

 for(std::set<int>::iterator it = zone.m_verts.begin(); it != zone.m_verts.end(); ++it)
   {
     VectorXs r = qs.segment<2>(2* *it) - cm;
     VectorXs rperp = r;
     std::swap(rperp[0], rperp[1]);
     rperp[0]*=-1;
     qe.segment<2>(2* *it) = cm + p + cos(w)*r + sin(w)*rperp;
     qdote.segment<2>(2* *it) = (qe.segment<2>(2* *it) - qs.segment<2>(2* *it))/dt;
   }
}


// Performs iterative geometric collision response until collision-free end-of-time-step positions and velocities are found. See the assignment
// instructions for details.
// Inputs:
//   scene:   The simulation scene. Get masses, radii, etc. from here. Do *NOT* get any positions or velocities from here.
//   qs:      The start-of-time-step positions.
//   qe:      The predicted end-of-time-step positions.
//   qdote:   The predicted end-of-time-step velocities.
//   dt:      The time step size.
// Outputs:
//   qm:    The final, collision-free end-of-time-step positions. (qm in the assignment instructions.)
//   qdotm: Same as qm, but for velocities.
// Possibly useful functions: detectCollisions, performFailsafe. You may find it helpful to write other helper functions for manipulating (merging,
// growing, etc) impact zones.
void HybridCollisionHandler::applyGeometricCollisionHandling(const TwoDScene &scene, CollisionDetector &detector, const VectorXs &qs, const VectorXs &qe, const VectorXs &qdote, double dt, VectorXs &qm, VectorXs &qdotm)
{
  // Initialization goes here

  ImpactZones Z;
  int iter=0;

  std::vector<CollisionInfo> collisions = detectCollisions(scene, detector, qs, qe);
  qm = qe;
  qdotm = qdote;

  growImpactZones(scene, Z, collisions);
  
  while(true)
    {
 
      // Make sure Z, qm, and qdotm are up to date (equal to what the algorithm in the instructions says they should be) here, or else you will fail test
      // scenes!
      m_out->serializeImpactZones(Z, qm, qdotm, iter); //DO NOT EDIT OR REMOVE
      if(m_in) {if(!m_in->compareIZ(Z, qm, qdotm, iter)) return;}    //or you will fail test scenes!
      iter++;

      // Body of algorithm loop goes here
      for(int i=0; i<(int)Z.size(); i++)
	performFailsafe(scene, qs, Z[i], dt, qm, qdotm);
      collisions = detectCollisions(scene, detector, qs, qm);
      ImpactZones oldzones=Z;
      growImpactZones(scene, Z, collisions);
      if(zonesEqual(oldzones, Z))
	break;
    }

  m_out->serializeImpactZones(Z, qm, qdotm, iter); //DO NOT EDIT OR REMOVE
  if(m_in) {if(!m_in->compareIZ(Z, qm, qdotm, iter)) return;}    //or you will fail test scenes!
  if(m_in) m_in->doneWithImpactZones(iter);
}
