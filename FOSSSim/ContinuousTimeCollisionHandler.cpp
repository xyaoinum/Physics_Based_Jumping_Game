#include "ContinuousTimeCollisionHandler.h"
#include <iostream>
#include "ContinuousTimeUtilities.h"
#include "CollisionDetector.h"

// Loops over all edges, particles, and half-planes in the simulation, and
// applies collision detection and handling to each pair.
//
// Note: No collision detection is done between edges that are connected
// by a particle (since these are always colliding.)
//
// Does not need to be changed by students.

void ContinuousTimeCollisionHandler::handleCollisions(TwoDScene &scene, CollisionDetector &detector, const VectorXs &oldpos, VectorXs &oldvel, scalar dt)
{
  class ContinuousTimeCallback : public DetectionCallback
  {
  public:
    ContinuousTimeCallback(TwoDScene &scene, const VectorXs &oldpos, double dt, ContinuousTimeCollisionHandler &ctch) : scene(scene), oldpos(oldpos), dt(dt), ctch(ctch) {}

    virtual void ParticleParticleCallback(int idx1, int idx2)
    {
      VectorXs n(2);
      double time;
      if(ctch.detectParticleParticle(scene, oldpos, idx1, idx2, n, time))
      {
        ctch.addParticleParticleImpulse(idx1, idx2, n, time);
        ctch.respondParticleParticle(scene, oldpos, idx1, idx2, n, time, dt);
      }
    }

    virtual void ParticleEdgeCallback(int vidx, int eidx)
    {
      VectorXs n(2);
      double time;
      if(ctch.detectParticleEdge(scene, oldpos, vidx, eidx, n, time))
      {
        ctch.addParticleEdgeImpulse(vidx, eidx, n, time);
        ctch.respondParticleEdge(scene, oldpos, vidx, eidx, n, time, dt);
      }
    }

    virtual void ParticleHalfplaneCallback(int vidx, int hidx)
    {
      VectorXs n(2);
      double time;
      if(ctch.detectParticleHalfplane(scene, oldpos, vidx, hidx, n, time))
      {
        ctch.addParticleHalfplaneImpulse(vidx, hidx, n, time);
        ctch.respondParticleHalfplane(scene, oldpos, vidx, hidx, n, time, dt);
      }
    }

    TwoDScene &scene;
    const VectorXs &oldpos;
    double dt;
    ContinuousTimeCollisionHandler &ctch;

  };

  ContinuousTimeCallback callback(scene, oldpos, dt, *this);

  detector.performCollisionDetection(scene, oldpos, scene.getX(), callback);
}

std::string ContinuousTimeCollisionHandler::getName() const
{
  return "Continuous-Time Collision Handling";
}

// Responds to a collision detected between two particles by applying an impulse
// to the velocities of each one.
void ContinuousTimeCollisionHandler::respondParticleParticle(TwoDScene &scene, const VectorXs &oldpos, int idx1, int idx2, const VectorXs &n, double time, double dt)
{
  const VectorXs &M = scene.getM();
  VectorXs dx = (scene.getX()-oldpos)/dt;
  VectorXs &newv = scene.getV();
  VectorXs &newx = scene.getX();

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
      newv.segment<2>(2*idx1) += numerator/denom1 * nhat;
      newx.segment<2>(2*idx1) += dt* numerator/denom1 * nhat;
    }
  if(!scene.isFixed(idx2))
    {
      newv.segment<2>(2*idx2) -= numerator/denom2 * nhat;
      newx.segment<2>(2*idx2) -= dt * numerator/denom2 * nhat;
    }
}

// Responds to a collision detected between a particle and an edge by applying
// an impulse to the velocities of each one.
void ContinuousTimeCollisionHandler::respondParticleEdge(TwoDScene &scene, const VectorXs &oldpos, int vidx, int eidx, const VectorXs &n, double time, double dt)
{
  const VectorXs &M = scene.getM();
  VectorXs dx = (scene.getX()-oldpos)/dt;
  VectorXs &newv = scene.getV();
  VectorXs &newx = scene.getX();

  int eidx1 = scene.getEdges()[eidx].first;
  int eidx2 = scene.getEdges()[eidx].second;
  

  VectorXs dx1 = dx.segment<2>(2*vidx);
  VectorXs dx2 = dx.segment<2>(2*eidx1);
  VectorXs dx3 = dx.segment<2>(2*eidx2);

  VectorXs x1 = oldpos.segment<2>(2*vidx) + time*dt*dx1;
  VectorXs x2 = oldpos.segment<2>(2*eidx1) + time*dt*dx2;
  VectorXs x3 = oldpos.segment<2>(2*eidx2) + time*dt*dx3;
  
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
      newv.segment<2>(2*vidx) += numerator/denom1 * nhat;
      newx.segment<2>(2*vidx) += dt*numerator/denom1 * nhat;
    }
  if(!scene.isFixed(eidx1))
    {
      newv.segment<2>(2*eidx1) -= (1.0-alpha)*numerator/denom2 * nhat;
      newx.segment<2>(2*eidx1) -= dt*(1.0-alpha)*numerator/denom2 * nhat;
    }
  if(!scene.isFixed(eidx2))
    {
      newv.segment<2>(2*eidx2) -= alpha * numerator/denom3 * nhat;
      newx.segment<2>(2*eidx2) -= dt*alpha*numerator/denom3*nhat;
    }

}


// Responds to a collision detected between a particle and a half-plane by 
// applying an impulse to the velocity of the particle.
void ContinuousTimeCollisionHandler::respondParticleHalfplane(TwoDScene &scene, const VectorXs &oldpos, int vidx, int pidx, const VectorXs &n, double time, double dt)
{
  VectorXs nhat = n;
  nhat.normalize();
  double cfactor = (1.0+getCOR())/2.0;
  VectorXs dx1 = (scene.getX()-oldpos)/dt;
  scene.getV().segment<2>(2*vidx) -= 2*cfactor*dx1.segment<2>(2*vidx).dot(nhat)*nhat;
  scene.getX().segment<2>(2*vidx) -= dt*2*cfactor*dx1.segment<2>(2*vidx).dot(nhat)*nhat;
}






// BEGIN STUDENT CODE //


// Given the start position (oldpos) and end position (scene.getX) of two
// particles, and assuming the particles moved in a straight line between the
// two positions, determines whether the two particles were overlapping and
// approaching at any point during that motion.
// If so, returns true, sets n to the the vector between the two particles
// at the time of collision, and sets t to the time (0 = start position, 
// 1 = end position) of collision.
// Inputs:
//   scene:  The scene data structure. The new positions and radii of the 
//           particles can be obtained from here.
//   oldpos: The old positions of the particles.
//   idx1:   The index of the first particle. (Ie, the degrees of freedom
//           corresponding to this particle are entries 2*idx1 and 2*idx1+1 in
//           scene.getX().
//   idx2:   The index of the second particle.
// Outputs:
//   n:    The vector between the two particles at the time of collision.
//   time: The time (scaled to [0,1]) when the two particles collide.
//   Returns true if the two particles overlap and are approaching at some point
//   during the motion.
bool ContinuousTimeCollisionHandler::detectParticleParticle(const TwoDScene &scene, const VectorXs &oldpos, int idx1, int idx2, VectorXs &n, double &time)
{
  VectorXs dx = scene.getX()-oldpos;

  VectorXs x1 = oldpos.segment<2>(2*idx1);
  VectorXs x2 = oldpos.segment<2>(2*idx2);

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
// If so, returns true, sets n to the the vector between the particle and the
// edge at the time of collision, and sets t to the time (0 = start position, 
// 1 = end position) of collision.
// Inputs:
//   scene:  The scene data structure. 
//   oldpos: The old positions of the particle and edge.
//   vidx:   The index of the particle.
//   eidx:   The index of the edge.
// Outputs:
//   n:    The shortest vector between the particle and edge at the time of 
//         collision.
//   time: The time (scaled to [0,1]) when the two objects collide.
//   Returns true if the particle and edge overlap and are approaching at
//   some point during the motion.
bool ContinuousTimeCollisionHandler::detectParticleEdge(const TwoDScene &scene, const VectorXs &oldpos, int vidx, int eidx, VectorXs &n, double &time)
{
  VectorXs dx = scene.getX() - oldpos;
  
  VectorXs x1 = oldpos.segment<2>(2*vidx);
  VectorXs x2 = oldpos.segment<2>(2*scene.getEdge(eidx).first);
  VectorXs x3 = oldpos.segment<2>(2*scene.getEdge(eidx).second);

  VectorXs dx1 = dx.segment<2>(2*vidx);
  VectorXs dx2 = dx.segment<2>(2*scene.getEdge(eidx).first);
  VectorXs dx3 = dx.segment<2>(2*scene.getEdge(eidx).second);

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

  double r1 = scene.getRadius(vidx);
  double r2 = scene.getEdgeRadii()[eidx];

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


  std::vector<double> velpoly;
  
  double alpha = (x1-x2).dot(x3-x2)/(x3-x2).dot(x3-x2);
  velpoly.push_back( (alpha * (dx3-dx2) + (dx2-dx1) ).dot( -alpha * (dx3-dx2)+dx1-dx2) );
  velpoly.push_back( (alpha * (x3-x2) + (x2-x1) ).dot( alpha * (dx3-dx2) + dx1-dx2) );

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
      return true;
    }
  return false;
  

}

// Given start positions (oldpos) and end positions (scene.getX) of a
// particle and a half-plane, and assuming the particle endpoints moved in 
// a straight line between the two positions, determines whether the two 
// objects were overlapping and approaching at any point during that motion.
// If so, returns true, sets n to the the vector between the particle and the
// half-plane at the time of collision, and sets t to the time (0 = start 
// position, 1 = end position) of collision.
// Inputs:
//   scene:  The scene data structure. 
//   oldpos: The old positions of the particle.
//   vidx:   The index of the particle.
//   eidx:   The index of the half-plane. The vectors (px, py) and (nx, ny) can
//           be retrieved by calling scene.getHalfplane(pidx).
// Outputs:
//   n:    The shortest vector between the particle and half-plane at the time 
//         of collision.
//   time: The time (scaled to [0,1]) when the two objects collide.
//   Returns true if the particle and half-plane overlap and are approaching at
//   some point during the motion.
bool ContinuousTimeCollisionHandler::detectParticleHalfplane(const TwoDScene &scene, const VectorXs &oldpos, int vidx, int pidx, VectorXs &n, double &time)
{
  VectorXs dx = scene.getX() - oldpos;

  VectorXs x1 = oldpos.segment<2>(2*vidx);
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
