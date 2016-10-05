#ifndef CONTINUOUS_TIME_COLLISION_HANDLER_H
#define CONTINUOUS_TIME_COLLISION_HANDLER_H

#include "CollisionHandler.h"
#include <vector>
#include <iostream>


class ContinuousTimeCollisionHandler : public CollisionHandler
{
public:
  ContinuousTimeCollisionHandler(double COR) : CollisionHandler(COR) {}

  virtual void handleCollisions(TwoDScene &scene, CollisionDetector &detector, const VectorXs &oldpos, VectorXs &oldvel, scalar dt );

  virtual std::string getName() const;

 private:

  bool detectParticleParticle(const TwoDScene &scene, const VectorXs &oldpos, int idx1, int idx2, VectorXs &n, double &time);

  bool detectParticleEdge(const TwoDScene &scene, const VectorXs &oldpos, int vidx, int eidx, VectorXs &n, double &time);

  bool detectParticleHalfplane(const TwoDScene &scene, const VectorXs &oldpos, int vidx, int pidxs, VectorXs &n, double &time);


  void respondParticleParticle(TwoDScene &scene, const VectorXs &oldpos, int idx1, int idx2, const VectorXs &n, double time, double dt);

  void respondParticleEdge(TwoDScene &scene, const VectorXs &oldpos, int vidx, int eidx, const VectorXs &n, double time, double dt);

  void respondParticleHalfplane(TwoDScene &scene, const VectorXs &oldpos, int vidx, int pidx, const VectorXs &n, double time, double dt);

};

#endif
