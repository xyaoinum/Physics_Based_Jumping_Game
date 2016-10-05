#include "CollisionHandler.h"

class SimpleCollisionHandler : public CollisionHandler
{
public:
  SimpleCollisionHandler(double COR) : CollisionHandler(COR) {}

  virtual void handleCollisions(TwoDScene &scene, CollisionDetector &detector, const VectorXs &oldpos, VectorXs &oldvel, scalar dt );

  virtual std::string getName() const;

 private:

  bool detectParticleParticle(TwoDScene &scene, int idx1, int idx2, VectorXs &n);
  bool detectParticleEdge(TwoDScene &scene, int vidx, int eidx, VectorXs &n);

  bool detectParticleHalfplane(TwoDScene &scene, int vidx, int pidxs, VectorXs &n);


  void respondParticleParticle(TwoDScene &scene, int idx1, int idx2, const VectorXs &n);

  void respondParticleEdge(TwoDScene &scene, int vidx, int eidx, const VectorXs &n);

  void respondParticleHalfplane(TwoDScene &scene, int vidx, int pidx, const VectorXs &n);

};
