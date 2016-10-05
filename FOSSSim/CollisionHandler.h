#ifndef __COLLISION_HANDLER__
#define __COLLISION_HANDLER__

#include "TwoDScene.h"
#include "MathDefs.h"
#include <fstream>
#include <vector>

class CollisionDetector;

struct CollisionInfo
{
 public:
  enum collisiontype {PE, PP, PH};

CollisionInfo(collisiontype type, int idx1, int idx2, const VectorXs &n, double time) : m_type(type), m_idx1(idx1), m_idx2(idx2), m_n(n) {}
  
  collisiontype m_type;
  int m_idx1, m_idx2;
  double m_time;
  VectorXs m_n;
};

class CollisionHandler
{
public:
 CollisionHandler(double COR) : m_COR(COR) {}

  virtual ~CollisionHandler() {}
  
  virtual void handleCollisions( TwoDScene& scene, CollisionDetector &detector, const VectorXs &oldpos, VectorXs &oldvel, scalar dt ) = 0;
  
  virtual std::string getName() const = 0;

  // Returns the coefficient of restitution set in the scene XML
  double getCOR() {return m_COR;}

  void clearImpulses() {m_impulses.clear();}

  const std::vector<CollisionInfo> &getImpulses() const {return m_impulses;}

  void serializeImpulses(std::ofstream &ofs);

  void loadImpulses(std::vector<CollisionInfo> &impulses, std::ifstream &ifs);

 protected:
  void addParticleParticleImpulse(int idx1, int idx2, const VectorXs &n, double time);

  void addParticleEdgeImpulse(int vidx, int eidx, const VectorXs &n, double time);

  void addParticleHalfplaneImpulse(int vidx, int fidx, const VectorXs &n, double time);

 private:

  double m_COR;

  std::vector<CollisionInfo> m_impulses;

};

#endif
