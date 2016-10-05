#ifndef HYBRID_COLLISION_HANDLER_H
#define HYBRID_COLLISION_HANDLER_H

#include "CollisionHandler.h"
#include <vector>
#include <set>
#include <list>

class HybridCollisionComparison;

struct ImpactZone
{
ImpactZone(std::set<int> vertices, bool halfplane) : m_verts(vertices), m_halfplane(halfplane) {}
  std::set<int> m_verts;
  bool m_halfplane;

  bool operator==(const ImpactZone &other) const
  {
    return m_verts == other.m_verts && m_halfplane == other.m_halfplane;
  }

  bool operator<(const ImpactZone &other) const
  {
    if(!m_halfplane && other.m_halfplane)
      return true;
    if(m_halfplane && !other.m_halfplane)
      return false;
    if(m_verts.size() == 0)
      {
	if(other.m_verts.size() > 0)
	  return true;
	return false;
      }
    if(other.m_verts.size() == 0)
      return false;
    return *m_verts.begin() < *other.m_verts.begin();
  }
};

typedef std::vector<ImpactZone> ImpactZones;

class HybridCollisionHandler : public CollisionHandler
{
public:
  HybridCollisionHandler(int maxiters, double COR);
  ~HybridCollisionHandler();

  virtual void handleCollisions(TwoDScene &scene, CollisionDetector &detector, const VectorXs &oldpos, VectorXs &oldvel, scalar dt );

  bool applyIterativeImpulses(const TwoDScene &scene, CollisionDetector &detector, const VectorXs &qs, const VectorXs &qe, const VectorXs &qdote, double dt, VectorXs &qefinal, VectorXs &qdotefinal);

  std::vector<CollisionInfo> detectCollisions(const TwoDScene &scene, CollisionDetector &detector, const VectorXs &qs, const VectorXs &qe);

  void applyImpulses(const TwoDScene &scene, const std::vector<CollisionInfo> &collisions, const VectorXs &qs, const VectorXs &qe, const VectorXs &qdote, double dt, VectorXs &qm, VectorXs &qdotm);

  void applyGeometricCollisionHandling(const TwoDScene &scene, CollisionDetector &detector, const VectorXs &qs, const VectorXs &qe, const VectorXs &qdote, double dt, VectorXs &qefinal, VectorXs &qdotefinal);

  void performFailsafe(const TwoDScene &scene, const VectorXs &oldpos, const ImpactZone &zone, double dt, VectorXs &qe, VectorXs &qdote);

  virtual std::string getName() const;

  void loadHybridData(std::ifstream &ifs);
  void serializeHybridData(std::ofstream &ofs);

  bool failed();

 private:

  const int m_maxiters;

  bool detectParticleParticle(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int idx1, int idx2, VectorXs &n, double &time);

  bool detectParticleEdge(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int vidx, int eidx, VectorXs &n, double &time);

  bool detectParticleHalfplane(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int vidx, int pidxs, VectorXs &n, double &time);


  void respondParticleParticle(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int idx1, int idx2, const VectorXs &n, double time, double dt, VectorXs &qm, VectorXs &qdotm);

  void respondParticleEdge(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int vidx, int eidx, const VectorXs &n, double time, double dt, VectorXs &qm, VectorXs &qdotm);

  void respondParticleHalfplane(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, int vidx, int pidx, const VectorXs &n, double time, double dt, VectorXs &qm, VectorXs &qdotm);

  HybridCollisionComparison *m_in, *m_out;

};

#endif
