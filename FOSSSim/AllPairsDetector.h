#ifndef ALL_PAIRS_DETECTOR
#define ALL_PAIRS_DETECTOR

#include "CollisionDetector.h"

class AllPairsDetector : public CollisionDetector
{
 public:
  virtual void performCollisionDetection(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, DetectionCallback &dc);
};

#endif
