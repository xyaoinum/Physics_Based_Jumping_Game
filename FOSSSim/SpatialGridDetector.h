#ifndef SPATIAL_GRID_DETECTOR_H
#define SPATIAL_GRID_DETECTOR_H

#include "CollisionDetector.h"

class SpatialGridDetector : public CollisionDetector
{
 public:
  virtual void performCollisionDetection(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, DetectionCallback &dc)
};


#endif
