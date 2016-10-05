#include "AllPairsDetector.h"
#include "TwoDScene.h"

void AllPairsDetector::performCollisionDetection(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, DetectionCallback &dc)
{
  for(int i=0; i<(int)scene.getNumParticles(); i++)
  {
    for(int j=i+1; j<(int)scene.getNumParticles(); j++)
    {
      dc.ParticleParticleCallback(i,j);
    }

    for(int e=0; e<(int)scene.getNumEdges(); e++)
    {
      if(scene.getEdge(e).first != i && scene.getEdge(e).second != i)
        dc.ParticleEdgeCallback(i,e);
    }
      
    for(int h=0; h<(int)scene.getNumHalfplanes(); h++)
    {
      dc.ParticleHalfplaneCallback(i,h);
    }
  }
}
