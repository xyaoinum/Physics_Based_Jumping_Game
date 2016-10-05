#include "ContestDetector.h"
#include <iostream>
#include "TwoDScene.h"
#include <set>

// For all pairs of potentially-overlapping objects, applies the penalty force.
//
// Does not need to be modified by students.
void ContestDetector::performCollisionDetection(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, DetectionCallback &dc)
{
  if( (qs-qe).norm() > 1e-8)
    {
      std::cerr << "Contest collision detector is only designed for use with the penalty method!" << std::endl;
      exit(1);
    }

  PPList pppairs;
  PEList pepairs;
  PHList phpairs;

  findCollidingPairs(scene, qe, pppairs, pepairs, phpairs);
  for(PPList::iterator it = pppairs.begin(); it != pppairs.end(); ++it)
    dc.ParticleParticleCallback(it->first, it->second);
  for(PEList::iterator it = pepairs.begin(); it != pepairs.end(); ++it)
    {
      //particle never collides with an edge it's an endpoint of
      if(scene.getEdge(it->second).first != it->first && scene.getEdge(it->second).second != it->first)
	dc.ParticleEdgeCallback(it->first, it->second);
    }
  for(PHList::iterator it = phpairs.begin(); it != phpairs.end(); ++it)
    dc.ParticleHalfplaneCallback(it->first, it->second);
}

int hash(double min, double max, double val, int numcells)
{
  int res = (val-min)/(max-min)*numcells;
  return std::max(std::min(numcells-1, res), 0);
}


// Given particle positions, computes lists of *potentially* overlapping object
// pairs. How exactly to do this is up to you.
// Inputs: 
//   scene:  The scene object. Get edge information, radii, etc. from here. If 
//           for some reason you'd also like to use particle velocities in your
//           algorithm, you can get them from here too.
//   x:      The positions of the particle.
// Outputs:
//   pppairs: A list of (particle index, particle index) pairs of potentially
//            overlapping particles. IMPORTANT: Each pair should only appear
//            in the list at most once. (1, 2) and (2, 1) count as the same 
//            pair.
//   pepairs: A list of (particle index, edge index) pairs of potential
//            particle-edge overlaps.
//   phpairs: A list of (particle index, halfplane index) pairs of potential
//            particle-halfplane overlaps.
void ContestDetector::findCollidingPairs(const TwoDScene &scene, const VectorXs &x, PPList &pppairs, PEList &pepairs, PHList &phpairs)
{
  static int numcells = 0;
  if(numcells == 0)
    numcells = sqrt(scene.getNumParticles());
  double minx = x[0];
  double maxx = x[0];
  double miny = x[1];
  double maxy = x[1];
  for(int i=0; i<scene.getNumParticles(); i++)
    {
      if(x[2*i] > maxx)
	maxx = x[2*i];
      if(x[2*i] < minx)
	minx = x[2*i];

      if(x[2*i+1] > maxy)
	maxy = x[2*i+1];
      if(x[2*i+1] < miny)
	miny = x[2*i+1];

      for(int j=0; j<scene.getNumHalfplanes(); j++)
	{
	  phpairs.insert(std::pair<int, int>(i,j));
	}
    }

  struct Cell
  {
    std::set<int> verts;
    std::set<int> edges;
  };

  static Cell *hashgrid = NULL;
  if(!hashgrid)
    hashgrid = new Cell[numcells*numcells];

  for(int i=0; i<numcells*numcells; i++)
    {
      hashgrid[i].verts.clear();
      hashgrid[i].edges.clear();
    }
  

  for(int i=0; i<scene.getNumParticles(); i++)
    {
      double r = scene.getRadius(i);
      int px1 = hash(minx, maxx, x[2*i]-r, numcells);
      int px2 = hash(minx, maxx, x[2*i]+r, numcells);

      int py1 = hash(miny, maxy, x[2*i+1]-r, numcells);
      int py2 = hash(miny, maxy, x[2*i+1]+r, numcells);

      for(int a = px1; a <= px2; a++)
	{
	  for(int b = py1; b <= py2; b++)
	    {
	      hashgrid[numcells*a + b].verts.insert(i);
	    }
	}

    }

  for(int i=0; i<scene.getNumEdges(); i++)
    {
      double r = scene.getEdgeRadii()[i];
      int e1 = scene.getEdge(i).first;
      int e2 = scene.getEdge(i).second;
      
      int px1 = std::min(hash(minx, maxx, x[2*e1]-r, numcells),
			 hash(minx, maxx, x[2*e2]-r, numcells));
      int px2 = std::max(hash(minx, maxx, x[2*e1]+r, numcells),
			 hash(minx, maxx, x[2*e2]+r, numcells));

      int py1 = std::min(hash(miny, maxy, x[2*e1+1]-r, numcells),
			 hash(miny, maxy, x[2*e2+1]-r, numcells));
      int py2 = std::max(hash(miny, maxy, x[2*e1+1]+r, numcells),
			 hash(miny, maxy, x[2*e2+1]+r, numcells));

      for(int a = px1; a <= px2; a++)
	{
	  for(int b = py1; b <= py2; b++)
	    {
	      hashgrid[numcells*a+b].edges.insert(i);
	    }
	}
    }

  for(int i=0; i<numcells*numcells; i++)
    {
      for(std::set<int>::iterator it = hashgrid[i].verts.begin(); it != hashgrid[i].verts.end(); ++it)
	{
	  std::set<int>::iterator it2 = it;
	  for(++it2; it2 != hashgrid[i].verts.end(); ++it2)
	    {
	      pppairs.insert(std::pair<int, int>(*it, *it2));
	    }
	  for(std::set<int>::iterator it2 = hashgrid[i].edges.begin(); it2 != hashgrid[i].edges.end(); ++it2)
	    {
	      pepairs.insert(std::pair<int, int>(*it, *it2));
	    }
	}
    }
      
}
