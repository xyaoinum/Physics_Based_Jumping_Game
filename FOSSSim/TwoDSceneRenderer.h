#ifndef __TWO_D_SCENE_RENDERER_H__
#define __TWO_D_SCENE_RENDERER_H__

#include <Eigen/StdVector>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <iostream>

#include "TwoDScene.h"
#include "MathUtilities.h"
#include "RenderingUtilities.h"
#include "CollisionHandler.h"

#include "RigidBodies/RigidBodyScene.h"

// TODO: Get display controller out of here
// TODO: Make particle system and rigid body renderers that inherit from this

class TwoDimensionalDisplayController;

class TwoDSceneRenderer
{
public:
  
  // TODO: Gut this method
  TwoDSceneRenderer( const TwoDScene& scene, const TwoDimensionalDisplayController& dc, 
                     const std::vector<renderingutils::Color>& particle_colors, const std::vector<renderingutils::Color>& edge_colors, 
                     const std::vector<renderingutils::Color> &halfplane_colors, const std::vector<renderingutils::ParticlePath>& particle_paths );
  
  TwoDSceneRenderer( const TwoDimensionalDisplayController& dc );
  
  void renderRigdBodySimulation( const RigidBodyScene& scene );
  void circleRigidBody( const RigidBody& rb ) const;
  void circleMajorRigidBodySimulationResiduals( const RigidBodyScene& orclscn, const RigidBodyScene& testscn, const scalar& eps = 1.0e-8 ) const;

  void updateParticleSimulationState( const TwoDScene& scene );
  void renderParticleSimulation( const TwoDScene& scene ) const;
  void circleMajorParticleSimulationResiduals( const TwoDScene& oracle_scene, const TwoDScene& testing_scene, const std::vector<CollisionInfo> *impulses, const std::vector<CollisionInfo> * otherimpulses, scalar eps = 1.0e-9 ) const;
  
  // Returns a reference to the vector containing particle colors
  std::vector<renderingutils::Color>& getParticleColors();
  const std::vector<renderingutils::Color>& getParticleColors() const;
  
  // Returns a reference to the vector containing rigid body colors
  std::vector<renderingutils::Color>& getRigidBodyColors();
  const std::vector<renderingutils::Color>& getRigidBodyColors() const;  
  
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
private:
  void initializeCircleRenderer( int num_points );
  void initializeSemiCircleRenderer( int num_points );
  
  void renderSolidCircle( const Eigen::Vector2d& center, double radius ) const;
  void renderCircle( const Eigen::Vector2d& center, double radius ) const;

  void renderSweptEdge( const Eigen::Vector2d& x0, const Eigen::Vector2d& x1, double radius ) const;
 
  void renderImpulse( const TwoDScene &scene, const CollisionInfo &impulse, bool buggy) const;

  void renderHalfplane( const VectorXs &x, const VectorXs &n ) const;
  
  
  
  void renderRigdBody( const RigidBody& rb, const renderingutils::Color& color ) const;

  void renderTreatOfTrickOrTreat(const RigidBody & rb) const;
  
  const TwoDimensionalDisplayController& m_dc;
  
  // TODO: Move this out of here and into some subclass
  // Particle System rendering state
  std::vector<renderingutils::Color> m_particle_colors;
  std::vector<renderingutils::Color> m_edge_colors;
  std::vector<renderingutils::Color> m_halfplane_colors;
  std::vector<renderingutils::ParticlePath> m_particle_paths;
  
  // TODO: Same here, create a rigid body rendering class
  std::vector<renderingutils::Color> m_rb_colors;
  
  // Precomputed points for a circle
  std::vector<std::pair<double,double> > m_circle_points;
  std::vector<std::pair<double,double> > m_semi_circle_points;
};

#endif
