#ifndef __RIGID_BODY_SIMULATION_H__
#define __RIGID_BODY_SIMULATION_H__

#include "FOSSSim/ExecutableSimulation.h"

#include "RigidBodyScene.h"
#include "RigidBodyExplicitEuler.h"
#include "OpenGLRenderer.h"
#include "FOSSSim/TwoDSceneRenderer.h"
#include "FOSSSim/TwoDSceneSVGRenderer.h"
#include "RigidBodySceneGrader.h"

#include "RigidBodyCollisionDetector.h"
#include "RigidBodyCollisionResolver.h"

class RigidBodySimulation : public ExecutableSimulation
{
public:

  RigidBodySimulation( RigidBodyScene* scene, RigidBodyScene* comparison_scene, RigidBodyStepper* stepper, TwoDSceneRenderer* scene_renderer, std::vector<OpenGLRenderer*> oglrenderers, TwoDSceneSVGRenderer* svg_renderer, RigidBodyCollisionDetector* collision_detector, RigidBodyCollisionResolver* collision_resolver );

  virtual ~RigidBodySimulation();
  
  /////////////////////////////////////////////////////////////////////////////
  // Simulation Control Functions

  virtual void beforeSimulationStarts() { }
  
  virtual void stepSystem( const scalar& dt );

  /////////////////////////////////////////////////////////////////////////////
  // Scene Benchmarking Functions
  
  virtual void updateSceneComparison();

  virtual void printErrorInformation( bool print_pass );

  /////////////////////////////////////////////////////////////////////////////
  // Rendering Functions

  // TODO: Scrap these two functions
  virtual void initializeOpenGLRenderer();
  virtual void renderSceneDifferencesOpenGL();
  

  virtual void renderSceneOpenGL();

  virtual void updateOpenGLRendererState();

  virtual void computeCameraCenter( renderingutils::Viewport& view );
  
  /////////////////////////////////////////////////////////////////////////////
  // SVG Rendering Functions

  virtual void renderSceneSVG( const std::string& name );

  virtual void updateSVGRendererState();

  /////////////////////////////////////////////////////////////////////////////
  // Serialization Functions
  
  virtual void copyComparisonSceneToScene();

  virtual void serializeScene( std::ofstream& outputstream );

  // Serializes 'one-time' computations at the start of the simulation.
  //  For this simulation, mass, center of mass, and moment of inertia in body space.
  virtual void serializeStartOfSimState( std::ofstream& ofs );
  // Deserializes and verifies 'one-time' computations at the start of the simulation.
  //  For this simulation, mass, center of mass, and moment of inertia in body space.
  virtual void deserializeAndVerifyStartOfSimState( std::ifstream& ifs );  
  
  // Serializes desired data at each time step (e.g. momentum). This function is called immediatly after serialize scene.
  //  For this simulation, momentum, 'linear' angular momentum, 'spin' angular momentum, 'linear' kinetic energy,
  //  'spin' kinetic energy', and potential energy.
  virtual void serializePerStepQuantities( std::ofstream& ofs );
  // Deserializes and verifies desired data at each time step.
  //  For this simulation, momentum, 'linear' angular momentum, 'spin' angular momentum, 'linear' kinetic energy,
  //  'spin' kinetic energy', and potential energy.
  virtual void deserializeAndVerifyPerStepQuantities( std::ifstream& ifs );  
  
  virtual void serializeHybridData(std::ofstream &ofs);
  virtual void loadHybridData(std::ifstream &ifs);

  virtual void loadComparisonScene( std::ifstream& inputstream );

  /////////////////////////////////////////////////////////////////////////////
  // Status Functions
  
  virtual std::string getSolverName();

  virtual std::string getCollisionHandlerName();
  
  virtual void outputCallback( std::ostream& strm );
  
	RigidBodyScene * getScene(void) const { return m_scene; }

protected:

  RigidBodyScene* m_scene;
  RigidBodyScene* m_comparison_scene;
  RigidBodyStepper* m_integrator;
  TwoDSceneRenderer* m_scene_renderer;
  TwoDSceneSVGRenderer* m_svg_renderer;
  std::vector<OpenGLRenderer*> m_opengl_renderers;
  RigidBodySceneGrader* m_grader;

  RigidBodyCollisionDetector* m_collision_detector;
  RigidBodyCollisionResolver* m_collision_resolver;
};

#endif
