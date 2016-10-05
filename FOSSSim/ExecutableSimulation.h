#ifndef __EXECUTABLE_SIMULATION_H__
#define __EXECUTABLE_SIMULATION_H__

#include <string>
#include <Eigen/Core>
#include "MathDefs.h"
#include "RenderingUtilities.h"

class ExecutableSimulation
{
public:

  virtual ~ExecutableSimulation()
  {}

  /////////////////////////////////////////////////////////////////////////////
  // Simulation Control Functions
  
  virtual void beforeSimulationStarts() { }
  virtual void stepSystem( const scalar& dt ) = 0;
  
  /////////////////////////////////////////////////////////////////////////////
  // UI Interaction
  
  virtual void mouse(int button, int state, int x, int y) { }
  virtual void motion(int x, int y) { }
  virtual void passiveMotion(int x, int y) { }
  virtual void keyboard(unsigned char key, int x, int y) { }
  
  virtual void getFeatures(std::vector<Matrix2s> & features) { }
  
  /////////////////////////////////////////////////////////////////////////////
  // Scene Benchmarking Functions

  virtual void updateSceneComparison() = 0;
  virtual void printErrorInformation( bool print_pass ) = 0;
  
  /////////////////////////////////////////////////////////////////////////////
  // OpenGL Rendering Functions
  
  // TODO: Combine both of these
  virtual void renderSceneOpenGL() = 0;
  
  // TODO: Scrap these two functions, they are no longer needed
  virtual void renderSceneDifferencesOpenGL() = 0;
  virtual void initializeOpenGLRenderer() = 0;
  
  virtual void updateOpenGLRendererState() = 0;

  virtual void computeCameraCenter( renderingutils::Viewport& view ) = 0;
  
  /////////////////////////////////////////////////////////////////////////////
  // SVG Rendering Functions

  virtual void renderSceneSVG( const std::string& name ) = 0;

  virtual void updateSVGRendererState() = 0;

  /////////////////////////////////////////////////////////////////////////////
  // Serialization Functions

  virtual void copyComparisonSceneToScene() = 0;
  virtual void serializeScene( std::ofstream& outputstream ) = 0;

  // Serializes 'one-time' computations at the start of the simulation (e.g. moment of inertia in body-space)
  virtual void serializeStartOfSimState( std::ofstream& ofs );
  // Deserializes and verifies 'one-time' computations at the start of the simulation (e.g. moment of inertia in body-space)
  virtual void deserializeAndVerifyStartOfSimState( std::ifstream& ifs );

  // Serializes desired data at each time step (e.g. momentum). This function is called immediatly after serialize scene.
  virtual void serializePerStepQuantities( std::ofstream& ofs );
  // Deserializes and verifies desired data at each time step.
  virtual void deserializeAndVerifyPerStepQuantities( std::ifstream& ifs );


  virtual void serializeHybridData(std::ofstream &ofs) =0 ;
  virtual void loadComparisonScene( std::ifstream& inputstream ) = 0;
  virtual void loadHybridData( std::ifstream &ifs ) = 0;

  /////////////////////////////////////////////////////////////////////////////
  // Status Functions

  virtual std::string getSolverName() = 0;
  virtual std::string getCollisionHandlerName() = 0;
  
  virtual void outputCallback( std::ostream& strm ) = 0;
};

#endif
