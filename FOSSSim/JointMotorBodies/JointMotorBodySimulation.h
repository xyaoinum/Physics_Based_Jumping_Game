#ifndef __JOINT_MOTOR_BODY_SIMULATION_H__
#define __JOINT_MOTOR_BODY_SIMULATION_H__

#include "FOSSSim/ExecutableSimulation.h"

#include "JointMotorBodyScene.h"
#include "../RigidBodies/OpenGLRenderer.h"
#include "FOSSSim/TwoDSceneRenderer.h"
#include "FOSSSim/TwoDSceneSVGRenderer.h"
#include "../RigidBodies/RigidBodySceneGrader.h"
#include "../RigidBodies/RigidBodyStepper.h"
#include "../RigidBodies/RigidBodyCollisionDetector.h"
#include "JointMotorBodySimulationJudge.h"

class JointMotorBodySimulation : public ExecutableSimulation
{
public:
  class ScheduledPushEvent
  {
  public:
    Vector2s impulse;
    int rb;
    scalar time;
    scalar alpha;
  };
  
public:

  JointMotorBodySimulation( JointMotorBodyScene* scene, JointMotorBodyScene* comparison_scene, RigidBodyStepper* stepper, TwoDSceneRenderer* scene_renderer, std::vector<OpenGLRenderer*> oglrenderers, RigidBodyCollisionDetector* collision_detector, JointMotorBodySimulationJudge * judge, std::vector<ScheduledPushEvent> * pushes );

  virtual ~JointMotorBodySimulation();
  
  
  JointMotorBodyScene* get_scene(){
      return m_scene;
  }
  
  
  
  /////////////////////////////////////////////////////////////////////////////
  // Simulation Control Functions

  virtual void beforeSimulationStarts();
  
  virtual void stepSystem( const scalar& dt );
  
  /////////////////////////////////////////////////////////////////////////////
  // UI Interaction
  
  virtual void keyboard(unsigned char key, int x, int y);
  virtual void mouse(int button, int state, int x, int y);
  virtual void motion(int x, int y);
  virtual void passiveMotion(int x, int y);

  virtual void getFeatures(std::vector<Matrix2s> & features);
  
  /////////////////////////////////////////////////////////////////////////////
  // Scene Benchmarking Functions
  
  virtual void updateSceneComparison();

  virtual void printErrorInformation( bool print_pass );

  virtual void printDetectorReport();
  
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
  
  virtual void loadComparisonScene( std::ifstream& inputstream );

  virtual void serializeHybridData(std::ofstream &ofs);
  virtual void loadHybridData(std::ifstream &ifs);
  
  /////////////////////////////////////////////////////////////////////////////
  // Status Functions
  
  virtual std::string getSolverName();

  virtual std::string getCollisionHandlerName();
  
  virtual void outputCallback( std::ostream& strm );
  
//	JointMotorBodyScene * getScene(void) const { return m_scene; }

   JointMotorBodySimulationJudge * judge()  { return m_judge; }
  
protected:

  JointMotorBodyScene* m_scene;
//  JointMotorBodyScene* m_comparison_scene;
  RigidBodyStepper* m_integrator;
  TwoDSceneRenderer* m_scene_renderer;
  std::vector<OpenGLRenderer*> m_opengl_renderers;
//  RigidBodySceneGrader* m_grader;

  RigidBodyCollisionDetector* m_collision_detector;
  
  JointMotorBodySimulationJudge * m_judge;
  
  std::vector<ScheduledPushEvent> * m_pushes;
  
};

#endif
