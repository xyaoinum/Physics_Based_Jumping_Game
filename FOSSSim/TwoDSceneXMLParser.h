#ifndef __TWO_D_SCENE_XML_PARSER_H__
#define __TWO_D_SCENE_XML_PARSER_H__

#include <Eigen/StdVector>

#include <iostream>
#include <fstream>
#include <limits>

#include "rapidxml.hpp"

#include "TwoDScene.h"

#include "ExplicitEuler.h"
#include "SemiImplicitEuler.h"
#include "ImplicitEuler.h"
#include "LinearizedImplicitEuler.h"

#include "SpringForce.h"
#include "GravitationalForce.h"
#include "ConstantForce.h"
#include "SimpleGravityForce.h"
#include "DragDampingForce.h"
#include "VortexForce.h"

#include "StringUtilities.h"
#include "RenderingUtilities.h"
#include "CollisionHandler.h"

#include "SimpleCollisionHandler.h"
#include "ContinuousTimeCollisionHandler.h"
#include "PenaltyForce.h"

#include "TwoDSceneRenderer.h"
#include "TwoDSceneSVGRenderer.h"
#include "ExecutableSimulation.h"
#include "TwoDimensionalDisplayController.h"
#include "ParticleSimulation.h"

#include "RigidBodies/RigidBodySimulation.h"
#include "RigidBodies/RigidBodyExplicitEuler.h"
#include "RigidBodies/RigidBodySymplecticEuler.h"
#include "RigidBodies/RigidBodyForce.h"
#include "RigidBodies/RigidBodySpringForce.h"
#include "RigidBodies/RigidBodyGravityForce.h"
#include "RigidBodies/RigidBodyWindForce.h"
#include "RigidBodies/OpenGLRenderer.h"
#include "RigidBodies/OpenGLRigidBodySpringForceRenderer.h"
#include "RigidBodies/OpenGLRigidBodyGroundRenderer.h"
#include "RigidBodies/SVGRenderer.h"
#include "RigidBodies/SVGRigidBodySpringForceRenderer.h"
#include "RigidBodies/RigidBodyAllPairsCollisionDetector.h"
#include "RigidBodies/RigidBodyGroundCollisionDetector.h"
#include "RigidBodies/RigidBodyLCPCollisionResolver.h"
#include "RigidBodies/RigidBodyVelocityProjectionCollisionResolver.h"

#include "JointMotorBodies/JointMotorBodySimulation.h"
#include "JointMotorBodies/JointMotorBodyScene.h"
#include "JointMotorBodies/JointMotorBody.h"
#include "JointMotorBodies/JointMotorBodyController.h"
#include "JointMotorBodies/MonopodRobot.h"
#include "JointMotorBodies/MonopodRobotController.h"
#include "JointMotorBodies/JointMotorBodySimulationJudge.h"
#include "JointMotorBodies/OpenGLRigidBodyScheduledImpulseRenderer.h"

// REALLY USEFULL TODOs
//   TODO: Improve error messages to display all valid options, etc. Could define an option class that knows its valid options and bounds on values.

// LESS USEFULL TODOs
//   TODO: Write method for computing number of a given property
//   TODO: Add some additional error checking for repeat properties, etc
//   TODO: Abstract out common code
//   TODO: Check for invalid properties

class TwoDSceneXMLParser
{
public:
  
  void loadExecutableSimulation( const std::string& file_name, bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, renderingutils::Viewport& view, scalar& dt, scalar& max_time, scalar& steps_per_sec_cap, renderingutils::Color& bgcolor, std::string& description, std::string& scenetag );
  
private:
  
  void loadParticleSimulation( bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, 
                              renderingutils::Viewport& view, scalar& dt, renderingutils::Color& bgcolor, rapidxml::xml_node<>* node );
  
  void loadRigidBodySimulation( bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, renderingutils::Viewport& view, scalar& dt, renderingutils::Color& bgcolor, rapidxml::xml_node<>* node );
  
  void loadJointMotorBodySimulation( bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, renderingutils::Viewport& view, scalar& dt, renderingutils::Color& bgcolor, rapidxml::xml_node<>* node );
  
  void loadXMLFile( const std::string& filename, std::vector<char>& xmlchars, rapidxml::xml_document<>& doc );
  
  bool loadTextFileIntoString( const std::string& filename, std::string& filecontents );
  
  void loadSimulationType( rapidxml::xml_node<>* node, std::string& simtype );
  
  /////////////////////////////////////////////////////////////////////////////
  // Camera parsing
  
  void loadViewport( rapidxml::xml_node<> *node, renderingutils::Viewport &view);
  
  /////////////////////////////////////////////////////////////////////////////
  // Rigid body parsing
  
  void loadRigidBodyVertices( rapidxml::xml_node<>* node, std::vector<Vector2s>& vertices, std::vector<scalar>& masses );
  
  void loadRigidBodies( rapidxml::xml_node<>* node, const std::vector<Vector2s>& vertices, const std::vector<scalar>& masses, std::vector<RigidBody>& rigidbodies );
  
  void loadRigidBodyIntegrator( rapidxml::xml_node<>* node, RigidBodyStepper** stepper, scalar& dt );
  
  void loadRigidBodyCollisionHandling( rapidxml::xml_node<>* node, RigidBodyCollisionDetector** collision_detector, RigidBodyCollisionResolver** collision_resolver );
  
  void loadRigidBodySpringForces( rapidxml::xml_node<>* node, std::vector<RigidBodySpringForce*>& rbforces );
  
  void loadRigidBodyGravityForces( rapidxml::xml_node<>* node, std::vector<RigidBodyGravityForce*>& rbforces );
  
  void loadRigidBodyWindForces( rapidxml::xml_node<>* node, std::vector<RigidBodyWindForce*>& rbforces );
  
  void loadRigidBodyColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& rbcolors );  
  
  void loadRigidBodySpringColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& colors );
  
  /////////////////////////////////////////////////////////////////////////////
  // Joint motor body parsing
  
  void loadJointMotorBodyCollisionHandling( rapidxml::xml_node<>* node, RigidBodyCollisionDetector** collision_detector );
  
  void loadJointMotorBodies( JointMotorBodyScene * scene, rapidxml::xml_node<>* node, std::vector<JointMotorBody *> & jmbs, const std::vector<RigidBody>& rbs, std::vector<RigidBodyForce *> & controller_forces);

  void loadJointMotorBodySimulationDetectors( rapidxml::xml_node<>* node, JointMotorBodyScene * scene, JointMotorBodySimulationJudge ** judge );
  
  void loadScheduledPushes( rapidxml::xml_node<> * node, JointMotorBodyScene * scene, std::vector<JointMotorBodySimulation::ScheduledPushEvent> & pushes );
  
  
  
  
  void loadParticles( rapidxml::xml_node<>* node, TwoDScene& twodscene );
  
  void loadSceneTag( rapidxml::xml_node<>* node, std::string& scenetag );
  
  void loadEdges( rapidxml::xml_node<>* node, TwoDScene& twodscene );
  
  void loadHalfplanes( rapidxml::xml_node<> *node, TwoDScene &twoscene);
  
  void loadSpringForces( rapidxml::xml_node<>* node, TwoDScene& twodscene );
  
  void loadGravitationalForces( rapidxml::xml_node<>* node, TwoDScene& twodscene );
  
  void loadSimpleGravityForces( rapidxml::xml_node<>* node, TwoDScene& twodscene );
  
  void loadConstantForces( rapidxml::xml_node<>* node, TwoDScene& twodscene );
  
  void loadDragDampingForces( rapidxml::xml_node<>* node, TwoDScene& twodscene );
  
  void loadVorexForces( rapidxml::xml_node<>* node, TwoDScene& twodscene );
  
  void loadIntegrator( rapidxml::xml_node<>* node, SceneStepper** scenestepper, scalar& dt );
  
  void loadCollisionHandler( rapidxml::xml_node<>* node, TwoDScene &scene, CollisionHandler **handler, CollisionDetector &detector);
  
  void loadCollisionDetector( rapidxml::xml_node<> *node, TwoDScene &scene, CollisionDetector **handler);
  
  void loadMaxTime( rapidxml::xml_node<>* node, scalar& max_t );
  
  void loadMaxSimFrequency( rapidxml::xml_node<>* node, scalar& max_freq );
  
  void loadBackgroundColor( rapidxml::xml_node<>* node, renderingutils::Color& color );
  
  void loadParticleColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& particle_colors );
  
  void loadEdgeColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& edge_colors );
  
  void loadHalfplaneColors( rapidxml::xml_node<> *node, std::vector<renderingutils::Color> &halfplane_colors );
  
  void loadParticlePaths( rapidxml::xml_node<>* node, double dt, std::vector<renderingutils::ParticlePath>& particle_paths );
  
  void loadSceneDescriptionString( rapidxml::xml_node<>* node, std::string& description_string );
};

#endif
