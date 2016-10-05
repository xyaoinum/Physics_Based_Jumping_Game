#ifndef __PARTICLE_SIMULATION_H__
#define __PARTICLE_SIMULATION_H__

#include "ExecutableSimulation.h"

#include "TwoDScene.h"
#include "TwoDSceneRenderer.h"
#include "TwoDSceneSVGRenderer.h"
#include "SceneStepper.h"
#include "TwoDSceneGrader.h"
#include "CollisionHandler.h"
#include "TwoDSceneSerializer.h"
#include "HybridCollisionHandler.h"
#include "CollisionDetector.h"

// TODO: Move code out of header!

class ParticleSimulation : public ExecutableSimulation
{
public:

 ParticleSimulation( TwoDScene* scene, TwoDScene* comparison_scene, CollisionDetector *collision_detector, CollisionHandler* collisions_handler, SceneStepper* scene_stepper, TwoDSceneRenderer* scene_renderer, TwoDSceneSVGRenderer* svg_renderer )
  : m_scene(scene)
  , m_comparison_scene(comparison_scene)
  , m_scene_stepper(scene_stepper)
  , m_collision_detector(collision_detector)
  , m_collision_handler(collisions_handler)
  , m_comparison_impulses()
  , m_scene_renderer(scene_renderer)
  , m_svg_renderer(svg_renderer)
  , m_scene_grader(NULL)
  {
    if( comparison_scene != NULL ) m_scene_grader = new TwoDSceneGrader;
  }

  virtual ~ParticleSimulation()
  {
    if( m_scene != NULL )
    {
      delete m_scene;
      m_scene = NULL;
    }

    if( m_comparison_scene != NULL )
    {
      delete m_comparison_scene;
      m_comparison_scene = NULL;
    }

    if( m_scene_stepper != NULL )
    {
      delete m_scene_stepper;
      m_scene_stepper = NULL;
    }

    if( m_collision_handler != NULL )
    {
      delete m_collision_handler;
      m_collision_handler = NULL;
    }

    if( m_scene_renderer != NULL )
    {
      delete m_scene_renderer;
      m_scene_renderer = NULL;
    }

    if( m_svg_renderer != NULL )
    {
      delete m_svg_renderer;
      m_svg_renderer = NULL;
    }

    if( m_scene_grader != NULL )
    {
      delete m_scene_grader;
      m_scene_grader = NULL;
    }
  }
  
  /////////////////////////////////////////////////////////////////////////////
  // Simulation Control Functions

  virtual void beforeSimulationStarts() { }
  
  virtual void stepSystem( const scalar& dt )
  {
    assert( m_scene != NULL );
    assert( m_scene_stepper != NULL );

    VectorXs oldpos = m_scene->getX();
    VectorXs oldvel = m_scene->getV();

    // Step the simulated scene forward
    m_scene_stepper->stepScene( *m_scene, dt );

    // Do collision response
    if( m_collision_handler != NULL )
    {
      m_collision_handler->clearImpulses();
      m_collision_handler->handleCollisions( *m_scene, *m_collision_detector,oldpos, oldvel, dt );
    }
 
    // Check for obvious problems in the simulated scene
    #ifdef DEBUG
      m_scene->checkConsistency();
    #endif
  }

  /////////////////////////////////////////////////////////////////////////////
  // Scene Benchmarking Functions
  
  virtual void updateSceneComparison()
  {
    assert( m_scene != NULL );
    assert( m_comparison_scene != NULL );
    assert( m_scene_grader != NULL );
    m_scene_grader->addToAccumulatedResidual( *m_scene, *m_comparison_scene );
    if(m_collision_handler && m_collision_handler->getName() == "Hybrid Collision Handling" && ((HybridCollisionHandler *)m_collision_handler)->failed())
    {
      m_scene_grader->setCollisionsFailed();
    }
  }

  virtual void printErrorInformation( bool print_pass )
  {
    assert( m_scene_grader != NULL );

    scalar accumulatedPositionResidual = m_scene_grader->getAccumulatedPositionResidual();
    scalar accumulatedVelocityResidual = m_scene_grader->getAccumulatedVelocityResidual();
    scalar maxPositionResidual = m_scene_grader->getMaxPositionResidual();
    scalar maxVelocityResidual = m_scene_grader->getMaxVelocityResidual();

    
    if( !print_pass )
    {
      std::cout << "Accumulated Position Residual: " << accumulatedPositionResidual << std::endl;
      std::cout << "Accumulated Velocity Residual: " << accumulatedVelocityResidual << std::endl;
      std::cout << "Maximum Position Residual: "     << maxPositionResidual         << std::endl;
      std::cout << "Maximum Velocity Residual: "     << maxVelocityResidual         << std::endl;

      std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink;
      std::cout << "Simulation did not run to completion. No pass/fail status will be reported." << std::endl;      
    }
    else
    {
      bool accumulatedPositionResidualAcceptable = m_scene_grader->accumulatedPositionResidualPassed();
      bool accumulatedVelocityResidualAcceptable = m_scene_grader->accumulatedVelocityResidualPassed();
      bool maxPositionResidualAcceptable = m_scene_grader->maxPositionResidualPassed();
      bool maxVelocityResidualAcceptable = m_scene_grader->maxVelocityResidualPassed();
      bool collisionsPassed = m_scene_grader->collisionsPassed();
      
      bool total_success = accumulatedPositionResidualAcceptable && accumulatedVelocityResidualAcceptable && maxPositionResidualAcceptable && maxVelocityResidualAcceptable && collisionsPassed;
      
      std::cout << "Accumulated Position Residual: " << accumulatedPositionResidual << "   " << (accumulatedPositionResidualAcceptable?"Passed.":"Failed.") << std::endl;
      std::cout << "Accumulated Velocity Residual: " << accumulatedVelocityResidual << "   " << (accumulatedVelocityResidualAcceptable?"Passed.":"Failed.") << std::endl;
      std::cout << "Maximum Position Residual: "     << maxPositionResidual << "   "         << (maxPositionResidualAcceptable?"Passed.":"Failed.") << std::endl;
      std::cout << "Maximum Velocity Residual: "     << maxVelocityResidual << "   "         << (maxVelocityResidualAcceptable?"Passed.":"Failed.") << std::endl;
      std::cout << "Hybrid Collisions Passed: " << (collisionsPassed ? "Yes." : "No.") << std::endl;

      std::cout << "Overall success: " << (total_success?"Passed.":"Failed.") << std::endl;

      // TODO: Add checks that residual.txt was actually written!
      std::ofstream residfile;
      residfile.open("residual.txt");
      residfile << accumulatedPositionResidualAcceptable << "   " << accumulatedPositionResidual << std::endl;
      residfile << accumulatedVelocityResidualAcceptable << "   " << accumulatedVelocityResidual << std::endl;
      residfile << maxPositionResidualAcceptable << "   " << maxPositionResidual << std::endl;
      residfile << maxVelocityResidualAcceptable << "   " << maxVelocityResidual << std::endl;
      residfile << collisionsPassed << std::endl;
      residfile.close();      

      std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Saved residual to residual.txt." << std::endl;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Rendering Functions

  virtual void initializeOpenGLRenderer()
  {
  }

  virtual void renderSceneOpenGL()
  {
    assert( m_scene_renderer != NULL );
    assert( m_scene_renderer != NULL );
    
    m_scene_renderer->renderParticleSimulation(*m_scene);

    if( m_comparison_scene != NULL )
    {
      std::vector<CollisionInfo> comparison_impulses;
      m_scene_renderer->circleMajorParticleSimulationResiduals( *m_scene, *m_comparison_scene, m_collision_handler ? &m_collision_handler->getImpulses() : NULL, &comparison_impulses);
    }
  }

  virtual void renderSceneDifferencesOpenGL()
  {
  }

  virtual void updateOpenGLRendererState()
  {
    assert( m_scene_renderer != NULL );
    m_scene_renderer->updateParticleSimulationState(*m_scene);
  }

  virtual void computeCameraCenter( renderingutils::Viewport& view )
  {
    const VectorXs& x = m_scene->getX();

    // Compute the bounds on all particle positions
    scalar max_x = -std::numeric_limits<scalar>::infinity();
    scalar min_x =  std::numeric_limits<scalar>::infinity();
    scalar max_y = -std::numeric_limits<scalar>::infinity();
    scalar min_y =  std::numeric_limits<scalar>::infinity();
    for( int i = 0; i < m_scene->getNumParticles(); ++i )
    {
      if( x(2*i) > max_x )   max_x = x(2*i);
      if( x(2*i) < min_x )   min_x = x(2*i);
      if( x(2*i+1) > max_y ) max_y = x(2*i+1);
      if( x(2*i+1) < min_y ) min_y = x(2*i+1);
    }

    // Set center of view to center of bounding box
    view.cx = 0.5*(max_x+min_x);
    view.cy = 0.5*(max_y+min_y);

    // Set the zoom such that all particles are in view
    view.rx = 0.5*(max_x-min_x);
    if( view.rx == 0.0 ) view.rx = 1.0;
    view.ry = 0.5*(max_y-min_y);
    if( view.ry == 0.0 ) view.ry = 1.0;
  }
  
  /////////////////////////////////////////////////////////////////////////////
  // SVG Rendering Functions

  virtual void renderSceneSVG( const std::string& name )
  {
    assert( m_svg_renderer != NULL );
    
    if( m_comparison_scene == NULL ) 
    {
      m_svg_renderer->renderScene(name);
    }
    else 
    {
      m_svg_renderer->renderComparisonScene( name, *m_comparison_scene,
                                            m_collision_handler != NULL ? &m_collision_handler->getImpulses() : NULL,
                                            &m_comparison_impulses );
    }
  }

  virtual void updateSVGRendererState()
  {
    assert( m_svg_renderer != NULL );
    m_svg_renderer->updateState();
  }

  /////////////////////////////////////////////////////////////////////////////
  // Serialization Functions
  
  virtual void copyComparisonSceneToScene()
  {
    assert( m_scene != NULL );
    assert( m_comparison_scene != NULL );
    m_scene->copyState( *m_comparison_scene );
  }

  virtual void serializeScene( std::ofstream& outputstream )
  {
    assert( m_scene != NULL );
    m_scene_serializer.serializeScene( *m_scene, outputstream );

    if( m_collision_handler != NULL )
    {
      m_collision_handler->serializeImpulses( outputstream );
    }    
  }

  virtual void loadComparisonScene( std::ifstream& inputstream )
  {
    assert( m_comparison_scene != NULL );
    m_scene_serializer.loadScene( *m_comparison_scene, inputstream );
    
    if( m_collision_handler != NULL )
    {
      m_comparison_impulses.clear();
      if( m_collision_handler != NULL ) 
      {
        m_collision_handler->loadImpulses( m_comparison_impulses, inputstream );
      }
    }
  }

  virtual void loadHybridData(std::ifstream &inputstream)
  {
    if(m_collision_handler != NULL)
    {
      if(m_collision_handler->getName() == "Hybrid Collision Handling")
      {
        ((HybridCollisionHandler *)m_collision_handler)->loadHybridData(inputstream);
      }
    }
  }

  virtual void serializeHybridData(std::ofstream &outputstream)
  {
    if(m_collision_handler != NULL)
    {
      if(m_collision_handler->getName() == "Hybrid Collision Handling")
      {
        ((HybridCollisionHandler *)m_collision_handler)->serializeHybridData(outputstream);
      }
    }
  }


  /////////////////////////////////////////////////////////////////////////////
  // Status Functions
  
  virtual std::string getSolverName()
  {
    assert( m_scene_stepper != NULL );
    return m_scene_stepper->getName();
  }

  virtual std::string getCollisionHandlerName()
  {
    if( m_collision_handler == NULL ) 
    {
      return "Collisions disabled";
    }
    return m_collision_handler->getName();
  }

  virtual void outputCallback( std::ostream& strm )
  {
  }

private:
  TwoDScene* m_scene;
  TwoDScene* m_comparison_scene;

  SceneStepper* m_scene_stepper;


  CollisionDetector *m_collision_detector;
  CollisionHandler* m_collision_handler;

  std::vector<CollisionInfo> m_comparison_impulses;

  TwoDSceneRenderer* m_scene_renderer;
  TwoDSceneSVGRenderer* m_svg_renderer;
  
  TwoDSceneSerializer m_scene_serializer;
  
  TwoDSceneGrader* m_scene_grader;
};

#endif
