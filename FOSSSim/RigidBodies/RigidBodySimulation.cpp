#include "RigidBodySimulation.h"

RigidBodySimulation::RigidBodySimulation( RigidBodyScene* scene, RigidBodyScene* comparison_scene, RigidBodyStepper* stepper, TwoDSceneRenderer* scene_renderer, std::vector<OpenGLRenderer*> oglrenderers, TwoDSceneSVGRenderer* svg_renderer, RigidBodyCollisionDetector* collision_detector, RigidBodyCollisionResolver* collision_resolver )
: m_scene(scene)
, m_comparison_scene(comparison_scene)
, m_integrator(stepper)
, m_scene_renderer(scene_renderer)
, m_svg_renderer(svg_renderer)
, m_opengl_renderers(oglrenderers)
, m_grader(NULL)
, m_collision_detector(collision_detector)
, m_collision_resolver(collision_resolver)
{
  assert( m_scene != NULL );
  assert( m_integrator != NULL );
  for( std::vector<OpenGLRenderer*>::size_type i = 0; i < m_opengl_renderers.size(); ++i ) assert( m_opengl_renderers[i] != NULL );

  if( m_comparison_scene != NULL ) m_grader = new RigidBodySceneGrader;
}

RigidBodySimulation::~RigidBodySimulation()
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
  if( m_integrator != NULL )
  {
    delete m_integrator;
    m_integrator = NULL;
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
  for( std::vector<OpenGLRenderer*>::size_type i = 0; i < m_opengl_renderers.size(); ++i )
  {
    if( m_opengl_renderers[i] != NULL )
    {
      delete m_opengl_renderers[i];
      m_opengl_renderers[i] = NULL;
    }
  }
  if( m_grader != NULL )
  {
    delete m_grader;
    m_grader = NULL;
  }
  if( m_collision_detector != NULL )
  {
    delete m_collision_detector;
    m_collision_detector = NULL;
  }
  if( m_collision_resolver != NULL )
  {
    delete m_collision_resolver;
    m_collision_resolver = NULL;
  }
}
  
/////////////////////////////////////////////////////////////////////////////
// Simulation Control Functions

void RigidBodySimulation::stepSystem( const scalar& dt )
{
  assert( dt > 0.0 );
  assert( m_scene != NULL );
  assert( m_integrator != NULL );

  // Extract the start-of-step positions for continuous time detection
  //if( m_collision_detector != NULL ) m_collision_detector->copyPreStepState(m_scene->getRigidBodies());

  // Step the simulated scene forward
  m_integrator->stepScene( *m_scene, dt );

  // Extract the end-of-step positions for continuous time detection
  //if( m_collision_detector != NULL ) m_collision_detector->copyPostStepState(m_scene->getRigidBodies());

  // Run detection
  if( m_collision_detector != NULL )
  {
    std::set<RigidBodyCollision> collisions;
    m_collision_detector->detectCollisions(m_scene->getRigidBodies(),collisions);

    assert( m_collision_resolver != NULL );
    m_collision_resolver->resolveCollisions( m_scene->getRigidBodies(), collisions );
  }

  // Check for obvious problems in the simulated scene
  //#ifdef DEBUG
  //  m_scene->checkConsistency();
  //#endif
}

/////////////////////////////////////////////////////////////////////////////
// Scene Benchmarking Functions

void RigidBodySimulation::updateSceneComparison()
{
  assert( m_scene != NULL );
  assert( m_comparison_scene != NULL );
  assert( m_grader != NULL );
  m_grader->addDifferencesToResiduals( *m_scene, *m_comparison_scene );
}

void RigidBodySimulation::printErrorInformation( bool print_pass )
{
  m_grader->printSummary( print_pass );
}

/////////////////////////////////////////////////////////////////////////////
// Rendering Functions

// TODO: Scrap these two functions
void RigidBodySimulation::initializeOpenGLRenderer()
{}
void RigidBodySimulation::renderSceneDifferencesOpenGL()
{}


void RigidBodySimulation::renderSceneOpenGL()
{
  assert( m_scene != NULL );
  assert( m_scene_renderer != NULL );
  
  // Render the rigid bodies
  m_scene_renderer->renderRigdBodySimulation(*m_scene);
  
  // Render any other misc. effects
  for( std::vector<OpenGLRenderer*>::size_type i = 0; i < m_opengl_renderers.size(); ++i ) 
  {
    m_opengl_renderers[i]->render(m_scene->getRigidBodies());
  }

  if( m_comparison_scene != NULL )
  {
  //  std::cout << "TEMPORARY PROBLEM: COMPARISON RENDERING NOT IMPLEMENTED" << std::endl;
  //  std::vector<ImpulseInfo> comparison_impulses;
  //  m_scene_renderer->circleMajorResiduals(*m_scene,*m_comparison_scene,NULL,&comparison_impulses);
    m_scene_renderer->circleMajorRigidBodySimulationResiduals( *m_scene, *m_comparison_scene );
  }
}

void RigidBodySimulation::updateOpenGLRendererState()
{
  // None of our effects currently track state, so nothing to do here
  //assert( m_scene_renderer != NULL );
  //m_scene_renderer->updateState();
}

void RigidBodySimulation::computeCameraCenter( renderingutils::Viewport& view )
{
  const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();

  scalar max_x = -std::numeric_limits<scalar>::infinity();
  scalar min_x =  std::numeric_limits<scalar>::infinity();
  scalar max_y = -std::numeric_limits<scalar>::infinity();
  scalar min_y =  std::numeric_limits<scalar>::infinity();    

  std::cout << "camera" << std::endl;
  
  // For each rigid body
  //for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  for( std::vector<RigidBody>::size_type i = 0; i < 2; ++i )
  {
    for( int j = 0; j < rbs[i].getNumVertices(); ++j )
    {
      Vector2s vrt = rbs[i].getWorldSpaceVertex(j);

      if( vrt.x() > max_x ) max_x = vrt.x();
      if( vrt.x() < min_x ) min_x = vrt.x();
      if( vrt.y() > max_y ) max_y = vrt.y();
      if( vrt.y() < min_y ) min_y = vrt.y();        
    }
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

void RigidBodySimulation::renderSceneSVG( const std::string& name )
{
  assert( m_svg_renderer != NULL );
  assert( m_scene != NULL );

  m_svg_renderer->renderRigidBodyScene(name,*m_scene);

  //if( m_comparison_scene == NULL ) 
  //{
  //  m_svg_renderer->renderScene(name);
  //}
  //else 
  //{
  //  m_svg_renderer->renderComparisonScene( name, *m_comparison_scene,
  //                                        m_collision_handler != NULL ? &m_collision_handler->getImpulses() : NULL,
  //                                        &m_comparison_impulses );
  //}
}

void RigidBodySimulation::updateSVGRendererState()
{
  // None of our effects currently track state, so nothing to do here
  //assert( m_svg_renderer != NULL );
  //m_svg_renderer->updateState();
}

/////////////////////////////////////////////////////////////////////////////
// Serialization Functions

void RigidBodySimulation::copyComparisonSceneToScene()
{
  assert( m_scene != NULL );
  assert( m_comparison_scene != NULL );

  *m_scene = *m_comparison_scene;
}

void RigidBodySimulation::serializeScene( std::ofstream& outputstream )
{
  assert( outputstream.is_open() );
  assert( m_scene != NULL );

  m_scene->serialize(outputstream);

  //if( m_collision_handler != NULL )
  //{
  //  m_collision_handler->serializeImpulses( outputstream );
  //}    
}

void RigidBodySimulation::serializeStartOfSimState( std::ofstream& ofs )
{
  assert(ofs.is_open());
  assert( m_scene != NULL );

  const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();

  // Serialize the total mass of each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar M = rbs[i].getM();
    ofs.write((char*)&M,sizeof(scalar));
  }
  // Serialize the center of mass of each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    Vector2s X = rbs[i].getX();
    ofs.write((char*)X.data(),X.size()*sizeof(scalar));
  }
  // Serialize the moment of inertia in body space of each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar I = rbs[i].getI();
    ofs.write((char*)&I,sizeof(scalar));
  }
}

void RigidBodySimulation::deserializeAndVerifyStartOfSimState( std::ifstream& ifs )
{
  assert(ifs.is_open());
  assert( m_scene != NULL );
  assert( m_grader != NULL );

  const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();

  // Deserialize the total mass of each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar M = std::numeric_limits<scalar>::signaling_NaN();
    ifs.read((char*)&M,sizeof(scalar));

    // Compute the the total mass with the oracle
    scalar Moracle = rbs[i].getM();
    m_grader->checkMassComputation( i, M, Moracle );
  }
  // Deserialize the center of mass of each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    Vector2s X; X.setConstant(std::numeric_limits<scalar>::signaling_NaN());
    ifs.read((char*)X.data(),X.size()*sizeof(scalar));

    // Compute the center of mass with the oracle
    Vector2s Xoracle = rbs[i].getX();
    m_grader->checkCenterOfMassComputation( i, X, Xoracle );
  }
  // Serialize the moment of inertia in body space of each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar I = std::numeric_limits<scalar>::signaling_NaN();
    ifs.read((char*)&I,sizeof(scalar));

    // Compute the momento f inertia with the oracle
    scalar Ioracle = rbs[i].getI();
    m_grader->checkMomentOfInertiaComputation( i, I, Ioracle );
  }
}

void RigidBodySimulation::serializePerStepQuantities( std::ofstream& ofs )
{
  assert(ofs.is_open());
  assert( m_scene != NULL );

  const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();

  // Serialize the momentum of each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    Vector2s p = rbs[i].computeTotalMomentum();
    ofs.write((char*)p.data(),p.size()*sizeof(scalar));
  }
  // Serialize the center-of-mass' contribution to angular momentum
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar Lcm = rbs[i].computeCenterOfMassAngularMomentum();
    ofs.write((char*)&Lcm,sizeof(scalar));
  }
  // Serialize the spin about the center-of-mass' contribution to angular momentum
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar Lspin = rbs[i].computeSpinAngularMomentum();
    ofs.write((char*)&Lspin,sizeof(scalar));
  }
  // Serialize the center-of-mass' contribution to kinetic energy
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar Tcm = rbs[i].computeCenterOfMassKineticEnergy();
    ofs.write((char*)&Tcm,sizeof(scalar));
  }  
  // Serialize the spin about the center-of-mass' contribution to kinetic energy
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar Tspin = rbs[i].computeSpinKineticEnergy();
    ofs.write((char*)&Tspin,sizeof(scalar));
  }
  // Serialize the total potential energy
  scalar U = m_scene->computeTotalPotentialEnergy();
  ofs.write((char*)&U,sizeof(scalar));
}

void RigidBodySimulation::deserializeAndVerifyPerStepQuantities( std::ifstream& ifs )
{
  assert(ifs.is_open());
  assert( m_scene != NULL );
  assert( m_grader != NULL );  

  const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();

  // Serialize the momentum of each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    Vector2s P; P.setConstant(std::numeric_limits<scalar>::signaling_NaN());
    ifs.read((char*)P.data(),P.size()*sizeof(scalar));

    Vector2s Poracle = rbs[i].computeTotalMomentum();
    m_grader->checkMomentumComputation( i, P, Poracle );
  }
  // Deserialize the center-of-mass' contribution to angular momentum
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar Lcm = std::numeric_limits<scalar>::signaling_NaN();
    ifs.read((char*)&Lcm,sizeof(scalar));

    scalar Lcmorcl = rbs[i].computeCenterOfMassAngularMomentum();
    m_grader->checkCenterOfMassAngularMomentumComputation( i, Lcm, Lcmorcl );
  }
  // Deserialize the spin about the center-of-mass' contribution to angular momentum
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar Lspin = std::numeric_limits<scalar>::signaling_NaN();
    ifs.read((char*)&Lspin,sizeof(scalar));

    scalar Lspinorcl = rbs[i].computeSpinAngularMomentum();
    m_grader->checkSpinAngularMomentumComputation( i, Lspin, Lspinorcl );
  }
  // Deserialize the center-of-mass' contribution to kinetic energy
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar Tcm = std::numeric_limits<scalar>::signaling_NaN();
    ifs.read((char*)&Tcm,sizeof(scalar));

    scalar Tcmorcl = rbs[i].computeCenterOfMassKineticEnergy();
    m_grader->checkCenterOfMassKineticEnergyComputation( i, Tcm, Tcmorcl );
  }
  // Deserialize the spin about the center-of-mass' contribution to kinetic energy
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    scalar Tspin = std::numeric_limits<scalar>::signaling_NaN();
    ifs.read((char*)&Tspin,sizeof(scalar));

    scalar Tspinorcl = rbs[i].computeSpinKineticEnergy();
    m_grader->checkSpinKineticEnergyComputation( i, Tspin, Tspinorcl );
  }
  // Deserialize the total potential energy
  scalar U = std::numeric_limits<scalar>::signaling_NaN(); //m_scene->computeTotalPotentialEnergy();
  ifs.read((char*)&U,sizeof(scalar));
  scalar Uorcl = m_scene->computeTotalPotentialEnergy();
  m_grader->checkTotalPotentialEnergyComputation( U, Uorcl );
}

void RigidBodySimulation::loadComparisonScene( std::ifstream& inputstream )
{
  assert( inputstream.is_open() );
  assert( m_comparison_scene != NULL );

  m_comparison_scene->deserialize(inputstream);

  //if( m_collision_handler != NULL )
  //{
  //  m_comparison_impulses.clear();
  //  if( m_collision_handler != NULL ) m_collision_handler->loadImpulses( m_comparison_impulses, inputstream );
  //}
}

void RigidBodySimulation::loadHybridData(std::ifstream &ifs)
{
}

void RigidBodySimulation::serializeHybridData(std::ofstream &ofs)
{
}

/////////////////////////////////////////////////////////////////////////////
// Status Functions

std::string RigidBodySimulation::getSolverName()
{
  assert( m_integrator != NULL );
  return m_integrator->getName();
}

std::string RigidBodySimulation::getCollisionHandlerName()
{
  if( m_collision_detector != NULL )
  {
    assert( m_collision_resolver != NULL );

    return m_collision_detector->getName() + ", " + m_collision_resolver->getName();
  }

  return "disabled";
}

void RigidBodySimulation::outputCallback( std::ostream& strm )
{
}
