#include "JointMotorBodySimulation.h"
#include "JointMotorBodyController.h"
#include <Eigen/Dense>
#include "FOSSSim/quadprog/QuadProg++.hh"

extern int g_current_step;
extern scalar g_dt;

JointMotorBodySimulation::JointMotorBodySimulation( JointMotorBodyScene* scene, JointMotorBodyScene* comparison_scene, RigidBodyStepper* stepper, TwoDSceneRenderer* scene_renderer, std::vector<OpenGLRenderer*> oglrenderers, RigidBodyCollisionDetector* collision_detector, JointMotorBodySimulationJudge * judge, std::vector<ScheduledPushEvent> * pushes )
    : m_scene(scene)
    //, m_comparison_scene(comparison_scene)
    , m_integrator(stepper)
    , m_scene_renderer(scene_renderer)
    , m_opengl_renderers(oglrenderers)
    //, m_grader(NULL)
    , m_collision_detector(collision_detector)
    , m_judge(judge)
    , m_pushes(pushes)
{
    assert( m_scene != NULL );
    assert( m_integrator != NULL );
    for( std::vector<OpenGLRenderer*>::size_type i = 0; i < m_opengl_renderers.size(); ++i ) assert( m_opengl_renderers[i] != NULL );
    
    
    
    //  if( m_comparison_scene != NULL ) m_grader = new RigidBodySceneGrader;
    
}

JointMotorBodySimulation::~JointMotorBodySimulation()
{
    if( m_scene != NULL )
    {
        delete m_scene;
        m_scene = NULL;
    }
    //  if( m_comparison_scene != NULL )
    //  {
    //    delete m_comparison_scene;
    //    m_comparison_scene = NULL;
    //  }
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
    for( std::vector<OpenGLRenderer*>::size_type i = 0; i < m_opengl_renderers.size(); ++i )
    {
        if( m_opengl_renderers[i] != NULL )
        {
            delete m_opengl_renderers[i];
            m_opengl_renderers[i] = NULL;
        }
    }
    //  if( m_grader != NULL )
    //  {
    //    delete m_grader;
    //    m_grader = NULL;
    //  }
    if( m_collision_detector != NULL )
    {
        delete m_collision_detector;
        m_collision_detector = NULL;
    }
}

/////////////////////////////////////////////////////////////////////////////
// Simulation Control Functions

void JointMotorBodySimulation::beforeSimulationStarts()
{
    // print detector information
    std::cout << "The goal of this test is to satisfy the following detectors:" << std::endl;
    for (int i = 0; i < m_judge->ndetectors(); i++)
    {
        const JointMotorBodySimulationJudge::Detector * detector = m_judge->detector(i);
        std::cout << "Detector " << i << ": " << detector->detailedDescription() << std::endl;
    }
}

void JointMotorBodySimulation::keyboard(unsigned char key, int x, int y)
{
      //std::cout << m_scene->getRigidBodies().size() << std::endl;
    /*
  if (key == 'd' || key == 'D')
  {
    if (m_judge->ndetectors() > 0)
    {
      m_judge->showAll(!m_judge->detector(0)->visible());
      glutPostRedisplay();
    }
  }
  
  */
    m_scene->set_key(key);
    m_scene->set_buf(15);
    
}

void JointMotorBodySimulation::mouse(int button, int state, int x, int y)
{
    m_judge->mouse(button, state, x, y);
}

void JointMotorBodySimulation::motion(int x, int y)
{
    m_judge->motion(x, y);
}

void JointMotorBodySimulation::passiveMotion(int x, int y)
{
    m_judge->passiveMotion(x, y);
}

void JointMotorBodySimulation::getFeatures(std::vector<Matrix2s> & features)
{
    for (size_t i = 0; i < m_scene->getRigidBodies().size(); i++)
    {
        RigidBody * rb = m_scene->getRigidBody(i);
        Matrix2s bb;  // bounding box of the rigid body
        bb.col(0) = rb->getX();
        bb.col(1) = rb->getX();
        if (bb.col(0).y() > 0) bb.col(0).y() = 0; // include ground
        
        scalar r = rb->getRadius();
        for (int j = 0; j < rb->getVertices().size(); j += 2)
        {
            Vector2s x = rb->computeWorldSpacePosition(rb->getVertices().segment<2>(j));      
            bb.col(0).x() = std::min(x.x() - r, bb.col(0).x());
            bb.col(0).y() = std::min(x.y() - r, bb.col(0).y());
            bb.col(1).x() = std::max(x.x() + r, bb.col(1).x());
            bb.col(1).y() = std::max(x.y() + r, bb.col(1).y());
        }
        
        features.push_back(bb);
    }
    
    m_judge->getFeatures(features);
}

void JointMotorBodySimulation::stepSystem( const scalar& dt )
{
    assert( dt > 0.0 );
    assert( m_scene != NULL );
    assert( m_integrator != NULL );
    
    m_scene->dt() = dt;
    
    // Step the simulated scene forward
    m_integrator->stepScene( *m_scene, dt );
    
    // Run detection
    std::set<RigidBodyCollision> collisions;
    if( m_collision_detector != NULL )
        m_collision_detector->detectCollisions(m_scene->getRigidBodies(),collisions);
    
    // Velocity projection to resolve collisions and enforce joint constraints
    std::vector<RigidBody> & rbs = m_scene->getRigidBodies();
    int R = rbs.size();
    int C = collisions.size();
    int J = 0;
    for (size_t i = 0; i < m_scene->getJointMotorBodies().size(); i++)
        J += (m_scene->getJointMotorBody(i)->nlinks() - 1) * 2;
    
    int V = 0;
    std::vector<int> vmap(R);
    for (int i = 0; i < R; i++)
        if (!rbs[i].isFixed())
            vmap[i] = V++;
        else 
            vmap[i] = -1;			
    
    // construct the constraints for collision
    MatrixXs N(V * 3, C);
    std::vector<VectorXs> eta(C, VectorXs::Zero(V * 3));
    
    int i = 0;
    for (std::set<RigidBodyCollision>::iterator c = collisions.begin(); c != collisions.end(); c++, i++)
    {
        MatrixXs gamma = MatrixXs::Zero(2, V * 3);
        scalar theta0 = (c->i0 < 0 ? 0 : rbs[c->i0].getTheta());
        scalar theta1 = (c->i1 < 0 ? 0 : rbs[c->i1].getTheta());
        
        if (c->i0 >= 0 && !rbs[c->i0].isFixed())
        {
            gamma.block(0, vmap[c->i0] * 3, 2, 2) = MatrixXs::Identity(2, 2);
            if (rbs[c->i0].getI() > 0)  // for zero moment of inertia, rotation is disabled
                gamma.block(0, vmap[c->i0] * 3 + 2, 2, 1) = Vector2s(-sin(theta0) * c->r0.x() - cos(theta0) * c->r0.y(), cos(theta0) * c->r0.x() - sin(theta0) * c->r0.y());
        }
        
        if (c->i1 >= 0 && !rbs[c->i1].isFixed())
        {
            gamma.block(0, vmap[c->i1] * 3, 2, 2) = -MatrixXs::Identity(2, 2);
            if (rbs[c->i1].getI() > 0)  // for zero moment of inertia, rotation is disabled
                gamma.block(0, vmap[c->i1] * 3 + 2, 2, 1) = Vector2s(sin(theta1) * c->r1.x() + cos(theta1) * c->r1.y(), -cos(theta1) * c->r1.x() + sin(theta1) * c->r1.y());
        }
        
        eta[i] = -gamma.transpose() * c->nhat;
    }
    
    for (i = 0; i < C; i++)
    {
        N.block(0, i, V * 3, 1) = eta[i];
    }
    
    // construct the constraints for joints
    MatrixXs L(V * 3, J);
    
    int j = 0;
    for (size_t i = 0; i < m_scene->getJointMotorBodies().size(); i++)
    {
        for (int k = 1; k < m_scene->getJointMotorBody(i)->nlinks(); k++)
        {
            JointMotorBody::Link * link = m_scene->getJointMotorBody(i)->link(k);
            JointMotorBody::Link * parent = link->parent;
            MatrixXs gamma = MatrixXs::Zero(2, V * 3);
            
            int i0 = link->rbindex;
            int i1 = parent->rbindex;
            Vector2s r0 = link->axis_in_self_body_space;
            Vector2s r1 = link->axis_in_parent_body_space;
            
            scalar theta0 = rbs[i0].getTheta();
            scalar theta1 = rbs[i1].getTheta();
            
            assert(!rbs[i0].isFixed());
            assert(!rbs[i1].isFixed());
            
            assert(rbs[i0].getI() > 0);
            assert(rbs[i1].getI() > 0);
            
            gamma.block(0, vmap[i0] * 3, 2, 2) = MatrixXs::Identity(2, 2);
            gamma(0, vmap[i0] * 3 + 2) = -sin(theta0) * r0.x() - cos(theta0) * r0.y();
            gamma(1, vmap[i0] * 3 + 2) = cos(theta0) * r0.x() - sin(theta0) * r0.y();
            
            gamma.block(0, vmap[i1] * 3, 2, 2) = -MatrixXs::Identity(2, 2);
            gamma(0, vmap[i1] * 3 + 2) = sin(theta1) * r1.x() + cos(theta1) * r1.y();
            gamma(1, vmap[i1] * 3 + 2) = -cos(theta1) * r1.x() + sin(theta1) * r1.y();
            
            L.col(j * 2 + 0) = gamma.row(0).transpose();
            L.col(j * 2 + 1) = gamma.row(1).transpose();
            
            j++;
        }
    }
    
    // construct the constraints for friction
    MatrixXs F(V * 3, C);
    
    i = 0;
    for (std::set<RigidBodyCollision>::iterator c = collisions.begin(); c != collisions.end(); c++, i++)
    {
        MatrixXs gamma = MatrixXs::Zero(2, V * 3);
        scalar theta0 = (c->i0 < 0 ? 0 : rbs[c->i0].getTheta());
        scalar theta1 = (c->i1 < 0 ? 0 : rbs[c->i1].getTheta());
        
        if (c->i0 >= 0 && !rbs[c->i0].isFixed())
        {
            gamma.block(0, vmap[c->i0] * 3, 2, 2) = MatrixXs::Identity(2, 2);
            if (rbs[c->i0].getI() > 0)  // for zero moment of inertia, rotation is disabled
                gamma.block(0, vmap[c->i0] * 3 + 2, 2, 1) = Vector2s(-sin(theta0) * c->r0.x() - cos(theta0) * c->r0.y(), cos(theta0) * c->r0.x() - sin(theta0) * c->r0.y());
        }
        
        if (c->i1 >= 0 && !rbs[c->i1].isFixed())
        {
            gamma.block(0, vmap[c->i1] * 3, 2, 2) = -MatrixXs::Identity(2, 2);
            if (rbs[c->i1].getI() > 0)  // for zero moment of inertia, rotation is disabled
                gamma.block(0, vmap[c->i1] * 3 + 2, 2, 1) = Vector2s(sin(theta1) * c->r1.x() + cos(theta1) * c->r1.y(), -cos(theta1) * c->r1.x() + sin(theta1) * c->r1.y());
        }
        
        Vector2s tangent(-c->nhat.y(), c->nhat.x());
        F.col(i) = -gamma.transpose() * tangent;
    }
    
    //std::cout << "num of collisions: " << i << std::endl;
    
    // solve the minimization
    VectorXs M(V * 3);
    for (int i = 0; i < R; i++)
    {
        if (!rbs[i].isFixed())
        {
            M(vmap[i] * 3 + 0) = M(vmap[i] * 3 + 1) = rbs[i].getM();
            M(vmap[i] * 3 + 2) = (rbs[i].getI() > 0 ? rbs[i].getI() : 1); // for zero moment of inertia the lack of rotation dof will be reflected in constraints, not here
        }
    }
    
    VectorXs qdotminus(V * 3);
    for (int i = 0; i < R; i++)
    {
        if (!rbs[i].isFixed())
        {
            qdotminus(vmap[i] * 3 + 0) = rbs[i].getV().x();
            qdotminus(vmap[i] * 3 + 1) = rbs[i].getV().y();
            qdotminus(vmap[i] * 3 + 2) = rbs[i].getOmega();
        }
    }
    VectorXs mq = M.asDiagonal() * qdotminus;
    
    //	std::cout << "M: " << M.transpose() << std::endl;
    //	std::cout << "mq: " << mq.transpose() << std::endl;
    //	std::cout << "N: " << N << std::endl;
    
    // construct equality constraints matrix, eliminating redundant constraints (QuadProg will complain if we don't)
    MatrixXs E_init(V * 3, J + C); E_init.block(0, 0, V * 3, J) = L; E_init.block(0, J, V * 3, C) = F;
    std::vector<int> E_index;
    for (int i = 0; i < J + C; i++)
    {
        int j = 0;
        for (j = 0; j < i; j++)
            if ((E_init.col(i) - E_init.col(j)).norm() < 1e-10)
                break;
        if (j == i)
            E_index.push_back(i);
    }
    MatrixXs E(V * 3, E_index.size());
    for (int i = 0; i < E_index.size(); i++)
        E.col(i) = E_init.col(E_index[i]);
    
    // Matrix in quadratic form of objective
    QuadProgPP::Matrix<scalar> G;
    // Equality constraints
    QuadProgPP::Matrix<scalar> CE;
    // Inequality constraints
    QuadProgPP::Matrix<scalar> CI;
    
    QuadProgPP::Vector<scalar> g0;
    QuadProgPP::Vector<scalar> ce0;
    QuadProgPP::Vector<scalar> ci0;
    QuadProgPP::Vector<scalar> x;
    
    G.resize(V * 3, V * 3);
    for (int i = 0; i < V * 3; i++) for (int j = 0; j < V * 3; j++) G[i][j] = (i == j ? M(i) : 0);
    g0.resize(V * 3);
    for (int i = 0; i < V * 3; i++) g0[i] = -mq(i);
    
    CE.resize(V * 3, E.cols());
    for (int i = 0; i < V * 3; i++) for (int j = 0; j < E.cols(); j++) CE[i][j] = E(i, j);
    ce0.resize(E.cols());
    for (int i = 0; i < E.cols(); i++) ce0[i] = 0;
    
    CI.resize(V * 3, C);
    for (int i = 0; i < V * 3; i++) for (int j = 0; j < C; j++) CI[i][j] = N(i, j);
    ci0.resize(C);
    for (int i = 0; i < C; i++) ci0[i] = 0;
    
    //  std::cout << "G size " << G.nrows() << "x" << G.ncols() << std::endl;
    //  std::cout << "g0 size " << g0.size() << std::endl;
    //  std::cout << "CE size " << CE.nrows() << "x" << CE.ncols() << std::endl;
    //  std::cout << "ce0 size " << ce0.size() << std::endl;
    //  std::cout << "CI size " << CI.nrows() << "x" << CI.ncols() << std::endl;
    //  std::cout << "ci0 size " << ci0.size() << std::endl;
    //  
    //  std::cout << "G = \n" << M.transpose() << std::endl;
    //  std::cout << "CE = \n" << E << std::endl;
    //  std::cout << "CI = \n" << N << std::endl;  
    
    x.resize(V * 3);
    solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    
    for (int i = 0; i < R; i++)
    {
        if (!rbs[i].isFixed())
        {
            Vector2s newv(x[vmap[i] * 3 + 0], x[vmap[i] * 3 + 1]);
            rbs[i].getX() += (newv - rbs[i].getV()) * dt; // counteract the position drift (because we are using symplectic euler -- it really should be an integral part of the time stepping here)
            rbs[i].getV() = newv;
            rbs[i].getOmega() = x[vmap[i] * 3 + 2];
        }
    }
    
    // scheduled impulses
    for (size_t i = 0; i < m_pushes->size(); i++)
    {
        ScheduledPushEvent & e = m_pushes->at(i);
        if (e.alpha == 0 && g_current_step * g_dt > e.time)
        {
            e.alpha = 1;
            m_scene->getRigidBody(e.rb)->getV() += e.impulse / m_scene->getRigidBody(e.rb)->getM();
        }
    }
    
    // judge update
    m_judge->updateDetectors(m_scene, dt);
    
    static scalar et_count_down = 3.0;
    int r = m_judge->result();
    if (r == 1)
    {
        if (et_count_down == 3.0)
            std::cout << "All detectors satisfied, simulation ending in 3 seconds..." << std::endl;
        et_count_down -= dt;
        if (et_count_down <= 0)
            exit(0);  // early termination upon satisfaction of all detectors
    } else if (r == 2)
    {
        if (et_count_down == 3.0)
            std::cout << "Passive detectors failed, simulation ending in 3 seconds..." << std::endl;
        et_count_down -= dt;
        if (et_count_down <= 0)
            exit(0);  // early termination upon failure of any passive detectors. once one of them fail, there is no change this simulation can pass this test.
    }
}

/////////////////////////////////////////////////////////////////////////////
// Scene Benchmarking Functions

void JointMotorBodySimulation::updateSceneComparison()
{
    //  assert( m_scene != NULL );
    //  assert( m_comparison_scene != NULL );
    //  assert( m_grader != NULL );
    //  m_grader->addDifferencesToResiduals( *m_scene, *m_comparison_scene );
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::printErrorInformation( bool print_pass )
{
    //  m_grader->printSummary( print_pass );
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::printDetectorReport()
{
    std::cout << outputmod::startgreen << "Detector results: " << outputmod::endgreen << std::endl;
    m_judge->report(std::cout);
    std::cout << "Overall success: " << (m_judge->result() == 1 ? "Passed." : "Failed.") << std::endl;
}


/////////////////////////////////////////////////////////////////////////////
// Rendering Functions

// TODO: Scrap these two functions
void JointMotorBodySimulation::initializeOpenGLRenderer()
{
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::renderSceneDifferencesOpenGL()
{
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::renderSceneOpenGL()
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
    
    //  if( m_comparison_scene != NULL )
    //  {
    //  //  std::cout << "TEMPORARY PROBLEM: COMPARISON RENDERING NOT IMPLEMENTED" << std::endl;
    //  //  std::vector<ImpulseInfo> comparison_impulses;
    //  //  m_scene_renderer->circleMajorResiduals(*m_scene,*m_comparison_scene,NULL,&comparison_impulses);
    //    m_scene_renderer->circleMajorRigidBodySimulationResiduals( *m_scene, *m_comparison_scene );
    //  }
    
    m_judge->render();
}

void JointMotorBodySimulation::updateOpenGLRendererState()
{
    // None of our effects currently track state, so nothing to do here
    //assert( m_scene_renderer != NULL );
    //m_scene_renderer->updateState();
}

void JointMotorBodySimulation::computeCameraCenter( renderingutils::Viewport& view )
{
    //std::cout << "enter camera jointmotor" << std::endl;
    
    const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();
    
    scalar max_x = -std::numeric_limits<scalar>::infinity();
    scalar min_x =  std::numeric_limits<scalar>::infinity();
    scalar max_y = -std::numeric_limits<scalar>::infinity();
    scalar min_y = std::numeric_limits<scalar>::infinity();
    
    //scalar min_y = 0;    // ground
    
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
    
    if(max_y - min_y < 10){
        max_y = min_y + 20;
    }
    min_y -= 7;
    max_y -= 7;
    
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

void JointMotorBodySimulation::renderSceneSVG( const std::string& name )
{
    // Not supported
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::updateSVGRendererState()
{
    // Not supported
    assert(!"Method not supported by JointMotorBodySimulation");
}

/////////////////////////////////////////////////////////////////////////////
// Serialization Functions

void JointMotorBodySimulation::copyComparisonSceneToScene()
{
    //  assert( m_scene != NULL );
    //  assert( m_comparison_scene != NULL );
    //
    //  *m_scene = *m_comparison_scene;
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::serializeScene( std::ofstream& outputstream )
{
    //  assert( outputstream.is_open() );
    //  assert( m_scene != NULL );
    //
    //  m_scene->serialize(outputstream);
    //
    //  //if( m_collision_handler != NULL )
    //  //{
    //  //  m_collision_handler->serializeImpulses( outputstream );
    //  //}    
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::serializeStartOfSimState( std::ofstream& ofs )
{
    //  assert(ofs.is_open());
    //  assert( m_scene != NULL );
    //
    //  const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();
    //
    //  // Serialize the total mass of each rigid body
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar M = rbs[i].getM();
    //    ofs.write((char*)&M,sizeof(scalar));
    //  }
    //  // Serialize the center of mass of each rigid body
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    Vector2s X = rbs[i].getX();
    //    ofs.write((char*)X.data(),X.size()*sizeof(scalar));
    //  }
    //  // Serialize the moment of inertia in body space of each rigid body
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar I = rbs[i].getI();
    //    ofs.write((char*)&I,sizeof(scalar));
    //  }
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::deserializeAndVerifyStartOfSimState( std::ifstream& ifs )
{
    //  assert(ifs.is_open());
    //  assert( m_scene != NULL );
    //  assert( m_grader != NULL );
    //
    //  const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();
    //
    //  // Deserialize the total mass of each rigid body
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar M = std::numeric_limits<scalar>::signaling_NaN();
    //    ifs.read((char*)&M,sizeof(scalar));
    //
    //    // Compute the the total mass with the oracle
    //    scalar Moracle = rbs[i].getM();
    //    m_grader->checkMassComputation( i, M, Moracle );
    //  }
    //  // Deserialize the center of mass of each rigid body
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    Vector2s X; X.setConstant(std::numeric_limits<scalar>::signaling_NaN());
    //    ifs.read((char*)X.data(),X.size()*sizeof(scalar));
    //
    //    // Compute the center of mass with the oracle
    //    Vector2s Xoracle = rbs[i].getX();
    //    m_grader->checkCenterOfMassComputation( i, X, Xoracle );
    //  }
    //  // Serialize the moment of inertia in body space of each rigid body
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar I = std::numeric_limits<scalar>::signaling_NaN();
    //    ifs.read((char*)&I,sizeof(scalar));
    //
    //    // Compute the momento f inertia with the oracle
    //    scalar Ioracle = rbs[i].getI();
    //    m_grader->checkMomentOfInertiaComputation( i, I, Ioracle );
    //  }
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::serializePerStepQuantities( std::ofstream& ofs )
{
    //  assert(ofs.is_open());
    //  assert( m_scene != NULL );
    //
    //  const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();
    //
    //  // Serialize the momentum of each rigid body
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    Vector2s p = rbs[i].computeTotalMomentum();
    //    ofs.write((char*)p.data(),p.size()*sizeof(scalar));
    //  }
    //  // Serialize the center-of-mass' contribution to angular momentum
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar Lcm = rbs[i].computeCenterOfMassAngularMomentum();
    //    ofs.write((char*)&Lcm,sizeof(scalar));
    //  }
    //  // Serialize the spin about the center-of-mass' contribution to angular momentum
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar Lspin = rbs[i].computeSpinAngularMomentum();
    //    ofs.write((char*)&Lspin,sizeof(scalar));
    //  }
    //  // Serialize the center-of-mass' contribution to kinetic energy
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar Tcm = rbs[i].computeCenterOfMassKineticEnergy();
    //    ofs.write((char*)&Tcm,sizeof(scalar));
    //  }  
    //  // Serialize the spin about the center-of-mass' contribution to kinetic energy
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar Tspin = rbs[i].computeSpinKineticEnergy();
    //    ofs.write((char*)&Tspin,sizeof(scalar));
    //  }
    //  // Serialize the total potential energy
    //  scalar U = m_scene->computeTotalPotentialEnergy();
    //  ofs.write((char*)&U,sizeof(scalar));
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::deserializeAndVerifyPerStepQuantities( std::ifstream& ifs )
{
    //  assert(ifs.is_open());
    //  assert( m_scene != NULL );
    //  assert( m_grader != NULL );  
    //
    //  const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();
    //
    //  // Serialize the momentum of each rigid body
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    Vector2s P; P.setConstant(std::numeric_limits<scalar>::signaling_NaN());
    //    ifs.read((char*)P.data(),P.size()*sizeof(scalar));
    //
    //    Vector2s Poracle = rbs[i].computeTotalMomentum();
    //    m_grader->checkMomentumComputation( i, P, Poracle );
    //  }
    //  // Deserialize the center-of-mass' contribution to angular momentum
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar Lcm = std::numeric_limits<scalar>::signaling_NaN();
    //    ifs.read((char*)&Lcm,sizeof(scalar));
    //
    //    scalar Lcmorcl = rbs[i].computeCenterOfMassAngularMomentum();
    //    m_grader->checkCenterOfMassAngularMomentumComputation( i, Lcm, Lcmorcl );
    //  }
    //  // Deserialize the spin about the center-of-mass' contribution to angular momentum
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar Lspin = std::numeric_limits<scalar>::signaling_NaN();
    //    ifs.read((char*)&Lspin,sizeof(scalar));
    //
    //    scalar Lspinorcl = rbs[i].computeSpinAngularMomentum();
    //    m_grader->checkSpinAngularMomentumComputation( i, Lspin, Lspinorcl );
    //  }
    //  // Deserialize the center-of-mass' contribution to kinetic energy
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar Tcm = std::numeric_limits<scalar>::signaling_NaN();
    //    ifs.read((char*)&Tcm,sizeof(scalar));
    //
    //    scalar Tcmorcl = rbs[i].computeCenterOfMassKineticEnergy();
    //    m_grader->checkCenterOfMassKineticEnergyComputation( i, Tcm, Tcmorcl );
    //  }
    //  // Deserialize the spin about the center-of-mass' contribution to kinetic energy
    //  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    //  {
    //    scalar Tspin = std::numeric_limits<scalar>::signaling_NaN();
    //    ifs.read((char*)&Tspin,sizeof(scalar));
    //
    //    scalar Tspinorcl = rbs[i].computeSpinKineticEnergy();
    //    m_grader->checkSpinKineticEnergyComputation( i, Tspin, Tspinorcl );
    //  }
    //  // Deserialize the total potential energy
    //  scalar U = std::numeric_limits<scalar>::signaling_NaN(); //m_scene->computeTotalPotentialEnergy();
    //  ifs.read((char*)&U,sizeof(scalar));
    //  scalar Uorcl = m_scene->computeTotalPotentialEnergy();
    //  m_grader->checkTotalPotentialEnergyComputation( U, Uorcl );
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::loadComparisonScene( std::ifstream& inputstream )
{
    //  assert( inputstream.is_open() );
    //  assert( m_comparison_scene != NULL );
    //
    //  m_comparison_scene->deserialize(inputstream);
    //
    //  //if( m_collision_handler != NULL )
    //  //{
    //  //  m_comparison_impulses.clear();
    //  //  if( m_collision_handler != NULL ) m_collision_handler->loadImpulses( m_comparison_impulses, inputstream );
    //  //}
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::loadHybridData(std::ifstream &ifs)
{
    assert(!"Method not supported by JointMotorBodySimulation");
}

void JointMotorBodySimulation::serializeHybridData(std::ofstream &ofs)
{
    assert(!"Method not supported by JointMotorBodySimulation");
}


/////////////////////////////////////////////////////////////////////////////
// Status Functions

std::string JointMotorBodySimulation::getSolverName()
{
    assert( m_integrator != NULL );
    return m_integrator->getName();
}

std::string JointMotorBodySimulation::getCollisionHandlerName()
{
    if( m_collision_detector != NULL )
        return m_collision_detector->getName();
    
    return "disabled";
}

void JointMotorBodySimulation::outputCallback( std::ostream& strm )
{
    
}
