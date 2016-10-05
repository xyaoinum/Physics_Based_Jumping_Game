#include "TwoDSceneXMLParser.h"
#include "HybridCollisionHandler.h"
#include "CollisionDetector.h"
#include "AllPairsDetector.h"
#include "ContestDetector.h"

void TwoDSceneXMLParser::loadExecutableSimulation( const std::string& file_name, bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, renderingutils::Viewport& view, scalar& dt, scalar& max_time, scalar& steps_per_sec_cap, renderingutils::Color& bgcolor, std::string& description, std::string& scenetag )
{
    // Load the xml document
    std::vector<char> xmlchars;
    rapidxml::xml_document<> doc;
    loadXMLFile( file_name, xmlchars, doc );
    
    // Attempt to locate the root node
    rapidxml::xml_node<>* node = doc.first_node("scene");
    if( node == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse xml scene file. Failed to locate root <scene> node. Exiting." << std::endl;
        exit(1);
    }
    
    // Determine what simulation type this is (particle, rigid body, etc)
    std::string simtype;
    loadSimulationType( node, simtype );
    
    // Parse common state
    loadMaxTime( node, max_time );
    loadMaxSimFrequency( node, steps_per_sec_cap );
    loadViewport( node, view );
    loadBackgroundColor( node, bgcolor );
    loadSceneDescriptionString( node, description );
    loadSceneTag( node, scenetag );
    
    // Parse the user-requested simulation type. The default is a particle simulation.
    if( simtype == "" || simtype == "particle-system" )
    {
        loadParticleSimulation( simulate_comparison, rendering_enabled, display_controller, execsim, view, dt, bgcolor, node );
    }
    else if( simtype == "rigid-body" )
    {
        loadRigidBodySimulation( simulate_comparison, rendering_enabled, display_controller, execsim, view, dt, bgcolor, node );
        //std::cout << "MAX_TIME: " << max_time << std::endl;
        //std::cout << "DT: " << dt << std::endl;
    }
    else if( simtype == "joint-motor-body" )
    {
        loadJointMotorBodySimulation( simulate_comparison, rendering_enabled, display_controller, execsim, view, dt, bgcolor, node );
    }
    else 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid simtype '" << simtype << "' specified. Valid options are 'particle-system' and 'rigid-body'. Exiting." << std::endl;
        exit(1);
    }
}

void TwoDSceneXMLParser::loadParticleSimulation( bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, renderingutils::Viewport& view, scalar& dt, renderingutils::Color& bgcolor, rapidxml::xml_node<>* node )
{
    TwoDScene* scene = new TwoDScene;
    
    // Scene
    loadParticles( node, *scene );
    loadEdges( node, *scene );
    loadHalfplanes( node, *scene );
    
    // Forces
    loadSpringForces( node, *scene );
    loadSimpleGravityForces( node, *scene );
    loadGravitationalForces( node, *scene );
    loadConstantForces( node, *scene );
    loadDragDampingForces( node, *scene );
    loadVorexForces( node, *scene );
    
    // Integrator/solver
    SceneStepper* scene_stepper = NULL;
    loadIntegrator( node, &scene_stepper, dt );
    assert( scene_stepper != NULL );
    assert( dt > 0.0 );
    
    CollisionDetector *collision_detector = NULL;
    loadCollisionDetector(node, *scene, &collision_detector);
    assert(collision_detector != NULL);
    CollisionHandler* collision_handler = NULL;
    loadCollisionHandler(node, *scene, &collision_handler, *collision_detector);
    
    
    // Rendering state
    std::vector<renderingutils::Color> particle_colors;
    particle_colors.resize(scene->getNumParticles(),renderingutils::Color(0.650980392156863,0.294117647058824,0.0));
    loadParticleColors( node, particle_colors );
    
    std::vector<renderingutils::Color> edge_colors;
    edge_colors.resize(scene->getNumEdges(),renderingutils::Color(0.0,0.388235294117647,0.388235294117647));
    loadEdgeColors( node, edge_colors );
    
    std::vector<renderingutils::Color> halfplane_colors;
    halfplane_colors.resize(scene->getNumHalfplanes(), renderingutils::Color(0,0,0));
    loadHalfplaneColors( node, halfplane_colors );
    
    std::vector<renderingutils::ParticlePath> particle_paths;
    loadParticlePaths( node, dt, particle_paths );  
    
    
    TwoDScene* comparison_scene = NULL;
    if( simulate_comparison )
    {
        comparison_scene = new TwoDScene;
        comparison_scene->copyState(*scene);
    }
    
    TwoDSceneRenderer* scene_renderer = NULL;
    if( rendering_enabled )
    {
        scene_renderer = new TwoDSceneRenderer(*scene,display_controller,particle_colors,edge_colors,halfplane_colors,particle_paths);
        scene_renderer->updateParticleSimulationState(*scene);
    }
    
    TwoDSceneSVGRenderer* svg_renderer = new TwoDSceneSVGRenderer(*scene,display_controller,particle_colors,edge_colors,halfplane_colors,particle_paths,512,512,bgcolor);
    svg_renderer->updateState();
    
    *execsim = new ParticleSimulation(scene, comparison_scene, collision_detector, collision_handler, scene_stepper, scene_renderer, svg_renderer);
}

void TwoDSceneXMLParser::loadRigidBodySimulation( bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, renderingutils::Viewport& view, scalar& dt, renderingutils::Color& bgcolor, rapidxml::xml_node<>* node )
{
    // Create a new rigid body scene
    RigidBodyScene* scene = new RigidBodyScene;
    
    // Load forces acting on and between rigid bodies
    std::vector<RigidBodySpringForce*> rbspringforces;
    loadRigidBodySpringForces( node, rbspringforces );
    std::vector<RigidBodyGravityForce*> rbgravityforces;
    loadRigidBodyGravityForces( node, rbgravityforces );
    std::vector<RigidBodyWindForce*> rbwindforces;
    loadRigidBodyWindForces( node, rbwindforces );
    
    for( std::vector<RigidBodyGravityForce*>::size_type i = 0; i < rbgravityforces.size(); ++i ) scene->insertForce( rbgravityforces[i] );
    for( std::vector<RigidBodySpringForce*>::size_type i = 0; i < rbspringforces.size(); ++i ) scene->insertForce( rbspringforces[i] );
    for( std::vector<RigidBodyWindForce*>::size_type i = 0; i < rbwindforces.size(); ++i ) scene->insertForce( rbwindforces[i] );
    
    // Load locations and masses of rigid body boundary vertices
    std::vector<Vector2s> rbvrts;
    std::vector<scalar> rbmasses;
    loadRigidBodyVertices( node, rbvrts, rbmasses );
    assert( rbvrts.size() == rbmasses.size() );
    
    // Load the rigid bodies themselves
    std::vector<RigidBody> rigidbodies;
    loadRigidBodies( node, rbvrts, rbmasses, rigidbodies );
    for( std::vector<RigidBody>::size_type i = 0; i < rigidbodies.size(); ++i ) scene->addRigidBody(rigidbodies[i]);
    
    // Load the rigid body integrator
    RigidBodyStepper* stepper = NULL;
    loadRigidBodyIntegrator( node, &stepper, dt );
    
    // Setup remaining renderers
    TwoDSceneRenderer* scene_renderer = new TwoDSceneRenderer(display_controller);
    
    // Load the rigid body colors
    std::vector<renderingutils::Color>& rbcolors = scene_renderer->getRigidBodyColors();
    for( int i = 0; i < (int) rigidbodies.size(); ++i ) rbcolors.push_back( rigidbodies[i].isFixed() ? renderingutils::Color(153./255.,0./255.,26./255.) : renderingutils::Color(0.0,0.0,0.0));
    loadRigidBodyColors( node, rbcolors );
    
    // Generate renderes for forces, etc
    std::vector<OpenGLRenderer*> oglrenderers;
    
    // Create spring renderers
    std::vector<renderingutils::Color> rbspringcolors(rbspringforces.size(),renderingutils::Color(0.0,0.0,1.0));
    loadRigidBodySpringColors( node, rbspringcolors );
    
    for( std::vector<RigidBodySpringForce*>::size_type i = 0; i < rbspringforces.size(); ++i ) 
    {
        oglrenderers.push_back(new OpenGLRigidBodySpringForceRenderer(*rbspringforces[i],rbspringcolors[i]));
    }
    
    // If requested by the user, create a comparison scene
    RigidBodyScene* comparison_scene = NULL;
    if( simulate_comparison ) comparison_scene = new RigidBodyScene(*scene);
    
    // Always want the ability to render svgs
    std::vector<SVGRenderer*> svg_renderers;
    for( std::vector<RigidBodySpringForce*>::size_type i = 0; i < rbspringforces.size(); ++i ) 
    {
        svg_renderers.push_back(new SVGRigidBodySpringForceRenderer(*rbspringforces[i],rbspringcolors[i]));
    }  
    TwoDSceneSVGRenderer* svg_renderer = new TwoDSceneSVGRenderer(display_controller,512,512,bgcolor,rbcolors,svg_renderers);
    
    // Load the rigid body collision detector and response method
    RigidBodyCollisionDetector* collision_detector = NULL;
    RigidBodyCollisionResolver* collision_resolver = NULL;
    loadRigidBodyCollisionHandling( node, &collision_detector, &collision_resolver );
    
    // Create the executable scene
    *execsim = new RigidBodySimulation(scene,comparison_scene,stepper,scene_renderer,oglrenderers,svg_renderer,collision_detector,collision_resolver);
}

void TwoDSceneXMLParser::loadJointMotorBodySimulation( bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, renderingutils::Viewport& view, scalar& dt, renderingutils::Color& bgcolor, rapidxml::xml_node<>* node )
{
    // Create a new rigid body scene
    JointMotorBodyScene* scene = new JointMotorBodyScene;
    
    // Load forces acting on and between rigid bodies
    std::vector<RigidBodySpringForce*> rbspringforces;
    loadRigidBodySpringForces( node, rbspringforces );
    std::vector<RigidBodyGravityForce*> rbgravityforces;
    loadRigidBodyGravityForces( node, rbgravityforces );
    std::vector<RigidBodyWindForce*> rbwindforces;
    loadRigidBodyWindForces( node, rbwindforces );
    
    for( std::vector<RigidBodyGravityForce*>::size_type i = 0; i < rbgravityforces.size(); ++i ) scene->insertForce( rbgravityforces[i] );
    for( std::vector<RigidBodySpringForce*>::size_type i = 0; i < rbspringforces.size(); ++i ) scene->insertForce( rbspringforces[i] );
    for( std::vector<RigidBodyWindForce*>::size_type i = 0; i < rbwindforces.size(); ++i ) scene->insertForce( rbwindforces[i] );
    
    // Load locations and masses of rigid body boundary vertices
    std::vector<Vector2s> rbvrts;
    std::vector<scalar> rbmasses;
    loadRigidBodyVertices( node, rbvrts, rbmasses );
    assert( rbvrts.size() == rbmasses.size() );
    
    // Load the rigid bodies
    std::vector<RigidBody> rigidbodies;
    loadRigidBodies( node, rbvrts, rbmasses, rigidbodies );
    for( std::vector<RigidBody>::size_type i = 0; i < rigidbodies.size(); ++i ) scene->addRigidBody(rigidbodies[i]);
    
    std::vector<JointMotorBody *> jmbs;
    std::vector<RigidBodyForce *> controller_forces;
    loadJointMotorBodies( scene, node, jmbs, rigidbodies, controller_forces );
    for( size_t i = 0; i < jmbs.size(); ++i ) scene->addJointMotorBody(jmbs[i]);
    for( size_t i = 0; i < controller_forces.size(); ++i ) scene->insertForce( controller_forces[i] );
    
    // insert soft springs for countering joint drifts (joint constraints are enforced at velocity level in JointMotorBodySimulation::stepSysteM() )
    for (size_t i = 0; i < jmbs.size(); i++)
    {
        for (int j = 1; j < jmbs[i]->nlinks(); j++)
        {
            int i0 = jmbs[i]->link(j)->rbindex;
            int i1 = jmbs[i]->link(j)->parent->rbindex;
            Vector2s r0 = jmbs[i]->link(j)->axis_in_self_body_space;
            Vector2s r1 = jmbs[i]->link(j)->axis_in_parent_body_space;
            RigidBodySpringForce * sf = new RigidBodySpringForce(100, 0, i0, r0, i1, r1);
            scene->insertForce(sf);
        }
    }
    
    // Load the rigid body integrator
    RigidBodyStepper* stepper = NULL;
    loadRigidBodyIntegrator( node, &stepper, dt );
    
    // Load judge detectors
    JointMotorBodySimulationJudge * judge;
    loadJointMotorBodySimulationDetectors(node, scene, &judge);
    
    // Setup remaining renderers
    TwoDSceneRenderer* scene_renderer = new TwoDSceneRenderer(display_controller);
    
    // Load the rigid body colors
    std::vector<renderingutils::Color>& rbcolors = scene_renderer->getRigidBodyColors();
    for( int i = 0; i < (int) rigidbodies.size(); ++i ) rbcolors.push_back( false ? renderingutils::Color(153./255.,0./255.,26./255.) : renderingutils::Color(0.0,0.0,0.0));
    loadRigidBodyColors( node, rbcolors );
    
    // Load the scheduled push events
    std::vector<JointMotorBodySimulation::ScheduledPushEvent> * pushes = new std::vector<JointMotorBodySimulation::ScheduledPushEvent>;
    loadScheduledPushes(node, scene, *pushes);
    
    // Generate renderes for forces, etc
    std::vector<OpenGLRenderer*> oglrenderers;
    
    // Create spring renderers
    std::vector<renderingutils::Color> rbspringcolors(rbspringforces.size(),renderingutils::Color(0.0,0.0,1.0));
    loadRigidBodySpringColors( node, rbspringcolors );
    
    for( std::vector<RigidBodySpringForce*>::size_type i = 0; i < rbspringforces.size(); ++i ) 
        oglrenderers.push_back(new OpenGLRigidBodySpringForceRenderer(*rbspringforces[i],rbspringcolors[i]));
    
    oglrenderers.push_back(new OpenGLRigidBodyGroundRenderer(renderingutils::Color(0.6, 0.3, 0.1)));
    
    for (size_t i = 0; i < controller_forces.size(); i++)
        if (MonopodRobotController * mrc = dynamic_cast<MonopodRobotController *>(controller_forces[i]))
            oglrenderers.push_back(new OpenGLRigidBodySpringForceRenderer(*(mrc->robot()->legSpringForce()), renderingutils::Color(0.3, 0.5, 0.9)));
    
    for (size_t i = 0; i < pushes->size(); i++)
        oglrenderers.push_back(new OpenGLRigidBodyScheduledImpulseRenderer(renderingutils::Color(1.0, 0.6, 0.1), pushes->at(i)));
    
    // If requested by the user, create a comparison scene
    JointMotorBodyScene* comparison_scene = NULL;
    if( simulate_comparison ) comparison_scene = new JointMotorBodyScene(*scene);
    
    //  // Always want the ability to render svgs
    //  std::vector<SVGRenderer*> svg_renderers;
    //  for( std::vector<RigidBodySpringForce*>::size_type i = 0; i < rbspringforces.size(); ++i ) 
    //  {
    //    svg_renderers.push_back(new SVGRigidBodySpringForceRenderer(*rbspringforces[i],rbspringcolors[i]));
    //  }  
    //  TwoDSceneSVGRenderer* svg_renderer = new TwoDSceneSVGRenderer(display_controller,512,512,bgcolor,rbcolors,svg_renderers);
    
    // Load the rigid body collision detector and response method
    RigidBodyCollisionDetector* collision_detector = NULL;
    loadJointMotorBodyCollisionHandling( node, &collision_detector );
    
    scene->parseScene();
    
    // Create the executable scene
    *execsim = new JointMotorBodySimulation(scene,comparison_scene,stepper,scene_renderer,oglrenderers,collision_detector,judge,pushes);
}

void TwoDSceneXMLParser::loadXMLFile( const std::string& filename, std::vector<char>& xmlchars, rapidxml::xml_document<>& doc )
{
    // Attempt to read the text from the user-specified xml file
    std::string filecontents;
    if( !loadTextFileIntoString(filename,filecontents) )
    {
        std::cerr << "\033[31;1mERROR IN TWODSCENEXMLPARSER:\033[m XML scene file " << filename << ". Failed to read file." << std::endl;
        exit(1);
    }
    
    // Copy string into an array of characters for the xml parser
    for( int i = 0; i < (int) filecontents.size(); ++i ) xmlchars.push_back(filecontents[i]);
    xmlchars.push_back('\0');
    
    // Initialize the xml parser with the character vector
    doc.parse<0>(&xmlchars[0]);
}

bool TwoDSceneXMLParser::loadTextFileIntoString( const std::string& filename, std::string& filecontents )
{
    // Attempt to open the text file for reading
    std::ifstream textfile(filename.c_str(),std::ifstream::in);
    if(!textfile) return false;
    
    // Read the entire file into a single string
    std::string line;
    while(getline(textfile,line)) filecontents.append(line);
    
    textfile.close();
    
    return true;
}

void TwoDSceneXMLParser::loadSimulationType( rapidxml::xml_node<>* node, std::string& simtype )
{
    assert( node != NULL );
    rapidxml::xml_node<>* nd = node->first_node("simtype");
    
    if( node->first_node("simtype") ) if( node->first_node("simtype")->first_attribute("type") ) simtype = node->first_node("simtype")->first_attribute("type")->value();
}

void TwoDSceneXMLParser::loadRigidBodyVertices( rapidxml::xml_node<>* node, std::vector<Vector2s>& vertices, std::vector<scalar>& masses )
{
    assert( node != NULL );
    
    int vertex = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("rigidbodyvertex"); nd; nd = nd->next_sibling("rigidbodyvertex") )
    {
        // Extract the vertex's initial position
        Vector2s pos(0.0,0.0);
        if( nd->first_attribute("x") ) 
        {
            std::string attribute(nd->first_attribute("x")->value());
            if( !stringutils::extractFromString(attribute,pos.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of x attribute for rigidbodyvertex " << vertex << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse x attribute for rigidbodyvertex " << vertex << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( nd->first_attribute("y") ) 
        {
            std::string attribute(nd->first_attribute("y")->value());
            if( !stringutils::extractFromString(attribute,pos.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of y attribute for rigidbodyvertex " << vertex << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse y attribute for rigidbodyvertex " << vertex << ". Exiting." << std::endl;
            exit(1);
        }
        vertices.push_back(pos);
        
        // Extract the particle's mass
        scalar mass = std::numeric_limits<scalar>::signaling_NaN();
        if( nd->first_attribute("m") ) 
        {
            std::string attribute(nd->first_attribute("m")->value());
            if( !stringutils::extractFromString(attribute,mass) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of m attribute for rigidbodyvertex " << vertex << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse m attribute for rigidbodyvertex " << vertex << ". Exiting." << std::endl;
            exit(1);
        }
        masses.push_back(mass);   
        
        //    // Extract the particle's initial velocity
        //    Vector2s vel;
        //    if( nd->first_attribute("vx") ) 
        //    {
        //      std::string attribute(nd->first_attribute("vx")->value());
        //      if( !stringutils::extractFromString(attribute,vel.x()) )
        //      {
        //        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vx attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
        //        exit(1);
        //      }
        //    }
        //    else 
        //    {
        //      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vx attribute for particle " << particle << ". Exiting." << std::endl;
        //      exit(1);
        //    }
        //    
        //    if( nd->first_attribute("vy") ) 
        //    {
        //      std::string attribute(nd->first_attribute("vy")->value());
        //      if( !stringutils::extractFromString(attribute,vel.y()) )
        //      {
        //        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vy attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
        //        exit(1);
        //      }
        //    }
        //    else 
        //    {
        //      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vy attribute for particle " << particle << ". Exiting." << std::endl;
        //      exit(1);
        //    }
        //    twodscene.setVelocity( particle, vel );
        //
        //    // Determine if the particle is fixed
        //    bool fixed;
        //    if( nd->first_attribute("fixed") ) 
        //    {
        //      std::string attribute(nd->first_attribute("fixed")->value());
        //      if( !stringutils::extractFromString(attribute,fixed) )
        //      {
        //        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fixed attribute for particle " << particle << ". Value must be boolean. Exiting." << std::endl;
        //        exit(1);
        //      }
        //    }
        //    else 
        //    {
        //      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fixed attribute for particle " << particle << ". Exiting." << std::endl;
        //      exit(1);
        //    }
        //    twodscene.setFixed( particle, fixed );
        //    
        //    // Extract the particle's radius, if present
        //    scalar radius = 0.1;
        //    if( nd->first_attribute("radius") )
        //    {
        //      std::string attribute(nd->first_attribute("radius")->value());
        //      if( !stringutils::extractFromString(attribute,radius) )
        //      {
        //        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse radius attribute for particle " << particle << ". Value must be scalar. Exiting." << std::endl;
        //        exit(1);
        //      }
        //    }    
        //    twodscene.setRadius( particle, radius );
        //    
        //    // Extract the particle's tag, if present
        //    if( nd->first_attribute("tag") )
        //    {
        //      std::string tag(nd->first_attribute("tag")->value());
        //      tags[particle] = tag;
        //    }    
        //    
        //    //std::cout << "Particle: " << particle << "    x: " << pos.transpose() << "   v: " << vel.transpose() << "   m: " << mass << "   fixed: " << fixed << std::endl;
        //    //std::cout << tags[particle] << std::endl;
        //    
        ++vertex;
    }  
}

void TwoDSceneXMLParser::loadRigidBodies( rapidxml::xml_node<>* node, const std::vector<Vector2s>& vertices, const std::vector<scalar>& masses, std::vector<RigidBody>& rigidbodies )
{
    int body = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("rigidbody"); nd; nd = nd->next_sibling("rigidbody") )
    {
        // Extract the indices that make up this rigid body
        std::vector<int> vrtidxs;
        for( rapidxml::xml_attribute<>* vrtnd = nd->first_attribute("p"); vrtnd; vrtnd = vrtnd->next_attribute("p") )
        {
            vrtidxs.push_back(-1);
            if( !stringutils::extractFromString(vrtnd->value(),vrtidxs.back()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of p attribute for rigidbody " << body << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }
            // Ensure that the requested vertex actually was specified
            if( vrtidxs.back() < 0 || vrtidxs.back() >= (int) masses.size() )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of p attribute for rigidbody " << body << ". Value must be in range [0," << (masses.size()-1) << "] inclusive. Value given of " << vrtidxs.back() << ". Exiting." << std::endl;
                exit(1);
            }
        }
        
        //    // Must have at least 3 indices
        //    if( vrtidxs.size() < 2 ) 
        //    {
        //      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse rigidbody " << body << ". Must specify at least 2 vertices. Exiting." << std::endl;
        //      exit(1);
        //    }
        
        // Extract the radius of the rigid body
        scalar radius = std::numeric_limits<scalar>::signaling_NaN();
        if( nd->first_attribute("r") ) 
        {
            std::string attribute(nd->first_attribute("r")->value());
            if( !stringutils::extractFromString(attribute,radius) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for rigidbody " << body << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
            if( radius <= 0.0 )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for rigidbody " << body << ". Value must be positive. Value give of " << radius << ". Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse r attribute for rigidbody " << body << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the velocity of the rigid body's center of mass
        Vector2s v;
        if( nd->first_attribute("vx") )
        {
            std::string attribute(nd->first_attribute("vx")->value());
            if( !stringutils::extractFromString(attribute,v.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vx attribute for rigidbody " << body << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vx attribute for rigidbody " << body << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( nd->first_attribute("vy") )
        {
            std::string attribute(nd->first_attribute("vy")->value());
            if( !stringutils::extractFromString(attribute,v.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vy attribute for rigidbody " << body << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vy attribute for rigidbody " << body << ". Exiting." << std::endl;
            exit(1);
        }    
        
        // Extract the initial angular velocity
        scalar omega;
        if( nd->first_attribute("omega") )
        {
            std::string attribute(nd->first_attribute("omega")->value());
            if( !stringutils::extractFromString(attribute,omega) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of omega attribute for rigidbody " << body << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse omega attribute for rigidbody " << body << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Determine if the rigid body is fixed
        bool fixed = false;
        if( nd->first_attribute("fixed") )
        {
            std::string attribute(nd->first_attribute("fixed")->value());
            if( !stringutils::extractFromString(attribute,fixed) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fixed attribute for rigidbody " << body << ". Value must be a boolean. Exiting." << std::endl;
                exit(1);
            }      
        }
        
        // Load the rigid body's tag
        std::string tag = "";
        if( nd->first_attribute("tag") )
        {
            tag = std::string(nd->first_attribute("tag")->value());
        }
        
        // Create the rigid body
        VectorXs bodiesverts(2*vrtidxs.size());
        for( std::vector<int>::size_type i = 0; i < vrtidxs.size(); ++i ) bodiesverts.segment<2>(2*i) = vertices[vrtidxs[i]];
        VectorXs bodiesmasses(vrtidxs.size());
        for( std::vector<int>::size_type i = 0; i < vrtidxs.size(); ++i ) bodiesmasses(i) = masses[vrtidxs[i]];
        
        rigidbodies.push_back(RigidBody(v,omega,bodiesverts,bodiesmasses,radius,fixed));
        rigidbodies.back().tag() = tag;
        
        ++body;
    }  
}

void TwoDSceneXMLParser::loadRigidBodyIntegrator( rapidxml::xml_node<>* node, RigidBodyStepper** stepper, scalar& dt )
{
    assert( node != NULL );
    
    dt = -1.0;
    
    // Attempt to locate the integrator node
    rapidxml::xml_node<>* nd = node->first_node("rigidbodyintegrator");
    if( nd == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No rigidbodyintegrator specified. Exiting." << std::endl;
        exit(1);
    }
    
    // Attempt to load the integrator type
    rapidxml::xml_attribute<>* typend = nd->first_attribute("type"); 
    if( typend == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No rigidbodyintegrator 'type' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    std::string integratortype(typend->value());
    
    if( integratortype == "explicit-euler" ) *stepper = new RigidBodyExplicitEuler;
    else if( integratortype == "symplectic-euler" ) *stepper = new RigidBodySymplecticEuler;
    else
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid rigidbodyintegrator 'type' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    
    // Attempt to load the timestep
    rapidxml::xml_attribute<>* dtnd = nd->first_attribute("dt"); 
    if( dtnd == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No rigidbodyintegrator 'dt' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    
    dt = std::numeric_limits<scalar>::signaling_NaN();
    if( !stringutils::extractFromString(std::string(dtnd->value()),dt) )
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'dt' attribute for rigidbodyintegrator. Value must be numeric. Exiting." << std::endl;
        exit(1);
    }  
}

void TwoDSceneXMLParser::loadRigidBodyCollisionHandling( rapidxml::xml_node<>* node, RigidBodyCollisionDetector** collision_detector, RigidBodyCollisionResolver** collision_resolver )
{
    assert( node != NULL );
    assert( *collision_detector == NULL );
    assert( *collision_resolver == NULL );
    
    rapidxml::xml_node<>* nd = node->first_node("rigidbodycollisionhandling");
    if( nd == NULL ) return;
    
    // Load the detection method
    rapidxml::xml_attribute<>* detectionnode = nd->first_attribute("detection"); 
    if( detectionnode == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No rigidbodycollisionhandling 'detection' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    std::string detectiontype(detectionnode->value());
    
    if( detectiontype == "all-pairs" ) *collision_detector = new RigidBodyAllPairsCollisionDetector;
    else if (detectiontype == "ground" ) *collision_detector = new RigidBodyGroundCollisionDetector;
    else
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid rigidbodycollisionhandling 'detection' attribute specified. Valid option are all-pairs, ground. Exiting." << std::endl;
        exit(1);
    }
    
    
    // Load the response method
    rapidxml::xml_attribute<>* responsenode = nd->first_attribute("response"); 
    if( responsenode == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No rigidbodycollisionhandling 'response' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    std::string responsetype(responsenode->value());
    
    if( responsetype == "lcp" ) *collision_resolver = new RigidBodyLCPCollisionResolver;
    else if( responsetype == "velocity-projection" ) *collision_resolver = new RigidBodyVelocityProjectionCollisionResolver;
    else
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid rigidbodycollisionhandling 'response' attribute specified. Valid options are lcp and velocity-projection. Exiting." << std::endl;
        exit(1);
    }
}

void TwoDSceneXMLParser::loadRigidBodySpringForces( rapidxml::xml_node<>* node, std::vector<RigidBodySpringForce*>& rbforces )
{
    assert( node != NULL );
    
    
    // <rigidbodyspring i="0" pix="1.0" piy="0.0" j="1" pjx="-1.0" pjy="0.0" k="1.0" l0="2.0"/>
    int forcenum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("rigidbodyspringforce"); nd; nd = nd->next_sibling("rigidbodyspringforce") )
    {
        // Extract the index of the first rigid body
        int i = -1;
        if( nd->first_attribute("i") )
        {
            std::string attribute(nd->first_attribute("i")->value());
            if( !stringutils::extractFromString(attribute,i) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for rigidbodyspringforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for rigidbodyspringforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the index of the second rigid body
        int j = -1;
        if( nd->first_attribute("j") )
        {
            std::string attribute(nd->first_attribute("j")->value());
            if( !stringutils::extractFromString(attribute,j) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for rigidbodyspringforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for rigidbodyspringforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the location in body coordinates the spring is attached to on the first rigid body
        Vector2s pi(std::numeric_limits<scalar>::signaling_NaN(),std::numeric_limits<scalar>::signaling_NaN());
        if( nd->first_attribute("pix") )
        {
            std::string attribute(nd->first_attribute("pix")->value());
            if( !stringutils::extractFromString(attribute,pi.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of pix attribute for rigidbodyspringforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse pix attribute for rigidbodyspringforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        if( nd->first_attribute("piy") )
        {
            std::string attribute(nd->first_attribute("piy")->value());
            if( !stringutils::extractFromString(attribute,pi.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of piy attribute for rigidbodyspringforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse piy attribute for rigidbodyspringforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the location in body coordinates the spring is attached to on the second rigid body
        Vector2s pj(std::numeric_limits<scalar>::signaling_NaN(),std::numeric_limits<scalar>::signaling_NaN());
        if( nd->first_attribute("pjx") )
        {
            std::string attribute(nd->first_attribute("pjx")->value());
            if( !stringutils::extractFromString(attribute,pj.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of pjx attribute for rigidbodyspringforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse pjx attribute for rigidbodyspringforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        if( nd->first_attribute("pjy") )
        {
            std::string attribute(nd->first_attribute("pjy")->value());
            if( !stringutils::extractFromString(attribute,pj.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of pjy attribute for rigidbodyspringforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse pjy attribute for rigidbodyspringforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the spring stiffness
        scalar k = -1.0;
        if( nd->first_attribute("k") )
        {
            std::string attribute(nd->first_attribute("k")->value());
            if( !stringutils::extractFromString(attribute,k) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of k attribute for rigidbodyspringforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse k attribute for rigidbodyspringforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }    
        
        // Extract the spring rest length
        scalar l0 = -1.0;
        if( nd->first_attribute("l0") )
        {
            std::string attribute(nd->first_attribute("l0")->value());
            if( !stringutils::extractFromString(attribute,l0) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of l0 attribute for rigidbodyspringforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse l0 attribute for rigidbodyspringforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the optional damping coefficient
        scalar b = 0;
        if( nd->first_attribute("b") )
        {
            std::string attribute(nd->first_attribute("b")->value());
            if( !stringutils::extractFromString(attribute,b) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for rigidbodyspringforce " << forcenum << ". Value must be a scalar. Exiting." << std::endl;
                exit(1);
            }
        }
        
        RigidBodySpringForce* newforce = new RigidBodySpringForce( k, l0, i, pi, j, pj, b );
        rbforces.push_back(newforce);
        
        ++forcenum;
    }  
}

void TwoDSceneXMLParser::loadRigidBodyGravityForces( rapidxml::xml_node<>* node, std::vector<RigidBodyGravityForce*>& rbforces )
{
    assert( node != NULL );
    
    int forcenum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("rigidbodygravityforce"); nd; nd = nd->next_sibling("rigidbodygravityforce") )
    {
        Vector2s gforce;
        
        // Extract the x component of the force
        if( nd->first_attribute("fx") )
        {
            std::string attribute(nd->first_attribute("fx")->value());
            if( !stringutils::extractFromString(attribute,gforce.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fx attribute for rigidbodygravityforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fx attribute for rigidbodygravityforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }    
        
        // Extract the spring rest length
        if( nd->first_attribute("fy") )
        {
            std::string attribute(nd->first_attribute("fy")->value());
            if( !stringutils::extractFromString(attribute,gforce.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fy attribute for rigidbodyspringforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fy attribute for rigidbodyspringforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        //std::cout << "Gravity Force: " << gforce.transpose() << std::endl;
        rbforces.push_back(new RigidBodyGravityForce(gforce));
        ++forcenum;
    }
}

void TwoDSceneXMLParser::loadRigidBodyWindForces( rapidxml::xml_node<>* node, std::vector<RigidBodyWindForce*>& rbforces )
{
    assert( node != NULL );
    
    int forcenum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("rigidbodywindforce"); nd; nd = nd->next_sibling("rigidbodywindforce") )
    {
        // Determine how many quadrature points to use per edge
        int pointsperedge = -1;
        if( nd->first_attribute("pointsperedge") )
        {
            std::string attribute(nd->first_attribute("pointsperedge")->value());
            if( !stringutils::extractFromString(attribute,pointsperedge) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of pointsperedge attribute for rigidbodywindforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of pointsperedge attribute for rigidbodywindforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        if( pointsperedge < 2 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of pointsperedge attribute for rigidbodywindforce " << forcenum << ". Must specify at least 2 points per edge." <<  std::endl;
            exit(1);
        }
        
        // Load the scalar strength of the force
        scalar beta = -1.0;
        if( nd->first_attribute("beta") )
        {
            std::string attribute(nd->first_attribute("beta")->value());
            if( !stringutils::extractFromString(attribute,beta) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of beta attribute for rigidbodywindforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse beta attribute for rigidbodywindforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        if( beta < 0.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of beta attribute for rigidbodywindforce " << forcenum << ". Value must be greater or equal to 0." <<  std::endl;
            exit(1);
        }
        
        // Load the wind force itself
        Vector2s force;
        
        // Extract the x component of the force
        if( nd->first_attribute("fx") )
        {
            std::string attribute(nd->first_attribute("fx")->value());
            if( !stringutils::extractFromString(attribute,force.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fx attribute for rigidbodywindforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fx attribute for rigidbodywindforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }    
        
        // Extract the spring rest length
        if( nd->first_attribute("fy") )
        {
            std::string attribute(nd->first_attribute("fy")->value());
            if( !stringutils::extractFromString(attribute,force.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fy attribute for rigidbodywindforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fy attribute for rigidbodywindforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        rbforces.push_back( new RigidBodyWindForce( pointsperedge, beta, force ) );
        
        ++forcenum;
        
        //std::cout << "POINTS PER EDGE: " << pointsperedge << std::endl;
        //std::cout << "BETA: " << beta << std::endl;
        //std::cout << "FORCE: " << force.transpose() << std::endl;    
    }  
}

void TwoDSceneXMLParser::loadRigidBodyColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& rbcolors )
{
    assert( node != NULL );
    
    int colornum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("rigidbodycolor"); nd; nd = nd->next_sibling("rigidbodycolor") )
    {
        // Determine which particle this color corresponds to
        int body = -1;
        if( nd->first_attribute("body") )
        {
            std::string attribute(nd->first_attribute("body")->value());
            if( !stringutils::extractFromString(attribute,body) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of body attribute for rigidbodycolor " << colornum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of body attribute for rigidbodycolor " << colornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( body < 0 || body >= (int) rbcolors.size() )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of body attribute for rigidbodycolor " << colornum << ". Invalid rigid body specified. Valid range is " << 0 << "..." << rbcolors.size()-1 << std::endl;
            exit(1);
        }
        
        // Read in the red color channel 
        if( nd->first_attribute("r") )
        {
            std::string attribute(nd->first_attribute("r")->value());
            if( !stringutils::extractFromString(attribute,rbcolors[colornum].r) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for rigidbodycolor " << colornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for rigidbodycolor " << colornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( rbcolors[colornum].r < 0.0 || rbcolors[colornum].r > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for rigidbodycolor " << colornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        // Read in the green color channel
        if( nd->first_attribute("g") )
        {
            std::string attribute(nd->first_attribute("g")->value());
            if( !stringutils::extractFromString(attribute,rbcolors[colornum].g) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for rigidbodycolor " << colornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for rigidbodycolor " << colornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( rbcolors[colornum].g < 0.0 || rbcolors[colornum].g > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for rigidbodycolor " << colornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        // Read in the blue color channel 
        double blue = -1.0;
        if( nd->first_attribute("b") )
        {
            std::string attribute(nd->first_attribute("b")->value());
            if( !stringutils::extractFromString(attribute,rbcolors[colornum].b) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for rigidbodycolor " << colornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for rigidbodycolor " << colornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( rbcolors[colornum].b < 0.0 || rbcolors[colornum].b > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for rigidbodycolor " << colornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        ++colornum;
    }  
}

void TwoDSceneXMLParser::loadRigidBodySpringColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& colors )
{
    assert( node != NULL );
    
    int colornum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("rigidbodyspringcolor"); nd; nd = nd->next_sibling("rigidbodyspringcolor") )
    {
        // Determine which spring this color corresponds to
        int spring = -1;
        if( nd->first_attribute("spring") )
        {
            std::string attribute(nd->first_attribute("spring")->value());
            if( !stringutils::extractFromString(attribute,spring) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of spring attribute for rigidbodyspringcolor " << colornum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of spring attribute for rigidbodyspringcolor " << colornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( spring < 0 || spring >= (int) colors.size() )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of spring attribute for rigidbodyspringcolor " << colornum << ". Invalid rigid body specified. Valid range is " << 0 << "..." << colors.size()-1 << std::endl;
            exit(1);
        }
        
        // Read in the red color channel 
        if( nd->first_attribute("r") )
        {
            std::string attribute(nd->first_attribute("r")->value());
            if( !stringutils::extractFromString(attribute,colors[spring].r) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for rigidbodyspringcolor " << colornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for rigidbodyspringcolor " << colornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( colors[spring].r < 0.0 || colors[spring].r > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for rigidbodyspringcolor " << colornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        // Read in the green color channel
        if( nd->first_attribute("g") )
        {
            std::string attribute(nd->first_attribute("g")->value());
            if( !stringutils::extractFromString(attribute,colors[spring].g) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for rigidbodyspringcolor " << colornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for rigidbodyspringcolor " << colornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( colors[spring].g < 0.0 || colors[spring].g > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for rigidbodyspringcolor " << colornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        // Read in the blue color channel 
        double blue = -1.0;
        if( nd->first_attribute("b") )
        {
            std::string attribute(nd->first_attribute("b")->value());
            if( !stringutils::extractFromString(attribute,colors[spring].b) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for rigidbodyspringcolor " << colornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for rigidbodyspringcolor " << colornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( colors[spring].b < 0.0 || colors[spring].b > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for rigidbodyspringcolor " << colornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        ++colornum;
    }
}

void TwoDSceneXMLParser::loadJointMotorBodyCollisionHandling( rapidxml::xml_node<>* node, RigidBodyCollisionDetector** collision_detector )
{
    assert( node != NULL );
    assert( *collision_detector == NULL );
    
    rapidxml::xml_node<>* nd = node->first_node("rigidbodycollisionhandling");
    if( nd == NULL ) return;
    
    // Load the detection method
    rapidxml::xml_attribute<>* detectionnode = nd->first_attribute("detection"); 
    if( detectionnode == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No rigidbodycollisionhandling 'detection' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    std::string detectiontype(detectionnode->value());
    
    if( detectiontype == "all-pairs" ) *collision_detector = new RigidBodyAllPairsCollisionDetector;
    else if (detectiontype == "ground" ) *collision_detector = new RigidBodyGroundCollisionDetector;
    else
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid rigidbodycollisionhandling 'detection' attribute specified. Valid option are all-pairs, ground. Exiting." << std::endl;
        exit(1);
    }
    
}

void TwoDSceneXMLParser::loadJointMotorBodies( JointMotorBodyScene * scene, rapidxml::xml_node<>* node, std::vector<JointMotorBody *> & jmbs, const std::vector<RigidBody>& rbs, std::vector<RigidBodyForce *> & controller_forces )
{
    // create a link for each rigid body in the scene
    std::vector<JointMotorBody::Link *> rblinks;
    for (size_t i = 0; i < rbs.size(); i++)
    {
        JointMotorBody::Link * l = new JointMotorBody::Link();
        l->rbindex = i;
        l->parent = NULL;
        rblinks.push_back(l);
    }
    
    // load the joint motor bodies
    int jmbody = 0;
    for( rapidxml::xml_node<>* jmbnode = node->first_node("jointmotorbody"); jmbnode; jmbnode = jmbnode->next_sibling("jointmotorbody"))
    {
        // load jmb type
        std::string type;
        if( jmbnode->first_attribute("type") )
        {
            type = jmbnode->first_attribute("type")->value();
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse type attribute for joint motor body " << jmbody << ". Exiting." << std::endl;
            exit(1);
        }
        
        // load type-specific members
        scalar mr_ls_k, mr_ls_l0, mr_ls_l0min, mr_ls_l0max, mr_ls_b;
        if (type == "monopod-robot")
        {
            rapidxml::xml_node<> * ls_nd = jmbnode->first_node("legspring");
            if( ls_nd->first_attribute("k") )
            {
                std::string attribute(ls_nd->first_attribute("k")->value());
                if( !stringutils::extractFromString(attribute,mr_ls_k) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of k attribute for legspring of monopod robot " << jmbody << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse k attribute for legspring of monopod robot " << jmbody << ". Exiting." << std::endl;
                exit(1);
            }
            
            if( ls_nd->first_attribute("l0") )
            {
                std::string attribute(ls_nd->first_attribute("l0")->value());
                if( !stringutils::extractFromString(attribute,mr_ls_l0) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of l0 attribute for legspring of monopod robot " << jmbody << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse l0 attribute for legspring of monopod robot " << jmbody << ". Exiting." << std::endl;
                exit(1);
            }
            
            if( ls_nd->first_attribute("l0min") )
            {
                std::string attribute(ls_nd->first_attribute("l0min")->value());
                if( !stringutils::extractFromString(attribute,mr_ls_l0min) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of l0min attribute for legspring of monopod robot " << jmbody << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse l0min attribute for legspring of monopod robot " << jmbody << ". Exiting." << std::endl;
                exit(1);
            }
            
            if( ls_nd->first_attribute("l0max") )
            {
                std::string attribute(ls_nd->first_attribute("l0max")->value());
                if( !stringutils::extractFromString(attribute,mr_ls_l0max) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of l0max attribute for legspring of monopod robot " << jmbody << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse l0max attribute for legspring of monopod robot " << jmbody << ". Exiting." << std::endl;
                exit(1);
            }
            
            if( ls_nd->first_attribute("b") )
            {
                std::string attribute(ls_nd->first_attribute("b")->value());
                if( !stringutils::extractFromString(attribute,mr_ls_b) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for legspring of monopod robot " << jmbody << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            }
        }
        
        // load the links
        int link = 0;
        JointMotorBody::Link * root;
        for( rapidxml::xml_node<>* nd = jmbnode->first_node("link"); nd; nd = nd->next_sibling("link") )
        {
            // Extract the corresponding rigid body index
            int body = 0;
            if( nd->first_attribute("body") )
            {
                std::string attribute(nd->first_attribute("body")->value());
                if( !stringutils::extractFromString(attribute,body) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of body attribute for link " << link << " of joint motor body " << jmbody << ". Value must be an integer. Exiting." << std::endl;
                    exit(1);
                }      
            }
            else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse body attribute for link " << link << " of joint motor body " << jmbody << ". Exiting." << std::endl;
                exit(1);
            }
            assert(body >= 0 && body < (int)rblinks.size());
            
            // Extract the parent
            int parent = 0;
            if( nd->first_attribute("parent") )
            {
                std::string attribute(nd->first_attribute("parent")->value());
                if( !stringutils::extractFromString(attribute,parent) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of parent attribute for link " << link << " of joint motor body " << jmbody << ". Value must be an integer. Exiting." << std::endl;
                    exit(1);
                }      
            }
            else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse parent attribute for link " << link << " of joint motor body " << jmbody << ". Exiting." << std::endl;
                exit(1);
            }
            assert(parent >= -1 && parent < (int)rblinks.size());
            
            // Extract the joint axis
            Vector2s axis;
            if( nd->first_attribute("axisx") )
            {
                std::string attribute(nd->first_attribute("axisx")->value());
                if( !stringutils::extractFromString(attribute,axis.x()) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of axisx attribute for link " << link << " of joint motor body " << jmbody << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }      
            }
            else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse axisx attribute for link " << link << " of joint motor body " << jmbody << ". Exiting." << std::endl;
                exit(1);
            }
            if( nd->first_attribute("axisy") )
            {
                std::string attribute(nd->first_attribute("axisy")->value());
                if( !stringutils::extractFromString(attribute,axis.y()) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of axisx attribute for link " << link << " of joint motor body " << jmbody << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }      
            }
            else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse axisx attribute for link " << link << " of joint motor body " << jmbody << ". Exiting." << std::endl;
                exit(1);
            }
            
            // initialize the joint motor link
            JointMotorBody::Link * l = rblinks[body];
            if (parent >= 0)
            {
                l->parent = rblinks[parent];
                rblinks[parent]->children.push_back(l);
            } else
            {
                root = l;
            }
            l->axis = axis;
            assert(!rbs[body].isFixed());
            
            link++;
        }
        
        // create the joint motor body
        if (type == "monopod-robot")
        {
            assert(root);
            assert(root->children.size() > 0);
            MonopodRobot * mr = new MonopodRobot(scene, root, root->children[0], mr_ls_k, mr_ls_l0, mr_ls_l0min, mr_ls_l0max, mr_ls_b);
            jmbs.push_back(mr);
            
            MonopodRobotController * mrc = new MonopodRobotController(scene, mr);
            controller_forces.push_back(mrc);
        } else
        {
            JointMotorBody * jmb = new JointMotorBody(scene);
            jmb->addLink(root, NULL);
            jmb->buildLinkSerialization();
            jmb->computeAxisVectors();      
            jmbs.push_back(jmb);
            
            JointMotorBodyController * jmbc = new JointMotorBodyController(scene, jmb);
            controller_forces.push_back(jmbc);
        }
        
        jmbody++;
        
    }
    
}

void TwoDSceneXMLParser::loadJointMotorBodySimulationDetectors( rapidxml::xml_node<>* node, JointMotorBodyScene * scene, JointMotorBodySimulationJudge ** judge )
{
    *judge = new JointMotorBodySimulationJudge();
    
    int detector = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("detector"); nd; nd = nd->next_sibling("detector") )
    {
        // Extract the corresponding rigid body index
        std::string type;
        if( nd->first_attribute("type") )
        {
            std::string attribute(nd->first_attribute("type")->value());
            type = attribute;
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse type attribute for detector " << detector << ". Exiting." << std::endl;
            exit(1);
        }
        
        int jmb;
        if( nd->first_attribute("jointmotorbody") )
        {
            std::string attribute(nd->first_attribute("jointmotorbody")->value());
            if( !stringutils::extractFromString(attribute,jmb) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of jointmotorbody attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                exit(1);
            }
        } else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse jointmotorbody attribute for detector " << detector << ". Exiting." << std::endl;
            exit(1);
        }    
        
        if (type == "mr-min-jump-height")
        {
            scalar ymin, ymax;
            if( nd->first_attribute("ymin") )
            {
                std::string attribute(nd->first_attribute("ymin")->value());
                if( !stringutils::extractFromString(attribute,ymin) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of ymin attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse ymin attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            if( nd->first_attribute("ymax") )
            {
                std::string attribute(nd->first_attribute("ymax")->value());
                if( !stringutils::extractFromString(attribute,ymax) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of ymax attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse ymax attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            
            int njump;
            if( nd->first_attribute("njump") )
            {
                std::string attribute(nd->first_attribute("njump")->value());
                if( !stringutils::extractFromString(attribute,njump) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of njump attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse njump attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            
            MonopodRobot * robot = dynamic_cast<MonopodRobot *>(scene->getJointMotorBody(jmb));
            assert(robot);
            
            (*judge)->addDetector(new JointMotorBodySimulationJudge::MR_JumpingDetector(detector, robot, ymin, ymax, njump));
            
        } else if (type == "mr-land-in-specified-zone")
        {
            scalar xmin, xmax, duration;
            if( nd->first_attribute("xmin") )
            {
                std::string attribute(nd->first_attribute("xmin")->value());
                if( !stringutils::extractFromString(attribute,xmin) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of xmin attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse xmin attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            if( nd->first_attribute("xmax") )
            {
                std::string attribute(nd->first_attribute("xmax")->value());
                if( !stringutils::extractFromString(attribute,xmax) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of xmax attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse xmax attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            if( nd->first_attribute("duration") )
            {
                std::string attribute(nd->first_attribute("duration")->value());
                if( !stringutils::extractFromString(attribute,duration) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of duration attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse duration attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            
            MonopodRobot * robot = dynamic_cast<MonopodRobot *>(scene->getJointMotorBody(jmb));
            assert(robot);
            
            (*judge)->addDetector(new JointMotorBodySimulationJudge::MR_LandingDetector(detector, robot, xmin, xmax, duration));
            
        } else if (type == "mr-balance")
        {
            scalar duration;
            if( nd->first_attribute("duration") )
            {
                std::string attribute(nd->first_attribute("duration")->value());
                if( !stringutils::extractFromString(attribute,duration) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of duration attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse duration attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            
            MonopodRobot * robot = dynamic_cast<MonopodRobot *>(scene->getJointMotorBody(jmb));
            assert(robot);
            
            (*judge)->addDetector(new JointMotorBodySimulationJudge::MR_BalancingDetector(detector, robot, duration));
            
        } else if (type == "mr-velocity")
        {
            scalar vmin, vmax, duration;
            if( nd->first_attribute("vmin") )
            {
                std::string attribute(nd->first_attribute("vmin")->value());
                if( !stringutils::extractFromString(attribute,vmin) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vmin attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vmin attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            if( nd->first_attribute("vmax") )
            {
                std::string attribute(nd->first_attribute("vmax")->value());
                if( !stringutils::extractFromString(attribute,vmax) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vmax attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vmax attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            if( nd->first_attribute("duration") )
            {
                std::string attribute(nd->first_attribute("duration")->value());
                if( !stringutils::extractFromString(attribute,duration) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of duration attribute for detector " << detector << ". Value must be a scalar. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse duration attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            
            MonopodRobot * robot = dynamic_cast<MonopodRobot *>(scene->getJointMotorBody(jmb));
            assert(robot);
            
            (*judge)->addDetector(new JointMotorBodySimulationJudge::MR_VelocityDetector(detector, robot, vmin, vmax, duration));
            
        } else if (type == "mr-treat")
        {
            int treatrb;
            if( nd->first_attribute("treatrb") )
            {
                std::string attribute(nd->first_attribute("treatrb")->value());
                if( !stringutils::extractFromString(attribute,treatrb) )
                {
                    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of treatrb attribute for detector " << detector << ". Value must be an integer. Exiting." << std::endl;
                    exit(1);
                }
            } else 
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse treatrb attribute for detector " << detector << ". Exiting." << std::endl;
                exit(1);
            }
            
            MonopodRobot * robot = dynamic_cast<MonopodRobot *>(scene->getJointMotorBody(jmb));
            assert(robot);
            
            (*judge)->addDetector(new JointMotorBodySimulationJudge::MR_TreatDetector(detector, robot, treatrb));
            
        }
        
        detector++;
        
    }    
    
}

void TwoDSceneXMLParser::loadScheduledPushes( rapidxml::xml_node<> * node, JointMotorBodyScene * scene, std::vector<JointMotorBodySimulation::ScheduledPushEvent> & pushes )
{
    pushes.clear();
    
    int pushevent = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("scheduledimpulse"); nd; nd = nd->next_sibling("scheduledimpulse") )
    {
        JointMotorBodySimulation::ScheduledPushEvent e;
        
        if( nd->first_attribute("ix") ) 
        {
            std::string attribute(nd->first_attribute("ix")->value());
            if( !stringutils::extractFromString(attribute,e.impulse.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of ix attribute for scheduled impulse event " << pushevent << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse ix attribute for scheduled impulse event " << pushevent << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( nd->first_attribute("iy") ) 
        {
            std::string attribute(nd->first_attribute("iy")->value());
            if( !stringutils::extractFromString(attribute,e.impulse.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of iy attribute for scheduled impulse event " << pushevent << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse iy attribute for scheduled impulse event " << pushevent << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( nd->first_attribute("rigidbody") ) 
        {
            std::string attribute(nd->first_attribute("rigidbody")->value());
            if( !stringutils::extractFromString(attribute,e.rb) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of rigidbody attribute for scheduled impulse event " << pushevent << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse rigidbody attribute for scheduled impulse event " << pushevent << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( nd->first_attribute("t") ) 
        {
            std::string attribute(nd->first_attribute("t")->value());
            if( !stringutils::extractFromString(attribute,e.time) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of t attribute for scheduled impulse event " << pushevent << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse t attribute for scheduled impulse event " << pushevent << ". Exiting." << std::endl;
            exit(1);
        }
        
        e.alpha = 0.0;
        
        pushes.push_back(e);
    }
    
}








void TwoDSceneXMLParser::loadParticles( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
    assert( node != NULL );
    
    // Count the number of particles
    int numparticles = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("particle"); nd; nd = nd->next_sibling("particle") ) ++numparticles;
    
    twodscene.resizeSystem(numparticles);
    
    //std::cout << "Num particles " << numparticles << std::endl;
    
    std::vector<std::string>& tags = twodscene.getParticleTags();
    
    int particle = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("particle"); nd; nd = nd->next_sibling("particle") )
    {
        // Extract the particle's initial position
        Vector2s pos;
        if( nd->first_attribute("px") ) 
        {
            std::string attribute(nd->first_attribute("px")->value());
            if( !stringutils::extractFromString(attribute,pos.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of px attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse px attribute for particle " << particle << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( nd->first_attribute("py") ) 
        {
            std::string attribute(nd->first_attribute("py")->value());
            if( !stringutils::extractFromString(attribute,pos.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of py attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse py attribute for particle " << particle << ". Exiting." << std::endl;
            exit(1);
        }
        twodscene.setPosition( particle, pos );
        
        // Extract the particle's initial velocity
        Vector2s vel;
        if( nd->first_attribute("vx") ) 
        {
            std::string attribute(nd->first_attribute("vx")->value());
            if( !stringutils::extractFromString(attribute,vel.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vx attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vx attribute for particle " << particle << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( nd->first_attribute("vy") ) 
        {
            std::string attribute(nd->first_attribute("vy")->value());
            if( !stringutils::extractFromString(attribute,vel.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vy attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vy attribute for particle " << particle << ". Exiting." << std::endl;
            exit(1);
        }
        twodscene.setVelocity( particle, vel );
        
        // Extract the particle's mass
        scalar mass = std::numeric_limits<scalar>::signaling_NaN();
        if( nd->first_attribute("m") ) 
        {
            std::string attribute(nd->first_attribute("m")->value());
            if( !stringutils::extractFromString(attribute,mass) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of m attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse m attribute for particle " << particle << ". Exiting." << std::endl;
            exit(1);
        }
        twodscene.setMass( particle, mass );
        
        // Determine if the particle is fixed
        bool fixed;
        if( nd->first_attribute("fixed") ) 
        {
            std::string attribute(nd->first_attribute("fixed")->value());
            if( !stringutils::extractFromString(attribute,fixed) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fixed attribute for particle " << particle << ". Value must be boolean. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fixed attribute for particle " << particle << ". Exiting." << std::endl;
            exit(1);
        }
        twodscene.setFixed( particle, fixed );
        
        // Extract the particle's radius, if present
        scalar radius = 0.1;
        if( nd->first_attribute("radius") )
        {
            std::string attribute(nd->first_attribute("radius")->value());
            if( !stringutils::extractFromString(attribute,radius) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse radius attribute for particle " << particle << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }
        }    
        twodscene.setRadius( particle, radius );
        
        // Extract the particle's tag, if present
        if( nd->first_attribute("tag") )
        {
            std::string tag(nd->first_attribute("tag")->value());
            tags[particle] = tag;
        }    
        
        //std::cout << "Particle: " << particle << "    x: " << pos.transpose() << "   v: " << vel.transpose() << "   m: " << mass << "   fixed: " << fixed << std::endl;
        //std::cout << tags[particle] << std::endl;
        
        ++particle;
    }
}

void TwoDSceneXMLParser::loadSceneTag( rapidxml::xml_node<>* node, std::string& scenetag )
{
    assert( node != NULL );
    
    if( node->first_node("scenetag") )
    {
        if( node->first_node("scenetag")->first_attribute("tag") )
        {
            scenetag = node->first_node("scenetag")->first_attribute("tag")->value();
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of tag attribute for scenetag. Value must be string. Exiting." << std::endl;
            exit(1);
        }
    }
}

void TwoDSceneXMLParser::loadHalfplanes( rapidxml::xml_node<> *node, TwoDScene &twodscene)
{
    assert( node != NULL );
    
    twodscene.clearHalfplanes();
    
    int halfplane = 0;
    for( rapidxml::xml_node<> *nd = node->first_node("halfplane"); nd; nd = nd->next_sibling("halfplane") )
    {
        VectorXs x(2);
        VectorXs n(2);
        
        if( nd->first_attribute("px") )
        {
            std::string attribute(nd->first_attribute("px")->value());
            if( !stringutils::extractFromString(attribute, x[0]) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of px attribute for halfplane " << halfplane << ". Value must be real number. Exiting." << std::endl;
                exit(0);
            }
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of px attribute for halfplane " << halfplane << ". Exiting." << std::endl;
            exit(0);
        }
        if( nd->first_attribute("py") )
        {
            std::string attribute(nd->first_attribute("py")->value());
            if( !stringutils::extractFromString(attribute, x[1]) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of py attribute for halfplane " << halfplane << ". Value must be real number. Exiting." << std::endl;
                exit(0);
            }
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of py attribute for halfplane " << halfplane << ". Exiting." << std::endl;
            exit(0);
        }
        if( nd->first_attribute("nx") )
        {
            std::string attribute(nd->first_attribute("nx")->value());
            if( !stringutils::extractFromString(attribute, n[0]) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of nx attribute for halfplane " << halfplane << ". Value must be real number. Exiting." << std::endl;
                exit(0);
            }
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of nx attribute for halfplane " << halfplane << ". Exiting." << std::endl;
            exit(0);
        }
        if( nd->first_attribute("ny") )
        {
            std::string attribute(nd->first_attribute("ny")->value());
            if( !stringutils::extractFromString(attribute, n[1]) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of ny attribute for halfplane " << halfplane << ". Value must be real number. Exiting." << std::endl;
                exit(0);
            }
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of ny attribute for halfplane " << halfplane << ". Exiting." << std::endl;
            exit(0);
        }
        twodscene.insertHalfplane(std::pair<VectorXs, VectorXs>(x,n));
        
        ++halfplane;
    }
}

void TwoDSceneXMLParser::loadEdges( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
    assert( node != NULL );
    
    twodscene.clearEdges();
    
    int edge = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("edge"); nd; nd = nd->next_sibling("edge") )
    {
        std::pair<int,int> newedge;
        if( nd->first_attribute("i") )
        {
            std::string attribute(nd->first_attribute("i")->value());
            if( !stringutils::extractFromString(attribute,newedge.first) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for edge " << edge << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for edge " << edge << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( nd->first_attribute("j") )
        {
            std::string attribute(nd->first_attribute("j")->value());
            if( !stringutils::extractFromString(attribute,newedge.second) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for edge " << edge << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for edge " << edge << ". Exiting." << std::endl;
            exit(1);
        }
        
        scalar radius = 0.015;
        if( nd->first_attribute("radius") )
        {
            std::string attribute(nd->first_attribute("radius")->value());
            if( !stringutils::extractFromString(attribute,radius) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse radius attribute for edge " << edge << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }
        }
        
        //std::cout << "Edge: " << edge << "    i: " << newedge.first << "   j: " << newedge.second << std::endl;
        
        twodscene.insertEdge( newedge, radius );
        
        ++edge;
    }
}

void TwoDSceneXMLParser::loadSpringForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
    assert( node != NULL );
    
    // Extract the edge the force acts across
    int forcenum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("springforce"); nd; nd = nd->next_sibling("springforce") )
    {
        int edge = -1;
        
        if( nd->first_attribute("edge") )
        {
            std::string attribute(nd->first_attribute("edge")->value());
            if( !stringutils::extractFromString(attribute,edge) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of edge attribute for springforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of edge attribute for springforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }      
        
        std::pair<int,int> newedge(twodscene.getEdge(edge));
        
        // Extract the spring stiffness
        scalar k = std::numeric_limits<scalar>::signaling_NaN();
        if( nd->first_attribute("k") ) 
        {
            std::string attribute(nd->first_attribute("k")->value());
            if( !stringutils::extractFromString(attribute,k) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of k attribute for springforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse k attribute for springforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the spring rest length
        scalar l0 = std::numeric_limits<scalar>::signaling_NaN();
        if( nd->first_attribute("l0") ) 
        {
            std::string attribute(nd->first_attribute("l0")->value());
            if( !stringutils::extractFromString(attribute,l0) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of l0 attribute for springforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse l0 attribute for springforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the optional damping coefficient
        scalar b = 0.0;
        if( nd->first_attribute("b") ) 
        {
            std::string attribute(nd->first_attribute("b")->value());
            if( !stringutils::extractFromString(attribute,b) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for springforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        
        //std::cout << "Springforce: " << forcenum << "    i: " << newedge.first << "   j: " << newedge.second << "   k: " << k << "   l0: " << l0 << std::endl;
        
        twodscene.insertForce(new SpringForce(newedge,k,l0,b));
        
        ++forcenum;
    }
    
    //SpringForce( const std::pair<int,int>& endpoints, const scalar& k, const scalar& l0 )
}

void TwoDSceneXMLParser::loadGravitationalForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
    assert( node != NULL );
    
    // Extract the edge the force acts across
    int forcenum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("gravitationalforce"); nd; nd = nd->next_sibling("gravitationalforce") )
    {
        std::pair<int,int> newedge;
        if( nd->first_attribute("i") )
        {
            std::string attribute(nd->first_attribute("i")->value());
            if( !stringutils::extractFromString(attribute,newedge.first) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for gravitationalforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for gravitationalforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( nd->first_attribute("j") )
        {
            std::string attribute(nd->first_attribute("j")->value());
            if( !stringutils::extractFromString(attribute,newedge.second) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for gravitationalforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for gravitationalforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the gravitational constant
        scalar G = std::numeric_limits<scalar>::signaling_NaN();
        if( nd->first_attribute("G") )
        {
            std::string attribute(nd->first_attribute("G")->value());
            if( !stringutils::extractFromString(attribute,G) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of G attribute for gravitationalforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse G attribute for gravitationalforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        //std::cout << "GravitationalForce: " << forcenum << "    i: " << newedge.first << "   j: " << newedge.second << "   G: " << G << std::endl;
        
        twodscene.insertForce(new GravitationalForce(newedge,G));
        
        ++forcenum;
    }
    
    //SpringForce( const std::pair<int,int>& endpoints, const scalar& k, const scalar& l0 )  
}


void TwoDSceneXMLParser::loadConstantForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
    assert( node != NULL );
    
    // Load each constant force
    int forcenum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("constantforce"); nd; nd = nd->next_sibling("constantforce") )
    {
        Vector2s constforce;
        constforce.setConstant(std::numeric_limits<scalar>::signaling_NaN());
        
        // Extract the x component of the force
        if( nd->first_attribute("fx") ) 
        {
            std::string attribute(nd->first_attribute("fx")->value());
            if( !stringutils::extractFromString(attribute,constforce.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fx attribute for constantforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fx attribute for constantforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the y component of the force
        if( nd->first_attribute("fy") ) 
        {
            std::string attribute(nd->first_attribute("fy")->value());
            if( !stringutils::extractFromString(attribute,constforce.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fy attribute for constantforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fy attribute for constantforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        //std::cout << "x: " << constforce.transpose() << std::endl;
        
        twodscene.insertForce(new ConstantForce(constforce));
        
        ++forcenum;
    }
}

void TwoDSceneXMLParser::loadDragDampingForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
    assert( node != NULL );
    
    int forcenum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("dragdamping"); nd; nd = nd->next_sibling("dragdamping") )
    {
        Vector2s constforce;
        constforce.setConstant(std::numeric_limits<scalar>::signaling_NaN());
        
        // Extract the linear damping coefficient
        scalar b = std::numeric_limits<scalar>::signaling_NaN();
        if( nd->first_attribute("b") )
        {
            std::string attribute(nd->first_attribute("b")->value());
            if( !stringutils::extractFromString(attribute,b) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for dragdamping " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse b attribute for dragdamping " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        //std::cout << "x: " << constforce.transpose() << std::endl;
        
        twodscene.insertForce(new DragDampingForce(b));
        
        ++forcenum;
    }  
}

void TwoDSceneXMLParser::loadVorexForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
    assert( node != NULL );
    
    // Extract the edge the force acts across
    int forcenum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("vortexforce"); nd; nd = nd->next_sibling("vortexforce") )
    {
        std::pair<int,int> newedge;
        if( nd->first_attribute("i") )
        {
            std::string attribute(nd->first_attribute("i")->value());
            if( !stringutils::extractFromString(attribute,newedge.first) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for vortexforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for vortexforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( nd->first_attribute("j") )
        {
            std::string attribute(nd->first_attribute("j")->value());
            if( !stringutils::extractFromString(attribute,newedge.second) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for vortexforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for vortexforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the 'Biot-Savart' constant
        scalar kbs = std::numeric_limits<scalar>::signaling_NaN();
        if( nd->first_attribute("kbs") )
        {
            std::string attribute(nd->first_attribute("kbs")->value());
            if( !stringutils::extractFromString(attribute,kbs) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of kbs attribute for vortexforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse kbs attribute for vortexforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the viscosity constant
        scalar kvc = std::numeric_limits<scalar>::signaling_NaN();
        if( nd->first_attribute("kvc") )
        {
            std::string attribute(nd->first_attribute("kvc")->value());
            if( !stringutils::extractFromString(attribute,kvc) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of kvc attribute for vortexforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse kvc attribute for vortexforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }    
        
        //std::cout << "VortexForce: " << forcenum << "    i: " << newedge.first << "   j: " << newedge.second << "   kbs: " << kbs << "   kvc: " << kvc << std::endl;
        
        twodscene.insertForce(new VortexForce(newedge,kbs,kvc));
        
        ++forcenum;
    }
}

void TwoDSceneXMLParser::loadSimpleGravityForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
    assert( node != NULL );
    
    // Load each constant force
    int forcenum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("simplegravity"); nd; nd = nd->next_sibling("simplegravity") )
    {
        Vector2s constforce;
        constforce.setConstant(std::numeric_limits<scalar>::signaling_NaN());
        
        // Extract the x component of the force
        if( nd->first_attribute("fx") ) 
        {
            std::string attribute(nd->first_attribute("fx")->value());
            if( !stringutils::extractFromString(attribute,constforce.x()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fx attribute for constantforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fx attribute for constantforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        // Extract the y component of the force
        if( nd->first_attribute("fy") ) 
        {
            std::string attribute(nd->first_attribute("fy")->value());
            if( !stringutils::extractFromString(attribute,constforce.y()) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fy attribute for constantforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        else 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fy attribute for constantforce " << forcenum << ". Exiting." << std::endl;
            exit(1);
        }
        
        //std::cout << "x: " << constforce.transpose() << std::endl;
        
        twodscene.insertForce(new SimpleGravityForce(constforce));
        
        ++forcenum;
    }  
}

void TwoDSceneXMLParser::loadCollisionDetector( rapidxml::xml_node<> *node, TwoDScene &scene, CollisionDetector **detector )
{
    assert(node != NULL);
    
    rapidxml::xml_node<> *nd = node->first_node("collisiondetection");
    if( nd == NULL )
    {
        *detector = new AllPairsDetector();
        return;
    }
    
    // Load value of type
    rapidxml::xml_attribute<> *typend = nd->first_attribute("type");
    if(typend == NULL)
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse type attribute for collisiondetector node. Exiting." << std::endl;
        exit(1);
    }
    std::string type = typend->value();
    if(type == "allpairs")
        *detector = new AllPairsDetector();
    else if(type == "contest")
        *detector = new ContestDetector();
    else
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid type attribute in collisiondetector node. Exiting." << std::endl;
        exit(1);
    }
}

void TwoDSceneXMLParser::loadCollisionHandler( rapidxml::xml_node<>* node, TwoDScene &scene, CollisionHandler **handler, CollisionDetector &detector )
{
    assert( node != NULL );
    
    rapidxml::xml_node<>* nd = node->first_node("collision");
    if( nd == NULL )
    {
        *handler = NULL;
        return;
    }
    
    // Load value of COR
    double cor = 1.0;
    rapidxml::xml_attribute<> *cornd = nd->first_attribute("COR");
    if( cornd != NULL )
    { 
        if( !stringutils::extractFromString(std::string(cornd->value()),cor) )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'COR' attribute for collision handler. Value must be numeric. Exiting." << std::endl;
            exit(1);
        }
    }
    
    // Attempt to load the collision handler type
    rapidxml::xml_attribute<> *typend = nd->first_attribute("type");
    if( typend == NULL )
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No collision handler 'type' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    std::string handlertype(typend->value());
    if(handlertype == "none") *handler = NULL;
    else if(handlertype == "simple") *handler = new SimpleCollisionHandler(cor);
    else if(handlertype == "continuous-time") *handler = new ContinuousTimeCollisionHandler(cor);
    else if(handlertype == "hybrid")
    {
        // Load max iteration attribute
        double maxiters=10;
        rapidxml::xml_attribute<> *maxiternd = nd->first_attribute("maxiters");
        if(maxiternd != NULL)
        {
            if(!stringutils::extractFromString(std::string(maxiternd->value()), maxiters))
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'maxiters' attribute for collision handler. Value must be numeric. Exiting."<< std::endl;
                exit(1);
            }
        }
        *handler = new HybridCollisionHandler(maxiters, cor);
        
        // Attempt to load stiffness attribute
        double k=10;
        rapidxml::xml_attribute<> *knd = nd->first_attribute("k");
        if(knd != NULL)
        {
            if(!stringutils::extractFromString(std::string(knd->value()), k))
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'k' attribute for collision handler. Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        
        double thickness=0.1;
        rapidxml::xml_attribute<> *thicknd = nd->first_attribute("thickness");
        if(thicknd != NULL)
        {
            if(!stringutils::extractFromString(std::string(thicknd->value()), thickness))
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'thickness' attribute for collision handler. Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        scene.insertForce(new PenaltyForce(scene, detector, k, thickness));
    }
    else if(handlertype == "penalty")
    {
        // Attempt to load stiffness attribute
        double k=100;
        rapidxml::xml_attribute<> *knd = nd->first_attribute("k");
        if(knd != NULL)
        {
            if(!stringutils::extractFromString(std::string(knd->value()), k))
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'k' attribute for collision handler. Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        
        double thickness=0.01;
        rapidxml::xml_attribute<> *thicknd = nd->first_attribute("thickness");
        if(thicknd != NULL)
        {
            if(!stringutils::extractFromString(std::string(thicknd->value()), thickness))
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'thickness' attribute for collision handler. Value must be numeric. Exiting." << std::endl;
                exit(1);
            }
        }
        *handler = NULL;
        scene.insertForce(new PenaltyForce(scene, detector, k, thickness));
    }
    else
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid collision handler 'type' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    
}

void TwoDSceneXMLParser::loadIntegrator( rapidxml::xml_node<>* node, SceneStepper** scenestepper, scalar& dt )
{
    assert( node != NULL );
    
    dt = -1.0;
    
    // Attempt to locate the integrator node
    rapidxml::xml_node<>* nd = node->first_node("integrator");
    if( nd == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No integrator specified. Exiting." << std::endl;
        exit(1);
    }
    
    // Attempt to load the integrator type
    rapidxml::xml_attribute<>* typend = nd->first_attribute("type"); 
    if( typend == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No integrator 'type' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    std::string integratortype(typend->value());
    
    if( integratortype == "explicit-euler" ) *scenestepper = new ExplicitEuler;
    else if( integratortype == "forward-backward-euler" ) *scenestepper = new SemiImplicitEuler;
    else if( integratortype == "implicit-euler" ) *scenestepper = new ImplicitEuler;
    else if( integratortype == "linearized-implicit-euler" ) *scenestepper = new LinearizedImplicitEuler;
    else
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid integrator 'type' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    
    // Attempt to load the timestep
    rapidxml::xml_attribute<>* dtnd = nd->first_attribute("dt"); 
    if( dtnd == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No integrator 'dt' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    
    dt = std::numeric_limits<scalar>::signaling_NaN();
    if( !stringutils::extractFromString(std::string(dtnd->value()),dt) )
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'dt' attribute for integrator. Value must be numeric. Exiting." << std::endl;
        exit(1);
    }
    
    //std::cout << "Integrator: " << (*scenestepper)->getName() << "   dt: " << dt << std::endl;
}

void TwoDSceneXMLParser::loadMaxTime( rapidxml::xml_node<>* node, scalar& max_t )
{
    assert( node != NULL );
    
    // Attempt to locate the duraiton node
    rapidxml::xml_node<>* nd = node->first_node("duration");
    if( nd == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No duration specified. Exiting." << std::endl;
        exit(1);
    }
    
    // Attempt to load the duration value
    rapidxml::xml_attribute<>* timend = nd->first_attribute("time"); 
    if( timend == NULL ) 
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No duration 'time' attribute specified. Exiting." << std::endl;
        exit(1);
    }
    
    max_t = std::numeric_limits<scalar>::signaling_NaN();
    if( !stringutils::extractFromString(std::string(timend->value()),max_t) )
    {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'time' attribute for duration. Value must be numeric. Exiting." << std::endl;
        exit(1);
    }
}

void TwoDSceneXMLParser::loadViewport(rapidxml::xml_node<>* node, renderingutils::Viewport &view)
{
    assert( node != NULL );
    
    if(node->first_node("viewport") )
    {
        rapidxml::xml_attribute<> *cx = node->first_node("viewport")->first_attribute("cx");
        if(cx == NULL)
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No viewport 'cx' attribute specified. Exiting." << std::endl;
            exit(1);
        }
        if(!stringutils::extractFromString(std::string(cx->value()),view.cx))
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'cx' attribute for viewport. Value must be scalar. Exiting." << std::endl;
            exit(1);
        }
        rapidxml::xml_attribute<> *cy = node->first_node("viewport")->first_attribute("cy");
        if(cy == NULL)
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No viewport 'cy' attribute specified. Exiting." << std::endl;
            exit(1);
        }
        if(!stringutils::extractFromString(std::string(cy->value()),view.cy))
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'cy' attribute for viewport. Value must be scalar. Exiting." << std::endl;
            exit(1);
        }
        rapidxml::xml_attribute<> *size = node->first_node("viewport")->first_attribute("size");
        if(size == NULL)
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No viewport 'size' attribute specified. Exiting." << std::endl;
            exit(1);
        }
        if(!stringutils::extractFromString(std::string(size->value()),view.size))
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'size' attribute for viewport. Value must be scalar. Exiting." << std::endl;
            exit(1);
        }
    }
}

void TwoDSceneXMLParser::loadMaxSimFrequency( rapidxml::xml_node<>* node, scalar& max_freq )
{
    assert( node != NULL );
    
    // Attempt to locate the duraiton node
    if( node->first_node("maxsimfreq") ) 
    {
        // Attempt to load the duration value
        rapidxml::xml_attribute<>* atrbnde = node->first_node("maxsimfreq")->first_attribute("max"); 
        if( atrbnde == NULL ) 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No maxsimfreq 'max' attribute specified. Exiting." << std::endl;
            exit(1);
        }
        
        if( !stringutils::extractFromString(std::string(atrbnde->value()),max_freq) )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'max' attribute for maxsimfreq. Value must be scalar. Exiting." << std::endl;
            exit(1);
        }
    }
}




void TwoDSceneXMLParser::loadBackgroundColor( rapidxml::xml_node<>* node, renderingutils::Color& color )
{
    if( rapidxml::xml_node<>* nd = node->first_node("backgroundcolor") )
    {
        // Read in the red color channel 
        double red = -1.0;
        if( nd->first_attribute("r") )
        {
            std::string attribute(nd->first_attribute("r")->value());
            if( !stringutils::extractFromString(attribute,red) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for backgroundcolor. Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for backgroundcolor. Exiting." << std::endl;
            exit(1);
        }
        
        if( red < 0.0 || red > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for backgroundcolor. Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        // Read in the green color channel 
        double green = -1.0;
        if( nd->first_attribute("g") )
        {
            std::string attribute(nd->first_attribute("g")->value());
            if( !stringutils::extractFromString(attribute,green) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for backgroundcolor. Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for backgroundcolor. Exiting." << std::endl;
            exit(1);
        }
        
        if( green < 0.0 || green > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for backgroundcolor. Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        // Read in the blue color channel 
        double blue = -1.0;
        if( nd->first_attribute("b") )
        {
            std::string attribute(nd->first_attribute("b")->value());
            if( !stringutils::extractFromString(attribute,blue) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for backgroundcolor. Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for backgroundcolor. Exiting." << std::endl;
            exit(1);
        }
        
        if( blue < 0.0 || blue > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for backgroundcolor. Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        //std::cout << red << "   " << green << "   " << blue << std::endl;
        
        color.r = red;
        color.g = green;
        color.b = blue;  
    }
}

void TwoDSceneXMLParser::loadParticleColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& particle_colors )
{
    int particlecolornum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("particlecolor"); nd; nd = nd->next_sibling("particlecolor") )
    {
        // Determine which particle this color corresponds to
        int particle = -1;
        if( nd->first_attribute("i") )
        {
            std::string attribute(nd->first_attribute("i")->value());
            if( !stringutils::extractFromString(attribute,particle) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for particlecolor " << particlecolornum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for particlecolor " << particlecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( particle < 0 || particle >= (int) particle_colors.size() )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for particlecolor " << particlecolornum << ". Invalid particle specified. Valid range is " << 0 << "..." << particle_colors.size()-1 << std::endl;
            exit(1);
        }
        
        
        // Read in the red color channel 
        double red = -1.0;
        if( nd->first_attribute("r") )
        {
            std::string attribute(nd->first_attribute("r")->value());
            if( !stringutils::extractFromString(attribute,red) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlecolor " << particlecolornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlecolor " << particlecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( red < 0.0 || red > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlecolor " << particlecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        // Read in the green color channel 
        double green = -1.0;
        if( nd->first_attribute("g") )
        {
            std::string attribute(nd->first_attribute("g")->value());
            if( !stringutils::extractFromString(attribute,green) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlecolor " << particlecolornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlecolor " << particlecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( green < 0.0 || green > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlecolor " << particlecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        // Read in the blue color channel 
        double blue = -1.0;
        if( nd->first_attribute("b") )
        {
            std::string attribute(nd->first_attribute("b")->value());
            if( !stringutils::extractFromString(attribute,blue) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlecolor " << particlecolornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlecolor " << particlecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( blue < 0.0 || blue > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlecolor " << particlecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        particle_colors[particle] = renderingutils::Color(red,green,blue);
        
        ++particlecolornum;
    }
}

void TwoDSceneXMLParser::loadEdgeColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& edge_colors )
{
    int edgecolornum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("edgecolor"); nd; nd = nd->next_sibling("edgecolor") )
    {
        // Determine which particle this color corresponds to
        int edge = -1;
        if( nd->first_attribute("i") )
        {
            std::string attribute(nd->first_attribute("i")->value());
            if( !stringutils::extractFromString(attribute,edge) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for edgecolor " << edgecolornum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for edgecolor " << edgecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( edge < 0 || edge > (int) edge_colors.size() )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for edgecolor " << edgecolornum << ". Invalid edge specified. Valid range is " << 0 << "..." << edge_colors.size()-1 << std::endl;
            exit(1);
        }
        
        
        // Read in the red color channel 
        double red = -1.0;
        if( nd->first_attribute("r") )
        {
            std::string attribute(nd->first_attribute("r")->value());
            if( !stringutils::extractFromString(attribute,red) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for edgecolor " << edgecolornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for edgecolor " << edgecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( red < 0.0 || red > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for edgecolor " << edgecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        // Read in the green color channel 
        double green = -1.0;
        if( nd->first_attribute("g") )
        {
            std::string attribute(nd->first_attribute("g")->value());
            if( !stringutils::extractFromString(attribute,green) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for edgecolor " << edgecolornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for edgecolor " << edgecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( green < 0.0 || green > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for edgecolor " << edgecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        // Read in the blue color channel 
        double blue = -1.0;
        if( nd->first_attribute("b") )
        {
            std::string attribute(nd->first_attribute("b")->value());
            if( !stringutils::extractFromString(attribute,blue) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for edgecolor " << edgecolornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for edgecolor " << edgecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( blue < 0.0 || blue > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for edgecolor " << edgecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        //std::cout << "edge: " << edge << " r: " << red << " g: " << green << " b: " << blue << std::endl;
        
        edge_colors[edge] = renderingutils::Color(red,green,blue);
        
        ++edgecolornum;
    }  
}

void TwoDSceneXMLParser::loadHalfplaneColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& halfplane_colors )
{
    int halfplanecolornum = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("halfplanecolor"); nd; nd = nd->next_sibling("halfplanecolor") )
    {
        // Determine which particle this color corresponds to
        int halfplane = -1;
        if( nd->first_attribute("i") )
        {
            std::string attribute(nd->first_attribute("i")->value());
            if( !stringutils::extractFromString(attribute,halfplane) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for halfplanecolor " << halfplanecolornum << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for halfplanecolor " << halfplanecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( halfplane < 0 || halfplane > (int) halfplane_colors.size() )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for halfplanecolor " << halfplanecolornum << ". Invalid half-plane specified. Valid range is " << 0 << "..." << halfplane_colors.size()-1 << std::endl;
            exit(1);
        }
        
        
        // Read in the red color channel 
        double red = -1.0;
        if( nd->first_attribute("r") )
        {
            std::string attribute(nd->first_attribute("r")->value());
            if( !stringutils::extractFromString(attribute,red) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for halfplanecolor " << halfplanecolornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for halfplanecolor " << halfplanecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( red < 0.0 || red > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for halfplanecolor " << halfplanecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        // Read in the green color channel 
        double green = -1.0;
        if( nd->first_attribute("g") )
        {
            std::string attribute(nd->first_attribute("g")->value());
            if( !stringutils::extractFromString(attribute,green) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for halfplanecolor " << halfplanecolornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for halfplanecolor " << halfplanecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( green < 0.0 || green > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for halfplanecolor " << halfplanecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        // Read in the blue color channel 
        double blue = -1.0;
        if( nd->first_attribute("b") )
        {
            std::string attribute(nd->first_attribute("b")->value());
            if( !stringutils::extractFromString(attribute,blue) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for halfplanecolor " << halfplanecolornum << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for halfplanecolor " << halfplanecolornum << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( blue < 0.0 || blue > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for halfplanecolor " << halfplanecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        halfplane_colors[halfplane] = renderingutils::Color(red,green,blue);
        
        ++halfplanecolornum;
    }  
}

void TwoDSceneXMLParser::loadParticlePaths( rapidxml::xml_node<>* node, double dt, std::vector<renderingutils::ParticlePath>& particle_paths )
{
    int numpaths = 0;
    for( rapidxml::xml_node<>* nd = node->first_node("particlepath"); nd; nd = nd->next_sibling("particlepath") )
    {
        // Determine which particle this color corresponds to
        int particle = -1;
        if( nd->first_attribute("i") )
        {
            std::string attribute(nd->first_attribute("i")->value());
            if( !stringutils::extractFromString(attribute,particle) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for particlepath " << numpaths << ". Value must be integer. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for particlepath " << numpaths << ". Exiting." << std::endl;
            exit(1);
        }
        
        // How long the path should be buffered for
        double duration = -1.0;
        if( nd->first_attribute("duration") )
        {
            std::string attribute(nd->first_attribute("duration")->value());
            if( !stringutils::extractFromString(attribute,duration) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of duration attribute for particlepath " << numpaths << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of duration attribute for particlepath " << numpaths << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( duration < 0.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of duration attribute for particlepath " << numpaths << ". Value must be positive scalar." << std::endl;
            exit(1);
        }
        
        
        // Read in the red color channel 
        double red = -1.0;
        if( nd->first_attribute("r") )
        {
            std::string attribute(nd->first_attribute("r")->value());
            if( !stringutils::extractFromString(attribute,red) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlepath " << numpaths << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlepath " << numpaths << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( red < 0.0 || red > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlepath " << numpaths << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        // Read in the green color channel 
        double green = -1.0;
        if( nd->first_attribute("g") )
        {
            std::string attribute(nd->first_attribute("g")->value());
            if( !stringutils::extractFromString(attribute,green) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlepath " << numpaths << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlepath " << numpaths << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( green < 0.0 || green > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlepath " << numpaths << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        
        // Read in the blue color channel 
        double blue = -1.0;
        if( nd->first_attribute("b") )
        {
            std::string attribute(nd->first_attribute("b")->value());
            if( !stringutils::extractFromString(attribute,blue) )
            {
                std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlepath " << numpaths << ". Value must be scalar. Exiting." << std::endl;
                exit(1);
            }        
        }
        else
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlepath " << numpaths << ". Exiting." << std::endl;
            exit(1);
        }
        
        if( blue < 0.0 || blue > 1.0 )
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlepath " << numpaths << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
            exit(1);
        }
        
        particle_paths.push_back(renderingutils::ParticlePath( particle, ceil(duration/dt), renderingutils::Color(red,green,blue) ));
        
        //std::cout << particle << " " << duration << " " << red << " " << green << " " << blue << std::endl;
        
        ++numpaths;
    }  
}

void TwoDSceneXMLParser::loadSceneDescriptionString( rapidxml::xml_node<>* node, std::string& description_string )
{
    assert( node != NULL );
    
    description_string = "No description specified.";
    
    // Attempt to locate the integrator node
    rapidxml::xml_node<>* nd = node->first_node("description");
    if( nd != NULL ) 
    {
        rapidxml::xml_attribute<>* typend = nd->first_attribute("text"); 
        if( typend == NULL ) 
        {
            std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No text attribute specified for description. Exiting." << std::endl;
            exit(1);
        }
        description_string = typend->value();
    }
}



