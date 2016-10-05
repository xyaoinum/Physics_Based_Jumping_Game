#include <Eigen/StdVector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

#include <tclap/CmdLine.h>

// TODO: Clean these up, we don't need them all anymore.
#include "TwoDScene.h"
#include "Force.h"
#include "SpringForce.h"
#include "ExplicitEuler.h"
#include "SemiImplicitEuler.h"
#include "TwoDimensionalDisplayController.h"
#include "TwoDSceneRenderer.h"
#include "TwoDSceneXMLParser.h"
#include "TwoDSceneSerializer.h"
#include "TwoDSceneGrader.h"
#include "StringUtilities.h"
#include "MathDefs.h"
#include "TimingUtilities.h"
#include "RenderingUtilities.h"
#include "TwoDSceneSVGRenderer.h"
#include "YImage.h"
#include "CollisionHandler.h"
#include "CollisionDetector.h"

#include "ExecutableSimulation.h"
#include "ParticleSimulation.h"

//#define PNG_FD2263
#ifdef PNG_FD2263
#include </opt/local/include/png.h>
#endif

///////////////////////////////////////////////////////////////////////////////
// Contains the actual simulation, renderer, parser, and serializer
ExecutableSimulation* g_executable_simulation;


///////////////////////////////////////////////////////////////////////////////
// Rendering State
bool g_rendering_enabled = true;
double g_sec_per_frame;
double g_last_time = timingutils::seconds();

TwoDimensionalDisplayController g_display_controller(512,512);
renderingutils::Color g_bgcolor(1.0,1.0,1.0);


///////////////////////////////////////////////////////////////////////////////
// SVG Rendering State
bool g_svg_enabled = false;
std::string g_movie_dir;


///////////////////////////////////////////////////////////////////////////////
// Parser state
std::string g_xml_scene_file;
std::string g_description;
std::string g_scene_tag = "";


///////////////////////////////////////////////////////////////////////////////
// Scene input/output/comparison state
bool g_save_to_binary = false;
std::string g_binary_file_name;
std::ofstream g_binary_output;

bool g_simulate_comparison = false;
std::string g_comparison_file_name;
std::ifstream g_binary_input;


///////////////////////////////////////////////////////////////////////////////
// Simulation state
bool g_paused = true;
scalar g_dt = 0.0;
int g_num_steps = 0;
int g_current_step = 0;
bool g_simulation_ran_to_completion = false;



#ifdef PNG_FD2263
void startpng()
{
	
}

void endpng()
{
	
}

void writetopng()
{
	//
	// source of the code:
	// http://zarb.org/~gc/html/libpng.html
	//
	static int counter = 0;
	counter++;
	
	char filename[1024];
	sprintf(filename, "creative/ABMovie/frame%08d.png", counter);
	
	int x, y;
	
	int width, height;
	width = 512;
	height = 512;
	png_byte color_type = PNG_COLOR_TYPE_RGB;
	png_byte bit_depth = 8;
	
	png_structp png_ptr;
	png_infop info_ptr;
	int number_of_passes;
	
	/* create file */
	FILE *fp = fopen(filename, "wb");
	if (!fp)
		printf("[write_png_file] File %s could not be opened for writing", filename);
	
	
	/* initialize stuff */
	png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	
	if (!png_ptr)
		printf("[write_png_file] png_create_write_struct failed");
	
	info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr)
		printf("[write_png_file] png_create_info_struct failed");
	
	if (setjmp(png_jmpbuf(png_ptr)))
		printf("[write_png_file] Error during init_io");
	
	png_init_io(png_ptr, fp);
	
	
	/* write header */
	if (setjmp(png_jmpbuf(png_ptr)))
		printf("[write_png_file] Error during writing header");
	
	png_set_IHDR(png_ptr, info_ptr, width, height,
				 bit_depth, color_type, PNG_INTERLACE_NONE,
				 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
	
	png_write_info(png_ptr, info_ptr);
	
	
	/* write bytes */
	if (setjmp(png_jmpbuf(png_ptr)))
		printf("[write_png_file] Error during writing bytes");
	
	unsigned char image[width * height * 3];
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
	
	unsigned char * rows[height];
	for (int i = 0; i < height; i++)
	{
		rows[i] = image + (height - i - 1) * width * 3;
	}
	
	png_write_image(png_ptr, rows);
	
	/* end write */
	if (setjmp(png_jmpbuf(png_ptr)))
		printf("[write_png_file] Error during end of write");
	
	png_write_end(png_ptr, NULL);
	
	fclose(fp);
	
}

#else
void startpng()
{
	
}

void endpng()
{
	
}

void writetopng()
{
	
}

#endif

///////////////////////////////////////////////////////////////////////////////
// Simulation functions

void miscOutputCallback();
void sceneScriptingCallback();
void dumpPNG(const std::string &filename);

void stepSystem()
{
  assert( !(g_save_to_binary&&g_simulate_comparison) );

  // Determine if the simulation is complete
  if( g_current_step >= g_num_steps )
  {
    std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Simulation complete at time " << g_current_step*g_dt << ". Exiting." << std::endl;
    g_simulation_ran_to_completion = true;
    exit(0);
  }

  // If comparing simulations, copy state of comparison scene to simulated scene
//  if( g_simulate_comparison )
//  {
//    g_executable_simulation->copyComparisonSceneToScene();
//    g_executable_simulation->loadHybridData(g_binary_input);
//    // Deserialize and verify any per-step quantities (e.g. momentum)
//    g_executable_simulation->deserializeAndVerifyPerStepQuantities(g_binary_input);
//  }

  // Step the system forward in time
  g_executable_simulation->stepSystem(g_dt);
  g_current_step++;

  // If the user wants to save output to a binary
//  if( g_save_to_binary ) 
//  {
//    g_executable_simulation->serializeHybridData(g_binary_output);
//    g_executable_simulation->serializeScene(g_binary_output);
//    // Save any per-step quantities to verify later (e.g. momentum)
//    g_executable_simulation->serializePerStepQuantities(g_binary_output);
//  }
//
//
//  // Update the state of the renderers
//  if( g_rendering_enabled ) g_executable_simulation->updateOpenGLRendererState();
//  g_executable_simulation->updateSVGRendererState();
//
//  // If comparing simulations, load comparison scene's equivalent step
//  if( g_simulate_comparison ) g_executable_simulation->loadComparisonScene(g_binary_input);
//
//  // If the user wants to generate a SVG movie
//  if( g_svg_enabled )
//  {
//    // Generate a filename
//    std::stringstream name;
//    name << std::setfill('0');
//    name << g_movie_dir << "/frame" << std::setw(20) << g_current_step << ".svg";    
//    // Actually write the svg out
//    g_executable_simulation->renderSceneSVG(name.str());    
//  }  
//  
//  // If comparing simulations, compute the accumulated residual
//  if( g_simulate_comparison ) g_executable_simulation->updateSceneComparison();
  
  // If the user wants to generate a PNG movie
  #ifdef PNGOUT
    std::stringstream oss;
    oss << "pngs/frame" << std::setw(5) << std::setfill('0') << g_current_step << ".png";
    dumpPNG(oss.str());
  #endif

  // Execute the user-customized output callback
  miscOutputCallback();

  // Execute the user-customized scripting callback
  sceneScriptingCallback();
}

void headlessSimLoop()
{
  scalar nextpercent = 0.02;
  std::cout << outputmod::startpink << "Progress: " << outputmod::endpink;
  for( int i = 0; i < 50; ++i ) std::cout << "-";
  std::cout << std::endl;
  std::cout << "          ";
  while( true )
  {
    scalar percent_done = ((double)g_current_step)/((double)g_num_steps);
    if( percent_done >= nextpercent )
    {
      nextpercent += 0.02;
      std::cout << "." << std::flush;
    }
    stepSystem();
  }
}



///////////////////////////////////////////////////////////////////////////////
// Rendering and UI functions

void dumpPNG(const std::string &filename)
{
  #ifdef PNGOUT
    YImage image;
    image.resize(g_display_controller.getWindowWidth(), g_display_controller.getWindowHeight());

    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
    glReadBuffer(GL_BACK);

    glFinish();
    glReadPixels(0, 0, g_display_controller.getWindowWidth(), g_display_controller.getWindowHeight(), GL_RGBA, GL_UNSIGNED_BYTE, image.data());
    image.flip();

    image.save(filename.c_str());
  #endif
}

void reshape( int w, int h ) 
{
  g_display_controller.reshape(w,h);

  assert( renderingutils::checkGLErrors() );
}

// TODO: Move these functions to scene renderer?
void setOrthographicProjection() 
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	
	gluOrtho2D(0, g_display_controller.getWindowWidth(), 0, g_display_controller.getWindowHeight());
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

  assert( renderingutils::checkGLErrors() );
}

void renderBitmapString( float x, float y, float z, void *font, std::string s, int align = 0 )  // align = 0 for left, 1 for right, 2 for center
{
  float width = 0;
  for (std::string::iterator i = s.begin(); i != s.end(); i++)
  {
    char c = *i;
    width += glutBitmapWidth(font, c);
  }
  
	if (align == 0)
    glRasterPos3f(x, y, z);
  else if (align == 1)
    glRasterPos3f(x - width, y, z);
  else
    glRasterPos3f(x - width / 2, y, z);
  
	for( std::string::iterator i = s.begin(); i != s.end(); ++i )
	{
		char c = *i;
		glutBitmapCharacter(font, c);
	}

  assert( renderingutils::checkGLErrors() );
}

void drawHUD()
{
  setOrthographicProjection();
  glColor3f(1.0-g_bgcolor.r,1.0-g_bgcolor.g,1.0-g_bgcolor.b);
  renderBitmapString( 4, g_display_controller.getWindowHeight()-20, 0.0, GLUT_BITMAP_HELVETICA_18, stringutils::convertToString(g_current_step*g_dt) ); 
  glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

  assert( renderingutils::checkGLErrors() );
}


void centerCamera()
{
    
    //std::cout << "camera in main" << std::endl;
  renderingutils::Viewport view;

  g_executable_simulation->computeCameraCenter(view);

  scalar ratio = ((scalar)g_display_controller.getWindowHeight())/((scalar)g_display_controller.getWindowWidth());

  view.size = 1.2*std::max(ratio*view.rx,view.ry);

  g_display_controller.setCenterX(view.cx);
  g_display_controller.setCenterY(view.cy);
  g_display_controller.setScaleFactor(view.size);
}


void display()
{
  std::vector<Matrix2s> features;
  g_executable_simulation->getFeatures(features);
  //g_display_controller.adjustCamera(features);

  
  centerCamera();    
  g_display_controller.reshape(g_display_controller.getWindowWidth(),g_display_controller.getWindowHeight());
  glutPostRedisplay();
  
  glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);

  assert( g_executable_simulation != NULL );
  g_executable_simulation->renderSceneOpenGL();

  drawHUD();

  glutSwapBuffers();

  assert( renderingutils::checkGLErrors() );
}


void keyboard( unsigned char key, int x, int y )
{
  g_display_controller.keyboard(key,x,y);
  g_executable_simulation->keyboard(key,x,y);
  
  //std::cout << "key:" << (int)key << std::endl;
  
  //g_display_controller.cur_key = (int)key;
  
  if( key == 27 || key == 'q' )
  {
    exit(0);
  }
  else if( key == 's' || key =='S' )
  {
      //std::cout << "s in main" << std::endl;
    stepSystem();
    glutPostRedisplay();
  }
  else if( key == ' ' )
  {
    g_paused = !g_paused;
  }
  else if( key == 'c' || key == 'C' )
  {
    centerCamera();
    g_display_controller.reshape(g_display_controller.getWindowWidth(),g_display_controller.getWindowHeight());
    glutPostRedisplay();
  }
  else if( key == 'i' || key == 'I' )
  {
    std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Saving screenshot as 'output.svg'." << std::endl;
    g_executable_simulation->renderSceneSVG("output.svg");
  }

  assert( renderingutils::checkGLErrors() );
}

// Proccess 'special' keys
void special( int key, int x, int y )
{
  g_display_controller.special(key,x,y);
  
  assert( renderingutils::checkGLErrors() );
}

void mouse( int button, int state, int x, int y )
{
  g_display_controller.mouse(button,state,x,y);
  g_executable_simulation->mouse(button,state,x,y);
  
  assert( renderingutils::checkGLErrors() );
}

void motion( int x, int y ) 
{
  g_display_controller.motion(x,y);
  g_executable_simulation->motion(x,y);
  
  assert( renderingutils::checkGLErrors() );
}

void passiveMotion( int x, int y ) 
{
  g_executable_simulation->passiveMotion(x,y);
  
  assert( renderingutils::checkGLErrors() );
}

void idle()
{
  //std::cout << "g_last_time: " << g_last_time << std::endl;
  // Trigger the next timestep
  double current_time = timingutils::seconds();
  //std::cout << "current_time: " << current_time << std::endl;
  //std::cout << "g_sec_per_frame: " << g_sec_per_frame << std::endl;
  if( !g_paused && current_time-g_last_time >= g_sec_per_frame ) 
  {
    g_last_time = current_time;
    stepSystem();
    glutPostRedisplay();
  }
  
  assert( renderingutils::checkGLErrors() );
}

void initializeOpenGLandGLUT( int argc, char** argv )
{
  // Initialize GLUT
  glutInit(&argc,argv);
  glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);
  glutInitWindowSize(g_display_controller.getWindowWidth(),g_display_controller.getWindowHeight());
  glutCreateWindow("Forty One Sixty Seven Sim");
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutSpecialFunc(special);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutPassiveMotionFunc(passiveMotion);
  glutIdleFunc(idle);
  
  // Initialize OpenGL
	reshape(g_display_controller.getWindowWidth(),g_display_controller.getWindowHeight());
  glClearColor(g_bgcolor.r, g_bgcolor.g, g_bgcolor.b, 1.0);
  
  assert( renderingutils::checkGLErrors() );
}


///////////////////////////////////////////////////////////////////////////////
// Parser functions

void loadScene( const std::string& file_name )
{
  // Maximum time in the simulation to run for. This has nothing to do with run time, cpu time, etc. This is time in the 'virtual world'.
  scalar max_time;
  // Maximum frequency, in wall clock time, to execute the simulation for. This serves as a cap for simulations that run too fast to see a solution.
  scalar steps_per_sec_cap = 100.0;
  // Contains the center and 'scale factor' of the view
  renderingutils::Viewport view;

  // Load the simulation and pieces of rendring and UI state
  assert( g_executable_simulation == NULL );
  TwoDSceneXMLParser xml_scene_parser;
  xml_scene_parser.loadExecutableSimulation( file_name, g_simulate_comparison, g_rendering_enabled, g_display_controller, &g_executable_simulation,
                                             view, g_dt, max_time, steps_per_sec_cap, g_bgcolor, g_description, g_scene_tag );
  assert( g_executable_simulation != NULL );

  // If the user did not request a custom viewport, try to compute a reasonable default.
  if( view.size <= 0.0 )
  {
    centerCamera();
  }
  // Otherwise set the viewport per the user's request.
  else
  {
    g_display_controller.setCenterX(view.cx);
    g_display_controller.setCenterY(view.cy);
    g_display_controller.setScaleFactor(view.size);
  }

  // To cap the framerate, compute the minimum time a single timestep should take
  g_sec_per_frame = 1.0/steps_per_sec_cap;
  // Integer number of timesteps to take
  g_num_steps = ceil(max_time/g_dt);
  // We begin at the 0th timestep
  g_current_step = 0;  
}

void parseCommandLine( int argc, char** argv )
{
  try 
  {
    TCLAP::CmdLine cmd("Forty One Sixty Seven Sim");

    // XML scene file to load
    TCLAP::ValueArg<std::string> scene("s", "scene", "Simulation to run; an xml scene file", true, "", "string", cmd);

    // Begin the scene paused or running
    TCLAP::ValueArg<bool> paused("p", "paused", "Begin the simulation paused if 1, running if 0", false, true, "boolean", cmd);

    // Run the simulation with rendering enabled or disabled
    TCLAP::ValueArg<bool> display("d", "display", "Run the simulation with display enabled if 1, without if 0", false, true, "boolean", cmd);

    // These cannot be set at the same time
    // File to save output to
    TCLAP::ValueArg<std::string> output("o", "outputfile", "Binary file to save simulation state to", false, "", "string", cmd);
    // File to load for comparisons
    TCLAP::ValueArg<std::string> input("i", "inputfile", "Binary file to load simulation state from for comparison", false, "", "string", cmd);

    // Save svgs to a movie directory
    TCLAP::ValueArg<std::string> movie("m", "moviedir", "Directory to output svg screenshot to", false, "", "string", cmd);

    cmd.parse(argc, argv);

    if( output.isSet() && input.isSet() ) throw TCLAP::ArgException( "arguments i and o specified simultaneously", "arguments i and o", "invalid argument combination" );

    assert( scene.isSet() );
    g_xml_scene_file = scene.getValue();
    g_paused = paused.getValue();
    g_rendering_enabled = display.getValue();
    
    if( output.isSet() )
    {
      g_save_to_binary = true;
      g_binary_file_name = output.getValue();
    }
    
    if( input.isSet() )
    {
      g_simulate_comparison = true;
      g_comparison_file_name = input.getValue();
    }

    if( movie.isSet() )
    {
      g_svg_enabled = true;
      g_movie_dir = movie.getValue();
    }
  } 
  catch (TCLAP::ArgException& e) 
  {
    std::cerr << "error: " << e.what() << std::endl;
    exit(1);
  }
}



///////////////////////////////////////////////////////////////////////////////
// Various support functions

void miscOutputFinalization();

void cleanupAtExit()
{
  if( g_binary_output.is_open() )
  {
    std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Saved simulation to file '" << g_binary_file_name << "'." << std::endl;
    g_binary_output.close();
  }

  if( g_binary_input.is_open() )
  {
    std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Closing benchmarked simulation file '" << g_comparison_file_name << "'." << std::endl;
  }

//  if( g_executable_simulation != NULL && g_simulate_comparison ) g_executable_simulation->printErrorInformation( g_simulation_ran_to_completion );
  if (dynamic_cast<JointMotorBodySimulation *>(g_executable_simulation))
    (dynamic_cast<JointMotorBodySimulation *>(g_executable_simulation))->printDetectorReport();

  miscOutputFinalization();

  if( g_executable_simulation != NULL )
  {
    delete g_executable_simulation;
    g_executable_simulation = NULL;
  }  
}

std::ostream& fosssim_header( std::ostream& stream )
{
  stream << outputmod::startgreen << 
  "------------------------------------------    " << std::endl <<
  "  _____ ___  ____ ____ ____  _                " << std::endl <<
  " |  ___/ _ \\/ ___/ ___/ ___|(_)_ __ ___      " << std::endl <<
  " | |_ | | | \\___ \\___ \\___ \\| | '_ ` _ \\ " << std::endl <<
  " |  _|| |_| |___) |__) |__) | | | | | | |     " << std::endl << 
  " |_|   \\___/|____/____/____/|_|_| |_| |_|    " << std::endl <<
  "------------------------------------------    " 
  << outputmod::endgreen << std::endl;
  
  return stream;
}

//std::ofstream g_debugoutput;

void miscOutputInitialization()
{
  //g_debugoutput.open("debugoutput.txt");
  //g_debugoutput << "# Time   PotentialEnergy   KineticEnergy   TotalEnergy" << std::endl;
  //g_debugoutput << g_current_step*g_dt << "\t" << g_scene.computePotentialEnergy() << "\t" << g_scene.computeKineticEnergy() << "\t" << g_scene.computeTotalEnergy() << std::endl;
}

void miscOutputCallback()
{
  //std::cout << "T: " << g_current_step*g_dt << std::endl;
  g_executable_simulation->outputCallback(std::cout);
  //g_debugoutput << g_current_step*g_dt << "\t" << g_scene.computePotentialEnergy() << "\t" << g_scene.computeKineticEnergy() << "\t" << g_scene.computeTotalEnergy() << std::endl;
	writetopng();

}

void miscOutputFinalization()
{
  //g_debugoutput.close();
}

// Called at the end of each timestep. Intended for adding effects to creative scenes.
void sceneScriptingCallback()
{
//  // If the scene is one we wish to 'script'
//  if( g_scene_tag == "ParticleFountain" )
//  {
//    // Get the particle tags
//    const std::vector<std::string>& tags = g_scene.getParticleTags();
//    // Get the particle positions
//    VectorXs& x = g_scene.getX();
//    // Get the particle velocities
//    VectorXs& v = g_scene.getV();
//    // Get the particle colors
//    std::vector<renderingutils::Color>& pcolors = g_scene_renderer->getParticleColors();
//
//    // If any particles are tagged for teleportation and fall below -1.25
//    for( std::vector<std::string>::size_type i = 0; i < tags.size(); ++i ) if( tags[i] == "teleport" && x(2*i+1) < -1.25 )
//    {
//      // Return this particle to the origin
//      x.segment<2>(2*i).setZero();
//      // Give this particle some random upward velocity
//      double vx = 0.2*(((double)rand())/((double)RAND_MAX)-0.5);
//      double vy = 0.15*((double)rand())/((double)RAND_MAX);
//      v.segment<2>(2*i) << vx, vy;
//      // Assign the particle a random color
//      pcolors[i].r = ((double)rand())/((double)RAND_MAX);
//      pcolors[i].g = ((double)rand())/((double)RAND_MAX);
//      pcolors[i].b = ((double)rand())/((double)RAND_MAX);
//    }
//  }
	
	if (g_scene_tag == "AngryBirds")
	{
		static int fcount = 0;
		fcount++;
		
		RigidBodyScene * scene = ((RigidBodySimulation *)g_executable_simulation)->getScene();
		if (fcount == 710)
		{
			//scene->getRigidBodies()[31].getX() = scene->getRigidBodies()[30].getX();
			//scene->getRigidBodies()[30].getX() = Vector2s(30, 0);
			scene->getRigidBodies()[30].getV() = Vector2s(-2, -1);
		}
		if (fcount == 740)
		{
			scene->getRigidBodies()[31].getX() = Vector2s(30, 10);
		}
	}
}

int main( int argc, char** argv )
{
  // Parse command line arguments
  parseCommandLine( argc, argv );
  assert( !(g_save_to_binary && g_simulate_comparison) );

  // Function to cleanup at progarm exit
  atexit(cleanupAtExit);

  // Load the user-specified scene
  loadScene(g_xml_scene_file);

  // If requested, open the binary output file
  if( g_save_to_binary )
  {
    // Attempt to open the binary
    g_binary_output.open(g_binary_file_name.c_str());
    if( g_binary_output.fail() ) 
    {
      std::cerr << outputmod::startred << "ERROR IN INITIALIZATION: "  << outputmod::endred << "Failed to open binary output file: " << " `" << g_binary_file_name << "`   Exiting." << std::endl;
      exit(1);
    }
    // Save the initial conditions
    g_executable_simulation->serializeScene(g_binary_output);
    // Save any start-of-simulation state to verify later (e.g. moment of inertia computations)
    g_executable_simulation->serializeStartOfSimState(g_binary_output);
    // Save any per-step quantities to verify later (e.g. momentum)
    g_executable_simulation->serializePerStepQuantities(g_binary_output);
  }
  // If requested, open the input file for the scene to benchmark
  else if( g_simulate_comparison )
  {
    // Attempt to open the binary
    g_binary_input.open(g_comparison_file_name.c_str());
    if( g_binary_input.fail() ) 
    {
      std::cerr << outputmod::startred << "ERROR IN INITIALIZATION: "  << outputmod::endred << "Failed to open binary input file: " << " `" << argv[3] << "`   Exiting." << std::endl;
      exit(1);
    }
    assert( g_executable_simulation != NULL );
    // Load the initial conditions
    g_executable_simulation->loadComparisonScene(g_binary_input);
  }

  // Initialization for OpenGL and GLUT
  if( g_rendering_enabled ) initializeOpenGLandGLUT(argc,argv);

  // Print a header
  std::cout << fosssim_header << std::endl;

  // Print some status info about this FOSSSim build
  #ifdef FOSSSIM_VERSION
    std::cout << outputmod::startblue << "FOSSSim Version: "  << outputmod::endblue << FOSSSIM_VERSION << std::endl;
  #endif
  #ifdef CMAKE_BUILD_TYPE
    std::cout << outputmod::startblue << "Build type: " << outputmod::endblue << CMAKE_BUILD_TYPE << std::endl;
  #endif
  #ifdef EIGEN_VECTORIZE
    std::cout << outputmod::startblue << "Vectorization: " << outputmod::endblue << "Enabled" << std::endl;
  #else
    std::cout << outputmod::startblue << "Vectorization: " << outputmod::endblue << "Disabled" << std::endl;
  #endif

  std::cout << outputmod::startblue << "Scene: " << outputmod::endblue << g_xml_scene_file << std::endl;
  std::cout << outputmod::startblue << "Integrator: " << outputmod::endblue << g_executable_simulation->getSolverName() << std::endl;
  std::cout << outputmod::startblue << "Collision Handling: " << outputmod::endblue << g_executable_simulation->getCollisionHandlerName() << std::endl;
  std::cout << outputmod::startblue << "Description: " << outputmod::endblue << g_description << std::endl;

  if( g_save_to_binary ) std::cout << outputmod::startpink << "FOSSSim message: "  << outputmod::endpink << "Saving simulation to: " << g_binary_file_name << std::endl;
  if( g_simulate_comparison ) std::cout << outputmod::startpink << "FOSSSim message: "  << outputmod::endpink << "Benchmarking simulation in: " << g_comparison_file_name << std::endl;

  // Load any start-of-simulation state to verify (e.g. moment of inertia computations) and verify that the computations are correct
  if( g_simulate_comparison )
  {
    g_executable_simulation->deserializeAndVerifyStartOfSimState(g_binary_input);
    //g_executable_simulation->deserializeAndVerifyPerStepQuantities(g_binary_input);
  }

  g_executable_simulation->beforeSimulationStarts();
  
  miscOutputInitialization();
  miscOutputCallback();

  if( g_rendering_enabled ){
      glutMainLoop();
  }else{
      headlessSimLoop();
  }

  return 0;
}
