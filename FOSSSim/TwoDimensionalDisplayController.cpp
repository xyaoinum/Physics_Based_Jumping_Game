#include "TwoDimensionalDisplayController.h"

TwoDimensionalDisplayController::TwoDimensionalDisplayController( int width, int height )
: m_window_width(width)
, m_window_height(height)
, m_scale_factor(1.0)
, m_center_x(0.0)
, m_center_y(0.0)
, m_left_drag(false)
, m_right_drag(false)
, m_last_x(0)
, m_last_y(0)
{}

void TwoDimensionalDisplayController::reshape( int w, int h ) 
{
  assert( renderingutils::checkGLErrors() );
  // Record the new width and height
  m_window_width = w;
  m_window_height = h;
  // Reset the coordinate system before modifying
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // Set the coordinate system to achieve the desired zoom level, center
  double ratio = (double)h/(double)w;
  gluOrtho2D(m_center_x-m_scale_factor/ratio,m_center_x+m_scale_factor/ratio,m_center_y-m_scale_factor,m_center_y+m_scale_factor);
  // Set the viewport to be the entire window
  glViewport(0, 0, w, h);
  // Render the scene
  glutPostRedisplay();
  assert( renderingutils::checkGLErrors() );
}

int TwoDimensionalDisplayController::getWorldWidth() const
{
  double ratio = m_window_height/m_window_width;
  return 2*m_scale_factor/ratio;
}

int TwoDimensionalDisplayController::getWorldHeight() const
{
  return 2*m_scale_factor;
}

void TwoDimensionalDisplayController::keyboard( unsigned char key, int x, int y )
{
  if( key == '-' || key == '_' )
  {
    m_scale_factor += 0.1;
    reshape(m_window_width,m_window_height);
  }
  else if( key == '=' || key == '+' )
  {
    m_scale_factor = std::max(0.1,m_scale_factor-0.1);
    reshape(m_window_width,m_window_height);
  }
}

void TwoDimensionalDisplayController::special( int key, int x, int y )
{
  if( GLUT_KEY_UP == key ) 
  {
    m_center_y += 0.1;
    reshape(m_window_width,m_window_height);
  }
  else if( GLUT_KEY_DOWN == key ) 
  {
    m_center_y -= 0.1;
    reshape(m_window_width,m_window_height);
  }
  else if( GLUT_KEY_LEFT == key ) 
  {
    m_center_x -= 0.1;
    reshape(m_window_width,m_window_height);
  }
  else if( GLUT_KEY_RIGHT == key ) 
  {
    m_center_x += 0.1;
    reshape(m_window_width,m_window_height);
  }
}  

void TwoDimensionalDisplayController::mouse( int button, int state, int x, int y )
{
  if( !m_right_drag && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
  {
    m_left_drag = true;
    m_last_x = x;
    m_last_y = y;
  }
  if( button == GLUT_LEFT_BUTTON && state == GLUT_UP )
  {
    m_left_drag = false;
  }
  
  if( !m_left_drag && button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN )
  {
    m_right_drag = true;
    m_last_x = x;
    m_last_y = y;
  }
  if( button == GLUT_RIGHT_BUTTON && state == GLUT_UP )
  {
    m_right_drag = false;
  }
}

void TwoDimensionalDisplayController::translateView( double dx, double dy )
{
  double percent_x = dx/((double)m_window_width);
  double percent_y = dy/((double)m_window_height);
  double translate_x = percent_x*2.0*m_scale_factor*((double)m_window_width)/((double)m_window_height);
  double translate_y = percent_y*2.0*m_scale_factor;
  m_center_x -= translate_x;
  m_center_y += translate_y;
  reshape(m_window_width,m_window_height);
}

void TwoDimensionalDisplayController::zoomView( double dx, double dy )
{
  double percent_x = dx/((double)m_window_width);
  double percent_y = dy/((double)m_window_height);
  
  double scale;
  if( std::fabs(percent_x) > std::fabs(percent_y) ) scale = -percent_x;
  else scale = percent_y;
  
  m_scale_factor += 2.0*scale;
  
  reshape(m_window_width,m_window_height);
}

void TwoDimensionalDisplayController::motion( int x, int y ) 
{
  if( m_left_drag ) 
  {
    double dx = x - m_last_x;
    double dy = y - m_last_y;
    m_last_x = x;
    m_last_y = y;
    translateView( dx, dy );
  }
  if( m_right_drag ) 
  {
    double dx = x - m_last_x;
    double dy = y - m_last_y;
    m_last_x = x;
    m_last_y = y;
    zoomView( dx, dy );
  }
}

void TwoDimensionalDisplayController::adjustCamera(std::vector<Matrix2s> & features)
{
//  std::cout << "features" << std::endl;
//  for (size_t i = 0; i < features.size(); i++) std::cout << "lb: " << features[i].col(0).transpose() << " rt: " << features[i].col(1).transpose() << std::endl;
  
  int w = m_window_width;
  int h = m_window_height;
  
  double aspect_ratio = (double)h / (double)w;
  
  Matrix2s camera_view_boundaries;
  camera_view_boundaries.col(0) = Vector2s(m_center_x - m_scale_factor / aspect_ratio, m_center_y - m_scale_factor);
  camera_view_boundaries.col(1) = Vector2s(m_center_x + m_scale_factor / aspect_ratio, m_center_y + m_scale_factor);

  Matrix2s camera_expand_boundaries;
  camera_expand_boundaries.col(0) = Vector2s(m_center_x - 0.8 * m_scale_factor / aspect_ratio, m_center_y - 0.8 * m_scale_factor);
  camera_expand_boundaries.col(1) = Vector2s(m_center_x + 0.8 * m_scale_factor / aspect_ratio, m_center_y + 0.8 * m_scale_factor);
  
  Matrix2s camera_shrink_boundaries;
  camera_shrink_boundaries.col(0) = Vector2s(m_center_x - 0.1 * m_scale_factor / aspect_ratio, m_center_y - 0.1 * m_scale_factor);
  camera_shrink_boundaries.col(1) = Vector2s(m_center_x + 0.1 * m_scale_factor / aspect_ratio, m_center_y + 0.1 * m_scale_factor);
  
//  std::cout << "camera view: \n" << camera_view_boundaries << "\nexpand: \n" << camera_expand_boundaries << "\nshrink: \n" << camera_shrink_boundaries << "\n" << std::endl;
  
  Matrix2s feature_bounding_box;
  feature_bounding_box.col(0) = Vector2s(std::numeric_limits<scalar>::infinity(), std::numeric_limits<scalar>::infinity());
  feature_bounding_box.col(1) = -feature_bounding_box.col(0);
  for (size_t i = 0; i < features.size(); i++)
  {
    Vector2s lb = features[i].col(0);
    Vector2s rt = features[i].col(1);
    if (lb.x() < feature_bounding_box.col(0).x())
      feature_bounding_box.col(0).x() = lb.x();
    if (lb.y() < feature_bounding_box.col(0).y())
      feature_bounding_box.col(0).y() = lb.y();
    if (rt.x() > feature_bounding_box.col(1).x())
      feature_bounding_box.col(1).x() = rt.x();
    if (rt.y() > feature_bounding_box.col(1).y())
      feature_bounding_box.col(1).y() = rt.y();
  }

  Vector2s lb = feature_bounding_box.col(0);
  Vector2s rt = feature_bounding_box.col(1);
  
  Matrix2s camera_view_movement;
  camera_view_movement.setZero();
  
  if (lb.x() < camera_expand_boundaries.col(0).x())
    camera_view_movement.col(0).x() = lb.x() - camera_expand_boundaries.col(0).x();
  if (lb.x() > camera_shrink_boundaries.col(0).x())
    camera_view_movement.col(0).x() = lb.x() - camera_shrink_boundaries.col(0).x();
  
  if (lb.y() < camera_expand_boundaries.col(0).y())
    camera_view_movement.col(0).y() = lb.y() - camera_expand_boundaries.col(0).y();
  if (lb.y() > camera_shrink_boundaries.col(0).y())
    camera_view_movement.col(0).y() = lb.y() - camera_shrink_boundaries.col(0).y();
  
  if (rt.x() > camera_expand_boundaries.col(1).x())
    camera_view_movement.col(1).x() = rt.x() - camera_expand_boundaries.col(1).x();
  if (rt.x() < camera_shrink_boundaries.col(1).x())
    camera_view_movement.col(1).x() = rt.x() - camera_shrink_boundaries.col(1).x();
  
  if (rt.y() > camera_expand_boundaries.col(1).y())
    camera_view_movement.col(1).y() = rt.y() - camera_expand_boundaries.col(1).y();
  if (rt.y() < camera_shrink_boundaries.col(1).y())
    camera_view_movement.col(1).y() = rt.y() - camera_shrink_boundaries.col(1).y();
  
  m_center_x += (camera_view_movement.col(0).x() + camera_view_movement.col(1).x()) / 2;
  m_center_y += (camera_view_movement.col(0).y() + camera_view_movement.col(1).y()) / 2;
  m_scale_factor += std::max(camera_view_movement.col(1).x() - camera_view_movement.col(0).x(), camera_view_movement.col(1).y() - camera_view_movement.col(0).y()) / 2;
  
  reshape(m_window_width,m_window_height);
}

int TwoDimensionalDisplayController::getWindowWidth() const
{
  return m_window_width;
}

int TwoDimensionalDisplayController::getWindowHeight() const
{
  return m_window_height;
}

double TwoDimensionalDisplayController::getCenterX() const
{
  return m_center_x;
}

double TwoDimensionalDisplayController::getCenterY() const
{
  return m_center_y;
}


void TwoDimensionalDisplayController::setCenterX( double x )
{
  m_center_x = x;
}

void TwoDimensionalDisplayController::setCenterY( double y )
{
  m_center_y = y;
}

void TwoDimensionalDisplayController::setScaleFactor( double scale )
{
  m_scale_factor = scale;
}

double TwoDimensionalDisplayController::getScaleFactor() const
{
  return m_scale_factor;
}


