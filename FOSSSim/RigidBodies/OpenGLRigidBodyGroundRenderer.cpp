#include "OpenGLRigidBodyGroundRenderer.h"
#include "../TwoDimensionalDisplayController.h"

extern TwoDimensionalDisplayController g_display_controller;

OpenGLRigidBodyGroundRenderer::OpenGLRigidBodyGroundRenderer( const renderingutils::Color& color )
: m_color(color)
{}

void OpenGLRigidBodyGroundRenderer::render( const std::vector<RigidBody>& rbs )
{
  glPushMatrix();
  
  glColor3d(m_color.r,m_color.g,m_color.b);
  glBegin(GL_TRIANGLES);
  glVertex4d(-10000,0,0,1);
  glVertex4d(10000,0,0,1);
  glVertex4d(0,-10000,0,1);
  glEnd();
  
  glColor3d(0, 0, 0);
  glBegin(GL_LINES);
  for (scalar x = (int)(g_display_controller.getCenterX() - 100); x < g_display_controller.getCenterX() + 100; x += 1.0)
  {
    glVertex2d(x, 0);
    glVertex2d(x - 100, -100);
  }
  glEnd();
  
  glPopMatrix();

  assert( renderingutils::checkGLErrors() );
}
