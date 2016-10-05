#include "OpenGLRigidBodyScheduledImpulseRenderer.h"
#include "../TwoDimensionalDisplayController.h"

extern TwoDimensionalDisplayController g_display_controller;

OpenGLRigidBodyScheduledImpulseRenderer::OpenGLRigidBodyScheduledImpulseRenderer( const renderingutils::Color& color, JointMotorBodySimulation::ScheduledPushEvent & push )
: m_push(push)
, m_color(color)
{

}

void OpenGLRigidBodyScheduledImpulseRenderer::render( const std::vector<RigidBody>& rbs )
{
  Vector2s x = rbs[m_push.rb].getX();
  Vector2s size(0.1 * m_push.impulse.norm(), 0.15);

  glPushMatrix();

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  glTranslated(x.x(), x.y(), 0.0);
  glRotated(180 / PI * atan2(m_push.impulse.y(), m_push.impulse.x()), 0, 0, 1);
  glScaled(size.x(), size.y(), 1.0);
  
  if (m_push.alpha >= 0.01)
  {
    m_push.alpha *= 0.999;
    if (m_push.alpha < 0.01)
      m_push.alpha = -1;
      
    glutPostRedisplay();
  }
  
  glColor4d(m_color.r, m_color.g, m_color.b, std::max(0.0, m_push.alpha));
  
  glBegin(GL_TRIANGLE_FAN);
  glVertex2d(0, 0);
  glVertex2d(-2, 2);
  glVertex2d(-2, 1);
  glVertex2d(-5, 1);
  glVertex2d(-5, -1);
  glVertex2d(-2, -1);
  glVertex2d(-2, -2);
  glEnd();
  
  glDisable(GL_BLEND);
  
  glPopMatrix();

  assert( renderingutils::checkGLErrors() );
}
