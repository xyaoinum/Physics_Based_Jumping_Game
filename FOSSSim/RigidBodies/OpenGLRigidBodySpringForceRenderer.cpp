#include "OpenGLRigidBodySpringForceRenderer.h"

OpenGLRigidBodySpringForceRenderer::OpenGLRigidBodySpringForceRenderer( const RigidBodySpringForce& rbspfrc, const renderingutils::Color& color )
//: m_rbspfrc(rbspfrc)
: m_color(color)
, m_rb0(rbspfrc.getFirstRigidBody())
, m_rb1(rbspfrc.getSecondRigidBody())
, m_vc0(rbspfrc.getFirstEndpoint())
, m_vc1(rbspfrc.getSecondEndpoint())
{}

void OpenGLRigidBodySpringForceRenderer::render( const std::vector<RigidBody>& rbs )
{
  // Compute the position of the endpoint on the first rigid body
  Vector2s p0 = (m_rb0==-1) ? m_vc0 : rbs[m_rb0].computeWorldSpacePosition(m_vc0);

  // Compute the position of the endpoint on the second rigid body
  Vector2s p1 = (m_rb1==-1) ? m_vc1 : rbs[m_rb1].computeWorldSpacePosition(m_vc1);

  // Render the spring as a dotted line
  //glLineStipple(1, 0x00FF);
  //glLineStipple(1,0x0F0F);
  //glEnable(GL_LINE_STIPPLE);
  
  if ((m_rb0 >= 0 && rbs[m_rb0].tag() == "treat_of_trick_or_treat") || (m_rb1 >= 0 && rbs[m_rb1].tag() == "treat_of_trick_or_treat"))
    return;
  
  glLineWidth(3.0);
  
  //glColor3d(m_color.r,m_color.g,m_color.b);

  glColor3d(0.01,0.01,0.01);
  
  glBegin(GL_LINES);
    glVertex2d(p0.x(),p0.y());
    glVertex2d(p1.x(),p1.y());
  glEnd();

  glDisable(GL_LINE_STIPPLE);

  assert( renderingutils::checkGLErrors() );
}
