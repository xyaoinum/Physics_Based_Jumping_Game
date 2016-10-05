#include "SVGRigidBodySpringForceRenderer.h"

SVGRigidBodySpringForceRenderer::SVGRigidBodySpringForceRenderer( const RigidBodySpringForce& rbspfrc, const renderingutils::Color& color )
//: m_rbspfrc(rbspfrc)
: m_color(color)
, m_rb0(rbspfrc.getFirstRigidBody())
, m_rb1(rbspfrc.getSecondRigidBody())
, m_vc0(rbspfrc.getFirstEndpoint())
, m_vc1(rbspfrc.getSecondEndpoint())
{}

void SVGRigidBodySpringForceRenderer::render( const std::vector<RigidBody>& rbs, std::fstream& file )
{
  std::cout << "YOYOYOYOYOYOY" << std::endl;
//  // Compute the position of the endpoint on the first rigid body
//  Vector2s p0 = (m_rb0==-1) ? m_vc0 : rbs[m_rb0].computeWorldSpacePosition(m_vc0);
//
//  // Compute the position of the endpoint on the second rigid body
//  Vector2s p1 = (m_rb1==-1) ? m_vc1 : rbs[m_rb1].computeWorldSpacePosition(m_vc1);
//
//  // Render the spring as a dotted line
//  //glLineStipple(1, 0x00FF);
//  //glLineStipple(1,0x0F0F);
//  //glEnable(GL_LINE_STIPPLE);
//  glLineWidth(3.0);
//  glColor3d(m_color.r,m_color.g,m_color.b);
//
//  glBegin(GL_LINES);
//    glVertex2d(p0.x(),p0.y());
//    glVertex2d(p1.x(),p1.y());
//  glEnd();
//
//  glDisable(GL_LINE_STIPPLE);
//
//  assert( renderingutils::checkGLErrors() );
}

Vector2s SVGRigidBodySpringForceRenderer::getVertexOne( const std::vector<RigidBody>& rbs )
{
  return (m_rb0==-1) ? m_vc0 : rbs[m_rb0].computeWorldSpacePosition(m_vc0);
}

Vector2s SVGRigidBodySpringForceRenderer::getVertexTwo( const std::vector<RigidBody>& rbs )
{
  return (m_rb1==-1) ? m_vc1 : rbs[m_rb1].computeWorldSpacePosition(m_vc1);
}

renderingutils::Color SVGRigidBodySpringForceRenderer::getColor()
{
  return m_color;
}
