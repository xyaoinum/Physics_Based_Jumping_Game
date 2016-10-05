#ifndef __SVG_RIGID_BODY_SPRING_FORCE_RENDERER_H__
#define __SVG_RIGID_BODY_SPRING_FORCE_RENDERER_H__

#include "FOSSSim/RenderingUtilities.h"
#include "SVGRenderer.h"
#include "RigidBodySpringForce.h"

class SVGRigidBodySpringForceRenderer : public SVGRenderer
{

public:

  SVGRigidBodySpringForceRenderer( const RigidBodySpringForce& rbspfrc, const renderingutils::Color& color );

  virtual void render( const std::vector<RigidBody>& rbs, std::fstream& file );

  virtual Vector2s getVertexOne(const std::vector<RigidBody>& rbs);
  virtual Vector2s getVertexTwo(const std::vector<RigidBody>& rbs);
  
  virtual renderingutils::Color getColor();
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  renderingutils::Color m_color;
  
  // Instead just mirror the state from the spring. A little memory inefficient, but easy.
  int m_rb0;
  int m_rb1;
  Vector2s m_vc0;
  Vector2s m_vc1;

  // Ran into problems here when copying RigidBodyScenes. In copying the scenes, the 
  //  address to forces chagned. But the renderers are in RigidBodySimulation. Thus,
  //  to use a reference to a force here I need some way to push the changes, which gets messy...
  //const RigidBodySpringForce& m_rbspfrc;
  
};

#endif
