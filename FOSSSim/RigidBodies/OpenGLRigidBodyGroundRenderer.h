#ifndef __OPENGL_RIGID_BODY_GROUND_RENDERER_H__
#define __OPENGL_RIGID_BODY_GROUND_RENDERER_H__

#include "FOSSSim/RenderingUtilities.h"
#include "OpenGLRenderer.h"
#include "RigidBodySpringForce.h"

class OpenGLRigidBodyGroundRenderer : public OpenGLRenderer
{

public:

  OpenGLRigidBodyGroundRenderer( const renderingutils::Color& color );

  virtual void render( const std::vector<RigidBody>& rbs );

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  renderingutils::Color m_color;
  
};

#endif
