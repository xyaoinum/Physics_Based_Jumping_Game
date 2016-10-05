#ifndef __OPENGL_RIGID_BODY_SCHEDULED_IMPULSE_RENDERER_H__
#define __OPENGL_RIGID_BODY_SCHEDULED_IMPULSE_RENDERER_H__

#include "FOSSSim/RenderingUtilities.h"
#include "../RigidBodies/OpenGLRenderer.h"
#include "JointMotorBodySimulation.h"

class OpenGLRigidBodyScheduledImpulseRenderer : public OpenGLRenderer
{

public:

  OpenGLRigidBodyScheduledImpulseRenderer( const renderingutils::Color& color, JointMotorBodySimulation::ScheduledPushEvent & push );

  virtual void render( const std::vector<RigidBody>& rbs );

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  JointMotorBodySimulation::ScheduledPushEvent & m_push;
  renderingutils::Color m_color;
  
};

#endif
