#ifndef __OPENGL_RENDERER_H__
#define __OPENGL_RENDERER_H__

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include "RigidBody.h"

class OpenGLRenderer
{

public:

  virtual void render( const std::vector<RigidBody>& rbs ) = 0;

};

#endif
