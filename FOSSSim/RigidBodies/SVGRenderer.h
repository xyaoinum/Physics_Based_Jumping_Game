#ifndef __SVG_RENDERER_H__
#define __SVG_RENDERER_H__

#include "FOSSSim/RenderingUtilities.h"
#include "RigidBody.h"

class SVGRenderer
{

public:

  virtual void render( const std::vector<RigidBody>& rbs, std::fstream& file ) = 0;

  // TODO: Get rid of these
  // Ugh this is ugly. But the assignment goes out in 10 hours and we need to get
  // svg rendering up for spring forces.
  virtual Vector2s getVertexOne(const std::vector<RigidBody>& rbs) = 0;
  virtual Vector2s getVertexTwo(const std::vector<RigidBody>& rbs) = 0;
  virtual renderingutils::Color getColor() = 0;
};

#endif
