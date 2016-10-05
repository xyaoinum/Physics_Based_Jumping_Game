#include "RigidBodyGroundCollisionDetector.h"

void RigidBodyGroundCollisionDetector::detectCollisions(  std::vector<RigidBody>& rbs, std::set<RigidBodyCollision>& collisions )
{
    // Compute all vertex-edge collisions between all pairs of rigid bodies.
    collisions.clear();
    Vector2s n = Vector2s(0, -1);
    for (int i = 0; i < (int)rbs.size(); i++)
    {
        const RigidBody & rb = rbs[i];
        for (int k = 0; k < (int)rb.getNumVertices(); k++)
        {
            Vector2s p = rb.getWorldSpaceVertex(k);
            scalar r = rb.getRadius();
            if (p.y() - r < 0)
            {
                collisions.insert(RigidBodyCollision(i, -1, rb.computeBodySpacePosition(p + n * r), Vector2s::Zero(), n));
            }
        }
    }
}

std::string RigidBodyGroundCollisionDetector::getName() const
{
    return "ground";
}


