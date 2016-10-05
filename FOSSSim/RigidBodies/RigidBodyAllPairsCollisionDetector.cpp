#include "RigidBodyAllPairsCollisionDetector.h"


bool detectParticleEdge(
        const Vector2s & x1, 
        const Vector2s & x2, 
        const Vector2s & x3, 
        const Vector2s & v1, 
        const Vector2s & v2,
        const Vector2s & v3, 
        scalar r, 
        Vector2s & closest, 
        Vector2s & n)
{
    double alpha = (x1-x2).dot(x3-x2)/(x3-x2).dot(x3-x2);
    alpha = std::min(1.0, std::max(0.0, alpha));
    
    closest = x2 + alpha*(x3-x2);
    n = closest-x1;
    
    //std::cout << n.norm() << " " << r << " " << (n.norm() < r) << " " << (n.norm() - r);
    if(n.norm() < r) 
    {
        //std::cout << "check" << std::endl;
        //double relvel = (v1 - v2 - alpha*(v3-v2)).dot(n);
        //if(relvel > 0)
        {
            return true;
        }
    }
    //std::cout << std::endl;
    return false;
}

bool detectRigidBodyPE(int i, int j,  RigidBody & rb1,  RigidBody & rb2, std::set<RigidBodyCollision> & collisions)
{
    bool result = false;
    for (int k = 0; k < (int)rb1.getNumVertices(); k++)
    {
        Vector2s par = rb1.getWorldSpaceVertex(k);
        Vector2s vpar = rb1.computeWorldSpaceVelocity(par);
        for (int l = 0; l < (int)rb2.getNumVertices(); l++)
        {
            Vector2s e1 = rb2.getWorldSpaceVertex(l);
            Vector2s e2 = rb2.getWorldSpaceVertex((l + 1) % rb2.getNumVertices());
            Vector2s ve1 = rb2.computeWorldSpaceVelocity(e1);
            Vector2s ve2 = rb2.computeWorldSpaceVelocity(e2);
            Vector2s closest;
            Vector2s n;
            scalar r1 = rb1.getRadius();
            scalar r2 = rb2.getRadius();
            //std::cout << "[" << i << ", " << j << "] (" << k << " " << l << ") ";
            if (detectParticleEdge(par, e1, e2, vpar, ve1, ve2, r1 + r2, closest, n))
            {
                n.normalize();
                rb1.set_collision(true);
                rb2.set_collision(true);
                rb1.has_touched = true;
                rb2.has_touched = true;
                result = true;
                collisions.insert(RigidBodyCollision(i, j, rb1.computeBodySpacePosition(par + n * r1), rb2.computeBodySpacePosition(closest - n * r2), n));
            }
        }
    }
    return result;
}

void RigidBodyAllPairsCollisionDetector::detectCollisions(std::vector<RigidBody>& rbs, std::set<RigidBodyCollision>& collisions )
{
    // Your code goes here! 
    // Compute all vertex-edge collisions between all pairs of rigid bodies.
    
    //std::cout << "dsjklfjslkdfj" << std::endl;

    collisions.clear();
    Vector2s n = Vector2s(0, -1);
    
    
    for (int i = 0; i < (int)rbs.size(); i++)
    {
        rbs[i].set_collision(false);
    }
    
    
    for (int i = 0; i < (int)rbs.size(); i++)
    {
        RigidBody & rb = rbs[i];
        for (int k = 0; k < (int)rb.getNumVertices(); k++)
        {
            Vector2s p = rb.getWorldSpaceVertex(k);
            scalar r = rb.getRadius();
            if (p.y() - r < 0)
            {
                rb.set_collision(true);
                collisions.insert(RigidBodyCollision(i, -1, rb.computeBodySpacePosition(p + n * r), Vector2s::Zero(), n));
            }
        }
    }
    
    
    for (int i = (int)rbs.size()-1; i >=0; i--)
    {
        //rbs[i].set_collision(false);
        //std::cout << rbs[i].getV().transpose() << std::endl;
        for (int j = i-1; j >=0; j--)
        {
            if(!rbs[i].get_collision_enable() || !rbs[j].get_collision_enable()){
                continue;
            }
            detectRigidBodyPE(i, j, rbs[i], rbs[j], collisions);
            detectRigidBodyPE(j, i, rbs[j], rbs[i], collisions);
        }
        /*
        if(rbs[i].get_collision() == false){
            if(rbs[i].is_fragile == -1 && rbs[i].has_touched == true){
                rbs[i].is_fragile = 1;
            }
        }*/
    }
}

std::string RigidBodyAllPairsCollisionDetector::getName() const
{
    return "all-pairs";
}


