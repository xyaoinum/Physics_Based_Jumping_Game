#include "RigidBodyExplicitEuler.h"

RigidBodyExplicitEuler::~RigidBodyExplicitEuler()
{
}

bool RigidBodyExplicitEuler::stepScene( RigidBodyScene& scene, scalar dt )
{
  // Clear any previously computed forces in the rigid bodies
  std::vector<RigidBody>& rbs = scene.getRigidBodies();
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i ) rbs[i].getForce().setZero();
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i ) rbs[i].getTorque() = 0.0;
  
  // Add each force's contribution to each rigid body using previous step's state
  std::vector<RigidBodyForce*>& frcs = scene.getForces();
  for( std::vector<RigidBodyForce*>::size_type i = 0; i < frcs.size(); ++i ) frcs[i]->computeForceAndTorque(rbs);

  // For each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
  {
    if( rbs[i].isFixed() ) continue; 

    // Update center of mass position using previous step's velocity and angular velocity
    rbs[i].getX() += dt*rbs[i].getV();
    
    // Update the orientation using the previous step's angular momentum
    rbs[i].getTheta() += dt*rbs[i].getOmega();

    // Update the center of mass velocity using the force evaulated at the previous step's state
    rbs[i].getV() += dt*rbs[i].getForce()/rbs[i].getM();
    
    // Update the angular velocity using the torque evaluated at the previous step's state
    if (rbs[i].getI() > 0)
      rbs[i].getOmega() += dt*rbs[i].getTorque()/rbs[i].getI();
    
    // Update quantities derived from state
    rbs[i].updateDerivedQuantities();
  }

  return true;
}

std::string RigidBodyExplicitEuler::getName() const
{
  return "Rigid Body Explicit Euler";
}
