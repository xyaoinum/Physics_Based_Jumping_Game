#include "MonopodRobotController.h"
#include "MonopodRobotControllerStrategy.h"

MonopodRobotController::MonopodRobotController(JointMotorBodyScene * scene, MonopodRobot * robot) :
  JointMotorBodyController(scene, robot),
  m_robot(robot),
  m_strategy(NULL)
{
  
}

void MonopodRobotController::computeForceAndTorque(std::vector<RigidBody> & rbs)
{
  if (m_strategy == NULL) {
    m_strategy = new MonopodRobotControllerStrategy();
  }
  MonopodRobotControllerStrategy::Action action = m_strategy->generateAction(m_scene, m_robot);
  m_robot->setLegSpringRestLength(action.legSpringRestLength);
  
  // apply leg spring force
  m_robot->legSpringForce()->computeForceAndTorque(rbs);
  
  // apply joint torques
  RigidBody * headrb = m_robot->rigidBody(m_robot->head());
  RigidBody * footrb = m_robot->rigidBody(m_robot->foot());
  Vector2s joint_world = headrb->computeWorldSpacePosition(m_robot->foot()->axis_in_parent_body_space);
  
  scalar tau = action.legTorque;
  
  Vector2s head_c = joint_world - headrb->getX();
  Vector2s head_c_perp(-head_c.y(), head_c.x());
  Vector2s foot_c = joint_world - footrb->getX();
  Vector2s foot_c_perp(-foot_c.y(), foot_c.x());
  Vector2s leg_c = footrb->getX() - headrb->getX();
  Vector2s leg_c_perp(-leg_c.y(), leg_c.x());
  
  Vector2s f_head;
  Vector2s f_foot;
  scalar tau_head;
  scalar tau_foot;
  
  if (footrb->getI() > 0)
  {
    MatrixXs A = MatrixXs::Zero(3, 3);  // dofs: f_head, tau_head (f_foot is not free due to conservation of momentum; tau_foot is not free due to conservation of angular momemtum)
    VectorXs rhs = VectorXs::Zero(3);   // equations: 1. zero relative acceleration at the joint, 2. tau application on head
    
    A.block<2, 2>(0, 0) = Matrix2s::Identity() * (1 / headrb->getM() + 1 / footrb->getM()) - foot_c_perp * leg_c_perp.transpose() / footrb->getI();
    A.block<2, 1>(0, 2) = head_c_perp / headrb->getI() + foot_c_perp / footrb->getI();
    A.block<1, 2>(2, 0) = head_c_perp;
    A(2, 2) = -1;
    
    rhs(0) = 0;
    rhs(1) = 0;
    rhs(2) = tau;
    
    VectorXs x = A.fullPivLu().solve(rhs);
    f_head = x.segment<2>(0);
    f_foot = -f_head;
    tau_head = x(2);
    tau_foot = tau_head - leg_c_perp.dot(f_head);
  } else
  {
    // zero moment of inertia, tau_foot is no longer a dof
    MatrixXs A = MatrixXs::Zero(3, 2);  // dofs: f_head (f_foot is not free due to conservation of momentum; tau_foot is zero; tau_head is not free due to conservation of angular momentum)
    VectorXs rhs = VectorXs::Zero(3);   // equations: 1. zero relative acceleration at the joint, 2. tau application on head and 3. tau application on foot
    
    A.block<2, 2>(0, 0) = Matrix2s::Identity() * (1 / headrb->getM() + 1 / footrb->getM()) + head_c_perp * leg_c_perp.transpose() / headrb->getI();
    A.block<1, 2>(2, 0) = foot_c_perp;

    rhs(0) = 0;
    rhs(1) = 0;
    rhs(2) = tau;
    
//    std::cout << "A = \n" << A << std::endl << "rhs = " << rhs.transpose() << std::endl << "ATA = \n" << A.transpose() * A << std::endl << " ATrhs = " << rhs.transpose() * A << std::endl << "x = " << (A.transpose()  * A).fullPivLu().solve(A.transpose() * rhs);
    
    // approximate solve, since it's overdetermined (tau_foot is no longer a dof)
    VectorXs x = (A.transpose() * A).fullPivLu().solve(A.transpose() * rhs);
    f_head = x;
    f_foot = -f_head;
    tau_head = leg_c_perp.dot(f_head);
    tau_foot = 0;
//    std::cout << "fh = " << f_head.transpose() << "\nff = " << f_foot.transpose() << "\nth = " << tau_head << std::endl << "tf = " << tau_foot << std::endl;
  }
  
  headrb->getForce() += f_head;
  headrb->getTorque() += tau_head;
  footrb->getForce() += f_foot;
  footrb->getTorque() += tau_foot;
  
//  Vector2s head_c = joint_world - headrb->getX();
//  Vector2s head_c_perp(-head_c.y(), head_c.x());
//  scalar head_denom = headrb->getI() + headrb->getM() * head_c.squaredNorm();
//  headrb->getForce() += headrb->getM() * head_c_perp * tau / head_denom;
//  headrb->getTorque() -= headrb->getI() * tau / head_denom;
  
//  Vector2s foot_c = joint_world - footrb->getX();
//  Vector2s foot_c_perp(-foot_c.y(), foot_c.x());
//  scalar foot_denom = footrb->getI() + footrb->getM() * foot_c.squaredNorm();
//  footrb->getForce() -= footrb->getM() * foot_c_perp * tau / foot_denom;
//  footrb->getTorque() += footrb->getI() * tau / foot_denom;

//  std::cout << "headrb force = " << headrb->getForce().transpose() << " torque = " << headrb->getTorque() << std::endl;
//  std::cout << "footrb force = " << footrb->getForce().transpose() << " torque = " << footrb->getTorque() << std::endl;
  
  // head rotation damping
  headrb->getTorque() -= headrb->getOmega() * headrb->getI() * 0.3;
}
