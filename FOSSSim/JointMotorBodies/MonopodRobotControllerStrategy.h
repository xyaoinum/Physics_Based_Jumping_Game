#ifndef __MONOPOD_ROBOT_CONTROLLER_STRATEGY_H__
#define __MONOPOD_ROBOT_CONTROLLER_STRATEGY_H__

#include "MonopodRobot.h"
#include "JointMotorBodyScene.h"
#include "JointMotorBodySimulationJudge.h"
#include "JointMotorBodySimulation.h"

extern ExecutableSimulation * g_executable_simulation;

template<typename T>
std::vector< T*> getDetectorsOfType() {
   JointMotorBodySimulationJudge * judge = (dynamic_cast<JointMotorBodySimulation *>(g_executable_simulation))->judge();
  std::vector< T*> result;
  for (int i = 0; i < judge->ndetectors(); i++)
  {
    if ( T* d = dynamic_cast< T*>(judge->detector(i))) {
      result.push_back(d);
    }
  }

  return result;
}


class MonopodRobotControllerStrategy
{
public:
  struct Action
  {
    scalar legSpringRestLength;
    scalar legTorque;
  };

  MonopodRobotControllerStrategy() : 
    jumping_detectors(getDetectorsOfType<JumpingDetector>()),
    landing_detectors(getDetectorsOfType<LandingDetector>()),
    balancing_detectors(getDetectorsOfType<BalancingDetector>()),
    velocity_detectors(getDetectorsOfType<VelocityDetector>()),
    treat_detectors(getDetectorsOfType<TreatDetector>())
  {}
  
  // the detectors to be satisfied
  std::vector< JumpingDetector*>   jumping_detectors;
  std::vector< LandingDetector*>   landing_detectors;
  std::vector< BalancingDetector*> balancing_detectors; 
  std::vector< VelocityDetector*>  velocity_detectors; 
  std::vector< TreatDetector*>     treat_detectors;

  Action generateAction(JointMotorBodyScene * scene, MonopodRobot * robot);
};

#endif
