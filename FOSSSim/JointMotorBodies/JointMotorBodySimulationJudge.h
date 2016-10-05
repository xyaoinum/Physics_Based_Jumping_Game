#ifndef __JOINT_MOTOR_BODY_SIMULATION_JUDGE_H__
#define __JOINT_MOTOR_BODY_SIMULATION_JUDGE_H__

#include "JointMotorBodyScene.h"
#include "MonopodRobot.h"
#include <sstream>

class JointMotorBodySimulationJudge
{
public:
  class Detector
  {
  public:
    Detector(int index) :
      m_index(index),
      m_state(false),
      m_fulfilled(false),
      m_lastfulfilled(false),
      m_display(false),
      m_log(""),
      m_progress(0),
      m_frozenProgress(0),
      m_alpha(0)
    { }
    
    int index() const { return m_index; }
    
    void setFailure() { m_state = false; }
    void setSuccess() { m_state = true; }
    bool success() const { return m_state; }
    
    void setFulfilled() { m_lastfulfilled = m_fulfilled; m_fulfilled = true; }
    void setViolated()  { m_lastfulfilled = m_fulfilled; m_fulfilled = false; }
    bool fulfilled() const { return m_fulfilled; }
    bool lastFulfilled() const { return m_lastfulfilled; }
    
    void show(bool s) { m_display = s; }
    bool visible() const { return m_display; }
    
    scalar & progress() { return m_progress; }
    scalar & frozenProgress() { return m_frozenProgress; }
    scalar & alpha() { return m_alpha; }
    
    virtual void update(JointMotorBodyScene * scene, scalar dt) = 0;
    virtual void renderProgress() = 0;
    virtual void renderCue() = 0;
    virtual Matrix2s cueBoundingBox() = 0;
    
    std::string log() { return m_log.str(); }
    
    virtual std::string detailedDescription() const = 0;
    virtual std::string briefDescription() const = 0;
    virtual std::string buttonCaption() const = 0;
    
    void renderProgressIndicator(scalar percentage, bool good, scalar alpha);
    
  protected:
    const int m_index;
    bool m_state;
    bool m_fulfilled;
    bool m_lastfulfilled;
    bool m_display;
    std::stringstream m_log;
    scalar m_progress;
    scalar m_frozenProgress;
    scalar m_alpha;
  };

  // Monopod robot detectors:
  class MonopodRobotDetector : public Detector
  {
  public:
    MonopodRobotDetector(int index, MonopodRobot * robot) : Detector(index), m_robot(robot) { }

  protected:
    MonopodRobot * m_robot;
  };
  
  // detector: jump heights (local maxima in height over time) never get below a threshold
  class MR_JumpingDetector : public MonopodRobotDetector
  {
  public:
    MR_JumpingDetector(int index, MonopodRobot * robot, scalar ymin, scalar ymax, int njump) : MonopodRobotDetector(index, robot), m_ymin(ymin), m_ymax(ymax), m_njump(njump), m_peak(0), m_peak_recorded(false), m_touch_down_started(false) { }
    void update(JointMotorBodyScene * scene, scalar dt);
    void renderProgress();
    void renderCue();
    Matrix2s cueBoundingBox();
    std::string detailedDescription() const;
    std::string briefDescription() const;
    std::string buttonCaption() const { return buttonString; }
    scalar ymin() const { return m_ymin; }
    scalar ymax() const { return m_ymax; }
    int njump() const { return m_njump; } 
    void set_ymin(scalar y_min) {m_ymin = y_min;}
    void set_button(std::string s){
        buttonString = s;
    }
  
  protected:
        std::string buttonString;
    scalar m_ymin;
    scalar m_ymax;
    int m_njump;
    scalar m_peak;
    bool m_peak_recorded;
    bool m_touch_down_started;
  };
  
  // detector: land in a specified range of x position
  class MR_LandingDetector : public MonopodRobotDetector
  {
  public:
    MR_LandingDetector(int index, MonopodRobot * robot, scalar xmin, scalar xmax, scalar duration) : MonopodRobotDetector(index, robot), m_xmin(xmin), m_xmax(xmax), m_duration(duration) { }
    void update(JointMotorBodyScene * scene, scalar dt);
    void renderProgress();
    void renderCue();
    Matrix2s cueBoundingBox();
    std::string detailedDescription() const;
    std::string briefDescription() const;
    std::string buttonCaption() const { return "Landing"; }
    scalar xmin() const { return m_xmin; }
    scalar xmax() const { return m_xmax; }
    scalar duration() const { return m_duration; } 

  protected:
    scalar m_xmin;
    scalar m_xmax;
    scalar m_duration;
  };
  
  // detector: prevent the robot head from touching ground
  class MR_BalancingDetector : public MonopodRobotDetector
  {
  public:
    MR_BalancingDetector(int index, MonopodRobot * robot, scalar duration) : MonopodRobotDetector(index, robot), m_duration(duration) { if (duration < 0) setSuccess(); }
    void update(JointMotorBodyScene * scene, scalar dt);
    void renderProgress();
    void renderCue();
    Matrix2s cueBoundingBox();
    std::string detailedDescription() const;
    std::string briefDescription() const;
    std::string buttonCaption() const { return "Balancing"; }
    scalar duration() const { return m_duration; } 

  protected:
    scalar m_duration;
  };
  
  // detector: achieve a specified range of velocity
  class MR_VelocityDetector : public MonopodRobotDetector
  {
  public:
    MR_VelocityDetector(int index, MonopodRobot * robot, scalar vmin, scalar vmax, scalar duration) : MonopodRobotDetector(index, robot), m_vmin(vmin), m_vmax(vmax), m_duration(duration) { }
    void update(JointMotorBodyScene * scene, scalar dt);
    void renderProgress();
    void renderCue();
    Matrix2s cueBoundingBox();
    std::string detailedDescription() const;
    std::string briefDescription() const;
    std::string buttonCaption() const { return "Velocity"; }
    scalar vmin() const { return m_vmin; }
    scalar vmax() const { return m_vmax; }
    scalar duration() const { return m_duration; } 
    
  protected:
    scalar m_vmin;
    scalar m_vmax;
    scalar m_duration;
  };
  
  // detector: find the treat
  class MR_TreatDetector : public MonopodRobotDetector
  {
  public:
    MR_TreatDetector(int index, MonopodRobot * robot, int treatrb) : MonopodRobotDetector(index, robot), m_treatrb(treatrb) { }
    void update(JointMotorBodyScene * scene, scalar dt);
    void renderProgress();
    void renderCue();
    Matrix2s cueBoundingBox();
    std::string detailedDescription() const;
    std::string briefDescription() const;
    std::string buttonCaption() const { return "Treat"; }
    int treatrb() const { return m_treatrb; }
    
  protected:
    int m_treatrb;
  };
  
public:
  JointMotorBodySimulationJudge();
  
  void updateDetectors(JointMotorBodyScene * scene, scalar dt);
  void render();
  
  int result(); // result = 0: undetermined (not satisfied); result = 1: success; result = 2: failure
  void report(std::ostream & os);
  
  void addDetector(Detector * d) { if (!m_finalized) m_detectors.push_back(d); }
  void finalize() { m_finalized = true; }
  
  int ndetectors() const { return m_detectors.size(); }
   Detector * detector(int i)  { return m_detectors[i]; }
  
  void mouse(int button, int state, int x, int y);
  void motion(int x, int y);
  void passiveMotion(int x, int y);
  bool overButton(int x, int y, int i);
  
  void getFeatures(std::vector<Matrix2s> & features);
  
  void showAll(bool s) { for (size_t i = 0; i < m_detectors.size(); i++) m_detectors[i]->show(s); }
  
protected:
  std::vector<Detector *> m_detectors;
  bool m_finalized;
  
  bool m_lb;
  int m_buttonover;
};

typedef JointMotorBodySimulationJudge::MR_JumpingDetector JumpingDetector;
typedef JointMotorBodySimulationJudge::MR_TreatDetector TreatDetector;
typedef JointMotorBodySimulationJudge::MR_LandingDetector LandingDetector;
typedef JointMotorBodySimulationJudge::MR_VelocityDetector VelocityDetector;
typedef JointMotorBodySimulationJudge::MR_BalancingDetector BalancingDetector;

#endif
