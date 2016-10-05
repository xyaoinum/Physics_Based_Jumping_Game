#include "JointMotorBodySimulationJudge.h"
#include "../TwoDimensionalDisplayController.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

extern TwoDimensionalDisplayController g_display_controller;
extern bool g_rendering_enabled;

void renderBitmapString( float x, float y, float z, void *font, std::string s, int align = 0 );


JointMotorBodySimulationJudge::JointMotorBodySimulationJudge() : 
    m_finalized(false),
    m_lb(false),
    m_buttonover(-1)
{
    
}

void JointMotorBodySimulationJudge::updateDetectors(JointMotorBodyScene * scene, scalar dt)
{
    for (size_t i = 0; i < m_detectors.size(); i++)
        m_detectors[i]->update(scene, dt);
    
    if (g_rendering_enabled)
        glutPostRedisplay();
}

void JointMotorBodySimulationJudge::render()
{
    glLineWidth(1);
    
    // world space cues
    for (size_t i = 0; i < m_detectors.size(); i++)
        if (m_detectors[i]->visible())
            m_detectors[i]->renderCue();
    
    // HUD
    int w = g_display_controller.getWindowWidth();
    int h = g_display_controller.getWindowHeight();
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, w, 0, h, -1, 1);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glDisable(GL_DEPTH_TEST);
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    for (size_t i = 0; i < m_detectors.size(); i++)
    {
        Detector * d = m_detectors[i];
        
        // mouse over indicator
        if (m_buttonover == (int)i)
        {
            if (m_lb)
                glColor4f(0.0, 0.0, 1.0, 0.4);
            else
                glColor4f(0.0, 0.0, 1.0, 0.1);
            
            glBegin(GL_QUADS);
            glVertex2d(w - 125, h - 5  - 40 * i);
            glVertex2d(w - 5,   h - 5  - 40 * i);
            glVertex2d(w - 5,   h - 40 - 40 * i);
            glVertex2d(w - 125, h - 40 - 40 * i);
            glEnd();
        }
        
        // progress indicator
        if (d->success())
        {
            // nothing to do, detector already golden
        } else if (dynamic_cast<MR_BalancingDetector *>(d) && dynamic_cast<MR_BalancingDetector *>(d)->duration() < 0)
        {
            // passive balancing detector, no progress indicator
        } else
        {
            if (d->fulfilled())
            {
                // requirement fulfilled for now
                d->renderProgressIndicator(d->progress(), true, 1.0);
                d->frozenProgress() = d->progress();
            } else
            {
                if (d->lastFulfilled())
                {
                    // just switched to violated state from fulfilled state
                    d->alpha() = 1.0;
                    d->renderProgressIndicator(d->frozenProgress(), false, d->alpha());
                    glutPostRedisplay();
                    d->setViolated();
                } else
                {
                    // not newly violated; indicator keeps fading
                    d->alpha() -= 0.0003;
                    if (d->alpha() > 0)
                    {
                        d->renderProgressIndicator(d->frozenProgress(), false, d->alpha());
                        glutPostRedisplay();  // need redisplay here because the indicator needs to fade even if there's nothing else going on
                    }
                }
            }
        }
    }
    glDisable(GL_BLEND);
    
    for (size_t i = 0; i < m_detectors.size(); i++)
    {
        // button caption
        if (m_detectors[i]->success())
            glColor3f(0, 0.8, 0);
        else
            glColor3f(1, 0, 0);
        
        renderBitmapString(w - 65, h - 27 - 40 * i, 0.0, GLUT_BITMAP_HELVETICA_18, m_detectors[i]->buttonCaption(), 2);
        
        // detector visible indicator
        if (m_detectors[i]->visible())
        {
            glColor3f(0.6, 0, 1);
            glBegin(GL_LINES);
            glVertex2d(w - 125, h - 5  - 40 * i);
            glVertex2d(w - 5,   h - 5  - 40 * i);
            glVertex2d(w - 125, h - 40 - 40 * i);
            glVertex2d(w - 5,   h - 40 - 40 * i);
            glVertex2d(w - 5,   h - 5  - 40 * i);
            glVertex2d(w - 5,   h - 40 - 40 * i);
            glVertex2d(w - 125, h - 5  - 40 * i);
            glVertex2d(w - 125, h - 40 - 40 * i);
            glEnd();
        }
    }
    
    //  glEnable(GL_DEPTH_TEST);
    
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

int JointMotorBodySimulationJudge::result()
{
    for (size_t i = 0; i < m_detectors.size(); i++)
    {
        MR_BalancingDetector * d = dynamic_cast<MR_BalancingDetector *>(m_detectors[i]);
        if (d && d->duration() < 0 && !d->success())
            return 2; // doomed failure
    }
    
    for (size_t i = 0; i < m_detectors.size(); i++)
        if (!m_detectors[i]->success())
            return 0; // unsatisfied but not determined yet
    
    return 1;   // success
}

void JointMotorBodySimulationJudge::report(std::ostream & os)
{
    size_t maxlen = 0;
    for (size_t i = 0; i < m_detectors.size(); i++)
        if (m_detectors[i]->briefDescription().size() > maxlen)
            maxlen = m_detectors[i]->briefDescription().size();
    
    for (size_t i = 0; i < m_detectors.size(); i++)
        std::cout << "[" << i << "] " << m_detectors[i]->briefDescription() << std::string(maxlen - m_detectors[i]->briefDescription().size(), ' ') << ": " << (m_detectors[i]->success() ? "Success" : "Failure") << std::endl;  
}

void JointMotorBodySimulationJudge::mouse(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        // press
        m_lb = true;
        
    } else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
    {
        // release
        m_lb = false;
        if (m_buttonover >= 0)
            m_detectors[m_buttonover]->show(!m_detectors[m_buttonover]->visible());
    }
    glutPostRedisplay();
}

void JointMotorBodySimulationJudge::motion(int x, int y)
{
    
}

void JointMotorBodySimulationJudge::passiveMotion(int x, int y)
{
    int old = m_buttonover;
    m_buttonover = -1;
    
    for (size_t i = 0; i < m_detectors.size(); i++)
        if (overButton(x, y, i))
            m_buttonover = i;
    
    if (m_buttonover != old)
        glutPostRedisplay();
}

bool JointMotorBodySimulationJudge::overButton(int x, int y, int i)
{
    int w = g_display_controller.getWindowWidth();
    int h = g_display_controller.getWindowHeight();
    
    if (x >= w - 125 && x <= w - 5 && y >= 5 + 40 * i && y <= 40 + 40 * i)
        return true;
    else
        return false;
}

void JointMotorBodySimulationJudge::getFeatures(std::vector<Matrix2s> & features)
{
    for (size_t i = 0; i < m_detectors.size(); i++)
        features.push_back(m_detectors[i]->cueBoundingBox());
}







void JointMotorBodySimulationJudge::Detector::renderProgressIndicator(scalar percentage, bool good, scalar alpha)
{
    //  std::cout << "indicator for detector " << index() << ": " << (good ? "good" : "bad") << ", " << percentage * 100 << "%" << std::endl;
    //  
    const float colors[2][2][4] =   // good: done (rgba), rest (rgba); bad: done (rgba), rest (rgba)
    {
    { { 0.0, 0.3, 0.6, 1.0 }, { 0.0, 0.7, 1.0, 1.0 } },
    { { 0.7, 0.1, 0.1, 1.0 }, { 1.0, 0.4, 0.4, 1.0 } }
};
    const int N = 36;
    
    int w = g_display_controller.getWindowWidth();
    int h = g_display_controller.getWindowHeight();
    scalar xc = w - 150;
    scalar yc = h - 22 - 40 * index();
    scalar r = 19;
    
    if (good) glColor4f(colors[0][0][0], colors[0][0][1], colors[0][0][2], colors[0][0][3] * alpha);
    else      glColor4f(colors[1][0][0], colors[1][0][1], colors[1][0][2], colors[1][0][3] * alpha);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2d(xc, yc);
    for (int i = 0; i <= (int)(N * percentage); i++)
    {
        glVertex2d(xc + r * cos(-(scalar)i / N * PI * 2 + PI / 2), yc + r * sin(-(scalar)i / N * PI * 2 + PI / 2));
    }
    glVertex2d(xc + r * cos(-percentage * PI * 2 + PI / 2), yc + r * sin(-percentage * PI * 2 + PI / 2));
    glEnd();
    
    if (good) glColor4f(colors[0][1][0], colors[0][1][1], colors[0][1][2], colors[0][1][3] * alpha);
    else      glColor4f(colors[1][1][0], colors[1][1][1], colors[1][1][2], colors[1][1][3] * alpha);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2d(xc, yc);
    glVertex2d(xc + r * cos(-percentage * PI * 2 + PI / 2), yc + r * sin(-percentage * PI * 2 + PI / 2));
    for (int i = (int)(N * percentage) + 1; i <= N; i++)
    {
        glVertex2d(xc + r * cos(-(scalar)i / N * PI * 2 + PI / 2), yc + r * sin(-(scalar)i / N * PI * 2 + PI / 2));
    }
    glEnd();
    
}

void JointMotorBodySimulationJudge::MR_JumpingDetector::update(JointMotorBodyScene * scene, scalar dt)
{
    if (success())
        return;
 
    RigidBody * foot = m_robot->rigidBody(m_robot->foot());
    scalar y = foot->getX().y();
    
    if(y >= m_ymin){
        setFulfilled();
        setSuccess();
    }else{
        setViolated();
    }
    return;
    /*
    if (y > m_peak)
    {
        m_peak = y;
    } else
    {
        if (!m_peak_recorded)
        {
            m_peak_recorded = true;
            if (y >= m_ymin && y <= m_ymax)
            {
                setFulfilled();
                m_progress += 1.001 / m_njump;
                if (m_progress >= 1.0)
                    setSuccess();
            } else
            {
                setViolated();
                m_progress = 0;
            }
        }
    }
    
    RigidBody * foot = m_robot->rigidBody(m_robot->foot());
    if (foot->getX().y() - foot->getRadius() * 1.5 < 0) // foot touching ground
    {
        if (head->getV().y() < 0) // downward velocity (incoming)
        {
            m_touch_down_started = true;
        }
        
        if (head->getV().y() > 0 && m_touch_down_started) // upward velocity (departing)
        {
            m_peak = 0;
            m_peak_recorded = false;
            m_touch_down_started = false;
        }
    }
    */
}

void JointMotorBodySimulationJudge::MR_JumpingDetector::renderProgress()
{
    
}

void JointMotorBodySimulationJudge::MR_JumpingDetector::renderCue()
{
    glBegin(GL_LINES);
    if (success())
        glColor3f(0, 0.8, 0);
    else if (m_robot->rigidBody(m_robot->head())->getX().y() >= m_ymin)
        glColor3f(0.6, 0.0, 1.0);
    else
        glColor3f(1.0, 0, 0);
    glVertex2d(-1000, m_ymin);
    glVertex2d(1000, m_ymin);
    if (success())
        glColor3f(0, 0.8, 0);
    else if (m_robot->rigidBody(m_robot->head())->getX().y() <= m_ymax)
        glColor3f(0.6, 0.0, 1.0);
    else
        glColor3f(1.0, 0, 0);
    glVertex2d(-1000, m_ymax);
    glVertex2d(1000, m_ymax);
    glEnd();
}

Matrix2s JointMotorBodySimulationJudge::MR_JumpingDetector::cueBoundingBox()
{
    Matrix2s bb;
    bb.col(0) = Vector2s(m_robot->rigidBody(m_robot->head())->getX().x(), m_ymin);
    bb.col(1) = Vector2s(m_robot->rigidBody(m_robot->head())->getX().x(), m_ymax);
    return bb;
}

std::string JointMotorBodySimulationJudge::MR_JumpingDetector::detailedDescription() const
{
    std::stringstream ss;
    ss << "Use keyboard WASD to control the monopod! The goal is to reach the highest block within limited time";
    //ss << "Jumping Detector: the peak height of the jumps (bounce from the ground) must be between " << m_ymin << " and " << m_ymax << " for at least " << m_njump << " consecutive jumps.";
    return ss.str();
}

std::string JointMotorBodySimulationJudge::MR_JumpingDetector::briefDescription() const
{
    std::stringstream ss;
    ss << "Game Result: ";// << ymin = " << m_ymin << " ymax = " << m_ymax << " njump = " << m_njump;
    return ss.str();
}

void JointMotorBodySimulationJudge::MR_LandingDetector::update(JointMotorBodyScene * scene, scalar dt)
{
    if (success())
        return;
    
    RigidBody * head = m_robot->rigidBody(m_robot->head());
    bool fulfilled = (head->getX().x() >= m_xmin && head->getX().x() <= m_xmax);
    
    if (fulfilled)
    {
        setFulfilled();
        m_progress += dt / m_duration;
        if (m_progress >= 1.0)
            setSuccess();
    } else
    {
        setViolated();
        m_progress = 0;
    }
    
}

void JointMotorBodySimulationJudge::MR_LandingDetector::renderProgress()
{
    
}

void JointMotorBodySimulationJudge::MR_LandingDetector::renderCue()
{
    if (success())
        glColor3f(0, 0.8, 0);
    else if (fulfilled())
        glColor3f(0.6, 0.0, 1.0);
    else
        glColor3f(1.0, 0, 0);
    
    glBegin(GL_LINES);
    glVertex2d(m_xmin, 0);
    glVertex2d(m_xmin, 1000);
    glVertex2d(m_xmax, 0);
    glVertex2d(m_xmax, 1000);
    glEnd();
}

Matrix2s JointMotorBodySimulationJudge::MR_LandingDetector::cueBoundingBox()
{
    Matrix2s bb;
    bb.col(0) = Vector2s(m_xmin, m_robot->rigidBody(m_robot->head())->getX().y());
    bb.col(1) = Vector2s(m_xmax, m_robot->rigidBody(m_robot->head())->getX().y());
    return bb;
}

std::string JointMotorBodySimulationJudge::MR_LandingDetector::detailedDescription() const
{
    std::stringstream ss;
    ss << "Landing Detector: the horizontal coordinate of the robot head must stay inside the range between " << m_xmin << " and " << m_xmax << " for " << m_duration << " seconds.";
    return ss.str();
}

std::string JointMotorBodySimulationJudge::MR_LandingDetector::briefDescription() const
{
    std::stringstream ss;
    ss << "Landing Detector: xmin = " << m_xmin << " xmax = " << m_xmax << " duration = " << m_duration;
    return ss.str();
}

void JointMotorBodySimulationJudge::MR_BalancingDetector::update(JointMotorBodyScene * scene, scalar dt)
{
    if (m_duration < 0)
    {
        // passive detector: once failed, cannot turn back to success any more
        if (!success())
            return;
    } else
    {
        if (success())
            return;
    }
    
    RigidBody * head = m_robot->rigidBody(m_robot->head());
    RigidBody * foot = m_robot->rigidBody(m_robot->foot());
    bool fulfilled = true;
    for (int i = 0; i < head->getVertices().size() / 2; i++)
    {
        if (head->getVertices()[i * 2 + 1] + head->getX().y() - head->getRadius() - foot->getRadius() * 2 < 0)
        {
            fulfilled = false;
            setViolated();
            m_progress = 0;
            break;
        }
    }
    
    if (m_duration < 0)
    {
        // passive detector
        if (fulfilled)
        {
            setFulfilled();
        } else
        {
            setFailure();
        }
    } else
    {
        if (fulfilled)
        {
            setFulfilled();
            m_progress += dt / m_duration;
            if (m_progress >= 1.0)
                setSuccess();
        }
    }
    
}

void JointMotorBodySimulationJudge::MR_BalancingDetector::renderProgress()
{
    
}

void JointMotorBodySimulationJudge::MR_BalancingDetector::renderCue()
{
    if (success())
        glColor3f(0, 0.8, 0);
    else if (fulfilled())
        glColor3f(0.6, 0.0, 1.0);
    else
        glColor3f(1.0, 0, 0);
    
    Vector2s x = m_robot->rigidBody(m_robot->head())->getX();
    glBegin(GL_LINES);
    glVertex2d(x.x(), 0);
    glVertex2d(x.x(), x.y());
    glEnd();
}

Matrix2s JointMotorBodySimulationJudge::MR_BalancingDetector::cueBoundingBox()
{
    Matrix2s bb;
    bb.col(0) = m_robot->rigidBody(m_robot->head())->getX();
    bb.col(1) = m_robot->rigidBody(m_robot->head())->getX();
    return bb;
}

std::string JointMotorBodySimulationJudge::MR_BalancingDetector::detailedDescription() const
{
    std::stringstream ss;
    if (m_duration < 0)
        ss << "Balancing Detector (Passive): keep the robot in balance throughout the simulation (no duration requirement). This detector is passive, as it starts off green (success) and only turns red (failure) when violated.";
    else
        ss << "Balancing Detector: keep the robot head from touching the ground for at least " << m_duration << " seconds.";
    return ss.str();
}

std::string JointMotorBodySimulationJudge::MR_BalancingDetector::briefDescription() const
{
    std::stringstream ss;
    if (m_duration < 0)
        ss << "Balancing Detector: passive";
    else
        ss << "Balancing Detector: duration = " << m_duration;
    return ss.str();
}

void JointMotorBodySimulationJudge::MR_VelocityDetector::update(JointMotorBodyScene * scene, scalar dt)
{
    if (success())
        return;
    
    RigidBody * head = m_robot->rigidBody(m_robot->head());
    bool fulfilled = (head->getV().x() >= m_vmin && head->getV().x() <= m_vmax);
    
    if (fulfilled)
    {
        setFulfilled();
        m_progress += dt / m_duration;
        if (m_progress >= 1.0)
            setSuccess();
    } else
    {
        setViolated();
        m_progress = 0;
    }
    
}

void JointMotorBodySimulationJudge::MR_VelocityDetector::renderProgress()
{
    
}

void JointMotorBodySimulationJudge::MR_VelocityDetector::renderCue()
{
    if (success())
        glColor3f(0, 0.8, 0);
    else if (fulfilled())
        glColor3f(0.6, 0.0, 1.0);
    else
        glColor3f(1.0, 0, 0);
    
    Vector2s x = m_robot->rigidBody(m_robot->head())->getX();
    Vector2s v = m_robot->rigidBody(m_robot->head())->getV();
    glBegin(GL_LINES);
    glVertex2d(x.x() + (m_vmin - v.x()), x.y());
    glVertex2d(x.x() + (m_vmax - v.x()), x.y());
    glVertex2d(x.x() + (m_vmin - v.x()), 0);
    glVertex2d(x.x() + (m_vmin - v.x()), 1000);
    glVertex2d(x.x() + (m_vmax - v.x()), 0);
    glVertex2d(x.x() + (m_vmax - v.x()), 1000);
    glEnd();
}

Matrix2s JointMotorBodySimulationJudge::MR_VelocityDetector::cueBoundingBox()
{
    RigidBody * head = m_robot->rigidBody(m_robot->head());
    Matrix2s bb;
    bb.col(0) = Vector2s(head->getX().x() + (m_vmin - head->getV().x()), head->getX().y());
    bb.col(1) = Vector2s(head->getX().x() + (m_vmax - head->getV().x()), head->getX().y());
    return bb;
}

std::string JointMotorBodySimulationJudge::MR_VelocityDetector::detailedDescription() const
{
    std::stringstream ss;
    ss << "Velocity Detector: the horizontal velocity of the robot head must get in range between " << m_vmin << " and " << m_vmax << " for " << m_duration << " seconds.";
    return ss.str();
}

std::string JointMotorBodySimulationJudge::MR_VelocityDetector::briefDescription() const
{
    std::stringstream ss;
    ss << "Velocity Detector: vmin = " << m_vmin << " vmax = " << m_vmax << " duration = " << m_duration;
    return ss.str();
}

void JointMotorBodySimulationJudge::MR_TreatDetector::update(JointMotorBodyScene * scene, scalar dt)
{
    if (success())
        return;
    
    RigidBody * head = m_robot->rigidBody(m_robot->head());
    Vector2s headx = head->getX();
    RigidBody * treat = scene->getRigidBody(m_treatrb);
    Vector2s treatx = treat->getX();
    scalar r = treat->getRadius();
    bool fulfilled = (headx - treatx).norm() <= r;
    
    if (fulfilled)
    {
        setFulfilled();
        m_progress = 1.0;
        setSuccess();
    } else
    {
        setViolated();
        m_progress = 0;
    }
    
}

void JointMotorBodySimulationJudge::MR_TreatDetector::renderProgress()
{
    
}

void JointMotorBodySimulationJudge::MR_TreatDetector::renderCue()
{
    if (success())
        glColor3f(0, 0.8, 0);
    else if (fulfilled())
        glColor3f(0.6, 0.0, 1.0);
    else
        glColor3f(1.0, 0, 0);
    
    RigidBody * head = m_robot->rigidBody(m_robot->head());
    Vector2s headx = head->getX();
    RigidBody * treat = m_robot->m_scene->getRigidBody(m_treatrb);
    Vector2s treatx = treat->getX();
    scalar r = treat->getRadius();
    
    glBegin(GL_LINES);
    int NC = 12;
    for (int i = 0; i < NC; i++)
    {
        scalar t0 = i * 2 * PI / NC;
        scalar t1 = (i + 1) * 2 * PI / NC;
        glVertex2f(treatx.x() + r * cos(t0), treatx.y() + r * sin(t0));
        glVertex2f(treatx.x() + r * cos(t1), treatx.y() + r * sin(t1));
    }
    glVertex2f(treatx.x(), treatx.y());
    glVertex2f(headx.x(), headx.y());
    glEnd();
}

Matrix2s JointMotorBodySimulationJudge::MR_TreatDetector::cueBoundingBox()
{
    RigidBody * treat = m_robot->m_scene->getRigidBody(m_treatrb);
    Vector2s x = treat->getX();
    scalar r = treat->getRadius();
    
    Matrix2s bb;
    bb.col(0) = Vector2s(x.x() - r, x.y() - r);
    bb.col(0) = Vector2s(x.x() + r, x.y() + r);
    return bb;
}

std::string JointMotorBodySimulationJudge::MR_TreatDetector::detailedDescription() const
{
    std::stringstream ss;
    ss << "Treat Detector: the robot head's center of mass must reach the candy (within its circumference). The detector is satisfied instantly.";
    return ss.str();
}

std::string JointMotorBodySimulationJudge::MR_TreatDetector::briefDescription() const
{
    std::stringstream ss;
    ss << "Treat Detector: find the treat";
    return ss.str();
}

