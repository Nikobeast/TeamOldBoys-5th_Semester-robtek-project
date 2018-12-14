#include "fuzzyobstacleavoidance.h"

    //-------------------------------------------------------------------------------------------------------//

FuzzyObstacleAvoidance::FuzzyObstacleAvoidance(LaserScanner *pc_laser_scanner) : m_pcLaserScanner(pc_laser_scanner)
{

}

    //-------------------------------------------------------------------------------------------------------//

ControlOutput FuzzyObstacleAvoidance::getControlOutput()
{
    m_pflObsDistance->setValue(m_pcLaserScanner->getClosestDistance(-1.65, 1.65));
    m_pflObsDirection->setValue(m_pcLaserScanner->getClosestDirection(-1.65, 1.65));
    m_pflGoalAngle->setValue(Goals[0]);
    m_pflGoalDist->setValue(Goals[1]);

    m_pcFLEngine->process();

    ControlOutput out;
    out.direction = m_pflDirCorrection->getValue();
    out.speed     = m_pflSpeedFactor->getValue();

    return out;
}

    //-------------------------------------------------------------------------------------------------------//

void FuzzyObstacleAvoidance::buildController(cv::Vec2d Goal)
{
    Goals = Goal;
    using namespace fl;
    m_pcFLEngine = FllImporter().fromFile("../../github/clone_repository/fuzzyobstacleavoidance.fll");

    std::string status;
    if (not m_pcFLEngine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);


    m_pflObsDirection = m_pcFLEngine->getInputVariable("ObsDirection");
    m_pflObsDistance  = m_pcFLEngine->getInputVariable("ObsDistance");

    m_pflGoalAngle = m_pcFLEngine->getInputVariable("GoalAngle");
    m_pflGoalDist  = m_pcFLEngine->getInputVariable("GoalDist");

    m_pflDirCorrection= m_pcFLEngine->getOutputVariable("DirCorrection");
    m_pflSpeedFactor    = m_pcFLEngine->getOutputVariable("SpeedFactor");

}

    //-------------------------------------------------------------------------------------------------------//
