#ifndef FUZZYOBSTACLEAVOIDANCE_H
#define FUZZYOBSTACLEAVOIDANCE_H

    //-------------------------------------------------------------------------------------------------------//

#include "laserscanner.h"
#include <opencv2/opencv.hpp>
#include <fl/Headers.h>

struct ControlOutput
{
    float direction;
    float speed;
};

    //-------------------------------------------------------------------------------------------------------//

class FuzzyObstacleAvoidance
{
public:
    FuzzyObstacleAvoidance(LaserScanner* pc_laser_scanner);
    virtual ~FuzzyObstacleAvoidance() = default;

    virtual void buildController(cv::Vec2d);
    virtual ControlOutput getControlOutput();

protected:
    LaserScanner*        m_pcLaserScanner;

    fl::Engine*          m_pcFLEngine;
    fl::InputVariable*   m_pflObsDirection;
    fl::InputVariable*   m_pflObsDistance;
    fl::InputVariable*   m_pflGoalAngle;
    fl::InputVariable*   m_pflGoalDist;
    fl::OutputVariable*  m_pflDirCorrection;
    fl::OutputVariable*  m_pflSpeedFactor;
    cv::Vec2d Goals;

};

    //-------------------------------------------------------------------------------------------------------//

#endif // FUZZYOBSTACLEAVOIDANCE_H
