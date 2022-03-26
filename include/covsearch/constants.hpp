#include <map>

#pragma once

namespace cs {

extern const std::map<char, int> MOVINGAI_DICT;

enum PlanResultCode
{
    SUCCESS = 0,
    OPEN_EMPTY
};

enum Approach
{
    STANDARD = 0,
    FSMA, // Frontier Search + Macro Actions
    RANDOM // Frontier Search + Macro Actions
};

enum Environment
{
    XY = 0,
    XYTHETA
};

extern const int kApproach; // currently unused
extern const int kEnvironment;
extern const bool kEightConnected;
/// If when I'm travelling over 100 cells, I can afford to keep the sensor on
/// for only 50 of them, kSenseTravelRatio = (50/100).
// extern const double kSenseTravelRatio;
extern const double kCostConstant;
extern const double kTurnInPlaceCost;

extern const double kPatternLinearVel;
extern const double kPatternAngularVel;

// If a motion primitive has this value for angular velocity (action.rv), then
// it is a straight line primitive, lol (I have a deadline).
extern const float kInvalidAngularVelValue;

} // namespace cs

namespace vis {

extern const int kOneCellPx;
extern const int kXOrigin;
extern const int kYOrigin;
extern const int kSceneXLength;
extern const int kSceneYLength;
extern const int kTimerTick_ms;

extern const int kMapXLength;
extern const int kMapYLength;

} // namespace vis
