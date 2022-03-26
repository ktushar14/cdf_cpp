#include "covsearch/constants.hpp"

namespace cs {

const std::map<char, int> MOVINGAI_DICT =
{
	{'C', 1}, /* COVERED     */ /// movingai: traversable
	{'N', 2}, /* NOT COVERED */ /// movingai: water is only traversible from water
	{'O', 0}  /* OBSTACLE    */ /// movingai: not traversable (obstacle)
	// {'.', 1}, // traversable
	// {'@', -1}, // out of bounds
	// {'O', -1}, // out of bounds
	// {'S', 1},
	// {'(', 4}, // start
	// {'*', 6}, // path
	// {')', 8}, // goal
	// {'E', 10}, // expanded state
};

const int kApproach = FSMA;
const int kEnvironment = XYTHETA;
const bool kEightConnected = false;
// The larger this number (> 1.0) is, the less the planner will care about
// traveling over already covered cells.
// 1.0 means equal weight to both.
// const double kSenseTravelRatio = 1.35;
// const double kSenseTravelRatio = 3.0;
// const int kCostConstant = (int)((1-kSenseTravelRatio)*1e5 + 1); // 1e4 = total coverage cells in map
const double kCostConstant = 1e6;
const double kTurnInPlaceCost = 8.0;
const float kInvalidAngularVelValue = 123456.0;

extern const double kPatternLinearVel = 2.1;
extern const double kPatternAngularVel = 0.04;

} // namespace cs

namespace vis {

const int kOneCellPx = 12;
const int kXOrigin = 0;
const int kYOrigin = 0;
const int kSceneXLength = 50 * kOneCellPx;
const int kSceneYLength = 100 * kOneCellPx;
const int kTimerTick_ms = 10;

const int kMapXLength = 30;
const int kMapYLength = 50;

} // namespace vis
