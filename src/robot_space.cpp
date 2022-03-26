#include "covsearch/robot_space.hpp"

// auto std::hash<cs::StateEntry>::operator()(const argument_type& s) const
//     -> result_type
// {
//     size_t seed = 0;
//     boost::hash_combine(seed, boost::hash_range(s.state.begin(), s.state.end()));
//     return seed;
// }

// auto std::hash<cs::XYCell>::operator()(const argument_type& s) const
//     -> result_type
// {
//     size_t seed = 0;
//     boost::hash_combine(seed, s.x);
//     boost::hash_combine(seed, s.y);
//     return seed;
// }

namespace cs {

RobotSpace::RobotSpace()
:
numTotalPrims_(0),
numTotalAngles_(0),
numPrimsPerAngle_(0),
senseTravelRatio_(2.0),
bFirstPlannerCall_(false)
{
}

void RobotSpace::SetMap(const std::string& mapfile)
{
    movingai_ = std::make_unique<MovingAI>(mapfile);
    rows_ = movingai_->m_h;
    cols_ = movingai_->m_w;
    printf("rows, cols : %d, %d\n", rows_, cols_);
};

void RobotSpace::ReadMprims(std::ifstream& mprimFileStream)
{
    printf("Not implemented in base class\n");
}

void RobotSpace::ReadMacroActions(std::ifstream& macroactionFileStream)
{
    printf("Not implemented in base class\n");
}

int RobotSpace::actionCoordsToIdx(const std::pair<int, int>& coords) const
{
    auto primID = coords.first;
    auto discAngle = coords.second;
    return primID + numPrimsPerAngle_ * discAngle; // row-major linear indexing
}

std::pair<int, int> RobotSpace::actionIdxToCoords(const int& actionIdx) const
{
    int discAngle = actionIdx / numPrimsPerAngle_;
    int primID = actionIdx % numPrimsPerAngle_;
    return std::pair<int, int>(primID, discAngle);
}

int RobotSpace::actionCoordsToIdx_Macro(const std::pair<int, int>& coords) const
{
    auto primID = coords.first;
    auto discAngle = coords.second;
    return primID + numMacroActionsPerAngle_ * discAngle;
}

std::pair<int, int> RobotSpace::actionIdxToCoords_Macro(const int& actionIdx) const
{
    int discAngle = actionIdx / numMacroActionsPerAngle_;
    int primID = actionIdx % numMacroActionsPerAngle_;
    return std::pair<int, int>(primID, discAngle);
}

int RobotSpace::GetStartStateID() const { return startStateID_; }

int RobotSpace::GetGoalStateID() const { return goalStateID_; }

auto RobotSpace::ConvertStatesToWaypoints(
    std::vector<int>& state_ids,
    std::vector<int>& primIDs,
    double& lengthOfPathSoFar) -> std::vector<RobotState>
{
    printf("Not implemented in base class\n");
    std::vector<RobotState> dummy = {};
    return dummy;
}

CoveredCellsLookup RobotSpace::GetCoveredCells()
{
    printf("Not implemented in base class\n");
    CoveredCellsLookup dummy = {};
    return dummy;
}

RobotState RobotSpace::GetSelectedFrontierCell()
{
    printf("Not implemented in base class\n");
    RobotState dummy = {};
    return dummy;
}

} // namespace cs
