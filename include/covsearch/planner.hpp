// C++ standard library headers
#include <condition_variable>
#include <fstream>
#include <memory>
#include <thread>
#include <unordered_set>

// Other library headers
#include <smpl/search/arastar.h>

// Project utils
#include "covsearch/types.hpp"

// Project modules
#include "covsearch/robot_space.hpp"
#include "covsearch/wastar.hpp"
#include "covsearch/xyenv.hpp"
#include "covsearch/xytheta.hpp"

#pragma once

namespace cs {

class PlanClass {
public:
    Plan wps_;
    std::mutex mtx_;
    RobotState lastWp_;

    RobotState getWaypoint()
    {
        std::lock_guard lk(mtx_);
        if (wps_.empty()) {
            RobotState s = {};
            return s;
        } else {
            RobotState s = std::move(wps_.back()); // AAH! ???
            wps_.pop_back();
            return s;
        }
    }

    void SetNewPlan(Plan plan)
    {
        std::lock_guard lk(mtx_);
        if (!wps_.empty())
            wps_.clear();
        std::reverse(plan.begin(), plan.end());
        wps_ = plan;
        lastWp_ = plan.front();
    }
};

struct PlannerResult
{
    cs::Plan plan;
    int err;
};

class Planner {
public:
    /// METHODS ////////////////////////////////////////////////////////////////
    Planner();
    ~Planner();

    /// SETUP
    void Init(double& sense_travel_ratio);
    void ReadMprims(
        std::ifstream& mprimFileStream,
        std::ifstream& macroactionFileStream);
    auto SetMap(const std::string& mapfile) -> std::pair<int,int>;

    void GetRandomGoal(RobotState& goal);
    void GetCurrentGoalState(RobotState& currentGoal);

    void CopyLatestMap(MAP_t map);
    void UpdateMapWithCoveredCells(CoveredCellsLookup& coveredCells);
    void UpdateMapWithCell(XYCell& xycell);
    void CalcNumFrontierCells();

    void PlannerThread();
    void logCurrentMap();
    void Start();

    bool TrySendRequest(cs::RobotState startState);
    bool TryGetResult(cs::Plan& result); // CURRENTLY UNUSED
    PlannerResult GetNewPlan(cs::RobotState startState);

    void SetGoal(RobotState& inState);
    void SetStart(RobotState& inState);
    int RunSearch();

    /// Get random goal, compute plan, repeat till planning successful
    int NavigateToRandomGoal();

    /// Main Approach.
    /// Get best plan that passes through a frontier cell and senses it or
    /// executes a macro action at it.
    int FrontierWithMacroActions();

    bool inBounds(RobotState& state);
    int RandomCoverage();

    std::vector<int> GetSolutionStateIDs();
    cs::Plan GetContinuousSolution();

    std::vector<int> GetExpandedStateIDs();
    std::vector<RobotState> GetExpandedCoords();

    /// OBJECTS ////////////////////////////////////////////////////////////////
    std::ofstream logs_;
    int iteration_;
    double current_planning_time_ms_;
    double current_move_execution_time_ms_;
    PatternMetaData current_plan_pattern_md_;

    int numCoveredCells_;
    int numCellsToCover_;
    int numFrontierCells_;

    std::unique_ptr<WAStar> search_;
    std::unique_ptr<RobotSpace> env_;

    cs::RobotState currentStart_;
    cs::RobotState currentGoal_;

    int errorCode_;
    bool bFirstPlannerCall_;

    PlanClass planObj_;
    Plan randomPlan_;
    double lengthOfPathSoFar_;

    std::thread planningThread_;

    std::mutex plannerMtx_;

    std::condition_variable cvRequestReceived_;
    bool bRequestReceived_;

    std::condition_variable cvResultReady_;
    bool bResultReady_;

    std::condition_variable cvReceiving_;
    bool bReceiving_;
};

} // namespace cs
