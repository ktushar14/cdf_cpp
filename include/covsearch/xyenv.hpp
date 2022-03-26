#include <fstream>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include "robot_space.hpp"

#pragma once

namespace cs {

class XYEnv : public RobotSpace {
public:
    XYEnv();
    ~XYEnv() override;

    /// Required functions from RobotSpace /////////////////////////////////////
    void SetGoal(RobotState& goal) override;
    void SetStart(RobotState& start) override;
    void GetSuccs(
        int& parent_id,
        std::vector<int>* succs,
        std::vector<double>* costs,
        std::vector<int>* primIDs) override;
    double GetGoalHeuristic(int state_id) override;
    bool IsGoal(const int& stateID) const override;
    ////////////////////////////////////////////////////////////////////////////

    /// Given state IDs and mprim IDs, returns a plan of x,y states
    auto ConvertStatesToWaypoints(
        std::vector<int>& state_ids,
        std::vector<int>& primIDs,
        double& lengthOfPathSoFar) -> std::vector<RobotState> override;

private:
    void stateToCoord(RobotState& inState, RobotCoord& outCoord);
    void coordToState(RobotCoord& inCoord, RobotState& outState);

    bool inBounds(RobotState& state);
    bool inBounds(RobotCoord& coord);
    int getOrCreateState(RobotCoord& coord);
    int getHashEntry(RobotCoord& coord);
    auto getHashEntry(int state_id) const -> StateEntry*;
    int createHashEntry(RobotCoord& coord);
    int reserveHashEntry();

private:
    // states
    std::vector<StateEntry*> states_;

    RobotCoord goalCoord_;

    // actions and collision-checking
    std::unordered_map<int, std::unique_ptr<Action>> actions_;

    // maps from coords to stateID
    typedef StateEntry StateKey;
    typedef smpl::PointerValueHash<StateKey> StateHash;
    typedef smpl::PointerValueEqual<StateKey> StateEqual;
    smpl::hash_map<StateKey*, int, StateHash, StateEqual> stateToID_;
};

} // namespace cs
