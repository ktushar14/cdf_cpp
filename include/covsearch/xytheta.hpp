#include <fstream>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include "robot_space.hpp"
#include "types.hpp"

#pragma once

namespace cs {

class XYThetaEnv : public RobotSpace {
public:
    XYThetaEnv();
    ~XYThetaEnv() override;

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

    void ReadMprims(std::ifstream& mprimFileStream) override;
    void ReadMacroActions(std::ifstream& macroactionFileStream) override;
    auto ConvertStatesToWaypoints(
        std::vector<int>& state_ids,
        std::vector<int>& primIDs,
        double& lengthOfPathSoFar) -> std::vector<RobotState> override;
    CoveredCellsLookup GetCoveredCells() override;
    RobotState GetSelectedFrontierCell() override;

private:
    void stateToCoord(RobotState& inState, RobotCoord& outCoord);
    void coordToState(RobotCoord& inCoord, RobotState& outState);

    int getOrCreateState(RobotCoord& coord);
    int getHashEntry(RobotCoord& coord);
    auto getHashEntry(int state_id) const -> StateEntry*;
    int createHashEntry(RobotCoord& state);
    int reserveHashEntry();
    void reset();

    void storeCurrentPlanPatternMetaData(Action& action);

    bool validAction(StateEntry* parent, Action* a);
    bool validMacroAction(StateEntry* parent, Action* a);
    bool inBounds(RobotState& state);
    bool inBounds(int& x, int& y);

    void createAndStoreAction(Action& inAction);
    void createAndStoreMacroAction(Action& inAction);
    int createGoalState();

    void getSuccsFrontierState(
        int& parent_id,
        std::vector<int>* succs,
        std::vector<double>* costs,
        std::vector<int>* primIDs);
    double moveActionCost(StateEntry* state, Action* action);
    double senseActionCost(StateEntry* state, Action* action);

private:

    // states
    std::vector<StateEntry*> states_;

    // actions and collision-checking
    std::unordered_map<int, std::unique_ptr<Action>> actions_;
    std::unordered_map<int, std::unique_ptr<Action>> macroactions_;

    RobotCoord goalCoord_;

    // maps from coords to stateID
    typedef StateEntry StateKey;
    typedef smpl::PointerValueHash<StateKey> StateHash;
    typedef smpl::PointerValueEqual<StateKey> StateEqual;
    smpl::hash_map<StateKey*, int, StateHash, StateEqual> stateToID_;
};

} // namespace cs