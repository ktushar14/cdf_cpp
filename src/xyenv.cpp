#include <cassert>
#include <iostream>

#include "covsearch/angles.hpp"
#include "covsearch/xyenv.hpp"

#define TERM_DEBUG false
#define FOUR_GRID false

namespace cs {

XYEnv::XYEnv()
{
    bFrontier_ = false;

    assert(states_.empty());

    /// Create imaginary goal
    goalStateID_ = reserveHashEntry();
    assert(goalStateID_ == 0);

    /// TODO: Make sure to handle what getHashEntry(0) returns
}

XYEnv::~XYEnv()
{
}

void XYEnv::SetStart(RobotState& start_state)
{
    RobotCoord start_coord;
    stateToCoord(start_state, start_coord);

    startStateID_ = getOrCreateState(start_coord);
    assert(startStateID_ >= 0);
}

void XYEnv::SetGoal(RobotState& goal_state)
{
    RobotCoord goal_coord;
    stateToCoord(goal_state, goal_coord);

    goalCoord_ = goal_coord;
    goalStateID_ = getOrCreateState(goal_coord);
    assert(goalStateID_ >= 0);
}

bool XYEnv::IsGoal(const int& stateID) const
{
    auto* state_entry = getHashEntry(stateID);
    if (bFrontier_)
    {
        int r = state_entry->coord[1];
        int c = state_entry->coord[0];
        return isFrontierCell(r, c, movingai_->m_map, rows_, cols_);
    }
    else
    {
        return stateID == goalStateID_;
    }
}

auto XYEnv::ConvertStatesToWaypoints(
    std::vector<int>& state_ids,
    std::vector<int>& primIDs,
    double& lengthOfPathSoFar) -> std::vector<RobotState>
{
    /* param lengthOfPathSoFar currently unused here */
    std::vector<RobotState> sol_states;
    for (const auto& id : state_ids)
    {
        auto* entry = getHashEntry(id);
        auto sol_coord = entry->coord;
        RobotState sol_state;
        coordToState(sol_coord, sol_state);
        sol_states.push_back(sol_state);
    }
    return sol_states;
}

void XYEnv::GetSuccs(
    int& parent_id,
    std::vector<int>* succs,
    std::vector<double>* costs,
    std::vector<int>* primIDs)
{
    StateEntry* parent_entry = getHashEntry(parent_id);

    if (parent_id == goalStateID_)
    {
        /// GetSuccs should not be called on imaginary goal
        assert(parent_id == 0);
        assert(false);
    }
    else if (IsGoal(parent_id))
    {
        /// Successors of potential goal include only the imaginary goal
        succs->push_back(0);
        costs->push_back(1);
        primIDs->push_back(-1);
    }
    else
    {

#if FOUR_GRID
        int n = 4;
        int motions_x[n] = { -1, 1, 0, 0 };
        int motions_y[n] = { 0, 0, -1, 1 };
#else
        int n = 8;
        int motions_x[n] = { -1, 1, 0, 0, 1, -1, 1, -1 };
        int motions_y[n] = { 0, 0, -1, 1, -1, 1, 1, -1 };
#endif

        for (auto i = 0; i < n; ++i) {
            auto mx = motions_x[i];
            auto my = motions_y[i];

            RobotCoord succ_coord;
            succ_coord.push_back(parent_entry->coord[0] + mx);
            succ_coord.push_back(parent_entry->coord[1] + my);

            // reject successor if out of bounds
            if (!inBounds(succ_coord)) continue;

            // reject successor if in collision
            // TODO: cont to disc conversion (not simply (int) casting)
            auto idx = GETMAPINDEX((int)succ_coord[1], (int)succ_coord[0], rows_, cols_);
            if (movingai_->m_map[idx] == 0) continue;

    #if TERM_DEBUG
            std::cout << "E: succ_coord: (" << succ_coord[0] << ", " << succ_coord[1] << ")" << std::endl;
    #endif
            // printf("  [%f, %f]\n", succ_coord[0], succ_coord[1]);

            succs->push_back(getOrCreateState(succ_coord));
            if (FOUR_GRID)
            {
                costs->push_back(1);
            }
            else
            {
                int cost = (i > 3) ? 2 : 1;
                costs->push_back(cost);
            }
            primIDs->push_back(-1); // unused in x,y search
        }
    }
}

double XYEnv::GetGoalHeuristic(int state_id)
{
    if (state_id == 0) return 0;

    if (bFrontier_)
    {
        return 0;
    }
    else
    {
        auto* s = getHashEntry(state_id);
        return sqrt(
            (s->coord[0] - goalCoord_[0]) * (s->coord[0] - goalCoord_[0]) +
            (s->coord[1] - goalCoord_[1]) * (s->coord[1] - goalCoord_[1])
        );
    }
}

void XYEnv::stateToCoord(RobotState& inState, RobotCoord& outCoord)
{
    int x_disc = CONTXY2DISC(inState[0], 1.0);
    int y_disc = CONTXY2DISC(inState[1], 1.0);
    outCoord = { x_disc, y_disc };
}

void XYEnv::coordToState(RobotCoord& inCoord, RobotState& outState)
{
    double x_cont = (double)inCoord[0];
    double y_cont = (double)inCoord[1];
    outState = { x_cont, y_cont };
}

bool XYEnv::inBounds(RobotState& state)
{
    return (state[0] >= 0 && state[0] < cols_ && state[1] >= 0 && state[1] < rows_);
}

bool XYEnv::inBounds(RobotCoord& coord)
{
    return (coord[0] >= 0 && coord[0] < cols_ && coord[1] >= 0 && coord[1] < rows_);
}

int XYEnv::getOrCreateState(RobotCoord& coord)
{
    int state_id = getHashEntry(coord);
    if (state_id < 0) {
        state_id = createHashEntry(coord);
    }
    return state_id;
}

int XYEnv::getHashEntry(RobotCoord& coord)
{
    StateEntry state_entry;
    state_entry.coord = coord;
    auto sit = stateToID_.find(&state_entry);
    if (sit == stateToID_.end()) {
        return -1;
    }
    return sit->second;
}

auto XYEnv::getHashEntry(int state_id) const -> StateEntry*
{
    if (state_id < 0 || state_id >= (int)states_.size()) {
        return nullptr;
    }

    return states_[state_id];
}

int XYEnv::createHashEntry(RobotCoord& coord)
{
    int state_id = reserveHashEntry();
    StateEntry* state_entry = getHashEntry(state_id);

    state_entry->coord = coord;

    // map state -> state id
    stateToID_[state_entry] = state_id;

    return state_id;
}

int XYEnv::reserveHashEntry()
{
    StateEntry* entry = new StateEntry;
    int state_id = (int)states_.size();

    // map state id -> state
    states_.push_back(entry);

    return state_id;
}

} // namespace cs
