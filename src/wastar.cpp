#include "covsearch/wastar.hpp"
#include "covsearch/xyenv.hpp"
#include "covsearch/constants.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <cstdlib>

#define TERM_DEBUG false

namespace cs {

WAStar::WAStar(RobotSpace* environment, double epsilon)
    : env_(environment)
    , startStateID_(-1)
    , goalStateID_(-1)
    , eps_(epsilon)
{
}

WAStar::~WAStar() { reset(); }

void WAStar::SetGoal(int state_id)
{
    if (kEnvironment == XYTHETA)
    {
        assert(state_id == 0); // With imaginary goal, goal state ID = 0 always
    }
    else if (kEnvironment == XY)
    {
        assert(state_id >= 0);
    }
    goalStateID_ = state_id;
    // std::cout << "WAStar: Set goal with id " << goalStateID_ << std::endl;
}

void WAStar::SetStart(int state_id)
{
    assert(state_id >= 0);
    startStateID_ = state_id;
    // std::cout << "WAStar: Set start with id " << startStateID_ << std::endl;
}

auto WAStar::GetSolution() const -> std::vector<int>
{
    return solution_;
}

auto WAStar::GetSolutionPrimIDs() const -> std::vector<int>
{
    if (kApproach != RANDOM) {
        assert(!solutionPrimIDs_.empty());
    }
    return solutionPrimIDs_;
}

auto WAStar::GetExpandedStates() const -> std::vector<int>
{
    assert(!expanded_.empty());
    return expanded_;
}

int WAStar::Run()
{
    int err = 0; // TODO: add error codes for failures

    reset();
    expanded_.clear();
    numExpands_ = 0;

    int solution_cost;

    auto* start = getState(startStateID_);
    start->g = 0;
    start->f = computeKey(start);

    assert(open_.empty());
    open_.push(start);

    // while OPEN is not empty
    while (!open_.empty())
    {
        auto* best = open_.min();
        if (best->state_id == goalStateID_) {
            // with imaginary goal, extract path from parent of imaginary goal
            // (i.e. a potential goal)
            printf("  best state is imaginary goal, id = [%d], f = [%f]\n", best->state_id, best->f);
            // extractPath(best->bp, solution_cost);
            extractPath(best, solution_cost);
            break;
        }
        open_.pop();

        assert(!best->closed);
        expand(best);
    }

    // OPEN list exhausted
    if (open_.empty()) {
        std::cout << "WAStar: OPEN EMPTY" << std::endl;
        err = OPEN_EMPTY;
        return err;
    }

    /// PLANNING SUCCESSFUL
    std::cout << "WAStar: Cost: " << solution_cost << std::endl;
    return SUCCESS;
}

void WAStar::reset()
{
    open_.clear();
    for (auto* s : states_) {
        delete s;
        s = nullptr;
    }
    states_.clear();
}

void WAStar::expand(WAState* s)
{
#if TERM_DEBUG
    std::cout << "Expanding state " << s->state_id << " with g = " << s->g
              << std::endl;
#endif
    // printf("  expanding state [%d] with f value [%u]\n", s->state_id, s->f);

    std::vector<int> succs;
    std::vector<double> costs;
    std::vector<int> primIDs;

    env_->GetSuccs(s->state_id, &succs, &costs, &primIDs);
    assert(succs.size() == costs.size());

#if TERM_DEBUG
    std::cout << succs.size() << " successors" << std::endl;
#endif

    // printf("  got %d successors\n", (int)succs.size());

    for (auto sidx = 0; sidx < succs.size(); sidx++)
    {
        int succ_id = succs[sidx];
        double cost = costs[sidx];
        int primID = primIDs[sidx];

        auto* succ_state = getState(succ_id);

        if (!succ_state->closed) {
            double new_cost = s->g + cost;
            if (new_cost < succ_state->g) {
                succ_state->g = new_cost;
                succ_state->f = computeKey(succ_state);
                succ_state->bp = s;
                succ_state->primID = primID;
                if (open_.contains(succ_state)) {
                    open_.decrease(succ_state);
                } else {
                    open_.push(succ_state);
                }
            }
        }
    }
    s->closed = true;
    numExpands_++;
    expanded_.push_back(s->state_id);
}

double WAStar::computeKey(WAState* s) { return s->g + eps_ * s->h; }

double WAStar::computeHeuristic(WAState* s)
{
    return env_->GetGoalHeuristic(s->state_id);
}

auto WAStar::getState(int state_id) -> WAState*
{
    assert(state_id >= 0);

    // Size of states_ always equals one more than the largest state ID
    // This resizes states_ according to the new state ID required
    if (int(states_.size()) <= state_id) {
        states_.resize(state_id + 1, nullptr);
    }

    // Return the state at index = state_id
    auto& state = states_[state_id];

    // If element at index = state_id points to NULL,
    // create a new state with that ID
    if (state == nullptr) {
        state = createState(state_id); // ISSUE
    }

    return state;
}

auto WAStar::createState(int state_id) -> WAState*
{
    WAState* state = new WAState;
    state->state_id = state_id;
    initState(state);
    return state;
}

void WAStar::initState(WAState* state)
{
    state->g = std::numeric_limits<int>::max();
    state->h = computeHeuristic(state);
    state->f = std::numeric_limits<int>::max();
    state->closed = false;
    state->bp = nullptr;
}

void WAStar::extractPath(WAState* goal, int& solution_cost)
{
    solution_cost = goal->g;
    solution_.clear();
    for (auto* s = goal; s; s = s->bp) {
        solution_.push_back(s->state_id);
        solutionPrimIDs_.push_back(s->primID);
    }
    std::reverse(solution_.begin(), solution_.end());
    std::reverse(solutionPrimIDs_.begin(), solutionPrimIDs_.end());
}

bool WAStar::isGoal(int state_id) { return env_->IsGoal(state_id); }

} // namespace cs