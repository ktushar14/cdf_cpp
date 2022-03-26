#include "robot_space.hpp"
#include "xyenv.hpp"
#include "xytheta.hpp"
#include <smpl/heap/intrusive_heap.h>
#include <vector>

#pragma once

namespace cs {

struct WAState : public smpl::heap_element {
    int state_id; // corresponding graph state
    double g; // cost-to-come
    double h; // cost-to-go
    double f; // (g + eps*h) at time of insertion into OPEN
    WAState* bp;
    bool closed;
    int primID;
};

struct WAStateCompare {
    bool operator()(const WAState& s1, const WAState& s2) const
    {
        return s1.f < s2.f;
    }
};

class WAStar {
public:
    WAStar(RobotSpace* environment, double epsilon);
    ~WAStar();

    void SetGoal(int state_id);
    void SetStart(int state_id);

    auto GetSolution() const -> std::vector<int>;
    auto GetSolutionPrimIDs() const -> std::vector<int>;
    auto GetExpandedStates() const -> std::vector<int>;
    int Run();

    int startStateID_;
    int goalStateID_;

    double eps_;

    RobotSpace* env_;
    std::vector<WAState*> states_;
    smpl::intrusive_heap<WAState, WAStateCompare> open_;

    std::vector<int> solution_;
    std::vector<int> solutionPrimIDs_;
    std::vector<int> expanded_;
    int numExpands_;

    void reset();
    void expand(WAState* s);
    double computeKey(WAState* s);
    double computeHeuristic(WAState* s);
    auto getState(int state_id) -> WAState*;
    auto createState(int state_id) -> WAState*;
    void initState(WAState* state);
    void extractPath(WAState* goal, int& solution_cost);
    bool isGoal(int state_id);
};

} // namespace cs
