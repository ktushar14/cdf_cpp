#include <chrono>
#include <functional>
#include <iostream>
#include <vector>
#include <unordered_set>

#pragma once

namespace cs {

using BooleanMap = std::vector<std::vector<int>>;
typedef int* MAP_t;

using RobotState = std::vector<double>; // continuous state
using RobotCoord = std::vector<int>; // discrete state
using Plan = std::vector<RobotState>; // plan of continuous states

struct XYState
{
    double x; // continuous x coordinate
    double y; // continuous y coordinate
};

struct XYCell
{
    int x;
    int y;
};

struct XYCellEqual {
    bool operator()(const XYCell& a, const XYCell& b) const { return a.x == b.x && a.y == b.y; }
};

template<
    class Key,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = std::allocator<Key>>
using XYCellLookup = std::unordered_set<Key, Hash, KeyEqual, Allocator>;

typedef std::hash<XYCell> XYCellHash;
typedef XYCellLookup<XYCell, XYCellHash, XYCellEqual> CoveredCellsLookup;

struct XYThetaState
{
    double x;     // continuous x coordinate
    double y;     // continuous y coordinate
    double theta; // heading in radians
    double l;     // length of the primitive up to this intermediate point
};

struct PatternMetaData
{
    int n_forward_straight;
    int n_backward_straight;
    int n_forward_backward_curve;
    int n_backward_forward_curve;
    double straight_length;
    double arc_dtheta;
};

/// Actions that contain X,Y,Theta states. Actions can either be regular
/// motion primitives (resources/mprim/unicycle_res_1m.mprim) or macro-actions
/// (resources/mprim/macro.mprim).
struct Action
{
    int primID;
    XYThetaState startState;
    XYThetaState endState;

    /// Intermediate states, off-lattice in general. These must be read from file
    /// such that the intermediateStates.front() is NOT the same as startState.
    std::vector<XYThetaState> intermediateStates;

    /// Intermediate discrete cells the action goes over. Populated when reading
    /// in the continuous action from file (but could also theoretically be
    /// generated offline with the macro action).
    std::vector<XYCell> intermediateCells;

    float duration; // mprim duration
    float tv;       // translational/linear velocity
    float rv;       // rotational/angular velocity

    /// Non-negative only for coverage patterns.
    int xdim = -1;
    int ydim = -1;
    PatternMetaData meta_data;

    void reset()
    {
        startState = { 0.0, 0.0, 0.0, 0.0 };
        endState = { 0.0, 0.0, 0.0, 0.0 };
        intermediateStates = {};
        intermediateCells = {};
        xdim = -1;
        ydim = -1;
        primID = -1;
        duration = 0.0;
        tv = 0.0;
        rv = 0.0;
        meta_data = { -1, -1, -1, -1, -1, -1 };
    }

    void display()
    {
        std::cout << "Action:" << std::endl;
        std::cout << "primID:        " << primID << std::endl;
        std::cout << "Start:         " << startState.x << ", " << startState.y
                  << ", " << startState.theta << ", " << std::endl;
        std::cout << "End:           " << endState.x << ", " << endState.y << ", "
                  << endState.theta << ", " << std::endl;
        std::cout << "# int. states: " << intermediateStates.size() << std::endl;
    }
};

// Timing
using hr_clk = std::chrono::high_resolution_clock;

// Callbacks
using StateUpdateCB = std::function<void(RobotState&)>;
using MapUpdateCB = std::function<void(MAP_t)>; // MAP_t = int*

// Log levels
enum class LogLevel {
    L_TRACE = 0,
    L_DEBUG,
    L_INFO,
    L_WARN,
    L_ERROR,
    L_FATAL,

    LEVELS_COUNT
};

} // namespace cs

namespace cs {

#ifndef STATE_ENTRY
#define STATE_ENTRY

struct StateEntry
{
    RobotCoord coord; // discrete robot coordinates
};

inline bool operator==(const StateEntry& a, const StateEntry& b)
{
    return a.coord == b.coord;
}

#endif // STATE_ENTRY

} // namespace cs

namespace std {

template <>
struct hash<cs::StateEntry> {
    typedef cs::StateEntry argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

template <>
struct hash<cs::XYCell> {
    typedef cs::XYCell argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std