// #include <boost/functional/hash.hpp>
#include <smpl/types.h>

#include "covsearch/movingai.hpp"
#include "covsearch/types.hpp"
#include "covsearch/helpers.hpp"

#include <unordered_set>

#pragma once

// namespace cs {

// #ifndef STATE_ENTRY
// #define STATE_ENTRY

// struct StateEntry {
//     RobotState state; // discrete robot coordinates
// };

// inline bool operator==(const StateEntry& a, const StateEntry& b)
// {
//     return a.state == b.state;
// }

// #endif // STATE_ENTRY

// } // namespace cs

// namespace std {

// template <>
// struct hash<cs::StateEntry> {
//     typedef cs::StateEntry argument_type;
//     typedef std::size_t result_type;
//     result_type operator()(const argument_type& s) const;
// };

// template <>
// struct hash<cs::XYCell> {
//     typedef cs::XYCell argument_type;
//     typedef std::size_t result_type;
//     result_type operator()(const argument_type& s) const;
// };

// } // namespace std

namespace cs {

class RobotSpace {
public:
    RobotSpace();
    virtual ~RobotSpace(){};

    /////////////
    // METHODS //
    /////////////

    /// Required ///////////////////////////////////////////////////////////////
    virtual void SetGoal(RobotState& goal) = 0;
    virtual void SetStart(RobotState& start) = 0;
    virtual void GetSuccs(
        int& parent_id,
        std::vector<int>* succs,
        std::vector<double>* costs,
        std::vector<int>* primIDs) = 0;
    virtual double GetGoalHeuristic(int state_id) = 0;
    virtual bool IsGoal(const int& stateID) const = 0;
    ////////////////////////////////////////////////////////////////////////////

    virtual void Init(){};

    /// Sets the map in movingAI format
    virtual void SetMap(const std::string& mapfile) final;

    /// Does nothing for the base class. Read and set motion primitives
    /// according to the environment.
    virtual void ReadMprims(std::ifstream& mprimFileStream);

    /// Does nothing for the base class. Read and set macro actions.
    virtual void ReadMacroActions(std::ifstream& macroactionFileStream);

    int actionCoordsToIdx(const std::pair<int, int>& coords) const;
    auto actionIdxToCoords(const int& actionIdx) const -> std::pair<int, int>;

    int actionCoordsToIdx_Macro(const std::pair<int, int>& coords) const;
    auto actionIdxToCoords_Macro(const int& actionIdx) const -> std::pair<int, int>;

    virtual int GetStartStateID() const; /// simply returns startStateID_
    virtual int GetGoalStateID() const;  /// simply returns goalStateID_

    /// Applies actions correspondig to `primIDs` to states corresponding to
    /// `state_ids` and returns a full plan. If kApproach == FSMA, it also
    /// stores the cells covered by the last action in coveredCells_ and the
    /// chosen frontier cell in selectedFrontierCell_.
    virtual auto ConvertStatesToWaypoints(
        std::vector<int>& state_ids,
        std::vector<int>& primIDs,
        double& lengthOfPathSoFar) -> std::vector<RobotState>;

    /// Get cells covered in this planning iteration. These cells are stored
    /// in coveredCells_ at the end of each planning iteration (in the function
    /// ConvertStatesToWaypoints)
    virtual CoveredCellsLookup GetCoveredCells();

    /// Returns the frontier cell selected in the current planning iteration.
    virtual RobotState GetSelectedFrontierCell();

    /////////////
    // OBEJCTS //
    /////////////

    /// Start state ID
    int startStateID_;

    /// Goal state ID
    int goalStateID_;

    /// Map information
    int rows_;
    int cols_;
    std::unique_ptr<MovingAI> movingai_;

    /// Motion primitive information
    int numTotalPrims_;
    int numPrimsPerAngle_;
    int numTotalAngles_;

    /// Macro action information
    int numTotalMacroActions_;
    int numMacroActionsPerAngle_;
    int numTotalMacroAngles_;

    double senseTravelRatio_;

    /// Continous plan information
    double current_move_execution_time_ms_;
    PatternMetaData current_plan_pattern_md_;
    double lengthOfPathSoFar_;

    bool bFrontier_;
    RobotState selectedFrontierCell_;
    CoveredCellsLookup coveredCells_;
    mutable bool bFirstPlannerCall_;
};

} // namespace cs