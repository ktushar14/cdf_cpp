#include "covsearch/xytheta.hpp"
#include "covsearch/angles.hpp"
#include "covsearch/constants.hpp"

#include <cmath>

namespace cs {

XYThetaEnv::XYThetaEnv()
{
    bFrontier_ = false;

    assert(states_.empty());
    actions_.clear();
    macroactions_.clear();

    /// Create imaginary goal
    /// TODO: Make sure to handle what getHashEntry(0) returns
    goalStateID_ = reserveHashEntry();
    assert(goalStateID_ == 0);
    assert(states_.size() == 1);

    current_move_execution_time_ms_ = 0.0;
}

XYThetaEnv::~XYThetaEnv() {}

void XYThetaEnv::createAndStoreAction(Action& inAction)
{
    auto action = std::make_unique<Action>();
    *action = inAction;
    int startAngle = angles::ContToDiscTheta(inAction.startState.theta);
    int primID = inAction.primID;
    std::pair<int, int> actionCoords(primID, startAngle);
    int actionIdx = actionCoordsToIdx(actionCoords);
    actions_.insert_or_assign(actionIdx, std::move(action));
}

void XYThetaEnv::createAndStoreMacroAction(Action& inAction)
{
    auto action = std::make_unique<Action>();
    *action = inAction;

    /// Compute relative discrete cells traversed by entire action
    if (action->xdim > 0 && action->ydim > 0)
    {
        for (auto x = 0; x < action->xdim; ++x) {
        for (auto y = 0; y < action->ydim; ++y) {
            action->intermediateCells.push_back(XYCell{x, y});
        }
        }
    }
    else if (action->xdim < 0 && action->ydim > 0)
    {
        for (auto x = 0; x > action->xdim; --x) {
        for (auto y = 0; y < action->ydim; ++y) {
            action->intermediateCells.push_back(XYCell{x, y});
        }
        }
    }
    else if (action->xdim > 0 && action->ydim < 0)
    {
        for (auto x = 0; x < action->xdim; ++x) {
        for (auto y = 0; y > action->ydim; --y) {
            action->intermediateCells.push_back(XYCell{x, y});
        }
        }
    }
    else if (action->xdim < 0 && action->ydim < 0)
    {
        for (auto x = 0; x > action->xdim; --x) {
        for (auto y = 0; y > action->ydim; --y) {
            action->intermediateCells.push_back(XYCell{x, y});
        }
        }
    }
    assert(action->intermediateCells.size() == abs(action->xdim * action->ydim));

    /// Get action coordinates
    int startAngle = angles::ContToDiscTheta(inAction.startState.theta);
    int primID = inAction.primID;
    std::pair<int, int> actionCoords(primID, startAngle);
    int actionIdx = actionCoordsToIdx_Macro(actionCoords);

    macroactions_.insert_or_assign(actionIdx, std::move(action));
}

void XYThetaEnv::ReadMprims(std::ifstream& mprimFileStream)
{
    std::string line;
    char split_char = ' ';
    std::vector<std::string> tokens;
    Action currentAction;

    int numIntermediatePoses = 0;
    int mprimCount = 0;

    // read line
    while (std::getline(mprimFileStream, line))
    {
        std::istringstream lineStream(line);
        std::string field;

        // parse current line
        while (lineStream >> field)
        {
            if (field == "resolution_m:")
            {
                // UNUSED
                std::string resolution;
                lineStream >> resolution;
                double res = std::stod(resolution);
            }
            else if (field == "numberofangles:")
            {
                std::string num;
                lineStream >> num;
                numTotalAngles_ = std::stoi(num);
            }
            else if (field == "totalnumberofprimitives:")
            {
                std::string prims;
                lineStream >> prims;
                numTotalPrims_ = std::stoi(prims);
                numPrimsPerAngle_ = numTotalPrims_ / numTotalAngles_;
            }
            else if (field == "primID:")
            {
                std::string id;
                lineStream >> id;
                currentAction.reset();
                currentAction.primID = std::stoi(id);
            }
            else if (field == "startangle_c:")
            {
                std::string angle;
                lineStream >> angle;
                currentAction.startState.theta =
                    angles::DiscToContTheta(std::stoi(angle));
            }
            else if (field == "duration:")
            {
                std::string duration;
                lineStream >> duration;
                currentAction.duration = std::stof(duration);
            }
            else if (field == "tv:")
            {
                std::string tv;
                lineStream >> tv;
                currentAction.tv = std::stof(tv);
            }
            else if (field == "rv:")
            {
                std::string rv;
                lineStream >> rv;
                currentAction.rv = std::stof(rv);
            }
            else if (field == "endpose_c:")
            {
                /* UNUSED */
                std::string endpose;
                lineStream >> endpose;
                lineStream >> endpose;
                lineStream >> endpose;
            }
            else if (field == "additionalactioncostmult:")
            {
                /* UNUSED */
                std::string costmult;
                lineStream >> costmult;
            }
            else if (field == "intermediateposes,intermdistance:")
            {
                std::string num;
                lineStream >> num;
                numIntermediatePoses = std::stoi(num);
            }
            else
            {
                // intermediate continuous states
                int count = 0;
                do
                {
                    auto poses = split(line, split_char);
                    XYThetaState state = {
                        std::stod(poses[0]), // x
                        std::stod(poses[1]), // y
                        std::stod(poses[2]), // theta
                        std::stod(poses[3])  // length
                    };
                    if (count != 0) currentAction.intermediateStates.push_back(state);
                    if (count++ == numIntermediatePoses - 1)
                        break;
                }
                while (std::getline(mprimFileStream, line));

                currentAction.endState = currentAction.intermediateStates.back();
                createAndStoreAction(currentAction);
                ++mprimCount;
                break;
            }
        }
    }
    assert(mprimCount == numTotalPrims_);
}

void XYThetaEnv::ReadMacroActions(std::ifstream& macroactionFileStream)
{
    std::string line;
    char split_char = ' ';
    std::vector<std::string> tokens;
    Action currentAction;

    int numIntermediatePoses = 0;
    int mprimCount = 0;

    // read line
    while (std::getline(macroactionFileStream, line))
    {
        std::istringstream lineStream(line);
        std::string field;

        // parse current line
        while (lineStream >> field)
        {
            if (field == "resolution_m:")
            {
                // UNUSED
                std::string resolution;
                lineStream >> resolution;
                double res = std::stod(resolution);
            }
            else if (field == "numberofangles:")
            {
                std::string num;
                lineStream >> num;
                numTotalMacroAngles_ = std::stoi(num);
            }
            else if (field == "totalnumberofprimitives:")
            {
                std::string prims;
                lineStream >> prims;
                numTotalMacroActions_ = std::stoi(prims);
                numMacroActionsPerAngle_ = numTotalPrims_ / numTotalMacroAngles_;
            }
            else if (field == "primID:")
            {
                std::string id;
                lineStream >> id;
                currentAction.reset();
                currentAction.primID = std::stoi(id);
            }
            else if (field == "startangle_c:")
            {
                std::string angle;
                lineStream >> angle;
                currentAction.startState.theta =
                    angles::DiscToContTheta(std::stoi(angle));
            }
            else if (field == "xydims:")
            {
                std::string xdim, ydim;
                lineStream >> xdim;
                lineStream >> ydim;
                currentAction.xdim = std::stoi(xdim);
                currentAction.ydim = std::stoi(ydim);
            }
            else if (field == "n_forward_straight:")
            {
                std::string n_forward_straight;
                lineStream >> n_forward_straight;
                currentAction.meta_data.n_forward_straight =
                    std::stoi(n_forward_straight);
            }
            else if (field == "n_backward_straight:")
            {
                std::string n_backward_straight;
                lineStream >> n_backward_straight;
                currentAction.meta_data.n_backward_straight =
                    std::stoi(n_backward_straight);
            }
            else if (field == "n_forward_backward_curve:")
            {
                std::string n_forward_backward_curve;
                lineStream >> n_forward_backward_curve;
                currentAction.meta_data.n_forward_backward_curve =
                    std::stoi(n_forward_backward_curve);
            }
            else if (field == "n_backward_forward_curve:")
            {
                std::string n_backward_forward_curve;
                lineStream >> n_backward_forward_curve;
                currentAction.meta_data.n_backward_forward_curve =
                    std::stoi(n_backward_forward_curve);
            }
            else if (field == "straight_length:")
            {
                std::string straight_length;
                lineStream >> straight_length;
                currentAction.meta_data.straight_length =
                    std::stod(straight_length);
            }
            else if (field == "arc_dtheta:")
            {
                std::string arc_dtheta;
                lineStream >> arc_dtheta;
                currentAction.meta_data.arc_dtheta =
                    std::stod(arc_dtheta);
            }
            else if (field == "intermediateposes,intermdistance:")
            {
                std::string num;
                lineStream >> num;
                numIntermediatePoses = std::stoi(num);
            }
            else
            {
                // intermediate continuous states
                int count = 0;
                do
                {
                    auto poses = split(line, split_char);
                    XYThetaState state = {
                        std::stod(poses[0]), // x
                        std::stod(poses[1]), // y
                        std::stod(poses[2]), // theta
                        std::stod(poses[3])  // length
                    };
                    if (numIntermediatePoses > 1)
                    {
                        if (count != 0) currentAction.intermediateStates.push_back(state);
                    }
                    else
                    {
                        currentAction.intermediateStates.push_back(state);
                    }
                    if (count++ >= numIntermediatePoses - 1) break;
                }
                while (std::getline(macroactionFileStream, line));

                currentAction.endState = currentAction.intermediateStates.back();
                createAndStoreMacroAction(currentAction);
                ++mprimCount;
                break;
            }
        }
    }
    assert(mprimCount == numTotalMacroActions_);
}

void XYThetaEnv::SetStart(RobotState& start_state)
{
    RobotCoord start_coord;
    stateToCoord(start_state, start_coord);
    startStateID_ = getOrCreateState(start_coord);
    assert(startStateID_ >= 0);
}

/// Sets goal state to check for potential goals
void XYThetaEnv::SetGoal(RobotState& goal_state)
{
    stateToCoord(goal_state, goalCoord_);
    printf("Set goal coords to [%d, %d, %d]\n", goalCoord_[0], goalCoord_[1], goalCoord_[2]);
}

auto XYThetaEnv::ConvertStatesToWaypoints(
    std::vector<int>& stateIDs,
    std::vector<int>& actionIdxs,
    double& lengthOfPathSoFar) -> std::vector<RobotState>
{
    std::vector<RobotState> solutionWaypoints = {};
    coveredCells_.clear();
    selectedFrontierCell_ = {};
    current_move_execution_time_ms_ = 0.0;
    current_plan_pattern_md_ = { -1, -1, -1, -1, -1, -1 };
    lengthOfPathSoFar_ = lengthOfPathSoFar;

    auto at_imaginary_goal = [&](auto& idx){ return &stateIDs[idx] == &stateIDs.back(); };
    auto at_frontier = [&](auto& idx){ return idx == stateIDs.size() - 2; };

    /// iterate over all state IDs except the last one (imaginary goal)
    for (auto i = 0; i < stateIDs.size() - 1; ++i)
    {
        /// ignore imaginary goal state
        if (at_imaginary_goal(i)) break;

        auto* solState = getHashEntry(stateIDs[i]);
        int actionIdx = actionIdxs[i+1];

        Action* action = nullptr;
        RobotCoord solStartCoord = solState->coord;
        RobotState solStartState;
        coordToState(solStartCoord, solStartState);

        // Get current action
        if (!at_frontier(i))
        {
            action = actions_.at(actionIdx).get();
            current_move_execution_time_ms_ += action->duration * 1e3;
        }
        else if (at_frontier(i))
        {
            selectedFrontierCell_ = solStartState;
            action = macroactions_.at(actionIdx).get();
            storeCurrentPlanPatternMetaData(*action);

            // if (kApproach == FSMA)
            // {
            //     action = macroactions_.at(actionIdx).get();
            // }
            // else if (kApproach == STANDARD)
            // {
            //     break;
            // }
        }

        // Iterate over action's waypoints
        for (const auto& waypoint : action->intermediateStates)
        {
            double theta = 0.0;
            if (action->intermediateStates.size() > 1) {
                theta = smpl::normalize_angle_positive(waypoint.theta);
            } else {
                // single-cell sense; add zero to current theta
                theta = smpl::normalize_angle(solStartState[2] + waypoint.theta);
            }

            RobotState addToSol = {
                solStartState[0] + waypoint.x,
                solStartState[1] + waypoint.y,
                theta,
                lengthOfPathSoFar_ + waypoint.l // length of path traveled so far
            };

            solutionWaypoints.push_back(addToSol);

            XYCell covered = {
                CONTXY2DISC(addToSol[0], 1.0),
                CONTXY2DISC(addToSol[1], 1.0)
            };
            if (coveredCells_.find(covered) == coveredCells_.end())
            {
                coveredCells_.insert(covered);
            }
        }
        lengthOfPathSoFar_ += action->endState.l;
    }
    reset();

    if (solutionWaypoints.empty() && stateIDs.size() == 2)
    {
        auto* sol_entry = getHashEntry(stateIDs[0]);
        RobotState solState;
        coordToState(sol_entry->coord, solState);
        solutionWaypoints.push_back(solState);
    }
    else if (stateIDs.size() == 1)
    {
        printf("    HANDLE THIS\n");
    }

    auto x_lattice = CONTXY2DISC(solutionWaypoints.back()[0], 1.0);
    auto y_lattice = CONTXY2DISC(solutionWaypoints.back()[1], 1.0);
    solutionWaypoints.back()[0] = x_lattice;
    solutionWaypoints.back()[1] = y_lattice;

    return solutionWaypoints;
}

CoveredCellsLookup XYThetaEnv::GetCoveredCells()
{
    return coveredCells_;
}

RobotState XYThetaEnv::GetSelectedFrontierCell()
{
    return selectedFrontierCell_;
}

void XYThetaEnv::GetSuccs(
    int& parent_id,
    std::vector<int>* succs,
    std::vector<double>* costs,
    std::vector<int>* primIDs)
{
    using namespace angles;

    if (parent_id == goalStateID_)
    {
        /// GetSuccs should not be called on imaginary goal
        assert(parent_id == 0);
        assert(false && "GetSuccs called on the imaginary goal!");
    }
    else if (IsGoal(parent_id))
    {
        /* Always call this function. If macro actions only consist of the one
         * single-sense action, then this is automatically the frontier-only
         * approach without macro actions.
         */
        getSuccsFrontierState(parent_id, succs, costs, primIDs);

        /// If only frontier-search based coverage, successors of potential goal
        /// include only the imaginary goal (and the potential goal's (x,y) cell
        /// is what is covered).
        ///
        /// If frontier-search + macro-actions (or sense actions), successors of
        /// potential goal include:
        /// (1) sensing only the potential goal's (x,y)
        /// (2) executing the macro action at the potential goal's (x,y)
        /// In both cases, the imaginary goal is put inserted into the OPEN list
        /// or updated in the OPEN list as a successor with the correspoding f/g-value
        /// resulting from g(potential goal) + cost of sense (macro) action
        //
        // if (kApproach == STANDARD)
        // {
        //     // StateEntry* parent_entry = getHashEntry(parent_id);
        //     // auto xParent = parent_entry->state[0];
        //     // auto yParent = parent_entry->state[1];
        //     // auto thetaParent = parent_entry->state[2];
        //     // printf("    [STANDARD] expanding a frontier state [%f, %f, %f]\n",
        //     //        xParent, yParent, thetaParent);

        //     succs->push_back(0);
        //     costs->push_back(1);
        //     primIDs->push_back(-1);
        // }
        // else if (kApproach == FSMA)
        // {
        //     getSuccsFrontierState(parent_id, succs, costs, primIDs);
        // }
        // else
        // {
        //     assert(false && "INVALID APPROACH!");
        // }
    }
    else
    {
        /// GetSuccs called on a non-frontier state
        /// Only apply move actions (motion primitives)
        StateEntry* parent_entry = getHashEntry(parent_id);
        auto xParent = parent_entry->coord[0];
        auto yParent = parent_entry->coord[1];
        auto thetaParent = parent_entry->coord[2];

        // printf("GetSuccs called on [%f, %f, %f]", xParent, yParent, thetaParent);

        for (auto primid = 0; primid < numPrimsPerAngle_; ++primid)
        {
            auto actionCoords = std::make_pair(primid, thetaParent);
            auto actionIdx = actionCoordsToIdx(actionCoords);
            auto* action = actions_.at(actionIdx).get();

            assert(ContToDiscTheta(action->startState.theta) == thetaParent);

            if (!validAction(parent_entry, action)) {
                // printf("  MOVE ACTION INVALID\n");
                continue;
            }

            auto action_cost = moveActionCost(parent_entry, action);

            auto xSuccessor = xParent + action->endState.x;
            auto ySuccessor = yParent + action->endState.y;
            auto thetaSuccessor = action->endState.theta;
            RobotState successorState = { xSuccessor, ySuccessor, thetaSuccessor };

            // printf("  got successor [%f, %f, %f]\n", xSuccessor, ySuccessor, thetaSuccessor);

            RobotCoord succ_coord;
            stateToCoord(successorState, succ_coord);
            int succID = getOrCreateState(succ_coord);

            succs->push_back(succID);
            costs->push_back(action_cost);
            primIDs->push_back(actionIdx);
        }
    }
}

void XYThetaEnv::getSuccsFrontierState(
    int& parent_id,
    std::vector<int>* succs,
    std::vector<double>* costs,
    std::vector<int>* primIDs)
{
    using namespace angles;

    StateEntry* parent_entry = getHashEntry(parent_id);
    auto xParent = parent_entry->coord[0];
    auto yParent = parent_entry->coord[1];
    auto thetaParent = parent_entry->coord[2];

    // printf(
    //     "[FSMA] expanding a frontier state id = [%d], state = [%f, %f, %f]\n",
    //     parent_id, xParent, yParent, thetaParent);

    /// Iterate over macro actions.
    /// Currently it doesn't matter what angle the robot is at when executing
    /// a macro action, so iterate over all macro actions.
    ///
    /// The final successor state is always the imaginary goal.
    /// When extracting the full path, add the full macro action for the last
    /// action.
    for(
    auto actionIdx = 0;
    actionIdx < numTotalMacroActions_;
    ++actionIdx)
    {
        auto* action = macroactions_.at(actionIdx).get();
        if (!validMacroAction(parent_entry, action)) {
            // printf("  SENSE (MACRO) ACTION [%d] INVALID\n", actionIdx);
            continue;
        }

        // // TODO: Either disable this, or enable this and allow turn-in-place
        // // only at frontier cells
        // if (thetaParent != angles::ContToDiscTheta(action->startState.theta)) {
        //     continue;
        // }

        double cost = senseActionCost(parent_entry, action);

        // if (actionIdx == 0)
        // {
        //     printf("  sense single cell:  cost = %d\n", cost);
        // }
        // else if (actionIdx > 0 && actionIdx < numTotalMacroActions_-1)
        // {
        //     printf("  sense macro action [%d]: cost = %d\n", actionIdx, cost);
        // }
        // else if (actionIdx > numTotalMacroActions_-1)
        // {
        //     // assert(false && "wrong macro action index");
        // }

        succs->push_back(0);
        costs->push_back(cost);
        primIDs->push_back(actionIdx);
    }
}

double XYThetaEnv::moveActionCost(StateEntry* state, Action* action)
{
    auto length = action->endState.l;
    if (length == 0) {
        auto cost = kTurnInPlaceCost;
        assert(cost > 0);
        return cost;
    } else {
        auto cost = length;
        assert(cost > 0);
        return cost;
    }

    // double action_duration_seconds = action->duration;
    // int action_duration_ms = (int)(action_duration_seconds * 1e3);

    // if (action->startState.x == action->endState.x &&
    //     action->startState.y == action->endState.y)
    // {
    //     auto cost = action_duration_ms * kTurnInPlacePenalty;
    //     assert(cost > 0);
    //     return cost;
    // } else {
    //     auto cost = action_duration_ms;
    //     assert(cost > 0);
    //     return cost;
    // }
}

double XYThetaEnv::senseActionCost(StateEntry* parent_entry, Action* action)
{
    /// get discrete cells traversed by pattern
    auto n_traversed = action->intermediateCells.size();
    auto sourceCoord = parent_entry->coord;

    double distance_traveled = action->endState.l;

    /// iterate over them and count stuff
    int n_covered = 0;
    for (const auto& cell : action->intermediateCells)
    {
        auto r = sourceCoord[1] + cell.y;
        auto c = sourceCoord[0] + cell.x;
        if (inBounds(r, c))
        {
            auto idx = GETMAPINDEX(r, c, rows_, cols_);
            if (movingai_->m_map[idx] == 2)
            {
                assert(movingai_->m_map[idx] != 0); // must not be an obstacle (collision-checking done before this)
                ++n_covered; // if cell is uncovered, count as covered
            }
        }
    }
    assert(n_traversed >= 0);

    /// get number of turns in this pattern
    int n_turns = action->meta_data.n_forward_backward_curve + action->meta_data.n_backward_forward_curve;

    /// calculate sense action cost
    // int cost = n_traversed - senseTravelRatio_ * n_covered + kCostConstant;
    // double cost = distance_traveled - senseTravelRatio_ * n_covered + kCostConstant;
    double cost = distance_traveled + n_turns - senseTravelRatio_ * n_covered + kCostConstant;
    assert(cost > 0);

    return cost;
}

/// Returns true if no intermediate point in action takes the robot to a state
/// out of bounds or in collision.
bool XYThetaEnv::validAction(StateEntry* parent_entry, Action* a)
{
    RobotCoord parent_coord = parent_entry->coord;
    RobotState parent_state;
    coordToState(parent_coord, parent_state);

    for (const auto& s : a->intermediateStates)
    {
        RobotState int_point = {
            parent_state[0] + s.x,
            parent_state[1] + s.y,
            s.theta
        };

        int x_disc = CONTXY2DISC(int_point[0], 1.0);
        int y_disc = CONTXY2DISC(int_point[1], 1.0);
        auto& r = y_disc; auto& c = x_disc;

        if (!inBounds(r, c)) {
            // printf("[%d %d]  OUT OF BOUNDS -> REJECT\n", r, c);
            return false;
        }

        auto idx = GETMAPINDEX(r, c, rows_, cols_);

        if (movingai_->m_map[idx] == 0) {
            // printf("[%d %d]  OBSTACLE -> REJECT\n", r, c);
            return false;
        }
    }
    return true;
}

bool XYThetaEnv::validMacroAction(StateEntry* parent_entry, Action* a)
{
    for (const auto& s : a->intermediateCells)
    {
        auto r = parent_entry->coord[1] + s.y;
        auto c = parent_entry->coord[0] + s.x;

        if (!inBounds(r, c)) {
            return false;
        }

        auto idx = GETMAPINDEX(r, c, rows_, cols_);
        if (movingai_->m_map[idx] == 0) {
            return false;
        }
    }
    return true;
}

bool XYThetaEnv::IsGoal(const int& stateID) const
{
    auto* goal_entry = getHashEntry(stateID);
    RobotCoord goal_coord = goal_entry->coord;
    if (bFrontier_)
    {
        if (bFirstPlannerCall_ && stateID == startStateID_)
        {
            bFirstPlannerCall_ = false;
            return true;
        }
        else
        {
            int r = goal_coord[1];
            int c = goal_coord[0];
            return isFrontierCell(r, c, movingai_->m_map, rows_, cols_);
        }
    }
    else
    {
        return goal_coord[0] == goalCoord_[0] && goal_coord[1] == goalCoord_[1];
    }
}

double XYThetaEnv::GetGoalHeuristic(int state_id)
{
    if (state_id == 0) return 0;

    if (bFrontier_)
    {
        return 0;
    }
    else
    {
        auto* state_entry = getHashEntry(state_id);
        RobotCoord state_coord = state_entry->coord;
        return sqrt(
            (state_coord[0] - goalCoord_[0]) * (state_coord[0] - goalCoord_[0]) +
            (state_coord[1] - goalCoord_[1]) * (state_coord[1] - goalCoord_[1])
        );
    }
}

bool XYThetaEnv::inBounds(RobotState& state)
{
    return state[0] >= 0 && state[0] < cols_ &&
           state[1] >= 0 && state[1] < rows_ &&
           state[2] >= 0 && state[2] < 2 * M_PI;
}

bool XYThetaEnv::inBounds(int& r, int& c)
{
    return c >= 0 && c < cols_ &&
           r >= 0 && r < rows_;
}

void XYThetaEnv::stateToCoord(RobotState& inState, RobotCoord& outCoord)
{
    int x_disc = CONTXY2DISC(inState[0], 1.0);
    int y_disc = CONTXY2DISC(inState[1], 1.0);
    int theta_disc = angles::ContToDiscTheta(inState[2]);
    outCoord = { x_disc, y_disc, theta_disc };
}

void XYThetaEnv::coordToState(RobotCoord& inCoord, RobotState& outState)
{
    double x_cont = (double)inCoord[0];
    double y_cont = (double)inCoord[1];
    double theta_cont = angles::DiscToContTheta(inCoord[2]);
    outState = { x_cont, y_cont, theta_cont };
}

int XYThetaEnv::getOrCreateState(RobotCoord& coord)
{
    int state_id = getHashEntry(coord);
    if (state_id < 0) {
        state_id = createHashEntry(coord);
    }
    return state_id;
}

int XYThetaEnv::getHashEntry(RobotCoord& coord)
{
    StateEntry state_entry;
    state_entry.coord = coord;
    auto sit = stateToID_.find(&state_entry);
    if (sit == stateToID_.end()) {
        return -1;
    }
    return sit->second;
}

auto XYThetaEnv::getHashEntry(int state_id) const -> StateEntry*
{
    if (state_id < 0 || state_id >= (int)states_.size()) {
        return nullptr;
    }
    return states_[state_id];
}

int XYThetaEnv::createHashEntry(RobotCoord& coord)
{
    int state_id = reserveHashEntry();
    StateEntry* state_entry = getHashEntry(state_id);

    state_entry->coord = coord;

    // map state -> state id
    stateToID_[state_entry] = state_id;

    return state_id;
}

int XYThetaEnv::reserveHashEntry()
{
    StateEntry* entry = new StateEntry;
    int state_id = (int)states_.size();

    // map state id -> state
    states_.push_back(entry);

    return state_id;
}

void XYThetaEnv::reset()
{
    for (auto* s : states_)
    {
        if (s != nullptr)
        {
            delete s;
            s = nullptr;
        }
    }
    states_.clear();
    stateToID_.clear();

    // reserve state ID 0 for imaginary goal
    goalStateID_ = reserveHashEntry();
    assert(goalStateID_ == 0);
    assert(states_.size() == 1);
}

void XYThetaEnv::storeCurrentPlanPatternMetaData(Action& action)
{
    current_plan_pattern_md_.n_forward_straight = action.meta_data.n_forward_straight;
    current_plan_pattern_md_.n_backward_straight = action.meta_data.n_backward_straight;
    current_plan_pattern_md_.n_forward_backward_curve = action.meta_data.n_forward_backward_curve;
    current_plan_pattern_md_.n_backward_forward_curve = action.meta_data.n_backward_forward_curve;
    current_plan_pattern_md_.straight_length = action.meta_data.straight_length;
    current_plan_pattern_md_.arc_dtheta = action.meta_data.arc_dtheta;
}

} // namespace cs