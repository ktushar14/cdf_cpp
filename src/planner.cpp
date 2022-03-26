#include "covsearch/planner.hpp"
#include "covsearch/constants.hpp"
#include "covsearch/angles.hpp"

#include <iostream>
#include <cmath>

using sst = std::stringstream;

namespace cs {

Planner::Planner()
    : bResultReady_(false)
    , planObj_()
    , bRequestReceived_(false)
    , iteration_(-1)
    , current_planning_time_ms_(0.0)
    , current_move_execution_time_ms_(0.0)
    , errorCode_(-1)
    , numCoveredCells_(0)
    , numCellsToCover_(0)
    , numFrontierCells_(0)
    , bFirstPlannerCall_(false)
    , lengthOfPathSoFar_(0.0)
{
    // logs_.open("../logs/logs_planner.txt");
}

Planner::~Planner()
{
    // logs_.close();
}

void Planner::Init(double& sense_travel_ratio)
{
    if (kEnvironment == XY)
    {
        env_ = std::make_unique<XYEnv>();
    }
    else if (kEnvironment == XYTHETA)
    {
        env_ = std::make_unique<XYThetaEnv>();
    }
    env_->senseTravelRatio_ = sense_travel_ratio;
    search_ = std::make_unique<WAStar>(env_.get(), 1.0);
}

void Planner::ReadMprims(
    std::ifstream& mprimFileStream,
    std::ifstream& macroactionFileStream)
{
    env_->ReadMprims(mprimFileStream);
    if (kApproach == FSMA)
    {
        env_->ReadMacroActions(macroactionFileStream);
    }
}

auto Planner::SetMap(const std::string& mapfile) -> std::pair<int,int>
{
    env_->SetMap(mapfile);
    auto rows = env_->rows_;
    auto cols = env_->cols_;

    // count number of cells to cover
    for (auto r = 0; r < rows; ++r) {
    for (auto c = 0; c < cols; ++c) {
        auto idx = GETMAPINDEX(r, c, rows, cols);
        if (env_->movingai_->m_map[idx] == 2) {
            ++numCellsToCover_;
        }
    }
    }
    return std::pair<int,int>{ env_->rows_, env_->cols_ };
}

void Planner::CalcNumFrontierCells()
{
    numFrontierCells_ = 0;
    auto rows = env_->rows_;
    auto cols = env_->cols_;

    // count number of frontier cells
    for (auto r = 0; r < rows; ++r) {
    for (auto c = 0; c < cols; ++c) {
        if (isFrontierCell(r, c, env_->movingai_->m_map, rows, cols)) {
            ++numFrontierCells_;
        }
    }
    }
}

void Planner::GetRandomGoal(RobotState& goal)
{
    while (true)
    {
        int x,y;
        env_->movingai_->GetRandomState(x, y);
        auto idx = GETMAPINDEX((int)y, (int)x, env_->rows_, env_->cols_);

        if (env_->movingai_->m_map[idx] == 0)
        {
            printf("  goal %d, %d in collision\n", (int)x, (int)y);
            continue;
        }
        else
        {
            if (kEnvironment == XY)
            {
                goal = { (double)x, (double)y };
            }
            else if (kEnvironment == XYTHETA)
            {
                goal = { (double)x, (double)y, 0.0 };
            }
            break;
        }
    }
}

void Planner::GetCurrentGoalState(RobotState& currentGoal)
{
    std::lock_guard<std::mutex> lk(plannerMtx_);
    if (kApproach == RANDOM)
    {
        /// If random coverage, return a dummy state
        currentGoal = { 0.0, 0.0, 0.0 };
    }
    else
    {
        auto frontier_cell = env_->GetSelectedFrontierCell();
        currentGoal = { frontier_cell[0], frontier_cell[1], 0.0 }; // theta doesn't matter here
    }

}

void Planner::CopyLatestMap(MAP_t map)
{
    std::lock_guard<std::mutex> lk(plannerMtx_);
    auto r = env_->rows_;
    auto c = env_->cols_;
    memcpy(map, env_->movingai_->m_map, r*c*sizeof(decltype(*map)));
}

void Planner::UpdateMapWithCell(XYCell& xycell)
{
    auto r = xycell.y;
    auto c = xycell.x;
    auto idx = GETMAPINDEX(r, c, env_->rows_, env_->cols_);
    if (env_->movingai_->m_map[idx] != 0 && env_->movingai_->m_map[idx] != 1)
    {
        env_->movingai_->m_map[idx] = 1;
        ++numCoveredCells_;
    }
}

/// Get random goal, compute plan, repeat till planning successful
int Planner::NavigateToRandomGoal()
{
    int err = -1;
    while (err != SUCCESS)
    {
        GetRandomGoal(currentGoal_);
        SetGoal(currentGoal_);

        printf("currentStart_ :\n");
        for (auto s : currentStart_)
            printf("%f, ", s);
        printf("\n");

        printf("currentGoal_ :\n");
        for (auto s : currentGoal_)
            printf("%f, ", s);
        printf("\n");

        SetStart(currentStart_);

        err = RunSearch();
    }
    return err;
}

int Planner::FrontierWithMacroActions()
{
    int err = -1;

    SetStart(currentStart_);
    search_->SetGoal(0);

    /// Enable frontier search
    env_->bFrontier_ = true;

    err = RunSearch();

    if (err == SUCCESS) {
        printf("  Planner: SUCCESS\n");
    } else if (err == OPEN_EMPTY) {
        printf("  Planner: FAILURE (OPEN exhausted)\n");
    }

    return err;
}

bool Planner::inBounds(RobotState& state)
{
    return state[0] >= 0 && state[0] < env_->cols_ &&
           state[1] >= 0 && state[1] < env_->rows_ &&
           state[2] >= 0 && state[2] < 2 * M_PI;
}

int Planner::RandomCoverage()
{
    int err = -1;
    env_->bFrontier_ = false; /// disable frontier search

    /// Pick random angle
    std::random_device rd;     // only used once to initialise (seed) engine
    std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
    std::uniform_int_distribution<int> uni(0, 15); // guaranteed unbiased
    auto disc_angle = uni(rng); // random integer in range
    auto cont_angle = angles::DiscToContTheta(disc_angle);

    /// Path = long straight line
    auto x_here = currentStart_[0];
    auto y_here = currentStart_[1];
    randomPlan_ = {};
    for (auto a = 0.0; a < 1000; a += 0.1)
    {
        auto x = x_here + a * cos(cont_angle);
        auto y = y_here + a * sin(cont_angle);
        auto theta = currentStart_[2]; // does not matter for now
        RobotState state = { x, y, theta, lengthOfPathSoFar_ + a };
        lengthOfPathSoFar_ += a;

        /// If state out of bounds or in collision, stop
        int x_disc = CONTXY2DISC(x, 1.0);
        int y_disc = CONTXY2DISC(y, 1.0);
        auto& r = y_disc; auto& c = x_disc;

        if (!inBounds(state)) {
            // printf("[%d %d]  OUT OF BOUNDS -> REJECT\n", r, c);
            break;
        }

        auto idx = GETMAPINDEX(r, c, env_->rows_, env_->cols_);
        if (env_->movingai_->m_map[idx] == 0) {
            // printf("[%d %d]  OBSTACLE -> REJECT\n", r, c);
            break;
        }

        randomPlan_.push_back(state);
    }
    return SUCCESS;
}

// Notifies every thread waiting for it that it is ready to receive requests via
// the condition_variable cvReceiving_ (bReceiving_ = true). Once it receives a
// request, (bReceiving_ = false) it starts planning. Once done planning, it
// will wait for the next request (bReceiving_ = true).
void Planner::PlannerThread()
{
    std::unique_lock<std::mutex> lk(plannerMtx_);
    while (true)
    {
        /// Ready to receive requests
        bReceiving_ = true;
        cvReceiving_.notify_all();

        /// Wait until request received
        /// If errorCode_ is set to OPEN_EMPTY, it means that the previous
        /// request failed after exhausting the OPEN list. Assuming this means
        /// that there are no more frontier cells remaining, kill the planning
        /// thread.
        printf("Planner waiting for request ...\n");
        cvRequestReceived_.wait(lk, [&]() { return bRequestReceived_; });

        /// Request received; clear existing plan
        bReceiving_ = false;
        planObj_.wps_.clear();

        //////////////////////////////
        /// Call the main approach ///
        //////////////////////////////
        int err = -1;
        if (kApproach == FSMA)
        {
            err = FrontierWithMacroActions();
        }
        else if (kApproach == RANDOM)
        {
            int err = RandomCoverage();
        }
        else if (kApproach == STANDARD)
        {
            err = NavigateToRandomGoal();
        }

        ///////////////////
        /// Bookkeeping ///
        ///////////////////

        if (kApproach == RANDOM)
        {
            planObj_.SetNewPlan(randomPlan_);
        }
        else
        {
            /// Get continuous plan
            auto plan = GetContinuousSolution();
            planObj_.SetNewPlan(plan);
        }

        errorCode_ = err;

        /// Get execution time of current plan
        current_move_execution_time_ms_ = env_->current_move_execution_time_ms_;
        current_plan_pattern_md_ = env_->current_plan_pattern_md_;

        /// Continue ...
        bResultReady_ = true; // triggers GetNewPlan() to continue as result ready
        cvResultReady_.notify_all();
        bRequestReceived_ = false; // Done processing this request
    }
}

void Planner::logCurrentMap()
{
    auto rows = env_->rows_;
    auto cols = env_->cols_;
    logs_ << "map" << std::endl;
    for (auto r = 0; r < rows; ++r)
    {
        for (auto c = 0; c < cols; ++c)
        {
            auto idx = GETMAPINDEX(r, c, rows, cols);
            auto data = env_->movingai_->m_map[idx];
            logs_ << data << ",";
        }
        logs_ << std::endl;
    }
    logs_ << "end" << std::endl;
}

// Call this to start the planner. Once called, it spawns the planning thread
// and blocks until the planning thread receives a request. Once the planning
// thread receives a request, the function exits.
void Planner::Start()
{
    if (planningThread_.joinable()) {
        printf("Planner::planningThread_ already running\n");
        return;
    }

    planningThread_ = std::thread(&Planner::PlannerThread, this);
    printf("Planner::planningThread_ spawned.\n");

    // block and wait until planning thread receives a request
    std::unique_lock<std::mutex> lk(plannerMtx_);
    if (bReceiving_)
        return;
    cvReceiving_.wait(lk, [&]() { return bReceiving_ == true; });
}

PlannerResult Planner::GetNewPlan(cs::RobotState startState)
{
    printf("Planner::TrySendRequest()\n");
    if (!planningThread_.joinable())
    {
        printf("  Planner::planningThread_ not running! Cannot send request.\n");
        PlannerResult result;
        result.plan = cs::Plan();
        result.err = -1;
        return result;
    }

    /// Set current start state, set request received flag, notify planner
    {
        std::lock_guard<std::mutex> lk(plannerMtx_);
        currentStart_ = startState;
        bResultReady_ = false;
        bRequestReceived_ = true;
    }
    cvRequestReceived_.notify_one(); // triggers planning in PlannerThread

    /// Wait for planner result
    std::unique_lock<std::mutex> lk(plannerMtx_);
    printf("  Sent request to planner, waiting for result\n");
    cvResultReady_.wait(lk, [&]() { return bResultReady_; });

    /// Planner result found
    /// Set currentGoal_ to the frontier cell selected
    PlannerResult result;
    result.plan = planObj_.wps_;
    result.err = errorCode_;

    currentGoal_ = planObj_.wps_.front();
    return result;
}

/// CURRENTLY UNUSED
bool Planner::TryGetResult(cs::Plan& result)
{
    printf("Trying to get result ...\n");
    std::unique_lock<std::mutex> lk(plannerMtx_);
    printf("Waiting for result ...\n");
    cvReceiving_.wait(lk, [&]() { return bResultReady_; });

    result = planObj_.wps_;
    bResultReady_ = false;
    return !result.empty();

    // std::unique_lock<std::mutex> lk(plannerMtx_);
    // if (!bResultReady_) {
    //     printf("Result not ready\n");
    //     return false;
    // }
    // result = nextPlan_;
    // bResultReady_ = false;
    // return true;
}

void Planner::SetGoal(RobotState& inState)
{
    currentGoal_ = inState;
    env_->SetGoal(currentGoal_);
    search_->SetGoal(env_->GetGoalStateID());
}

void Planner::SetStart(RobotState& inState)
{
    env_->SetStart(inState);
    search_->SetStart(env_->GetStartStateID());
}

/// return 1 => SUCCESS
/// return 0 => FAILURE
int Planner::RunSearch()
{
    ++iteration_;
    CalcNumFrontierCells();

    /// If very first planner call, there will be zero frontier cells (if robot
    /// start is not marked covered). In this case alone, force the robot start
    /// location to be the selected frontier node and execute pattern here.
    if (numFrontierCells_ == 0 && bFirstPlannerCall_)
    {
        search_->env_->bFirstPlannerCall_ = true;
        bFirstPlannerCall_ = false;
    }

    auto then = hr_clk::now();
    int err = search_->Run();
    auto now = hr_clk::now();

    // planning time
    auto planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - then);
    current_planning_time_ms_ = planningTime.count();

    return err;
}

std::vector<int> Planner::GetSolutionStateIDs()
{
    return search_->GetSolution();
}

Plan Planner::GetContinuousSolution()
{
    auto solutionStateIDs = search_->GetSolution();
    auto solutionPrimIDs = search_->GetSolutionPrimIDs();
    auto continuous_solution = env_->ConvertStatesToWaypoints(solutionStateIDs, solutionPrimIDs, lengthOfPathSoFar_);
    lengthOfPathSoFar_ += env_->lengthOfPathSoFar_;
    return continuous_solution;
}

std::vector<int> Planner::GetExpandedStateIDs()
{
    return search_->GetExpandedStates();
}

std::vector<RobotState> Planner::GetExpandedCoords()
{
    printf("Planner::GetExpandedCoords() not implemented\n");
    std::vector<RobotState> dummy = {};
    return dummy;
}

} // namespace cs

// bool Planner::TrySendRequest(cs::RobotState startState)
// {
//     printf("Planner::TrySendRequest()\n");

//     if (!planningThread_.joinable()) {
//         printf("Planner::planningThread_ not running! Cannot send request.\n");
//         return false;
//     }
//     // if (bBusy_) return false;
//     // printf("Planner accepting requests\n");

//     currentStart_ = startState;

//     std::unique_lock<std::mutex> lk(plannerMtx_);
//     bRequestReceived_ = true;
//     lk.unlock();
//     cvRequestReceived_.notify_one();
//     return true;
// }