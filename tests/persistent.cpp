// C system headers
#include <cassert>

// C++ standard library headers
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <filesystem>
#include <cstdlib>
namespace fs = std::filesystem;

// Other library headers
#include <QApplication>

// Project headers
#include "covsearch/angles.hpp"
#include "covsearch/constants.hpp"
#include "covsearch/planner.hpp"
#include "covsearch/types.hpp"
#include "covsearch/wastar.hpp"
#include "covsearch/xyenv.hpp"
#include "qt/dialog.h"

#define QT_VISUALIZATION 1

// std::string g_mprimPath = "../mprim/ICRA_2.mprim";
std::string g_mprimPath = "../mprim/unicycle_lengths_res_1m.mprim";
cs::RobotState g_startState;
std::unique_ptr<cs::Planner> g_planner;
std::unique_ptr<vis::Dialog> g_dialog;
int g_rows, g_cols;
int g_plannerErrorCode;

// log files
std::ofstream g_coverage_log;
std::ofstream g_timing_log;
std::ofstream g_iterations_log;
std::ofstream g_expansions_log;
std::ofstream g_infofile;

enum CellStatus
{
    kCellUncovered = 0,
    kCellCovered,
    kCellBlocked
};

class CSVRow {
public:
    std::string const& operator[](std::size_t index) const
    {
        return data_[index];
    }

    std::size_t size() const { return data_.size(); }

    void readNextRow(std::istream& str)
    {
        std::string line;
        std::getline(str, line);

        std::stringstream lineStream(line);
        std::string cell;

        data_.clear();
        while (std::getline(lineStream, cell, ',')) {
            data_.push_back(cell);
        }
        if (!lineStream && cell.empty()) { // This checks for a trailing comma with
            // no data after it.
            data_.push_back(
                ""); // If there was a trailing comma then add an empty element.
        }
    }

private:
    std::vector<std::string> data_;
};

std::istream& operator>>(std::istream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
}

/// Currently not used.
void ReadBooleanMap(std::ifstream& file, cs::BooleanMap* coverageMap, int& rows,
    int& cols)
{
    CSVRow row;
    while (file >> row) {
        std::vector<int> map_row;
        if (row[0] == "rows") {
            rows = std::stoi(row[1]);
            continue;
        }
        if (row[0] == "cols") {
            cols = std::stoi(row[1]);
            continue;
        }
        for (auto i = 0; i < row.size(); ++i) {
            map_row.push_back(std::stoi(row[i]));
            continue;
        }
        coverageMap->push_back(map_row);
    }
    printf("Read map with %d rows and %d cols\n", (int)coverageMap->size(),
        (int)coverageMap->at(0).size());
}

void PrintBooleanMap(cs::BooleanMap* coverageMap)
{
    assert(coverageMap->size() != 0);
    auto numRows = coverageMap->size();
    auto numCols = coverageMap->at(0).size();
    for (auto r = 0; r < numRows; ++r) {
        for (auto c = 0; c < numCols; ++c) {
            printf("%d,", coverageMap->at(r).at(c));
        }
        printf("\n");
    }
}

/// Gets the updated map from the planner. This function is connected as a
/// callback in the dialog which calls it repeatedly to visualize the map.
void GetUpdatedMap(cs::MAP_t map)
{
    g_planner->CopyLatestMap(map);
}

/// Attempts to get the next state to be realized by the robot from the planner.
/// If not available, sends a request to the planner to compute a new plan and
/// waits till the result is ready. This function is connected as a callback in
/// the dialog which calls it repeatedly to visualize the robot.
void GetNextState(cs::RobotState& state)
{
    cs::RobotState s;
    s = g_planner->planObj_.getWaypoint();

    /// If next state not available, query planner for new plan.
    if (s.empty())
    {
        printf("  s empty, need new plan\n");

        cs::RobotState startState;
        if (!g_planner->planObj_.lastWp_.empty())
        {
            /// start is last state in previous plan
            startState = g_planner->planObj_.lastWp_;
        }
        else
        {
            /// very first plan, start is g_startState
            startState = g_startState;
        }

        auto planner_result = g_planner->GetNewPlan(startState);
        auto plan = planner_result.plan;
        g_plannerErrorCode = planner_result.err;

        /// Log timing information.
        /// "Collect" these logs in Planner::RunSearch().
        int iteration = g_planner->iteration_;
        double planning_time_ms = g_planner->current_planning_time_ms_;
        double move_execution_time_ms = g_planner->current_move_execution_time_ms_;
        auto pmd = g_planner->current_plan_pattern_md_; // pattern meta data

        int num_frontier_cells = g_planner->numFrontierCells_;
        int num_expansions = g_planner->search_->numExpands_;
        double avg_time_per_expansion = (double)planning_time_ms/num_expansions;

        g_timing_log
            <<        iteration
            << "," << planning_time_ms
            << "," << move_execution_time_ms
            << "," << (pmd.n_forward_straight + pmd.n_backward_straight)
            << "," << (pmd.n_forward_backward_curve + pmd.n_backward_forward_curve)
            << "," << pmd.straight_length
            << "," << pmd.arc_dtheta
        << std::endl;

        g_iterations_log
            <<        iteration
            << "," << planning_time_ms
            << "," << num_frontier_cells
            << "," << num_expansions
            << "," << avg_time_per_expansion
        << std::endl;

        if (g_plannerErrorCode == cs::OPEN_EMPTY)
        {
            /// DIRTY AF HACK! Clean up later.
            g_coverage_log.close();
            g_timing_log.close();
            g_iterations_log.close();
            g_expansions_log.close();
            std::terminate();
        }

        if (cs::kEnvironment == cs::XY)
        {
            printf("  Goal achieved: [%f, %f]\n", plan.front().at(0), plan.front().at(1));
        }
        else if (cs::kEnvironment == cs::XYTHETA)
        {
            printf("  Goal achieved: [%f, %f, %f]\n", plan.front().at(0), plan.front().at(1), plan.front().at(2));
        }

        g_dialog->SetCurrentRobotPlan(plan);
        s = g_planner->planObj_.getWaypoint();
        return;
    }

    cs::CoveredCellsLookup covered;
    SensorFootprintAbsolute(s[0], s[1], covered);
    for (auto xycell : covered)
    {
        /* NOT DONE ANYMORE: g_planner->UpdateMapWithCell(xycell); */
        auto& map = g_planner->env_->movingai_->m_map;
        auto& r = xycell.y;
        auto& c = xycell.x;
        auto idx = GETMAPINDEX(r, c, g_planner->env_->rows_, g_planner->env_->cols_);
        if (map[idx] != 0 && map[idx] != 1)
        {
            map[idx] = 1;
            ++g_planner->numCoveredCells_;
            // log current robot state and map coverage percentage
        }
        g_coverage_log << s[0] << "," << s[1] << "," << s[2] << "," << s[3] << "," << g_planner->numCoveredCells_ << std::endl;
    }

    // set state for visualization
    state = {
        s[0] * vis::kOneCellPx,
        s[1] * vis::kOneCellPx,
        s[2]
    };
}

void GetCurrentGoalState(cs::RobotState& state)
{
    g_planner->GetCurrentGoalState(state);
}

int main(int argc, char** argv)
{
    using namespace cs;

    if (argc < 4)
    {
        printf("Need more arguments!\n");
        printf("Usage: ./persistent [path to map] [path to pattern file] [sense-travel-cost ratio (lambda)\n");
        printf("Example: ./persistent ../maps/corridors_and_rooms/0.map ../patterns/patterns_20_20.txt 2.0\n");
        exit (EXIT_FAILURE);
    }

    std::string map_file_path(argv[1]);
    std::string patterns_file_path(argv[2]);
    double sense_travel_ratio = std::stoi(argv[3]);

    std::ifstream mprimFileStream(g_mprimPath);
    std::ifstream macroactionFileStream(patterns_file_path);

    // Create log directories
    std::vector<std::string> map_path_seg_list = split(map_file_path, '/');
    std::string map_type = map_path_seg_list.end()[-2];
    std::vector<std::string> map_file_split = split(map_path_seg_list.back(), '.');
    std::string map_name = map_file_split[0];

    std::vector<std::string> patterns_path_seg_list = split(patterns_file_path, '/');
    std::string patterns_file = patterns_path_seg_list.end()[-1];
    std::vector<std::string> patterns_file_split = split(patterns_file, '.');
    std::string patterns_type = patterns_file_split[0];

    std::string map_type_log_dir = "../results/" + map_type + "/";
    std::string this_map_log_dir = map_type_log_dir + map_name + "/";

    std::string this_map_pattern_dir;

    if (kApproach == RANDOM) {
        this_map_pattern_dir = this_map_log_dir + "random" + "/";
    } else {
        this_map_pattern_dir = this_map_log_dir + patterns_type + "/";
    }

    std::string coverage_logs_path = this_map_pattern_dir + "coverage.txt";
    std::string timing_logs_path = this_map_pattern_dir + "timing.txt";
    std::string iterations_log_path = this_map_pattern_dir + "iterations.txt";
    std::string expansions_log_path = this_map_pattern_dir + "expansions.txt";
    // std::string plan_logs_path = this_map_pattern_dir + "plan.txt";

    printf("map_type_log_dir: %s\n", map_type_log_dir.c_str());
    printf("this_map_log_dir: %s\n", this_map_log_dir.c_str());
    printf("this_map_pattern_dir: %s\n", this_map_pattern_dir.c_str());

    fs::create_directory(map_type_log_dir);
    fs::create_directory(this_map_log_dir);
    fs::create_directory(this_map_pattern_dir);

    if (kEnvironment == XY)
    {
        g_startState = { 1.0, 20.0 };
    }
    else if (kEnvironment == XYTHETA)
    {
        g_startState = { 1.0, 1.0, 0.0 };
    }
    g_planner = std::make_unique<Planner>();
    g_planner->Init(sense_travel_ratio);

    if (patterns_type != "frontier_only") {
        g_planner->bFirstPlannerCall_ = true;
    }

    auto rc = g_planner->SetMap(map_file_path);
    g_rows = rc.first;
    g_cols = rc.second;
    g_planner->ReadMprims(mprimFileStream, macroactionFileStream);

    printf("Read all mprims and patterns\n");

    // Open log files
    g_coverage_log.open(coverage_logs_path);
    g_coverage_log << "x,y,theta,length,covered" << std::endl;

    g_timing_log.open(timing_logs_path);
    g_timing_log << "iteration,planning_time_ms,move_exec_time_ms,n_straight,n_arc,l,dtheta" << std::endl;

    g_iterations_log.open(iterations_log_path);
    g_iterations_log << "iteration,search_time,num_frontier_cells,num_expansions,avg_time_per_expansion" << std::endl;

    g_expansions_log.open(expansions_log_path);
    g_expansions_log << "iteration_number,expansion_number,expansion_time,evaluated_pattern_dim" << std::endl;

    // SET ROBOT START CELL AS COVERED if Frontier only
    if (patterns_type == "frontier_only") {
        cs::XYCell start_cell = {
            CONTXY2DISC(g_startState[0], 1.0),
            CONTXY2DISC(g_startState[1], 1.0)
        };
        g_planner->UpdateMapWithCell(start_cell);
    }

    // START PLANNING THREAD, WAITS FOR REQUESTS
    g_planner->Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

#if QT_VISUALIZATION

    QApplication a(argc, argv);
    printf("Created QApplication\n");
    // a.aboutQt(); // Prints Qt version info (currently have 5.9.5)

    g_dialog = std::make_unique<vis::Dialog>();
    g_dialog->SetupGraphicsScene(g_rows, g_cols);
    g_dialog->SetupMapItem(g_rows, g_cols);
    g_dialog->SetupRobotItem();
    g_dialog->SetupTimer();
    g_dialog->Show();

    // start timer
    g_dialog->StartTimer();

    // connect robot state update callback
    g_dialog->robotItem_->ConnectStateUpdateCB(
        [&](RobotState& state) { return GetNextState(state); }
    );

    // connect map update callback
    g_dialog->mapItem_->ConnectMapUpdateCB(
        [&](MAP_t updatedMap) { return GetUpdatedMap(updatedMap); }
    );

    // connect goal state update callback
    g_dialog->mapItem_->ConnectGoalUpdateCB(
        [&](RobotState& state) { return GetCurrentGoalState(state); }
    );

    printf("Dialog setup done\n");
    a.exec();

#else

    g_coverage_log.open("../logs/paths.txt");
    while (true)
    {
        // Kinda like a 10-ms timer ticking
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        RobotState s;
        s = g_planner->planObj_.getWaypoint();

        if (s.empty()) {
            printf("  s empty, need new plan\n");

            RobotState startState;
            if (!g_planner->planObj_.lastWp_.empty()) {
                startState = g_planner->planObj_.lastWp_;
                printf("  Last state: %f, %f, %f\n", startState[0], startState[1], startState[2]);
            } else {
                startState = g_startState;
            }

            auto plan = g_planner->GetNewPlan(startState);

            if (!plan.empty())
                printf("  CoverageSystem: got result from planner.\n");
            s = g_planner->planObj_.getWaypoint();
        }

        // if (s.empty()) {
        //     g_coverage_log << "Empty" << std::endl;
        // } else {
        //     g_coverage_log << s[0] << "," << s[1] << "," << s[2] << std::endl;
        // }
    }

#endif

g_coverage_log.close();
g_timing_log.close();
g_iterations_log.close();
g_expansions_log.close();
}