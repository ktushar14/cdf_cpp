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

// Project headers
#include "covsearch/angles.hpp"
#include "covsearch/constants.hpp"
#include "covsearch/planner.hpp"
#include "covsearch/types.hpp"
#include "covsearch/wastar.hpp"
#include "covsearch/xyenv.hpp"

std::string g_mprimPath = "../mprim/ICRA.mprim";
cs::RobotState g_startState;
std::unique_ptr<cs::Planner> P;
int g_rows, g_cols;
int g_plannerErrorCode;
double g_lengthOfPathSoFar = 0.0;

// log files
std::ofstream g_coverage_log;
std::ofstream g_timing_log;
std::ofstream g_info_log;

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

int main(int argc, char** argv)
{
    using namespace cs;

    /////////////////
    // Read inputs //
    /////////////////
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

    ////////////////////////////
    // Create log directories //
    ////////////////////////////
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
    std::string info_logs_path = this_map_pattern_dir + "info.txt";

    printf("map_type_log_dir: %s\n", map_type_log_dir.c_str());
    printf("this_map_log_dir: %s\n", this_map_log_dir.c_str());
    printf("this_map_pattern_dir: %s\n", this_map_pattern_dir.c_str());

    fs::create_directory(map_type_log_dir);
    fs::create_directory(this_map_log_dir);
    fs::create_directory(this_map_pattern_dir);

    /////////////////////////////////////////////////////
    // Initialization for planner and some bookkeeping //
    /////////////////////////////////////////////////////
    if (kEnvironment == XY)
    {
        g_startState = { 1.0, 20.0 };
    }
    else if (kEnvironment == XYTHETA)
    {
        g_startState = { 1.0, 1.0, 0.0 };
    }

    P = std::make_unique<Planner>();
    P->Init(sense_travel_ratio);

    if (patterns_type != "frontier_only") {
        P->bFirstPlannerCall_ = true;
    }

    auto rc = P->SetMap(map_file_path);
    g_rows = rc.first;
    g_cols = rc.second;
    P->ReadMprims(mprimFileStream, macroactionFileStream);

    g_coverage_log.open(coverage_logs_path);
    g_coverage_log << "x,y,theta,length,covered" << std::endl;

    g_timing_log.open(timing_logs_path);
    // g_timing_log << "iteration,planning_time_ms,move_exec_time_ms,pattern_exec_time_ms" << std::endl;
    g_timing_log << "iteration,planning_time_ms,move_exec_time_ms,n_straight,n_arc,l,dtheta" << std::endl;

    g_info_log.open(info_logs_path);

    // SET ROBOT START CELL AS COVERED if Frontier only
    if (patterns_type == "frontier_only") {
        cs::XYCell start_cell = {
            CONTXY2DISC(g_startState[0], 1.0),
            CONTXY2DISC(g_startState[1], 1.0)
        };
        P->UpdateMapWithCell(start_cell);
    }

    // Log total number of uncovered cells in map
    g_info_log << "total_uncovered_cells," << P->numCellsToCover_ << std::endl;

    ////////////////////////
    // Main coverage loop //
    ////////////////////////

    Plan current_plan = {};
    cs::RobotState currentStartState;
    P->env_->lengthOfPathSoFar_ = 0.0;

    while (true)
    {
        if (current_plan.empty())
        {
            if (!currentStartState.empty())
            {
                printf("[ERROR] currentStartState should be empty here!\n");
                exit (EXIT_FAILURE);
            }
            currentStartState = g_startState;
        }

        ///////////////////
        // Query planner //
        ///////////////////

        P->currentStart_ = g_startState;
        P->SetStart(P->currentStart_);
        P->search_->SetGoal(0);
        P->env_->bFrontier_ = true;

        if (kApproach == RANDOM)
        {
            int err = P->RandomCoverage();
            current_plan = P->randomPlan_;
        }
        else if (kApproach == FSMA)
        {
            int err = P->RunSearch(); // P->iteration_ is incremented here and
                                      // P->current_planning_time_ms_ is set here

            P->errorCode_ = err;
            if (err == SUCCESS) {
                printf("[Planner]: SUCCESS\n");
            } else if (err == OPEN_EMPTY) {
                printf("[Planner]: OPEN EMPTY\n");
                g_coverage_log.close();
                g_timing_log.close();
                g_info_log.close();
                exit(0);
            }

            current_plan = P->GetContinuousSolution();
        }

        g_lengthOfPathSoFar += P->env_->lengthOfPathSoFar_;

        //////////////////////////////////////////
        // Log planning and execution time data //
        //////////////////////////////////////////

        int iteration               = P->iteration_;
        double planning_time_ms     = P->current_planning_time_ms_;
        auto move_execution_time_ms = P->env_->current_move_execution_time_ms_;
        auto pmd                    = P->env_->current_plan_pattern_md_;

        g_timing_log
            <<        iteration
            << "," << planning_time_ms
            << "," << move_execution_time_ms
            << "," << (pmd.n_forward_straight + pmd.n_backward_straight)
            << "," << (pmd.n_forward_backward_curve + pmd.n_backward_forward_curve)
            << "," << pmd.straight_length
            << "," << pmd.arc_dtheta
        << std::endl;

        ////////////////////
        // "Execute" plan //
        ////////////////////

        for (RobotState& robot_state : current_plan)
        {
            //////////////////////////////////////////////////////////
            // Compute footprint and log coverage and distance data //
            //////////////////////////////////////////////////////////

            cs::CoveredCellsLookup covered;
            SensorFootprintAbsolute(robot_state[0], robot_state[1], covered);
            for (auto xycell : covered)
            {
                auto& map = P->env_->movingai_->m_map;
                auto& r = xycell.y;
                auto& c = xycell.x;
                auto idx = GETMAPINDEX(r, c, P->env_->rows_, P->env_->cols_);
                if (map[idx] != 0 && map[idx] != 1)
                {
                    map[idx] = 1;
                    ++P->numCoveredCells_;
                    // std::cout
                    //     <<            robot_state[0]
                    //     << "  |  " << robot_state[1]
                    //     << "  |  " << robot_state[2]
                    //     << "  |  " << robot_state[3]
                    //     << "  |  " << P->numCoveredCells_
                    // << std::endl;
                }
            }

            g_coverage_log
                <<        robot_state[0]
                << "," << robot_state[1]
                << "," << robot_state[2]
                << "," << robot_state[3]
                << "," << P->numCoveredCells_
            << std::endl;

            // If coverage has reached 70%, STOP (for very large maps)
            int num_total = P->numCellsToCover_;
            int num_covered = P->numCoveredCells_;
            double fraction = (num_covered/num_total);
            // printf("percent covered: %f\n", fraction*100);
            if (fraction > 0.69) {
                g_coverage_log.close();
                g_timing_log.close();
                g_info_log.close();
                exit(0);
            }
        }

        /////////////////////////
        // Set new start state //
        /////////////////////////

        currentStartState = current_plan.back();
    }

    g_coverage_log.close();
    g_timing_log.close();
    g_info_log.close();
}