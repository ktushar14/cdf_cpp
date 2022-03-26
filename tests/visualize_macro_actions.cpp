#include <string>
#include <fstream>
#include <memory>
#include <mutex>

#include <QApplication>

#include "covsearch/robot_space.hpp"
#include "covsearch/wastar.hpp"
#include "covsearch/xytheta.hpp"
#include "covsearch/constants.hpp"
#include "covsearch/types.hpp"
#include "covsearch/helpers.hpp"
#include "covsearch/angles.hpp"
#include "qt/dialog.h"

using namespace cs;

class PlanClass {
public:
    cs::Plan wps_;
    std::mutex mtx_;
    cs::RobotState lastWp_;

    cs::RobotState getWaypoint()
    {
        std::lock_guard lk(mtx_);
        if (wps_.empty()) {
            cs::RobotState s = {};
            return s;
        } else {
            cs::RobotState s = std::move(wps_.back()); // AAH! ???
            wps_.pop_back();
            return s;
        }
    }

    void SetNewPlan(cs::Plan plan)
    {
        std::lock_guard lk(mtx_);
        if (!wps_.empty())
            wps_.clear();
        std::reverse(plan.begin(), plan.end());
        wps_ = plan;
        lastWp_ = plan.front();
    }
};

PlanClass g_plan_obj;
std::unique_ptr<cs::RobotSpace> g_env;
std::unique_ptr<vis::Dialog> g_dialog;
double g_robotStart[2] = { 0.0, 0.0};
int g_actionIdx = 0;

int numTotalMacroActions_;
int numMacroActionsPerAngle_;
int numTotalMacroAngles_;

int numTotalPrims_;
int numPrimsPerAngle_;
int numTotalAngles_;

std::unordered_map<int, std::unique_ptr<Action>> macroactions_;

int actionCoordsToIdx_Macro(const std::pair<int, int>& coords)
{
    auto primID = coords.first;
    auto discAngle = coords.second;
    return primID + numMacroActionsPerAngle_ * discAngle;
}

std::pair<int, int> actionIdxToCoords_Macro(const int& actionIdx)
{
    int discAngle = actionIdx / numMacroActionsPerAngle_;
    int primID = actionIdx % numMacroActionsPerAngle_;
    return std::pair<int, int>(primID, discAngle);
}

void createAndStoreMacroAction(Action& inAction)
{
    auto action = std::make_unique<Action>();
    *action = inAction;
    int startAngle = angles::ContToDiscTheta(inAction.startState.theta);
    int primID = inAction.primID;
    std::pair<int, int> actionCoords(primID, startAngle);
    int actionIdx = actionCoordsToIdx_Macro(actionCoords);
    macroactions_.insert_or_assign(actionIdx, std::move(action));
}

void ReadMacroActions(std::ifstream& macroactionFileStream)
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
            else if (field == "intermediateposes:")
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

void GetCurrentGoalState(cs::RobotState& state)
{
    state = {
        -10.0 * vis::kOneCellPx,
        -10.0 * vis::kOneCellPx,
        0.0
    };
}

void GetUpdatedMap(cs::MAP_t map)
{
    auto r = g_env->rows_;
    auto c = g_env->cols_;
    memcpy(map, g_env->movingai_->m_map, r*c*sizeof(decltype(*map)));
}

void GetNextState(cs::RobotState& state)
{
    // set state for visualization
    cs::RobotState s;
    s = g_plan_obj.getWaypoint();

    if (s.empty())
    {
        if (g_actionIdx == macroactions_.size()-1)
        {
            printf("DONE!\n");
            getchar();
        }

        cs::Plan plan;
        auto* action = macroactions_[g_actionIdx].get();
        for (auto wp : action->intermediateStates)
        {
            RobotState s =
            {
                g_robotStart[0] + wp.x,
                g_robotStart[1] + wp.y,
                wp.theta
            };

            plan.push_back(s);
        }
        g_plan_obj.SetNewPlan(plan);
        g_dialog->SetCurrentRobotPlan(plan);
        ++g_actionIdx;
    }
    else
    {
        state = {
            s[0] * vis::kOneCellPx,
            s[1] * vis::kOneCellPx,
            s[2]
        };
    }
}

int main(int argc, char** argv)
{
    /// Create dummy environment
    g_env = std::make_unique<cs::XYThetaEnv>();

    /// Read macro actions
    auto macroactionPath = argv[1];
    std::ifstream macroactionFileStream(macroactionPath);
    ReadMacroActions(macroactionFileStream);

    /// Read dummy map
    std::string mapfile = "../resources/maps/emptymap.map";
    g_env->SetMap(mapfile);

    /// Create visualization window and setup stuff
    QApplication a(argc, argv);
    g_dialog = std::make_unique<vis::Dialog>();
    g_dialog->SetupGraphicsScene(g_env->rows_, g_env->cols_);
    g_dialog->SetupMapItem(g_env->rows_, g_env->cols_);
    g_dialog->SetupRobotItem();
    g_dialog->SetupTimer();
    g_dialog->Show();
    g_dialog->StartTimer();

    /// Dummy callbacks for robot, map, and goal updates
    g_dialog->robotItem_->ConnectStateUpdateCB(
        [&](cs::RobotState& state) { return GetNextState(state); });

    // connect map update callback
    g_dialog->mapItem_->ConnectMapUpdateCB(
        [&](cs::MAP_t updatedMap) { return GetUpdatedMap(updatedMap); });

    g_dialog->mapItem_->ConnectGoalUpdateCB(
        [&](cs::RobotState& state) { return GetCurrentGoalState(state); });

    a.exec();
}