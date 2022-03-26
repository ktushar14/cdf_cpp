#include <smpl/unicycle/dubins.h>
#include <smpl/angles.h>

#include <iostream>
#include <fstream>
#include <limits>

// Run this and visualize with scripts/visdubins.py
int main()
{
    // set start and goal
    smpl::Pose2D start = { 0.0, 0.0, smpl::to_radians(0.0) }; // x, y, theta
    smpl::Pose2D goal  = { 30.0, 0.0, smpl::to_radians(0.0) }; // x, y, theta
    double radius = 0.5;

    smpl::DubinsMotion dubinsMotions[6];

    // compute paths
    // n probably means the first n paths in dubinsMotions[6] are valid
    int n = smpl::MakeDubinsPaths(start, goal, radius, dubinsMotions);

    // find the shortest path
    int shortest_idx = -1;
    double shortest_length = std::numeric_limits<double>::max();
    for (auto i = 0; i < n; ++i) {
        const auto& motion = dubinsMotions[i];
        if (motion.length() < shortest_length) {
            shortest_length = motion.length();
            shortest_idx = i;
        }
    }

    // and print it
    auto shortest_motion = dubinsMotions[shortest_idx];
    auto t_start = shortest_motion.t0();
    auto t_end = shortest_motion.t3();

    std::ofstream pathfile;
    pathfile.open("../scripts/dubins.txt");
    for (auto t = t_start; t <= t_end; t += 0.005) {
    // for (auto t = t_start; t <= t_end; t += 0.01) {
        auto pose = shortest_motion(t);
        pathfile << pose.x << "," << pose.y << "," << pose.theta << std::endl;
    }
    pathfile.close();
}