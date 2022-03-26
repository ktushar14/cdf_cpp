#include <smpl/unicycle/dubins.h>
#include <smpl/angles.h>
#include <smpl/spatial.h>
#include <smpl/unicycle/pose_2d.h>

#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>

double DEFAULT_RADIUS = 0.5;
double DEFAULT_LATTICE_RESOLUTION = 1.0;
double CURVE_SAMPLING_RESOLUTION = 0.03;

double STRAIGHT_SAMPLING_RESOLUTION;

enum DubinsPathType
{
    STRAIGHT = 0,
    CURVE
};

enum MacroActionType
{
    BOTTOM_LEFT = 0,
    TOP_LEFT,
    TOP_RIGHT,
    BOTTOM_RIGHT
};

MacroActionType g_macroActionTypes[4] =
{
    BOTTOM_LEFT, TOP_LEFT, TOP_RIGHT, BOTTOM_RIGHT
};

std::string OUTFILE_PREFIX = "../patterns/patterns_sampled_";

#define CONTXY2DISC(X, CELLSIZE) (int)std::round((X / CELLSIZE))

static
auto dist(const smpl::Vector2& u, const smpl::Vector2& v) -> double
{
    return (u - v).norm();
}

smpl::DubinsMotion GetDubinsPath(smpl::Pose2D& start, smpl::Pose2D& goal, double& radius)
{
    // Compute Dubins paths (n probably means the first n paths in
    // dubinsMotions[6] are valid)
    smpl::DubinsMotion dubinsMotions[6];
    int n = smpl::MakeDubinsPaths(start, goal, radius, dubinsMotions);

    // Find the shortest path by iterating over path lengths
    int shortest_idx = -1;
    double shortest_length = std::numeric_limits<double>::max();
    for (auto i = 0; i < n; ++i) {
        const auto& motion = dubinsMotions[i];
        if (motion.length() < shortest_length) {
            shortest_length = motion.length();
            shortest_idx = i;
        }
    }
    assert(shortest_idx >= 0 && shortest_idx < 6);

    return dubinsMotions[shortest_idx];
}

std::vector<smpl::Pose2D> SampleDubinsPath(
    smpl::DubinsMotion& dubinsPath, double& resolution)
{
    assert(resolution > 0);

    auto t_start = dubinsPath.t0();
    auto t_end = dubinsPath.t3();

    std::vector<smpl::Pose2D> sampled_path;

    for (double t = t_start; t < t_end; t += resolution)
    {
        auto pose = dubinsPath(t);
        if (t == t_start)
        {
            pose.x = CONTXY2DISC(pose.x, DEFAULT_LATTICE_RESOLUTION);
            pose.y = CONTXY2DISC(pose.y, DEFAULT_LATTICE_RESOLUTION);
        }
        // fprintf(fhandle, "%.2f,%.2f,%.2f,-1\n", pose.x, pose.y, pose.theta);
        // fprintf(fhandle, "%.2f %.2f %.2f -1\n", pose.x, pose.y, pose.theta);
        sampled_path.push_back(pose);
    }
    // add last state and make sure it is on the lattice
    auto pose = dubinsPath(t_end);
    pose.x = CONTXY2DISC(pose.x, DEFAULT_LATTICE_RESOLUTION);
    pose.y = CONTXY2DISC(pose.y, DEFAULT_LATTICE_RESOLUTION);
    sampled_path.push_back(pose);

    return sampled_path;
}

double IntermediateDubinsPathLength(smpl::DubinsMotion& dubinsPath, double& t)
{
    using namespace smpl;
    assert(t >= 0);

    auto t_start = dubinsPath.t0();
    auto t_end   = dubinsPath.t3();
    auto t_1     = dubinsPath.t1();
    auto t_2     = dubinsPath.t2();

    if (t == t_start) return 0.0;
    if (t == t_end) return dubinsPath.length();

    assert(t > t_start && t < t_end);

    if (t < t_1)
    {
        // on start turn
        Pose2D here = dubinsPath(t);
        Pose2D start = dubinsPath(t_start);
        return dubinsPath.radius * shortest_angle_dist(here.theta, start.theta);
    }
    else if (t > t_2)
    {
        // on goal turn
        Pose2D here = dubinsPath(t);
        Pose2D pivot2 = dubinsPath(t_2);
        return
            dubinsPath.radius * dubinsPath.arc1
            + dist(pos(dubinsPath.straight_start()), pos(dubinsPath.straight_end()))
            + dubinsPath.radius * shortest_angle_dist(here.theta, pivot2.theta);
    }
    else
    {
        // on straight segment
        Pose2D here = dubinsPath(t);
        Pose2D pivot1 = dubinsPath(t_1);
        return
            dubinsPath.radius * dubinsPath.arc1 + dist(pos(here), pos(pivot1));
    }
}

std::vector<double> SamplePathLengths(
    smpl::DubinsMotion& dubinsPath, double& resolution, const double& length_up_to_here)
{
    auto t_start = dubinsPath.t0();
    auto t_end = dubinsPath.t3();
    std::vector<double> cumulative_lengths;

    for (double t = t_start; t < t_end; t += resolution)
    {
        double intermediate_length = length_up_to_here + IntermediateDubinsPathLength(dubinsPath, t);
        cumulative_lengths.push_back(intermediate_length);
    }
    // last state
    double intermediate_length = length_up_to_here + IntermediateDubinsPathLength(dubinsPath, t_end);
    cumulative_lengths.push_back(intermediate_length);

    return cumulative_lengths;
}

/// Straight-line macro actions computed separately.
void ComputeAndPrintStraightMacroAction(int& xsize, int& ysize, std::string& filepath)
{
    assert(xsize == 1 || ysize == 1);

    // Generate straight-line macro action
    double length = 0.0;
    smpl::DubinsMotion continuous_path;
    if (xsize == 1)
    {
        smpl::Pose2D start =
            { 0.0, 0.0, smpl::to_radians(+90.0) };
        smpl::Pose2D end =
            { 0.0, (double)(ysize-1), smpl::to_radians(+90.0) };

        length = (double)(ysize-1);

        continuous_path = GetDubinsPath(start, end, DEFAULT_RADIUS);
    }
    else if (ysize == 1)
    {
        smpl::Pose2D start =
            { 0.0, 0.0, smpl::to_radians(0.0) };
        smpl::Pose2D end =
            { (double)(xsize-1), 0.0, smpl::to_radians(0.0) };

        length = (double)(xsize-1);

        continuous_path = GetDubinsPath(start, end, DEFAULT_RADIUS);
    }

    std::vector<smpl::Pose2D> sampled_macro_action;
    std::vector<double> cumulative_lengths;

    auto resolution = STRAIGHT_SAMPLING_RESOLUTION;
    // auto resolution = continuous_path.length()/200;
    auto sampled_path = SampleDubinsPath(continuous_path, resolution);
    auto sampled_lengths = SamplePathLengths(continuous_path, resolution, 0.0);
    assert(sampled_lengths.size() == sampled_path.size());

    sampled_macro_action.insert(sampled_macro_action.end(), sampled_path.begin(), sampled_path.end());
    cumulative_lengths.insert(cumulative_lengths.end(), sampled_lengths.begin(), sampled_lengths.end());

    int nposes = (int)sampled_macro_action.size();

    FILE* fhandle = fopen(filepath.c_str(), "a+");

    fprintf(fhandle, "xydims: %d %d\n", xsize, ysize);

    fprintf(fhandle, "n_forward_straight: 1\n");
    fprintf(fhandle, "n_backward_straight: 0\n");
    fprintf(fhandle, "n_forward_backward_curve: 0\n");
    fprintf(fhandle, "n_backward_forward_curve: 0\n");
    fprintf(fhandle, "straight_length: %.4f\n", length);
    fprintf(fhandle, "arc_dtheta: 0\n");

    fprintf(fhandle, "intermediateposes,intermdistance: %d\n", nposes);
    for (auto i = 0; i < nposes; ++i) {
        auto pose = sampled_macro_action[i];
        auto l = cumulative_lengths[i];
        fprintf(fhandle, "%.4f %.4f %.4f %.4f\n", pose.x, pose.y, smpl::normalize_angle_positive(pose.theta), l);
    }
    fclose(fhandle);

    // printf("Generated [%d x %d] straight pattern | ", xsize, ysize);
}

/// Rectangular macro actions.
void ComputeAndPrintMacroAction(
    int& xsize, int& ysize, const MacroActionType& type, std::string& filepath)
{
    auto even = [&](int x) { return x % 2 == 0; };
    auto odd =  [&](int x) { return !even(x); };

    int n_forward_straight,
    n_backward_straight,
    n_forward_backward_curve,
    n_backward_forward_curve;

    switch (type)
    {
    case BOTTOM_RIGHT:
    case BOTTOM_LEFT:
        n_forward_straight = even(xsize) ? std::floor(xsize/2) : std::floor(xsize/2)+1;
        n_backward_straight = std::floor(xsize/2);
        n_forward_backward_curve = std::floor(xsize/2);
        n_backward_forward_curve = odd(xsize) ? std::floor(xsize/2) : std::floor(xsize/2)-1;
        break;
    case TOP_RIGHT:
    case TOP_LEFT:
        n_backward_straight = even(xsize) ? std::floor(xsize/2) : std::floor(xsize/2)+1;
        n_forward_straight = std::floor(xsize/2);
        n_backward_forward_curve = std::floor(xsize/2);
        n_forward_backward_curve = odd(xsize) ? std::floor(xsize/2) : std::floor(xsize/2)-1;
        break;
    }


    int n_total_prims = n_forward_straight
                      + n_backward_straight
                      + n_forward_backward_curve
                      + n_backward_forward_curve;

    double forward_straight_x_starts[n_forward_straight];
    double forward_straight_x_ends[n_forward_straight];
    double forward_straight_y_starts[n_forward_straight];
    double forward_straight_y_ends[n_forward_straight];

    double backward_straight_x_starts[n_backward_straight];
    double backward_straight_x_ends[n_backward_straight];
    double backward_straight_y_starts[n_backward_straight];
    double backward_straight_y_ends[n_backward_straight];

    double forward_backward_x_starts[n_forward_backward_curve];
    double forward_backward_x_ends[n_forward_backward_curve];
    double forward_backward_y_starts[n_forward_backward_curve];
    double forward_backward_y_ends[n_forward_backward_curve];

    double backward_forward_x_starts[n_backward_forward_curve];
    double backward_forward_x_ends[n_backward_forward_curve];
    double backward_forward_y_starts[n_backward_forward_curve];
    double backward_forward_y_ends[n_backward_forward_curve];

    smpl::DubinsMotion continuous_macro_action[n_total_prims];
    DubinsPathType continuous_macro_action_type[n_total_prims];
    for (auto i = 0; i < n_total_prims; ++i)
        continuous_macro_action[i] = smpl::DubinsMotion();

    /// Determine start and end poses for each macro action segment

    switch (type)
    {
    case BOTTOM_LEFT:
        for (auto i = 0; i < n_forward_straight; ++i)
        {
            forward_straight_x_starts[i] = 2*i;
            forward_straight_x_ends[i]   = 2*i;

            forward_straight_y_starts[i] = 0;
            forward_straight_y_ends[i]   = ysize-1;
        }

        for (auto i = 0; i < n_backward_straight; ++i)
        {
            backward_straight_x_starts[i] = 2*i + 1;
            backward_straight_x_ends[i]   = 2*i + 1;

            backward_straight_y_starts[i] = ysize-1;
            backward_straight_y_ends[i]   = 0;
        }

        for (auto i = 0; i < n_forward_backward_curve; ++i)
        {
            forward_backward_x_starts[i] = 2*i;
            forward_backward_x_ends[i]   = 2*i + 1;

            forward_backward_y_starts[i] = ysize-1;
            forward_backward_y_ends[i]   = ysize-1;
        }

        for (auto i = 0; i < n_backward_forward_curve; ++i)
        {
            backward_forward_x_starts[i] = 2*i + 1;
            backward_forward_x_ends[i]   = 2*i + 2;

            backward_forward_y_starts[i] = 0;
            backward_forward_y_ends[i]   = 0;
        }
    break;

    case TOP_LEFT:
        for (auto i = 0; i < n_backward_straight; ++i)
        {
            backward_straight_x_starts[i] = 2*i;
            backward_straight_x_ends[i]   = 2*i;

            backward_straight_y_starts[i] = 0;
            backward_straight_y_ends[i]   = -(ysize-1);
        }

        for (auto i = 0; i < n_forward_straight; ++i)
        {
            forward_straight_x_starts[i] = 2*i + 1;
            forward_straight_x_ends[i]   = 2*i + 1;

            forward_straight_y_starts[i] = -(ysize-1);
            forward_straight_y_ends[i]   = 0;
        }

        for (auto i = 0; i < n_forward_backward_curve; ++i)
        {
            forward_backward_x_starts[i] = 2*i + 1;
            forward_backward_x_ends[i]   = 2*i + 2;

            forward_backward_y_starts[i] = 0;
            forward_backward_y_ends[i]   = 0;
        }

        for (auto i = 0; i < n_backward_forward_curve; ++i)
        {
            backward_forward_x_starts[i] = 2*i;
            backward_forward_x_ends[i]   = 2*i + 1;

            backward_forward_y_starts[i] = -(ysize-1);
            backward_forward_y_ends[i]   = -(ysize-1);
        }
    break;

    case TOP_RIGHT:
        for (auto i = 0; i < n_backward_straight; ++i)
        {
            backward_straight_x_starts[i] = 0 - (2*i);
            backward_straight_x_ends[i]   = 0 - (2*i);

            backward_straight_y_starts[i] = 0;
            backward_straight_y_ends[i]   = -(ysize-1);
        }

        for (auto i = 0; i < n_forward_straight; ++i)
        {
            forward_straight_x_starts[i] = 0 - (2*i + 1);
            forward_straight_x_ends[i]   = 0 - (2*i + 1);

            forward_straight_y_starts[i] = -(ysize-1);
            forward_straight_y_ends[i]   = 0;
        }

        for (auto i = 0; i < n_forward_backward_curve; ++i)
        {
            forward_backward_x_starts[i] = 0 - (2*i + 1);
            forward_backward_x_ends[i]   = 0 - (2*i + 2);

            forward_backward_y_starts[i] = 0;
            forward_backward_y_ends[i]   = 0;
        }

        for (auto i = 0; i < n_backward_forward_curve; ++i)
        {
            backward_forward_x_starts[i] = 0 - (2*i);
            backward_forward_x_ends[i]   = 0 - (2*i + 1);

            backward_forward_y_starts[i] = -(ysize-1);
            backward_forward_y_ends[i]   = -(ysize-1);
        }
    break;

    case BOTTOM_RIGHT:
        for (auto i = 0; i < n_forward_straight; ++i)
        {
            forward_straight_x_starts[i] = 0 - (2*i);
            forward_straight_x_ends[i]   = 0 - (2*i);

            forward_straight_y_starts[i] = 0;
            forward_straight_y_ends[i]   = ysize-1;
        }

        for (auto i = 0; i < n_backward_straight; ++i)
        {
            backward_straight_x_starts[i] = 0 - (2*i + 1);
            backward_straight_x_ends[i]   = 0 - (2*i + 1);

            backward_straight_y_starts[i] = ysize-1;
            backward_straight_y_ends[i]   = 0;
        }

        for (auto i = 0; i < n_forward_backward_curve; ++i)
        {
            forward_backward_x_starts[i] = 0 - (2*i);
            forward_backward_x_ends[i]   = 0 - (2*i + 1);

            forward_backward_y_starts[i] = ysize-1;
            forward_backward_y_ends[i]   = ysize-1;
        }

        for (auto i = 0; i < n_backward_forward_curve; ++i)
        {
            backward_forward_x_starts[i] = 0 - (2*i + 1);
            backward_forward_x_ends[i]   = 0 - (2*i + 2);

            backward_forward_y_starts[i] = 0;
            backward_forward_y_ends[i]   = 0;
        }
    break;
    }

    // generate straight forward paths
    for (auto i = 0; i < n_forward_straight; ++i)
    {
        smpl::Pose2D start =
            { forward_straight_x_starts[i], forward_straight_y_starts[i], smpl::to_radians(+90.0) };
        smpl::Pose2D end =
            { forward_straight_x_ends[i], forward_straight_y_ends[i], smpl::to_radians(+90.0) };

        auto path = GetDubinsPath(start, end, DEFAULT_RADIUS);
        // PrintPathToFile(path, filepath);

        switch (type)
        {
        case BOTTOM_RIGHT:
        case BOTTOM_LEFT:
            continuous_macro_action[4*i] = path;
            continuous_macro_action_type[4*i] = STRAIGHT;
            break;
        case TOP_RIGHT:
        case TOP_LEFT:
            continuous_macro_action[4*i + 2] = path;
            continuous_macro_action_type[4*i + 2] = STRAIGHT;
            break;
        }
    }

    // generate forward backward curves
    for (auto i = 0; i < n_forward_backward_curve; ++i)
    {
        smpl::Pose2D start =
            { forward_backward_x_starts[i], forward_backward_y_starts[i], smpl::to_radians(90.0) };
        smpl::Pose2D end =
            { forward_backward_x_ends[i], forward_backward_y_ends[i], smpl::to_radians(-90.0) };

        auto path = GetDubinsPath(start, end, DEFAULT_RADIUS);
        // PrintPathToFile(path, filepath);

        switch (type)
        {
        case BOTTOM_RIGHT:
        case BOTTOM_LEFT:
            continuous_macro_action[4*i + 1] = path;
            continuous_macro_action_type[4*i + 1] = CURVE;
            break;
        case TOP_RIGHT:
        case TOP_LEFT:
            continuous_macro_action[4*i + 3] = path;
            continuous_macro_action_type[4*i + 3] = CURVE;
            break;
        }
    }

    // generate straight backward paths
    for (auto i = 0; i < n_backward_straight; ++i)
    {
        smpl::Pose2D start =
            { backward_straight_x_starts[i], backward_straight_y_starts[i], smpl::to_radians(-90.0) };
        smpl::Pose2D end =
            { backward_straight_x_ends[i], backward_straight_y_ends[i], smpl::to_radians(-90.0) };

        auto path = GetDubinsPath(start, end, DEFAULT_RADIUS);
        // PrintPathToFile(path, filepath);

        switch (type)
        {
        case BOTTOM_RIGHT:
        case BOTTOM_LEFT:
            continuous_macro_action[4*i + 2] = path;
            continuous_macro_action_type[4*i + 2] = STRAIGHT;
            break;
        case TOP_RIGHT:
        case TOP_LEFT:
            continuous_macro_action[4*i] = path;
            continuous_macro_action_type[4*i] = STRAIGHT;
            break;
        }
    }

    // generate backward forward curves
    for (auto i = 0; i < n_backward_forward_curve; ++i)
    {
        smpl::Pose2D start =
            { backward_forward_x_starts[i], backward_forward_y_starts[i], smpl::to_radians(-90.0) };
        smpl::Pose2D end =
            { backward_forward_x_ends[i], backward_forward_y_ends[i], smpl::to_radians(+90.0) };

        auto path = GetDubinsPath(start, end, DEFAULT_RADIUS);
        // PrintPathToFile(path, filepath);

        switch (type)
        {
        case BOTTOM_RIGHT:
        case BOTTOM_LEFT:
            continuous_macro_action[4*i + 3] = path;
            continuous_macro_action_type[4*i + 3] = CURVE;
            break;
        case TOP_RIGHT:
        case TOP_LEFT:
            continuous_macro_action[4*i + 1] = path;
            continuous_macro_action_type[4*i + 1] = CURVE;
            break;
        }
    }

    // Sample each individual path to generate macro action. While sampling also
    // calculate cumulative path lengths.
    std::vector<smpl::Pose2D> sampled_macro_action;
    std::vector<double> cumulative_lengths;
    double length_up_to_here = 0.0;

    for (auto i = 0; i < n_total_prims; ++i)
    {
        auto path = continuous_macro_action[i];
        // if (path.length() > 0) continue;
        double resolution = 0.0;
        if (continuous_macro_action_type[i] == STRAIGHT) {
            resolution = STRAIGHT_SAMPLING_RESOLUTION;
            // resolution = path.length()/200;
        } else if (continuous_macro_action_type[i] == CURVE) {
            resolution = CURVE_SAMPLING_RESOLUTION;
        }

        auto sampled_path = SampleDubinsPath(path, resolution);
        auto sampled_lengths = SamplePathLengths(path, resolution, length_up_to_here);
        assert(sampled_path.size() == sampled_lengths.size());
        // length_up_to_here += sampled_lengths.back();
        length_up_to_here += path.length();

        sampled_macro_action.insert(sampled_macro_action.end(), sampled_path.begin(), sampled_path.end());
        cumulative_lengths.insert(cumulative_lengths.end(), sampled_lengths.begin(), sampled_lengths.end());
    }

    int nposes = (int)sampled_macro_action.size();
    assert(cumulative_lengths.size() == nposes);

    FILE* fhandle = fopen(filepath.c_str(), "a+");
    switch (type)
    {
        case BOTTOM_LEFT:
            fprintf(fhandle, "xydims: %d %d\n", +xsize, +ysize);
            break;
        case TOP_LEFT:
            fprintf(fhandle, "xydims: %d %d\n", +xsize, -ysize);
            break;
        case TOP_RIGHT:
            fprintf(fhandle, "xydims: %d %d\n", -xsize, -ysize);
            break;
        case BOTTOM_RIGHT:
            fprintf(fhandle, "xydims: %d %d\n", -xsize, +ysize);
            break;
    }

    fprintf(fhandle, "n_forward_straight: %d\n", n_forward_straight);
    fprintf(fhandle, "n_backward_straight: %d\n", n_backward_straight);
    fprintf(fhandle, "n_forward_backward_curve: %d\n", n_forward_backward_curve);
    fprintf(fhandle, "n_backward_forward_curve: %d\n", n_backward_forward_curve);
    fprintf(fhandle, "straight_length: %.4f\n", (double)(ysize-1));
    fprintf(fhandle, "arc_dtheta: 1.5708\n");

    fprintf(fhandle, "intermediateposes,intermdistance: %d\n", nposes);
    for (auto i = 0; i < nposes; ++i) {
        auto pose = sampled_macro_action[i];
        auto l = cumulative_lengths[i];
        fprintf(fhandle, "%.4f %.4f %.4f %.4f\n", pose.x, pose.y, smpl::normalize_angle_positive(pose.theta), l);
    }
    fclose(fhandle);

    // printf("Generated [%d x %d] pattern | ", xsize, ysize);
}

int main(int argc, char** argv)
{
    assert(argc == 3);
    int x_max = std::stoi(argv[1]);
    int y_max = std::stoi(argv[2]);

    int N_straight    = (x_max - 1) + (y_max - 1);
    int N_rectangular = (x_max - 1) * (y_max - 1);
    int N_total       =  N_straight + (4 * N_rectangular) + 1;

    auto filepath = OUTFILE_PREFIX + std::to_string(x_max) + "_" + std::to_string(y_max) + ".txt";
    FILE* outfile = fopen(filepath.c_str(), "w");

    fprintf(outfile, "resolution_m: 1.0\n");
    fprintf(outfile, "numberofangles: 1\n");
    fclose(outfile);

    int count = 0;

    if (x_max == 60)
    {
        outfile = fopen(filepath.c_str(), "a+");

        // fprintf(outfile, "totalnumberofprimitives: 485\n");
        // fprintf(outfile, "totalnumberofprimitives: 65\n");
        // fprintf(outfile, "totalnumberofprimitives: 37\n");
        fprintf(outfile, "totalnumberofprimitives: 25\n"); // ONLY SQUARE

        /// Single cell "pattern"
        fprintf(outfile, "primID: 0\n");
        fprintf(outfile, "startangle_c: 0\n");
        fprintf(outfile, "xydims: 1 1\n");
        fprintf(outfile, "n_forward_straight: 0\n");
        fprintf(outfile, "n_backward_straight: 0\n");
        fprintf(outfile, "n_forward_backward_curve: 0\n");
        fprintf(outfile, "n_backward_forward_curve: 0\n");
        fprintf(outfile, "straight_length: 0\n");
        fprintf(outfile, "arc_dtheta: 0\n");
        fprintf(outfile, "intermediateposes,intermdistance: 1\n");
        fprintf(outfile, "0.0 0.0 0.0 0.0\n");
        fclose(outfile);

        ++count;

        // int sizes[11] = { 6, 11, 16, 21, 26, 31, 36, 41, 46, 51, 56 };
        // int sizes[4] = { 10, 30, 50, 60 };
        // int sizes[3] = { 10, 40, 60 };
        int sizes[6] = { 10, 20, 30, 40, 50, 60 }; // ONLY SQUARE

        STRAIGHT_SAMPLING_RESOLUTION = 0.005;

        for (auto xsize : sizes) {
        for (auto ysize : sizes) {

            // FOR ONLY SQUARE PATTERNS
            if (xsize != ysize) continue;

            for (auto type : g_macroActionTypes)
            {
                outfile = fopen(filepath.c_str(), "a+");
                fprintf(outfile, "primID: %d\n", count);
                fprintf(outfile, "startangle_c: 0\n");
                fclose(outfile);
                ComputeAndPrintMacroAction(xsize, ysize, type, filepath);
                ++count;
            }
        }
        }
    }
    else if (x_max == 160)
    {
        outfile = fopen(filepath.c_str(), "a+");

        // fprintf(outfile, "totalnumberofprimitives: 785\n");
        // fprintf(outfile, "totalnumberofprimitives: 65\n");
        // fprintf(outfile, "totalnumberofprimitives: 37\n");
        fprintf(outfile, "totalnumberofprimitives: 25\n"); // ONLY SQUARE

        /// Single cell "pattern"
        fprintf(outfile, "primID: 0\n");
        fprintf(outfile, "startangle_c: 0\n");
        fprintf(outfile, "xydims: 1 1\n");
        fprintf(outfile, "n_forward_straight: 0\n");
        fprintf(outfile, "n_backward_straight: 0\n");
        fprintf(outfile, "n_forward_backward_curve: 0\n");
        fprintf(outfile, "n_backward_forward_curve: 0\n");
        fprintf(outfile, "straight_length: 0\n");
        fprintf(outfile, "arc_dtheta: 0\n");
        fprintf(outfile, "intermediateposes,intermdistance: 1\n");
        fprintf(outfile, "0.0 0.0 0.0 0.0\n");

        fclose(outfile);
        ++count;

        // int sizes[14] = { 12,  23,  34,  45,  56,  67,  78,  89, 100, 111, 122, 133, 144, 155 };
        // int sizes[4] = { 10, 60, 110, 160 };
        // int sizes[3] = { 10, 90, 160 };
        int sizes[6] = { 10, 40, 70, 100, 130, 160 }; // ONLY SQAURE

        STRAIGHT_SAMPLING_RESOLUTION = 0.005;

        for (auto xsize : sizes) {
        for (auto ysize : sizes) {

            // FOR ONLY SQUARE PATTERNS
            if (xsize != ysize) continue;

            for (auto type : g_macroActionTypes)
            {
                outfile = fopen(filepath.c_str(), "a+");
                fprintf(outfile, "primID: %d\n", count);
                fprintf(outfile, "startangle_c: 0\n");
                fclose(outfile);
                ComputeAndPrintMacroAction(xsize, ysize, type, filepath);
                ++count;
            }
        }
        }
    }
    else
    {
        printf("UNSUPPORTTED DIMENSIONS FOR SAMPLED PATTERNS !!!\n");
        exit (EXIT_FAILURE);
    }
    printf("count: %d\n", count);
}