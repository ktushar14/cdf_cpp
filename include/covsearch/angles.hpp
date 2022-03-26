#include <cmath>
#include <smpl/angles.h>
//------------------------------------------------------------------------------
// Functions from smpl/angles.h:
// namespace smpl {
//
//     Convert an angle specified in radians to degrees:
//     constexpr double to_degrees(double rads);
//
//     Convert an angle specified in degrees to radians:
//     constexpr double to_radians(double degs);
//
//     Normalize an angle into the range [-pi, pi]:
//     inline double normalize_angle(double angle);
//
//     inline double normalize_angle_positive(double angle);
//
//     Return the shortest signed difference between two angles:
//     inline double shortest_angle_diff(double af, double ai);
//
//     Return the shortest distance between two angles:
//     inline double shortest_angle_dist(double af, double ai);
//
//     inline double minor_arc_diff(double af, double ai);
//
//     inline double major_arc_diff(double af, double ai);
//
//     inline double minor_arc_dist(double af, double ai);
//
//     inline double major_arc_dist(double af, double ai);
//
//     Return the closest angle equivalent to af that is numerically greater
//     than ai:
//     inline double unwind(double ai, double af);
// }
//------------------------------------------------------------------------------

#pragma once

const int DEFAULT_NUM_ANGLES = 16; // TODO: move to constants file

namespace angles {

// converts continuous (radians) version of angle into discrete
// maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2, ...
inline int ContToDiscTheta(double theta, int numAngles)
{
    double delta = 2 * M_PI / numAngles;
    return (int)(smpl::normalize_angle_positive(theta + delta / 2) / delta);
}

inline int ContToDiscTheta(double theta)
{
    return ContToDiscTheta(theta, DEFAULT_NUM_ANGLES);
}

inline double DiscToContTheta(int theta, int numAngles)
{
    double delta = 2 * M_PI / numAngles;
    return theta * delta;
}

inline double DiscToContTheta(int theta)
{
    return DiscToContTheta(theta, DEFAULT_NUM_ANGLES);
}

// Returns true if x lies in the range (L, U) where x, U, L are all
// angles in the range [0, 2*pi]
inline bool angle_lies_between(double x, double L, double U)
{
    if (L < U) {
        return (x > L && x < U);
    } else if (L > U) {
        return (x > L || x < U);
    } else {
        return false;
    }
}

} // namespace angles
