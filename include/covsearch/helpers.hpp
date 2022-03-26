// project includes
#include <covsearch/types.hpp>

// system includes

// standard includes
#include <sstream>
#include <cmath>

#pragma once

#define INTERIOR_CELLS_ARE_FRONTIER_CELLS false

namespace cs
{

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (X*YSIZE + Y)
#define CONTXY2DISC(X, CELLSIZE) (int)std::round((X / CELLSIZE))

// inline
// int CONTXY2DISC(const double& x, const double& cell_size)
// {
//     double x_idx = x / cell_size;
//     if (x_idx - std::floor(x_idx) == 0.5)
//     {
//         if ((int)std::floor(x_idx) % 2 == 0) // even
//         {
//             return std::floor(x_idx);
//         }
//         else
//         {
//             return std::floor(x_idx) + 1;
//         }
//     }
//     else if (x_idx - std::floor(x_idx) < 0.5)
//     {
//         return std::floor(x_idx);
//     }
//     else
//     {
//         return std::floor(x_idx) + 1;
//     }
// }

inline
void reset(std::stringstream& ss)
{
    ss.str("");
    ss.clear();
}

inline
auto split(std::string& s, char delim) -> std::vector<std::string>
{
    std::string item;
    std::stringstream ss(s);
    std::vector<std::string> elems;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


inline
bool isInBounds(int r, int c, int nrows, int ncols)
{
    return r >= 0 && c >= 0 && r < nrows && c < ncols;
}

inline
bool isFrontierCell(int r, int c, MAP_t map, int nrows, int ncols)
{
    /// if this cell is an obstacle (0) or marked covered (1),
    /// it is not a frontier cell
    auto idx_this = GETMAPINDEX(r, c, nrows, ncols);
    if (map[idx_this] == 1 || map[idx_this] == 0) {
        return false;
    }

    int num_uncovered_neighbors = 0;
    int num_obstacle_neighbors = 0;

    for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {

        if (dr == 0 && dc == 0) continue;
        int rtest = r+dr;
        int ctest = c+dc;
        auto idx = GETMAPINDEX(rtest, ctest, nrows, ncols);

        // An uncovered cell with at least one covered neighbor is a frontier cell
        if (isInBounds(rtest, ctest, nrows, ncols) & map[idx] == 1)
        {
            return true;
        }
        else if (isInBounds(rtest, ctest, nrows, ncols) & map[idx] == 2)
        {
            ++num_uncovered_neighbors;
        }
        else if (isInBounds(rtest, ctest, nrows, ncols) & map[idx] == 0)
        {
            ++num_obstacle_neighbors;
        }
    }
    }

    // if (num_uncovered_neighbors + num_obstacle_neighbors == 8) return true;
    // if (num_obstacle_neighbors == 8) return false;
    return false;
}

// /// If any cell adjacent to (r,c) is uncovered, (r,c) is a frontier cell.
// inline
// bool isFrontierCell(int r, int c, MAP_t map, int nrows, int ncols)
// {
//     /// if this cell is an obstacle (0) or marked covered (1),
//     /// it is not a frontier cell
//     auto idx_this = GETMAPINDEX(r, c, nrows, ncols);
//     if (map[idx_this] == 1 || map[idx_this] == 0) {
//         return false;
//     }

//     int count_uncovered = 0;

//     bool at_least_one_covered_neighbor = false;
//     bool at_least_one_uncovered_neighbor = false;

//     int num_covered_neighbors = 0;
//     int num_obstacle_neighbors = 0;

//     for (int dr = -1; dr <= 1; ++dr) {
//     for (int dc = -1; dc <= 1; ++dc) {
//         if (dr == 0 && dc == 0) continue;
//         int rtest = r+dr;
//         int ctest = c+dc;
//         auto idx = GETMAPINDEX(rtest, ctest, nrows, ncols);
//         if (isInBounds(rtest, ctest, nrows, ncols) & map[idx] == 2)
//         {
//             at_least_one_uncovered_neighbor = true;
//             if (at_least_one_covered_neighbor) {
//                 return true;
//             }
//         }
//         else if (isInBounds(rtest, ctest, nrows, ncols) & map[idx] == 1)
//         {
//             at_least_one_covered_neighbor = true;
//             ++num_covered_neighbors;
//             if (at_least_one_uncovered_neighbor) {
//                 return true;
//             }
//         }
//         else if (isInBounds(rtest, ctest, nrows, ncols) & map[idx] == 0)
//         {
//             ++num_obstacle_neighbors;
//         }
//     }
//     }

//     if (num_covered_neighbors + num_obstacle_neighbors == 8) {
//         return true;
//     } else {
//         return false;
//     }
// }

inline
void SensorFootprintAbsolute(
    double& inX, double& inY, CoveredCellsLookup& outCells)
{
    bool b = false;
    auto cell = XYCell{(int)std::round(inX), (int)std::round(inY)};
    if (outCells.find(cell) == outCells.end()) outCells.insert(cell);

    if (abs(inX - std::floor(inX)) == 0.5)
    {
        auto cell = XYCell{(int)std::round(inX)-1, (int)std::round(inY)};
        if (outCells.find(cell) == outCells.end()) outCells.insert(cell);
        b = true;
    }

    if (abs(inY - std::floor(inY)) == 0.5)
    {
        auto cell = XYCell{(int)std::round(inX), (int)std::round(inY)-1};
        if (outCells.find(cell) == outCells.end()) outCells.insert(cell);
        if (b)
        {
            auto cell = XYCell{(int)std::round(inX)-1, (int)std::round(inY)-1};
            if (outCells.find(cell) == outCells.end()) outCells.insert(cell);
        }
    }
    return;
}

}  // namespace cs
