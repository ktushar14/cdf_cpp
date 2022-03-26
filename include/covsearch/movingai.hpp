// project includes
#include "covsearch/types.hpp"

// system includes

// standard includes
#include <random>

#pragma once

namespace cs
{

class MovingAI
{
public:
    MovingAI(const std::string& fname);
    ~MovingAI();

    void GetRandomState(int& d1, int& d2);

    // void SavePath(
    //     const std::vector<MapState>& solpath,
    //     int iter=-1);
    // void SaveExpansions(
    //     int iter, double w1, double w2,
    //     const EXPANDS_t& expansions);

    bool IsValid(const int& dim1, const int& dim2) const;
    bool IsTraversible(const int& dim1, const int& dim2) const;

    int CellType(const int& dim1, const int& dim2) const;
    int CellType(const int& dim1, const int& dim2, char& c) const;

    void readFile();

    std::string m_fname;
    MAP_t m_map;
    int m_h, m_w;

    std::random_device m_dev;
    std::mt19937 m_rng;
    std::uniform_real_distribution<double> m_distD;

};

}  // namespace cs
