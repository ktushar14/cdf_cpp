#include <covsearch/types.hpp>

#include <boost/functional/hash.hpp>

auto std::hash<cs::StateEntry>::operator()(const argument_type& s) const
    -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

auto std::hash<cs::XYCell>::operator()(const argument_type& s) const
    -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
}
