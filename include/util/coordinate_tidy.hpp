#ifndef COORDINATE_TIDY
#define COORDINATE_TIDY

#include <algorithm>
#include <cstdint>
#include <iterator>

#include "engine/api/match_parameters.hpp"
#include "util/coordinate_calculation.hpp"

#include <boost/assert.hpp>

namespace osrm
{
namespace util
{
namespace tidy
{

using Coordinates = decltype(engine::api::MatchParameters::coordinates);
using Timestamps = decltype(engine::api::MatchParameters::timestamps);
using Mapping = std::vector<std::size_t>;

struct Thresholds
{
    double distance_in_meters;
    std::int32_t duration_in_seconds;
};

struct Result
{
    Coordinates tidied_coordinates;
    Timestamps tidied_timestamps;
    Mapping original_to_tidied;
};

inline Result tidy(const Coordinates &coords, const Timestamps &tss, Thresholds cfg = {15., 5})
{
    BOOST_ASSERT(coords.size() == tss.size());

    if (coords.empty() || tss.empty())
        return Result{{}, {}, {}};

    Result result;

    // Estimated tidying ratio
    result.tidied_coordinates.reserve(coords.size() * .75);
    result.tidied_timestamps.reserve(tss.size() * .75);
    result.original_to_tidied.reserve(coords.size());

    // Otherwise more than one items to tidy, first is always good
    result.tidied_coordinates.push_back(coords.front());
    result.tidied_timestamps.push_back(tss.front());
    result.original_to_tidied.push_back(0);

    auto starts = std::make_pair(begin(coords), begin(tss));
    auto ends = std::make_pair(end(coords), end(tss));

    Thresholds running{0., 0};

    // Walk over adjacent (coord, ts)-pairs, with rhs being the candidate to discard or keep
    while (starts.first != ends.first - 1 && starts.second != ends.second - 1)
    {
        auto lhs_coord = *starts.first;
        auto lhs_ts = *starts.second;

        auto rhs_coord = *(starts.first + 1);
        auto rhs_ts = *(starts.second + 1);

        auto distance_delta = util::coordinate_calculation::haversineDistance(lhs_coord, rhs_coord);
        auto duration_delta = rhs_ts - lhs_ts;

        BOOST_ASSERT(distance_delta >= 0);
        BOOST_ASSERT(duration_delta >= 0);

        running.distance_in_meters += distance_delta;
        running.duration_in_seconds += duration_delta;

        if (running.distance_in_meters >= cfg.distance_in_meters &&
            running.duration_in_seconds >= cfg.duration_in_seconds)
        {
            result.tidied_coordinates.push_back(rhs_coord);
            result.tidied_timestamps.push_back(rhs_ts);
            running = {0., 0}; // reset running distance and time
        }

        auto at = result.tidied_coordinates.size() - 1;
        result.original_to_tidied.push_back(at);

        ++starts.first;
        ++starts.second;
    }

    BOOST_ASSERT(result.original_to_tidied.size() == coords.size());
    BOOST_ASSERT(result.original_to_tidied.size() == tss.size());
    BOOST_ASSERT(result.tidied_coordinates.size() == result.tidied_timestamps.size());

    BOOST_ASSERT(std::is_sorted(begin(result.original_to_tidied), end(result.original_to_tidied)));

    return result;
}

} // ns tidy
} // ns util
} // ns osrm

#endif
