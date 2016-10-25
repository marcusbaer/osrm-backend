#ifndef OSRM_GEOJSON_DEBUG_POLICIES
#define OSRM_GEOJSON_DEBUG_POLICIES

#include <vector>

#include "extractor/query_node.hpp"
#include "util/json_container.hpp"
#include "util/typedefs.hpp"

namespace osrm
{
namespace util
{

struct NodeIdVectorToLineString
{
    NodeIdVectorToLineString(const std::vector<extractor::QueryNode> &node_coordinates);

    // converts a vector of node ids into a linestring geojson feature
    util::json::Object operator()(const std::vector<NodeID> &node_ids) const;

    const std::vector<extractor::QueryNode> &node_coordinates;
};

} /* namespace util */
} /* namespace osrm */

#endif /* OSRM_GEOJSON_DEBUG_POLICIES */
