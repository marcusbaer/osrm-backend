#include "util/geojson_debug_policies.hpp"
#include "util/coordinate.hpp"

namespace osrm
{
namespace util
{

NodeIdVectorToLineString::NodeIdVectorToLineString(
    const std::vector<extractor::QueryNode> &node_coordinates)
    : node_coordinates(node_coordinates)
{
}

// converts a vector of node ids into a linestring geojson feature
util::json::Object NodeIdVectorToLineString::operator()(const std::vector<NodeID> &node_ids) const
{
    util::json::Object result;
    result.values["type"] = "Feature";
    util::json::Object geometry;
    geometry.values["type"] = "Linestring";
    util::json::Array coordinates;

    const auto node_id_to_coordinate = [this](const NodeID nid) -> util::json::Array {
        auto coordinate = node_coordinates[nid];
        util::json::Array json_coordinate;
        json_coordinate.values.push_back(static_cast<double>(toFloating(coordinate.lon)));
        json_coordinate.values.push_back(static_cast<double>(toFloating(coordinate.lat)));
        return json_coordinate;
    };

    std::transform(node_ids.begin(),
                   node_ids.end(),
                   std::back_inserter(coordinates.values),
                   node_id_to_coordinate);
    geometry.values["coordinates"] = coordinates;
    result.values["geometry"] = geometry;
    return result;
}

} /* namespace util */
} /* namespace osrm */
