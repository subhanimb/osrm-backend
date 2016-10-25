#include "util/geojson_debug_policies.hpp"
#include "util/geojson_debug_policy_toolkit.hpp"
#include "util/coordinate.hpp"

#include <algorithm>

namespace osrm
{
namespace util
{

//----------------------------------------------------------------
NodeIdVectorToLineString::NodeIdVectorToLineString(
    const std::vector<extractor::QueryNode> &node_coordinates)
    : node_coordinates(node_coordinates)
{
}

// converts a vector of node ids into a linestring geojson feature
util::json::Object NodeIdVectorToLineString::operator()(const std::vector<NodeID> &node_ids) const
{
    util::json::Array coordinates;
    std::transform(node_ids.begin(),
                   node_ids.end(),
                   std::back_inserter(coordinates.values),
                   NodeIdToCoordinate(node_coordinates));

    return makeFeature("LineString", coordinates);
}

//----------------------------------------------------------------
NodeIdVectorToMultiPoint::NodeIdVectorToMultiPoint(
    const std::vector<extractor::QueryNode> &node_coordinates)
    : node_coordinates(node_coordinates)
{
}
util::json::Object NodeIdVectorToMultiPoint::operator()(const std::vector<NodeID> &node_ids) const
{
    util::json::Array coordinates;
    std::transform(node_ids.begin(),
                   node_ids.end(),
                   std::back_inserter(coordinates.values),
                   NodeIdToCoordinate(node_coordinates));

    return makeFeature("MultiPoint", coordinates);
}

//----------------------------------------------------------------
util::json::Object CoordinateVectorToMultiPoint::
operator()(const std::vector<util::Coordinate> &input_coordinates) const
{
    const auto coordinates = makeJsonArray(input_coordinates);
    return makeFeature("MultiPoint", coordinates);
}

//----------------------------------------------------------------
util::json::Object CoordinateVectorToLineString::
operator()(const std::vector<util::Coordinate> &input_coordinates) const
{
    const auto coordinates = makeJsonArray(input_coordinates);
    return makeFeature("LineString", coordinates);
}

} /* namespace util */
} /* namespace osrm */
