#ifndef OSRM_EXTRACTOR_GUIDANCE_INTERSECTION_HPP_
#define OSRM_EXTRACTOR_GUIDANCE_INTERSECTION_HPP_

#include <string>
#include <utility>
#include <vector>

#include "util/guidance/toolkit.hpp"
#include "extractor/guidance/turn_instruction.hpp"
#include "util/guidance/toolkit.hpp"
#include "util/typedefs.hpp" // EdgeID

namespace osrm
{
namespace extractor
{
namespace guidance
{

// Every Turn Operation describes a way of switching onto a segment, indicated by an EdgeID. The
// associated turn is described by an angle and an instruction that is used to announce it.
// The Turn Operation indicates what is exposed to the outside of the turn analysis.
struct TurnOperation final
{
    EdgeID eid;
    double angle;
    double bearing;
    TurnInstruction instruction;
    LaneDataID lane_data_id;
};

// A Connected Road is the internal representation of a potential turn. Internally, we require
// full list of all connected roads to determine the outcome.
// The reasoning behind is that even invalid turns can influence the perceived angles, or even
// instructions themselves. An pososible example can be described like this:
//
// aaa(2)aa
//          a - bbbbb
// aaa(1)aa
//
// will not be perceived as a turn from (1) -> b, and as a U-turn from (1) -> (2).
// In addition, they can influence whether a turn is obvious or not. b->(2) would also be no
// turn-operation,
// but rather a name change.
//
// If this were a normal intersection with
//
// cccccccc
//            o  bbbbb
// aaaaaaaa
//
// We would perceive a->c as a sharp turn, a->b as a slight turn, and b->c as a slight turn.
struct ConnectedRoad final
{
    ConnectedRoad(const TurnOperation turn, const bool entry_allowed = false);

    // a turn may be relevant to good instructions, even if we cannot enter the road
    bool entry_allowed;
    TurnOperation turn;
};

// small helper function to print the content of a connected road
std::string toString(const ConnectedRoad &road);

struct Intersection
{
    typedef std::vector<ConnectedRoad>::const_iterator const_iterator;
    typedef std::vector<ConnectedRoad>::iterator iterator;

    inline iterator begin() { return connected_roads.begin(); }
    inline iterator end() { return connected_roads.end(); }

    inline const_iterator begin() const { return connected_roads.begin(); }
    inline const_iterator end() const { return connected_roads.end(); }

    inline const_iterator cbegin() const { return connected_roads.cbegin(); }
    inline const_iterator cend() const { return connected_roads.cend(); }

    inline const ConnectedRoad &operator[](std::size_t index) const
    {
        return connected_roads[index];
    }

    inline ConnectedRoad &operator[](std::size_t index) { return connected_roads[index]; }

    inline ConnectedRoad &front() { return connected_roads.front(); }
    inline ConnectedRoad &back() { return connected_roads.back(); }

    inline void pop_back() { connected_roads.pop_back(); }

    inline iterator erase(iterator position) { return connected_roads.erase(position); }
    inline iterator erase(iterator position, iterator end)
    {
        return connected_roads.erase(position, end);
    }

    inline void clear() { connected_roads.clear(); }
    inline bool empty() const { return connected_roads.empty(); }
    inline iterator insert(iterator position, const ConnectedRoad &road)
    {
        return connected_roads.insert(position, road);
    }

    std::size_t size() const { return connected_roads.size(); };
    inline void push_back(const ConnectedRoad &road) { connected_roads.push_back(road); }

    ConnectedRoad &at(std::size_t index) { return connected_roads.at(index); }
    const ConnectedRoad &at(std::size_t index) const { return connected_roads.at(index); }

    inline iterator findClosestTurn(double angle)
    {
        return std::min_element(connected_roads.begin(),
                                connected_roads.end(),
                                [angle](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
                                    return util::guidance::angularDeviation(lhs.turn.angle, angle) <
                                           util::guidance::angularDeviation(rhs.turn.angle, angle);
                                });
    }

    inline const_iterator findClosestTurn(double angle) const
    {
        return std::min_element(connected_roads.begin(),
                                connected_roads.end(),
                                [angle](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
                                    return util::guidance::angularDeviation(lhs.turn.angle, angle) <
                                           util::guidance::angularDeviation(rhs.turn.angle, angle);
                                });
    }

    std::vector<ConnectedRoad> connected_roads;
};

Intersection::const_iterator findClosestTurn(const Intersection &intersection, const double angle);
Intersection::iterator findClosestTurn(Intersection &intersection, const double angle);

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif /*OSRM_EXTRACTOR_GUIDANCE_INTERSECTION_HPP_*/
