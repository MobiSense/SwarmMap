//
// Created by Halcao on 2020/7/2.
//

#ifndef EDGE_SLAM_MAPSLICE_HPP
#define EDGE_SLAM_MAPSLICE_HPP

#include <vector>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace ORB_SLAM2 {
class KeyFrame;
class MapPoint;
class MapElementUpdateBase;

class MapSlice {
public:
    MapSlice(const std::vector<KeyFrame *> &keyframes, const std::vector<MapPoint *> &mappoints,
             const std::vector<MapElementUpdateBase *> &updates);
    MapSlice() = default;

    std::vector<KeyFrame *> KFs;
    std::vector<MapPoint *> MPs;
    std::vector<MapElementUpdateBase *> updates;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & KFs;
        ar & MPs;
        ar & updates;
    }
};
}

#endif //EDGE_SLAM_MAPSLICE_HPP
