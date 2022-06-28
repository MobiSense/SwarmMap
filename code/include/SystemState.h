//
// Created by Halcao on 2020/12/23.
//

#ifndef EDGE_SLAM_SYSTEMSTATE_H
#define EDGE_SLAM_SYSTEMSTATE_H

#include <opencv2/core/core.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace ORB_SLAM2 {

enum TrackingState : int;

struct SystemState {
    cv::Mat location;
    bool bVelocityBurst;
    bool bStable;
    uint8_t nTracked;
    size_t lostCount;

//    friend class boost::serialization::access;
//    template<class Archive>
//    void serialize(Archive & ar, __attribute__((unused)) const unsigned int version) {
//        ar & location;
//        ar & bVelocityBurst;
//        ar & bStable;
//        ar & nTracked;
//        ar & lostCount;
//    }

//    std::string toString() const {
//        std::stringstream out;
//        boost::archive::text_oarchive oa(out);
////        boost::archive::binary_oarchive oa(out);
//        // start to serialize
//        try {
//            oa << this;
//        } catch (boost::archive::archive_exception &e) {
//        }
//
//        return out.str();
//    }
};

}
#endif //EDGE_SLAM_SYSTEMSTATE_H
