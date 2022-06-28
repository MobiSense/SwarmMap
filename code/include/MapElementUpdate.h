#include <utility>
#include <string>
#include <boost/serialization/access.hpp>

//
// Created by Halcao on 2020/4/18.
//

#ifndef EDGE_SLAM_KEYFRAMEUPDATE_H
#define EDGE_SLAM_KEYFRAMEUPDATE_H
namespace ORB_SLAM2 {

    enum MapElementUpdateType {
        MapPointType,
        KeyFrameType,
        MapEventType
    };

    class MapElementUpdateBase {
    public:
        static unsigned long curId;
        unsigned long id = -1ul;
        unsigned long mnId = -1ul;
        // keyframe function name
        std::string funcName;
//        MapElementUpdateType type;

        MapElementUpdateBase() = default;
        MapElementUpdateBase(unsigned long _mnId, std::string _func) : id(++curId), mnId(_mnId),
                                                                       funcName(std::move(_func)) {};

        virtual MapElementUpdateType getType() { return MapEventType; };
        virtual ~MapElementUpdateBase() = default;
    };

    template<typename T>
    class KeyFrameUpdate : public MapElementUpdateBase {

    public:
        KeyFrameUpdate() = default;
        KeyFrameUpdate(unsigned long _mnId, std::string _func, T _arg) : MapElementUpdateBase(_mnId, _func),
                                                                         arg(_arg) {};
        T arg;

        MapElementUpdateType getType() override {
            return MapElementUpdateType::KeyFrameType;
        }

        ~KeyFrameUpdate() override = default;
    };

    template<typename T>
    class MapPointUpdate : public MapElementUpdateBase {

    public:
        MapPointUpdate() = default;
        MapPointUpdate(unsigned long _mnId, std::string _func, T _arg) : MapElementUpdateBase(_mnId, _func),
                                                                         arg(_arg) {};
        T arg;

        MapElementUpdateType getType() override {
            return MapElementUpdateType::MapPointType;
        }

        ~MapPointUpdate() override = default;
    };

    template<typename T>
    class MapEventUpdate : public MapElementUpdateBase {

    public:
        MapEventUpdate() = default;
        MapEventUpdate(unsigned long _mnId, std::string _func, T _arg) : MapElementUpdateBase(_mnId, _func),
                                                                         arg(_arg) {};
        T arg;

        MapElementUpdateType getType() override {
            return MapElementUpdateType::MapEventType;
        }

        ~MapEventUpdate() override = default;
    };
}

#endif //EDGE_SLAM_KEYFRAMEUPDATE_H
