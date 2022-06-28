//
// Created by Halcao on 2020/4/20.
//

#ifndef EDGE_SLAM_MAPUPDATER_H
#define EDGE_SLAM_MAPUPDATER_H

#include "Map.h"
#include "MapElementUpdate.h"
#include "KeyFrameDatabase.h"
#include "LoopClosing.h"
#include <functional>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

namespace ORB_SLAM2 {
/**
 * Steps:
 * 1. Structure for MapUpdater
 * 2. Serialization: KeyFrameUpdate
 * 3. Apply Update
 */

    using KeyFrameUpdateHandler = std::function<void(KeyFrame *, MapElementUpdateBase *)>;
    using MapPointUpdateHandler = std::function<void(MapPoint *, MapElementUpdateBase *)>;
    using MapEventUpdateHandler = std::function<void(Map *, MapElementUpdateBase *)>;
    class KeyFrame;
    class MapSlice;

    class MapUpdater {
    public:
        MapUpdater() = delete;

        static vector<unsigned int> Apply(Map *pMap, vector<MapElementUpdateBase *> &updates);

        template <class Archive>
        static void RegisterType(Archive &ar);

        static void Serialize(const MapSlice &slice, string &result);
        static void Deserialize(MapSlice &slice, const string &result);

    private:
        static map<string, KeyFrameUpdateHandler> kfHandlerMap;
        // helps to initialize the handler map
        static map<string, KeyFrameUpdateHandler> initKeyFrameHandler();

        static map<string, MapPointUpdateHandler> mpHandlerMap;
        static map<string, MapPointUpdateHandler> initMapPointHandler();

        static map<string, MapEventUpdateHandler> mapHandlerMap;
        static map<string, MapEventUpdateHandler> initMapHandler();

    };
}

#endif //EDGE_SLAM_MAPUPDATER_H
