//
// Created by Halcao on 2022/2/24.
//

#ifndef EDGE_SLAM_MAPIT_H
#define EDGE_SLAM_MAPIT_H

#include <vector>
#include <map>
#include <mutex>
#include "MapSlice.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace ORB_SLAM2 {

class MapPoint;
class KeyFrame;
class Map;
class MapElementUpdateBase;

using std::map;
using std::vector;
using std::mutex;

class Mapit {
public:
    explicit Mapit(Map *_pMap);

    // add map element update item
    void Add(MapElementUpdateBase *update);

    std::vector<MapElementUpdateBase *> DequeueUpdates();


    // delete redundant updates and compress it
    static void Aggregate(map<unsigned long, KeyFrame *> &allKFs,
                                map<unsigned long, MapPoint *> &allMPs,
                                vector<MapElementUpdateBase *> &updates);

    void Push(std::string &result);

    static void Merge(Map *pMap1, Map *pMap2, const g2o::Sim3 &T12);

    // clear map updates
    void Clear(bool bAddUpdate=false);

    void ReceivePush(const MapSlice &slice);

private:
    Map *pMap;
    vector<MapElementUpdateBase *> mapUpdates;
    mutex mMutexElementUpdate;

    void Pull();

    void ReplyPull();

    void ReplyPull(MapSlice &slice);
};

}
#endif //EDGE_SLAM_MAPIT_H
