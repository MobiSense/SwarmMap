//
// Created by Halcao on 2020/6/19.
//

#ifndef EDGE_SLAM_MAPMANAGER_HPP
#define EDGE_SLAM_MAPMANAGER_HPP

#include <vector>
#include <map>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2 {
using std::vector;
using std::map;

class MapManager {
public:
    vector<Map *> GetAllMaps();
    static void MergeMap(Map *pMap1, Map *pMap2, const g2o::Sim3 &T12);
    static void Fuse(Map *pMap, const vector<MapPoint *>& otherMapPoints);
    static void Fuse(Map *pMap1, Map *pMap2);
    static void Register(Map *pMap);
    static bool IsInSameGroup(Map *pMap1, Map *pMap2);
    static void MoveToMapGroup(Map *src, Map *dst);
    static void SaveGlobalMap(const string& filename);
    static vector<Map *> GetGroup(Map *pMap);
private:
    // group base map Id -> map group
    static map<unsigned long, vector<Map *> > mapGroups;
    // global base map id
    static unsigned long baseMapId;
    static map<unsigned long, Map *> mapDict;
    // map id -> group id
    static void KeyFrameCulling(KeyFrame *&mpCurrentKeyFrame, vector<KeyFrame *> &retainedKeyframes);
};
}

#endif //EDGE_SLAM_MAPMANAGER_HPP
