/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>
#include "MapElementUpdate.h"
#include "AgentMediator.h"
//#include "System.h"
#include <unordered_map>
#include <Thirdparty/g2o/g2o/types/sim3.h>

//namespace g2o {
//    struct Sim3 {};
//}

namespace ORB_SLAM2
{

const unsigned long MAP_BASE = 1000000;

class MapPoint;
class KeyFrame;
class MapUpdater;
class LoopClosing;
class MapSlice;
class Mapit;
class AgentMediator;
class ConnectionService;
// TODO(halcao): remove this class
class System;

class Map
{
public:
    Map(bool isInMediator_=false);

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);

    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs, bool bAddUpdate=true);
    void AddOriginKeyFrame(KeyFrame *pKF, bool bAddUpdate=true);
    void InformNewBigChange(bool bAddUpdate=true);
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear(bool bAddUpdate=true);

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // TODO(ch): do it in the server
    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    // protected:
public:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;
    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

    std::set<unsigned long> restorationKFQueue;
    std::set<unsigned long> restorationMPQueue;
    map<unsigned long, KeyFrame *> allKFs;
    map<unsigned long, MapPoint *> allMPs;
    static map<unsigned long, Map *> allMaps;
    unsigned long mnId;
    unsigned long mnIdBase;
    vector<unsigned long> mvLoopClosingQueue;
    // reference to real loop closer object
    LoopClosing *mpLoopCloser;
    // Transform local to world
//    cv::Mat mTwl;
    g2o::Sim3 mTwl;
private:
    static unsigned long nNextId;

    // variables maintained by Halcao
    std::mutex mMutexElementUpdate;
//    vector<MapElementUpdateBase *> mapUpdates;
    void UpdateElementsGlobalPos();
    bool isInMediator = false;
    Mapit* pMapit;
    ConnectionService *pConnectionService;
public:
    unsigned long groupId = -1ul;
    // TODO(halcao): add getter and setter
    bool mbBaseMap = false;
    bool mbMerged = false;
    Map *mpBaseRef = nullptr;
    // map is base in the group
    bool IsBase();
    bool SetTransform(const g2o::Sim3 &newTwl);

    void RegisterKeyFrame(KeyFrame * pKF);
    void RegisterMapPoint(MapPoint * pMP);
    void DeregisterMapPoint(unsigned long id);
    void DeregisterKeyFrame(unsigned long id);
    map<unsigned long, KeyFrame *> GetKeyFrameMaps();
    map<unsigned long, MapPoint *> GetMapPointMaps();

    static Map* GetMap(unsigned long id);
    KeyFrame *GetKeyFrame(unsigned long id, bool findInLocal=false);
    MapPoint *GetMapPoint(unsigned long id, bool findInLocal=false);

    void AddUpdate(MapElementUpdateBase* update);

    void ArchiveMap(MapSlice &slice);
    void UpdateMap(const MapSlice &slice);
    Mapit* GetMapit() { return pMapit; };
    ConnectionService* GetConnectionService() { return pConnectionService; };
    bool TryConnect(AgentMediator *pMediator);
    bool TryConnect(const std::string &ip, int port);
    static unsigned long ClaimId();

    bool TryConnect(ORB_SLAM2::System *pSLAM);
};

} //namespace ORB_SLAM

#endif // MAP_H
