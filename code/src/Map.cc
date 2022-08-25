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

//#include "MapUpdater.h"
#include <mutex>
#include "Map.h"
#include "Mapit.h"
#include "BoostArchiver.h"
#include "Timer.h"
#include <LoopClosing.h>
#include <MapUpdater.h>
#include <MapSlice.h>
#include <LandmarkScoring.h>
#include <MediatorScheduler.h>
//#include <boost/archive/binary_oarchive.hpp>
//#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "ConnectionService.h"
#include <CLogger.h>
// TODO(halcao): Remove this include.
//#include "System.h"

namespace ORB_SLAM2 {

    map<unsigned long, Map *> Map::allMaps;
    unsigned long Map::nNextId = 0;

    Map::Map(bool isInMediator_) : mnMaxKFid(0), mnBigChangeIdx(0), isInMediator(isInMediator_) {
        mnId = nNextId++;

        mnIdBase = mnId * MAP_BASE;

        allMaps[mnId] = this;
        KeyFrame::nNextIds[mnId] = 0;

        pMapit = new Mapit(this);
    }

    bool Map::TryConnect(AgentMediator *pMediator) {
        pConnectionService = new ConnectionService(pMediator);
    }

    bool Map::TryConnect(System *pSLAM) {
        pConnectionService = new ConnectionService(pSLAM);
    }


bool Map::TryConnect(const std::string &ip, int port) {
        return false;
    }

    void Map::AddKeyFrame(KeyFrame *pKF) {
        {
            unique_lock<mutex> lock(mMutexMap);
            if (mspKeyFrames.empty()) pKF->SetFirst(true);

            mspKeyFrames.insert(pKF);
            if (pKF->mnId > mnMaxKFid) {
                mnMaxKFid = pKF->mnId;
            }
        }

        RegisterKeyFrame(pKF);
    }

    void Map::AddMapPoint(MapPoint *pMP) {
        {
            unique_lock<mutex> lock(mMutexMap);
            mspMapPoints.insert(pMP);
        }

        RegisterMapPoint(pMP);
    }

    void Map::EraseMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }


    void Map::EraseKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        debug("EraseKeyFrame id {}", pKF->mnId);
        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs, bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange(bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;

        if (bAddUpdate) {
            pMapit->Add(new MapEventUpdate<int>(mnId, __func__, 0));
        }
    }

    int Map::GetLastBigChangeIdx() {
        unique_lock<mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    void Map::AddOriginKeyFrame(KeyFrame *pKF, bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexMap);

        mvpKeyFrameOrigins.push_back(pKF);
        if (bAddUpdate) {
            auto id = pKF->mnId;
            pMapit->Add(new MapEventUpdate<unsigned long>(mnId, __func__, id));
        }
    }

    vector<KeyFrame *> Map::GetAllKeyFrames() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

    vector<MapPoint *> Map::GetAllMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

    long unsigned int Map::MapPointsInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    vector<MapPoint *> Map::GetReferenceMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetMaxKFid() {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void Map::clear(bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexMap);

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();

        restorationKFQueue.clear();
        restorationMPQueue.clear();

        allMPs.clear();
        allKFs.clear();

        // clear updates
        pMapit->Clear(bAddUpdate);
    }

    void Map::RegisterKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        if (pKF->mnId > mnMaxKFid) {
            mnMaxKFid = pKF->mnId;
        }

        allKFs[pKF->mnId] = pKF;
    }

    void Map::DeregisterKeyFrame(unsigned long id) {
        unique_lock<mutex> lock(mMutexMap);
        if (allKFs.count(id)) {
            allKFs.erase(id);
        }
    }

    map<unsigned long, KeyFrame *> Map::GetKeyFrameMaps() {
        unique_lock<mutex> lock(mMutexMap);

        return allKFs;
    }

    void Map::AddUpdate(MapElementUpdateBase *update) {
        pMapit->Add(update);
    }


    KeyFrame *Map::GetKeyFrame(unsigned long id, bool findInLocal) {
        // if the map is in the mediator, and is not specified to find in the local map,
        // we will perform a global search between the peer mediators
        if (isInMediator && !findInLocal) {
            return MediatorScheduler::GetInstance().GetKeyFrame(id);
        }

        unique_lock<mutex> lock(mMutexMap);

        if (allKFs.count(id) == 0 || id == -1ul) return nullptr;

        return allKFs[id];
    }

    MapPoint *Map::GetMapPoint(unsigned long id, bool findInLocal) {
        // if the map is in the mediator, and is not specified to find in the local map,
        // we will perform a global search between the peer mediators
        if (isInMediator && !findInLocal) {
            return MediatorScheduler::GetInstance().GetMapPoint(id);
        }

        unique_lock<mutex> lock(mMutexMap);

        if (allMPs.count(id) == 0 || id == -1ul) return nullptr;

        return allMPs[id];
    }

    void Map::RegisterMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);

        allMPs[pMP->mnId] = pMP;
    }

    void Map::DeregisterMapPoint(unsigned long id) {
        unique_lock<mutex> lock(mMutexMap);
        if (allMPs.count(id)) allMPs.erase(id);
    }

    map<unsigned long, MapPoint *> Map::GetMapPointMaps() {
        unique_lock<mutex> lock(mMutexMap);

        return allMPs;
    }

    Map *Map::GetMap(unsigned long id) {
        return allMaps[id];
    }

    void PrintUpdatesStat(vector<MapElementUpdateBase *> &updates) {
        map<string, int> count;
        for (auto update: updates) {
            const string type = update->getType() == MapPointType ? "MapPoint" :
                                update->getType() == KeyFrameType ? "KeyFrame" : "Map";
            const auto key = type + string(":") + update->funcName;

            count[key] += 1;
        }

        // Declare vector of pairs
        vector<pair<string, int> > A;

        // Copy key-value pair from Map
        // to vector of pairs
        for (auto& it : count) {
            A.emplace_back(it);
        }

        // Sort using comparator function
        sort(A.begin(), A.end(), [](const pair<string, int>& a, const pair<string, int>& b){
            return a.second < b.second;
        });

        // Print the sorted value
        for (auto& it : A) {

            cout << it.first << ' '
                 << it.second << endl;
        }
    }

    void Map::ArchiveMap(MapSlice &slice) {
        unique_lock<mutex> lock(mMutexMap);

        // get all keyframes and mappoints
        vector<KeyFrame *> KFs;
        vector<MapPoint *> MPs;

        string log;
        string badLog;
        int nKF = 0;
        // filter out already serialized
        for (auto itr: allKFs) {
            if (itr.second && itr.second->mbToBeSerialized) {
                log += std::to_string(itr.second->mnId) + ", ";
//                itr.second->ComputeBoW();
                itr.second->SetupSerializationVariable();
                KFs.push_back(itr.second);
                ++nKF;
            }
        }
        debug("ready to send {} keyframes: {}", nKF, log);

        for (auto itr: allMPs) {
            if (itr.second && itr.second->mbToBeSerialized) {
                itr.second->SetupSerializationVariable();
                MPs.push_back(itr.second);
            }
        }
        debug("ready to send {} mappoints", MPs.size());

        auto updates = pMapit->DequeueUpdates();
        // compress extra data, compress multiple items into one

        slice.KFs = KFs;
        slice.MPs = MPs;
        slice.updates = updates;
        Mapit::Aggregate(allKFs, allMPs, slice.updates);

//        PrintUpdatesStat(slice.updates);


//        debug("current updates: {}", updates.size());
    }

    void Map::UpdateMap(const MapSlice &slice) {
        auto MPs = slice.MPs;
        auto KFs = slice.KFs;
        auto updates = slice.updates;


        // TODO(halcao): refactor it
//        // check clear event in update
        if (!updates.empty()
            && updates.front()->getType() == MapEventType
            && updates.front()->funcName == "clear") {
            this->clear(false);
            mpLoopCloser->mpKeyFrameDB->clear();

            KeyFrame::nNextIds[this->mnId] = 0;

            this->allKFs.clear();
            this->allMPs.clear();

            updates.erase(updates.begin());
            info("Map {} cleared", this->mnId);
        }

        /** KeyFrame Restore
         * 1. attach mpKeyFrameDB
         * 2. attach mpORBvocabulary
         * 3. call keyframe restore
         */
        debug("ready to add {} keyframes", KFs.size());
        // add new kf to the map
        string log;
        for (auto &itr: KFs) {
            if (!itr) continue;

            log += std::to_string(itr->mnId) + ", ";
            itr->mpKeyFrameDB = mpLoopCloser->mpKeyFrameDB;
            itr->ComputeBoW();
            itr->mpMap = this;
            if (itr->isBad()) {
                this->RegisterKeyFrame(itr);
            } else {
                this->AddKeyFrame(itr);
            }
            itr->mbToBeSerialized = false;
        }
        debug("{}", log);

        for (auto &itr: MPs) {
            if (!itr) continue;

            itr->mpMap = this;
            if (itr->isBad()) {
                this->RegisterMapPoint(itr);
            } else {
                this->AddMapPoint(itr);
            }

            itr->mbToBeSerialized = false;
        }

        // restore previous mappoint with restoration failure
        auto mpQueue = this->restorationMPQueue;
        // clear before restoration, because the process may add new id to the queue
        this->restorationMPQueue.clear();
        for (auto id: mpQueue) {
            auto mp = this->GetMapPoint(id);
            if (mp) {
                mp->RestoreSerialization();
                mp->UpdateGlobalPos(true);
            }
        }

        // restore previous keyframe with restoration failure
        auto kfQueue = this->restorationKFQueue;
        // clear before restoration, because the process may add new id to the queue
        this->restorationKFQueue.clear();
        for (auto id: kfQueue) {
            auto kf = this->GetKeyFrame(id);
            if (kf) {
                KFs.push_back(kf);
                debug("Restore kf {} again", kf->mnId);
            }
        }

        // After adding all MPs and KFs to the database, we start to reconstruct the reference
        for (auto &itr: MPs) {
            if (!itr) continue;

            itr->RestoreSerialization();
            itr->UpdateGlobalPos(true);
        }

        for (auto &itr: KFs) {
            if (!itr) continue;

            itr->RestoreSerialization();
            itr->UpdateGlobalPose(true);
        }

        info("[Map loaded]: map {} add {} keyframes", this->mnId, KFs.size());
        info("[Map loaded]: map {} add {} mappoints", this->mnId, MPs.size());

        if (updates.size() > 0) {
            auto updateResult = MapUpdater::Apply(this, updates);
            info("[Map loaded]: apply {} updates", updateResult.size());
        }
    }


    g2o::Sim3 interpolate(const g2o::Sim3 &origin, const g2o::Sim3 &other) {
        const double ratio = 0.9;

        auto r = origin.rotation().slerp(ratio, other.rotation());
        auto t = ratio * origin.translation() + (1 - ratio) * other.translation();
        auto s = ratio * origin.scale() + (1 - ratio) * other.scale();

        return g2o::Sim3(r, t, s);
    }

    bool Map::SetTransform(const g2o::Sim3 &newTwl) {
        // prevent large scale change
        bool isAccepted = false;
        if (!mbMerged) {
            mTwl = newTwl;
            // mark the maps as merged
            mbMerged = true;
            isAccepted = true;
        } else {
            auto ratio = newTwl.scale() / mTwl.scale();
            if (ratio > 0.8 && ratio < 1.2) {
                mTwl = interpolate(mTwl, newTwl);
                info("new {} / old {} = ratio {} is accepted", newTwl.scale(), mTwl.scale(), ratio);
                isAccepted = true;
            } else {
                info("new {} / old {} = ratio {} is rejected", newTwl.scale(), mTwl.scale(), ratio);
                isAccepted = false;
            }
        }

        if (isAccepted) {
            // if successfully set, update global keyframe pose and mappoint pos
            UpdateElementsGlobalPos();
        }

        return isAccepted;
    }

    bool Map::IsBase() {
        if (!mpBaseRef) return false;
        return mpBaseRef->mnId == this->mnId;
    }

    void Map::UpdateElementsGlobalPos() {
        unique_lock<mutex> lock(mMutexMapUpdate);

        // Update map elements pos and pose
        for (auto &itr: allKFs) {
            if (!itr.second) continue;
            itr.second->UpdateGlobalPose();
        }
        for (auto &itr: allMPs) {
            if (!itr.second) continue;
            itr.second->UpdateGlobalPos();
        }
    }

unsigned long Map::ClaimId() {
    unsigned long newId = nNextId++;

    return newId;
}

} //namespace ORB_SLAM
