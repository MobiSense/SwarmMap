//
// Created by Halcao on 2022/2/24.
//

#include "Mapit.h"
#include "Map.h"
#include "MapElementUpdate.h"
#include "MapUpdater.h"
#include "MediatorScheduler.h"
#include "CLogger.h"
#include "MapManager.h"

namespace ORB_SLAM2 {

Mapit::Mapit(Map *_pMap) : pMap(_pMap) {}

void Mapit::Add(MapElementUpdateBase *update) {
    unique_lock<mutex> lock(mMutexElementUpdate);

    // make sure only to send changes applied on old objects.
    switch (update->getType()) {
        case KeyFrameType: {

            auto kf = pMap->allKFs[update->mnId];
            if (kf->mbToBeSerialized) {
                // the keyframe is not serialized, the update is already applied on it
                return;
            }
            break;
        }
        case MapPointType: {
            auto mp = pMap->allMPs[update->mnId];
            if (!mp || mp->mbToBeSerialized) {
                if (!mp) {
                    warn("add update mappoint mnId {} null", update->mnId);
                }
                // the mappoint is not serialized, the update is already applied on it
                return;
            }
            break;
        }
        case MapEventType: {
            break;
        }
    }

    this->mapUpdates.push_back(update);
}

void Mapit::Aggregate(map<unsigned long, KeyFrame *> &allKFs, map<unsigned long, MapPoint *> &allMPs, vector<MapElementUpdateBase *> &updates) {
    // Only keep the latest SetWorldPos and SetPose
    // MapPoint mnId -> latest update id
    unordered_map<unsigned long, unsigned long> posMap;
    // KeyFrame mnId -> latest update id
    unordered_map<unsigned long, unsigned long> poseMap;
    // last tracked MapPoint id
    unordered_map<string, set<unsigned long>> duplicatedMap;

    const auto duplicatedKeys = set<string>({"SetLastTrackedTime", "IncreaseFound", "IncreaseVisible", "UpdateNormalAndDepth", "ComputeDistinctiveDescriptors"});

    string badPoints;
    string badKeyframes;
    for (auto &update: updates) {
        if (update->funcName == "SetWorldPos") {
            posMap[update->mnId] = update->id;
        } else if (update->funcName == "SetPose") {
            poseMap[update->mnId] = update->id;
        } else if (duplicatedKeys.find(update->funcName) != duplicatedKeys.end()) {
            // if funcName in duplicatedKeys
            duplicatedMap[update->funcName].insert(update->mnId);
        }
        if (update->funcName == "SetBadFlag") {
            if (update->getType() == MapPointType) {
                badPoints += to_string(update->mnId) + ", ";
            } else if (update->getType() == KeyFrameType) {
                badKeyframes += to_string(update->mnId) + ", ";
            }
        }
    }

    info("these points are going to be set bad {}", badPoints);
    info("these keyframes are going to be set bad {}", badKeyframes);

    size_t dupPosCount = 0;
    size_t dupPoseCount = 0;
    size_t dupIncreasedCount = 0;
    // remove SetWorldPos and SetPose with id **not** equaling to the latest one
    updates.erase(std::remove_if(updates.begin(), updates.end(), [&](MapElementUpdateBase *&update){
        if (update->funcName == "SetWorldPos" && update->id != posMap[update->mnId]) {
            dupPosCount++;
            return true;
        } else if (update->funcName == "SetPose" && update->id != poseMap[update->mnId]) {
            dupPoseCount++;
            return true;
        } else if (duplicatedKeys.find(update->funcName) != duplicatedKeys.end()) {
            dupIncreasedCount++;
            return true;
        }

        if (update->getType() == MapPointType) {
            auto mp = allMPs[update->mnId];

            if (update->funcName == "AddObservation" && mp) {
                auto arg = dynamic_cast<MapPointUpdate<std::pair<unsigned long, size_t> > *>(update)->arg;
                auto kf = allKFs[arg.first];
                if (!kf || kf->isBad()) {
                    debug("send update revolves bad elements");
                    return true;
                }
            }

            if (mp && mp->isBad()) {
                return true;
            }
        }

        return false;
    }), updates.end());

    info("removed {} dup pose updates and {} dup pos updates and {} dup increased updates", dupPoseCount, dupPosCount, dupIncreasedCount);

    // remove duplicated and add the latest one (overwritten)
    for (auto &map: duplicatedMap) {
        const auto key = map.first;
        const auto itemSet = map.second;
        for (auto &id: itemSet) {
            auto mp = allMPs[id];
            if (!mp || mp->isBad()) continue;

            if (key == "SetLastTrackedTime") {
                updates.push_back(new MapPointUpdate<double>(id, "SetLastTrackedTime", mp->GetLastTrackedTime()));
            } else if (key == "IncreaseFound") {
                // stackable
                updates.push_back(new MapPointUpdate<int>(id, "SetFound", mp->GetFound()));
            } else if (key == "IncreaseVisible") {
                // stackable
                updates.push_back(new MapPointUpdate<int>(id, "SetVisible", mp->GetVisible()));
            } else if (key == "UpdateNormalAndDepth" || key == "ComputeDistinctiveDescriptors") {
                updates.push_back(new MapPointUpdate<int>(mp->mnId, key, int(0)));
            }
        }
    }
}

// Sends logs
void Mapit::Push(string &result) {
    MapSlice slice;
    pMap->ArchiveMap(slice);
    MapUpdater::Serialize(slice, result);
}

// Receives logs and apply them
void Mapit::ReceivePush(const MapSlice &slice) {
    this->pMap->UpdateMap(slice);
}

// Merge the maps together
void Mapit::Merge(Map *pMap1, Map *pMap2, const g2o::Sim3 &T12) {
    MapManager::MergeMap(pMap1, pMap2, T12);
}


// Client request optimized map from client
void Mapit::Pull() {
    //
}

// Server sends logs and optimized map to client
void Mapit::ReplyPull(MapSlice &slice) {
    // get the latest 5 allKFs in the map
    auto allKFs = pMap->GetAllKeyFrames();

    vector<MapElementUpdateBase *> updates;

    vector<KeyFrame *> KFs;
    for (size_t i = allKFs.size(); i > 0; i--) {
        auto kf = allKFs[i - 1];
        if (kf->isBad()) continue;
        KFs.push_back(kf);
        if (KFs.size() == 5) break;
    }

    vector<MapPoint *> MPs;
    for (size_t i = 0; i < KFs.size(); i++) {
        auto kf = KFs[i];
        auto mps = kf->GetMapPointMatches();
        for (size_t j = 0; j < mps.size(); j++) {
            auto mp = mps[j];
            if (mp && mp->isBad()) continue;
            MPs.push_back(mp);
        }
    }

    slice.KFs = KFs;
    slice.MPs = MPs;
}

vector<MapElementUpdateBase *> Mapit::DequeueUpdates() {
    unique_lock<mutex> lock(mMutexElementUpdate);
    auto copy = vector<MapElementUpdateBase *>(this->mapUpdates.begin(), this->mapUpdates.end());
    this->mapUpdates.clear();
    return copy;
}

void Mapit::Clear(bool bAddUpdate) {
    unique_lock<mutex> lock(mMutexElementUpdate);
    this->mapUpdates.clear();
    if (bAddUpdate) {
        this->mapUpdates.push_back(new MapEventUpdate<int>(pMap->mnId, "clear", 0));
    }
}

}