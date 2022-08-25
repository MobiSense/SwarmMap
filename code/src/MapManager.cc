//
// Created by Halcao on 2020/6/19.
//

#include <Map.h>
#include <MapManager.h>
#include <ORBmatcher.h>
#include "Converter.h"
#include "AgentMediator.h"
#include "CLogger.h"

namespace ORB_SLAM2 {

unsigned long MapManager::baseMapId = -1ul;
map<unsigned long, vector<Map *> > MapManager::mapGroups;

// transform pMap2 to base map pMap1 with T12
void MapManager::MergeMap(Map *pMap1, Map *pMap2, const g2o::Sim3 &T12) {
    info("Merge Map {} and {}", pMap1->mnId, pMap2->mnId);

    if (pMap1->mpBaseRef && pMap2->mpBaseRef) {
        // if both merged, check whether in the same group
        // 如果都融合过了，那么看是否在一个组里
        if (pMap1->groupId == pMap2->groupId) {
            // in the same group, find the base one and apply sim3 on the other one
            // 如果在同一个组，通过 pMap1 到该组 base 的变换，构造出 pMap2 到该组 base 的变换
            auto Twl = pMap1->mTwl * T12;
            pMap2->SetTransform(Twl);
        } else {
            // not in the same group, then we can merge the two groups
            // 不在同一个组，则将两个组融合

            auto group1 = mapGroups[pMap1->groupId];
            auto group2 = mapGroups[pMap2->groupId];
            if (group1.size() > group2.size()) {
                // group 1 is base map group
                // 如果组1的元素比较多，组1为 base

                auto Tlw2 = pMap2->mTwl.inverse();
                for (auto &pMap: group2) {
                    // construct the transform from pMap to pMap1
                    // 构造 pMap 到 pMap1 的 base 的变换
                    auto newTwl = pMap1->mTwl * T12 * Tlw2 * pMap->mTwl;
                    pMap->SetTransform(newTwl);
                    // Move pMap to the group of pMap1
                    // 移到 pMap1 所在的组
                    MoveToMapGroup(pMap, pMap1);
                }
            } else {
                // group 2 is base map group
                // 如果组2元素比较多，组2为 base

                auto Tlw1 = pMap1->mTwl.inverse();
                for (auto &pMap: group1) {
                    // 构造 pMap 到 pMap2 的 base 的变换
                    auto newTwl = pMap2->mTwl * T12.inverse() * Tlw1 * pMap->mTwl;
                    pMap->SetTransform(newTwl);
                    // Move pMap to the group of pMap2
                    // 移到 pMap2 所在的组
                    MoveToMapGroup(pMap, pMap2);
                }
            }
        }
    } else {
        if (pMap1->mpBaseRef == nullptr && pMap2->mpBaseRef == nullptr) {
            // if both are not merged, set the first map `pMap1` as base map
            // 如果两个地图都没融合，将 pMap1 设置为 base map
            pMap1->mpBaseRef = pMap1;
        }

        // one is merged, another is not
        // add not merged map into the group
        // 这里两个地图，只有一个 mpBaseRef 为空
        if (pMap1->mpBaseRef == nullptr) {
            // pMap2 group is the base map group
            // pMap1 的 mpBaseRef 为空，pMap2 为 base map
            // TODO(halcao): validate this
//            auto newTw1 = pMap2->mTwl * T12.inverse();
            auto newTw1 = pMap2->mTwl * T12;
            pMap1->SetTransform(newTw1);
            pMap1->mpBaseRef = pMap2->mpBaseRef;

            MoveToMapGroup(pMap1, pMap2);
        } else {
            // pMap1 group is the base map group
            // pMap2 的 mpBaseRef 为空，pMap1 为 base map
            // convert T12 to the transform of pMap2 and pMap1->mpBaseRef
            auto newT12 = pMap1->mTwl * T12;
            pMap2->SetTransform(newT12);
            pMap2->mpBaseRef = pMap1->mpBaseRef;

            MoveToMapGroup(pMap2, pMap1);
        }
    }

        // TODO(halcao): fuse map
    if (baseMapId == pMap1->mnId) {
        Fuse(pMap2, pMap1->GetAllMapPoints());
    } else {
        Fuse(pMap1, pMap2->GetAllMapPoints());
    }

    // Update map elements pos and pose
    for (auto &itr: pMap2->allKFs) {
        if (!itr.second) continue;
        itr.second->UpdateConnections(false);
    }
    for (auto &itr: pMap1->allKFs) {
        if (!itr.second) continue;
        itr.second->UpdateConnections(false);
    }

////         TODO(halcao): run global BA
//    AgentMediator::RunGlobalBundleAdjustment(pMap1, pMap2);

}

void MapManager::Fuse(Map *pMap, const vector<MapPoint *>& otherMapPoints) {
    info("fuse map {} with another", pMap->mnId);
    int fused = 0;
    ORBmatcher matcher(0.8);
    for (const auto &pKF: pMap->GetAllKeyFrames()) {
        if (!pKF) continue;

//            cv::Mat cvTwl = Converter::toCvMat(pMap->mTwl);
        vector<MapPoint *> vpReplacePoints(otherMapPoints.size(), nullptr);
        matcher.Fuse(pKF, pKF->GetGlobalPose(), otherMapPoints, 4, vpReplacePoints);

        // Get Map Mutex
//            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = otherMapPoints.size();
        for (int i = 0; i < nLP; i++) {
            MapPoint *pRep = vpReplacePoints[i];
            if (pRep && otherMapPoints[i] && pRep->isBad() == false && otherMapPoints[i]->isBad() == false) {
                pRep->Replace(otherMapPoints[i], false);
//                vMapPointPairs.emplace_back(pRep, otherMapPoints[i]);
                ++fused;
            }
        }
    }

    info("Map {} fused count {}", pMap->mnId, fused);

    vector<KeyFrame *> retainedKeyframes;
    for (auto &pKF: pMap->GetAllKeyFrames()) {
        KeyFrameCulling(pKF, retainedKeyframes);
    }
}

void MapManager::Fuse(Map *pMap1, Map *pMap2) {
    Fuse(pMap1, pMap2->GetAllMapPoints());
//        Fuse(pMap2, pMap1->GetAllMapPoints());
}

void MapManager::KeyFrameCulling(KeyFrame *& mpCurrentKeyFrame, vector<KeyFrame *> &retainedKeyframes) {
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    auto vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    size_t nCulling = 0;

    // TODO(halcao): do not delete without dependency map
    for (auto pKF : vpLocalKeyFrames) {
        if (pKF->isFirst())
            continue;
        const vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();

        const int thObs = 3;
        int nRedundantObservations = 0;
        int nMPs = 0;
        for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
            MapPoint *pMP = vpMapPoints[i];
            if (!pMP || pMP->isBad()) continue;

            nMPs++;
            if (pMP->Observations() <= thObs) continue;

            const int &scaleLevel = pKF->mvKeysUn[i].octave;
            const map<KeyFrame *, size_t> observations = pMP->GetObservations();
            int nObs = 0;
            for (const auto & observation : observations) {
                KeyFrame *pKFi = observation.first;
                if (pKFi == pKF)
                    continue;
                const int &scaleLeveli = pKFi->mvKeysUn[observation.second].octave;

                if (scaleLeveli <= scaleLevel + 1) {
                    nObs++;
                    if (nObs >= thObs)
                        break;
                }
            }
            if (nObs >= thObs) {
                nRedundantObservations++;
            }
        }

        if (nRedundantObservations > 0.85 * nMPs) {
            pKF->SetBadFlag();
            pKF->SetErase();
            pKF->mRelocScore = -1;
            nCulling += 1;
        }
    }

    info("Culling keyframes {} / {}", nCulling, vpLocalKeyFrames.size());
}


void MapManager::Register(Map *pMap) {
    vector<Map *> vec{ pMap };
    auto groupId = pMap->mnId;
    // groupId is the index in mapGroups
    mapGroups[groupId] = vec;

    pMap->groupId = groupId;
}

void MapManager::MoveToMapGroup(Map *src, Map *dst) {
    if (src->groupId == dst->groupId) return;

    mapGroups[dst->groupId].push_back(src);

    if (mapGroups[src->groupId].size() > 1) {
        // if srcMap group contains more than 1 map, remove srcMap from the group
        auto &srcGroup = mapGroups[src->groupId];
        srcGroup.erase(remove_if(srcGroup.begin(), srcGroup.end(), [src](const Map *map){
            return map->mnId == src->mnId;
        }), srcGroup.end());
    } else {
        assert(mapGroups[src->groupId].size() == 1);
        // if srcMap group contains only srcMap, remove the group
        mapGroups.erase(src->groupId);
    }

    src->groupId = dst->groupId;
    src->mpBaseRef = dst->mpBaseRef;
}

// detect whether the two maps is in the same group
bool MapManager::IsInSameGroup(Map *pMap1, Map *pMap2) {
    if (!pMap1 || !pMap2 || pMap1->groupId == -1ul || pMap2->groupId == -1ul) return false;

    return pMap1->groupId == pMap2->groupId;
}

vector<Map *> MapManager::GetGroup(Map *pMap) {
    return mapGroups[pMap->groupId];
}

void MapManager::SaveGlobalMap(const string& baseName) {
    for (auto &itr: mapGroups) {
        auto group = itr.second;
        auto filename = baseName + "-group-" + to_string(itr.first);
        cout << "Saving keyframe trajectory to " << filename << " with " << group.size() << " members ...\n";

        vector<KeyFrame *> vpKFs;
        for (auto &subMap: group) {
            auto vpSubKFs = subMap->GetAllKeyFrames();
            vpKFs.insert(vpKFs.end(), vpSubKFs.begin(), vpSubKFs.end());
        }
//        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
        sort(vpKFs.begin(), vpKFs.end(), [](KeyFrame *kf1, KeyFrame *kf2) {
            return kf1->mTimeStamp < kf2->mTimeStamp;
        });

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        //cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (auto pKF : vpKFs) {
            // pKF->SetPose(pKF->GetPose()*Two);

            if (pKF->isBad())
                continue;

            cv::Mat R = pKF->GetGlobalRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetGlobalCameraCenter();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1)
              << " " << t.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }

        f.close();
        cout << "group trajectory saved!\n";
    }
}
}
