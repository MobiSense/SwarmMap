//
// Created by Halcao on 2020/7/31.
//

#include <vector>
#include "LandmarkScoring.h"
#include "MapSlice.h"
#include "MapPoint.h"
#include "Timer.h"
#include "CLogger.h"

namespace ORB_SLAM2 {

    std::map<unsigned long, std::vector<MapPointScoreItem>> LandmarkScoring::mappointMap;
    std::map<unsigned long, std::vector<KeyFrameScoreItem>> LandmarkScoring::keyframeMap;

    // max values: used to normalize scores
    std::map<unsigned long, double> LandmarkScoring::maxTrackedLength;
    std::map<unsigned long, double> LandmarkScoring::maxDistance;
    std::map<unsigned long, double> LandmarkScoring::maxAngle;
    std::map<unsigned long, double> LandmarkScoring::maxVelocity;
    std::map<unsigned long, double> LandmarkScoring::maxUpdateFreq;
    std::map<unsigned long, double> LandmarkScoring::maxObservedCount;

    // STS-related max values
    double LandmarkScoring::maxMG;
    double LandmarkScoring::maxMS;

    int LandmarkScoring::round = 1;

    double LandmarkScoring::GetRequestPriority(const MapSlice &slice) {
        // MG: Map elements generation speed (unobserved)
        double mg = 0;
        for (auto &mp: slice.MPs) {
            if (mappointMap[mp->mnId].empty()) {
                mg += 1;
            };
        }

        for (auto &mp: slice.KFs) {
            if (keyframeMap[mp->mnId].empty()) {
                mg += 1;
            };
        }

        // update max
        LandmarkScoring::maxMG = max(LandmarkScoring::maxMG, mg);
        // normalize mg
        double mg_norm = LandmarkScoring::maxMG == 0 ? 0 : (mg / LandmarkScoring::maxMG);

        // MS: Map-point score
        double ms = 0;

        int recentMPs = 0;
        const auto now = Timer::globalInstance().get();

        for (auto &itr: mappointMap) {
            if (itr.second.empty()) continue;
            const auto &item = itr.second.back();

            // filter only recent mappoints(ranked less than 5s)
            if (now - item.timestamp > 5) continue;

            recentMPs++;
            ms += item.score;
        }

        ms = recentMPs == 0 ? 0 : (ms / double(recentMPs));

        LandmarkScoring::maxMS = max(LandmarkScoring::maxMS, ms);

        double ms_norm = LandmarkScoring::maxMS == 0 ? 0 : (ms / LandmarkScoring::maxMS);

        return mg_norm - ms_norm;
    }

    // get a normalized vector pointing from mp to kf
    cv::Mat getNormalizedRay(KeyFrame *&kf, MapPoint *&mp) {
        cv::Mat ray = kf->GetGlobalCameraCenter() - mp->GetGlobalPos();
        cv::normalize(ray, ray, 1.0,0.0, cv::NORM_L2);
        return ray;
    }

    void RankByLength(MapPointScoreItem &item, MapPoint *&mp) {
        auto observations = mp->GetObservations();
        item.observedCount = observations.size();
        // if observations size less than 2
        if (observations.size() < 2) {
            item.maxDistance = 0;
            item.maxAngle = 0;

            return;
        }

        auto observingKeyframes = std::vector<KeyFrame *>();
        observingKeyframes.reserve(observations.size());
        for (auto &pair : observations) {
            if (!pair.first) continue;
            observingKeyframes.push_back(pair.first);
        }
        // sort by timestamp order
        sort(observingKeyframes.begin(), observingKeyframes.end(), [](KeyFrame* const& kf1, KeyFrame* const& kf2) {
            return kf1->mTimeStamp < kf2->mTimeStamp;
        });
        // calculate tracked length

        auto prev = observingKeyframes.front();
        auto totalLength = 0.0;
        for (size_t i = 1; i < observingKeyframes.size(); i++) {
            auto current = observingKeyframes[i];
            if (!current) {
                continue;
            }
            if (!prev) {
                prev = current;
            }
            const auto length = cv::norm(current->GetGlobalCameraCenter(), prev->GetGlobalCameraCenter());
            totalLength += length;
            prev = current;
        }
        item.trackedLength = totalLength;

        double maxDistance = 0;
        double maxAngle = 0;
        // save the ray from MapPoint to observing KeyFrame
        auto observingRays = unordered_map<unsigned long, cv::Mat>();
        for (size_t i = 0; i < observingKeyframes.size(); ++i) {
            for (auto j = i + 1; j < observingKeyframes.size(); ++j) {
                auto kf1 = observingKeyframes[i];
                auto kf2 = observingKeyframes[j];

                if (!kf1 || !kf2) continue;

                // get kf1 kf2 distance
                const auto distance = cv::norm(kf1->GetGlobalCameraCenter(), kf2->GetGlobalCameraCenter());
                maxDistance = max(distance, maxDistance);

                // get kf1 kf2 angle
                // if ray1(from mp to kf1 center) is not set
                if (!observingRays.count(kf1->mnId)) {
                    observingRays[kf1->mnId] = getNormalizedRay(kf1, mp);
                }
                // if ray2(from mp to kf2 center) is not set
                if (!observingRays.count(kf2->mnId)) {
                    observingRays[kf2->mnId] = getNormalizedRay(kf2, mp);
                }

                const auto ray1 = observingRays[kf1->mnId];
                const auto ray2 = observingRays[kf2->mnId];

                const auto dot = ray1.dot(ray2);
                double angle;
                if (dot > 1.0) {
                    angle = 0.0;
                } else if (dot < -1.0) {
                    angle = CV_PI;
                } else {
                    angle = std::acos(dot);
                }
                maxAngle = max(angle, maxAngle);
            }
        }
        item.maxDistance = maxDistance;
        item.maxAngle = maxAngle;

        // Moving velocity
        auto refKF = mp->GetReferenceKeyFrame();
        if (refKF != nullptr) {
            auto parentKF = refKF->GetParent();
            if (parentKF != nullptr && refKF->GetOriginMapId() == parentKF->GetOriginMapId()) {
                auto refTranslation = refKF->GetTranslation();
                auto parentTranslation = parentKF->GetTranslation();
                auto length = cv::norm(refTranslation, parentTranslation);
                auto time = refKF->mTimeStamp - parentKF->mTimeStamp;
                item.velocity = length / time;
            }
        }
    }

    void SetNotErase(vector<MapPoint *> vpMP, vector<KeyFrame *> vpKF) {
        const auto RETIRE_TIME = 50;
        const auto CHILD_TIME = 10;

        const auto VICINITY_TIME = 10;

        for (auto &mp: vpMP) {
            auto &item = LandmarkScoring::mappointMap[mp->mnId].back();
            const auto aliveTime = item.timestamp - item.createdTime;
            const auto activeTime = item.timestamp - item.lastTrackedTime;
            if (aliveTime < CHILD_TIME) {
                // not to be erased
                item.score = -1;
            }

            if (activeTime < VICINITY_TIME) {
                // not to be erased
                item.score = -1;
            }

            if (aliveTime > RETIRE_TIME) {
                // too old, may be already out of the corresponding zone, so try not to erase
//                item.score = -1;
            }
        }

        for (auto &kf: vpKF) {
            auto &item = LandmarkScoring::keyframeMap[kf->mnId].back();
            const auto aliveTime = item.timestamp - item.createdTime;
            // TODO(halcao): set item last tracked time
//            const auto activeTime = item.timestamp - item.lastTrackedTime;
            if (aliveTime < CHILD_TIME) {
                // not to be erased
                item.score = -1;
            }

//            if (activeTime < VICINITY_TIME) {
//                // not to be erased
//                item.score = -1;
//            }
        }

    }

    void LandmarkScoring::CalcFinalScore(MapPointScoreItem &item, const unsigned long mapId) {
        if (item.score < 0) return;

        const auto globalValues = vector<double>{LandmarkScoring::maxAngle[mapId], LandmarkScoring::maxDistance[mapId],
                                                 LandmarkScoring::maxTrackedLength[mapId], LandmarkScoring::maxVelocity[mapId],
                                                 LandmarkScoring::maxUpdateFreq[mapId], LandmarkScoring::maxObservedCount[mapId]};
        const auto values = vector<double>{
                item.maxAngle, item.maxDistance,
                item.trackedLength, item.velocity,
                double(item.updateFreq), double(item.observedCount)
        };

        double score = 0;
        for (size_t i = 0; i < values.size(); i++) {
            if (globalValues[i] == 0) continue;

            score += values[i] / globalValues[i];
        }

        item.score += score;
    }

    std::vector<double> LandmarkScoring::Rank(const MapSlice &slice) {

        std::vector<KeyFrame *> vpKF = slice.KFs;
        std::vector<MapPoint *> vpMP = slice.MPs;
        LandmarkScoring::round += 1;

        vector<MapPointScoreItem> items;
        vector<MapPoint *> goodMPs;
        vector<KeyFrame *> goodKFs;

        for (auto &mp: vpMP) {
            if (mp->isBad()) continue;
            goodMPs.push_back(mp);
        }

        for (auto &kf: vpKF) {
            if (kf->isBad() || !kf->isGenuine) continue;

            goodKFs.push_back(kf);
        }

        // Map-point evaluation
        // setup: a mapping from mp mnId to the count of its relevant updates
        unordered_map<id_t, int> mpUpdateMap;
        for (auto &update: slice.updates) {
            if (update->getType() != MapPointType) continue;
            mpUpdateMap[update->mnId] += 1;
        }

        for (auto &mp: goodMPs) {

            // init the map with initial score items
            auto item = MapPointScoreItem();
            item.mnId = mp->mnId;
            item.round = LandmarkScoring::round;

            // Observed number
            auto observations = mp->GetObservations();
            item.observedCount = observations.size();

            // Update frequency
            item.updateFreq = mpUpdateMap[mp->mnId];

            // Length related metrics
            RankByLength(item, mp);

            const auto mapId = mp->GetOriginMapId();
            LandmarkScoring::maxTrackedLength[mapId] = std::max(LandmarkScoring::maxTrackedLength[mapId], item.trackedLength);
            LandmarkScoring::maxDistance[mapId] = std::max(LandmarkScoring::maxDistance[mapId], item.maxDistance);
            LandmarkScoring::maxAngle[mapId] = std::max(LandmarkScoring::maxAngle[mapId], item.maxAngle);
            LandmarkScoring::maxVelocity[mapId] = std::max(LandmarkScoring::maxVelocity[mapId], item.velocity);
            LandmarkScoring::maxUpdateFreq[mapId] = std::max(LandmarkScoring::maxUpdateFreq[mapId], double(item.updateFreq));
            LandmarkScoring::maxObservedCount[mapId] = std::max(LandmarkScoring::maxObservedCount[mapId], double(item.observedCount));

            mappointMap[mp->mnId].push_back(item);
        }

        // get mp final scores
        for (auto &mp: goodMPs) {
            auto &item = mappointMap[mp->mnId].back();

            CalcFinalScore(item, mp->GetOriginMapId());
        }

        // get kf final scores
        for (auto &kf: goodKFs) {
            KeyFrameScoreItem item;
            item.round = LandmarkScoring::round;
            // current time
            item.timestamp = Timer::globalInstance().get();
            // MapPoint creation time
            item.createdTime = kf->mCreatedTime;

            double score = 0;
            auto observations = kf->GetMapPointMatches();
            for (auto &mp: observations) {
                if (!mp || mappointMap[mp->mnId].empty()) continue;

                auto &item = mappointMap[mp->mnId].back();

                score += item.score;
            }
            if (observations.size() > 0) {
                score /= double(observations.size());
            }
            item.score = score;

            keyframeMap[kf->mnId].push_back(item);
        }
    }

    void LandmarkScoring::Save(const string &filename) {
        info("Save landmark scoring result to file: {}", filename);

        {
            ofstream f;
            string mpName = filename + "mp.csv";
            f.open(mpName);
            f << fixed;

            f << "id,round,timestamp,createdTime,lastTrackedTime,observedCount,trackedLength,maxDistance,maxAngle,score" << endl;
            for (auto &itr: mappointMap) {
                auto &vec = itr.second;
                for (auto &item: vec) {
                    f << setprecision(6) << itr.first << "," << item.round << "," << item.timestamp << "," << item.createdTime << "," << item.lastTrackedTime << "," << item.observedCount << ","
                      << item.trackedLength << "," << item.maxDistance << "," << item.maxAngle << "," << item.score << endl;
                }
                //
            }

            f.close();
        }
        {
            ofstream f;
            string mpName = filename + "kf.csv";
            f.open(mpName);
            f << fixed;

            f << "id,round,timestamp,createdTime,speed,matchedRatio,totalPoints,pointScore,connectedKF,bestCovisibleCount,score" << endl;
            for (auto &itr: keyframeMap) {
                auto &vec = itr.second;
                for (auto &item: vec) {
                    f << setprecision(6) << itr.first << "," << item.round << "," << item.timestamp << "," << item.createdTime << "," << item.speed << "," << item.matchedRatio << ","
                      << item.totalPoints << "," << item.pointScore << "," << item.connectedKF << "," << item.bestCovisibleCount << "," << item.score << endl;
                }
                //
            }

            f.close();
        }
        info("landmark scoring saved!");
    }

std::vector<double> LandmarkScoring::GetScores(const std::vector<KeyFrame *> KFs) {
    vector<double> result(KFs.size(), 0);
    for (size_t i = 0; i < KFs.size(); i++) {
        auto &kf = KFs[i];

        if (keyframeMap[kf->mnId].empty()) continue;
        result[i] = keyframeMap[kf->mnId].back().score;
    }

    return result;
}

std::vector<double> LandmarkScoring::GetScores(const std::vector<MapPoint *> MPs) {
    vector<double> result(MPs.size(), 0);
    for (size_t i = 0; i < MPs.size(); i++) {
        auto &mp = MPs[i];

        if (mappointMap[mp->mnId].empty()) continue;
        result[i] = mappointMap[mp->mnId].back().score;
    }

    return result;
}
}