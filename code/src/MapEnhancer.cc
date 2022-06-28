//
// Created by zjl on 11/16/2020.
//

#include <unordered_set>
#include "MapEnhancer.h"
#include "MapSlice.h"
#include "MapManager.h"
#include "LandmarkScoring.h"
#include "CLogger.h"

namespace ORB_SLAM2 {

inline bool isInImage(const float &x, const float &y) {
    // todo: check if this value can get
    return (x >= Frame::mnMinX && x < Frame::mnMaxX && y >= Frame::mnMinY && y <Frame:: mnMaxY);
}

// isIdentical: generate the same keyframe: used for testing
KeyFrame *MapEnhancer::GenerateKeyFrame(const cv::Mat& Scw, KeyFrame *pRefKF, bool isIdentical) {
    auto originPoints = pRefKF->GetMapPointMatches();
    cout << "origin points: " << originPoints.size() << endl;
    auto relatedKFs = pRefKF->GetConnectedKeyFrames();
    auto MPs = set<MapPoint *>(originPoints.begin(), originPoints.end());
    if(!isIdentical) {
        for (auto &kf: relatedKFs) {
            if (!kf->isGenuine) continue;
            auto points = kf->GetMapPointMatches();
            copy(points.begin(), points.end(), inserter(MPs, MPs.end()));
        }
    }

    auto vpPoints = vector<MapPoint *>(MPs.begin(), MPs.end());
//        auto vpPoints = originPoints;

    const float &fx = Frame::fx;
    const float &fy = Frame::fy;
    const float &cx = Frame::cx;
    const float &cy = Frame::cy;

    // Decompose Scw
    cv::Mat sRcw = Scw.rowRange(0, 3).colRange(0, 3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw / scw;
    cv::Mat tcw = Scw.rowRange(0, 3).col(3) / scw;
    cv::Mat Ow = -Rcw.t() * tcw;

    int nmatches = 0;

    int N = 0;
    vector<MapPoint *> mvpMapPointsQualified;
    vector<cv::KeyPoint> mvKeysUn;
    cv::Mat mDescriptors(vpPoints.size(), 32, CV_8U);
    // For each Candidate MapPoint Project and Match
    for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++) {
        MapPoint *pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if (!pMP || pMP->isBad())
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetGlobalPos();

        // Transform into Camera Coords.
        cv::Mat p3Dc = Rcw * p3Dw + tcw;

        // Depth must be positive
        if (p3Dc.at<float>(2) < 0.0)
            continue;

        // Project into Image
        const float invz = 1 / p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0) * invz;
        const float y = p3Dc.at<float>(1) * invz;

        const float u = fx * x + cx;
        const float v = fy * y + cy;

        // Point must be inside the image
        if (!isInImage(u, v))
            continue;

        // Depth must be inside the scale invariance region of the point
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw - Ow;
        const float dist = cv::norm(PO);

        if (dist < minDistance || dist > maxDistance)
            continue;

        // judge part ended
        // use corresponding keypoint in keyframe as keypoint in this synthesized frame

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist)
            continue;

        // mvKeysUn, N, mvDepth, mvpmappoint
        KeyFrame *pRefFrame = pMP->GetReferenceKeyFrame();
        if (!pRefFrame || pRefFrame->isBad())
            continue;

        int nIdx = pMP->GetIndexInKeyFrame(pRefFrame);
        cv::KeyPoint kp = pRefFrame->mvKeysUn[nIdx];
        kp.pt.x = u;
        kp.pt.y = v;
        mvKeysUn.push_back(kp);
        mvpMapPointsQualified.push_back(pMP);
        cv::Mat mSingleDescriptor = pRefFrame->mDescriptors.row(nIdx);
        mSingleDescriptor.copyTo(mDescriptors.row(N));
        N++;

//            mDescriptors = mDescriptors(cv::Range(0, N), cv::Range::all()).clone();
    }

    info("from kf {} generated keyframe has {} MapPoints", pRefKF->mnId, N);

    mDescriptors = mDescriptors.rowRange(0, N);

    if (mvpMapPointsQualified.empty()) return nullptr;

    auto pMap = pRefKF->mpMap;
    auto K = pRefKF->mK;
    auto pGeneratedKF = new KeyFrame(pMap, pRefKF->mpKeyFrameDB, nullptr, Scw, K, mvKeysUn, mDescriptors,
                                     mvpMapPointsQualified);

    // add observation
    for (int i = 0; i < N; ++i) {
        MapPoint *pMp = mvpMapPointsQualified[i];
        pMp->AddObservation(pGeneratedKF, i, false);
    }

    pGeneratedKF->UpdateConnections(false);
    pGeneratedKF->isGenuine = false;

    pGeneratedKF->mnScaleLevels = pRefKF->mnScaleLevels;
    pGeneratedKF->mfScaleFactor = pRefKF->mfScaleFactor;
    pGeneratedKF->mfLogScaleFactor = pRefKF->mfLogScaleFactor;
    pGeneratedKF->mvScaleFactors = pRefKF->mvScaleFactors;
    pGeneratedKF->mvLevelSigma2 = pRefKF->mvLevelSigma2;
    pGeneratedKF->mvInvLevelSigma2 = pRefKF->mvInvLevelSigma2;

    // TODO(halcao): optimize the pose

    return pGeneratedKF;
}

//vector<MapPoint *> MapEnhancer::GetVirtualKeyFrame(MapSlice &slice) {
KeyFrame * MapEnhancer::GetVirtualKeyFrame(const MapSlice &slice) {
    unordered_map<id_t, Map *> maps;
    for (auto &kf: slice.KFs) {
        if (!kf || kf->isBad() || kf->mpMap == nullptr || maps[kf->mpMap->mnId] == nullptr) continue;

        maps[kf->mpMap->mnId] = kf->mpMap;
        auto subMaps = MapManager::GetGroup(kf->mpMap);
        for (auto &map: subMaps) {
            if (map == nullptr) continue;

            maps[map->mnId] = map;
        }
    }

    // if the map is isolated, return empty
    if (maps.size() == 1) return {};

    // calculate the average score of the kf pairs

    const auto scores = LandmarkScoring::GetScores(slice.KFs);

    if (slice.KFs.size() <= 1) return {};

    vector<double> avgScores(slice.KFs.size() - 1, INT_MAX);
    double minScore = INT_MAX;
    int minIdx = 0;

    for (size_t i = 0; i < slice.KFs.size() - 1; ++i) {
        auto kf = slice.KFs[i];
        auto nextKF = slice.KFs[i + 1];

        if (!kf || !nextKF) continue;

        // get the average score
        const cv::Mat vec = nextKF->GetGlobalTranslation() - kf->GetGlobalTranslation();
        const double dist = cv::norm(vec);
        const double score = (scores[i] + scores[i + 1]) / dist;
        avgScores[i] = score;

        // find the min score
        minScore = std::min(minScore, score);
        if (score == minScore) minIdx = i;
    }

    // position to insert the keyframe
    const cv::Mat loc = (slice.KFs[minIdx]->GetGlobalTranslation() +
            slice.KFs[minIdx + 1]->GetGlobalTranslation()) / 2;


    cv::Mat Tcw = slice.KFs[minIdx]->GetGlobalPose();
    loc.copyTo(Tcw.rowRange(0, 3).col(3));

    auto kf = GenerateKeyFrame(Tcw, slice.KFs[minIdx], false);
    return kf;
//    // find the relevant better map-points
//    unordered_set<MapPoint *> candidateMPs;
//    vector<KeyFrame *> nearKFs;
//    for (size_t i = minIdx - 1; i < slice.KFs.size(); ++i) {
//        if (i < 0) continue;
//
//        auto kf = slice.KFs[i];
//
//        nearKFs.push_back(kf);
//        const auto points = kf->GetMapPoints();
//        std::copy(points.begin(), points.end(),std::inserter(candidateMPs,candidateMPs.end()));
//    }
//


    return {};
}

// find the bottom 20% score of the vector
double findBottom(const std::vector<double> &scores) {
    if (scores.size() < 2) return 0;

    vector<double> dup(scores);
    sort(dup.begin(), dup.end());
    const size_t idx = dup.size() * 0.2;
    if (idx <= 0) return 0;
    return dup[idx];
}

void MapEnhancer::Compress(Map *pMap) {
    if (pMap == nullptr) return;

    const auto KFs = pMap->GetAllKeyFrames();
    const auto MPs = pMap->GetAllMapPoints();

    const auto kfScores = LandmarkScoring::GetScores(KFs);
    const auto mpScores = LandmarkScoring::GetScores(MPs);

    const auto kfThreshold = findBottom(kfScores);
    const auto mpThreshold = findBottom(mpScores);

    // TODO(halcao): fine-tune the threshold
    return;

    // remove the low-score keyframes
    size_t kfCount = 0;
    for (size_t i = 0; i < KFs.size(); ++i) {
        auto kf = KFs[i];
        if (kfScores[i] < kfThreshold) {
            kf->SetBadFlag(false);
            kfCount++;
        }
    }

    // remove the low-score map-points
    size_t mpCount = 0;
    for (size_t i = 0; i < MPs.size(); ++i) {
        auto mp = MPs[i];
        if (mpScores[i] < mpThreshold) {
            mp->SetBadFlag(false);
            mpCount++;
        }
    }

    info("{} keyframes and {} map-points are going to be removed", kfCount, mpCount);
}
}




