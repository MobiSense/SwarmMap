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

#include "MapPoint.h"
#include "ORBmatcher.h"
#include "Timer.h"
#include <Thirdparty/g2o/g2o/types/sim3.h>
#include "Converter.h"
#include "CLogger.h"

#include<mutex>

namespace ORB_SLAM2 {

    long unsigned int MapPoint::nNextId = 0;
    mutex MapPoint::mGlobalMutex;

    MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap) :
            mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(nullptr), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
            mbToBeSerialized(true), mDynamic(0) {
        Pos.copyTo(mWorldPos);
        mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

        mTimeStamp = Timer::globalInstance().get();
        mLastTrackedTime = mTimeStamp;
        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = (nNextId++) + mpMap->mnIdBase;

        pMap->RegisterMapPoint(this);
    }

    MapPoint::MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF) :
            mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
            mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(nullptr), mnVisible(1),
            mnFound(1), mbBad(false), mpReplaced(nullptr), mpMap(pMap), mbToBeSerialized(false), mDynamic(0) {
        Pos.copyTo(mWorldPos);
        cv::Mat Ow = pFrame->GetCameraCenter();
        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / cv::norm(mNormalVector);

        cv::Mat PC = Pos - Ow;
        const float dist = cv::norm(PC);
        const int level = pFrame->mvKeysUn[idxF].octave;
        const float levelScaleFactor = pFrame->mvScaleFactors[level];
        const int nLevels = pFrame->mnScaleLevels;

        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

        pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

        mTimeStamp = Timer::globalInstance().get();
        mLastTrackedTime = mTimeStamp;
        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;

//        pMap->RegisterMapPoint(this);
    }

    void MapPoint::SetWorldPos(const cv::Mat &Pos, bool bAddUpdate) {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);

        if (bAddUpdate) {
            mpMap->AddUpdate(new MapPointUpdate<cv::Mat>(
                    this->mnId, __func__, Pos));
        }

        Pos.copyTo(mWorldPos);
        UpdateGlobalPos(false);
    }

    void MapPoint::UpdateGlobalPos(bool bLock) {
        auto lock = bLock ? unique_lock<mutex>(mMutexPos) : unique_lock<mutex>();
        auto pos = mpMap->mTwl.map(Converter::toVector3d(mWorldPos));
        mGlobalPos = Converter::toCvMat(pos);
    }

    void MapPoint::SetGlobalPos(const cv::Mat &globalPos) {
        auto pos = mpMap->mTwl.inverse().map(Converter::toVector3d(globalPos));
        SetWorldPos(Converter::toCvMat(pos));
    }

    cv::Mat MapPoint::GetWorldPos() {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos.clone();
    }

    cv::Mat MapPoint::GetGlobalPos() {
        unique_lock<mutex> lock(mMutexPos);
        if (!mGlobalPos.data) UpdateGlobalPos(false);

        return mGlobalPos.clone();
    }

    cv::Mat MapPoint::GetNormal() {
        unique_lock<mutex> lock(mMutexPos);
        return mNormalVector.clone();
    }

    KeyFrame *MapPoint::GetReferenceKeyFrame() {
        unique_lock<mutex> lock(mMutexFeatures);
        if (!mpRefKF) {
            mpRefKF = mpMap->GetKeyFrame(mnRefKFId);
            if (mpRefKF) return mpRefKF;

            for (auto obs: mObservations) {
                if (obs.first) {
                    mpRefKF = obs.first;
                    return mpRefKF;
                }
            }
        }
        return mpRefKF;
    }

    void MapPoint::AddObservation(KeyFrame *pKF, size_t idx, bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexFeatures);

        if (!pKF) {
            warn("mp {} AddObservation error", mnId);
            return;
        }

        if (idx >= size_t(pKF->mDescriptors.rows)) {
            warn("observation index {} > mDescriptors.rows {}", idx, pKF->mDescriptors.rows);
            return;
        }

        if (bAddUpdate) {
            mpMap->AddUpdate(new MapPointUpdate<std::pair<unsigned long, size_t> >(
                    this->mnId, __func__, std::make_pair<unsigned long, size_t>((unsigned long) (pKF->mnId),
                                                                                size_t(idx))));
//            debug("mp {} AddObservation kf {} idx {}", this->mnId, pKF->mnId, idx);
        }

        mObservations[pKF] = idx;

        if (pKF->mvuRight[idx] >= 0)
            nObs += 2;
        else
            nObs++;
    }

    void MapPoint::EraseObservation(KeyFrame *pKF, bool bAddUpdate) {
        if (!pKF) return;

        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);

            if (bAddUpdate) {
                mpMap->AddUpdate(new MapPointUpdate<unsigned long>(
                        this->mnId, __func__, (unsigned long) (pKF->mnId)));
            }

            if (mObservations.count(pKF)) {
                int idx = mObservations[pKF];
                if (pKF->mvuRight[idx] >= 0)
                    nObs -= 2;
                else
                    nObs--;

                auto itr = mObservations.find(pKF);
                if (itr != mObservations.end()) {
                    mObservations.erase(itr);
                }
//            mObservations.erase(pKF);

                if (mpRefKF == pKF)
                    mpRefKF = mObservations.begin()->first;

                // If only 2 observations or less, discard point
                if (nObs <= 2)
                    bBad = true;
            }
        }

        if (bBad)
            SetBadFlag(false);
    }

    map<KeyFrame *, size_t> MapPoint::GetObservations() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    int MapPoint::Observations() {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    void MapPoint::SetBadFlag(bool bAddUpdate) {
        map<KeyFrame *, size_t> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);

            // if the function is called from EraseObservation, there is already an update sent
            if (bAddUpdate) {
                mpMap->AddUpdate(new MapPointUpdate<int>(this->mnId, __func__, int(0)));
            }

            mbBad = true;
            obs = mObservations;

            mObservations.clear();
        }
        for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            KeyFrame *pKF = mit->first;
            pKF->EraseMapPointMatch(mit->second, false);
        }

        mpMap->EraseMapPoint(this);
        SetSerialized(false);
    }

    MapPoint *MapPoint::GetReplaced() {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mpReplaced;
    }

    void MapPoint::Replace(MapPoint *pMP, bool bAddUpdate) {
        if (pMP->mnId == this->mnId)
            return;

        if (bAddUpdate) {
            mpMap->AddUpdate(new MapPointUpdate<unsigned long>(
                    this->mnId, __func__, (unsigned long) (pMP->mnId)));
        }

        int nvisible, nfound;
        map<KeyFrame *, size_t> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs = mObservations;
            mObservations.clear();
            mbBad = true;
            nvisible = mnVisible;
            nfound = mnFound;
            mpReplaced = pMP;
        }

        for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            // Replace measurement in keyframe
            KeyFrame *pKF = mit->first;

            if (!pMP->IsInKeyFrame(pKF)) {
                pKF->ReplaceMapPointMatch(mit->second, pMP, false);
                pMP->AddObservation(pKF, mit->second, false);
            } else {
                pKF->EraseMapPointMatch(mit->second, false);
            }
        }
        pMP->IncreaseFound(nfound, false);
        pMP->IncreaseVisible(nvisible, false);
        pMP->ComputeDistinctiveDescriptors(false);

        mpMap->EraseMapPoint(this);
    }

    bool MapPoint::isBad() {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mbBad;
    }

    void MapPoint::IncreaseVisible(int n, bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexFeatures);
        if (bAddUpdate) {
            mpMap->AddUpdate(new MapPointUpdate<int>(
                    this->mnId, __func__, (int) (n)));

            // add update follow the parent function bAddUpdate
            const auto now = Timer::globalInstance().get();
            this->SetLastTrackedTime(now);
        }

        mnVisible += n;
    }

    void MapPoint::IncreaseFound(int n, bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexFeatures);

        if (bAddUpdate) {
            mpMap->AddUpdate(new MapPointUpdate<int>(
                    this->mnId, __func__, (int) (n)));
        }

        mnFound += n;
    }

    float MapPoint::GetFoundRatio() {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound) / mnVisible;
    }

    void MapPoint::ComputeDistinctiveDescriptors(bool bAddUpdate) {
        if (bAddUpdate) {
            mpMap->AddUpdate(new MapPointUpdate<int>(
                    this->mnId, __func__, int(0)));
        }

        // Retrieve all observed descriptors
        vector<cv::Mat> vDescriptors;

        map<KeyFrame *, size_t> observations;

        {
            unique_lock<mutex> lock1(mMutexFeatures);
            if (mbBad)
                return;
            observations = mObservations;
        }

        if (observations.empty())
            return;

        vDescriptors.reserve(observations.size());

        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKF = mit->first;

            if (!pKF->isBad()) {
                if (mit->second >= size_t(pKF->mDescriptors.rows)) {
                    continue;
                }
                vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
            }
        }

        if (vDescriptors.empty())
            return;

        // Compute distances between them
        const size_t N = vDescriptors.size();

        float distances[N][N];
        for (size_t i = 0; i < N; i++) {
            distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++) {
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
                distances[i][j] = distij;
                distances[j][i] = distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for (size_t i = 0; i < N; i++) {
            vector<int> vDists(distances[i], distances[i] + N);
            sort(vDists.begin(), vDists.end());
            int median = vDists[0.5 * (N - 1)];

            if (median < BestMedian) {
                BestMedian = median;
                BestIdx = i;
            }
        }

        {
            unique_lock<mutex> lock(mMutexFeatures);
            mDescriptor = vDescriptors[BestIdx].clone();
        }
    }

    cv::Mat MapPoint::GetDescriptor() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }

    int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return static_cast<int>(mObservations[pKF]);
        else
            return -1;
    }

    bool MapPoint::IsInKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);

        if (mObservations.empty() || !pKF) return false;

        return (mObservations.count(pKF));
    }

    void MapPoint::UpdateNormalAndDepth(bool bAddUpdate) {
        if (bAddUpdate) {
            mpMap->AddUpdate(new MapPointUpdate<int>(
                    this->mnId, __func__, int(0)));
        }

        map<KeyFrame *, size_t> observations;
        KeyFrame *pRefKF;
        cv::Mat Pos;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if (mbBad)
                return;
            observations = mObservations;
            pRefKF = mpRefKF;
            Pos = mWorldPos.clone();
        }

        if (observations.empty())
            return;

        cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
        int n = 0;
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKF = mit->first;
            cv::Mat Owi = pKF->GetCameraCenter();
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali / cv::norm(normali);
            n++;
            if (!mpRefKF) {
                mpRefKF = mit->first;
                pRefKF = mpRefKF;
            }
        }

        cv::Mat PC = Pos - pRefKF->GetCameraCenter();
        const float dist = cv::norm(PC);
        const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
        const float levelScaleFactor = pRefKF->mvScaleFactors[level];
        const int nLevels = pRefKF->mnScaleLevels;

        {
            unique_lock<mutex> lock3(mMutexPos);
            mfMaxDistance = dist * levelScaleFactor;
            mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
            mNormalVector = normal / n;
        }
    }

    float MapPoint::GetMinDistanceInvariance() {
        unique_lock<mutex> lock(mMutexPos);
        return 0.8f * mfMinDistance;
    }

    float MapPoint::GetMaxDistanceInvariance() {
        unique_lock<mutex> lock(mMutexPos);
        return 1.2f * mfMaxDistance;
    }

    int MapPoint::PredictScale(const float &currentDist, const float &logScaleFactor, const int mnScaleLevel) {
        float ratio;
        {
            unique_lock<mutex> lock3(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / logScaleFactor);
        return max(0, min(nScale, mnScaleLevel - 1));
    }

    void MapPoint::SetupSerializationVariable() {
        unique_lock<mutex> lock(mMutexFeatures);
        // replace mObservations
        mIdObservations.clear();
        KeyFrame *firstObservedKF = nullptr;
        for (auto &itr: mObservations) {
            if (itr.first && !(itr.first->isBad())) {
                mIdObservations[itr.first->mnId] = itr.second;

                if (!itr.first->isBad() && firstObservedKF == nullptr) {
                    firstObservedKF = itr.first;
                    mpRefKF = firstObservedKF;
                }
            } else {
                auto id = itr.first ? itr.first->mnId : -1ul;
                debug("Storing MapPoint {}: observations empty or {} bad", itr.second, id);
            }
        }

        // replace mpRefKF
        if (mpRefKF) {
            mnRefKFId = mpRefKF->mnId;
        } else {
            // TODO(halcao): handle this
//            assert(mpRefKF != nullptr);
            mnRefKFId = -1ul;
        }

        {
            unique_lock<mutex> lock2(mMutexPos);
            // replace mpReplaced
            if (mpReplaced) {
                mnReplacedId = mpReplaced->mnId;
            } else {
                mnReplacedId = -1ul;
            }
        }
    }

    void MapPoint::RestoreSerialization() {
        /**
         * MapPoint
         * 1. replace mpRefKF -> mnRefKFId
         * 2. replace mpReplaced -> mnReplacedID
         * 3. replace mObservations to mIdObservations
         */
        unique_lock<mutex> lock(mMutexFeatures);

        mObservations.clear();
        for (auto &itr: mIdObservations) {
            KeyFrame *pKF = mpMap->GetKeyFrame(itr.first);

            if (!pKF) {
                if (itr.first != -1ul) {
                    mpMap->restorationMPQueue.insert(this->mnId);
                    debug("Restoring MapPoint {}: observation {} empty", this->mnId, itr.first);
                }
            } else {
                mObservations.emplace(pKF, itr.second);
            }
        }

        mpRefKF = mpMap->GetKeyFrame(mnRefKFId);

        {
            unique_lock<mutex> lock2(mMutexPos);
            mpReplaced = mpMap->GetMapPoint(mnReplacedId);
        }
        // Update global pos
//        UpdateGlobalPos();
    }

    unsigned long MapPoint::GetOriginMapId() const {
        return mnId / MAP_BASE;
    }

    void MapPoint::SetLastTrackedTime(double time, bool bAddUpdate) {
        if (bAddUpdate) {
            mpMap->AddUpdate(new MapPointUpdate<double>(this->mnId, __func__, double(time)));
        }
        this->mLastTrackedTime = time;
    }

    double MapPoint::GetLastTrackedTime() const {
        return this->mLastTrackedTime;
    }

    void MapPoint::SetFound(int n) {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound = n;
    }

    void MapPoint::SetVisible(int n) {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible = n;
    }

    int MapPoint::GetVisible() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mnVisible;
    }

} //namespace ORB_SLAM
