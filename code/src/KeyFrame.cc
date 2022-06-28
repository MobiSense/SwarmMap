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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <Map.h>
#include<mutex>
#include "Timer.h"
#include "CLogger.h"

namespace ORB_SLAM2 {

//    long unsigned int KeyFrame::nNextId = 0;
    map<unsigned long, unsigned long> KeyFrame::nNextIds;

    KeyFrame::KeyFrame() :
            mnFrameId(0), mTimeStamp(0.0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mfGridElementWidthInv(0.0), mfGridElementHeightInv(0.0), mnTrackReferenceForFrame(0), mnFuseTargetForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mLoopScore(0.0), mnRelocQuery(0),
            mnRelocWords(0), mRelocScore(0.0), mnBAGlobalForKF(0), fx(0.0), fy(0.0), cx(0.0), cy(0.0), invfx(0.0),
            invfy(0.0), mbf(0.0), mb(0.0), mThDepth(0.0), N(0), mnScaleLevels(0), mfScaleFactor(0),
            mfLogScaleFactor(0.0), mnMinX(0), mnMinY(0), mnMaxX(0), mnMaxY(0), mpKeyFrameDB(NULL),
            mbFirstConnection(true), mpParent(NULL), mbNotErase(false), mbToBeErased(false),
            mbBad(false), mHalfBaseline(0.0), mpMap(NULL), mbToBeSerialized(true), mCreatedTime(0) {
        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++) {
            mGrid[i].resize(mnGridRows);
        }
    }

    KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB) :
            mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mLoopScore(0.0), mnRelocQuery(0), mnRelocWords(0), mRelocScore(0.0),
            mnBAGlobalForKF(0),
            fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
            mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
            mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
            mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
            mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
            mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
            mnMaxY(F.mnMaxY), mK(F.mK), mpKeyFrameDB(pKFDB), mvpMapPoints(F.mvpMapPoints),
            mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
            mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb / 2), mpMap(pMap), mbToBeSerialized(true),
            mCreatedTime(Timer::globalInstance().get()) {
        mnId = (KeyFrame::nNextIds[mpMap->mnId]++) + mpMap->mnIdBase;

        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++) {
            mGrid[i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
                mGrid[i][j] = F.mGrid[i][j];
        }

        SetPose(F.mTcw, false);

        pMap->RegisterKeyFrame(this);

    }

    KeyFrame::KeyFrame(Map* pMap, KeyFrameDatabase* pKeyFrameDB, ORBextractor* pORBextractor, cv::Mat Scw, const cv::Mat K, vector<cv::KeyPoint>& vKeysUn, cv::Mat Descriptors, vector<MapPoint* >& vpMapPoints) :
            mnFrameId(-1), mTimeStamp(0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mfGridElementWidthInv(Frame::mfGridElementWidthInv), mfGridElementHeightInv(Frame::mfGridElementHeightInv),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mLoopScore(0.0), mnRelocQuery(0), mnRelocWords(0), mRelocScore(0.0),
            mnBAGlobalForKF(0),
            fx(Frame::fx), fy(Frame::fy), cx(Frame::cx), cy(Frame::cy), invfx(Frame::invfx), invfy(Frame::invfy),
            mbf(0), mb(0), mThDepth(0),
            // parts that's need to be generated - start
            N(vKeysUn.size()), mvKeys(vKeysUn), mvKeysUn(vKeysUn),
            mvuRight(vector<float>(N,-1)), mvDepth(vector<float>(N,-1)), mDescriptors(Descriptors),
            // mBowVec(F.mBowVec), mFeatVec(F.mFeatVec),
            // parts that's need to be generated - end
            //mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor), mfLogScaleFactor(F.mfLogScaleFactor),
            // mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2), mvInvLevelSigma2(F.mvInvLevelSigma2),
            mnMinX(Frame::mnMinX), mnMinY(Frame::mnMinY), mnMaxX(Frame::mnMaxX),
            mnMaxY(Frame::mnMaxY), mK(K), mpKeyFrameDB(pKeyFrameDB), mvpMapPoints(vpMapPoints),
            mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
            mbToBeErased(false), mbBad(false), mHalfBaseline(0), mpMap(pMap), mbToBeSerialized(true),
            mCreatedTime(Timer::globalInstance().get()){

        mnId = (KeyFrame::nNextIds[mpMap->mnId]++) + mpMap->mnIdBase;

        int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++) {
            mGrid[i].resize(mnGridRows);
        }
        AssignFeaturesToGrid();

        if (pORBextractor) {
            mnScaleLevels = pORBextractor->GetLevels();
            mfScaleFactor = pORBextractor->GetScaleFactor();
            mfLogScaleFactor = log(mfScaleFactor);
            mvScaleFactors = pORBextractor->GetScaleFactors();
            mvLevelSigma2 = pORBextractor->GetScaleSigmaSquares();
            mvInvLevelSigma2 = pORBextractor->GetInverseScaleSigmaSquares();
        } else {
//            warn("MAKE SURE YOU WILL INITIALIZE THESE VARIABLES!");
        }

        SetPose(Scw, false);
        ComputeBoW();
        pMap->RegisterKeyFrame(this);
    }

    void KeyFrame::ComputeBoW() {
        if (mBowVec.empty() || mFeatVec.empty()) {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            mpKeyFrameDB->mpVoc->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void KeyFrame::SetPose(const cv::Mat &Tcw_, bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexPose);

        if (bAddUpdate) {
            mpMap->AddUpdate(new KeyFrameUpdate<cv::Mat>(this->mnId, __func__, Tcw_));
        }

        Tcw_.copyTo(Tcw);
        cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc * tcw;

        Twc = cv::Mat::eye(4, 4, Tcw.type());
        Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
        Ow.copyTo(Twc.rowRange(0, 3).col(3));
        cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
        Cw = Twc * center;

        // do not lock twice
        UpdateGlobalPose(false);
    }

    void KeyFrame::UpdateGlobalPose(bool bLock) {
        auto lock = bLock ? unique_lock<mutex>(mMutexPose) : unique_lock<mutex>();

        // Set Global Pose
        mGlobalTcw = Tcw * Converter::toCvMat(mpMap->mTwl.inverse()) * mpMap->mTwl.scale();
        mGlobalTcw.at<float>(3, 3) = 1;

        cv::Mat globalRcw = mGlobalTcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat globaltcw = mGlobalTcw.rowRange(0, 3).col(3);

        cv::Mat globalRwc = globalRcw.t();
        mGlobalOw = -globalRwc * globaltcw;

        mGlobalTwc = cv::Mat::eye(4, 4, Tcw.type());
        globalRwc.copyTo(mGlobalTwc.rowRange(0, 3).colRange(0, 3));
        mGlobalOw.copyTo(mGlobalTwc.rowRange(0, 3).col(3));
        // Stereo
        //        cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
        //        Cw = Twc * center;
    }

    void KeyFrame::SetGlobalPose(const cv::Mat &globalTcw_) {
        cv::Mat Tcw_ = globalTcw_ * Converter::toCvMat(mpMap->mTwl) / mpMap->mTwl.scale();
        Tcw_.at<float>(3, 3) = 1;
        SetPose(Tcw_);
    }

    cv::Mat KeyFrame::GetPose() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.clone();
    }

    cv::Mat KeyFrame::GetPoseInverse() {
        unique_lock<mutex> lock(mMutexPose);
        return Twc.clone();
    }

    cv::Mat KeyFrame::GetCameraCenter() {
        unique_lock<mutex> lock(mMutexPose);
        return Ow.clone();
    }

    cv::Mat KeyFrame::GetGlobalPose() {
        unique_lock<mutex> lock(mMutexPose);

        // if null, update it
        if (!mGlobalTcw.data) UpdateGlobalPose(false);

        return mGlobalTcw.clone();
    }

    cv::Mat KeyFrame::GetGlobalPoseInverse() {
        unique_lock<mutex> lock(mMutexPose);
        // if null, update it
        if (!mGlobalTwc.data) UpdateGlobalPose(false);

        return mGlobalTwc.clone();
    }

    cv::Mat KeyFrame::GetGlobalCameraCenter() {
        unique_lock<mutex> lock(mMutexPose);

        if (!mGlobalOw.data) UpdateGlobalPose(false);

        return mGlobalOw.clone();
    }

    cv::Mat KeyFrame::GetStereoCenter() {
        unique_lock<mutex> lock(mMutexPose);
        return Cw.clone();
    }

    cv::Mat KeyFrame::GetRotation() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0, 3).colRange(0, 3).clone();
    }

    cv::Mat KeyFrame::GetTranslation() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0, 3).col(3).clone();
    }

    cv::Mat KeyFrame::GetGlobalRotation() {
        unique_lock<mutex> lock(mMutexPose);
        // if null, update it
        if (!mGlobalTcw.data) UpdateGlobalPose(false);

        return mGlobalTcw.rowRange(0, 3).colRange(0, 3).clone();
    }

    cv::Mat KeyFrame::GetGlobalTranslation() {
        unique_lock<mutex> lock(mMutexPose);
        // if null, update it
        if (!mGlobalTcw.data) UpdateGlobalPose(false);

        return mGlobalTcw.rowRange(0, 3).col(3).clone();
    }

    void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight, bool bAddUpdate) {
        {
            unique_lock<mutex> lock(mMutexConnections);

            if (bAddUpdate) {
                mpMap->AddUpdate(new KeyFrameUpdate<std::pair<unsigned long, int> >(
                        this->mnId, __func__, std::make_pair<unsigned long, int>((unsigned long) (pKF->mnId),
                                                                                 int(weight))));
            }

            if (!mConnectedKeyFrameWeights.count(pKF))
                mConnectedKeyFrameWeights[pKF] = weight;
            else if (mConnectedKeyFrameWeights[pKF] != weight)
                mConnectedKeyFrameWeights[pKF] = weight;
            else
                return;
        }

        UpdateBestCovisibles();
    }

    void KeyFrame::UpdateBestCovisibles() {
        unique_lock<mutex> lock(mMutexConnections);

        if (mConnectedKeyFrameWeights.empty()) return;

        vector<pair<int, KeyFrame *> > vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        for (auto mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
             mit != mend; mit++)
            vPairs.emplace_back(mit->second, mit->first);

        sort(vPairs.begin(), vPairs.end());
        list<KeyFrame *> lKFs;
        list<int> lWs;
        for (auto & vPair : vPairs) {
            lKFs.push_front(vPair.second);
            lWs.push_front(vPair.first);
        }

        mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
    }

    set<KeyFrame *> KeyFrame::GetConnectedKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        set<KeyFrame *> s;
        for (auto & mConnectedKeyFrameWeight : mConnectedKeyFrameWeights)
            s.insert(mConnectedKeyFrameWeight.first);
        return s;
    }

    vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        return mvpOrderedConnectedKeyFrames;
    }

    vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
        unique_lock<mutex> lock(mMutexConnections);
        if ((int) mvpOrderedConnectedKeyFrames.size() < N)
            return mvpOrderedConnectedKeyFrames;
        else
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);

    }

    vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w) {
        unique_lock<mutex> lock(mMutexConnections);
        auto result = vector<KeyFrame *>();
        if (mvpOrderedConnectedKeyFrames.empty())
            return result;

        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w,
                                               KeyFrame::weightComp);
        if (it == mvOrderedWeights.end())
            return vector<KeyFrame *>();
        else {
            int n = it - mvOrderedWeights.begin();
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
        }
    }

    int KeyFrame::GetWeight(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexConnections);
        if (mConnectedKeyFrameWeights.count(pKF))
            return mConnectedKeyFrameWeights[pKF];
        else
            return 0;
    }

    void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx, bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexFeatures);

        if (idx >= mvpMapPoints.size()) {
            warn("AddMapPoint idx {} gte mvpMapPoints.size() {} ", idx, mvpMapPoints.size());
            return;
        }
        if (bAddUpdate) {
            mpMap->AddUpdate(new KeyFrameUpdate<std::pair<unsigned long, size_t> >(this->mnId,__func__,
                                                                                   std::make_pair<unsigned long, size_t>((unsigned long) (pMP->mnId), size_t(idx))
            ));
        }

        if (pMP->isBad()) {
            debug("kf {} AddMapPoint MP {} is bad idx {}", mnId, pMP->mnId, idx);
        }

        mvpMapPoints[idx] = pMP;
    }

    void KeyFrame::EraseMapPointMatch(const size_t &idx, bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexFeatures);

        if (idx >= mvpMapPoints.size()) {
            warn("EraseMapPointMatch idx {} gte mvpMapPoints.size() {} ", idx, mvpMapPoints.size());
            return;
        }

        if (bAddUpdate) {
            mpMap->AddUpdate(new KeyFrameUpdate<size_t>(this->mnId, __func__, size_t(idx)));
        }
        mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }

    void KeyFrame::EraseMapPointMatch(MapPoint *pMP, bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexFeatures);

        int idx = pMP->GetIndexInKeyFrame(this);
        if (idx < mvpMapPoints.size() && idx > -1) {
            if (bAddUpdate) {
                mpMap->AddUpdate(new KeyFrameUpdate<size_t>(this->mnId, __func__, size_t(idx)));
            }

            mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
        } else {
            if (!pMP->isBad()) {
                warn("EraseMapPointMatch idx {} gte mvpMapPoints.size() {} ", idx, mvpMapPoints.size());
            }
        }
    }


    void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP, bool bAddUpdate) {
        unique_lock<mutex> lock(mMutexFeatures);

        if (idx >= mvpMapPoints.size()) {
            if (idx == -1ul) return;

            warn("ReplaceMapPointMatch idx {} gte mvpMapPoints.size() {} ", idx, mvpMapPoints.size());
            return;
        }

        auto id = pMP ? pMP->mnId : -1ul;
        if (bAddUpdate) {
            mpMap->AddUpdate(new KeyFrameUpdate<std::pair<unsigned long, size_t> >(
                    this->mnId,
                    __func__,
                    std::make_pair<unsigned long, size_t>((unsigned long) (id), size_t(idx))
            ));
        }

        // TODO(halcao): Invalid write of size 8
        mvpMapPoints[idx] = pMP;
    }

    set<MapPoint *> KeyFrame::GetMapPoints() {
        unique_lock<mutex> lock(mMutexFeatures);
        set<MapPoint *> s;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
            if (!mvpMapPoints[i])
                continue;
            MapPoint *pMP = mvpMapPoints[i];
            if (!pMP->isBad())
                s.insert(pMP);
        }
        return s;
    }

    int KeyFrame::TrackedMapPoints(const int &minObs) {
        unique_lock<mutex> lock(mMutexFeatures);

        int nPoints = 0;
        const bool bCheckObs = minObs > 0;
        for (int i = 0; i < N; i++) {
            MapPoint *pMP = mvpMapPoints[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    if (bCheckObs) {
                        if (mvpMapPoints[i]->Observations() >= minObs)
                            nPoints++;
                    } else
                        nPoints++;
                }
            }
        }

        return nPoints;
    }

    vector<MapPoint *> KeyFrame::GetMapPointMatches() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints;
    }

    MapPoint *KeyFrame::GetMapPoint(const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        if (idx >= mvpMapPoints.size()) {
            warn("GetMapPoint idx {} gte mvpMapPoints.size() {} ", idx, mvpMapPoints.size());
            return static_cast<MapPoint *>(nullptr);
        }
        return mvpMapPoints[idx];
    }

    void KeyFrame::UpdateConnections(bool bAddUpdate) {
        if (bAddUpdate) {
            mpMap->AddUpdate(new KeyFrameUpdate<int>(this->mnId, __func__, 0));
        }

        map<KeyFrame *, int> KFcounter;

        vector<MapPoint *> vpMP;

        {
            unique_lock<mutex> lockMPs(mMutexFeatures);
            vpMP = mvpMapPoints;
        }

        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;

            if (!pMP)
                continue;

            if (pMP->isBad())
                continue;

            map<KeyFrame *, size_t> observations = pMP->GetObservations();

            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
                if (mit->first->mnId == mnId)
                    continue;
                KFcounter[mit->first]++;
            }
        }

        // This should not happen
        if (KFcounter.empty())
            return;

        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        int nmax = 0;
        KeyFrame *pKFmax = NULL;
        int th = 15;

        vector<pair<int, KeyFrame *> > vPairs;
        vPairs.reserve(KFcounter.size());
        for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
            if (mit->second > nmax) {
                nmax = mit->second;
                pKFmax = mit->first;
            }
            if (mit->second >= th) {
                vPairs.push_back(make_pair(mit->second, mit->first));
                (mit->first)->AddConnection(this, mit->second, false);
            }
        }

        if (!pKFmax) {
            warn("keyframe {} has no pKFmax", mnId);
            return;
        }

        if (vPairs.empty()) {
            vPairs.push_back(make_pair(nmax, pKFmax));
            pKFmax->AddConnection(this, nmax, false);
        }

        sort(vPairs.begin(), vPairs.end());
        list<KeyFrame *> lKFs;
        list<int> lWs;
        for (size_t i = 0; i < vPairs.size(); i++) {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            unique_lock<mutex> lockCon(mMutexConnections);

            // mspConnectedKeyFrames = spConnectedKeyFrames;
            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

            if (mbFirstConnection && isFirst() == false) {
                mpParent = mvpOrderedConnectedKeyFrames.front();
                if (mpParent) {
                    auto peers = mpParent->GetChilds();
                    if (peers.find(this) == peers.end()) {
                        debug("{} will add child this->mnId {}", mpParent->mnId, this->mnId);
                        mpParent->AddChild(this);
                        mbFirstConnection = false;
                    }
                }
            }
        }
    }

    void KeyFrame::AddChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        if (pKF->mnId == mnId) {
            error("child id equals to self");
            return;
        }
        if (mpParent && pKF->mnId == mpParent->mnId) {
            error("child id equals to self");
            return;
        }

        debug("{} AddChild {}", this->mnId, pKF->mnId);
        mspChildrens.insert(pKF);
    }

    void KeyFrame::EraseChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.erase(pKF);
    }

    void KeyFrame::ChangeParent(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        // FIXME(halcao): pKF sometimes can be itself
        if (pKF->mnId == mnId) {
            error("parent id equals to self");
            return;
        }
        mpParent = pKF;
        pKF->AddChild(this);
    }

    set<KeyFrame *> KeyFrame::GetChilds() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens;
    }

    KeyFrame *KeyFrame::GetParent() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mpParent;
    }

    bool KeyFrame::hasChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens.count(pKF);
    }

    void KeyFrame::AddLoopEdge(KeyFrame *pKF, bool bAddUpdate) {
        unique_lock<mutex> lockCon(mMutexConnections);

        if (bAddUpdate) {
            mpMap->AddUpdate(new KeyFrameUpdate<unsigned long>(
                    this->mnId,
                    __func__,
                    pKF->mnId));
        }

        mbNotErase = true;
        mspLoopEdges.insert(pKF);
    }

    set<KeyFrame *> KeyFrame::GetLoopEdges() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspLoopEdges;
    }

    void KeyFrame::SetNotErase() {
        unique_lock<mutex> lock(mMutexConnections);
        mbNotErase = true;
    }

    void KeyFrame::SetErase() {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mspLoopEdges.empty()) {
                mbNotErase = false;
            }
        }

        if (mbToBeErased) {
            SetBadFlag();
        }
    }

    void KeyFrame::SetBadFlag(bool bAddUpdate) {
        {
            unique_lock<mutex> lock(mMutexConnections);

            if (bAddUpdate) {
                mpMap->AddUpdate(new KeyFrameUpdate<int>(this->mnId, __func__, 0));
            }

            if (isFirst()) {
                return;
            } else if (mbNotErase) {
                mbToBeErased = true;
                return;
            }
        }

        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
             mit != mend; mit++)
            mit->first->EraseConnection(this);


        for (auto &mp: mvpMapPoints) {
            if (mp) {
                mp->EraseObservation(this, false);
            }
        }

        {
            unique_lock<mutex> lock(mMutexConnections);
            unique_lock<mutex> lock1(mMutexFeatures);

            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            // Update Spanning Tree
            set<KeyFrame *> sParentCandidates;
            if (mpParent) {
                sParentCandidates.insert(mpParent);
            }

            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            while (!mspChildrens.empty() && !sParentCandidates.empty()) {
                bool bContinue = false;

                int max = -1;
                KeyFrame *pC;
                KeyFrame *pP;

                for (auto pKF: mspChildrens) {
                    if (pKF || pKF->isBad())
                        continue;

                    // Check if a parent candidate is connected to the keyframe
                    vector<KeyFrame *> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
                        for (set<KeyFrame *>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end();
                             spcit != spcend; spcit++) {
                            if (vpConnected[i]->mnId == (*spcit)->mnId) {
                                int w = pKF->GetWeight(vpConnected[i]);
                                if (w > max) {
                                    // find child with max weight
                                    pC = pKF;
                                    pP = vpConnected[i];
                                    max = w;
                                    bContinue = true;
                                }
                            }
                        }
                    }
                }

                if (bContinue) {
                    pC->ChangeParent(pP);
                    debug("{} will add child {}", pP->mnId, pC->mnId);
                    sParentCandidates.insert(pC);
                    mspChildrens.erase(pC);
                    debug("{} remove children {}", this->mnId, pC->mnId);
                } else {
                    break;
                }
            }

            if (mpParent) {
                mpParent->EraseChild(this);
                mTcp = Tcw * mpParent->GetPoseInverse();

                // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
                if (!mspChildrens.empty()) {
                    for (set<KeyFrame *>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {
                        (*sit)->ChangeParent(mpParent);
                        debug("{} remove children {}", this->mnId, (*sit)->mnId);
                    }
                }

                mbToBeSerialized = false;
            } else {
                // still need to send to remote
                mbToBeSerialized = true;
            }

            mbBad = true;
        }

        mpMap->EraseKeyFrame(this);
        mpKeyFrameDB->erase(this);
    }

    bool KeyFrame::isBad() {
        unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }

    void KeyFrame::EraseConnection(KeyFrame *pKF) {
        if (!pKF) return;

        bool bUpdate = false;
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mConnectedKeyFrameWeights.count(pKF)) {
                mConnectedKeyFrameWeights.erase(pKF);
                bUpdate = true;
            }
        }

        if (bUpdate)
            UpdateBestCovisibles();
    }

    vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int) floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= mnGridCols)
            return vIndices;

        const int nMaxCellX = min((int) mnGridCols - 1, (int) ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int) floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= mnGridRows)
            return vIndices;

        const int nMaxCellY = min((int) mnGridRows - 1, (int) ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
                const vector<size_t> vCell = mGrid[ix][iy];
                for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool KeyFrame::IsInImage(const float &x, const float &y) const {
        return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
    }

    cv::Mat KeyFrame::UnprojectStereo(int i) {
        const float z = mvDepth[i];
        if (z > 0) {
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

            unique_lock<mutex> lock(mMutexPose);
            return Twc.rowRange(0, 3).colRange(0, 3) * x3Dc + Twc.rowRange(0, 3).col(3);
        } else
            return cv::Mat();
    }

    float KeyFrame::ComputeSceneMedianDepth(const int q) {
        vector<MapPoint *> vpMapPoints;
        cv::Mat Tcw_;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = mvpMapPoints;
            Tcw_ = Tcw.clone();
        }

        vector<float> vDepths;
        vDepths.reserve(N);
        cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
        Rcw2 = Rcw2.t();
        float zcw = Tcw_.at<float>(2, 3);
        for (int i = 0; i < N; i++) {
            if (mvpMapPoints[i]) {
                MapPoint *pMP = mvpMapPoints[i];
                cv::Mat x3Dw = pMP->GetWorldPos();
                float z = Rcw2.dot(x3Dw) + zcw;
                vDepths.push_back(z);
            }
        }

        sort(vDepths.begin(), vDepths.end());

        return vDepths[(vDepths.size() - 1) / q];
    }

    double KeyFrame::GetMinCovisibilityScore() {
        const vector<KeyFrame *> vpConnectedKeyFrames = GetVectorCovisibleKeyFrames();
        const DBoW2::BowVector &CurrentBowVec = mBowVec;
        double minScore = 1;
        for (auto &pKF: vpConnectedKeyFrames) {
            if (!pKF || pKF->isBad())
                continue;
            const DBoW2::BowVector &BowVec = pKF->mBowVec;

            double score = mpKeyFrameDB->mpVoc->score(CurrentBowVec, BowVec);

            if (score < minScore)
                minScore = score;
        }
        return minScore;
    }

    void KeyFrame::SetupSerializationVariable() {
        {
            // replace mvpMapPoints
            unique_lock<mutex> lock_feature(mMutexFeatures);
            mvnMapPointIds.clear();
            for (auto &mp: mvpMapPoints) {
                mvnMapPointIds.push_back((mp && !mp->isBad()) ? mp->mnId : -1ul);
                if (mp && mp->isBad()) {
                    debug("{} observation {} is bad", mnId, mp->mnId);
                    if (mbBad) {
                        debug("I am kf {}, I am bad too", mnId);
                    }
                }
            }
        }

        {
            unique_lock<mutex> lock_connection(mMutexConnections);
            // replace mConnectedKeyFrameIdWeights
            mConnectedKeyFrameIdWeights.clear();
            for (auto &itr: mConnectedKeyFrameWeights) {
                if (itr.first) {
                    mConnectedKeyFrameIdWeights[itr.first->mnId] = itr.second;
                }
            }

            // replace mvpOrderedConnectedKeyFrames
            mvnOrderedConnectedKeyFrameIds.clear();
            for (auto &kf: mvpOrderedConnectedKeyFrames) {
                mvnOrderedConnectedKeyFrameIds.push_back(kf ? kf->mnId : -1ul);
            }

            // replace mpParent
            if (mpParent) {
                mnParentId = mpParent->mnId;
            } else {
                mnParentId = -1ul;
            }

            // replace mspChildrens
            msnChildrenIds.clear();
            for (auto &kf: mspChildrens) {
                trace("{} record child {}", this->mnId, kf->mnId);
                msnChildrenIds.insert(kf->mnId);
            }

            // replace mspLoopEdges
            msnLoopEdgeIds.clear();
            for (auto &kf: mspLoopEdges) {
                msnLoopEdgeIds.insert(kf->mnId);
            }

        }
    }

    void KeyFrame::RestoreSerialization() {
        /**
         * KeyFrame
         * 3. replace mvpMapPoints to mvnMapPointIds
         * 4. replace mConnectedKeyFrameWeights to mvnOrderedConnectedKeyFrameIds
         * 5. replace mpParent to mnParentId
         * 6. replace mspChildrens to msnChildrenIds
         * 7. replace mspLoopEdges to msnLoopEdgeIds
         */

        { // lock feature
            unique_lock<mutex> lock_feature(mMutexFeatures);
            mvpMapPoints.clear();
            for (auto &id: mvnMapPointIds) {
                auto mp = mpMap->GetMapPoint(id);
                mvpMapPoints.push_back(mp);
                if (!mp && id != -1ul) {
                    mpMap->restorationKFQueue.insert(this->mnId);
                    debug("keyframe {} mappoint {} null", mnId, id);
                }
            }
        } // end lock feature

        { // lock connection
            unique_lock<mutex> lock_connection(mMutexConnections);

            mspChildrens.clear();
            for (auto &id: msnChildrenIds) {
                auto kf = mpMap->GetKeyFrame(id);
                mspChildrens.insert(kf);
                trace("{} restore child {}", this->mnId, kf->mnId);
                if (!kf && id != -1ul) {
                    mpMap->restorationKFQueue.insert(this->mnId);
                    warn("keyframe {} child {} null", mnId, id);
                }
            }

            mspLoopEdges.clear();
            for (auto &id: msnLoopEdgeIds) {
                auto kf = mpMap->GetKeyFrame(id);
                mspLoopEdges.insert(kf);
                if (!kf && id != -1ul) {
                    mpMap->restorationKFQueue.insert(this->mnId);
                    debug("keyframe {} loop edge {} null", mnId, id);
                }
            }

            mpParent = mpMap->GetKeyFrame(mnParentId);
        } // end lock connection

        // Update global pose
//        UpdateGlobalPose();

        /**
         * This function uses `mvpMapPoints` to update:
         * - mConnectedKeyFrameWeights
         * - mvpOrderedConnectedKeyFrames
         * - mvOrderedWeights
         * - mpParent
         */
        UpdateConnections(false);

        if (isFirst() == false && !mpParent && mnParentId != -1ul) {
            debug("keyframe {} parent {} null", mnId, mnParentId);
        }
    }

    void KeyFrame::SetFirst(bool bFirst) {
        unique_lock<mutex> lock(mMutexFirst);
        this->mbFirst = bFirst;
    }

    unsigned long KeyFrame::GetOriginMapId() {
        return mnId / MAP_BASE;
    }

    void KeyFrame::AssignFeaturesToGrid() {
        int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid[i][j].reserve(nReserve);

        for(int i=0;i<N;i++)
        {
            const cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }

    bool KeyFrame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
        posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            return false;

        return true;
    }

} //namespace ORB_SLAM
