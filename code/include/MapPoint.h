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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "Frame.h"
#include "Map.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <opencv2/core/core.hpp>
#include <mutex>
#include <vector>


namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
    typedef unsigned long id_t;
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);
    MapPoint():
            mnId(-1ul), nObs(0), mbTrackInView(false), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame *>(NULL)), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0)
    {
    }

    void SetWorldPos(const cv::Mat &Pos, bool bAddUpdate=true);
    cv::Mat GetWorldPos();
    cv::Mat GetGlobalPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*, size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF, size_t idx, bool bAddUpdate=true);
    void EraseObservation(KeyFrame* pKF, bool bAddUpdate=true);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag(bool bAddUpdate=true);
    bool isBad();

    void Replace(MapPoint* pMP, bool bAddUpdate=true);
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1, bool bAddUpdate=true);
    void IncreaseFound(int n=1, bool bAddUpdate=true);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors(bool bAddUpdate=true);

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth(bool bAddUpdate=true);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, const float &logScaleFactor, const int mnScaleLevel);

    static bool lId(MapPoint* pMP1, MapPoint* pMP2){
        return pMP1->mnId<pMP2->mnId;
    }
public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX = 0;
    float mTrackProjY = 0;
    float mTrackProjXR = 0;
    bool mbTrackInView = false;
    int mnTrackScaleLevel = 0;
    float mTrackViewCos = 0;
    long unsigned int mnTrackReferenceForFrame = 0;
    long unsigned int mnLastFrameSeen = 0;

    // Variables used by local mapping
//    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

    // Varaibles used by dynamic runner, stands for dynamic state
    std::vector<int> mDynamicVoteQueue;
    // 0 for static, 1 for dynamic
    int mDynamic;


protected:
     // Position in absolute coordinates
     cv::Mat mWorldPos;
     cv::Mat mGlobalPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame *, size_t> mObservations;

     std::map<unsigned long, size_t> mIdObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;
    // -1 stands for null
    unsigned long mnRefKFId = -1ul;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad = false;
     MapPoint* mpReplaced;
     // -1 stands for null
     unsigned long mnReplacedId = -1ul;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
     // TODO: mutex needed?
     // flag whether the object is to be serialized
     double mLastTrackedTime = 0;
public:
    // The reference to map
    Map* mpMap;
    double mTimeStamp = 0;

    void SetLastTrackedTime(double time, bool bAddUpdate=true);
    double GetLastTrackedTime() const;

    mutable bool mbToBeSerialized = true;

    void SetSerialized(bool flag) const {
        this->mbToBeSerialized = flag;
    }

    void SetupSerializationVariable();

    void RestoreSerialization();

    void UpdateGlobalPos(bool bLock=true);

    void SetGlobalPos(const cv::Mat &globalPos);

    void SetFound(int n);

    void SetVisible(int n);

    unsigned long GetOriginMapId() const;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, __attribute__((unused))  const unsigned int version) {
        ar & mnId & mnFirstKFid & mnFirstFrame & nObs;

//        // Tracking related vars
        ar & mTrackProjX;
        ar & mTrackProjY;
        ar & mTrackProjXR;
        ar & mbTrackInView;
        ar & mnTrackScaleLevel;
        ar & mTrackViewCos;
        ar & mnTrackReferenceForFrame;
        ar & mnLastFrameSeen;

        // tracked time
        ar & mTimeStamp;
        ar & mLastTrackedTime;

        ar & mnFuseCandidateForKF;

        // don't save the mutex
        {
            unique_lock<mutex> lock_pose(mMutexPos);
            ar & mWorldPos;
            ar & mGlobalPos;
            ar & mNormalVector;
            ar & mfMinDistance & mfMaxDistance;
        }

        {
            unique_lock<mutex> lock(mMutexFeatures);
            ar & mIdObservations;
            ar & mDescriptor;

            ar & mnRefKFId;
            ar & mnVisible & mnFound;
            ar & nObs;

            unique_lock<mutex> lock2(mMutexPos);
            ar & mbBad;
            ar & mnReplacedId;
        }

        if (Archive::is_saving::value) {
            // only serialized once
            SetSerialized(false);
        } else {
            // only serialized once
            SetSerialized(true);
        }
    }

    int GetVisible();
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
