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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <Thirdparty/DBoW2/DBoW2/BowVector.h>
#include <Thirdparty/DBoW2/DBoW2/FeatureVector.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <mutex>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(); // Default constructor for serialization, need to deal with const member

    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    KeyFrame(Map* pMap, KeyFrameDatabase* pKeyFrameDB, ORBextractor* pORBextractor, cv::Mat Scw, cv::Mat K, vector<cv::KeyPoint>& vKeysUn, cv::Mat Descriptors, vector<MapPoint* >& vpMapPoints);

    // Pose functions
    void SetPose(const cv::Mat &Tcw, bool bAddUpdate=true);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight, bool bAddUpdate=true);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections(bool bAddUpdate=true);
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF, bool bAddUpdate=true);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx, bool bAddUpdate=true);
    void EraseMapPointMatch(const size_t &idx, bool bAddUpdate=true);
    void EraseMapPointMatch(MapPoint* pMP, bool bAddUpdate=true);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP, bool bAddUpdate=true);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag(bool bAddUpdate=true);
    bool isBad();

    // Is the first keyframe id
    inline bool isFirst() {
        unique_lock<mutex> lock(mMutexFirst);
        return mbFirst;
    }

    unsigned long GetOriginMapId();

    void SetFirst(bool bFirst);

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    cv::Mat GetGlobalPose();
    cv::Mat GetGlobalPoseInverse();
    cv::Mat GetGlobalCameraCenter();
    cv::Mat GetGlobalTranslation();
    cv::Mat GetGlobalRotation();

    // The following variables are accesed from only 1 thread or never change (no mutex needed).

    // added: grid feature assignment in synthesized frame
    void AssignFeaturesToGrid();
    // added: Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
public:

    static map<unsigned long, unsigned long> nNextIds;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;
    const double mCreatedTime;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;



    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
//    long unsigned int mnBALocalForKF;
//    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    // originally const and not static
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    // originally const
    int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    // distorted key points
    const std::vector<cv::KeyPoint> mvKeys;

    // undistorted
    // originally const: mvDepth, mDescriptors
    std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    std::vector<float> mvDepth; // negative value for monocular points
    cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale (originally const)
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;

    // score of importance
    double mScore;

    // whether is synthesized
    bool isGenuine = true;

// The following variables need to be accessed trough a mutex to be thread safe.
protected:
    // SE3 Pose and camera center
    cv::Mat mGlobalTcw;
    cv::Mat mGlobalTwc;
    cv::Mat mGlobalOw;

    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;
    std::vector<unsigned long> mvnMapPointIds;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    std::map<unsigned long, int> mConnectedKeyFrameIdWeights;
    std::vector<unsigned long> mvnOrderedConnectedKeyFrameIds;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection = true;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    unsigned long mnParentId = -1ul;
    std::set<unsigned long> msnChildrenIds;
    std::set<unsigned long> msnLoopEdgeIds;


    // Bad flags
    bool mbNotErase = false;
    bool mbToBeErased = false;
    bool mbBad = false;

    // first flag
    bool mbFirst = false;
    mutex mMutexFirst;

    float mHalfBaseline; // Only for visualization

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;

//public:
//    void SetORBvocabulary(const ORBVocabulary *porbv) { mpORBvocabulary = porbv;}

public:
    // The reference to map
    Map* mpMap;
        // TODO: mutex needed?
    // flag whether the object is to be serialized
    bool mbToBeSerialized = true;

    void SetupSerializationVariable();

    void RestoreSerialization();

    double GetMinCovisibilityScore();

    void UpdateGlobalPose(bool bLock=true);

    void SetGlobalPose(const cv::Mat &globalTcw_);

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, __attribute__((unused)) const unsigned int version) {
        ar & mnId;
        ar & const_cast<long unsigned int &>(mnFrameId);
        ar & const_cast<double &>(mTimeStamp);
        ar & const_cast<double &>(mCreatedTime);

        // Grid related vars
        ar & const_cast<int &>(mnGridCols);
        ar & const_cast<int &>(mnGridRows);
        ar & const_cast<float &>(mfGridElementWidthInv);
        ar & const_cast<float &>(mfGridElementHeightInv);

        // Tracking related vars
        ar & mnTrackReferenceForFrame & mnFuseTargetForKF;

        ar & mnLoopQuery & mnLoopWords & mLoopScore & mnRelocQuery &
        mnRelocWords & mRelocScore;

        // calibration parameters
        ar & const_cast<float &>(fx) & const_cast<float &>(fy) &
        const_cast<float &>(cx) & const_cast<float &>(cy);
        ar & const_cast<float &>(invfx) & const_cast<float &>(invfy) &
        const_cast<float &>(mbf);
        ar & const_cast<float &>(mb) & const_cast<float &>(mThDepth);

        // Number of KeyPoints;
        ar & const_cast<int &>(N);

        // KeyPoints, stereo coordinate and descriptors
        ar & const_cast<std::vector<cv::KeyPoint> &>(mvKeys);
        ar & const_cast<std::vector<cv::KeyPoint> &>(mvKeysUn);
        ar & const_cast<std::vector<float> &>(mvuRight);
        ar & const_cast<std::vector<float> &>(mvDepth);
        ar & const_cast<cv::Mat &>(mDescriptors);

        // Bow
//        ar & mBowVec & mFeatVec;

        // Pose relative to parent
        ar & mTcp;

        // Scale related
        ar & const_cast<int &>(mnScaleLevels) & const_cast<float &>(mfScaleFactor) &
        const_cast<float &>(mfLogScaleFactor);
        ar & const_cast<std::vector<float> &>(mvScaleFactors) &
        const_cast<std::vector<float> &>(mvLevelSigma2) &
        const_cast<std::vector<float> &>(mvInvLevelSigma2);

        // Image bounds and calibration
        ar & const_cast<int &>(mnMinX) & const_cast<int &>(mnMinY) &
        const_cast<int &>(mnMaxX) & const_cast<int &>(mnMaxY);
        ar & const_cast<cv::Mat &>(mK);

        // mutex needed vars, but don't lock mutex in the save/load procedure
        {
            unique_lock<mutex> lock_pose(mMutexPose);
            ar & Tcw & Twc & Ow & Cw;
            ar & mGlobalTcw & mGlobalTwc & mGlobalOw;
        }
        {
            unique_lock<mutex> lock_feature(mMutexFeatures);
            ar & mvnMapPointIds;
        }

        {
            // Grid related
            unique_lock<mutex> lock_connection(mMutexConnections);
            ar & mGrid;
            ar & mConnectedKeyFrameIdWeights;
            ar & mvnOrderedConnectedKeyFrameIds;
            ar & mvOrderedWeights;
            // Spanning Tree and Loop Edges
            ar & mbFirstConnection;
            ar & mnParentId;
            ar & msnChildrenIds;
            ar & msnLoopEdgeIds;
            // Bad flags
            ar & mbNotErase & mbToBeErased & mbBad & mHalfBaseline;
        }

        {
            unique_lock<mutex> lock(mMutexFirst);
            ar & mbFirst;
        }

        // only serialized once
        if (Archive::is_saving::value) {
            // if it is serialization, make sure that the structure would not be serialized again
            mbToBeSerialized = false;
        } else {
            // if it is deserialization, make sure that the structure could be serialized again
            // for map storage or distribution to other agent
            mbToBeSerialized = true;
        }
    }
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
