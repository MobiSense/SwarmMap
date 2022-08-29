//
// Created by Halcao on 2020/5/14.
//

#ifndef EDGE_SLAM_AGENTMEDIATOR_H
#define EDGE_SLAM_AGENTMEDIATOR_H

#include <string>
#include <thread>
#include <set>
#include <unordered_map>
#include <unordered_map>
#include <Eigen/Core>
#include <ORBVocabulary.h>
#include "SystemState.h"

namespace ORB_SLAM2 {

using std::string;
class Viewer;
class FrameDrawer;
class MapDrawer;
class KeyFrame;
class Map;
class MapSlice;
class LoopClosing;
class KeyFrameDatabase;
enum TrackingState: int;
//struct SystemState;

class AgentMediator {
public:

    typedef pair<set<KeyFrame *>, int> ConsistentGroup;

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    AgentMediator(const string &strSettingsFile, ORBVocabulary *pVoc, const bool isGlobal_ = false, const bool bUseViewer = true, const bool bUseMapViewer = true);

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    void SaveMap(const string &filename);
    void CheckOverlapCandidates(const AgentMediator *other);
    inline Map *GetMap() {
        return mpMap;
    }

    static void RunGlobalBundleAdjustment(Map *pMap1, Map* pMap2);
    id_t mnId;

    void SetCurrentLocation(const cv::Mat &mCurrentLocation);

    SystemState GetState() const;

    void SetState(SystemState mState);

    bool isGlobal() const {
        return mbGlobal;
    }

    static void SegmentMaps(KeyFrame *pKF, KeyFrame *pRefKF);

private:
    static id_t nNextId;
    static unordered_map<id_t, KeyFrameDatabase *> databaseMap;

    SystemState mState;
    SystemState mLastState;

    cv::Mat lastLocation;
    cv::Mat lastVelocity;
    double averageSpeed = 0;

    // if its global client
    bool mbGlobal;
    // SLAM Map
    Map* mpMap;
    vector<MapSlice> mvMapSegments;
    vector<double> mvSegmentScores;
    // the id of the last kf in the latest map slice
    id_t mnLastKFId = 0;


    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Viewer* mpViewer;
    KeyFrameDatabase* mpKeyFrameDatabase;

    // threads
    std::thread* mptLoopClosing;
    std::thread* mptViewer = nullptr;

    // last loop closure checked ids
    std::unordered_map<unsigned long, unsigned long> lastCheckedKFIds;
    std::unordered_map<unsigned long, unsigned long> lastCheckedMPIds;

    // used for loop detection
    std::unordered_map<unsigned long, std::vector<ConsistentGroup> > mvConsistentGroupsMap;
    std::unordered_map<unsigned long, std::vector<KeyFrame*> > mvpEnoughConsistentCandidatesMap;

    static void GetSim3(KeyFrame *pCurrentKF, vector<KeyFrame *> candidates);

    bool DetectLoop(KeyFrame *mpCurrentKF, float minScore, const set<KeyFrame *>& vpCandidateKFs);

    static void UmeyamaForSim3Transform(const Eigen::Matrix3Xd& srcPoses, Eigen::Matrix3Xd destPoses, cv::Mat &mR, cv::Mat &mt, double &ds);

    void MergeKeyFrameDatabases();

    void SegmentMapByKeyFrame(const KeyFrame *pKF);
};
};

#endif //EDGE_SLAM_AGENTMEDIATOR_H
