//
// Created by zjl on 11/4/2020.
//

#ifndef EDGE_SLAM_DYNAMICRUNNER_H
#define EDGE_SLAM_DYNAMICRUNNER_H

#include "DynamicExtractor.h"
#include "KeyFrame.h"
#include "CLogger.h"
#include <mutex>
#include <unistd.h>
#include <queue>



namespace ORB_SLAM2 {

    typedef pair<KeyFrame*, cv::Mat> imKFPair;

    struct PairCmp{
        bool operator() (imKFPair& a, imKFPair& b)
        {
            return a.first->mScore < b.first->mScore;
        }
    };

    class DynamicRunner {
    public:
        static DynamicRunner * getInstance();

        void Run();
        void Initialize(const std::string &strModelPath, int maxUsage=1, bool useOpticalFlow=false,
                          float confThreshold = 0.5, float maskThreshold = 0.3, int batchSize = 8);
        void InsertKeyFrame(KeyFrame* pKF);
        void InsertImage(KeyFrame* pKF, cv::Mat imGray);
        void RequestFinish();
        bool CheckFinish();
        void RequestReset();
        void ResetIfRequested();

    protected:
        int mBatchSize;
        bool mbFinishRequested;
        bool mbResetRequested;
        std::mutex mMutexReset;
        std::mutex mMutexQueue;
        DynamicExtractor* mMaskExtractor;
        // KeyFrame pending to extract mask
        std::list<KeyFrame*> mlKeyFrames;
        std::priority_queue<imKFPair, vector<imKFPair>, PairCmp> mqImageQueue;
//        std::priority_queue<pair<KeyFrame*, cv::Mat>> mqImageQueue;
        DynamicRunner(){}
    };
}

#endif //EDGE_SLAM_DYNAMICRUNNER_H
