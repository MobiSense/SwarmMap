//
// Created by zjl on 11/4/2020.
//

#include "DynamicRunner.h"

using namespace std;
using namespace cv;

namespace ORB_SLAM2 {

    DynamicRunner *DynamicRunner::getInstance()
    {
        static DynamicRunner instance;
        return &instance;
    }

    void DynamicRunner::Initialize(const string &strModelPath, int maxUsage, bool useOpticalFlow, float confThreshold, float maskThreshold, int batchSize)
    {
        if(mMaskExtractor == NULL)
        {
            mMaskExtractor = new DynamicExtractor(strModelPath, maxUsage, useOpticalFlow, confThreshold, maskThreshold);
            mBatchSize = batchSize;
        }
    }

    void DynamicRunner::Run()
    {
        while (true)
        {
            if(CheckFinish())
            {
                info("runner's job done");
                break;
            }
            ResetIfRequested();
//            if(!mlKeyFrames.empty())
            if(!mqImageQueue.empty())
            {
//                KeyFrame* pKF = mlKeyFrames.front();
                Mat imGray;
                KeyFrame * pKF;
                {
                    unique_lock<mutex> lock(mMutexQueue);
                    imGray = mqImageQueue.top().second;
                    pKF = mqImageQueue.top().first;
                    mqImageQueue.pop();
                    info("runner now deal with keyframe {}", pKF->mnFrameId);
//                    cout << "runner now deal with keyframe " << pKF->mnFrameId << endl;
                }


//                int batchSize = mBatchSize <= mqImageQueue.size() ? mBatchSize : mqImageQueue.size();
//                vector<Mat> imGrays, masks;
//                for (int i=0; i<batchSize; i++)
//                {
//                    imGrays.push_back(mqImageQueue.top().second);
//                    mqImageQueue.pop();
//                }

                cv::Mat mask = cv::Mat(imGray.size(), CV_8U, cv::Scalar(255));
                mMaskExtractor->extractMaskDirect(imGray, mask);

                int dynamic_cnt = 0;
                vector<MapPoint*> mpPoints = pKF->GetMapPointMatches();
                for (int i=0; i<pKF->N; i++)
                {
                    if(mpPoints[i]==NULL)
                    {
                        continue;
                    }
                    cv::KeyPoint kp = pKF->mvKeys[i];
                    vector<int> & voteQueue = mpPoints[i]->mDynamicVoteQueue;
                    int x = (int)kp.pt.x, y = (int)kp.pt.y;
                    if(x > mask.size[1] || y > mask.size[0])
                        continue;
                    if(mask.at<uchar>(y, x)!=0)
                    {
                        // static vote
                        voteQueue.push_back(0);
                    }
                    else{
                        // dynamic vote
                        voteQueue.push_back(1);
                    }
                    if(count(voteQueue.begin(), voteQueue.end(), 1) >= 0.9 * count(voteQueue.begin(), voteQueue.end(), 0))
                    {
                        mpPoints[i]->mDynamic = 1;
                        dynamic_cnt++;
                    }
                    else{
                        mpPoints[i]->mDynamic = 0;
                    }
                }
                mlKeyFrames.pop_front();
            }
        }
    }

    void DynamicRunner::InsertKeyFrame(KeyFrame *pKF)
    {
        mlKeyFrames.push_back(pKF);
    }

    void DynamicRunner::RequestFinish()
    {
        mbFinishRequested = true;
    }

    bool DynamicRunner::CheckFinish()
    {
        return mbFinishRequested;
    }

    void DynamicRunner::RequestReset()
    {

        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        while(1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if(!mbResetRequested)
                    break;
            }
            usleep(1000);
        }
    }

    void DynamicRunner::ResetIfRequested()
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            mlKeyFrames.clear();
            mbResetRequested = false;
            info("runner has now {} key frames", mqImageQueue.size());
//            cout << "runner has now " << mlKeyFrames.size() << "key frames" << endl;
        }
    }

    void DynamicRunner::InsertImage(KeyFrame *pKF, cv::Mat imGray)
    {
//        mqImageQueue.push(make_pair(pKF, imGray));
        unique_lock<mutex> lock(mMutexQueue);
        mqImageQueue.emplace(pKF, imGray);
    }


}