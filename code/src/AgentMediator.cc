//
// Created by Halcao on 2020/5/14.
//

#include <ORBVocabulary.h>
#include <boost/archive/binary_oarchive.hpp>

#include "Optimizer.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "AgentMediator.h"
#include "MapUpdater.h"
#include "BoostArchiver.h"
#include "ORBmatcher.h"
#include "PnPsolver.h"
#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "MapManager.h"
#include "Mapit.h"
#include "MapEnhancer.h"
#include "MapSlice.h"
#include "MediatorScheduler.h"
//#include "SystemState.h"
#include <CLogger.h>

#include <utility>

namespace ORB_SLAM2 {
id_t AgentMediator::nNextId = 0;
unordered_map<id_t, KeyFrameDatabase *> AgentMediator::databaseMap;


AgentMediator::AgentMediator(const string &strSettingsFile, ORBVocabulary *pVoc, const bool bUseViewer,
                             const bool isGlobal_) : mbGlobal(isGlobal_) {
    mpKeyFrameDatabase = new KeyFrameDatabase(*pVoc);

    // the map is in the mediator
    mpMap = new Map(true);

    MapManager::Register(mpMap);

    // Client Mediator has the same id with its map
    mnId = mpMap->mnId;

    databaseMap[mnId] = mpKeyFrameDatabase;


    // monocular so fixedScale = false
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, pVoc, false);
    mptLoopClosing = new thread(&LoopClosing::Run, mpLoopCloser);

    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
    // system and tracking null
    mpViewer = new Viewer(nullptr, mpFrameDrawer, mpMapDrawer, nullptr, strSettingsFile);
    if (bUseViewer) {
        auto id = mbGlobal ? "main" : to_string(mpMap->mnId / 2);
        mpViewer->SetTitle("Server Map Viewer " + id, "Server Frame Viewer " + id);

        mptViewer = new thread(&Viewer::Run, mpViewer);
    }

    MediatorScheduler::GetInstance().RegisterMediator(this);
}

void AgentMediator::Shutdown() {
    debug("wait pLoopCloser to finish");
    mpLoopCloser->RequestFinish();
    // Wait until all thread have effectively stopped
    while (!mpLoopCloser->isFinished()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (mptLoopClosing && mptLoopClosing->joinable()) mptLoopClosing->join();

    if (mptViewer) {
        info("wait pViewer to finish");
        mpViewer->RequestFinish();
        while (!mpViewer->isFinished()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (mptViewer->joinable()) {
            mptViewer->join();
        }
    }
}

void AgentMediator::SaveMap(const string &filename) {
    std::ofstream out(filename, std::ios_base::binary);
    if (!out) {
        error("Cannot Write to Mapfile: {}", filename);
        exit(-1);
    }
    info("Saving Mapfile: {}", filename);
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    for (auto &itr: mpMap->GetKeyFrameMaps()) {
        if (!itr.second) continue;
        itr.second->SetupSerializationVariable();
    }

    for (auto &itr: mpMap->GetMapPointMaps()) {
        itr.second->SetupSerializationVariable();
    }

    if (isGlobal()) {
        MergeKeyFrameDatabases();
    } else {
        mpKeyFrameDatabase->SetupSerializationVariable();
    }

    oa << mpMap;
    oa << mpKeyFrameDatabase;
    info("...done");
    out.close();
}

void AgentMediator::MergeKeyFrameDatabases() {
    // merge the other kfdb
    mpKeyFrameDatabase->mvInvertedFileId.clear();
    for (auto &pair: databaseMap) {
        auto db = pair.second;
        if (db == mpKeyFrameDatabase) {
            continue;
        }

        if (mpKeyFrameDatabase->mvInvertedFileId.empty()) {
            mpKeyFrameDatabase->mvInvertedFileId.resize(db->mvInvertedFile.size());
        }

        for (size_t i = 0; i < db->mvInvertedFile.size(); ++i) {
            auto &list = db->mvInvertedFile[i];
            for (auto &kf: list) {
                auto id = kf ? kf->mnId : -1ul;
                mpKeyFrameDatabase->mvInvertedFileId[i].push_back(id);
            }
        }
    }
}

void AgentMediator::CheckOverlapCandidates(const AgentMediator *other) {
    auto vpKFs = other->mpMap->allKFs;
    auto vpMPs = other->mpMap->allMPs;
    // init variables
    if (lastCheckedKFIds.count(other->mpMap->mnId) == 0) lastCheckedKFIds[other->mpMap->mnId] = 0;
    if (lastCheckedMPIds.count(other->mpMap->mnId) == 0) lastCheckedMPIds[other->mpMap->mnId] = 0;

    // Get last tracked id
    auto lastCheckedKFId = lastCheckedKFIds[other->mpMap->mnId];
    auto lastCheckedMPId = lastCheckedMPIds[other->mpMap->mnId];

    for (auto &itr: vpMPs) {
        auto mp = itr.second;
        // if tracked, ignore it
        if (!mp || mp->mnId <= lastCheckedMPId) continue;
        lastCheckedMPId = max(lastCheckedMPId, mp->mnId);

        if (!mp->isBad()) {
            mpMap->AddMapPoint(mp);
        } else {
            mpMap->RegisterMapPoint(mp);
        }
    }
    lastCheckedMPIds[other->mpMap->mnId] = lastCheckedMPId;

    for (auto &itr: vpKFs) {
        auto kf = itr.second;
        // if tracked, ignore it
        if (!kf || kf->mnId <= lastCheckedKFId) continue;
        lastCheckedKFId = max(lastCheckedKFId, kf->mnId);

        if (!kf->isBad()) {
            mpMap->AddKeyFrame(kf);
        } else {
            mpMap->RegisterKeyFrame(kf);
        }

        auto minScore = kf->GetMinCovisibilityScore();

        set<KeyFrame *> candidates;
        for (auto &pair: AgentMediator::databaseMap) {
            if (pair.first == mnId) continue;
            auto kfDB = pair.second;

            auto kfs = kfDB->DetectLoopCandidates(kf, minScore);
            for (auto &candidate: kfs) {
                // not in the same map
                if (candidate->GetOriginMapId() != kf->GetOriginMapId()) {
                    candidates.insert(candidate);
                }
            }
        }

        debug("Global check map {} kf {} candidates count: {}", kf->mpMap->mnId, kf->mnId, candidates.size());

        if (DetectLoop(kf, minScore, candidates)) {
            AgentMediator::GetSim3(kf, mvpEnoughConsistentCandidatesMap[kf->mpMap->mnId]);
        }
//            mpKeyFrameDatabase->add(kf);
    }

    lastCheckedKFIds[other->mpMap->mnId] = lastCheckedKFId;
}

void AgentMediator::GetSim3(KeyFrame *pCurrentKF, vector<KeyFrame *> candidates) {
    const auto nInitialCandidates = candidates.size();
    const bool mbFixScale = false;

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75, true);

    vector<Sim3Solver *> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint *> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);
    // map points of pCurrentKF
    vector<MapPoint *> vpCurrentMapPoints = pCurrentKF->GetMapPointMatches();

    // currentKF from world
    cv::Mat R1w = pCurrentKF->GetRotation();
    cv::Mat t1w = pCurrentKF->GetTranslation();


    size_t nCandidates = 0; //candidates with enough matches

    for (size_t i = 0; i < nInitialCandidates; i++) {
        KeyFrame *pKF = candidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if (pKF->isBad()) {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = 0;

        // if pKF is base map and pCurrentKF's map is merged
//            if (pKF->mpMap->mbBaseMap && pCurrentKF->mpMap->mbMerged) {
//                vvpMapPointMatches[i] = vector<MapPoint*>(pCurrentKF->GetMapPointMatches().size(), nullptr);
//                auto vpSrcMPs = pKF->GetMapPointMatches();
//                auto vpMapPoints = vector<MapPoint *>();
//                std::copy_if(vpSrcMPs.begin(), vpSrcMPs.end(), std::back_inserter(vpMapPoints), [](MapPoint *pMP){ return pMP != nullptr; });
//                nmatches = matcher.SearchByProjection(pCurrentKF, pCurrentKF->GetGlobalPose(), vpMapPoints, vvpMapPointMatches[i], 10);
//            } else {
        // vvpMapPointMatches[i] stores cancidates[i]'s mappoints that match current keyframe's mappoints, stored in the same id
        nmatches = matcher.SearchByBoW(pCurrentKF, pKF, vvpMapPointMatches[i]);

//            }


        debug("kf {} and {} has {} matches", pCurrentKF->mnId, pKF->mnId, nmatches);

        if (nmatches < 20) {
            vbDiscarded[i] = true;
            continue;
        } else {
            Sim3Solver *pSolver = new Sim3Solver(pCurrentKF, pKF, vvpMapPointMatches[i], mbFixScale);
            pSolver->SetRansacParameters(0.99, 20, 300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

//        if (nCandidates > 0) {
    debug("candidate total count: {}, nCandidates: {} with enough matches", nInitialCandidates, nCandidates);
//        }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is successful or all fail
    while (nCandidates > 0 && !bMatch) {
        for (size_t i = 0; i < nInitialCandidates; i++) {
            if (vbDiscarded[i])
                continue;

            KeyFrame *pKF = candidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver *pSolver = vpSim3Solvers[i];
            cv::Mat Scm = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if (!Scm.empty() && nInliers > 20) {
                debug("RANSAC Get a Sim3");
                vector<MapPoint *> vpMapPointMatches(vvpMapPointMatches[i].size(), nullptr);
                for (size_t j = 0, jend = vbInliers.size(); j < jend; j++) {
                    if (vbInliers[j])
                        vpMapPointMatches[j] = vvpMapPointMatches[i][j];
                }

                // get inliner mappoints, perform umeyama
                //Camera 2 from world
                cv::Mat R2w = pKF->GetRotation();
                cv::Mat t2w = pKF->GetTranslation();
                Eigen::Matrix3Xd srcPoints(3, nInliers), destPoints(3, nInliers);
                int pos = 0;
                for (size_t j = 0, jend = vbInliers.size(); j < jend; j++) {
                    if (vbInliers[j]) {
                        cv::Mat sPos = vpMapPointMatches[j]->GetWorldPos(), dPos = vpCurrentMapPoints[j]->GetWorldPos();
                        // need to transfer from world pos to camera pos
                        sPos = R2w * sPos + t2w;
                        dPos = R1w * dPos + t1w;
                        srcPoints.col(pos) << sPos.at<float>(0), sPos.at<float>(1), sPos.at<float>(2);
                        destPoints.col(pos) << dPos.at<float>(0), dPos.at<float>(1), dPos.at<float>(2);
                        pos++;
                    }
                }
                cv::Mat UmeR, Umet;
                double umeScale;

                UmeyamaForSim3Transform(srcPoints, destPoints, UmeR, Umet, umeScale);

                // Umeyama done
                // B=sRA+t
//                cv::Mat R = pSolver->GetEstimatedRotation();
//                cv::Mat t = pSolver->GetEstimatedTranslation();
//                const float s = pSolver->GetEstimatedScale();
//                    matcher.SearchBySim3(pCurrentKF, pKF, vpMapPointMatches, s, R, t, 7.5);
                matcher.SearchBySim3(pCurrentKF, pKF, vpMapPointMatches, umeScale, UmeR, Umet, 7.5);

                // gScm transform pKF to pCurrentKF
//                    g2o::Sim3 gScm(Converter::toMatrix3d(R), Converter::toVector3d(t), s);
                g2o::Sim3 gScm(Converter::toMatrix3d(UmeR), Converter::toVector3d(Umet), umeScale);
                nInliers = Optimizer::OptimizeSim3(pCurrentKF, pKF, vpMapPointMatches, gScm, 10,
                                                   mbFixScale);

                // If optimization is successful stop RANSACs and continue
                debug("nInliers {}", nInliers);
                if (nInliers >= 40) {
//                    if (nInliers >= 20) {
                    bMatch = true;

                    g2o::Sim3 T12 = g2o::Sim3(Converter::toMatrix3d(pCurrentKF->GetRotation()),
                                              Converter::toVector3d(pCurrentKF->GetTranslation()),
                                              1.0).inverse() * gScm *
                                    g2o::Sim3(Converter::toMatrix3d(pKF->GetRotation()),
                                              Converter::toVector3d(pKF->GetTranslation()), 1.0);
                    cout << "Merge map Sim3 scale " << T12 << endl;

                    info("map {} kf {} and map {} kf {} has a sim3", pCurrentKF->mpMap->mnId, pCurrentKF->mnId,
                         pKF->mpMap->mnId, pKF->mnId);
                    info("Start to Merge Map");

                    // cancel not erase
                    pCurrentKF->SetErase();
                    for (auto &pKF: candidates) {
                        pKF->SetErase();
                    }
//                    MapManager::MergeMap(pCurrentKF->mpMap, pKF->mpMap, T12);
                    Mapit::Merge(pCurrentKF->mpMap, pKF->mpMap, T12);
                    SegmentMaps(pCurrentKF, pKF);

                    break;
                }
            }
        }
    }

    pCurrentKF->SetErase();
    for (auto &pKF: candidates) {
        pKF->SetErase();
    }
    if (!bMatch) return;
}

bool AgentMediator::DetectLoop(KeyFrame *mpCurrentKF, float minScore, const set<KeyFrame *> &vpCandidateKFs) {
    int mnCovisibilityConsistencyTh = 3;

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame *> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

    // If there are no loop candidates, just add new keyframe and return false
    if (vpCandidateKFs.empty()) {
        mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId].clear();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidatesMap[mpCurrentKF->mpMap->mnId].clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId].size(), false);
    for (auto &pCandidateKF: vpCandidateKFs) {
        set<KeyFrame *> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for (size_t iG = 0, iendG = mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId].size(); iG < iendG; iG++) {
            set<KeyFrame *> sPreviousGroup = mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId][iG].first;

            bool bConsistent = false;
            for (set<KeyFrame *>::iterator sit = spCandidateGroup.begin(), send = spCandidateGroup.end();
                 sit != send; sit++) {
                if (sPreviousGroup.count(*sit)) {
                    bConsistent = true;
                    bConsistentForSomeGroup = true;
                    break;
                }
            }

            if (bConsistent) {
                int nPreviousConsistency = mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId][iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if (!vbConsistentGroup[iG]) {
                    ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG] = true; //this avoid to include the same group more than once
                }
                if (nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent) {
                    mvpEnoughConsistentCandidatesMap[mpCurrentKF->mpMap->mnId].push_back(pCandidateKF);
                    bEnoughConsistent = true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if (!bConsistentForSomeGroup) {
            ConsistentGroup cg = make_pair(spCandidateGroup, 0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroupsMap[mpCurrentKF->mpMap->mnId] = vCurrentConsistentGroups;

    debug("loop candidates - after / before {} / {}",
          mvpEnoughConsistentCandidatesMap[mpCurrentKF->mpMap->mnId].size(), vpCandidateKFs.size());

    if (mvpEnoughConsistentCandidatesMap[mpCurrentKF->mpMap->mnId].empty()) { return false; }

    return true;
}

void AgentMediator::RunGlobalBundleAdjustment(Map *pMap1, Map *pMap2) {
    bool bStopGBA = false;
    unsigned long nLoopKF = 0;

    vector<KeyFrame *> vpKF1 = pMap1->GetAllKeyFrames();
    vector<MapPoint *> vpMP1 = pMap1->GetAllMapPoints();

    vector<KeyFrame *> vpKF2 = pMap2->GetAllKeyFrames();
    vector<MapPoint *> vpMP2 = pMap2->GetAllMapPoints();

    vpKF1.insert(vpKF1.end(), vpKF2.begin(), vpKF2.end());
    vpMP1.insert(vpMP1.end(), vpMP2.begin(), vpMP2.end());


    info("Start BundleAdjustment");
    Optimizer::BundleAdjustment(vpKF1, vpMP1, 10, &bStopGBA, nLoopKF, false, true);
}

void
AgentMediator::UmeyamaForSim3Transform(const Eigen::Matrix3Xd& srcPoses, Eigen::Matrix3Xd destPoses, cv::Mat &mR, cv::Mat &mt,
                                       double &ds) {

    mR = cv::Mat::eye(3, 3, CV_32F);
    mt = cv::Mat::zeros(3, 1, CV_32F);

    Eigen::Matrix<double, 4, 4> mRt = Eigen::umeyama(srcPoses, destPoses, true), mRf = Eigen::umeyama(srcPoses,
                                                                                                      destPoses,
                                                                                                      false);
//        cout << mRt << endl;
//        cout << mRf << endl;
    Eigen::Matrix3d eR = mRf.block(0, 0, 3, 3);
    Eigen::Vector3d et = mRt.block(0, 3, 3, 1);

    ds = mRt.block(0, 0, 3, 3).sum() / eR.sum();
//        cout << ds << endl;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mR.at<float>(i, j) = eR(i, j);
        }
    }
    for (int i = 0; i < 3; i++) {
        mt.at<float>(i, 0) = et(i);
    }

}

void AgentMediator::SetCurrentLocation(const cv::Mat &currentLocation) {
    if (currentLocation.empty()) return;

    auto globalLoc = currentLocation * Converter::toCvMat(mpMap->mTwl.inverse()) * mpMap->mTwl.scale();
    mpMapDrawer->SetCurrentCameraPose(globalLoc);
}

SystemState AgentMediator::GetState() const {
    return mState;
}

void AgentMediator::SetState(SystemState state) {
    if (!mState.location.empty()) {
        this->mLastState = mState;
    }
    AgentMediator::mState = std::move(state);
    this->SetCurrentLocation(mState.location);
}

// static function
void AgentMediator::SegmentMaps(KeyFrame* pKF, KeyFrame *pRefKF) {
    if (pKF->mpMap->mnId == pRefKF->mpMap->mnId) {
        // the two KeyFrames comes from the same map
        if (pKF->mnId > pRefKF->mnId) {
            // perform inner map segmentation, make sure the two KeyFrames' ids are in ascending order
            std::swap(pKF, pRefKF);
        }
    }

    const auto mediator1 = MediatorScheduler::GetInstance().GetMediator(pKF->mpMap->mnId);
    if (mediator1 != nullptr) {
        mediator1->SegmentMapByKeyFrame(pKF);
    }

    const auto mediator2 = MediatorScheduler::GetInstance().GetMediator(pRefKF->mpMap->mnId);
    if (mediator2 != nullptr) {
        mediator2->SegmentMapByKeyFrame(pRefKF);
    }
}

void AgentMediator::SegmentMapByKeyFrame(const KeyFrame *pKF) {
    // make a slice from lastKFid to pKF id

    if (pKF->mnId < this->mnLastKFId) return;

    auto KFs = pKF->mpMap->GetAllKeyFrames();

    if (KFs.size() < 2) return;

    // sort KFs by its id
    sort(KFs.begin(), KFs.end(), [](KeyFrame * const& lhs, KeyFrame * const& rhs) {
        return lhs->mnId < rhs->mnId;
    });

//    if (this->mnLastKFId == 0) this->mnLastKFId = KFs.front()->mnId;

    const auto lastKFId = this->mnLastKFId;

    // find the first KeyFrame id that is not in the slice
    auto begin = find_if(KFs.begin(), KFs.end(), [lastKFId](const KeyFrame *kf) {
        return kf->mnId > lastKFId;
    });

    if (begin == KFs.end()) return;

    // find the pKF index in the array
    auto end = find(begin, KFs.end(), pKF);

    if (end == KFs.end()) return;

    const int MIN_SEG_LENGTH = 8;

    if (begin != KFs.end() && end - begin >= MIN_SEG_LENGTH) {
        MapSlice s;
        s.KFs = vector<KeyFrame *>(begin, end);
        // TODO(halcao): visualize this slice
        this->mvMapSegments.push_back(s);

        // get average score of the slice


        this->mnLastKFId = (*end)->mnId;
        info("Map segmentation: slice from {} to {}", (*begin)->mnId, (*end)->mnId);
    }
}
}
