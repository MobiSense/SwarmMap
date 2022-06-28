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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>
#include <CLogger.h>
#include <ORBmatcher.h>
#include <g2o/EdgeSim3RelativeXYZ.h>

namespace ORB_SLAM2 {

    void Optimizer::GlobalBundleAdjustment(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                           const bool bRobust) {
        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
        BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
    }


    void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                     int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                     const bool bRobust, const bool bGlobal) {
        vector<bool> vbNotIncludedMP;
        vbNotIncludedMP.resize(vpMP.size());

        g2o::SparseOptimizer optimizer;
        auto linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        auto solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        long unsigned int maxKFid = 0;

        // Set KeyFrame vertices
        for (auto pKF : vpKFs) {
            if (pKF->isBad())
                continue;
            auto vSE3 = new g2o::VertexSE3Expmap();
            auto pose = bGlobal ? pKF->GetGlobalPose() : pKF->GetPose();
            vSE3->setEstimate(Converter::toSE3Quat(pose));
            vSE3->setId(pKF->mnId);
            vSE3->setFixed(pKF->isFirst());
            optimizer.addVertex(vSE3);
            if (pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;
        }

        const float thHuber2D = sqrt(5.99);
        const float thHuber3D = sqrt(7.815);

        // Set MapPoint vertices
        for (size_t i = 0; i < vpMP.size(); i++) {
            MapPoint *pMP = vpMP[i];
            if (pMP->isBad())
                continue;

            auto vPoint = new g2o::VertexSBAPointXYZ();
            auto pos = bGlobal ? pMP->GetGlobalPos() : pMP->GetWorldPos();
            vPoint->setEstimate(Converter::toVector3d(pos));
            auto id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            int nEdges = 0;
            //SET EDGES
            for (auto &mit: observations) {

                KeyFrame *pKF = mit.first;
                if (pKF->isBad() || pKF->mnId > maxKFid)
                    continue;

                nEdges++;

                const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit.second];

                if (pKF->mvuRight[mit.second] < 0) {
                    // monocular
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    auto e = new g2o::EdgeSE3ProjectXYZ();

                    auto v0 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id));
                    auto v1 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId));

                    if (!v0 || !v1) {
                        debug("bundle adjustment null v0: {}, v1: {}", id, pKF->mnId);
                        continue;
                    }
                    e->setVertex(0, v0);
                    e->setVertex(1, v1);
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    if (bRobust) {
                        auto rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;

                    optimizer.addEdge(e);
                } else {
                    // stereo
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKF->mvuRight[mit.second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    auto e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    if (bRobust) {
                        auto rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber3D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;
                    e->bf = pKF->mbf;

                    optimizer.addEdge(e);
                }
            }

            if (nEdges == 0) {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMP[i] = true;
            } else {
                vbNotIncludedMP[i] = false;
            }
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);

        // Recover optimized data

        //Keyframes
        for (auto pKF: vpKFs) {
            if (pKF->isBad())
                continue;
            auto vSE3 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
            const g2o::SE3Quat &SE3quat = vSE3->estimate();
            if (nLoopKF == 0) {
                if (bGlobal) {
                    pKF->SetGlobalPose(Converter::toCvMat(SE3quat));
                } else {
                    pKF->SetPose(Converter::toCvMat(SE3quat));
                }
            } else {
                pKF->mTcwGBA.create(4, 4, CV_32F);
                Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
                pKF->mnBAGlobalForKF = nLoopKF;
            }
        }

        //Points
        for (size_t i = 0; i < vpMP.size(); i++) {
            if (vbNotIncludedMP[i])
                continue;

            MapPoint *pMP = vpMP[i];

            if (pMP->isBad())
                continue;
            auto vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                    pMP->mnId + maxKFid + 1));

            if (nLoopKF == 0) {
                if (bGlobal) {
                    pMP->SetGlobalPos(Converter::toCvMat(vPoint->estimate()));
                } else {
                    pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
                }
                pMP->UpdateNormalAndDepth();
            } else {
                pMP->mPosGBA.create(3, 1, CV_32F);
                Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
                pMP->mnBAGlobalForKF = nLoopKF;
            }
        }

    }

    int Optimizer::PoseOptimization(Frame *pFrame, const bool bGlobal) {
        g2o::SparseOptimizer optimizer;

        auto linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        auto solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences = 0;

        // Set Frame vertex
        auto vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        // Set MapPoint vertices
        const int N = pFrame->N;

        vector<g2o::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
        vector<size_t> vnIndexEdgeMono;
        vpEdgesMono.reserve(N);
        vnIndexEdgeMono.reserve(N);

        vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesStereo.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        const float deltaMono = sqrt(5.991);
        const float deltaStereo = sqrt(7.815);


        {
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for (int i = 0; i < N; i++) {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (!pMP) continue;

                // Monocular observation
                if (pFrame->mvuRight[i] < 0) {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double, 2, 1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    obs << kpUn.pt.x, kpUn.pt.y;

                    auto e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    auto rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
                    cv::Mat Xw = bGlobal ? pMP->GetGlobalPos() : pMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                } else  // Stereo observation
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    //SET EDGE
                    Eigen::Matrix<double, 3, 1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    const float &kp_ur = pFrame->mvuRight[i];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaStereo);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
                    e->bf = pFrame->mbf;
                    cv::Mat Xw = bGlobal ? pMP->GetGlobalPos() : pMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }

            }
        }


        if (nInitialCorrespondences < 3)
            return 0;

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
        const int its[4] = {10, 10, 10, 10};

        int nBad = 0;
        for (size_t it = 0; it < 4; it++) {

            vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
            optimizer.initializeOptimization(0);
            optimizer.optimize(its[it]);

            nBad = 0;
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
                auto e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];

                if (pFrame->mvbOutlier[idx]) {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Mono[it]) {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                } else {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                }

                if (it == 2)
                    e->setRobustKernel(nullptr);
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
                auto e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (pFrame->mvbOutlier[idx]) {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Stereo[it]) {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                } else {
                    e->setLevel(0);
                    pFrame->mvbOutlier[idx] = false;
                }

                if (it == 2)
                    e->setRobustKernel(nullptr);
            }

            if (optimizer.edges().size() < 10)
                break;
        }

        // Recover optimized pose and return number of inliers
        auto vSE3_recov = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
        auto SE3quat_recov = vSE3_recov->estimate();
        cv::Mat pose = Converter::toCvMat(SE3quat_recov);
        pFrame->SetPose(pose);

        return nInitialCorrespondences - nBad;
    }

    void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap) {
        // Local KeyFrames: First Breath Search from Current Keyframe
        list<KeyFrame *> lLocalKeyFrames;

        lLocalKeyFrames.push_back(pKF);
        set<unsigned long> sLocalKeyFrameIds;
        sLocalKeyFrameIds.insert(pKF->mnId);

        const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
        for (auto pKFi: vNeighKFs) {
            sLocalKeyFrameIds.insert(pKFi->mnId);
            if (!pKFi->isBad())
                lLocalKeyFrames.push_back(pKFi);
        }

        // Local MapPoints seen in Local KeyFrames
        list<MapPoint *> lLocalMapPoints;
        set<unsigned long> sLocalMapPointIds;

        for (auto &lit: lLocalKeyFrames) {
            vector<MapPoint *> vpMPs = lit->GetMapPointMatches();
            for (auto pMP: vpMPs) {
                if (!pMP || pMP->isBad()) continue;

                if (sLocalMapPointIds.count(pMP->mnId) == 0) {
                    lLocalMapPoints.push_back(pMP);
                    sLocalMapPointIds.insert(pMP->mnId);
                }
            }
        }

        // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
        list<KeyFrame *> lFixedCameras;
        set<unsigned long> sFixedKeyFrameIds;
        for (auto &lit: lLocalMapPoints) {
            map<KeyFrame *, size_t> observations = lit->GetObservations();
            for (auto &mit: observations) {
                auto pKFi = mit.first;
                if (sLocalKeyFrameIds.count(pKFi->mnId) == 0 && sFixedKeyFrameIds.count(pKFi->mnId) == 0) {
                    sFixedKeyFrameIds.insert(pKFi->mnId);

                    if (pKFi->isBad()) continue;

                    lFixedCameras.push_back(pKFi);
                }
            }
        }

        // Setup optimizer
        g2o::SparseOptimizer optimizer;

        auto linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        auto solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        unsigned long maxKFid = 0;

        // Set Local KeyFrame vertices
        for (auto &pKFi: lLocalKeyFrames) {
            auto vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(pKFi->isFirst());
            optimizer.addVertex(vSE3);

            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }

        // Set Fixed KeyFrame vertices
        for (auto &pKFi: lFixedCameras) {
            auto vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);

            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }

        // Set MapPoint vertices
        auto nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

        vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const float thHuberMono = sqrt(5.991);
        const float thHuberStereo = sqrt(7.815);

        for (auto &pMP: lLocalMapPoints) {
            auto vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            auto id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            //Set edges
            for (auto &mit: observations) {

                auto pKFi = mit.first;

                if (pKFi->isBad()) continue;

                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit.second];

                // Monocular observation
                if (pKFi->mvuRight[mit.second] < 0) {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    auto e = new g2o::EdgeSE3ProjectXYZ();

                    auto v0 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id));
                    auto v1 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId));
                    if (!v0 || !v1) {
                        warn("pointer v0 or v1 null");
                    }

                    e->setVertex(0, v0);
                    e->setVertex(1, v1);
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    auto rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                } else // Stereo observation
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKFi->mvuRight[mit.second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    auto rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }

        if (pbStopFlag)
            if (*pbStopFlag)
                return;

        optimizer.initializeOptimization();
        optimizer.optimize(5);

        bool bDoMore = true;

        if (pbStopFlag)
            if (*pbStopFlag)
                bDoMore = false;

        if (bDoMore) {

            // Check inlier observations
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
                g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
                MapPoint *pMP = vpMapPointEdgeMono[i];

                if (pMP->isBad())
                    continue;

                if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                    e->setLevel(1);
                }

                e->setRobustKernel(nullptr);
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
                auto e = vpEdgesStereo[i];
                MapPoint *pMP = vpMapPointEdgeStereo[i];

                if (pMP->isBad())
                    continue;

                if (e->chi2() > 7.815 || !e->isDepthPositive()) {
                    e->setLevel(1);
                }

                e->setRobustKernel(nullptr);
            }

            // Optimize again without the outliers

            optimizer.initializeOptimization(0);
            optimizer.optimize(10);

        }

        vector<pair<KeyFrame *, MapPoint *> > vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.emplace_back(pKFi, pMP);
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive()) {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.emplace_back(pKFi, pMP);
            }
        }

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        if (!vToErase.empty()) {
            for (auto &itr : vToErase) {
                KeyFrame *pKFi = itr.first;
                MapPoint *pMPi = itr.second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        // Recover optimized data

        //Keyframes
        for (auto &pKFi: lLocalKeyFrames) {
            auto vSE3 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
            const g2o::SE3Quat &SE3quat = vSE3->estimate();
            pKFi->SetPose(Converter::toCvMat(SE3quat));
        }

        //Points
        for (auto &pMP: lLocalMapPoints) {
            auto vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                    pMP->mnId + maxKFid + 1));
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
    }


    void Optimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                           const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                           const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                           const bool &bFixScale) {
        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        auto linearSolver =
                new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
        auto solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
        auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        solver->setUserLambdaInit(1e-16);
        optimizer.setAlgorithm(solver);

        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

        map<unsigned long, g2o::Sim3> vScw;
        map<unsigned long, g2o::Sim3> vCorrectedSwc;

        const int minFeat = 100;

        // Set KeyFrame vertices
        for (auto &pKF: vpKFs) {
            if (pKF->isBad())
                continue;
            auto VSim3 = new g2o::VertexSim3Expmap();

            auto nIDi = pKF->mnId;

            auto it = CorrectedSim3.find(pKF);

            if (it != CorrectedSim3.end()) {
                vScw[nIDi] = it->second;
                VSim3->setEstimate(it->second);
            } else {
                Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
                Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
                g2o::Sim3 Siw(Rcw, tcw, 1.0);
                vScw[nIDi] = Siw;
                VSim3->setEstimate(Siw);
            }

            if (pKF == pLoopKF)
                VSim3->setFixed(true);

            VSim3->setId(nIDi);
            VSim3->setMarginalized(false);
            VSim3->_fix_scale = bFixScale;

            optimizer.addVertex(VSim3);
        }


        set<pair<long unsigned int, long unsigned int> > sInsertedEdges;

        const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

        // Set Loop edges
        for (const auto &loopConnection : LoopConnections) {
            KeyFrame *pKF = loopConnection.first;
            const auto nIDi = pKF->mnId;
            const set<KeyFrame *> &spConnections = loopConnection.second;
            const g2o::Sim3 Siw = vScw[nIDi];
            const g2o::Sim3 Swi = Siw.inverse();

            for (auto sit : spConnections) {
                const auto nIDj = sit->mnId;
                if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(sit) < minFeat)
                    continue;

                const g2o::Sim3 Sjw = vScw[nIDj];
                const g2o::Sim3 Sji = Sjw * Swi;

                auto e = new g2o::EdgeSim3();
                auto v1 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj));
                auto v0 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi));
                if (v1 == nullptr || v0 == nullptr) {
                    // TODO(halcao): find reason why the variables could be null
                    warn("v1 {}, v0 {} null", nIDj, nIDi);
                    continue;
                }
//                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
//                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                e->setVertex(1, v1);
                e->setVertex(0, v0);
                e->setMeasurement(Sji);

                e->information() = matLambda;

                optimizer.addEdge(e);

                sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
            }
        }

        // Set normal edges
        for (auto pKF : vpKFs) {
            auto nIDi = pKF->mnId;

            g2o::Sim3 Swi;

            auto iti = NonCorrectedSim3.find(pKF);

            if (iti != NonCorrectedSim3.end())
                Swi = (iti->second).inverse();
            else
                Swi = vScw[nIDi].inverse();

            KeyFrame *pParentKF = pKF->GetParent();

            // Spanning tree edge
            if (pParentKF) {
                auto nIDj = pParentKF->mnId;

                g2o::Sim3 Sjw;

                auto itj = NonCorrectedSim3.find(pParentKF);

                if (itj != NonCorrectedSim3.end())
                    Sjw = itj->second;
                else
                    Sjw = vScw[nIDj];

                g2o::Sim3 Sji = Sjw * Swi;

                auto e = new g2o::EdgeSim3();
                auto v1 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj));
                auto v0 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi));
                if (v1 == nullptr || v0 == nullptr) {
                    // TODO(halcao): find reason why the variables could be null
                    warn("v1 {}, v0 {} null", nIDj, nIDi);
                    continue;
                }
                e->setVertex(1, v1);
                e->setVertex(0, v0);
                e->setMeasurement(Sji);

                e->information() = matLambda;
                optimizer.addEdge(e);
            }

            // Loop edges
            const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
            for (auto pLKF : sLoopEdges) {
                if (pLKF->mnId < pKF->mnId) {
                    g2o::Sim3 Slw;

                    auto itl = NonCorrectedSim3.find(pLKF);

                    if (itl != NonCorrectedSim3.end())
                        Slw = itl->second;
                    else
                        Slw = vScw[pLKF->mnId];

                    g2o::Sim3 Sli = Slw * Swi;
                    auto el = new g2o::EdgeSim3();
                    el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
                    el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                    el->setMeasurement(Sli);
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                }
            }

            // Covisibility graph edges
            const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
            for (const auto pKFn: vpConnectedKFs) {
                if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
                    if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
                        if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId), max(pKF->mnId, pKFn->mnId))))
                            continue;

                        g2o::Sim3 Snw;

                        auto itn = NonCorrectedSim3.find(pKFn);

                        if (itn != NonCorrectedSim3.end())
                            Snw = itn->second;
                        else
                            Snw = vScw[pKFn->mnId];

                        g2o::Sim3 Sni = Snw * Swi;

                        auto en = new g2o::EdgeSim3();
                        auto v1 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId));
                        auto v0 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi));
                        if (!v0 || !v1) {
                            warn("v0 {} or v1 {} null", nIDi, pKFn->mnId);
                            continue;
                        }
                        en->setVertex(1, v1);
                        en->setVertex(0, v0);
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                    }
                }
            }
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(20);

        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
        for (const auto &pKFi: vpKFs) {
            auto nIDi = pKFi->mnId;

            auto VSim3 = dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
            g2o::Sim3 CorrectedSiw = VSim3->estimate();
            vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
            Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = CorrectedSiw.translation();
            double s = CorrectedSiw.scale();

            eigt *= (1. / s); //[R t/s;0 1]

            cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

            pKFi->SetPose(Tiw);
        }

        // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
        for (auto &pMP: vpMPs) {
            if (pMP->isBad())
                continue;

            unsigned long nIDr;
            if (pMP->mnCorrectedByKF == pCurKF->mnId) {
                nIDr = pMP->mnCorrectedReference;
            } else {
                KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
                nIDr = pRefKF->mnId;
            }


            g2o::Sim3 Srw = vScw[nIDr];
            g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

            cv::Mat P3Dw = pMP->GetWorldPos();
            Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
            Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMP->SetWorldPos(cvCorrectedP3Dw);

            pMP->UpdateNormalAndDepth();
        }
    }

    int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12,
                                const float th2, const bool bFixScale) {
        g2o::SparseOptimizer optimizer;

        auto linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        auto solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // Calibration
        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;

        // Camera poses
        const cv::Mat R1w = pKF1->GetRotation();
        const cv::Mat t1w = pKF1->GetTranslation();
        const cv::Mat R2w = pKF2->GetRotation();
        const cv::Mat t2w = pKF2->GetTranslation();

        // Set Sim3 vertex
        auto vSim3 = new g2o::VertexSim3Expmap();
        vSim3->_fix_scale = bFixScale;
        vSim3->setEstimate(g2oS12);
        vSim3->setId(0);
        vSim3->setFixed(false);
        vSim3->_principle_point1[0] = K1.at<float>(0, 2);
        vSim3->_principle_point1[1] = K1.at<float>(1, 2);
        vSim3->_focal_length1[0] = K1.at<float>(0, 0);
        vSim3->_focal_length1[1] = K1.at<float>(1, 1);
        vSim3->_principle_point2[0] = K2.at<float>(0, 2);
        vSim3->_principle_point2[1] = K2.at<float>(1, 2);
        vSim3->_focal_length2[0] = K2.at<float>(0, 0);
        vSim3->_focal_length2[1] = K2.at<float>(1, 1);
        optimizer.addVertex(vSim3);

        // Set MapPoint vertices
        const int N = vpMatches1.size();
        const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
        vector<g2o::EdgeSim3ProjectXYZ *> vpEdges12;
        vector<g2o::EdgeInverseSim3ProjectXYZ *> vpEdges21;
        vector<size_t> vnIndexEdge;

        vnIndexEdge.reserve(2 * N);
        vpEdges12.reserve(2 * N);
        vpEdges21.reserve(2 * N);

        const float deltaHuber = sqrt(th2);

        int nCorrespondences = 0;

        for (int i = 0; i < N; i++) {
            if (!vpMatches1[i])
                continue;

            MapPoint *pMP1 = vpMapPoints1[i];
            MapPoint *pMP2 = vpMatches1[i];

            const int id1 = 2 * i + 1;
            const int id2 = 2 * (i + 1);

            auto i2 = pMP2->GetIndexInKeyFrame(pKF2);

            if (!pMP1 || !pMP2) continue;

            if (pMP1->isBad() || pMP2->isBad() || i2 < 0) continue;

            auto vPoint1 = new g2o::VertexSBAPointXYZ();
            cv::Mat P3D1w = pMP1->GetWorldPos();
            cv::Mat P3D1c = R1w * P3D1w + t1w;
            vPoint1->setEstimate(Converter::toVector3d(P3D1c));
            vPoint1->setId(id1);
            vPoint1->setFixed(true);
            optimizer.addVertex(vPoint1);

            auto vPoint2 = new g2o::VertexSBAPointXYZ();
            cv::Mat P3D2w = pMP2->GetWorldPos();
            cv::Mat P3D2c = R2w * P3D2w + t2w;
            vPoint2->setEstimate(Converter::toVector3d(P3D2c));
            vPoint2->setId(id2);
            vPoint2->setFixed(true);
            optimizer.addVertex(vPoint2);

            nCorrespondences++;

            // Set edge x1 = S12*X2
            Eigen::Matrix<double, 2, 1> obs1;
            const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
            obs1 << kpUn1.pt.x, kpUn1.pt.y;

            auto e12 = new g2o::EdgeSim3ProjectXYZ();
            e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
            e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e12->setMeasurement(obs1);
            const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
            e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

            auto rk1 = new g2o::RobustKernelHuber;
            e12->setRobustKernel(rk1);
            rk1->setDelta(deltaHuber);
            optimizer.addEdge(e12);

            // Set edge x2 = S21*X1
            Eigen::Matrix<double, 2, 1> obs2;
            const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;

            auto e21 = new g2o::EdgeInverseSim3ProjectXYZ();

            e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
            e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e21->setMeasurement(obs2);
            float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
            e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

            auto rk2 = new g2o::RobustKernelHuber;
            e21->setRobustKernel(rk2);
            rk2->setDelta(deltaHuber);
            optimizer.addEdge(e21);

            vpEdges12.push_back(e12);
            vpEdges21.push_back(e21);
            vnIndexEdge.push_back(i);
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        // Check inliers
        int nBad = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++) {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            if (e12->chi2() > th2 || e21->chi2() > th2) {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = nullptr;
                optimizer.removeEdge(e12);
                optimizer.removeEdge(e21);
                vpEdges12[i] = nullptr;
                vpEdges21[i] = nullptr;
                nBad++;
            }
        }

        int nMoreIterations;
        if (nBad > 0)
            nMoreIterations = 10;
        else
            nMoreIterations = 5;

        if (nCorrespondences - nBad < 10)
            return 0;

        // Optimize again only with inliers

        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++) {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            if (e12->chi2() > th2 || e21->chi2() > th2) {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = nullptr;
            } else {
                nIn++;
            }
        }

        // Recover optimized Sim3
        auto vSim3_recov = dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
        g2oS12 = vSim3_recov->estimate();

        return nIn;
    }

    int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pTargetKF, const float th2, g2o::Sim3 &T12) {
        g2o::SparseOptimizer optimizer;

        auto linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        auto solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // Calibration
        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pTargetKF->mK;

        // initialize vSim3
        auto vSim3 = new g2o::VertexSim3Expmap();

        vSim3->_fix_scale = false;
        vSim3->setEstimate(T12);
        vSim3->setId(0);
        vSim3->setFixed(false);
        vSim3->_principle_point1[0] = K1.at<float>(0, 2);
        vSim3->_principle_point1[1] = K1.at<float>(1, 2);
        vSim3->_focal_length1[0] = K1.at<float>(0, 0);
        vSim3->_focal_length1[1] = K1.at<float>(1, 1);
        vSim3->_principle_point2[0] = K2.at<float>(0, 2);
        vSim3->_principle_point2[1] = K2.at<float>(1, 2);
        vSim3->_focal_length2[0] = K2.at<float>(0, 0);
        vSim3->_focal_length2[1] = K2.at<float>(1, 1);
        optimizer.addVertex(vSim3);

        // TODO(halcao): arg settings? & refactor it
        ORBmatcher matcher(0.8, true);

        int nLastId = 0;

        unsigned long baseMapId = pKF1->mpMap->mnId;
        unsigned long otherMapId = pTargetKF->mpMap->mnId;

        vector<g2o::EdgeSim3ProjectXYZ *> vpEdges12;
        vector<g2o::EdgeInverseSim3ProjectXYZ *> vpEdges21;
        vector<size_t> vnIndexEdge;

        const float deltaHuber = sqrt(th2);

        int nCorrespondences = 0;

//        vnIndexEdge.reserve(2 * N);
//        vpEdges12.reserve(2 * N);
//        vpEdges21.reserve(2 * N);

        int nKF = 0;
        int nSameMP = 0;
        const int minFeat = 100;

//        auto vpCovKF = pTargetKF->GetCovisiblesByWeight(minFeat);
//        vector<KeyFrame *> vpCovKF;
        auto vpCovKF = pTargetKF->GetVectorCovisibleKeyFrames();
//         delete kf from pKF1 map
        for (auto itr = vpCovKF.begin(); itr != vpCovKF.end(); ) {
            if ((*itr)->mpMap->mnId != otherMapId) {
                itr = vpCovKF.erase(itr);
            } else {
                ++itr;
            }
        }
//        vpCovKF.push_back(pTargetKF);

        const cv::Mat RTw = pTargetKF->GetRotation();
        const cv::Mat tTw = pTargetKF->GetTranslation();

        for (auto &pKF2: vpCovKF) {
            nKF++;
            const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
            // match
            vector<MapPoint *> vpMatches(vpMapPoints1.size(), nullptr);
            int nmatches = matcher.SearchByBoW(pKF1, pKF2, vpMatches);

            if (nmatches <= 20) continue;

            // Set MapPoint vertices
            const int N = vpMatches.size();

            // Camera poses
            const cv::Mat R1w = pKF1->GetRotation();
            const cv::Mat t1w = pKF1->GetTranslation();
            const cv::Mat R2w = pKF2->GetRotation();
            const cv::Mat t2w = pKF2->GetTranslation();

            for (int i = 0; i < N; i++) {
                if (!vpMatches[i]) continue;

                MapPoint *pMP1 = vpMapPoints1[i];
                MapPoint *pMP2 = vpMatches[i];

                if (!pMP1 || !pMP2) continue;

                if (pMP1->mpMap->mnId != baseMapId || pMP2->mpMap->mnId != otherMapId)
                    continue;

                const int id1 = nLastId + 2 * i + 1;
                const int id2 = nLastId + 2 * (i + 1);

                auto i2 = pMP2->GetIndexInKeyFrame(pKF2);

                if (pMP1->isBad() || pMP2->isBad() || i2 < 0) continue;

                auto vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w * P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                auto vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = RTw * P3D2w + tTw;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);


                nCorrespondences++;

                // Set edge x1 = S12*X2
                Eigen::Matrix<double, 2, 1> obs1;
                const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
                obs1 << kpUn1.pt.x, kpUn1.pt.y;

                auto e12 = new g2o::EdgeSim3ProjectXYZ();
                e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
                e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                e12->setMeasurement(obs1);
                const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
                e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

                auto rk1 = new g2o::RobustKernelHuber;
                e12->setRobustKernel(rk1);
                rk1->setDelta(deltaHuber);
                optimizer.addEdge(e12);

                // Set edge x2 = S21*X1
                Eigen::Matrix<double, 2, 1> obs2;
                const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
                obs2 << kpUn2.pt.x, kpUn2.pt.y;

                /*
                 * Here is the key operation, because T12 is the transform from pTargetKF -> pKF1 and can not apply to
                 * pKF2. We project the MP1 to pKF2, and transform the observation key point kpUn2 from pKF2 to pTargetKF
                 * coordinate.
                 */

                cv::Mat kp1(3, 1, CV_32F);
                kp1.at<float>(0, 0) = kpUn2.pt.x;
                kp1.at<float>(1, 0) = kpUn2.pt.y;
                kp1.at<float>(2, 0) = 1;

                cv::Mat tmp = K1.inv() * kp1;
                cv::Mat kp1New(4, 1, CV_32F);
                kp1New.at<float>(0, 0) = tmp.at<float>(0, 0);
                kp1New.at<float>(1, 0) = tmp.at<float>(1, 0);
                kp1New.at<float>(2, 0) = tmp.at<float>(2, 0);
                kp1New.at<float>(3, 0) = 1;

                cv::Mat tmp2 = pTargetKF->GetPose() * pKF2->GetPoseInverse() * kp1New;
                tmp2 = tmp2.rowRange(0, 3);
//                cv::Mat kp2 = Z1/Z2 * K2 * tmp2;
                cv::Mat kp2 = K2 * tmp2;
                kp2 /= kp2.at<float>(2, 0);

                if (pTargetKF->mnId == pKF2->mnId) {
                    debug("origin x {}, y {}", kpUn2.pt.x, kpUn2.pt.y);
                    debug("new x {}, y {}", kp2.at<float>(0, 0), kp2.at<float>(0, 1));

                    auto newPos = T12.map(Converter::toVector3d(P3D2c));
                    newPos /= newPos.z();
                    newPos = Converter::toMatrix3d(K2) * newPos;
                    debug("calc x {}, y {}, z {}", newPos.x(), newPos.y(), newPos.z());

                    ++nSameMP;
                }

                debug("origin x {}, y {}", kpUn2.pt.x, kpUn2.pt.y);
                debug("new x {}, y {}", kp2.at<float>(0, 0), kp2.at<float>(0, 1));

                auto newPos = T12.inverse().map(Converter::toVector3d(P3D1c));
                // project
                newPos /= newPos.z();
                //
                newPos = Converter::toMatrix3d(K2) * newPos;
                debug("calc x {}, y {}, z {}", newPos.x(), newPos.y(), newPos.z());


                Eigen::Matrix<double, 2, 1> newObs2;
                newObs2 << double(kp2.at<float>(0, 0)), double(kp2.at<float>(0, 1));

                auto e21 = new g2o::EdgeInverseSim3ProjectXYZ();

                e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
                e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                e21->setMeasurement(newObs2);
                float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
                e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

                auto rk2 = new g2o::RobustKernelHuber;
                e21->setRobustKernel(rk2);
                rk2->setDelta(deltaHuber);
                optimizer.addEdge(e21);

                vpEdges12.push_back(e12);
                vpEdges21.push_back(e21);
                vnIndexEdge.push_back(nLastId + i);
            }
            nLastId += 2 * N;
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        // Check inliers
        int nBad = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++) {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            auto e12chi2 = e12->chi2();
            auto e21chi2 = e21->chi2();

            if (e12chi2 > th2 || e21chi2 > th2) {
//                size_t idx = vnIndexEdge[i];
//                vpMatches[idx] = nullptr;
                optimizer.removeEdge(e12);
                optimizer.removeEdge(e21);
                vpEdges12[i] = nullptr;
                vpEdges21[i] = nullptr;
                nBad++;
            }
        }

        debug("total nKF {}, nSameMP {}", nKF, nSameMP);

        int nMoreIterations;
        if (nBad > 0)
            nMoreIterations = 10;
        else
            nMoreIterations = 5;

        if (nCorrespondences - nBad < 10) {
            debug("nCorrespondences - nBad {}", nCorrespondences - nBad);
            return 0;
        }

        // Optimize again only with inliers

        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++) {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            if (e12->chi2() > th2 || e21->chi2() > th2) {
                size_t idx = vnIndexEdge[i];
//                vpMatches1[idx] = nullptr;
            } else {
                nIn++;
            }
        }

        // Recover optimized Sim3
        auto vSim3_recov = dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
        T12 = vSim3_recov->estimate();

        debug("total inliers / nKF: {}", double(nIn) / nKF);

        return nIn;
    }


    int Optimizer::OptimizeSim3ByMapPoint(KeyFrame *pKF1, KeyFrame *pTargetKF, const float th2, g2o::Sim3 &Twl) {
        g2o::SparseOptimizer optimizer;

        auto linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        auto solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // Calibration
        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pTargetKF->mK;

        // initialize vSim3
        auto vSim3 = new g2o::VertexSim3Expmap();

        vSim3->_fix_scale = false;
        vSim3->setEstimate(Twl);
        vSim3->setId(0);
        vSim3->setFixed(false);
        vSim3->_principle_point1[0] = K1.at<float>(0, 2);
        vSim3->_principle_point1[1] = K1.at<float>(1, 2);
        vSim3->_focal_length1[0] = K1.at<float>(0, 0);
        vSim3->_focal_length1[1] = K1.at<float>(1, 1);
        vSim3->_principle_point2[0] = K2.at<float>(0, 2);
        vSim3->_principle_point2[1] = K2.at<float>(1, 2);
        vSim3->_focal_length2[0] = K2.at<float>(0, 0);
        vSim3->_focal_length2[1] = K2.at<float>(1, 1);
        optimizer.addVertex(vSim3);

        ORBmatcher matcher(0.8, true);

        unsigned long baseMapId = pKF1->mpMap->mnId;
        unsigned long otherMapId = pTargetKF->mpMap->mnId;

        vector<g2o::EdgeSim3RelativeXYZ *> vpEdges;
        const float deltaHuber = sqrt(th2);
        int nCorrespondences = 0;

        int nMP = 0;
        int nLastId = 0;

        const int minFeat = 50;
//        auto vpCovKF = pTargetKF->GetVectorCovisibleKeyFrames();
        auto vpCovKF = pTargetKF->GetCovisiblesByWeight(minFeat);

        for (auto itr = vpCovKF.begin(); itr != vpCovKF.end(); ) {
            if ((*itr)->mpMap->mnId != otherMapId) {
                itr = vpCovKF.erase(itr);
            } else {
                ++itr;
            }
        }

        for (auto &pKF2: vpCovKF) {
            auto vpMapPoints = pKF1->GetMapPointMatches();
            vector<MapPoint *> vpMatches(vpMapPoints.size(), nullptr);
            int nmatches = matcher.SearchByBoW(pKF1, pKF2, vpMatches);
            if (nmatches <= 20) continue;

            // Set MapPoint vertices
            const int N = vpMatches.size();
            const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();

            for (int i = 0; i < N; i++) {
                if (!vpMatches[i]) continue;

                MapPoint *pMP1 = vpMapPoints1[i];
                MapPoint *pMP2 = vpMatches[i];
                if (!pMP1 || !pMP2) continue;

                if (pMP1->mpMap->mnId != baseMapId || pMP2->mpMap->mnId != otherMapId)
                    continue;

                const int id1 = nLastId + 2 * i + 1;
                const int id2 = nLastId + 2 * (i + 1);

                auto i2 = pMP2->GetIndexInKeyFrame(pKF2);

                if (pMP1->isBad() || pMP2->isBad() || i2 < 0) continue;


                auto vPoint = new g2o::VertexSBAPointXYZ();
                cv::Mat P3Dw2 = pMP2->GetWorldPos();
                vPoint->setEstimate(Converter::toVector3d(P3Dw2));
                vPoint->setId(id1);
                vPoint->setFixed(true);
                optimizer.addVertex(vPoint);

                nCorrespondences++;

                cv::Mat P3Dw1 = pMP1->GetWorldPos();
                auto e1 = new g2o::EdgeSim3RelativeXYZ;
                e1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
                e1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                e1->setMeasurement(Converter::toVector3d(P3Dw1));
//                const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
//                e1->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);
                auto rk1 = new g2o::RobustKernelHuber;
                e1->setRobustKernel(rk1);
                rk1->setDelta(deltaHuber);
                optimizer.addEdge(e1);

                vpEdges.push_back(e1);

                ++nMP;
            }
            nLastId += 2 * N;
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        // Check inliers
        int nBad = 0;
        for (size_t i = 0; i < vpEdges.size(); i++) {
            g2o::EdgeSim3RelativeXYZ *e1 = vpEdges[i];
            if (!e1) continue;

            auto e1chi2 = e1->chi2();

            if (e1chi2 > th2) {
//                size_t idx = vnIndexEdge[i];
//                vpMatches[idx] = nullptr;
                optimizer.removeEdge(e1);
                vpEdges[i] = nullptr;
                nBad++;
            }
        }

        debug("total nMP {}", nMP);

        int nMoreIterations;
        if (nBad > 0)
            nMoreIterations = 10;
        else
            nMoreIterations = 5;

        if (nCorrespondences - nBad < 10) {
            debug("nCorrespondences - nBad {}", nCorrespondences - nBad);
            return 0;
        }

        // Optimize again only with inliers

        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        for (size_t i = 0; i < vpEdges.size(); i++) {
            g2o::EdgeSim3RelativeXYZ *e1 = vpEdges[i];
            if (!e1) continue;

            auto e1chi2 = e1->chi2();

            if (e1chi2 <= th2) {
                nIn++;
            }
        }

        auto vSim3_recov = dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
        Twl = vSim3_recov->estimate();

        debug("total inliers / nMP: {}", double(nIn) / nMP);

        return nIn;
    }

    int Optimizer::OptimizeSim3ByMapPoint(KeyFrame *pKF1, KeyFrame *pTargetKF, vector<pair<MapPoint *, MapPoint*> > pairs, const float th2, g2o::Sim3 &Twl) {
        g2o::SparseOptimizer optimizer;

        auto linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        auto solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // Calibration
        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pTargetKF->mK;

        // initialize vSim3
        auto vSim3 = new g2o::VertexSim3Expmap();

        vSim3->_fix_scale = false;
        vSim3->setEstimate(Twl);
        vSim3->setId(0);
        vSim3->setFixed(false);
        vSim3->_principle_point1[0] = K1.at<float>(0, 2);
        vSim3->_principle_point1[1] = K1.at<float>(1, 2);
        vSim3->_focal_length1[0] = K1.at<float>(0, 0);
        vSim3->_focal_length1[1] = K1.at<float>(1, 1);
        vSim3->_principle_point2[0] = K2.at<float>(0, 2);
        vSim3->_principle_point2[1] = K2.at<float>(1, 2);
        vSim3->_focal_length2[0] = K2.at<float>(0, 0);
        vSim3->_focal_length2[1] = K2.at<float>(1, 1);
        optimizer.addVertex(vSim3);

        ORBmatcher matcher(0.8, true);

        unsigned long baseMapId = pKF1->mpMap->mnId;
        unsigned long otherMapId = pTargetKF->mpMap->mnId;
        if (pKF1->mpMap->mbBaseMap) {

        } else if (pTargetKF->mpMap->mbBaseMap) {
            swap(baseMapId, otherMapId);
        } else {
            // TODO(halcao): other case
            return 0;
        }

        vector<g2o::EdgeSim3RelativeXYZ *> vpEdges;
        const float deltaHuber = sqrt(th2);
        int nCorrespondences = 0;

        int nMP = 0;
        int nLastId = 0;

        for (size_t i = 0; i < pairs.size(); ++i) {
            auto pair = pairs[i];
            MapPoint *pMP1;
            MapPoint *pMP2;
            if (pair.first->mpMap->mnId == baseMapId && pair.second->mpMap->mnId == otherMapId) {
                pMP1 = pair.first;
                pMP2 = pair.second;
            } else if (pair.first->mpMap->mnId == otherMapId && pair.second->mpMap->mnId == baseMapId) {
                pMP1 = pair.second;
                pMP2 = pair.first;
            } else {
                continue;
            }

//            if (pairs.size() <= 20) continue;

            if (!pMP1 || !pMP2) continue;

            const int id1 = 2 * i + 1;
            const int id2 = 2 * (i + 1);

//            auto i2 = pMP2->GetIndexInKeyFrame(pKF2);

            if (pMP1->isBad() && pMP1->GetReplaced() != pMP2) {
                continue;
            }

            if (pMP2->isBad() && pMP2->GetReplaced() != pMP1) {
                continue;
            }

//            if (pMP1->isBad() || pMP2->isBad()) continue;

            auto vPoint = new g2o::VertexSBAPointXYZ();
            cv::Mat P3Dw2 = pMP2->GetWorldPos();
            vPoint->setEstimate(Converter::toVector3d(P3Dw2));
            vPoint->setId(id1);
            vPoint->setFixed(true);
            optimizer.addVertex(vPoint);

            nCorrespondences++;

            cv::Mat P3Dw1 = pMP1->GetWorldPos();
            auto e1 = new g2o::EdgeSim3RelativeXYZ;
            e1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
            e1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e1->setMeasurement(Converter::toVector3d(P3Dw1));
//                const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
//                e1->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);
            auto rk1 = new g2o::RobustKernelHuber;
            e1->setRobustKernel(rk1);
            rk1->setDelta(deltaHuber);
            optimizer.addEdge(e1);

            vpEdges.push_back(e1);

            ++nMP;
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        // Check inliers
        int nBad = 0;
        for (size_t i = 0; i < vpEdges.size(); i++) {
            g2o::EdgeSim3RelativeXYZ *e1 = vpEdges[i];
            if (!e1) continue;

            auto e1chi2 = e1->chi2();

            if (e1chi2 > th2) {
//                size_t idx = vnIndexEdge[i];
//                vpMatches[idx] = nullptr;
                optimizer.removeEdge(e1);
                vpEdges[i] = nullptr;
                nBad++;
            }
        }

        debug("total nMP {}", nMP);

        int nMoreIterations;
        if (nBad > 0)
            nMoreIterations = 10;
        else
            nMoreIterations = 5;

        if (nCorrespondences - nBad < 10) {
            debug("nCorrespondences - nBad {}", nCorrespondences - nBad);
            return 0;
        }

        // Optimize again only with inliers

        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        for (size_t i = 0; i < vpEdges.size(); i++) {
            g2o::EdgeSim3RelativeXYZ *e1 = vpEdges[i];
            if (!e1) continue;

            auto e1chi2 = e1->chi2();

            if (e1chi2 <= th2) {
                nIn++;
            }
        }

        auto vSim3_recov = dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
        Twl = vSim3_recov->estimate();

        debug("total inliers / nMP: {}", double(nIn) / nMP);

        return nIn;
    }
} //namespace ORB_SLAM
