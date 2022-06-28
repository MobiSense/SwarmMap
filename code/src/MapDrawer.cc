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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <cmath>
#include <include/Converter.h>

namespace ORB_SLAM2 {

    vector<int> MapDrawer::pointColors {
        0x000000,
//        0x36688d,
        0xbda589,
        0xa7414a,
        0x282726,
        0x6a8a82,
        0xa37c27
    };

    vector<int> MapDrawer::frameColors {
        0x0444bf,
        0xf22f08,
        0x583e2e,
        0x00743f,
        0x16235a,
        0xffff00,
        0x000
    };

    MapDrawer::MapDrawer(Map *pMap, const string &strSettingPath) : mpMap(pMap) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

    }

    void setColor(int hex) {
        int mask = (1 << 8) - 1;
        double r = double((hex >> 16) & mask) / 255.0;
        double g = double((hex >> 8) & mask) / 255.0;
        double b = double(hex & mask) / 255.0;
        glColor3f(r, g, b);
    }

    void MapDrawer::DrawMapPoints() {
        const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = mpMap->GetReferenceMapPoints();

        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if (vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;

            auto color = pointColors[(vpMPs[i]->GetOriginMapId() / 2) % pointColors.size()];
            setColor(color);
//            cv::Mat pos = vpMPs[i]->GetWorldPos();
            cv::Mat pos = vpMPs[i]->GetGlobalPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        }
        glEnd();

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
//        glColor3f(1.0, 0.0, 0.0);

        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
            if ((*sit)->isBad())
                continue;

//            cv::Mat pos = (*sit)->GetWorldPos();
            auto color = pointColors[((*sit)->GetOriginMapId() / 2) % pointColors.size()];
            setColor(color);

            cv::Mat pos = (*sit)->GetGlobalPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));

        }

        glEnd();
    }

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
        const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

        if (bDrawKF) {
            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];

                const float &w = mKeyFrameSize * frameScaleFactor;
                const float h = w * 0.75;
                const float z = w * 0.6;

//                cv::Mat Twc = pKF->GetPoseInverse().t();
                cv::Mat Twc = pKF->GetGlobalPoseInverse().t();

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);

                auto color = frameColors[(pKF->GetOriginMapId() / 2) % frameColors.size()];
//                if (pKF->mRelocScore == -1) {
//                    color = frameColors[6];
//                } else
                if (!pKF->isGenuine) {
                    color = frameColors[5];
                }
                setColor(color);

                glBegin(GL_LINES);
                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }
        }

        if (bDrawGraph) {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

//            sort(vpKFs.begin(), vpKFs.end(), [](KeyFrame *a, KeyFrame *b){
//                return a->mnId < b->mnId;
//            });

            KeyFrame* lastValidKF = nullptr;
            map<unsigned long, KeyFrame *> lastValidKFs;
            for (size_t i = 0; i < vpKFs.size(); i++) {
                bool hasConnection = false;

                // Covisibility Graph
//                const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
//                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                cv::Mat Ow = vpKFs[i]->GetGlobalCameraCenter();
//                if (!vCovKFs.empty()) {
//                    for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
//                         vit != vend; vit++) {
//                        if ((*vit)->mnId < vpKFs[i]->mnId)
//                            continue;
//
//
////                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
//                        cv::Mat Ow2 = (*vit)->GetGlobalCameraCenter();
//                        glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
//                        glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
//                        hasConnection = true;
//                    }
//                }

                // Spanning tree
                KeyFrame *pParent = vpKFs[i]->GetParent();
                if (pParent) {
//                    cv::Mat Owp = pParent->GetCameraCenter();
                    cv::Mat Owp = pParent->GetGlobalCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
                    hasConnection = true;
                }

                // Loops
//                set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
//                for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
//                    if ((*sit)->mnId < vpKFs[i]->mnId)
//                        continue;
////                    cv::Mat Owl = (*sit)->GetCameraCenter();
//                    cv::Mat Owl = (*sit)->GetGlobalCameraCenter();
//                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
//                    glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
//                    hasConnection = true;
//                }

                auto lastKF = lastValidKFs[vpKFs[i]->GetOriginMapId()];
                if (!hasConnection && lastKF && vpKFs[i]->mnId - lastKF->mnId < 10) {
//                if (!hasConnection && lastKF) {
                    cv::Mat Owl = lastKF->GetGlobalCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
                } else {
                    int last = i-1;
                    while (last >= 0 && (vpKFs[last]->GetOriginMapId() != vpKFs[i]->GetOriginMapId() || vpKFs[last]->mnId >= vpKFs[i]->mnId)) {
                        last--;
                    }
                    if (last >= 0 && vpKFs[i]->mnId - vpKFs[last]->mnId < 15) {
                        cv::Mat Owl = vpKFs[last]->GetGlobalCameraCenter();
                        glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                        glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
                    }
                }

                lastValidKFs[vpKFs[i]->GetOriginMapId()] = vpKFs[i];
            }

            glEnd();
        }
    }

    void MapDrawer::DrawCurrentCamera(const bool bDrawKF, pangolin::OpenGlMatrix &Twc) {
        if (!bDrawKF) return;

        const float &w = mCameraSize * frameScaleFactor;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }


    void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw) {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) {
        if (!mCameraPose.empty()) {
            cv::Mat Rwc(3, 3, CV_32F);
            cv::Mat twc(3, 1, CV_32F);
            {
                unique_lock<mutex> lock(mMutexCamera);
                Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
                twc = -Rwc * mCameraPose.rowRange(0, 3).col(3);
            }

            M.m[0] = Rwc.at<float>(0, 0);
            M.m[1] = Rwc.at<float>(1, 0);
            M.m[2] = Rwc.at<float>(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwc.at<float>(0, 1);
            M.m[5] = Rwc.at<float>(1, 1);
            M.m[6] = Rwc.at<float>(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwc.at<float>(0, 2);
            M.m[9] = Rwc.at<float>(1, 2);
            M.m[10] = Rwc.at<float>(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15] = 1.0;
        } else
            M.SetIdentity();
    }

} //namespace ORB_SLAM
