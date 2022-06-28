/**
* This file is not part of ORB-SLAM2.
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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>

#include<opencv2/core/core.hpp>
#include <BoostArchiver.h>

#include <Map.h>
#include <Viewer.h>
#include <KeyFrameDatabase.h>
#include <ORBmatcher.h>
#include <PnPsolver.h>
#include <Optimizer.h>
#include <ORBextractor.h>
#include <Converter.h>
#include <include/MapEnhancer.h>

using namespace std;
using namespace ORB_SLAM2;

void GenerateKeyFrameBone(vector<KeyFrame *> keyframes, Map *&mpMap, KeyFrameDatabase *&keyFrameDatabase) {
    // generate keyframes parallel to the path

    KeyFrame *lastKF = nullptr;
    for (size_t i = 0; i < keyframes.size(); ++i) {
        auto kf = keyframes[i];
        if (!kf || kf->isBad() || !kf->isGenuine) continue;
        if (!lastKF) {
            lastKF = kf;
            continue;
        }
        auto lastTranslation = lastKF->GetGlobalTranslation();
        auto translation = kf->GetGlobalTranslation();
        auto length = cv::norm(lastTranslation, translation);
        auto offset = length * 0.08;

        auto pose = kf->GetGlobalPose();
        // synthesize keyframe at different pose
        for (size_t i = 0; i < 8; ++i) {
            auto newPose = pose.clone();
            for (size_t bit = 0; bit < 3; ++bit) {
                auto isOn = (i >> bit) & 1;
                if (isOn) {
                    newPose.row(bit).col(3) += offset * (i / 8 + 1);
                }
            }
            auto newKF = MapEnhancer::GenerateKeyFrame(newPose, kf);
            if (newKF) {
                mpMap->AddKeyFrame(newKF);
                keyFrameDatabase->add(newKF);
            }
        }

        lastKF = kf;
    }

    for (auto &itr: mpMap->allKFs) {
        auto kf = itr.second;
        if (kf->isGenuine) {
            kf->SetBadFlag();
        }
    }
}

void loadMap(const string &filename, Map *&pMap, KeyFrameDatabase *&pKeyFrameDatabase, ORBVocabulary *&pVoc) {
    std::ifstream in(filename, std::ios_base::binary);
    if (!in) {
        cerr << "Cannot Open Mapfile: " << filename << " , Create a new one" << std::endl;
        return;
    }
    cout << "Loading Mapfile: " << filename << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> pMap;
    ia >> pKeyFrameDatabase;
    in.close();

    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;

    pKeyFrameDatabase->SetORBvocabulary(pVoc);
    auto allKFs = pMap->allKFs;

    size_t nBad = 0;
    size_t nKFBad = 0;

    auto allMPs = pMap->allMPs;

    for (auto it: allKFs) {
        auto itr = it.second;

        itr->mpKeyFrameDB = pKeyFrameDatabase;
        itr->ComputeBoW();
        itr->mpMap = pMap;

        itr->RestoreSerialization();

        if (itr->isBad()) {
            nKFBad += 1;
        }
    }

    for (auto &it: allMPs) {
        auto itr = it.second;
        itr->mpMap = pMap;

        itr->RestoreSerialization();

        if (itr->isBad()) {
            nBad += 1;
        }
    }


    pKeyFrameDatabase->RestoreSerializationVariable(pMap);

    cout << " ...done" << endl;

    cout << "Keyframes: " << pMap->allKFs.size() << " bad count: " << nKFBad << endl;
    cout << "MapPoint: " << pMap->allMPs.size() << " bad count: " << nBad << endl;
}


//Calibration matrix
cv::Mat mK;
cv::Mat mDistCoef;
float mbf;
//Color order (true RGB, false BGR, ignored if grayscale)
bool mbRGB;
ORBextractor *pORBextractor;

void loadSettings(const string &strSettingPath) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if (DistCoef.rows == 5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
//    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if (mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    pORBextractor = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

}

bool Relocalization(Frame &currentFrame, KeyFrameDatabase *&pKeyFrameDB) {
    // Compute Bag of Words Vector
    currentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame *> vpCandidateKFs = pKeyFrameDB->DetectRelocalizationCandidates(&currentFrame);

    if (vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<PnPsolver *> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint *> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++) {
        KeyFrame *pKF = vpCandidateKFs[i];
//        if (pKF->isBad())
//            vbDiscarded[i] = true;
//        else {
            int nmatches = matcher.SearchByBoW(pKF, currentFrame, vvpMapPointMatches[i]);
            if (nmatches < 15) {
                vbDiscarded[i] = true;
                continue;
            } else {
                PnPsolver *pSolver = new PnPsolver(currentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
//        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch) {
        for (int i = 0; i < nKFs; i++) {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver *pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (!Tcw.empty()) {
//                Tcw.copyTo(currentFrame.mTcw);
                currentFrame.SetPose(Tcw);

                set<MapPoint *> sFound;

                const int np = vbInliers.size();

                for (int j = 0; j < np; j++) {
                    if (vbInliers[j]) {
                        currentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    } else
                        currentFrame.mvpMapPoints[j] = nullptr;
                }

                int nGood = Optimizer::PoseOptimization(&currentFrame, true);

                if (nGood < 10)
                    continue;

                for (int io = 0; io < currentFrame.N; io++)
                    if (currentFrame.mvbOutlier[io])
                        currentFrame.mvpMapPoints[io] = nullptr;

                // If few inliers, search by projection in a coarse window and optimize again
                if (nGood < 50) {
                    int nadditional = matcher2.SearchByProjection(currentFrame, vpCandidateKFs[i], sFound, 10, 100,
                                                                  true);

                    if (nadditional + nGood >= 50) {
                        nGood = Optimizer::PoseOptimization(&currentFrame, true);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if (nGood > 30 && nGood < 50) {
                            sFound.clear();
                            for (int ip = 0; ip < currentFrame.N; ip++)
                                if (currentFrame.mvpMapPoints[ip])
                                    sFound.insert(currentFrame.mvpMapPoints[ip]);
                            nadditional = matcher2.SearchByProjection(currentFrame, vpCandidateKFs[i], sFound, 3, 64,
                                                                      true);

                            // Final optimization
                            if (nGood + nadditional >= 50) {
                                nGood = Optimizer::PoseOptimization(&currentFrame, true);

                                for (int io = 0; io < currentFrame.N; io++)
                                    if (currentFrame.mvbOutlier[io])
                                        currentFrame.mvpMapPoints[io] = nullptr;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if (nGood >= 50) {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch) {
        return false;
    } else {
//        mnLastRelocFrameId = currentFrame.mnId;
        return true;
    }

}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps) {
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t / 1e9);

        }
    }
}

void LoadImagesTUM(const string &strImagePath, const string &strPathTimes, vector<string> &vstrImageFilenames,
                   vector<double> &vTimestamps) {
    ifstream f;
    f.open(strPathTimes.c_str());

    // skip first three lines
    string s0;
    getline(f, s0);
    getline(f, s0);
    getline(f, s0);

    while (!f.eof()) {
        string s;
        getline(f, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(strImagePath + "/" + sRGB);
        }
    }
}


void SaveKeyFrameTrajectoryTUM(vector<Frame> &frames, const string &filename) {
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    sort(frames.begin(), frames.end(), [](const Frame &f1, const Frame &f2) {
        return f1.mnId < f2.mnId;
    });


    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();


    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (auto frame : frames) {
        // pKF->SetPose(pKF->GetPose()*Two);

        if (!frame.mTcw.data)
            continue;

        cv::Mat R = frame.GetRotationInverse();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = frame.GetCameraCenter();
        f << setprecision(6) << frame.mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1)
          << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}


int main(int argc, char **argv) {
    if (argc != 7) {
        cerr << endl
             << "Usage: ./relocalizer map_file_path setting_file_path voc_path img_sequence path_to_times_file dataset"
             << endl;
        return 1;
    }


    const string filename = argv[1];
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    const string vocPath = argv[3];

    auto pVocabulary = new ORBVocabulary();
    bool bVocLoad = pVocabulary->loadFromBinaryFile(vocPath);
    if (!bVocLoad) {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << vocPath << endl;
        exit(-1);
    }

    auto pkfDB = new KeyFrameDatabase(*pVocabulary);

    // load map
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    Map *pMap = nullptr;
    loadMap(filename, pMap, pkfDB, pVocabulary);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(end - start).count();
    cout << "map load time: " << ttrack << endl;

    vector<string> filenames;
    vector<double> timestamps;
    vector<bool> results;
    vector<Frame> frames;
    cout << "loading image\n";

    string dataset = argv[6];

    if (dataset == "tum") {
        LoadImagesTUM(string(argv[4]), string(argv[5]), filenames, timestamps);
    } else if (dataset == "euroc") {
        LoadImages(string(argv[4]), string(argv[5]), filenames, timestamps);
    }

    auto imageCount = filenames.size();

    // Vector for tracking time statistics
    vector<float> trackTimes;
    trackTimes.resize(imageCount);
    results.resize(imageCount);
    frames.resize(imageCount);


    // initialize map & frame drawer
    auto pFrameDrawer = new FrameDrawer(pMap);
    auto pMapDrawer = new MapDrawer(pMap, argv[2]);
    loadSettings(argv[2]);
    auto pViewer = new Viewer(nullptr, pFrameDrawer, pMapDrawer, nullptr, argv[2]);
    auto tViewer = thread([&pViewer]() {
        pViewer->Run();
    });

    cout << "\n-------\n";
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << imageCount << endl;

    cv::Mat im;

    const size_t workerSize = 20;
    // divide image into worker groups
    vector<std::thread> threads(workerSize);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    for (size_t i = 0; i < imageCount; ++i) {
        im = cv::imread(filenames[i], cv::IMREAD_UNCHANGED);
        double timestamp = timestamps[i];
        if (!im.data) {
            cout << "load failed at: " << i << " " << filenames[i] << endl;
            continue;
        }
        // convert image
        if (im.channels() == 3) {
            cvtColor(im, im, mbRGB ? CV_RGB2GRAY : CV_BGR2GRAY);
        } else if (im.channels() == 4) {
            cvtColor(im, im, mbRGB ? CV_RGBA2GRAY : CV_BGRA2GRAY);
        }
        std::chrono::steady_clock::time_point trackStart = std::chrono::steady_clock::now();
        Frame mCurrentFrame = Frame(im, timestamp, pORBextractor, pVocabulary, mK, mDistCoef, mbf, 0);
        if (i == 0) {
            GenerateKeyFrameBone(pMap->GetAllKeyFrames(), pMap, pkfDB);
        }

        size_t threadIdx = i % workerSize;
        if (threads[threadIdx].joinable()) {
            threads[threadIdx].join();
        }
        //track(const size_t i, cv::Mat &im, double timestamp, KeyFrameDatabase *pkfDB, ORBVocabulary *&pVocabulary);
        threads[threadIdx] = thread([&, i, mCurrentFrame]() mutable {
            bool result = Relocalization(mCurrentFrame, pkfDB);
            std::chrono::steady_clock::time_point trackEnd = std::chrono::steady_clock::now();
            double trackTime = std::chrono::duration_cast<std::chrono::duration<double> >(
                    trackEnd - trackStart).count();

            if (result && mCurrentFrame.mTcw.data) {
                pMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
            }

            frames[i] = mCurrentFrame;
            trackTimes[i] = trackTime;
            results[i] = result;

            cout << "track " << i << " time: " << trackTime << "  matched: " << result << endl;
        });

    }

    for (auto &thread: threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    int matchedCount = count(results.begin(), results.end(), true);
    cout << "matched / total = " << matchedCount << " / " << imageCount << " : " << matchedCount / double(imageCount)
         << endl;

    double averageTime = accumulate(trackTimes.begin(), trackTimes.end(), 0.0) / imageCount;
    cout << "mean tracking time: " << averageTime << endl;

    SaveKeyFrameTrajectoryTUM(frames, "FrameTrajectory.txt");


    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double trackTime = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout << "total tracking time: " << trackTime << endl;


    if (pViewer) {
        cout << "press any key to stop\n";
        getchar();
    }

    if (pViewer) {
        cout << "wait pViewer to finish\n";
        pViewer->RequestFinish();
        while (!pViewer->isFinished()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        tViewer.join();
    }
    return 0;
}

