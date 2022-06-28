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


#include<iostream>
#include<algorithm>
#include<fstream>
#include <iomanip>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <Map.h>
#include <KeyFrameDatabase.h>
#include <future>
//#include <boost/archive/binary_oarchive.hpp>
//#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "popl.hpp"
#include <BoostArchiver.h>
#include <MapUpdater.h>
#include <ClientMediator.h>
#include <Timer.h>
#include <MapManager.h>
#include <Converter.h>
#include <LandmarkScoring.h>
#include <MapUpdater.h>
#include <MediatorScheduler.h>
#include <MapSlice.h>
#include <SystemState.h>
#include "ConnectionService.h"
#include "CLogger.h"
#include "Mapit.h"
#include "DataSetUtil.h"

using namespace std;
using namespace ORB_SLAM2;
using namespace popl;

ORB_SLAM2::ClientMediator *globalMediator = nullptr;
map<unsigned long, size_t> emptyCountMap;

std::atomic_bool b;

void GetTrackingInfo(ORB_SLAM2::System *SLAM, ORB_SLAM2::ClientMediator *mediator) {
    auto state = SLAM->GetSystemState();

    // TODO(halcao): network

    if (!state.bStable) {
        emptyCountMap[mediator->mnId] = 0;
    }

    SLAM->GetMap()->GetConnectionService()->ReportState(state);
}

void UploadMap(ORB_SLAM2::System *SLAM, ORB_SLAM2::ClientMediator *mediator) {
    info("receive map called");
    if (!SLAM || !SLAM->GetMap() || !mediator || !mediator->GetMap()) return;
    // simulate sending and receiving process

    FuncTimer();

    // get serialized result
    string result;
    SLAM->GetMap()->GetMapit()->Push(result);

    // TODO(halcao): network
    // text serialization min size equals 56
    if (result.size() <= 60) {
        emptyCountMap[mediator->mnId] += 1;
    } else {
        emptyCountMap[mediator->mnId] = 0;
    }

    // TODO(halcao): network connection

    // client id
    auto id = SLAM->GetMap()->mnId - 1;
    MediatorScheduler::GetInstance().EnqueueRequest(id, result);
}

string GetCurrentTime() {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%Y-%m-%d_%H:%M:%S", timeinfo);
    return string(buffer);
}

void track(ORB_SLAM2::System *SLAM, const std::string imageName, double tframe) {
    Timer t("client track", false);
    cv::Mat im = cv::imread(imageName, cv::IMREAD_UNCHANGED);
    if (im.empty()) {
        error("Failed to load image at: {}", imageName);
        return;
    }

    SLAM->TrackMonocular(im, tframe);
}

void Run(vector<System *> SLAMs, vector<ClientMediator *> mediators, size_t nClient) {
    b.store(true);
    size_t n = 0;
    while (b.load()) {
        std::this_thread::sleep_for(chrono::milliseconds(500));
        n += 1;
        for (size_t i = 0; i < nClient; i++) {
            GetTrackingInfo(SLAMs[i], mediators[i]);
        }

        if (n == 4) {
            n = 0;

            size_t stopCount = 0;
            vector<thread> threads(nClient);
            for (size_t i = 0; i < nClient; i++) {
                auto mediator = mediators[i];
                if (emptyCountMap[mediator->mnId] > 5) {
                    stopCount++;
                    continue;
                }

                if (nClient > 1) {
                    threads[i] = thread([&SLAMs, &mediator, i]() {
                        UploadMap(SLAMs[i], mediator);
                    });
                } else {
                    UploadMap(SLAMs[i], mediator);
                }
            }
            if (stopCount == nClient) {
                b.store(false);
            }

            for (size_t i = 0; i < nClient; i++) {
                if (threads[i].joinable()) {
                    threads[i].join();
                }
            }
        }
    }
}

void setupNetwork(vector<System *> SLAMs, vector<ClientMediator *> mediators) {
    for (size_t i = 0; i < SLAMs.size(); i++) {
        SLAMs[i]->GetMap()->TryConnect(mediators[i]);
        mediators[i]->GetMap()->TryConnect(SLAMs[i]);
    }
}

int main(int argc, char **argv) {
    OptionParser op("edgeSLAM - mono euroc");
    auto help_option   = op.add<Switch>("h", "help", "Usage: ./map_conveyance path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file/tum debug_level [use_viewer] [use_map_viewer]");
    auto voc_option = op.add<Value<std::string>>("v", "voc", "path to vocabulary");
    // multiple dataset
    auto dataset_option = op.add<Value<std::string>>("d", "dataset", "path to dataset config file");
    auto log_level_option  = op.add<Value<std::string>>("l", "log", "log level: error/warn/info/debug", "debug");
    auto viewer_option  = op.add<Value<bool>>("u", "viewer", "use viewer", true);
    auto map_viewer_option  = op.add<Value<bool>>("m", "mapviewer", "use map viewer", true);
    auto client_number_option  = op.add<Value<int>>("c", "client", "client number", 2);
    op.parse(argc, argv);

    // print auto-generated help message
    if (help_option->count() > 0) {
        cout << op << "\n";
    }

    if (!dataset_option->is_set()) {
        error("dataset type or times file not set");
        return 1;
    }

    // get dataset config set
    auto path = dataset_option->value();
    cv::FileStorage file(path, cv::FileStorage::READ);

    // get settings file path
    const auto settingsFile = file["SETTING"];

    // get dataset type
    string datasetType = file["TYPE"];

    // image folder paths
    vector<string> imagePaths;
    // time folder paths
    vector<string> timeFiles;
    file["IMAGES"] >> imagePaths;
    auto nDataset = imagePaths.size();

    if (datasetType == "euroc") {
        file["TIMES"] >> timeFiles;
        if (imagePaths.size() != timeFiles.size()) {
            error("the image path number should be the same as time files");
            return 2;
        }
    }

    // Retrieve paths to images
    // vector of image paths
    vector<vector<string>> imageDatasetList(nDataset);
    // vector of timestamps
    vector<vector<double>> timestampSetList(nDataset);

    // max length of the dataset
    size_t maxSeqLength = 0;
    for (size_t i = 0; i < nDataset; ++i) {
        const auto imagePath = imagePaths[i];
        if (datasetType == "tum") {
            DataSetUtil::LoadTUM(string(imagePath), imageDatasetList[i], timestampSetList[i]);
        } else if (datasetType == "euroc") {
            const auto timesFile = timeFiles[i];
            DataSetUtil::LoadEuRoC(string(imagePath), timesFile, imageDatasetList[i], timestampSetList[i]);
        } else if (datasetType == "kitti") {
            DataSetUtil::LoadKITTI(string(imagePath), imageDatasetList[i], timestampSetList[i]);
        }

        maxSeqLength = std::max(maxSeqLength, imageDatasetList[i].size());
    }

    CLogger::SetLevel(log_level_option->value());

    bool use_viewer = viewer_option->value();
    bool use_map_viewer = map_viewer_option->value();

    auto pVoc = new ORBVocabulary();
    const auto vocFile = voc_option->value();
    pVoc->loadFromBinaryFile(vocFile);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const size_t nClient = client_number_option->value();

    vector<ORB_SLAM2::System *> SLAMs;
    vector<ORB_SLAM2::ClientMediator *> mediators;

    if (nClient < nDataset) {
//        error("client number should not be less than dataset number");
//        return 2;
        info("client number is less than dataset number, will only run some of the datasets");
        nDataset = nClient;
    }

    // initialize slam systems and server handler(ClientMediator)
    for (size_t i = 0; i < nClient; ++i) {
        // KEY: the constructor will change argv[2] so that the gl and cv process can not be done
        ClientMediator *mediator;
        if (nClient == 1) {
            // make this mediator global
            mediator = new ORB_SLAM2::ClientMediator(settingsFile, pVoc, use_map_viewer, true);
            globalMediator = mediator;
        } else {
            mediator = new ORB_SLAM2::ClientMediator(settingsFile, pVoc, use_map_viewer);
        }
        auto SLAM = new ORB_SLAM2::System(vocFile, settingsFile, ORB_SLAM2::System::MONOCULAR, use_map_viewer);
        mediators.push_back(mediator);
        SLAMs.push_back(SLAM);
    }

    MediatorScheduler::GetInstance().getSLAMSystem = [&SLAMs](unsigned long i){
        if (i < SLAMs.size()) {
            return SLAMs[i];
        } else {
            return static_cast<System *>(nullptr);
        }
    };

    setupNetwork(SLAMs, mediators);

    if (nClient > 1) {
        auto pVoc2 = new ORBVocabulary();
        pVoc2->loadFromBinaryFile(vocFile);

        globalMediator = new ORB_SLAM2::ClientMediator(settingsFile, pVoc2, use_map_viewer, true);
    } else if (nClient <= 0) {
        error("ClientMediator number should be positive");
        return 2;
    }

    auto tRetriever = new thread(Run, SLAMs, mediators, nClient);

//    // Vector for tracking time statistics
//    vector<double> vTimesTrack(nImages, 0);
//
//    info("-------");
//    info("Start processing sequence ...");
//    info("Images in the sequence: {}", nImages);
//
//    const auto nClip = nImages / nClient;
//
//    size_t nOverlap = 20 * 5;

    // simplified thread pool
    vector<thread *> threads(nClient, nullptr);

    // Main loop
    for (size_t ni = 0; ni < maxSeqLength; ni++) {
        Timer tTrack("Total Track Time", false);

        // Pass the image to the SLAM system
        for (size_t i = 0; i < nClient; ++i) {
            size_t idx = ni;

            // if i > `nDataset - 1`, then run `nDataset - 1`;
            size_t datasetIdx = min(i, nDataset - 1);
            const auto &images = imageDatasetList[datasetIdx];
            const auto &timestamps = timestampSetList[datasetIdx];

            if (idx >= images.size()) continue;

            const auto imageName = images[idx];
            // Read image from file
            double tframe = timestamps[idx];

            auto SLAM = SLAMs[i];
            if (nClient > 1) {
                // wait last track to be done
                if (threads[i] != nullptr && threads[i]->joinable()) {
                    threads[i]->join();
                    delete threads[i];
                    threads[i] = nullptr;
                }

                threads[i] = new thread(track, SLAM, imageName, tframe);
            } else {
                track(SLAM, imageName, tframe);
            }
        }

//        auto const time = tTrack.get();
//        if (time < 30) {
//            usleep((30 - time)*1e3);
//        }

//        if (ni < vTimesTrack.size()) {
//            vTimesTrack[ni] = tTrack.get();
//            trace("full track time: {}", vTimesTrack[ni]);
//        }

//        double tframe = vTimestamps[ni];
//        // Wait to load the next frame
//        double T = 0;
//            if (ni < nImages - 1)
//                T = vTimestamps[ni + 1] - tframe;
//            else if (ni > 0)
//                T = tframe - vTimestamps[ni - 1];
//
//        if(vTimesTrack[ni]<T)
//            usleep((T-vTimesTrack[ni])*1e6);
    }

    for (auto &thread: threads) {
        if (thread != nullptr && thread->joinable()) {
            thread->join();
        }
    }

    if (use_viewer || use_map_viewer) {
        info("press any key to stop");
        getchar();
    }

    // Stop all threads
    if (b.load()) {
        b.store(false);
        debug("wait tRetriever to stop");
        tRetriever->join();
    }

    for (auto &SLAM: SLAMs) {
        SLAM->Shutdown();
    }

    for (auto &mediator: mediators) {
        mediator->Shutdown();
    }

    if (globalMediator) {
        globalMediator->Shutdown();
    }

    // Tracking time statistics
//    sort(vTimesTrack.begin(), vTimesTrack.end());
//    float totaltime = 0;
//    for (size_t ni = 0; ni < vTimesTrack.size(); ni++) {
//        totaltime += vTimesTrack[ni];
//    }
//    const auto trackSize = vTimesTrack.size();
//    info("median tracking time: {}", vTimesTrack[trackSize / 2]);
//    info("mean tracking time: {}", totaltime / trackSize);

    LandmarkScoring::Save("result" + GetCurrentTime());

    // Save camera trajectory
    for (size_t i = 0; i < nClient; ++i) {
        auto SLAM = SLAMs[i];
        auto mediator = mediators[i];
        debug("original map {} keyframe count: {}", SLAM->GetMap()->mnId, SLAM->GetMap()->KeyFramesInMap());
        debug("original map {} mappoint count: {}", SLAM->GetMap()->mnId, SLAM->GetMap()->MapPointsInMap());
        SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory-" + GetCurrentTime() + "-" + std::to_string(SLAM->GetMap()->mnId) + ".txt");
        SLAM->SaveMap("map-old-" + std::to_string(SLAM->GetMap()->mnId) + ".bin");

        debug("new map {} keyframe count: {}", mediator->GetMap()->mnId, mediator->GetMap()->KeyFramesInMap());
        debug("new map {} mappoint count: {}", mediator->GetMap()->mnId, mediator->GetMap()->MapPointsInMap());

        mediator->SaveMap("map-new-" + std::to_string(mediator->GetMap()->mnId) + ".bin");

//        delete SLAM;
//        delete mediator;
    }

    if (globalMediator && nClient > 1) {
//        globalMediator->SaveMap("map-global-" + std::to_string(globalMediator->GetMap()->mnId) + ".bin");
        globalMediator->SaveMap("map-global.bin");
    }

    MapManager::SaveGlobalMap("map");
    return 0;
}
