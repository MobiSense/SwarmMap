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
#include <AgentMediator.h>
#include <Timer.h>
#include <MapManager.h>
#include <Converter.h>
#include <LandmarkScoring.h>
#include <MapUpdater.h>
#include <MediatorScheduler.h>
#include <MapSlice.h>
#include <SystemState.h>
#include <ClientService.h>
//#include "ConnectionService.h"
#include "CLogger.h"
#include "Mapit.h"
#include "DataSetUtil.h"

using namespace std;
using namespace ORB_SLAM2;
using namespace popl;

ORB_SLAM2::AgentMediator *globalMediator = nullptr;
map<unsigned long, size_t> emptyCountMap;

std::atomic_bool b;

void RegisterRemote(ORB_SLAM2::System *SLAM, const string& host, const unsigned int &dispatchPort) {
    SLAM->GetMap()->TryConnect(SLAM);

    // get client id and port from the server
    info("Registering remote client");
    auto const tup = SLAM->GetMap()->GetClientService()->Register(host, dispatchPort);
    unsigned long id = std::get<0>(tup);
    unsigned int port = std::get<1>(tup);

    if (id == -1 || port == -1) {
        error("Failed to register remote client");
        exit(2);
    }
    info("Registered client with id: " + to_string(id) + " and port: " + to_string(port));

    // set id for the SLAM system
    SLAM->GetMap()->SetId(id);
    SLAM->SetViewerTitle("Map Viewer " + id, "Frame Viewer " + id);

    info("Connecting to the data channel");
    // connect to data channel
    SLAM->GetMap()->GetClientService()->Connect(host, port);

    info("Connected to the data channel");
}

void UploadTrackingInfo(ORB_SLAM2::System *SLAM) {
    auto state = SLAM->GetSystemState();

    if (!state.bStable) {
        emptyCountMap[SLAM->GetMap()->mnId] = 0;
    }

    SLAM->GetMap()->GetClientService()->ReportState(state);
}

void UploadMap(ORB_SLAM2::System *SLAM) {
    info("receive map called");
    if (!SLAM || !SLAM->GetMap()) return;
    // simulate sending and receiving process

    FuncTimer();

    // get serialized result
    string result;
    SLAM->GetMap()->GetMapit()->Push(result);

    const auto contentEmpty = result.size() <= 60;
    // prevent endless sending
    if (contentEmpty) {
        emptyCountMap[SLAM->GetMap()->mnId] += 1;
    } else {
        emptyCountMap[SLAM->GetMap()->mnId] = 0;
    }

    // check empty content count
    if (emptyCountMap[SLAM->GetMap()->mnId] > 5) {
        // stop running
        b.store(false);
        return;
    }

    SLAM->GetMap()->GetClientService()->PushMap(result);
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

void Run(System * SLAM) {
    // b: whether to stop the program
    b.store(true);
    size_t n = 0;
    while (b.load()) {
        // sleep 500 ms
        std::this_thread::sleep_for(chrono::milliseconds(500));
        n += 1;

        // every 500ms, upload tracking info
        UploadTrackingInfo(SLAM);

        if (n == 4) {
            // every 2 seconds, upload map
            n = 0;
            UploadMap(SLAM);
        }
    }
}

int main(int argc, char **argv) {
    OptionParser op("SwarmMap - Scaling Up Real-time Collaborative Visual SLAM at the Edge\nSwarmMap Client");
    auto help_option   = op.add<Switch>("h", "help", "print this message");
    auto voc_option = op.add<Value<std::string>>("v", "voc", "path to vocabulary");
    // multiple dataset
    auto dataset_option = op.add<Value<std::string>>("d", "dataset", "path to dataset config file");
    auto log_level_option  = op.add<Value<std::string>>("l", "log", "log level: error/warn/info/debug", "debug");
    auto viewer_option  = op.add<Value<bool>>("u", "viewer", "use frame viewer", true);
    auto map_viewer_option  = op.add<Value<bool>>("m", "mapviewer", "use map viewer", true);
    op.parse(argc, argv);

    // print auto-generated help message
    if (help_option->is_set() || argc == 1) {
        cout << op << "\n";
        return 0;
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

    if (nDataset != 1) {
        error("single agent only support one dataset");
        return 2;
    }

    if (datasetType == "euroc") {
        file["TIMES"] >> timeFiles;
        if (imagePaths.size() != timeFiles.size()) {
            error("the image path number should be the same as time files");
            return 2;
        }
    }

    // Retrieve paths to images
    // vector of image paths
    vector<string> imageList;
    // vector of timestamps
    vector<double> timestampList;

    const auto imagePath = imagePaths[0];
    if (datasetType == "tum") {
        DataSetUtil::LoadTUM(string(imagePath), imageList, timestampList);
    } else if (datasetType == "euroc") {
        const auto timesFile = timeFiles[0];
        DataSetUtil::LoadEuRoC(string(imagePath), timesFile, imageList, timestampList);
    } else if (datasetType == "kitti") {
        DataSetUtil::LoadKITTI(string(imagePath), imageList, timestampList);
    }

    CLogger::SetLevel(log_level_option->value());

    bool use_viewer = viewer_option->value();
    bool use_map_viewer = map_viewer_option->value();

    auto pVoc = new ORBVocabulary();
    const auto vocFile = voc_option->value();
    pVoc->loadFromBinaryFile(vocFile);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System system(vocFile, settingsFile, ORB_SLAM2::System::MONOCULAR, use_viewer, use_map_viewer);

    ORB_SLAM2::System *SLAM = &system;

    string host = file["HOST"];
    unsigned int port = stoi(file["PORT"]);

    RegisterRemote(SLAM, host, port);

    auto tRetriever = thread(Run, SLAM);

//    // Vector for tracking time statistics
//    vector<double> vTimesTrack(nImages, 0);
//
//    info("-------");
//    info("Start processing sequence ...");
//    info("Images in the sequence: {}", nImages);
//

    // Main loop
    for (size_t ni = 0; ni < imageList.size(); ni++) {
        Timer tTrack("Total Track Time", false);

        const auto imageName = imageList[ni];
        // Read image from file
        double tframe = timestampList[ni];

        track(SLAM, imageName, tframe);
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


    if (use_viewer || use_map_viewer) {
        info("press any key to stop");
//        getchar();
    }

    // Stop all threads
    if (b.load()) {
        b.store(false);
        debug("wait tRetriever to stop");
        tRetriever.join();
    }

    SLAM->Shutdown();


    // Tracking time statistics
//    sort(vTimesTrack.begin(), vTimesTrack.end());
//    float totaltime = 0;
//    for (size_t ni = 0; ni < vTimesTrack.size(); ni++) {
//        totaltime += vTimesTrack[ni];
//    }
//    const auto trackSize = vTimesTrack.size();
//    info("median tracking time: {}", vTimesTrack[trackSize / 2]);
//    info("mean tracking time: {}", totaltime / trackSize);



    debug("client map {} keyframe count: {}", SLAM->GetMap()->mnId, SLAM->GetMap()->KeyFramesInMap());
    debug("client map {} mappoint count: {}", SLAM->GetMap()->mnId, SLAM->GetMap()->MapPointsInMap());
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory-" + GetCurrentTime() + "-" + std::to_string(SLAM->GetMap()->mnId) + ".txt");
    SLAM->SaveMap("map-client-" + std::to_string(SLAM->GetMap()->mnId) + ".bin");

    return 0;
}
