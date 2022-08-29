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

using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

// global variables
ORB_SLAM2::AgentMediator *globalMediator = nullptr;
unordered_map<unsigned long, size_t> emptyCountMap;
std::shared_ptr<WS::Server::listener> dispatchService;
thread *tDispatch;
unordered_map<unsigned long, ORB_SLAM2::AgentMediator *> mediators;
string settingsFile;
ORBVocabulary *pVoc;
bool use_map_viewer;
bool use_viewer;

string GetCurrentTime() {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%Y-%m-%d_%H:%M:%S", timeinfo);
    return string(buffer);
}

void DispatchId(const string& message) {
    info("new dispatch request with msg: {}", message);

    auto const id = Map::ClaimId();
    if (mediators.find(id) != mediators.end()) return;

    auto mediator = new ORB_SLAM2::AgentMediator(settingsFile, pVoc, false, use_viewer, use_map_viewer);
    mediators[id] = mediator;

    mediator->GetMap()->TryConnect(mediator);

    auto service = mediator->GetMap()->GetServerService();

    while (!service->isConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const auto port = service->GetPort();

    info("dispatching id: {}, port: {}", id, port);
    // msg format "id,port"
    const auto msg = to_string(id) + " " + to_string(port);

    dispatchService->send(make_shared<string>(msg));

}

// dispatch id and connection port for each client
void SetupDispatchService(const unsigned short port) {
    auto const address = boost::asio::ip::make_address("0.0.0.0");
    boost::asio::io_context ioc{1};

    // Create and launch a listening port
    dispatchService = std::make_shared<WS::Server::listener>(ioc,
                                                             tcp::endpoint{address, port},
                                                             DispatchId);
    info("Dispatch service started on port: " + to_string(port));

    dispatchService->run();
    ioc.run();
}

int main(int argc, char **argv) {
    OptionParser op("SwarmMap - Scaling Up Real-time Collaborative Visual SLAM at the Edge\nSwarmMap Server");
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

    CLogger::SetLevel(log_level_option->value());

    use_viewer = viewer_option->value();
    use_map_viewer = map_viewer_option->value();

    pVoc = new ORBVocabulary();
    const auto vocFile = voc_option->value();
    pVoc->loadFromBinaryFile(vocFile);

    // get dataset config set
    auto path = dataset_option->value();
    cv::FileStorage file(path, cv::FileStorage::READ);

    // get settings file path
    string settings = file["SETTING"];
    settingsFile = settings;

    // global mediator
    auto pVoc2 = new ORBVocabulary();
    pVoc2->loadFromBinaryFile(vocFile);

    globalMediator = new ORB_SLAM2::AgentMediator(settingsFile, pVoc2, true, use_viewer, use_map_viewer);

//    string host = file["HOST"];
    unsigned int port = stoi(file["PORT"]);

    // start dispatch service
    tDispatch = new thread(SetupDispatchService, port);
    tDispatch->join();

    while (getchar() != 'q') {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    if (use_viewer || use_map_viewer) {
        info("press any key to stop");
//        getchar();
    }

    for (auto &pair: mediators) {
        pair.second->Shutdown();
//        mediator->Shutdown();
    }

    tDispatch->join();

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

//    LandmarkScoring::Save("result" + GetCurrentTime());

    // Save camera trajectory
    for (size_t i = 0; i < mediators.size(); ++i) {
        auto mediator = mediators[i];

        debug("server map {} keyframe count: {}", mediator->GetMap()->mnId, mediator->GetMap()->KeyFramesInMap());
        debug("server map {} mappoint count: {}", mediator->GetMap()->mnId, mediator->GetMap()->MapPointsInMap());

        mediator->SaveMap("map-server-" + std::to_string(mediator->GetMap()->mnId) + ".bin");

//        delete SLAM;
//        delete mediator;
    }

    if (globalMediator) {
//        globalMediator->SaveMap("map-global-" + std::to_string(globalMediator->GetMap()->mnId) + ".bin");
        globalMediator->SaveMap("map-global.bin");
    }

    MapManager::SaveGlobalMap("map");
    return 0;
}
