//
// Created by Halcao on 2022/8/25.
//

#ifndef SWARM_MAP_CLIENTSERVICE_H
#define SWARM_MAP_CLIENTSERVICE_H

#include <string>
#include <queue>
#include <thread>
#include "WebSocket.h"
//#include "System.h"
//#include "ConnectionService.h"
//#include "System.h"

namespace ORB_SLAM2 {
class System;
struct SystemState;
using std::vector;

class ClientService: public ConnectionService {
public:
    explicit ClientService(System *);
//    ClientService(System *);
    void Connect(const std::string &host, unsigned int port);
    void ReportState(const SystemState &state);

    // client sends to server
    void PushMap(const std::string &content);

protected:
    // content to be sent
    std::queue<std::string> contentQueue;
    bool mbConnected = false;
    std::mutex mContentMutex;
    std::thread *mThread;
    System *mpSLAM{};

    // Client connection service
    std::shared_ptr<WS::Client::session> service;
    void SendRequest(const Request &req);
    void OnRequest(const std::string &msg);
};
}

#endif //SWARM_MAP_CLIENTSERVICE_H
