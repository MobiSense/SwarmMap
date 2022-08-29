//
// Created by Halcao on 2022/8/25.
//

#ifndef SWARM_MAP_SERVERSERVICE_H
#define SWARM_MAP_SERVERSERVICE_H


#include <string>
#include <queue>
#include <thread>
//#include "SystemState.h"
#include "WebSocket.h"
//#include "ConnectionService.h"

namespace ORB_SLAM2 {
class AgentMediator;
struct SystemState;
using std::vector;

class ServerService: public ConnectionService {
public:
//    ServerService(AgentMediator *);
    explicit ServerService(AgentMediator *);
//    void Connect(const std::string &host, unsigned int port);
//    void ReportState(const SystemState &state);

    // server sends to client
    void DistributeMap(const std::string &content);

    unsigned int GetPort();
    bool isConnected() { return mbConnected; }
protected:
    // content to be sent
    std::queue<std::string> contentQueue;
    std::mutex mContentMutex;
    std::thread *mThread;
    bool mbConnected = false;
    AgentMediator *mpMediator{};

    std::shared_ptr<WS::Server::listener> service;

    void Bind(unsigned int port);
    void SendRequest(const Request &req);
    void OnRequest(const std::string &msg);
};
}


#endif //SWARM_MAP_SERVERSERVICE_H
