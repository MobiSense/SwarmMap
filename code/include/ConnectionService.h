//
// Created by Halcao on 2022/2/28.
//

#ifndef EDGE_SLAM_CONNECTIONSERVICE_H
#define EDGE_SLAM_CONNECTIONSERVICE_H

#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include "WebSocket.h"
#include "SystemState.h"

namespace ORB_SLAM2 {

class ClientMediator;
class System;
using std::vector;

class ConnectionService {
public:
//ConnectionService();
    explicit ConnectionService(ClientMediator *pMediator);

    explicit ConnectionService(System *pSLAM);

    ~ConnectionService();
    void Connect(const std::string &host, unsigned int port);
    void Disconnect();
    void ReportState(const SystemState &state);
    void ReportLocation(const cv::Mat &location);

    // client sends to server
    void PushMap(const std::string &content);

    // server sends to client
    void DistributeMap(const std::string &content);
    bool isClient() { return mpSLAM != nullptr; }
    bool isConnected() { return mbConnected; }
private:
    // content to be sent
    std::queue<std::string> contentQueue;
    System *mpSLAM{};
    ClientMediator *mpMediator{};
    bool mbConnected = false;
    std::mutex mContentMutex;
    std::thread *mThread;

    // Server connection service
    std::shared_ptr<WS::Server::listener> serverService;
    // Client connection service
    std::shared_ptr<WS::Client::session> clientService;

    std::string GetNewContent();

    void AddNewContent(std::string &content);

    void Run();

    void Bind(unsigned int port);

    void SendRequest(const Request &req);

    void OnRequest(const std::string &msg);
};
}
#endif //EDGE_SLAM_CONNECTIONSERVICE_H
