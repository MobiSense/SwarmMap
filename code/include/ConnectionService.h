//
// Created by Halcao on 2022/2/28.
//

#ifndef EDGE_SLAM_CONNECTIONSERVICE_H
#define EDGE_SLAM_CONNECTIONSERVICE_H

//#include <string>
//#include <queue>
//#include <mutex>
//#include <thread>
//#include "WebSocket.h"
//#include "SystemState.h"
//#include "ClientService.h"
//#include "ServerService.h"

namespace ORB_SLAM2 {

//class AgentMediator;
//class System;
//using std::vector;
//using tcp = boost::asio::ip::tcp;               // from <boost/asio/ip/tcp.hpp>
//namespace websocket = boost::beast::websocket;  // from <boost/beast/websocket.hpp>
//using work_guard_type = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;
//
//template<class Session>
//class ConnectionService {
//public:
////ConnectionService();
//    explicit ConnectionService(AgentMediator *pMediator);
//
//    explicit ConnectionService(System *pSLAM);
//
//    ConnectionService();
//
//    ~ConnectionService();
//    void Connect(const std::string &host, unsigned int port);
//    void Disconnect();
//
//
//protected:
//    // content to be sent
//    std::queue<std::string> contentQueue;
//    System *mpSLAM{};
//    AgentMediator *mpMediator{};
//    bool mbConnected = false;
//    std::mutex mContentMutex;
//    std::thread *mThread;
//
//    // Server dispatch id and corresponding service port
//    std::shared_ptr<WS::Server::listener> dispatchService;
//
//    // Server connection service
//    std::shared_ptr<WS::Server::listener> serverService;
//    // Client connection service
//    std::shared_ptr<WS::Client::session> clientService;
//
//    std::shared_ptr<Session> service;
//    std::string GetNewContent();
//
//    void AddNewContent(std::string &content);
//
//    void Run();
//
//    void Bind(unsigned int port);
//
//    void SendRequest(const Request &req);
//
//    void OnRequest(const std::string &msg);
//};
}
#endif //EDGE_SLAM_CONNECTIONSERVICE_H
