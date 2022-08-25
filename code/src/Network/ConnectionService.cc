//
// Created by Halcao on 2022/2/28.
//

#include "ConnectionService.h"
#include "AgentMediator.h"
#include "MapSlice.h"
#include "System.h"
#include "MapUpdater.h"
#include "WebSocket.h"
#include "MediatorScheduler.h"
#include "Mapit.h"
#include "BoostArchiver.h"
#include "CLogger.h"

namespace ORB_SLAM2 {
using tcp = boost::asio::ip::tcp;               // from <boost/asio/ip/tcp.hpp>
namespace websocket = boost::beast::websocket;  // from <boost/beast/websocket.hpp>
using work_guard_type = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;

//ConnectionService::ConnectionService(AgentMediator *pMediator) {
//    // server side
//
//    // first initialize the id dispatching service
//    if (!dispatchService.get()) {
//        auto const address = boost::asio::ip::make_address("0.0.0.0");
//        const int port = 10080;
//        boost::asio::io_context ioc{1};
//
//        // Create and launch a listening port
//        this->dispatchService = std::make_shared<WS::Server::listener>(ioc,
//                                                                     tcp::endpoint{address, port},
//                                                                     std::bind(&ConnectionService::OnRequest, this, std::placeholders::_1));
//        this->dispatchService->run();
//    }
//
//    this->mpMediator = pMediator;
//
//    const auto id = (unsigned int)(mpMediator->mnId);
//
//    unsigned int port = 2328 + id;
//    this->Bind(port);
//}

//ConnectionService::ConnectionService(System *pSLAM) {
//    // client side
//    this->mpSLAM = pSLAM;
//    const auto id = (unsigned int)(mpSLAM->GetMap()->mnId);
//
//    string host = "0.0.0.0";
//    unsigned int port = (unsigned int)(2328 + id - 1);
//    this->Connect(host, port);
//}

// Server binds the port
void ConnectionService::Bind(unsigned int port) {
    const auto id = mpMediator->mnId;

    mThread = new std::thread([this, port, id] {
        auto const address = boost::asio::ip::make_address("0.0.0.0");
        auto const threads = std::max<int>(1, 3);

        info("server {} start at port {}", id, port);

        // The io_context is required for all I/O
        boost::asio::io_context ioc{threads};

        // Create and launch a listening port
        this->serverService = std::make_shared<WS::Server::listener>(ioc,
                                                                     tcp::endpoint{address, port},
                                                                     std::bind(&ConnectionService::OnRequest, this, std::placeholders::_1));
        this->serverService->run();

        // Run the I/O service on the requested number of threads
        std::vector<std::thread> v;
        v.reserve(threads - 1);
        for(auto i = threads - 1; i > 0; --i) {
            v.emplace_back(
                    [&ioc] {
                        ioc.run();
                    });
        }

        ioc.run();
    });
}

// Client Connect to Server
void ConnectionService::Connect(const string &host, unsigned int port) {
    const auto id = (unsigned int)(mpSLAM->GetMap()->mnId);

    mThread = new std::thread([this, host, port, id] {
        auto const text = "Hello, world!";
        info("client {} connect to host {} port {}", id, host, port);

        // The io_context is required for all I/O
        boost::asio::io_context ioc;
        work_guard_type workGuard(ioc.get_executor());

        // Launch the asynchronous operation
        auto session = std::make_shared<WS::Client::session>(ioc,
                                                             std::bind(&ConnectionService::OnRequest, this, std::placeholders::_1));
        this->clientService = session;
        session->run(host.c_str(), std::to_string(port).c_str(), text);

        // Run the I/O service. The call will return when
        // the socket is closed.
        ioc.run();
    });
}


void ConnectionService::ReportState(const SystemState &state) {
    Request req;
    req.path = "ReportState";
    req.body = toString(state);

    this->SendRequest(req);
}

void ConnectionService::PushMap(const string &content) {
    Request req;
    req.path = "PushMap";
    req.body = content;

    this->SendRequest(req);
}

// server distributes the message to the client
void ConnectionService::DistributeMap(const std::string &content) {
    Request req;
    req.path = "DistributeMap";
    req.body = content;

    this->SendRequest(req);
}

void ConnectionService::SendRequest(const Request &req) {
    // serialize and send request
    std::string msg = ORB_SLAM2::toString(req);

//    if (isClient()) {
//        this->clientService->send(make_shared<std::string>(msg));
//    } else {
//        this->serverService->send(msg);
//    }
}

// handle received request
void ConnectionService::OnRequest(const std::string &msg) {
    if (msg.rfind("22", 0) != 0) {
        info("meta message: {}", msg);
        return;
    }


//    Request req;
//
//    try {
//        ORB_SLAM2::toObject<Request>(req, msg);
//    } catch (std::exception &e) {
//        error("parse request failed: {}", e.what());
//        return;
//    }
//
//    if (req.path == "DistributeMap") {
//        // update map on the client
//        if (mpSLAM) {
//            MapSlice slice;
//            MapUpdater::Deserialize(slice, req.body);
//            mpSLAM->GetMap()->UpdateMap(slice);
//        }
//    } else if (req.path == "ReportState") {
//        // update state on the server
//        SystemState state;
//
//        ORB_SLAM2::toObject<SystemState>(state, req.body);
//        mpMediator->SetState(state);
//    } else if (req.path == "PushMap") {
//        if (mpSLAM) {
//            // update map on the client
//            MapSlice slice;
//            MapUpdater::Deserialize(slice, req.body);
//            mpSLAM->GetMap()->GetMapit()->ReceivePush(slice);
//        } else if (mpMediator) {
//            // push the request into the queue
//            auto id = mpMediator->mnId;
//            MediatorScheduler::GetInstance().EnqueueRequest(id, req.body);
//        }
//    } else if (req.path == "GetId") {
//        const int id = Map::ClaimId();
//
//        // initialize corresponding listener.
////        dispatchService->send(id);
//    } else {
//        warn("unknown request: {}", req.path);
//    }
}

} // namespace ORB_SLAM2
