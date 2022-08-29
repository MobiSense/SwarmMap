//
// Created by Halcao on 2022/8/25.
//

#include "ServerService.h"
#include "SystemState.h"
//#include "MapSlice.h"
//#include "System.h"
#include "MapUpdater.h"
#include "AgentMediator.h"
//#include "WebSocket.h"
//#include "MediatorScheduler.h"
#include "Mapit.h"
#include "BoostArchiver.h"
#include "CLogger.h"
#include "MediatorScheduler.h"

namespace ORB_SLAM2 {
using tcp = boost::asio::ip::tcp;               // from <boost/asio/ip/tcp.hpp>
namespace websocket = boost::beast::websocket;  // from <boost/beast/websocket.hpp>
using work_guard_type = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;

ServerService::ServerService(AgentMediator *pMediator) {
    // server side
    this->mpMediator = pMediator;

    const auto id = (unsigned int) (mpMediator->mnId);

    unsigned int port = 2328 + id;
    this->Bind(port);
}

void ServerService::Bind(unsigned int port) {
    const auto id = mpMediator->mnId;


    mThread = new std::thread([this, port, id] {
        auto const address = boost::asio::ip::make_address("0.0.0.0");
        auto const threads = std::max<int>(1, 3);

        info("server {} start at port {}", id, port);

        // The io_context is required for all I/O
        boost::asio::io_context ioc{threads};

        // Create and launch a listening port
        this->service = std::make_shared<WS::Server::listener>(ioc,
                                                               tcp::endpoint{address, (unsigned short)port},
                                                               std::bind(&ServerService::OnRequest, this,
                                                                         std::placeholders::_1));
        this->service->run();

        // TODO(hao): lock
        this->mbConnected = true;
        // Run the I/O service on the requested number of threads
        std::vector<std::thread> v;
        v.reserve(threads - 1);
        for (auto i = threads - 1; i > 0; --i) {
            v.emplace_back(
                    [&ioc] {
                        ioc.run();
                    });
        }

        ioc.run();
    });
}

// get the port number service using
unsigned int ServerService::GetPort() {
    return this->service->get_port();
}

// server distributes the message to the client
void ServerService::DistributeMap(const std::string &content) {
    Request req;
    req.path = "DistributeMap";
    req.body = content;

    this->SendRequest(req);
}

void ServerService::SendRequest(const Request &req) {
    // serialize and send request
    std::string msg = ORB_SLAM2::toString(req);

    this->service->send(make_shared<std::string>(msg));
}

void ServerService::OnRequest(const std::string &msg) {
    if (msg.rfind("22", 0) != 0) {
        info("meta message: {}", msg);
        return;
    }

    Request req;

    try {
        ORB_SLAM2::toObject<Request>(req, msg);
    } catch (std::exception &e) {
        error("parse request failed: {}", e.what());
        return;
    }

    if (req.path == "ReportState") {
        // update state on the server
        SystemState state;

        ORB_SLAM2::toObject<SystemState>(state, req.body);
        mpMediator->SetState(state);
    } else if (req.path == "PushMap") {
        if (mpMediator) {
            // push the request into the queue
            auto id = mpMediator->mnId;
            MediatorScheduler::GetInstance().EnqueueRequest(id, req.body);
        }
    } else {
        warn("unknown request: {}", req.path);
    }
}

}
// namespace ORB_SLAM2