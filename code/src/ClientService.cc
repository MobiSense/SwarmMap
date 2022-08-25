//
// Created by Halcao on 2022/8/25.
//

#include "ClientService.h"
#include "SystemState.h"
//#include "MapSlice.h"
#include "MapUpdater.h"
#include "System.h"
//#include "WebSocket.h"
//#include "MediatorScheduler.h"
#include "Mapit.h"
#include "BoostArchiver.h"
#include "CLogger.h"

namespace ORB_SLAM2 {
using std::vector;
using tcp = boost::asio::ip::tcp;               // from <boost/asio/ip/tcp.hpp>
namespace websocket = boost::beast::websocket;  // from <boost/beast/websocket.hpp>
using work_guard_type = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;

ClientService::ClientService(ORB_SLAM2::System *pSLAM) {
    // client side
    this->mpSLAM = pSLAM;
    const auto id = (unsigned int)(mpSLAM->GetMap()->mnId);

    std::string host = "0.0.0.0";
    unsigned int port = (unsigned int)(2328 + id - 1);
    this->Connect(host, port);
}

// Client Connect to Server
void ClientService::Connect(const std::string &host, unsigned int port) {
    const auto id = (unsigned int)(mpSLAM->GetMap()->mnId);

    mThread = new std::thread([this, host, port, id] {
        auto const text = "Hello, world!";
        info("client {} connect to host {} port {}", id, host, port);

        // The io_context is required for all I/O
        boost::asio::io_context ioc;
        work_guard_type workGuard(ioc.get_executor());

        // Launch the asynchronous operation
        auto session = std::make_shared<WS::Client::session>(ioc,
                                                             std::bind(&ClientService::OnRequest, this, std::placeholders::_1));
        this->service = session;
        session->run(host.c_str(), std::to_string(port).c_str(), text);

        // Run the I/O service. The call will return when
        // the socket is closed.
        ioc.run();
    });
}

void ClientService::ReportState(const SystemState &state) {
    Request req;
    req.path = "ReportState";
    req.body = toString(state);

    this->SendRequest(req);
}

void ClientService::PushMap(const string &content) {
    Request req;
    req.path = "PushMap";
    req.body = content;

    this->SendRequest(req);
}

void ClientService::SendRequest(const Request &req) {
    // serialize and send request
    std::string msg = ORB_SLAM2::toString(req);

    this->service->send(make_shared<std::string>(msg));
}

void ClientService::OnRequest(const std::string &msg) {
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

    if (req.path == "DistributeMap") {
        // update map on the client
        if (mpSLAM) {
            MapSlice slice;
            MapUpdater::Deserialize(slice, req.body);
            mpSLAM->GetMap()->UpdateMap(slice);
        }
    } else if (req.path == "PushMap") {
        if (mpSLAM) {
            // update map on the client
            MapSlice slice;
            MapUpdater::Deserialize(slice, req.body);
            mpSLAM->GetMap()->GetMapit()->ReceivePush(slice);
        }
    }
}
}