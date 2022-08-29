//
// Created by Halcao on 2022/8/25.
//

#include <tuple>
#include <sstream>
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
namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using work_guard_type = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;

ClientService::ClientService(ORB_SLAM2::System *pSLAM) {
    // client side
    this->mpSLAM = pSLAM;
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

std::tuple<unsigned long, unsigned int> ClientService::Register(const std::string &host, unsigned int port) {
    try {
        auto const text = "Hello, world!";

        // The io_context is required for all I/O
        net::io_context ioc;

        // These objects perform our I/O
        tcp::resolver resolver{ioc};
        websocket::stream<tcp::socket> ws{ioc};

        info("client register to host {} port {}", host, port);
        // Look up the domain name
        auto const results = resolver.resolve(host.c_str(), std::to_string(port).c_str());

        // Make the connection on the IP address we get from a lookup
        net::connect(ws.next_layer(), results.begin(), results.end());

        // Set a decorator to change the User-Agent of the handshake
        ws.set_option(websocket::stream_base::decorator(
                [](websocket::request_type& req)
                {
                    req.set(http::field::user_agent,
                            std::string(BOOST_BEAST_VERSION_STRING) +
                            " websocket-client-coro");
                }));

        // Perform the websocket handshake
        ws.handshake(host, "/");

        // Send the message
        ws.write(net::buffer(std::string(text)));

        // This buffer will hold the incoming message
        beast::flat_buffer buffer;

        // Read a message into our buffer
        ws.read(buffer);

        // Close the WebSocket connection
        ws.close(websocket::close_code::normal);

        // If we get here then the connection is closed gracefully

        // The make_printable() function helps print a ConstBufferSequence
        std::cout << beast::make_printable(buffer.data()) << std::endl;

        const std::string msg = boost::beast::buffers_to_string(buffer.data());
        std::istringstream is(msg);
        unsigned long id;
        unsigned int port;
        is >> id >> port;

        return std::make_tuple(id, port);

    } catch(std::exception const& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return std::make_tuple(-1, -1);
    }
}
}