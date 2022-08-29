//
// Created by Halcao on 2022/4/3.
//

#include <spdlog/spdlog.h>
#include <thread>
#include <iostream>
#include <memory>
#include <chrono>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "WebSocket.h"
#include "BoostArchiver.h"

namespace ORB_SLAM2 {
std::string Request::toString() const {
    std::stringstream out;
    boost::archive::text_oarchive oa(out);
//        boost::archive::binary_oarchive oa(out);
    // start to serialize
    try {
        oa << this;
    } catch (boost::archive::archive_exception &e) {
    }

    return out.str();
}

namespace WS {
// Report a failure
void
fail(boost::system::error_code ec, char const *what) {
    std::cerr << what << ": " << ec.message() << "\n";
}
namespace Client {


}


namespace Server {

// broadcast a message to all websocket client sessions
void shared_state::send(const std::shared_ptr<const std::string>& ss) {
    // make a local list of all the weak pointers representing the sessions so that we an do the
    // actual sending without holding the mutex
    auto v = std::vector<session *>();
    {
        std::lock_guard<std::mutex> lock(mutex);
        v.reserve(sessions.size());
        for (auto p : sessions) {
            v.emplace_back(p);
        }
    }

    // for each session in our local list, try to acquire a strong pointer. If successful,
    // then send the message on that session
    for (const auto& wp : v) {
        if (!wp) continue;

//        std::cout << "Sent message: " << *ss << std::endl;
        wp->send(ss);
    }
};
}

}
}