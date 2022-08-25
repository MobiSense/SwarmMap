//
// Created by Halcao on 2022/4/3.
//

#ifndef EDGE_SLAM_WEBSOCKET_H
#define EDGE_SLAM_WEBSOCKET_H

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/strand.hpp>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <string>
#include <utility>

namespace ORB_SLAM2 {

struct Request {
    id_t src;
    id_t dst;
    std::string path;
    std::string body;

    std::string toString() const;
};

class ConnectionService {

};

namespace WS {

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

// Report a failure
void
fail(boost::system::error_code ec, char const *what);

namespace Client {
// Sends a WebSocket message and prints the response
class session : public std::enable_shared_from_this<session> {
    tcp::resolver resolver_;
    websocket::stream <beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    std::vector<std::shared_ptr<const std::string>> queue;
    std::string host_;
    std::function<void(const std::string&)> on_message;

public:
    // Resolver and socket require an io_context
    explicit
    session(net::io_context &ioc, std::function<void(const std::string&)> on_message)
            : resolver_(net::make_strand(ioc)), ws_(net::make_strand(ioc)), on_message(std::move(on_message)) {
    }

    // Start the asynchronous operation
    void
    run(
            char const *host,
            char const *port,
            __attribute__((unused)) char const *text) {
        // Save these for later
        host_ = host;

        // Look up the domain name
        resolver_.async_resolve(
                host,
                port,
                beast::bind_front_handler(
                        &session::on_resolve,
                        shared_from_this()));
    }

    void
    on_resolve(
            beast::error_code ec,
            tcp::resolver::results_type results) {
        if (ec)
            return fail(ec, "resolve");

        // Set the timeout for the operation
        beast::get_lowest_layer(ws_).expires_after(std::chrono::seconds(30));

        // Make the connection on the IP address we get from a lookup
        beast::get_lowest_layer(ws_).async_connect(
                results,
                beast::bind_front_handler(
                        &session::on_connect,
                        shared_from_this()));
    }

    void
    on_connect(beast::error_code ec, tcp::resolver::results_type::endpoint_type ep) {
        if (ec)
            return fail(ec, "connect");

        // Turn off the timeout on the tcp_stream, because
        // the websocket stream has its own timeout system.
        beast::get_lowest_layer(ws_).expires_never();

        // Set suggested timeout settings for the websocket
        ws_.set_option(
                websocket::stream_base::timeout::suggested(
                        beast::role_type::client));

        // Set a decorator to change the User-Agent of the handshake
        ws_.set_option(websocket::stream_base::decorator(
                [](websocket::request_type &req) {
                    req.set(http::field::user_agent,
                            std::string(BOOST_BEAST_VERSION_STRING) +
                            " websocket-client-async");
                }));

        // update the host string. This will provide the value of the
        // host HTTP header during the websocket handshake
        // the guide references: https://tools.ietf.org/html/rfc7230#section-5.4
        host_ += ':' + std::to_string(ep.port());


        // Perform the websocket handshake
        ws_.async_handshake(host_, "/",
                            beast::bind_front_handler(
                                    &session::on_handshake,
                                    shared_from_this()));
    }

    void
    on_handshake(beast::error_code ec) {
        if (ec)
            return fail(ec, "handshake");

        buffer_.consume(buffer_.size());
        net::post(ws_.get_executor(), beast::bind_front_handler(&session::on_read, shared_from_this(), ec, 5));
        std::cout << "Handshake successful." << std::endl;

//        // Send the message
//        ws_.async_write(
//                net::buffer(text_),
//                beast::bind_front_handler(
//                        &session::on_write,
//                        shared_from_this()));
    }

    void
    on_write(
            beast::error_code ec,
            std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        if (ec)
            return fail(ec, "write");

        queue.erase(queue.begin());

        // send the message if any
        if (!queue.empty()) {
            ws_.async_write(net::buffer(*queue.front()),
                           beast::bind_front_handler(&session::on_write, shared_from_this()));
        }
    }

    void
    on_read(
            beast::error_code ec,
            std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        if (ec)
            return fail(ec, "read");

        const std::string msg = boost::beast::buffers_to_string(buffer_.data());
        buffer_.consume(buffer_.size());
//        std::cout << "Client: Received message: " << msg << std::endl;
//        std::cout << "Client: Received message: " << std::endl;
        on_message(msg);

        ws_.async_read(buffer_, beast::bind_front_handler(&session::on_read, shared_from_this()));
    }

    void send(const std::shared_ptr<const std::string>& ss)
    {
//        std::cout <<"Attempting to send: " << *ss << std::endl;
//        std::cout <<"Attempting to send: " << std::endl;
        // post our work to the strand, this ensures
        // that members of 'this' will not be accessed concurrently
        net::post(ws_.get_executor(), beast::bind_front_handler(&session::on_send, shared_from_this(), ss));
    }

    void on_send(const std::shared_ptr<const std::string>& ss)
    {
//        std::cout << "Sending message" << std::endl;
        queue.push_back(ss);

        // check if we're already writing
        if (queue.size() > 1) {
            return;
        }

        // since we are not writing, send this immediately
        ws_.async_write(net::buffer(*queue.front()),
                       beast::bind_front_handler(&session::on_write, shared_from_this()));
    }

    void close() {
        // Close the WebSocket connection
        ws_.async_close(websocket::close_code::normal,
                        beast::bind_front_handler(
                                &session::on_close,
                                shared_from_this()));
    }

    void
    on_close(beast::error_code ec) {
        if (ec)
            return fail(ec, "close");

        // If we get here then the connection is closed gracefully

        // The make_printable() function helps print a ConstBufferSequence
//        std::cout << beast::make_printable(buffer_.data()) << std::endl;
    }
};
}

namespace Server {
class session;

//------------------------------------------------------------------------------
// represents the shared server state
class shared_state {
    const std::string doc_root;

    // this mutex synchronizes all access to sessions
    std::mutex mutex;

    // keep a list of all the connected clients
    std::unordered_set<session *> sessions;

public:
    explicit shared_state(std::string doc_root) { (void)doc_root; };

    const std::string &getDocRoot() const noexcept {
        return doc_root;
    }

    void join(session *session) {
        std::lock_guard<std::mutex> lock(mutex);
        sessions.insert(session);
    };

    void leave(session *session) {
        std::lock_guard<std::mutex> lock(mutex);
        sessions.erase(session);
    };

    // broadcast a message to all websocket client sessions
    void send(const std::string& message);
};

// Echoes back all received WebSocket messages
class session : public std::enable_shared_from_this<session> {
    websocket::stream <beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    std::vector<std::shared_ptr<const std::string>> queue;
    std::shared_ptr<shared_state> state;
    std::function<void(std::string)> on_message;

public:
    // Take ownership of the socket
    explicit
    session(tcp::socket &&socket, std::shared_ptr<shared_state> state, std::function<void(std::string)> on_message)
            : ws_(std::move(socket)), state(std::move(state)), on_message(std::move(on_message)) {
    }

    ~session() {
        std::cout << "~session()" << std::endl;
        state->leave(this);
    }

    // Start the asynchronous operation
    void
    run() {
        // Set suggested timeout settings for the websocket
        ws_.set_option(
                websocket::stream_base::timeout::suggested(
                        beast::role_type::server));

        // Set a decorator to change the Server of the handshake
        ws_.set_option(websocket::stream_base::decorator(
                [](websocket::response_type &res) {
                    res.set(http::field::server,
                            std::string(BOOST_BEAST_VERSION_STRING) +
                            " websocket-server-async");
                }));

        // Accept the websocket handshake
        ws_.async_accept(
                beast::bind_front_handler(
                        &session::on_accept,
                        shared_from_this()));
    }

    void
    on_accept(beast::error_code ec) {
        if (ec)
            return fail(ec, "accept");

        state->join(this);
        // Read a message
        do_read();
    }

    void
    do_read() {
        // Read a message into our buffer
        ws_.async_read(
                buffer_,
                beast::bind_front_handler(
                        &session::on_read,
                        shared_from_this()));
    }

    void
    on_read(
            beast::error_code ec,
            std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        // This indicates that the session was closed
        if (ec == websocket::error::closed)
            return;

        if (ec)
            fail(ec, "read");

        // Echo the message
//        ws_.text(ws_.got_text());
        const std::string msg = boost::beast::buffers_to_string(buffer_.data());
//        std::cout << "Server: Received message: " << msg << std::endl;
//        std::cout << "Server: Received message" << std::endl;

        // callback
        on_message(msg);

        buffer_.consume(buffer_.size());

        ws_.async_read(buffer_, beast::bind_front_handler(&session::on_read, shared_from_this()));
    }

    void send(const std::shared_ptr<const std::string>& ss) {
        // post our work to the strand, this ensures
        // that members of 'this' will not be accessed concurrently
        net::post(ws_.get_executor(), beast::bind_front_handler(&session::on_send, shared_from_this(), ss));
    }


    void on_send(const std::shared_ptr<const std::string>& ss)
    {
        queue.push_back(ss);

        // check if we're already writing
        if (queue.size() > 1) {
            std::cout << "queue.size() > 1" << std::endl;
            return;
        }

        // since we are not writing, send this immediately
        ws_.async_write(net::buffer(*queue.front()),
                       beast::bind_front_handler(&session::on_write, shared_from_this()));
    }

    void
    on_write(
            beast::error_code ec,
            std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        if (ec)
            return fail(ec, "write");

        // remove the string from the queue
        queue.erase(queue.begin());

        // send the message if any
        if (!queue.empty()) {
            ws_.async_write(net::buffer(*queue.front()),
                           beast::bind_front_handler(&session::on_write, shared_from_this()));
        }
    }
};


//------------------------------------------------------------------------------

// Accepts incoming connections and launches the sessions
class listener: public std::enable_shared_from_this<listener> {
    net::io_context &ioc_;
    tcp::acceptor acceptor_;
    std::shared_ptr<shared_state> state;
    std::function<void(const std::string&)> on_message;

public:
    listener(
            net::io_context &ioc,
            tcp::endpoint endpoint,
            std::function<void(const std::string&)> on_message)
            : ioc_(ioc), acceptor_(ioc), state(std::make_shared<shared_state>("/")), on_message(std::move(on_message)) {
        beast::error_code ec;

        // Open the acceptor
        acceptor_.open(endpoint.protocol(), ec);
        if (ec) {
            fail(ec, "open");
            return;
        }

        // Allow address reuse
        acceptor_.set_option(net::socket_base::reuse_address(true), ec);
        if (ec) {
            fail(ec, "set_option");
            return;
        }

        // Bind to the server address
        acceptor_.bind(endpoint, ec);
        if (ec) {
            fail(ec, "bind");
            return;
        }

        // Start listening for connections
        acceptor_.listen(
                net::socket_base::max_listen_connections, ec);
        if (ec) {
            fail(ec, "listen");
            return;
        }
    }

    void send(const std::string& message) {
        state->send(message);
    }

    // Start accepting incoming connections
    void
    run() {
        do_accept();
    }

private:
    void
    do_accept() {
        // The new connection gets its own strand
        acceptor_.async_accept(
                net::make_strand(ioc_),
                beast::bind_front_handler(
                        &listener::on_accept,
                        shared_from_this()));
    }

    void get_message(const std::string& message) {
        on_message(message);
    }

    void
    on_accept(beast::error_code ec, tcp::socket socket) {
        if (ec) {
            fail(ec, "accept");
        } else {
            // Create the session and run it
            std::make_shared<session>(std::move(socket), state,
                                      std::bind(&listener::get_message, shared_from_this(), std::placeholders::_1))->run();
        }

        // Accept another connection
        do_accept();
    }
};

}
}
}
#endif //EDGE_SLAM_WEBSOCKET_H
