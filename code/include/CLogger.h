//
// Created by Halcao on 2020/4/26.
//

#ifndef EDGE_SLAM_CLOGGER_H
#define EDGE_SLAM_CLOGGER_H

#include <iostream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace ORB_SLAM2 {
using std::string;
class CLogger {
public:
    static std::unique_ptr<CLogger> hclogger;

public:
    static CLogger &GetInstance() {
        static CLogger instance; // Guaranteed to be destroyed.
        // Instantiated on first use.
        return instance;
    }

    static std::shared_ptr<spdlog::logger> GetSpdlogger() {
        return GetInstance().spdlogger_;
    }

    static void SetLevel(const string level) {
        auto logger = GetSpdlogger();
        if (level == "trace") {
            logger->set_level(spdlog::level::trace);
        } else if (level == "debug") {
            logger->set_level(spdlog::level::debug);
        } else if (level == "info") {
            logger->set_level(spdlog::level::info);
        } else if (level == "warn") {
            logger->set_level(spdlog::level::warn);
        } else if (level == "err") {
            logger->set_level(spdlog::level::err);
        } else if (level == "critical") {
            logger->set_level(spdlog::level::critical);
        }
    }

    void SetSpdlogger(std::shared_ptr<spdlog::logger> &spdlogger) {
        spdlogger_ = std::move(spdlogger);
    }

    // assignment is private.
    CLogger &operator=(CLogger const &) = delete;

    CLogger(CLogger const &) = delete;

private:
    CLogger() {
        try {
            std::vector<spdlog::sink_ptr> sinks;
            sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
//        sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>
//                                (filename, "log", size_mb * 1024L * 1024, num_files));
            auto spdlogger =
                    std::make_shared<spdlog::logger>("SwarmMap", begin(sinks), end(sinks));
            spdlogger->set_pattern("[%H:%M:%S][%t]%^[%L]%v%$");
            spdlogger->set_level(spdlog::level::info);
            SetSpdlogger(spdlogger);

            spdlog::set_default_logger(this->spdlogger_);
        } catch (const spdlog::spdlog_ex &ex) {
            std::cout << "Log initialization failed: " << ex.what() << std::endl;
        }
    };

    ~CLogger() {
        spdlog::drop_all();
    }

    // Spd logger that does actual logging.
    std::shared_ptr<spdlog::logger> spdlogger_;
};

#ifndef CLOGGER_SPD_LOG
#define ClOGGER_SPD_LOG
#define trace(fmt, ...) \
do { \
    CLogger::GetSpdlogger()->trace("[{}:{}] " fmt, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define debug(fmt, ...) \
do { \
    CLogger::GetSpdlogger()->debug("[{}:{}] " fmt, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define info(fmt, ...) \
do { \
    CLogger::GetSpdlogger()->info("[{}:{}] " fmt, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define warn(fmt, ...) \
do { \
    CLogger::GetSpdlogger()->warn("[{}:{}] " fmt, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define error(fmt, ...) \
do { \
    CLogger::GetSpdlogger()->error("[{}:{}] " fmt, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define critical(fmt, ...) \
do { \
    CLogger::GetSpdlogger()->critical("[{}:{}] " fmt, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

};
#endif // ClOGGER_SPD_LOG
#endif //EDGE_SLAM_CLOGGER_H
