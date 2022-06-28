//
// Created by Halcao on 2020/5/26.
//

#include <CLogger.h>
#include "Timer.h"

namespace ORB_SLAM2 {
    Timer::Timer(std::string funcName_, bool _logTime) :
            logTime(_logTime), funcName(std::move(funcName_)), start(std::chrono::steady_clock::now()) {}

    Timer &Timer::globalInstance() {
        static Timer timer;
        timer.logTime = false;
        return timer;
    }

    double Timer::get() {
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::duration<double> >(end - start).count();
    }

    Timer::~Timer() {
        if (logTime) {
            if (isInfoLog) {
                info("{} track time: {}ms", funcName, get() * 1000);
            } else {
                trace("{} track time: {}ms", funcName, get() * 1000);
            }
        }
    }

    void Timer::setInfoLog(bool isInfoLog_) {
        this->isInfoLog = isInfoLog_;
    }
};