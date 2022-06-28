//
// Created by Halcao on 2020/5/26.
//

#ifndef EDGE_SLAM_TIMER_H
#define EDGE_SLAM_TIMER_H
#include<chrono>

namespace ORB_SLAM2 {
#define FuncTimer() Timer t(__func__, true);

    class Timer {
    public:
        explicit Timer(std::string funcName_="", bool logTime=true);
        ~Timer();

        double get();
        void setInfoLog(bool isInfoLog_);
        static Timer &globalInstance();
    private:
        bool logTime = true;
        bool isInfoLog = false;
        std::string funcName;
        std::chrono::steady_clock::time_point start;
    };}

#endif //EDGE_SLAM_TIMER_H
