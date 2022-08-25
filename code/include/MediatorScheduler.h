//
// Created by Halcao on 2020/12/20.
//

#ifndef EDGE_SLAM_MEDIATORSCHEDULER_H
#define EDGE_SLAM_MEDIATORSCHEDULER_H

#include <unordered_map>
#include <queue>
#include <functional>
#include <thread>
#include <mutex>
#include <MapSlice.h>

namespace ORB_SLAM2 {

class AgentMediator;
class System;
class KeyFrame;
class MapPoint;
using namespace std;

struct MediatorRequest {
    size_t src;
    size_t dst;
    MapSlice slice;
    AgentMediator *mediator;
    double contribScore;

    bool operator <(const MediatorRequest &other) const;
};

struct MediatorRequestComparator {
    bool operator()(const MediatorRequest *a, const MediatorRequest *b) {
        return *a < *b;
    }
};

class MediatorScheduler {
public:
    // TODO(halcao): make it static, not singleton
    static MediatorScheduler& GetInstance() {
        static MediatorScheduler instance;
        return instance;
    }

    MediatorScheduler(MediatorScheduler const&) = delete;
    void operator=(MediatorScheduler const&) = delete;

    void RegisterMediator(AgentMediator *mediator);
    AgentMediator *GetMediator(size_t id);
    void EnqueueRequest(unsigned long id, const string &message);

    KeyFrame *GetKeyFrame(unsigned long id);
    MapPoint *GetMapPoint(unsigned long id);

    void Run();
    void RequestStop();
    static void MapDistribute(AgentMediator *mediator);
public:
    AgentMediator *globalMediator = nullptr;
    std::function<System *(unsigned long)> getSLAMSystem;
private:
    MediatorScheduler();
    // TODO(halcao): make it static
    unordered_map<unsigned long, AgentMediator *> mediatorMap;
    unordered_map<unsigned long, thread *> threadPool;
    priority_queue<MediatorRequest *, std::vector<MediatorRequest *>, MediatorRequestComparator> queue;
    bool ShouldStop();
    bool mbShouldStop = false;
    std::mutex mMutexShouldStop;
    std::mutex mMutexQueue;
    std::thread *mptRunning;

    void ProcessRequest(const MediatorRequest *request) const;

    bool CheckQueueEmpty();

    MediatorRequest *FetchRequest();
};
}

#endif //EDGE_SLAM_MEDIATORSCHEDULER_H
