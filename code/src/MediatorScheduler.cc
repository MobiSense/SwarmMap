//
// Created by Halcao on 2020/12/20.
//

#include <MediatorScheduler.h>
#include <AgentMediator.h>
#include <MapUpdater.h>
#include <MapManager.h>
#include <Converter.h>
#include <LandmarkScoring.h>
#include <Optimizer.h>
#include <Mapit.h>
#include <ConnectionService.h>
#include <MapEnhancer.h>
#include <CLogger.h>

namespace ORB_SLAM2 {

bool MediatorRequest::operator<(const MediatorRequest &other) const {
    // STS: 3-virtual queues implementation

    const auto stateA = this->mediator->GetState();
    const auto stateB = other.mediator->GetState();

    // 1st priority: Lost Handling Queue
    // state LOST has a higher priority

    if (!stateA.bStable) return true;
    if (!stateB.bStable) return true;

    // 2nd priority: Lost Prevention Queue
    // state velocity burst has a higher priority
    if (stateA.bVelocityBurst) return true;
    if (stateB.bVelocityBurst) return true;

    // state with few tracked map points has a higher priority
    if (stateA.nTracked < 20) return true;
    if (stateB.nTracked < 20) return true;

    // 3rd priority: Map Enrichment Queue
    return this->contribScore > other.contribScore;
}

MediatorScheduler::MediatorScheduler() {
    this->mptRunning = new std::thread(&MediatorScheduler::Run, this);
}

void MediatorScheduler::RegisterMediator(AgentMediator *mediator) {
    mediatorMap[mediator->mnId] = mediator;

    if (mediator->isGlobal()) {
        assert(this->globalMediator == nullptr);
        this->globalMediator = mediator;
    }
}

void MediatorScheduler::EnqueueRequest(unsigned long id, const string &message) {
    // TODO(halcao): this path should be called in multiple thread
    auto mediator = mediatorMap[id];

    // TODO(halcao): get to know why global will return
//    if (mediator == nullptr || mediator->isGlobal() || client == nullptr) return;
    if (mediator == nullptr) return;

    MapSlice slice;
    MapUpdater::Deserialize(slice, message);

    // STS-related score
    const double score = LandmarkScoring::GetRequestPriority(slice);
    // build up a request object
    auto request = new MediatorRequest {
            .src = id,
            .dst = id,
            .slice = slice,
            .mediator = mediator,
            .contribScore = score
    };

    std::unique_lock<mutex> lock(mMutexQueue);
    // enqueue request
    this->queue.push(request);
}

bool MediatorScheduler::CheckQueueEmpty() {
    std::unique_lock<mutex> lock(mMutexQueue);
    return this->queue.empty();
}

MediatorRequest *MediatorScheduler::FetchRequest() {
    std::unique_lock<mutex> lock(mMutexQueue);
    info("queue start to process with size {}", this->queue.size());
    auto request = this->queue.top();
    this->queue.pop();
    return request;
}

void MediatorScheduler::ProcessRequest(const MediatorRequest *request) const {
//    request->mediator->GetMap()->UpdateMap(request->slice);
    request->mediator->GetMap()->GetMapit()->ReceivePush(request->slice);

    if (request->slice.MPs.empty() || request->slice.KFs.empty()) return;

    if (globalMediator && globalMediator != request->mediator) {
        globalMediator->CheckOverlapCandidates(request->mediator);
    }

    // rank the request
    LandmarkScoring::Rank(request->slice);

    // MBP: generate a virtual keyframe
    auto kf = MapEnhancer::GetVirtualKeyFrame(request->slice);
    if (kf) {
        request->mediator->GetMap()->AddKeyFrame(kf);
    }

    auto const mediatorMap = request->mediator->GetMap();
    if (mediatorMap->KeyFramesInMap() > 1) {
        // MBP: compress the map
        MapEnhancer::Compress(mediatorMap);

        bool stopGBA = false;
        Optimizer::GlobalBundleAdjustment(request->mediator->GetMap(), 10, &stopGBA, 0, false);
    }

    MapDistribute(request->mediator);

//    delete request;
}

void MediatorScheduler::Run() {
    while (!ShouldStop()) {
        // dequeue request
        if (CheckQueueEmpty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        auto request = FetchRequest();

        // handle request
        this->ProcessRequest(request);
    }
}

void MediatorScheduler::RequestStop() {
    std::unique_lock<mutex> lock(mMutexShouldStop);
    mbShouldStop = true;
}

bool MediatorScheduler::ShouldStop() {
    std::unique_lock<mutex> lock(mMutexShouldStop);
    return mbShouldStop;
}

void MediatorScheduler::MapDistribute(ORB_SLAM2::AgentMediator *mediator) {
    auto group = MapManager::GetGroup(mediator->GetMap());
    auto clientMapId = mediator->GetMap()->mnId;
    Map *pMap = nullptr;

    pMap = mediator->GetMap();

    if (!pMap) return;

    // pMap is not base, convert its coordinate into base(use it's global pos)
    auto KFs = pMap->GetAllKeyFrames();

    auto t12 = mediator->GetMap()->mTwl.inverse() * pMap->mTwl;

    // TODO(halcao): send only selected frame poses
    for (auto &kf: KFs) {
        if (kf->isBad()) continue;

        cv::Mat newPose = kf->GetPose() * Converter::toCvMat(t12.inverse()) * t12.scale();
        pMap->AddUpdate(new KeyFrameUpdate<cv::Mat>(kf->mnId, "SetPose", newPose));
    }

    auto MPs = pMap->GetAllMapPoints();
    for (auto &mp: MPs) {
        if (mp->isBad()) continue;

        auto pos = Converter::toCvMat(t12.map(Converter::toVector3d(mp->GetWorldPos())));
        pMap->AddUpdate(new MapPointUpdate<cv::Mat>(
                mp->mnId, "SetWorldPos", pos));
        // TODO(halcao): if mp is sent but set bad by the server, how to update in client?
        if (mp->mbToBeSerialized && mp->isBad()) {
            warn("mp {} is bad but will be sent back", mp->mnId);
        }
    }


    // TODO(halcao): test the following code
    string result;
    MapSlice slice1;
    pMap->ArchiveMap(slice1);
    MapUpdater::Serialize(slice1, result);

    pMap->GetConnectionService()->DistributeMap(result);
//    // TODO(halcao): network
//    MapSlice slice2;
//    MapUpdater::Deserialize(slice2, result);
//    SLAM->GetMap()->UpdateMap(slice2);
}

KeyFrame *MediatorScheduler::GetKeyFrame(const unsigned long id) {
    // find in global mediator
    if (globalMediator) {
        auto kf = globalMediator->GetMap()->GetKeyFrame(id, true);
        if (kf) return kf;
    }

    // find in other mediator
    for (auto &item: mediatorMap) {
        auto &mediator = item.second;
        if (mediator->isGlobal()) continue;

        auto kf = mediator->GetMap()->GetKeyFrame(id, true);
        if (kf) {
            return kf;
        }
    }

    return nullptr;
}

MapPoint *MediatorScheduler::GetMapPoint(const unsigned long id) {
    if (globalMediator) {
        // find in global mediator
        auto mp = globalMediator->GetMap()->GetMapPoint(id, true);
        if (mp) return mp;
    }

    // find in other mediator
    for (auto &item: mediatorMap) {
        auto &mediator = item.second;
        if (mediator->isGlobal()) continue;
        auto mp = mediator->GetMap()->GetMapPoint(id, true);
        if (mp) {
            return mp;
        }
    }

    return nullptr;
}

AgentMediator *MediatorScheduler::GetMediator(size_t id) {
    return mediatorMap[id];
}
}