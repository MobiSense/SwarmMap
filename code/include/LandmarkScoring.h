//
// Created by Halcao on 2020/7/31.
//

#ifndef EDGE_SLAM_LANDMARKSCORING_H
#define EDGE_SLAM_LANDMARKSCORING_H

#include <vector>
#include <map>
#include "MapSlice.h"

namespace ORB_SLAM2 {

class KeyFrame;
class MapPoint;

struct MapPointScoreItem {
    unsigned long mnId = 0;
    // - landmark creation time
    // - 创建时间
    double createdTime = 0;
    // - score item created time
    // - 评分项创建时间
    double timestamp = 0;
    // - landmark last tracked time
    // - 上次被观测到的时间
    double lastTrackedTime = 0;
    // - observed keyframe count
    // - 被观察的关键帧数量
    int observedCount = 0;
    // - number of the map-point was modified or updated in the last round
    int updateFreq = 0;
    // - velocity when map-point was created
    double velocity = 0;
    // - observed keyframes total tracked length
    // - 被观察的关键帧的路径长度之和
    double trackedLength = 0;
    // - max observing distance between keyframes
    // - 观测着landmark的关键帧之间最大的距离
    double maxDistance = 0;
    // - max observing keyframes angle
    // - 观测着landmark的关键帧之间最大的夹角
    double maxAngle = 0;
    // - number of evaluation round
    // - 被评估的轮数
    int round = 0;
    // - total score
    double score = 0;
};

struct KeyFrameScoreItem {
    unsigned long mnId = 0;
    // - keyframe creation time
    // - 创建时间
    double createdTime = 0;
    // - score item created time
    // - 评分项创建时间
    double timestamp = 0;
    // - keyframe speed
    double speed = 0;
    // - matched ratio: matched / total
    double matchedRatio = 0;
    // - total points
    double totalPoints = 0;
    double pointScore = 0;
    double connectedKF = 0;
    double bestCovisibleCount = 0;
    // - number of evaluation round
    // - 被评估的轮数
    int round = 0;
    // - total score
    double score = 0;
};

class LandmarkScoring {
public:

    static std::map<unsigned long, std::vector<MapPointScoreItem> > mappointMap;
    static std::map<unsigned long, std::vector<KeyFrameScoreItem>> keyframeMap;
    static void Save(const std::string &filename);

    static int round;

    static double GetRequestPriority(const ORB_SLAM2::MapSlice &slice);
    static std::vector<double> Rank(const MapSlice &slice);
    static std::vector<double> GetScores(const std::vector<KeyFrame *>KFs);
    static std::vector<double> GetScores(const std::vector<MapPoint *> MPs);
private:
    double startTime;
    // mp related
    static std::map<unsigned long, double> maxTrackedLength;
    static std::map<unsigned long, double> maxAngle;
    static std::map<unsigned long, double> maxDistance;
    static std::map<unsigned long, double> maxVelocity;
    static std::map<unsigned long, double> maxUpdateFreq;
    static std::map<unsigned long, double> maxObservedCount;

    static double maxMG;
    static double maxMS;


    static void CalcFinalScore(MapPointScoreItem &item, unsigned long mapId);
};
}

#endif //EDGE_SLAM_LANDMARKSCORING_H
