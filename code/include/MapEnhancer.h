//
// Created by zjl on 11/16/2020.
//

#ifndef EDGE_SLAM_MAPENHANCER_H
#define EDGE_SLAM_MAPENHANCER_H

#include <vector>
#include <opencv2/core/core.hpp>

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace ORB_SLAM2 {

class MapEnhancer  {
public:
    MapEnhancer() = delete;
    static KeyFrame *GetVirtualKeyFrame(const MapSlice &slice);
    static void Compress(Map *pMap);

private:
    static KeyFrame *GenerateKeyFrame(const cv::Mat& Scw, KeyFrame *pRefKF, bool isIdentical=false);
};

}

#endif //EDGE_SLAM_MAPENHANCER_H
