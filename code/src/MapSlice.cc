//
// Created by Halcao on 2020/7/2.
//

#include "MapSlice.h"

ORB_SLAM2::MapSlice::MapSlice(const std::vector<KeyFrame *> &keyframes, const std::vector<MapPoint *> &mappoints,
                              const std::vector<MapElementUpdateBase *> &updates) : KFs(keyframes),
                                                                                    MPs(mappoints), updates(updates) {}
