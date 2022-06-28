//
// Created by Halcao on 2020/5/17.
//

#ifndef EDGE_SLAM_DATASETUTIL_H
#define EDGE_SLAM_DATASETUTIL_H


#include <string>
#include <vector>

namespace ORB_SLAM2 {
    class DataSetUtil {
    public:
        static void LoadEuRoC(const std::string &strImagePath, const std::string &strPathTimes,
                              std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps);

        static void LoadTUM(const std::string &path, std::vector<std::string> &vstrImageFilenames,
                            std::vector<double> &vTimestamps);

        static void LoadKITTI(const std::string &path, std::vector<std::string> &vstrImageFilenames,
                              std::vector<double> &vTimestamps);
    };
}



#endif //EDGE_SLAM_DATASETUTIL_H
