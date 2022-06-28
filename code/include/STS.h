//
// Created by Halcao on 2022/2/28.
//

#ifndef EDGE_SLAM_STS_H
#define EDGE_SLAM_STS_H

namespace ORB_SLAM2 {

class ConnectionService;

class STS {
public:
    void ReportState();

    void SetConnectionService(ConnectionService *connectionService);
private:
    ConnectionService *mpConnectionService;
};

}

#endif //EDGE_SLAM_STS_H
