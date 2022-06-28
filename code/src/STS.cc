//
// Created by Halcao on 2022/2/28.
//

#include "STS.h"
#include "System.h"
#include "SystemState.h"

namespace ORB_SLAM2 {

void STS::ReportState() {
}

void STS::SetConnectionService(ConnectionService *connectionService) {
    this->mpConnectionService = connectionService;
}

}