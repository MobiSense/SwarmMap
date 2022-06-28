//
// Created by Halcao on 2020/6/9.
//

#ifndef EDGE_SLAM_EDGESIM3RELATIVEXYZ_H
#define EDGE_SLAM_EDGESIM3RELATIVEXYZ_H
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace g2o {
class EdgeSim3RelativeXYZ : public  BaseBinaryEdge<3, Vector3d,  VertexSBAPointXYZ, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3RelativeXYZ(): BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSim3Expmap>()
    {
    };
    virtual bool read(std::istream& is) {};
    virtual bool write(std::ostream& os) const {};

    void computeError()
    {
        const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

        Vector3d obs(_measurement);
        _error = obs-v1->estimate().map(v2->estimate());
    }

    // virtual void linearizeOplus();
};

} // end namespace

#endif //EDGE_SLAM_EDGESIM3RELATIVEXYZ_H
