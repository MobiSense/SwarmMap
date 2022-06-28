//
// Created by root on 7/8/19.
// Author: DanyLee
//

#ifndef BOOST_ARCHIVER_H
#define BOOST_ARCHIVER_H

#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <ctime>
#include <vector>

#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include <Thirdparty/g2o/g2o/types/sim3.h>
#include "Converter.h"
#include "Map.h"
#include "KeyFrameDatabase.h"
#include "SystemState.h"
#include "WebSocket.h"
//#include "KeyFrame.h"
//#include "Frame.h"
//#include "MapPoint.h"

BOOST_SERIALIZATION_ASSUME_ABSTRACT(ORB_SLAM2::MapElementUpdateBase)
//BOOST_SERIALIZATION_SPLIT_FREE:序列化调用save函数，反序列化调用load函数。
//BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)
namespace boost {
    namespace serialization {

        using namespace std;
        using namespace cv;
        using namespace ORB_SLAM2;
        using ORB_SLAM2::MapElementUpdateBase;
        using ORB_SLAM2::KeyFrameUpdate;
        using ORB_SLAM2::MapPointUpdate;
        using ORB_SLAM2::MapEventUpdate;

        typedef std::map<DBoW2::WordId, DBoW2::WordValue> BowVectorSuper;
        typedef std::map<DBoW2::NodeId, std::vector<unsigned int> > FeatureVectorSuper;

        /* DBoW2 BowVector */
        template<class Archive>
        void serialize(Archive &ar, DBoW2::BowVector &BowVec, const unsigned int file_version) {
            ar & boost::serialization::base_object<BowVectorSuper>(BowVec);
        }

        /* DBoW2 FeatureVector */
        template<class Archive>
        void serialize(Archive &ar, DBoW2::FeatureVector &FeatVec, const unsigned int file_version) {
            ar & boost::serialization::base_object<FeatureVectorSuper>(FeatVec);
        }

        /* CV KeyPoint */
        template<class Archive>
        void serialize(Archive &ar, ::cv::KeyPoint &kp, const unsigned int file_version) {
            ar & kp.pt.x;
            ar & kp.pt.y;
            ar & kp.size;
            ar & kp.angle;
            ar & kp.response;
            ar & kp.octave;
            ar & kp.class_id;
        }

        /*** Mat ***/
        template<class Archive>
        void serialize(Archive &ar, cv::Mat &mat, const unsigned int) {
            int cols, rows, type;
            bool continuous;

            if (Archive::is_saving::value) {
                cols = mat.cols;
                rows = mat.rows;
                type = mat.type();
                continuous = mat.isContinuous();
            }

            ar & cols & rows & type & continuous;

            if (Archive::is_loading::value)
                mat.create(rows, cols, type);

            if (continuous) {
                const unsigned int data_size = rows * cols * mat.elemSize();
                ar & boost::serialization::make_array(mat.ptr(), data_size);
            } else {
                const unsigned int row_size = cols * mat.elemSize();
                for (int i = 0; i < rows; i++) {
                    ar & boost::serialization::make_array(mat.ptr(i), row_size);
                }
            }
        }

        template<class Archive>
        void serialize(Archive &ar, g2o::Sim3 &sim3, const unsigned int file_version) {
            cv::Mat R;
            cv::Mat t;
            double s;

            cv::Mat mat;
            if (Archive::is_saving::value) {
                R = Converter::toCvMat(sim3.rotation().toRotationMatrix());
                t = Converter::toCvMat(sim3.translation());
                s = sim3.scale();
            }

            ar & R;
            ar & t;
            ar & s;

            if (Archive::is_loading::value) {
                sim3 = g2o::Sim3(Converter::toMatrix3d(R), Converter::toVector3d(t), s);
            }
        }

        /*** Frame ***/
        template<class Archive>
        void serialize(Archive &ar, Frame &frame, const unsigned int version) {

            // Frame timestamp.
            ar & frame.mTimeStamp;//double

            // Calibration matrix and OpenCV distortion parameters.
            ar & frame.mK;//cv::Mat
            ar & frame.fx;//float
            ar & frame.fy;//float
            ar & frame.cx;//float
            ar & frame.cy;//float
            ar & frame.invfx;//float
            ar & frame.invfy;//float
            ar & frame.mDistCoef;//cv::Mat

            // Stereo baseline multiplied by fx.
            ar & frame.mbf;//float

            // Stereo baseline in meters.
            ar & frame.mb;//float

            // Threshold close/far points. Close points are inserted from 1 view.
            // Far points are inserted as in the monocular case from 2 views.
            ar & frame.mThDepth;//float
            ar & frame.N;//int

            // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
            // In the stereo case, mvKeysUn is redundant as images must be rectified.
            // In the RGB-D case, RGB images can be distorted.
            ar & frame.mvKeys;//vector<cv::KeyPoint>
            ar & frame.mvKeysRight;//vector<cv::KeyPoint>
            ar & frame.mvKeysUn;//vector<cv::KeyPoint>

            // Corresponding stereo coordinate and depth for each keypoint.
            // "Monocular" keypoints have a negative value.
            ar & frame.mvuRight;//vector<float>
            ar & frame.mvDepth;//vector<float>

            // Bag of Words Vector structures.
            ar & frame.mBowVec;//DBoW2::BowVector
            ar & frame.mFeatVec;//DBoW2::FeatureVector

            // ORB descriptor, each row associated to a keypoint.
            ar & frame.mDescriptors;//cv::Mat
            ar & frame.mDescriptorsRight;//cv::Mat

            // Flag to identify outlier associations.
            ar & frame.mvbOutlier;//vector<bool>

            // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
            ar & frame.mfGridElementWidthInv;//float
            ar & frame.mfGridElementHeightInv;//float
            ar & frame.mGrid;//vector<std::size_t>

            // Camera pose.
            ar & frame.mTcw;//cv::Mat

            // Current and Next Frame id.
            ar & frame.mnId;//long unsigned int

            // Scale pyramid info.
            ar & frame.mnScaleLevels;//int
            ar & frame.mfScaleFactor;//float
            ar & frame.mfLogScaleFactor;//float
            ar & frame.mvScaleFactors;//vector<float>
            ar & frame.mvInvScaleFactors;//vector<float>
            ar & frame.mvLevelSigma2;//vector<float>
            ar & frame.mvInvLevelSigma2;//vector<float>

            // Undistorted Image Bounds (computed once).
            ar & frame.mnMinX;//float
            ar & frame.mnMaxX;//float
            ar & frame.mnMinY;//float
            ar & frame.mnMaxY;//float
            ar & frame.mbInitialComputations;//bool

        }

        /*** KeyFrameDataBase ***/
        template<class Archive>
        void serialize(Archive &ar, KeyFrameDatabase &kfDB, const unsigned int version) {
            // don't save associated vocabulary, KFDB restore by created explicitly from a new ORBvocabulary instance
            // inverted file
//            ar & kfDB.mvInvertedFile;
            ar & kfDB.mvInvertedFileId;
            // don't save mutex
        }

        /*** Map ***/
        template<class Archive>
        void serialize(Archive &ar, Map &map, const unsigned int version) {
            // don't save mutex
            ar & map.mspMapPoints;
            ar & map.mvpKeyFrameOrigins;
            ar & map.mspKeyFrames;
            ar & map.mvpReferenceMapPoints;
            ar & map.mnMaxKFid & map.mnBigChangeIdx;
            ar & map.allMPs;
            ar & map.allKFs;
        }

        /*** MapElementUpdateBase ***/
        template<class Archive>
        void serialize(Archive &ar, MapElementUpdateBase &update, const unsigned int file_version) {
            ar & update.id;
            ar & update.mnId;
            ar & update.funcName;
        }

        /*** KeyFrameUpdate ***/
        template<class Archive, class T>
        void serialize(Archive &ar, KeyFrameUpdate<T> &update, const unsigned int file_version) {
            ar & boost::serialization::base_object<MapElementUpdateBase>(update);
            ar & update.arg;
        }

        /*** MapPointUpdate ***/
        template<class Archive, class T>
        void serialize(Archive &ar, MapPointUpdate<T> &update, const unsigned int file_version) {
            ar & boost::serialization::base_object<MapElementUpdateBase>(update);
            ar & update.arg;
        }

        /*** MapEventUpdate ***/
        template<class Archive, class T>
        void serialize(Archive &ar, MapEventUpdate<T> &update, const unsigned int file_version) {
            ar & boost::serialization::base_object<MapElementUpdateBase>(update);
            ar & update.arg;
        }

        /*** Request ***/
        template<class Archive>
        void serialize(Archive &ar, ORB_SLAM2::Request &request, const unsigned int version) {
            ar & request.src;
            ar & request.dst;
            ar & request.path;
            ar & request.body;
        }

        /*** SystemState ***/
        template<class Archive>
        void serialize(Archive &ar, ORB_SLAM2::SystemState &state, const unsigned int version) {
            ar & state.location;
            ar & state.bVelocityBurst;
            ar & state.bStable;
            ar & state.nTracked;
            ar & state.lostCount;
        }
    }
}

namespace ORB_SLAM2 {
/*** toString function ***/
template<class Object>
std::string toString(const Object &obj) {
    std::stringstream out;
    boost::archive::text_oarchive oa(out);
//        boost::archive::binary_oarchive oa(out);
    // start to serialize
    try {
        oa << obj;
    } catch (boost::archive::archive_exception &e) {
    }

    return out.str();
}

template<class Object>
void toObject(Object &obj, const std::string &str) {
    std::stringstream in(str);
    boost::archive::text_iarchive ia(in);

    try {
        ia >> obj;
    } catch (boost::archive::archive_exception &e) {
    }
}
}

#endif // BOOST_ARCHIVER_H