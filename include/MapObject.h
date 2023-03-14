#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

// std c
#include <string>
#include <vector>
#include <mutex>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

#include "MapPoint.h"
#include "Box2D.h"
// #include "Map.h"
#include "Converter.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MatrixUtils.h"


namespace ORB_SLAM2
{
    class Frame;
    class MapPoint;
    class KeyFrame;
    // class Map;
    class Box2D;

    class MapObject
    {
    private:

    public:
        int mnId;           // object id in map
        int mnClass;
        int mnConfidence;
        // int mnAddedID;      
        int mnLastAddID;        // frame id when add to map
        int mnLastLastAddID;
        cv::Rect mLastRect;
        cv::Rect mLastLastRect;
        cv::Rect mRectProject;
        cv::Mat mSumPointsPos;
        cv::Mat mCenter3D;     
        bool bBadErase = false;

        // param
        float x_min, x_max, y_min, y_max, z_min, z_max;     // the boundary in XYZ direction.
        float lenth, width, height;
        // 8 vertices and center
        Eigen::Vector3d cuboidCenter;       // the center of the Cube, not the center of mass of the object
        // angle.
        float rotY = 0.0;        
        float rotP = 0.0;
        float rotR = 0.0;
        // line.
        float mfErrorParallel;
        float mfErrorYaw;
        std::vector<Eigen::Matrix<float,5,1> > mvAngleTimesAndScore;    // Score of sampling angle.
        
        // pose 
        cv::Mat pose;
        cv::Mat pose_without_yaw;

        // corners
        Eigen::Vector3d corner_1;
        Eigen::Vector3d corner_2;
        Eigen::Vector3d corner_3;
        Eigen::Vector3d corner_4;
        Eigen::Vector3d corner_5;
        Eigen::Vector3d corner_6;
        Eigen::Vector3d corner_7;
        Eigen::Vector3d corner_8;

        std::vector<Box2D*> mObjectFrame;
        std::vector<int> mObjectFrameId;
        std::vector<MapPoint*> mvpMapObjectMappoints;
        std::vector<MapPoint*> mvpMapCurrentNewMappoints;
        std::vector<MapPoint*> mvpMapObjectMappointsFilter;

        bool DataAssociateUpdate(Box2D* ObjectFrame, 
                                    Frame &mCurrentFrame, 
                                    int &img_width, int& img_height,
                                    int Flag);
        void ComputeProjectRectFrame(int &img_width, int& img_height, Frame &mCurrentFrame);
        bool WhetherOverlap(MapObject *CompareObj);
        void UpdateObjPose();      // update object pose.
        void CalculateObjectPose(); // calculate object pose by corners

        // void IsolationForestDeleteOutliers();
        void BigToSmall(MapObject *SmallObj, float overlap_x, float overlap_y, float overlap_z);
        void DivideEquallyTwoObjs(MapObject *AnotherObj, float overlap_x, float overlap_y, float overlap_z);
        void DealTwoOverlapObjs(MapObject *OverlapObj, float overlap_x, float overlap_y, float overlap_z);
        void MergeTwoMapObjs(MapObject *RepeatObj);
        void FilterMapPoints(vector<MapPoint *>& points_after_filter);
            

        std::map<int, int> mmAppearSametime;// object id and times simultaneous appearances .

            // std::set<int> msFrameId;
    
            // float mStandar_x, mStandar_y, mStandar_z;
            // float mCenterStandar_x, mCenterStandar_y, mCenterStandar_z;
            // float mCenterStandar; 

            // int nMayRepeat = 0;                 // maybe a repeat object.
            // std::map<int, int> mReObj;          // potential associated objects.


            // Cuboid3D mCuboid3D;                  // cuboid.
            // vector<cv::Mat> mvPointsEllipsoid;   // not used.




            // void ComputeProjectRectFrame(cv::Mat &image, Frame &mCurrentFrame); 
            // void WhetherMergeTwoMapObjs(Map *mpMap);
            // void MergeTwoMapObjs(Object_Map *RepeatObj);
            // bool DoubleSampleTtest(Object_Map *RepeatObj);
            // void DealTwoOverlapObjs(Object_Map *OverlapObj, float overlap_x, float overlap_y, float overlap_z);
            // // void BigToSmall(Object_Map *SmallObj, float overlap_x, float overlap_y, float overlap_z);          
            // void DivideEquallyTwoObjs(Object_Map *AnotherObj, float overlap_x, float overlap_y, float overlap_z);

            // // void UpdateObjScale(Eigen::Vector3d Scale);    // for optimization.




        MapObject(/* args */);
        ~MapObject();
        void ComputeMeanAndStandard();
        
    protected:
        std::mutex mMutexMapPoints;
        std::mutex mMutex;
    };
}

#endif