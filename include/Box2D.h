#ifndef BOX2D_H
#define BOX2D_H

// std c
#include <string>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include "MapPoint.h"
#include "Box2D.h"
#include "Map.h"
#include "Frame.h"
#include "MatrixUtils.h"

namespace ORB_SLAM2
{
    class Frame;
    class MapPoint;
    // class KeyFrame;
    class Map;
        
    class Box2D
    {
    private:
        /* data */
    public:

        std::string class_name;	// class name.
        int class_id = -1;			    // class id.
        float x = 0.0f;            // left
        float y = 0.0f;            // top
        float width = 0.0f;            // width
        float height = 0.0f;            // height
        float score = 0.0f;		// probability.
        // cv::Rect bbox2d;           // (integer) 2D bbox_2d x y w h
        // Eigen::Vector4d bbox_vec;        // center, width, height, need?

        cv::Rect mRectBBox;              // cv::Rect format.
        cv::Rect mRectFeaturePoints;    // the bounding box constructed by object feature points.
        cv::Mat sum_pos_3d;             // Summation of points observed in the current frame.
        cv::Mat ave_pos_3d;             // current object center (3d, world).
        std::vector<MapPoint*>  Obj_c_MapPonits;        // object points in current frame.
        std::vector<cv::KeyPoint>  Obj_Feature_2D;        // feature points in current frame.
        int mCountMappoint;                         // = Obj_c_MapPonits.size().

        int mnId;               // object in map

        // cv::Point2f box_center_2d;      // 2D center.
        // cv::Mat sum_pos_3d_map;     // Summation of points observed in the map.
        Eigen::MatrixXd mObjLinesEigen;  // lines


        // float mStandar_x, mStandar_y, mStandar_z;   // standard deviation
        // cv::Mat mAssMapObjCenter;                   // map object center.
        // cv::Point2f point_center_2d;    
        bool bad = false;
        bool bOnEdge;           // on the edge of the image.

        Box2D(int _c, float _x, float _y, float _w, float _h, float _s)
        {
            class_id = _c;
            x = _x;
            y = _y;
            width = _w;
            height = _h;
            score = _s;
            mRectBBox = cv::Rect(x, y, width, height);
        }

        Box2D(/* args */);
        ~Box2D();
        void ConvertCVRect();
        void ComputeMeanAndStandardBbox();         // compute the mean and standard deviation of object points in current frame.
        void RemoveOutliersByBoxPlot(Frame &mCurrentFrame); // remove outliers by boxplot.
        // void ObjectDataAssociation(Map* mpMap, Frame &mCurrentFrame, int& img_height, int& img_width, string &flag);    // data association.
        void MergeTwoFrameBbox(Box2D* ObjLastFrame);
    };



}

#endif