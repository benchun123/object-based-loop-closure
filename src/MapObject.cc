#include "MapObject.h"

namespace ORB_SLAM2
{

MapObject::MapObject(/* args */)
{ }

MapObject::~MapObject()
{ }

// BRIEF compute the mean and standard deviation of objects in the map.
void MapObject::ComputeMeanAndStandard()
{
    vector<MapPoint *> points_after_filter;
    FilterMapPoints(points_after_filter);
    // std::cout << "points_after_filter: " << points_after_filter.size() << std::endl;
    if(points_after_filter.size() == 0)
    {
        this->bBadErase = true;  
        return;      
    }

    mvpMapObjectMappointsFilter = points_after_filter;

    mSumPointsPos = 0;
    // remove bad points.
    {
        unique_lock<mutex> lock(mMutexMapPoints);
        vector<MapPoint *>::iterator pMP;
        int i = 0;
        for (pMP = points_after_filter.begin();
             pMP != points_after_filter.end();)
        {
            i++;

            if ((*pMP)->isBad())
            {
                pMP = points_after_filter.erase(pMP);
            }
            else
            {
                cv::Mat pos = (*pMP)->GetWorldPos();
                mSumPointsPos += pos;
                ++pMP;
            }
        }
    }

    // mean(3d center).
    mCenter3D = mSumPointsPos / (points_after_filter.size());

    // step 1. standard deviation in 3 directions.
    float sum_x2 = 0, sum_y2 = 0, sum_z2 = 0;
    vector<float> x_pt, y_pt, z_pt;
    for (size_t i = 0; i < points_after_filter.size(); i++)
    {
        cv::Mat pos = points_after_filter[i]->GetWorldPos();
        cv::Mat pos_ave = mCenter3D;

        // （x-x^）^2
        sum_x2 += (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0));
        sum_y2 += (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1));
        sum_z2 += (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2));

        x_pt.push_back(pos.at<float>(0));
        y_pt.push_back(pos.at<float>(1));
        z_pt.push_back(pos.at<float>(2));
    }
    // mStandar_x = sqrt(sum_x2 / (mvpMapObjectMappoints.size()));
    // mStandar_y = sqrt(sum_y2 / (mvpMapObjectMappoints.size()));
    // mStandar_z = sqrt(sum_z2 / (mvpMapObjectMappoints.size()));

    if (x_pt.size() == 0)
        return;

    // step 2. standard deviation of centroids (observations from different frames).
    // float sum_x2_c = 0, sum_y2_c = 0, sum_z2_c = 0;
    // vector<float> x_c, y_c, z_c;
    // for(size_t i = 0; i < this->mObjectFrame.size(); i++)
    // {
    //     cv::Mat pos = this->mObjectFrame[i]->_Pos;
    //     cv::Mat pos_ave = this->mCenter3D;
    //     // （x-x^）^2
    //     sum_x2_c += (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0));
    //     sum_y2_c += (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1));
    //     sum_z2_c += (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2));
    // }
    // mCenterStandar_x = sqrt(sum_x2_c / (this->mObjectFrame.size()));
    // mCenterStandar_y = sqrt(sum_y2_c / (this->mObjectFrame.size()));
    // mCenterStandar_z = sqrt(sum_z2_c / (this->mObjectFrame.size()));

    // step 3. update object center and scale.
    // if (this->mObjectFrame.size() < 5)
    {
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        sort(z_pt.begin(), z_pt.end());

        if ((x_pt.size() == 0) || (y_pt.size() == 0) || (z_pt.size() == 0))
        {
            this->bBadErase = true;
            return;
        }

        x_min = x_pt[0];
        x_max = x_pt[x_pt.size() - 1];

        y_min = y_pt[0];
        y_max = y_pt[y_pt.size() - 1];

        z_min = z_pt[0];
        z_max = z_pt[z_pt.size() - 1];

        lenth = x_max - x_min;
        width = y_max - y_min;
        height = z_max - z_min;
        // centre.
        cuboidCenter = Eigen::Vector3d((x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2);
        corner_1 = Eigen::Vector3d(x_min, y_min, z_min);
        corner_2 = Eigen::Vector3d(x_max, y_min, z_min);
        corner_3 = Eigen::Vector3d(x_max, y_max, z_min);
        corner_4 = Eigen::Vector3d(x_min, y_max, z_min);
        corner_5 = Eigen::Vector3d(x_min, y_min, z_max);
        corner_6 = Eigen::Vector3d(x_max, y_min, z_max);
        corner_7 = Eigen::Vector3d(x_max, y_max, z_max);
        corner_8 = Eigen::Vector3d(x_min, y_max, z_max);

        // mCuboid3D.corner_1_w = Eigen::Vector3d(x_min, y_min, z_min);
        // mCuboid3D.corner_2_w = Eigen::Vector3d(x_max, y_min, z_min);
        // mCuboid3D.corner_3_w = Eigen::Vector3d(x_max, y_max, z_min);
        // mCuboid3D.corner_4_w = Eigen::Vector3d(x_min, y_max, z_min);
        // mCuboid3D.corner_5_w = Eigen::Vector3d(x_min, y_min, z_max);
        // mCuboid3D.corner_6_w = Eigen::Vector3d(x_max, y_min, z_max);
        // mCuboid3D.corner_7_w = Eigen::Vector3d(x_max, y_max, z_max);
        // mCuboid3D.corner_8_w = Eigen::Vector3d(x_min, y_max, z_max);


    }

    // // step 4. update object pose.
    // std::cout << "map obj pose before " << rotY << pose << std::endl;
    UpdateObjPose();
    // CalculateObjectPose();
    // std::cout << "map obj pose after " << rotY << pose << std::endl;

/*
    // world -> object frame.
    vector<float> x_pt_obj, y_pt_obj, z_pt_obj;
    for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++)
    {
        // world frame.
        Eigen::Vector3d PointPos_world = Converter::toVector3d(mvpMapObjectMappoints[i]->GetWorldPos());

        // object frame.
        Eigen::Vector3d PointPos_object = Converter::toMatrix4d(this->pose).inverse() * PointPos_world;
        x_pt_obj.push_back(PointPos_object[0]);
        y_pt_obj.push_back(PointPos_object[1]);
        z_pt_obj.push_back(PointPos_object[2]);
    }

    if (x_pt_obj.size() == 0)
        return;

    // rank.
    int s = x_pt_obj.size();
    sort(x_pt_obj.begin(), x_pt_obj.end());
    sort(y_pt_obj.begin(), y_pt_obj.end());
    sort(z_pt_obj.begin(), z_pt_obj.end());

    float x_min_obj = x_pt_obj[0];
    float x_max_obj = x_pt_obj[s - 1];
    float y_min_obj = y_pt_obj[0];
    float y_max_obj = y_pt_obj[s - 1];
    float z_min_obj = z_pt_obj[0];
    float z_max_obj = z_pt_obj[s - 1];

    // update object vertices and translate it to world frame.
    mCuboid3D.corner_1 = this->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_2 = this->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_3 = this->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_4 = this->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_5 = this->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_6 = this->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_7 = this->mCuboid3D.pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj);
    mCuboid3D.corner_8 = this->mCuboid3D.pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj);

    // object frame -> world frame (without yaw, parallel to world frame).
    mCuboid3D.corner_1_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_2_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_3_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_4_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_5_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_6_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_7_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj);
    mCuboid3D.corner_8_w = this->mCuboid3D.pose_without_yaw * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj);

    // update scale and pose.
    this->lenth = x_max_obj - x_min_obj;
    this->width = y_max_obj - y_min_obj;
    this->height = z_max_obj - z_min_obj;
    this->cuboidCenter = (mCuboid3D.corner_2 + mCuboid3D.corner_8) / 2;
    UpdateObjPose();

    // maximum radius.
    float fRMax = 0.0;
    vector<cv::Mat> vCornerMat;
    vCornerMat.resize(8);
    for (int i = 0; i < 8; i++)
    {
        cv::Mat mDis = cv::Mat::zeros(3, 1, CV_32F);
        if (i == 0)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_1);
        if (i == 1)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_2);
        if (i == 2)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_3);
        if (i == 3)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_4);
        if (i == 4)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_5);
        if (i == 5)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_6);
        if (i == 6)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_7);
        if (i == 7)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_8);

        mDis = mCenter3D - vCornerMat[i];
        float fTmp = sqrt(mDis.at<float>(0) * mDis.at<float>(0) + mDis.at<float>(1) * mDis.at<float>(1) + mDis.at<float>(2) * mDis.at<float>(2));
        fRMax = max(fRMax, fTmp);
    }
    mCuboid3D.mfRMax = fRMax;

    // standard deviation of distance.
    float dis = 0;
    for (size_t i = 0; i < mObjectFrame.size(); i++)
    {
        float center_sum_x2 = 0, center_sum_y2 = 0, center_sum_z2 = 0;

        cv::Mat pos = mObjectFrame[i]->_Pos; 
        cv::Mat pos_ave = mCenter3D;

        center_sum_x2 = (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0)); // dis_x^2
        center_sum_y2 = (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1)); // dis_y^2
        center_sum_z2 = (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2)); // dis_z^2

        dis += sqrt(center_sum_x2 + center_sum_y2 + center_sum_z2);
    }
    mCenterStandar = sqrt(dis / (mObjectFrame.size()));

*/
} // ComputeMeanAndStandard() END ---------------------------------------------------------------------------------------

// BRIEF whether associate successful.
bool MapObject::DataAssociateUpdate(Box2D *ObjectFrame,
                                     Frame &mCurrentFrame,
                                     int &img_width, int& img_height,
                                     int Flag) // 1 Iou, 2 NP, 3 t-test，4 project. for debug.
{
    if (ObjectFrame->class_id != mnClass)
        return false;

    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    // std::cout << "Asso: Map object update: " << std::endl;
    std::cout << "Asso: Map object before center : " << mCenter3D.t() << " " << mvpMapObjectMappoints.size() << " " << rotY << std::endl;
    // step 1. whether the box projected into the image changes greatly after the new point cloud is associated.
    if ((Flag != 1) && (Flag != 4))
    {
        cv::Rect ProjectRect1;
        cv::Rect ProjectRect2;

        // projected bounding box1.
        this->ComputeProjectRectFrame(img_width, img_height, mCurrentFrame);
        ProjectRect1 = this->mRectProject;

        // mixed points of frame object and map object.
        vector<float> x_pt;
        vector<float> y_pt;
        for (int i = 0; i < ObjectFrame->Obj_c_MapPonits.size(); ++i)
        {
            MapPoint *pMP = ObjectFrame->Obj_c_MapPonits[i];
            cv::Mat PointPosWorld = pMP->GetWorldPos();
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }
        for (int j = 0; j < mvpMapObjectMappoints.size(); ++j)
        {
            MapPoint *pMP = mvpMapObjectMappoints[j];
            cv::Mat PointPosWorld = pMP->GetWorldPos();
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }

        // rank.
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];
        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        if (x_min < 0)
            x_min = 0;
        if (y_min < 0)
            y_min = 0;
        if (x_max > img_width)
            x_max = img_width;
        if (y_max > img_height)
            y_max = img_height;

        // projected bounding box2. 
        ProjectRect2 = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);

        // 4. calculate Iou
        float fIou = bboxOverlapratio(ProjectRect1, ProjectRect2);
        float fIou2 = bboxOverlapratioFormer(ProjectRect2, ObjectFrame->mRectBBox);
        if ((fIou < 0.5) && (fIou2 < 0.8))
            return false;
    }

    // step 2. update the ID of the last frame
    if (mnLastAddID != (int)mCurrentFrame.mnId)
    {
        mnLastLastAddID = mnLastAddID;    
        mnLastAddID = mCurrentFrame.mnId;

        mLastLastRect = mLastRect;
        mLastRect = ObjectFrame->mRectBBox;

        mnConfidence++;

        // ObjectFrame->current = true;

        mObjectFrame.push_back(ObjectFrame);
        mObjectFrameId.push_back(mCurrentFrame.mnId);      

    }
    else
        return false;

    ObjectFrame->mnId = mnId;               
    // ObjectFrame->mnWhichTime = mnConfidence;

    // step 3. Add the point cloud of the frame object to the map object
    std::cout << "Asso: Map object point size : " << ObjectFrame->Obj_c_MapPonits.size() << std::endl;
    for (size_t j = 0; j < ObjectFrame->Obj_c_MapPonits.size(); ++j)
    {
        MapPoint *pMP = ObjectFrame->Obj_c_MapPonits[j];
        if(pMP->isBad())
            continue;
        cv::Mat pointPos = pMP->GetWorldPos();
        cv::Mat mDis = mCenter3D - pointPos;
        // float fDis = sqrt(mDis.at<float>(0) * mDis.at<float>(0) + mDis.at<float>(1) * mDis.at<float>(1) + mDis.at<float>(2) * mDis.at<float>(2));

        // float th = 1.0;
        // if (mObjectFrame.size() > 5)
        //     th = 0.9;

        // if (fDis > th * mCuboid3D.mfRMax)
        //     continue;

        // if ((this->mObjectFrame.size() >= 10) && ((this->mnClass == 56) || (this->mnClass == 77)))
        // {
        //     Eigen::Vector3d scale = this->pose.inverse() * Converter::toVector3d(pointPos);
        //     if ((abs(scale[0]) > 1.2 * this->lenth / 2) ||
        //         (abs(scale[1]) > 1.2 * this->width / 2) ||
        //         (abs(scale[2]) > 1.2 * this->height / 2))
        //         continue;
        // }

        pMP->object_id = mnId;       
        pMP->object_class = mnClass; 

        // add points.        
        map<int, int>::iterator sit;
        sit = pMP->object_id_vector.find(pMP->object_id);
        if (sit != pMP->object_id_vector.end())
        {
            int sit_sec = sit->second;                                           
            pMP->object_id_vector.erase(pMP->object_id);                         
            pMP->object_id_vector.insert(make_pair(pMP->object_id, sit_sec + 1));
        }
        else
        {
            pMP->object_id_vector.insert(make_pair(pMP->object_id, 1));
        }

        {
            unique_lock<mutex> lock(mMutexMapPoints);
            bool new_point = true;
            // old points.
            for (size_t m = 0; m < mvpMapObjectMappoints.size(); ++m)
            {
                cv::Mat obj_curr_pos = pMP->GetWorldPos();
                cv::Mat obj_map_pos = mvpMapObjectMappoints[m]->GetWorldPos();

                if (cv::countNonZero(obj_curr_pos - obj_map_pos) == 0)
                {
                    // mvpMapObjectMappoints[m]->have_feature = pMP->have_feature;
                    // mvpMapObjectMappoints[m]->feature = pMP->feature;
                    new_point = false;

                    break;
                }
            }
            // new point.
            if (new_point)
            {
                mvpMapObjectMappoints.push_back(pMP);
                mvpMapCurrentNewMappoints.push_back(pMP);
                // std::cout << "Asso: add point to obj : "  << std::endl;

                cv::Mat x3d = pMP->GetWorldPos();
                mSumPointsPos += x3d;
            }
        }
    
    }

    // step 4. the historical point cloud is projected into the image, and the points not in the box(should not on the edge) are removed.
    if ((ObjectFrame->x > 25) && (ObjectFrame->y > 25) &&
        (ObjectFrame->x + ObjectFrame->width < img_width - 25) &&
        (ObjectFrame->y + ObjectFrame->height < img_height - 25))
    {
        unique_lock<mutex> lock(mMutexMapPoints); // lock.
        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            int sit_sec = 0;
            map<int , int>::iterator sit;
            sit = (*pMP)->object_id_vector.find(mnId);
            if (sit != (*pMP)->object_id_vector.end())
            {
                sit_sec = sit->second;
            }
            if (sit_sec > 8)
            {
                ++pMP;
                continue;
            }

            cv::Mat PointPosWorld = (*pMP)->GetWorldPos();
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            if ((u > 0 && u < img_width) && (v > 0 && v < img_height))
            {
                if (!ObjectFrame->mRectBBox.contains(cv::Point2f(u, v)))
                {
                    // cv::circle(image, cv::Point2f(u, v), 3, cv::Scalar(0, 215, 256), -1);
                    pMP = mvpMapObjectMappoints.erase(pMP);
                    mSumPointsPos -= PointPosWorld;
                }
                else
                {
                    ++pMP;
                }
            }
            else
            {
                ++pMP;
            }
        }
    }

    // step 5. update object mean.
    this->ComputeMeanAndStandard();

    // // step 6. i-Forest.
    // this->IsolationForestDeleteOutliers();

    mCurrentFrame.m3DObject.push_back(this);
    mCurrentFrame.mvFrameBbox.push_back(ObjectFrame);
    std::cout << "Asso: Map object center after: " << mCenter3D.t() << " " << mvpMapObjectMappoints.size() << " " << rotY << std::endl;

    return true;
} // MapObject::DataAssociateUpdate() END ---------------------------------------------



// BRIEF projecting points to the image, constructing a bounding box.
void MapObject::ComputeProjectRectFrame(int &img_width, int& img_height, Frame &mCurrentFrame)
{
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    vector<float> x_pt;
    vector<float> y_pt;
    for (int j = 0; j < mvpMapObjectMappoints.size(); j++)
    {
        MapPoint *pMP = mvpMapObjectMappoints[j];
        cv::Mat PointPosWorld = pMP->GetWorldPos();
        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        const float xc = PointPosCamera.at<float>(0);
        const float yc = PointPosCamera.at<float>(1);
        const float invzc = 1.0 / PointPosCamera.at<float>(2);

        float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
        float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

        x_pt.push_back(u);
        y_pt.push_back(v);

    }

    if (x_pt.size() == 0)
        return;

    sort(x_pt.begin(), x_pt.end());
    sort(y_pt.begin(), y_pt.end());
    float x_min = x_pt[0];
    float x_max = x_pt[x_pt.size() - 1];
    float y_min = y_pt[0];
    float y_max = y_pt[y_pt.size() - 1];

    if (x_min < 0)
        x_min = 0;
    if (y_min < 0)
        y_min = 0;
    if (x_max > img_width)
        x_max = img_width;
    if (y_max > img_height)
        y_max = img_height;

    mRectProject = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
} // ComputeProjectRectFrame() END ----------------------------------------------


// BRIEF check whether two objects overlap.
bool MapObject::WhetherOverlap(MapObject *CompareObj)
{
    // distance between two centers.
    float dis_x = abs(cuboidCenter(0) - CompareObj->cuboidCenter(0));
    float dis_y = abs(cuboidCenter(1) - CompareObj->cuboidCenter(1));
    float dis_z = abs(cuboidCenter(2) - CompareObj->cuboidCenter(2));

    float sum_lenth_half = lenth / 2 + CompareObj->lenth / 2;
    float sum_width_half = width / 2 + CompareObj->width / 2;
    float sum_height_half = height / 2 + CompareObj->height / 2;

    // whether overlap.
    if ((dis_x < sum_lenth_half) && (dis_y < sum_width_half) && (dis_z < sum_height_half))
        return true;
    else
        return false;
} // WhetherOverlap() END ----------------------------------------------------------------

// BRIEF update object pose.
void MapObject::UpdateObjPose()
{
    unique_lock<mutex> lock(mMutex);

    // Rotation matrix.
    float cp = cos(rotP);
    float sp = sin(rotP);
    float sr = sin(rotR);
    float cr = cos(rotR);
    float sy = sin(rotY);
    float cy = cos(rotY);
    Eigen::Matrix<double, 3, 3> REigen;
    REigen << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
        cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
        -sp, sr * cp, cr * cp;
    cv::Mat Ryaw = Converter::toCvMat(REigen);

    // Transformation matrix.
    cv::Mat Twobj = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat Rcw = Twobj.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Twobj.rowRange(0, 3).col(3);
    cv::Mat result = Rcw * Ryaw;

    Twobj.at<float>(0, 0) = result.at<float>(0, 0);
    Twobj.at<float>(0, 1) = result.at<float>(0, 1);
    Twobj.at<float>(0, 2) = result.at<float>(0, 2);
    //Twobj.at<float>(0, 3) = mCenter3D.at<float>(0);
    Twobj.at<float>(0, 3) = cuboidCenter[0];
    Twobj.at<float>(1, 0) = result.at<float>(1, 0);
    Twobj.at<float>(1, 1) = result.at<float>(1, 1);
    Twobj.at<float>(1, 2) = result.at<float>(1, 2);
    //Twobj.at<float>(1, 3) = mCenter3D.at<float>(1);
    Twobj.at<float>(1, 3) = cuboidCenter[1];
    Twobj.at<float>(2, 0) = result.at<float>(2, 0);
    Twobj.at<float>(2, 1) = result.at<float>(2, 1);
    Twobj.at<float>(2, 2) = result.at<float>(2, 2);
    //Twobj.at<float>(2, 3) = mCenter3D.at<float>(2);
    Twobj.at<float>(2, 3) = cuboidCenter[2];
    Twobj.at<float>(3, 0) = 0;
    Twobj.at<float>(3, 1) = 0;
    Twobj.at<float>(3, 2) = 0;
    Twobj.at<float>(3, 3) = 1;

    // note no yaw.
    cv::Mat Twobj_without_yaw = cv::Mat::eye(4, 4, CV_32F);
    Twobj_without_yaw.at<float>(0, 3) = mCenter3D.at<float>(0);
    Twobj_without_yaw.at<float>(1, 3) = cuboidCenter[1];
    Twobj_without_yaw.at<float>(2, 3) = mCenter3D.at<float>(2);

    // // SE3.
    // g2o::SE3Quat obj_pose = Converter::toSE3Quat(Twobj);
    // g2o::SE3Quat obj_pose_without_yaw = Converter::toSE3Quat(Twobj_without_yaw);

    this->pose = Twobj;
    this->pose_without_yaw = Twobj_without_yaw;
} // UpdateObjPose()


// BRIEF update object pose.
void MapObject::CalculateObjectPose()
{
    unique_lock<mutex> lock(mMutex);

    Eigen::Vector3d cuboid_loc = cuboidCenter;
    Eigen::Vector3d cuboid_dim = Eigen::Vector3d(lenth/2.0, width/2.0, height/2.0);
    Eigen::MatrixXd corners_3d(3,8);
    Eigen::MatrixXd corners_dist(3,8);
    Eigen::MatrixXd corners_body(3,8); // should be 1 or -1
    Eigen::MatrixXd corners_without_center(3,8); // points
    corners_3d.col(0) = corner_1;
    corners_3d.col(1) = corner_2;
    corners_3d.col(2) = corner_3;
    corners_3d.col(3) = corner_4;
    corners_3d.col(4) = corner_5;
    corners_3d.col(5) = corner_6;
    corners_3d.col(6) = corner_7;
    corners_3d.col(7) = corner_8;

    corners_dist.col(0) = corner_1-cuboid_loc;
    corners_dist.col(1) = corner_2-cuboid_loc;
    corners_dist.col(2) = corner_3-cuboid_loc;
    corners_dist.col(3) = corner_4-cuboid_loc;
    corners_dist.col(4) = corner_5-cuboid_loc;
    corners_dist.col(5) = corner_6-cuboid_loc;
    corners_dist.col(6) = corner_7-cuboid_loc;
    corners_dist.col(7) = corner_8-cuboid_loc;
    std::cout << "corner_1: " << corner_1.transpose() << std::endl;
    std::cout << "corner_2: " << corner_2.transpose() << std::endl;
    std::cout << "cuboid_loc: " << cuboid_loc.transpose() << std::endl;
    std::cout << "cuboid_dim: " << cuboid_dim.transpose() << std::endl;
    std::cout << "corners_3d: \n" << corners_3d << std::endl;
    std::cout << "corners_dist: \n" << corners_dist << std::endl;
    for (size_t i = 0; i < 8; i++)
    {
        corners_without_center(0,i) = corners_3d(0,i) - cuboid_loc(0);
        corners_without_center(1,i) = corners_3d(1,i) - cuboid_loc(1);
        corners_without_center(2,i) = corners_3d(2,i) - cuboid_loc(2);
        corners_body(0,i) = 2*double(corners_dist(0,i)>=cuboid_dim(0))-1; // switch to -1 or 1
        corners_body(1,i) = 2*double(corners_dist(1,i)>=cuboid_dim(1))-1;
        corners_body(2,i) = 2*double(corners_dist(2,i)>=cuboid_dim(2))-1;
    }
    std::cout << "corners_body: \n" << corners_body << std::endl;
    Eigen::Matrix3d scale_mat = cuboid_dim.asDiagonal();
    Eigen::MatrixXd temp = scale_mat * corners_body;
    Eigen::Matrix3d rot_matrix_temp = corners_3d * temp.transpose() * (temp* temp.transpose()).inverse();
    std::cout << "rot_matrix: \n" << rot_matrix_temp << std::endl;
    std::cout << "rot_matrix: \n" << rot_matrix_temp.eulerAngles(0,1,2).transpose() << std::endl;
    // Eigen::Vector3d rpy;
    // rot_to_euler_zyx(rot_matrix_temp, rpy(0), rpy(1), rpy(2));
    // std::cout << "rot_matrix: \n" << rpy.transpose() << std::endl;

    Eigen::Matrix4d obj_mat;
    obj_mat.setIdentity();
    obj_mat.block(0,0,3,3) = rot_matrix_temp;
    obj_mat.col(3).head(3) = cuboid_loc;

    // Transformation matrix.
    cv::Mat Twobj = cv::Mat::eye(4, 4, CV_32F);
    for (size_t i = 0; i < 4; i++)
        for (size_t j = 0; j < 4; j++)
            Twobj.at<float>(i, j) = obj_mat(i, j) ;
    
    this->pose = Twobj;
} // CalculateObjectPose()


// BRIEF big one gets smaller, the smaller one stays the same. (No significant effect! Almost no use.)
void MapObject::BigToSmall(MapObject *SmallObj, float overlap_x, float overlap_y, float overlap_z)
{
    // in which direction does the overlap occur.
    bool bxMin = false;
    bool bxMax = false;
    bool byMin = false;
    bool byMax = false;
    bool bzMin = false;
    bool bzMax = false;

    // x_min
    if ((SmallObj->x_min > this->x_min) && (SmallObj->x_min < this->x_max))
        bxMin = true;
    // x_max
    if ((SmallObj->x_max > this->x_min) && (SmallObj->x_max < this->x_max))
        bxMax = true;
    // y_min
    if ((SmallObj->y_min > this->y_min) && (SmallObj->y_min < this->y_max))
        byMin = true;
    // y_max
    if ((SmallObj->y_max > this->y_min) && (SmallObj->y_max < this->y_max))
        byMax = true;
    // z_min
    if ((SmallObj->z_min > this->z_min) && (SmallObj->z_min < this->z_max))
        bzMin = true;
    // z_max
    if ((SmallObj->z_max > this->z_min) && (SmallObj->z_max < this->z_max))
        bzMax = true;

    // false: one direction，ture: two directions.
    bool bx = false;
    bool by = false;
    bool bz = false;
    // x 
    if ((bxMin = true) && (bxMax = true))
        bx = true;
    else
        bx = false;
    // y 
    if ((byMin = true) && (byMax = true))
        by = true;
    else
        by = false;
    // z 
    if ((bzMin = true) && (bzMax = true))
        bz = true;
    else
        bz = false;

    // Which direction to eliminate?
    int nFlag; // 0:x   1:y   2:z   3: surround

    // x
    if ((bx == false) && (by == true) && (bz == true))
        nFlag = 0;
    // y
    if ((bx == true) && (by == false) && (bz == true))
        nFlag = 1;
    // z
    if ((bx == true) && (by == true) && (bz == false))
        nFlag = 2;

    if ((bx == false) && (by == false) && (bz == true))
    {
        if (min(overlap_x, overlap_y) == overlap_x)
            nFlag = 0;
        else if (min(overlap_x, overlap_y) == overlap_y)
            nFlag = 1;
    }

    if ((bx == false) && (by == true) && (bz == false))
    {
        if (min(overlap_x, overlap_z) == overlap_x)
            nFlag = 0;
        else if (min(overlap_x, overlap_z) == overlap_z)
            nFlag = 2;
    }

    if ((bx == true) && (by == false) && (bz == false))
    {
        if (min(overlap_y, overlap_z) == overlap_y)
            nFlag = 1;
        else if (min(overlap_y, overlap_z) == overlap_z)
            nFlag = 2;
    }

    //     7------6
    //    /|     /|
    //   / |    / |
    //  4------5  |
    //  |  3---|--2
    //  | /    | /
    //  0------1
    {
        unique_lock<mutex> lock(mMutexMapPoints); // lock.

        // remove points in the overlap volume.
        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            cv::Mat PointPosWorld = (*pMP)->GetWorldPos();

            // points in the smaller object.
            if ((PointPosWorld.at<float>(0) > SmallObj->x_min) && (PointPosWorld.at<float>(0) < SmallObj->x_max) &&
                (PointPosWorld.at<float>(1) > SmallObj->y_min) && (PointPosWorld.at<float>(1) < SmallObj->y_max) &&
                (PointPosWorld.at<float>(2) > SmallObj->z_min) && (PointPosWorld.at<float>(2) < SmallObj->z_max))
                pMP = mvpMapObjectMappoints.erase(pMP);
            else
            {
                ++pMP;
            }
        }
    }

    this->ComputeMeanAndStandard();
} // BigToSmall() END -----------------------------------------------------------------------------------------------------------------


// BRIEF Divide the overlap area of two objects equally.  (No significant effect! Almost no use.)
void MapObject::DivideEquallyTwoObjs(MapObject *AnotherObj, float overlap_x, float overlap_y, float overlap_z)
{
    {
        unique_lock<mutex> lock(mMutexMapPoints);

        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            cv::Mat PointPosWorld = (*pMP)->GetWorldPos();

            if (((PointPosWorld.at<float>(0) > AnotherObj->cuboidCenter(0) - (AnotherObj->lenth / 2 - overlap_x / 2)) &&
                 (PointPosWorld.at<float>(0) < AnotherObj->cuboidCenter(0) + (AnotherObj->lenth / 2 - overlap_x / 2))) &&
                ((PointPosWorld.at<float>(1) > AnotherObj->cuboidCenter(1) - (AnotherObj->width / 2 - overlap_y / 2)) &&
                 (PointPosWorld.at<float>(1) < AnotherObj->cuboidCenter(1) + (AnotherObj->width / 2 - overlap_y / 2))) &&
                ((PointPosWorld.at<float>(2) > AnotherObj->cuboidCenter(2) - (AnotherObj->height / 2 - overlap_z / 2)) &&
                 (PointPosWorld.at<float>(2) < AnotherObj->cuboidCenter(2) + (AnotherObj->height / 2 - overlap_z / 2))))
            {
                pMP = mvpMapObjectMappoints.erase(pMP);
            }

            else
            {
                ++pMP;
            }
        }
    }
} // DivideEquallyTwoObjs() END --------------------------------------------------------------------------------


// BRIEF Dealing with two overlapping objects.
void MapObject::DealTwoOverlapObjs(MapObject *OverlapObj, float overlap_x, float overlap_y, float overlap_z)
{
    bool bIou = false;       // false: Iou is large.
    bool bVolume = false;    // false: small volume difference.
    bool bSame_time = false; // false: doesn't simultaneous appearance.
    bool bClass = false;     // false: different classes.

    float fThis_obj_volume = (lenth * width) * height;
    float fOverlap_obj_volume = (OverlapObj->lenth * OverlapObj->width) * OverlapObj->height;

    // compute Iou.
    float overlap_volume = (overlap_x * overlap_y) * overlap_z;
    if ((overlap_volume / (fThis_obj_volume + fOverlap_obj_volume - overlap_volume)) >= 0.3)
        bIou = true;
    else
        bIou = false;

    // compute the volume difference.
    if ((fThis_obj_volume > 2 * fOverlap_obj_volume) || (fOverlap_obj_volume > 2 * fThis_obj_volume))
        bVolume = true;
    else
        bVolume = false;

    // whether simultaneous appearance.
    map<int, int>::iterator sit;
    sit = mmAppearSametime.find(OverlapObj->mnId);
    if (sit != mmAppearSametime.end())
    {
        if (sit->second > 3)
            bSame_time = true;
        else
            bSame_time = false;
    }
    else
        bSame_time = false;

    // class.
    if (mnClass == OverlapObj->mnClass)
        bClass = true;
    else
        bClass = false;

    // case 1: IOU is large, the volume difference is small, doesn't simultaneous appearance, same class --> the same object, merge them.
    if ((bIou == true) && (bVolume == false) && (bSame_time == false) && (bClass == true))
    {
        if (this->mObjectFrame.size() >= OverlapObj->mObjectFrame.size())
        {
            this->MergeTwoMapObjs(OverlapObj);
            OverlapObj->bBadErase = true;
        }
        else
        {
            OverlapObj->MergeTwoMapObjs(this);
            this->bBadErase = true;
        }
    }

    // case 2: may be a false detection.
    else if ((bVolume == true) && (bSame_time == false) && (bClass == true))
    {
        if ((this->mObjectFrame.size() >= OverlapObj->mObjectFrame.size()) && (fThis_obj_volume > fOverlap_obj_volume))
            OverlapObj->bBadErase = true;
        else if ((this->mObjectFrame.size() < OverlapObj->mObjectFrame.size()) && (fThis_obj_volume < fOverlap_obj_volume))
            this->bBadErase = true;
    }

    // case 3: divide the overlap area of two objects equally.  (No significant effect.)
    else if ((bIou == true) && (bVolume == false) && (bSame_time == true) && (bClass == true))
    {
        this->DivideEquallyTwoObjs(OverlapObj, overlap_x, overlap_y, overlap_z);
        OverlapObj->DivideEquallyTwoObjs(OverlapObj, overlap_x, overlap_y, overlap_z);

        this->ComputeMeanAndStandard();
        OverlapObj->ComputeMeanAndStandard();
    }

    // case 4: big one gets smaller, the smaller one stays the same. (No significant effect.)
    else if ((bIou == false) && (bVolume == true) && (bSame_time == true) && (bClass == false))
    {
        if (fThis_obj_volume > fOverlap_obj_volume)
            this->BigToSmall(OverlapObj, overlap_x, overlap_y, overlap_z);
        else if (fThis_obj_volume < fOverlap_obj_volume)
            OverlapObj->BigToSmall(this, overlap_x, overlap_y, overlap_z);
    }

    // case 5: 
    else if((bIou == true) && (bSame_time == false) && (bClass == true))
    {
        if (this->mObjectFrame.size()/2 >= OverlapObj->mObjectFrame.size())
        {
            this->MergeTwoMapObjs(OverlapObj);
            OverlapObj->bBadErase = true;
        }
        else if(OverlapObj->mObjectFrame.size()/2 >= this->mObjectFrame.size())
        {
            OverlapObj->MergeTwoMapObjs(this);
            this->bBadErase = true;
        }
    }

    // TODO case ...... and so on ......
} // DealTwoOverlapObjs() END ---------------------------------------------------------------

// BRIEF merge two objects.
void MapObject::MergeTwoMapObjs(MapObject *RepeatObj)
{
    // step 1. update points.
    for (int i = 0; i < RepeatObj->mvpMapObjectMappoints.size(); i++)
    {
        MapPoint *pMP = RepeatObj->mvpMapObjectMappoints[i];
        if(pMP->isBad())
            continue;
        cv::Mat pointPos = pMP->GetWorldPos();
        Eigen::Vector3d scale = Converter::toMatrix3d(this->pose.inv()) * Converter::toVector3d(pointPos);
        if ((abs(scale[0]) > 1.1 * this->lenth / 2) ||
            (abs(scale[1]) > 1.1 * this->width / 2) ||
            (abs(scale[2]) > 1.1 * this->height / 2))
        {
            continue;
        }

        pMP->object_id = mnId;       
        pMP->object_class = mnClass; 

        map<int, int>::iterator sit;
        sit = pMP->object_id_vector.find(pMP->object_id);
        if (sit != pMP->object_id_vector.end()) 
        {
            int sit_sec = sit->second;                                            
            pMP->object_id_vector.erase(pMP->object_id);                          
            pMP->object_id_vector.insert(make_pair(pMP->object_id, sit_sec + 1)); 
        }
        else
        {
            pMP->object_id_vector.insert(make_pair(pMP->object_id, 1));
        }
        {
            unique_lock<mutex> lock(mMutexMapPoints);
            bool new_point = true;
            // old points.
            for (size_t m = 0; m < mvpMapObjectMappoints.size(); ++m)
            {
                cv::Mat obj_curr_pos = pMP->GetWorldPos();
                cv::Mat obj_map_pos = mvpMapObjectMappoints[m]->GetWorldPos();

                if (cv::countNonZero(obj_curr_pos - obj_map_pos) == 0)
                {
                    // mvpMapObjectMappoints[m]->have_feature = pMP->have_feature;
                    // mvpMapObjectMappoints[m]->feature = pMP->feature;
                    new_point = false;

                    break;
                }
            }
            // new points.
            if (new_point)
            {
                mvpMapObjectMappoints.push_back(pMP);

                cv::Mat x3d = pMP->GetWorldPos();
                mSumPointsPos += x3d;
            }
        }
    }

    // step 2. update frame objects.
    for (int j = 0; j < RepeatObj->mObjectFrame.size(); j++)
    {
        Box2D *ObjectFrame = RepeatObj->mObjectFrame[j];
        int observe_id = RepeatObj->mObjectFrameId[j];

        ObjectFrame->mnId = mnId;
        mnConfidence++;

        mObjectFrame.push_back(ObjectFrame);
        mObjectFrameId.push_back(observe_id);      

    }

    // step 3. update the co-view relationship
    {
        map<int, int>::iterator sit;
        for (sit = RepeatObj->mmAppearSametime.begin(); sit != RepeatObj->mmAppearSametime.end(); sit++)
        {
            int nObjId = sit->first;
            int sit_sec = sit->second;

            map<int, int>::iterator sit2;
            sit2 = mmAppearSametime.find(nObjId);
            if (sit2 != mmAppearSametime.end())
            {
                int sit_sec2 = sit2->second;
                mmAppearSametime.erase(nObjId);
                mmAppearSametime.insert(make_pair(nObjId, sit_sec2 + sit_sec));
            }
            else
                mmAppearSametime.insert(make_pair(nObjId, 1));
        }
    }

    // step 4. update the last observed frame.
    int nOriginLastAddID = mnLastAddID;
    int nOriginLastLatsAddID = mnLastLastAddID;
    cv::Rect OriginLastRect = mLastRect;
    // this object appeared recently.
    if (mnLastAddID > RepeatObj->mnLastAddID)
    {
        if (nOriginLastLatsAddID > RepeatObj->mnLastAddID)
        {
        }
        else
        {
            mnLastLastAddID = RepeatObj->mnLastAddID;
            mLastLastRect = RepeatObj->mObjectFrame[RepeatObj->mObjectFrame.size() - 1]->mRectBBox;
        }
    }
    // RepeatObj appeared recently.
    else
    {
        mnLastAddID = RepeatObj->mnLastAddID;
        mLastRect = RepeatObj->mObjectFrame[RepeatObj->mObjectFrame.size() - 1]->mRectBBox;

        if (nOriginLastAddID > RepeatObj->mnLastLastAddID)
        {
            mnLastLastAddID = nOriginLastAddID;
            mLastLastRect = OriginLastRect;
        }
        else
        {
            mnLastLastAddID = RepeatObj->mnLastLastAddID;
            mLastLastRect = RepeatObj->mObjectFrame[RepeatObj->mObjectFrame.size() - 2]->mRectBBox;
        }
    }

    // step 5. update direction.
    // if (((mnClass == 73) || (mnClass == 64) || (mnClass == 65) 
    //     || (mnClass == 66) || (mnClass == 56)))
    {
        if(RepeatObj->mvAngleTimesAndScore.size() > 0)
        {
            for (auto &row_repeat : RepeatObj->mvAngleTimesAndScore)
            {
                bool new_measure = true;

                if(this->mvAngleTimesAndScore.size() > 0)
                {
                    for (auto &row_this : this->mvAngleTimesAndScore)
                    {
                        if(row_repeat[0] == row_this[0])
                        {
                            row_this[1] += row_repeat[1];

                            row_this[2] = row_this[2] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[2] * (row_repeat[1] / row_this[1]);
                            
                            row_this[3] = row_this[3] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[3] * (row_repeat[1] / row_this[1]);
                            
                            row_this[4] = row_this[4] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[4] * (row_repeat[1] / row_this[1]);

                            new_measure = false;
                            break;
                        }
                    }
                }

                if(new_measure == true)
                {
                    this->mvAngleTimesAndScore.push_back(row_repeat);
                }
            }
        }

        if(this->mvAngleTimesAndScore.size() > 0)
        {
            int best_num = 0;
            float best_score = 0.0;
            for(int i = 0; i < min(6, (int)this->mvAngleTimesAndScore.size()); i++)
            {
                float fScore = this->mvAngleTimesAndScore[i][2];
                if(fScore > best_score)
                {
                    best_score = fScore;
                    best_num = i;
                }
            }

            this->rotY = this->mvAngleTimesAndScore[best_num][0];
            this->mfErrorParallel = this->mvAngleTimesAndScore[best_num][3];
            this->mfErrorYaw = this->mvAngleTimesAndScore[best_num][4];
            this->UpdateObjPose();
        }
    }
} // MergeTwoMapObjs() END -----------------------------------------------------------

void MapObject::FilterMapPoints(vector<MapPoint *>& points_after_filter)
{
    // step 1: get object point association.
    // can we do this when validate cuboids?? then don't need to do it every time?
    // get ba points from point cuboid association
    vector<Vector3d> all_object_ba_points_xyz; // use for filter
    vector<MapPoint*> all_object_ba_points_pts; // use for filter and show
    for (size_t j = 0; j < mvpMapObjectMappoints.size(); j++)
        if (mvpMapObjectMappoints[j])
            if (!mvpMapObjectMappoints[j]->isBad())
            {
                all_object_ba_points_pts.push_back(mvpMapObjectMappoints[j]);
                all_object_ba_points_xyz.push_back(Converter::toVector3d(mvpMapObjectMappoints[j]->GetWorldPos()));
            }


    // step 2: filter bad object points
    // compute the mean, eliminate outlier points.
    double coarse_threshold = 1.0;
    double fine_threshold = 0.5;
    // pMObj->used_points_in_BA_filtered.clear();
    Eigen::Vector3d mean_point;
    mean_point.setZero();
    for (size_t j = 0; j < all_object_ba_points_xyz.size(); j++)
        mean_point += all_object_ba_points_xyz[j];
    mean_point /= (double)(all_object_ba_points_xyz.size());

    //NOTE  filtering of points!!!  remove outlier points
    Eigen::Vector3d mean_point_2;
    mean_point_2.setZero();
    int valid_point_num = 0;
    for (size_t j = 0; j < all_object_ba_points_xyz.size(); j++)
    {
        // std::cout << " delta: " << (mean_point - all_object_ba_points_xyz[j]).norm() << std::endl;
        if ((mean_point - all_object_ba_points_xyz[j]).norm() < coarse_threshold)
        {
            mean_point_2 += all_object_ba_points_xyz[j];
            valid_point_num++;
        }        
    }

    mean_point_2 /= (double)valid_point_num;

    Eigen::Vector3d mean_point_final;
    mean_point_final.setZero();
    std::vector<Eigen::Vector3d> good_points; // for car, if points are 4 meters away from center, usually outlier.
    for (size_t j = 0; j < all_object_ba_points_xyz.size(); j++)
    {
        if ((mean_point_2 - all_object_ba_points_xyz[j]).norm() < fine_threshold)
        {
            mean_point_final += all_object_ba_points_xyz[j];
            good_points.push_back(all_object_ba_points_xyz[j]);
            points_after_filter.push_back(all_object_ba_points_pts[j]);
        }
    }
    mean_point_final /= (double)(good_points.size());
    all_object_ba_points_xyz.clear();
    all_object_ba_points_xyz = good_points;        
    
    
    // if(points_after_filter.size()<10)
    // {
    //     points_after_filter = all_object_ba_points_pts;
    // }

}


}