#include "Box2D.h"

namespace ORB_SLAM2
{


Box2D::Box2D(/* args */)
{
}

Box2D::~Box2D()
{
}

void Box2D::ConvertCVRect()
{
    mRectBBox = cv::Rect(x, y, width, height);
}

// BRIEF compute the mean and standard deviation of object points in current frame.
void Box2D::ComputeMeanAndStandardBbox()
{
    // remove bad points.
    vector<MapPoint *>::iterator pMP;
    for (pMP = Obj_c_MapPonits.begin();
         pMP != Obj_c_MapPonits.end();)
    {
        cv::Mat pos = (*pMP)->GetWorldPos();

        if ((*pMP)->isBad())
        {
            pMP = Obj_c_MapPonits.erase(pMP);
            sum_pos_3d -= pos; // why ???
        }
        else
            ++pMP;
    }
    // mean(3d center)
    mCountMappoint = Obj_c_MapPonits.size();
    ave_pos_3d = sum_pos_3d / mCountMappoint;

    // // standard deviation in 3 directions.
    // float sum_x2 = 0, sum_y2 = 0, sum_z2 = 0;
    // for (size_t i = 0; i < Obj_c_MapPonits.size(); i++)
    // {
    //     MapPoint *pMP = Obj_c_MapPonits[i];
    //     if (pMP->isBad())
    //         continue;
    //     cv::Mat pos = pMP->GetWorldPos();
    //     cv::Mat pos_ave = ave_pos_3d;
    //     sum_x2 += (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0));
    //     sum_y2 += (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1));
    //     sum_z2 += (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2));
    // }
    // mStandar_x = sqrt(sum_x2 / (Obj_c_MapPonits.size()));
    // mStandar_y = sqrt(sum_y2 / (Obj_c_MapPonits.size()));
    // mStandar_z = sqrt(sum_z2 / (Obj_c_MapPonits.size()));
} // ComputeMeanAndStandardBbox().

// BRIEF Remove outliers (camera frame) by boxplot.
void Box2D::RemoveOutliersByBoxPlot(Frame &mCurrentFrame)
{
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    // world -> camera.
    vector<float> x_c;
    vector<float> y_c;
    vector<float> z_c;
    for (size_t i = 0; i < Obj_c_MapPonits.size(); i++)
    {
        MapPoint *pMP = Obj_c_MapPonits[i];

        cv::Mat PointPosWorld = pMP->GetWorldPos();
        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        x_c.push_back(PointPosCamera.at<float>(0));
        y_c.push_back(PointPosCamera.at<float>(1));
        z_c.push_back(PointPosCamera.at<float>(2));
    }

    // sort.
    sort(x_c.begin(), x_c.end());
    sort(y_c.begin(), y_c.end());
    sort(z_c.begin(), z_c.end());

    if ((z_c.size() / 4 <= 0) || (z_c.size() * 3 / 4 >= z_c.size() - 1))
        return;

    float Q1 = z_c[(int)(z_c.size() / 4)];
    float Q3 = z_c[(int)(z_c.size() * 3 / 4)];
    float IQR = Q3 - Q1;

    float min_th = Q1 - 1.5 * IQR;
    float max_th = Q3 + 1.5 * IQR;

    vector<MapPoint *>::iterator pMP;
    for (pMP = Obj_c_MapPonits.begin();
         pMP != Obj_c_MapPonits.end();)
    {
        cv::Mat PointPosWorld = (*pMP)->GetWorldPos();
        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        float z = PointPosCamera.at<float>(2);

        if (z > max_th || z < min_th)
            pMP = Obj_c_MapPonits.erase(pMP);   // remove.
        else
            ++pMP;
    }

    this->ComputeMeanAndStandardBbox();
} // RemoveOutliersByBoxPlot().

// BRIEF Merges objects between two adjacent frames.
void Box2D::MergeTwoFrameBbox(Box2D *ObjLastFrame)
{
    for (size_t m = 0; m < ObjLastFrame->Obj_c_MapPonits.size(); ++m)
    {
        bool bNewPoint = true;
        MapPoint *pMPLast = ObjLastFrame->Obj_c_MapPonits[m];
        if(pMPLast->isBad())
            continue;
        cv::Mat PosLast = pMPLast->GetWorldPos();
        // whether a new points.
        for (size_t n = 0; n < this->Obj_c_MapPonits.size(); ++n)
        {
            MapPoint *pMPCurr = this->Obj_c_MapPonits[n];
            cv::Mat PosCurr = pMPCurr->GetWorldPos();
            if (cv::countNonZero(PosLast - PosCurr) == 0)
            {
                bNewPoint = false;
                break;
            }
        }
        if (bNewPoint)
        {
            this->Obj_c_MapPonits.push_back(pMPLast);
            this->ComputeMeanAndStandardBbox();
        }
    }
} // Object_2D::MergeTwoFrameBbox(Object_2D* ObjLastFrame) END --------

/*
// BRIEF 2d objects (in the frame) associate with 3d objects (in the map).
void Box2D::ObjectDataAssociation(Map *mpMap, Frame &mCurrentFrame, int &img_width, int& img_height, string &flag)
{

    // const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    // const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    cv::Rect RectCurrent = mRectBBox;    // object bounding box in current frame.
    cv::Rect RectPredict;               // predicted bounding box according to last frame and next to last frame.
    cv::Rect RectProject;               // bounding box constructed by projecting points.
    float IouMax = 0;
    bool bAssoByIou = false;            // whether associated by IoU.
    int nAssoByIouId = -1;              // the associated map object ID.
    int IouMaxObjID = -1;               // temporary variable.
    float IouThreshold = 0.5;           // IoU threshold.

    // ****************************************************
    //         STEP 1. IoU data association.              *
    // ****************************************************
    std::vector<MapObject *> all_objects = mpMap->GetAllMapObjects();
    // for (int i = 0; i < (int)all_objects.size(); i++)
    // {
    //     if (all_objects[i]->bBadErase)
    //         continue;
    //     // object appeared in the last 30 frames.
    //     if (all_objects[i]->mnLastAddID > mCurrentFrame.mnId - 30)
    //         all_objects[i]->ComputeProjectRectFrame(img_width, img_height, mCurrentFrame);
    //     // else
    //     {
    //         all_objects[i]->mRectProject = cv::Rect(0, 0, 0, 0);
    //     }
    // }

    if((flag != "NA") && (flag != "NP"))
    {
        for (int i = 0; i < (int)all_objects.size(); i++)
        {
            if (class_id != all_objects[i]->mnClass)
            {
                std::cout << "Asso: object filter by class id " << std::endl;
                continue;
            }

            if (all_objects[i]->bBadErase)
            {
                std::cout << "Asso: object filter by bBadErase " << std::endl;
                continue;                
            }

            std::cout << "Asso: all_objects[i]->mnLastAddID: " << all_objects[i]->mnLastAddID << std::endl;
            std::cout << "Asso: mCurrentFrame.mnId: " << mCurrentFrame.mnId  << std::endl;
            if (all_objects[i]->mnLastAddID > mCurrentFrame.mnId - 20)// seen with in 20 frame
            // if (objMap->mnLastAddID < mCurrentFrame.mnId - 5)
            {
                // step 1.1 predict object bounding box according to last frame and next to last frame.
                if (all_objects[i]->mnLastLastAddID == mCurrentFrame.mnId - 2)
                {
                    // left-top.
                    float left_top_x = all_objects[i]->mLastRect.x * 2 - all_objects[i]->mLastLastRect.x;
                    if (left_top_x < 0)
                        left_top_x = 0;
                    float left_top_y = all_objects[i]->mLastRect.y * 2 - all_objects[i]->mLastLastRect.y;
                    if (left_top_y < 0)
                        left_top_y = 0;

                    // right-bottom.
                    float right_down_x = (all_objects[i]->mLastRect.x + all_objects[i]->mLastRect.width) * 2 - (all_objects[i]->mLastLastRect.x + all_objects[i]->mLastLastRect.width);
                    if (left_top_x > img_width)
                        right_down_x = img_width;
                    float right_down_y = (all_objects[i]->mLastRect.y + all_objects[i]->mLastRect.height) * 2 - (all_objects[i]->mLastLastRect.y + all_objects[i]->mLastLastRect.height);
                    if (left_top_y > img_height)
                        right_down_y = img_height;

                    float width = right_down_x - left_top_x;
                    float height = right_down_y - left_top_y;

                    // predicted bounding box.
                    RectPredict = cv::Rect(left_top_x, left_top_y, width, height);

                    // If two consecutive frames are observed, increase the threshold.
                    IouThreshold = 0.6;
                }
                else
                    RectPredict = all_objects[i]->mLastRect;

                // step 1.2 compute IoU, record the max IoU and the map object ID.
                float Iou = bboxOverlapratio(RectCurrent, RectPredict);
                if ((Iou > IouThreshold) && Iou > IouMax)
                {
                    IouMax = Iou;
                    IouMaxObjID = i;
                }
                std::cout << "RectCurrent: " << RectCurrent.x << " " << RectCurrent.y << std::endl;
                std::cout << "RectPredict: " << RectPredict.x << " " << RectPredict.y << std::endl;
            }
            else
                std::cout << "Asso: obj filter by more than 20 frame not seen "  << std::endl;
        }
        std::cout << "IouMax: " << IouMax  << " IouMaxObjID: " << IouMaxObjID << std::endl;
        // step 1.3 if the association is successful, update the map object.
        if ((IouMax > 0) && (IouMaxObjID >= 0))
        {
            // update.
            std::cout << "Asso: DataAssociateUpdate "  << std::endl;
            bool bFlag = all_objects[IouMaxObjID]->DataAssociateUpdate(this, mCurrentFrame, img_width, img_height, 1);
            if (bFlag)
            {
                bAssoByIou = true;              // associated by IoU.
                nAssoByIouId = IouMaxObjID;     // associated map object id.
            }
        }
    }
    // Iou data association END ----------------------------------------------------------------------------

/*
    // *************************************************
    //      STEP 2. Nonparametric data association     *
    // *************************************************
    bool bAssoByNp = false;
    int nAssoByNPId = -1;
    vector<int> vObjByNPId;     // potential associated objects.
    if((flag != "NA") && (flag != "IoU"))
    {
        for (int i = (int)all_objects.size() - 1; (i >= 0) && (old == false); i--)
        {
            if (class_id != all_objects[i]->mnClass)
                continue;

            if (all_objects[i]->bBadErase)
                continue;

            // step 2.1 nonparametric test.
            int nFlag = this->NoParaDataAssociation(all_objects[i], mCurrentFrame, image);

            if (nFlag == 0) // 0: skip the nonparametric test and continue with the subsequent t-test.
                break;
            if (nFlag == 2) // 2: association failed, compare next object.
                continue;
            else if (nFlag == 1) // 1: association succeeded, but there may be more than one.
                vObjByNPId.push_back(i);
        }

        // step 2.2 update data association and record potential associated objects.
        if (vObjByNPId.size() >= 1)
        {
            // case 1: if associated by IoU, the objects judged by nonparametric-test are marked as potential association objects.
            if (bAssoByIou)
            {
                for (int i = 0; i < vObjByNPId.size(); i++)
                {
                    if (vObjByNPId[i] == nAssoByIouId)
                        continue;

                    // Record potential association objects and potential association times.
                    map<int, int>::iterator sit;
                    sit = all_objects[nAssoByIouId]->mReObj.find(all_objects[vObjByNPId[i]]->mnId);
                    if (sit != all_objects[nAssoByIouId]->mReObj.end())
                    {
                        int sit_sec = sit->second;
                        all_objects[nAssoByIouId]->mReObj.erase(all_objects[vObjByNPId[i]]->mnId);
                        all_objects[nAssoByIouId]->mReObj.insert(make_pair(all_objects[vObjByNPId[i]]->mnId, sit_sec + 1));
                    }
                    else
                        all_objects[nAssoByIouId]->mReObj.insert(make_pair(all_objects[vObjByNPId[i]]->mnId, 1));
                }
            }
            // case 2: if association failed by IoU, 
            else
            {
                for (int i = 0; i < vObjByNPId.size(); i++)
                {
                    // update.
                    bool bFlag = all_objects[vObjByNPId[i]]->DataAssociateUpdate(this, mCurrentFrame, image, 2); // 2: NP.

                    // if association successful, other objects are marked as potential association objects.
                    if (bFlag)
                    {
                        bAssoByNp = true;               // associated by NP.
                        nAssoByNPId = vObjByNPId[i];    // associated map object id.

                        if (vObjByNPId.size() > i + 1)
                        {
                            for (int j = i + 1; j < vObjByNPId.size(); j++)
                            {
                                // Record potential association objects and potential association times.
                                map<int, int>::iterator sit;
                                sit = all_objects[vObjByNPId[i]]->mReObj.find(all_objects[vObjByNPId[j]]->mnId);
                                if (sit != all_objects[vObjByNPId[i]]->mReObj.end())
                                {
                                    int sit_sec = sit->second;
                                    all_objects[vObjByNPId[i]]->mReObj.erase(all_objects[vObjByNPId[j]]->mnId);
                                    all_objects[vObjByNPId[i]]->mReObj.insert(make_pair(all_objects[vObjByNPId[j]]->mnId, sit_sec + 1));
                                }
                                else
                                    all_objects[vObjByNPId[i]]->mReObj.insert(make_pair(all_objects[vObjByNPId[j]]->mnId, 1));
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
    // Nonparametric data association END --------------------------------------------------------------------------------------------------------

    if (old == false)
    {
        // ****************************************************
        //         STEP 3. Projected box data association     *
        // ****************************************************
        bool bAssoByProject = false;
        int nAssoByProId = -1;
        vector<int> vObjByProIouId;
        
        if((flag != "NA") && (flag != "IoU") && (flag != "NP"))
        {
            float fIouMax = 0.0;
            int ProIouMaxObjId = -1;
            for (int i = (int)all_objects.size() - 1; i >= 0; i--)
            {
                if (class_id != all_objects[i]->mnClass)
                    continue;

                if (all_objects[i]->bBadErase)
                    continue;

                int df = (int)all_objects[i]->mObjectFrame.size();

                if ((Obj_c_MapPonits.size() >= 10) && (df > 8))
                    continue;

                // step 3.1 compute IoU with bounding box constructed by projecting points.
                float fIou = bboxOverlapratio(RectCurrent, all_objects[i]->mRectProject);
                float fIou2 = bboxOverlapratio(mRectFeaturePoints, all_objects[i]->mRectProject);
                fIou = max(fIou, fIou2);

                // record the max IoU and map object id.
                if ((fIou >= 0.25) && (fIou > fIouMax))
                {
                    fIouMax = fIou;
                    ProIouMaxObjId = i;
                    vObjByProIouId.push_back(i);
                }
            }
            // step 3.2 update data association and record potential associated objects.
            if (fIouMax >= 0.25)
            {
                sort(vObjByProIouId.begin(), vObjByProIouId.end());

                if (bAssoByIou || bAssoByNp)
                {
                    for (int j = vObjByProIouId.size() - 1; j >= 0; j--)
                    {
                        int ReId;
                        if (bAssoByIou)
                            ReId = nAssoByIouId;
                        if (bAssoByNp)
                            ReId = nAssoByNPId;

                        if (vObjByProIouId[j] == ReId)
                            continue;

                        map<int, int>::iterator sit;
                        sit = all_objects[ReId]->mReObj.find(all_objects[vObjByProIouId[j]]->mnId);
                        if (sit != all_objects[ReId]->mReObj.end())
                        {
                            int sit_sec = sit->second;
                            all_objects[ReId]->mReObj.erase(all_objects[vObjByProIouId[j]]->mnId);
                            all_objects[ReId]->mReObj.insert(make_pair(all_objects[vObjByProIouId[j]]->mnId, sit_sec + 1));
                        }
                        else
                            all_objects[ReId]->mReObj.insert(make_pair(all_objects[vObjByProIouId[j]]->mnId, 1));
                    }
                }
                else
                {
                    // update.
                    bool bFlag = all_objects[ProIouMaxObjId]->DataAssociateUpdate(this, mCurrentFrame, image, 4); // 4: project iou.

                    // association succeeded.
                    if (bFlag)
                    {
                        bAssoByProject = true;          // associated by projecting box.
                        nAssoByProId = ProIouMaxObjId;  // associated map object id.
                    }

                    for (int j = vObjByProIouId.size() - 1; j >= 0; j--)
                    {
                        if (vObjByProIouId[j] == ProIouMaxObjId)
                            continue;

                        map<int, int>::iterator sit;
                        sit = all_objects[ProIouMaxObjId]->mReObj.find(all_objects[vObjByProIouId[j]]->mnId);
                        if (sit != all_objects[ProIouMaxObjId]->mReObj.end())
                        {
                            int sit_sec = sit->second;
                            all_objects[ProIouMaxObjId]->mReObj.erase(all_objects[vObjByProIouId[j]]->mnId);
                            all_objects[ProIouMaxObjId]->mReObj.insert(make_pair(all_objects[vObjByProIouId[j]]->mnId, sit_sec + 1));
                        }
                        else
                            all_objects[ProIouMaxObjId]->mReObj.insert(make_pair(all_objects[vObjByProIouId[j]]->mnId, 1));
                    }
                }
            }
        }
        // Projected box data association END ---------------------------------------------------------------------------------------

        // ************************************************
        //          STEP 4. t-test data association       *
        // ************************************************
        // step 4.1 Read t-distribution boundary value.
        float tTestData[122][9] = {0};
        ifstream infile;
        infile.open("./data/t_test.txt");
        for (int i = 0; i < 122; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                infile >> tTestData[i][j];
            }
        }
        infile.close();
        
        // step 4.2 t-test.
        bool bAssoByT = false;
        int nAssoByTId = -1;
        vector<int> vObjByTId;
        vector<int> vObjByTIdLower; // potential association.
        if((flag != "NA") && (flag != "IoU") && (flag != "NP"))
        {
            for (int i = (int)all_objects.size() - 1; i >= 0; i--)
            {
                if (class_id != all_objects[i]->mnClass)
                    continue;

                if (all_objects[i]->bBadErase)
                    continue;

                // t-test results in 3 directions.
                float t_test;
                float t_test_x, t_test_y, t_test_z;

                // Degrees of freedom.
                int df = (int)all_objects[i]->mObjectFrame.size();

                if (df <= 8)
                    continue;

                // Iou.
                float fIou = bboxOverlapratio(RectCurrent, all_objects[i]->mRectProject);
                float fIou2 = bboxOverlapratio(mRectFeaturePoints, all_objects[i]->mRectProject);
                fIou = max(fIou, fIou2);

                // The distance from points to the object center.
                float dis_x, dis_y, dis_z;
                dis_x = abs(all_objects[i]->mCenter3D.at<float>(0, 0) - ave_pos_3d.at<float>(0, 0));
                dis_y = abs(all_objects[i]->mCenter3D.at<float>(1, 0) - ave_pos_3d.at<float>(1, 0));
                dis_z = abs(all_objects[i]->mCenter3D.at<float>(2, 0) - ave_pos_3d.at<float>(2, 0));

                // t-test.
                t_test_x = dis_x / (all_objects[i]->mCenterStandar_x / sqrt(df));
                t_test_y = dis_y / (all_objects[i]->mCenterStandar_y / sqrt(df));
                t_test_z = dis_z / (all_objects[i]->mCenterStandar_z / sqrt(df));

                // Satisfy t test.  // 5->0.05.
                if ((t_test_x < tTestData[min((df - 1), 121)][5]) &&
                    (t_test_y < tTestData[min((df - 1), 121)][5]) &&
                    (t_test_z < tTestData[min((df - 1), 121)][5]))
                {
                    vObjByTId.push_back(i);
                }
                // If the T-test is not satisfied, but the IOU is large, reducing the significance.
                else if (fIou > 0.25)
                {
                    if ((t_test_x < tTestData[min((df - 1), 121)][8]) &&
                        (t_test_y < tTestData[min((df - 1), 121)][8]) &&
                        (t_test_z < tTestData[min((df - 1), 121)][8]))
                    {
                        vObjByTId.push_back(i);
                    }

                    else if ((fIou > 0.25) && ((t_test_x + t_test_y + t_test_z) / 3 < 10))
                    {
                        vObjByTId.push_back(i);
                    }
                    else
                    {
                        vObjByTIdLower.push_back(i);
                    }
                }
                else if ((t_test_x + t_test_y + t_test_z) / 3 < 4)
                {
                    all_objects[i]->ComputeProjectRectFrame(image, mCurrentFrame);

                    float fIou_force = bboxOverlapratio(RectCurrent, all_objects[i]->mRectProject);
                    float fIou2_force = bboxOverlapratio(mRectFeaturePoints, all_objects[i]->mRectProject);
                    fIou_force = max(fIou_force, fIou2_force);

                    if (fIou_force > 0.25)
                        vObjByTIdLower.push_back(i);
                }
            }

            // step 4.2 update data association and record potential associated objects.
            if (bAssoByIou || bAssoByNp || bAssoByProject)
            {
                int ReId;
                if (bAssoByIou)
                    ReId = nAssoByIouId;
                if (bAssoByNp)
                    ReId = nAssoByNPId;
                if (bAssoByProject)
                    ReId = nAssoByProId;

                if (vObjByTId.size() >= 1)
                {
                    for (int j = 0; j < vObjByTId.size(); j++)
                    {
                        if (vObjByTId[j] == ReId)
                            continue;

                        map<int, int>::iterator sit;
                        sit = all_objects[ReId]->mReObj.find(all_objects[vObjByTId[j]]->mnId);
                        if (sit != all_objects[ReId]->mReObj.end())
                        {
                            int sit_sec = sit->second;
                            all_objects[ReId]->mReObj.erase(all_objects[vObjByTId[j]]->mnId);
                            all_objects[ReId]->mReObj.insert(make_pair(all_objects[vObjByTId[j]]->mnId, sit_sec + 1));
                        }
                        else
                            all_objects[ReId]->mReObj.insert(make_pair(all_objects[vObjByTId[j]]->mnId, 1));
                    }
                }

                if (vObjByTIdLower.size() >= 0)
                {
                    for (int j = 0; j < vObjByTIdLower.size(); j++)
                    {
                        if (vObjByTIdLower[j] == ReId)
                            continue;

                        map<int, int>::iterator sit;
                        sit = all_objects[ReId]->mReObj.find(all_objects[vObjByTIdLower[j]]->mnId);
                        if (sit != all_objects[ReId]->mReObj.end())
                        {
                            int sit_sec = sit->second;
                            all_objects[ReId]->mReObj.erase(all_objects[vObjByTIdLower[j]]->mnId);
                            all_objects[ReId]->mReObj.insert(make_pair(all_objects[vObjByTIdLower[j]]->mnId, sit_sec + 1));
                        }
                        else
                            all_objects[ReId]->mReObj.insert(make_pair(all_objects[vObjByTIdLower[j]]->mnId, 1));
                    }
                }
            }
            else
            {
                if (vObjByTId.size() >= 1)
                {
                    for (int i = 0; i < vObjByTId.size(); i++)
                    {
                        bool bFlag = all_objects[vObjByTId[i]]->DataAssociateUpdate(this, mCurrentFrame, image, 3); // 3 是指 T 方法.

                        if (bFlag)
                        {
                            bAssoByT = true;
                            nAssoByTId = vObjByTId[i];

                            if (vObjByTId.size() > i)
                            {
                                for (int j = i + 1; j < vObjByTId.size(); j++)
                                {
                                    map<int, int>::iterator sit;
                                    sit = all_objects[nAssoByTId]->mReObj.find(all_objects[vObjByTId[j]]->mnId);
                                    if (sit != all_objects[nAssoByTId]->mReObj.end())
                                    {
                                        int sit_sec = sit->second;
                                        all_objects[nAssoByTId]->mReObj.erase(all_objects[vObjByTId[j]]->mnId);
                                        all_objects[nAssoByTId]->mReObj.insert(make_pair(all_objects[vObjByTId[j]]->mnId, sit_sec + 1));
                                    }
                                    else
                                        all_objects[nAssoByTId]->mReObj.insert(make_pair(all_objects[vObjByTId[j]]->mnId, 1));
                                }
                            }

                            if (vObjByTIdLower.size() >= 0)
                            {
                                for (int j = 0; j < vObjByTIdLower.size(); j++)
                                {
                                    if (vObjByTIdLower[j] == nAssoByTId)
                                        continue;

                                    map<int, int>::iterator sit;
                                    sit = all_objects[nAssoByTId]->mReObj.find(all_objects[vObjByTIdLower[j]]->mnId);
                                    if (sit != all_objects[nAssoByTId]->mReObj.end())
                                    {
                                        int sit_sec = sit->second;
                                        all_objects[nAssoByTId]->mReObj.erase(all_objects[vObjByTIdLower[j]]->mnId);
                                        all_objects[nAssoByTId]->mReObj.insert(make_pair(all_objects[vObjByTIdLower[j]]->mnId, sit_sec + 1));
                                    }
                                    else
                                        all_objects[nAssoByTId]->mReObj.insert(make_pair(all_objects[vObjByTIdLower[j]]->mnId, 1));
                                }
                            }

                            break; 
                        }
                    }
                }
            }
        }
        // t-test data association END ---------------------------------------------------------------------------------------


    // *************************************************
    //             STEP 4. create a new object         *
    // *************************************************
    // if (bAssoByIou || bAssoByNp || bAssoByProject || bAssoByT)
    //     return;
    if (bAssoByIou)
    {
        std::cout << "Asso: associated by existing object, not create new objects " << std::endl;
        return;
    }
    // If the object appears at the edge of the image, ignore.
    if ((this->x < 10) || (this->y < 10) ||
        (this->x + this->width > img_width - 10) ||
        (this->y + this->height > img_height - 10))
    {
        this->bad = true;
        return;
    }

    // create a 3d object in the map.
    MapObject *MapObjectSingle = new MapObject();
    MapObjectSingle->mObjectFrame.push_back(this);     
    MapObjectSingle->mnId = all_objects.size(); 
    MapObjectSingle->mnClass = class_id;             
    MapObjectSingle->mnConfidence = 1;              
    // MapObjectSingle->mbFirstObserve = true;            
    // MapObjectSingle->mnAddedID = mCurrentFrame.mnId;
    MapObjectSingle->mnLastAddID = mCurrentFrame.mnId;
    MapObjectSingle->mnLastLastAddID = mCurrentFrame.mnId;
    MapObjectSingle->mLastRect = mRectBBox;                 
    // MapObjectSingle->msFrameId.insert(mCurrentFrame.mnId); 
    MapObjectSingle->mSumPointsPos = sum_pos_3d;           
    MapObjectSingle->mCenter3D = ave_pos_3d;
    // this->mAssMapObjCenter = this->ave_pos_3d;
    std::cout << "Asso: add new object " << MapObjectSingle->mCenter3D.t() << std::endl;

    // add properties of the point and save it to the object.
    for (size_t i = 0; i < Obj_c_MapPonits.size(); i++)
    {
        MapPoint *pMP = Obj_c_MapPonits[i];

        pMP->object_id = MapObjectSingle->mnId;                           
        pMP->object_class = MapObjectSingle->mnClass;                    
        pMP->object_id_vector.insert(make_pair(MapObjectSingle->mnId, 1));

        // if (MapObjectSingle->mbFirstObserve == true) 
        //     pMP->First_obj_view = true;

        // save to the object.
        MapObjectSingle->mvpMapObjectMappoints.push_back(pMP);
    }

    mnId = MapObjectSingle->mnId;
    // mnWhichTime = MapObjectSingle->mnConfidence;
    // current = true;

    // save this 2d object to current frame (associates with a 3d object in the map).
    mCurrentFrame.mvFrameBbox.push_back(this);
    // mCurrentFrame.AppearNewObject = true;

    // will be added after filter. 
    // mCurrentFrame.m3DObject.push_back(MapObjectSingle); 

    // update object map.
    // MapObjectSingle->IsolationForestDeleteOutliers();
    MapObjectSingle->ComputeMeanAndStandard();
    // all_objects.push_back(MapObjectSingle);
    mpMap->AddMapObject(MapObjectSingle);// add to  mpMap->mspMapCuboids
    std::cout << "Asso: mpMap->AddMapObject(MapObjectSingle) "  << std::endl;
    // create a new object END ------------------------------------------------------

} // ObjectDataAssociation() END --------------------------------------------------------

*/




}
    
