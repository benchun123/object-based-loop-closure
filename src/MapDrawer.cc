/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
// #include <opencv2/core/eigen.hpp>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

    // std::vector<Eigen::Vector3f> box_colors;
	box_colors.push_back(Vector3f(230, 0, 0) / 255.0);	 // red  0
	box_colors.push_back(Vector3f(60, 180, 75) / 255.0);   // green  1
	box_colors.push_back(Vector3f(0, 0, 255) / 255.0);	 // blue  2
	box_colors.push_back(Vector3f(255, 0, 255) / 255.0);   // Magenta  3
	box_colors.push_back(Vector3f(255, 165, 0) / 255.0);   // orange 4
	box_colors.push_back(Vector3f(128, 0, 128) / 255.0);   //purple 5
	box_colors.push_back(Vector3f(0, 255, 255) / 255.0);   //cyan 6
	box_colors.push_back(Vector3f(210, 245, 60) / 255.0);  //lime  7
	box_colors.push_back(Vector3f(250, 190, 190) / 255.0); //pink  8
	box_colors.push_back(Vector3f(0, 128, 128) / 255.0);   //Teal  9
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::DrawFrameMapObjects() 
{
}
void MapDrawer::DrawAssoMapPoints() 
{
    const vector<MapObject*> &vObjs = mpMap->GetAllMapObjects();
    for(size_t i = 0; i < vObjs.size(); ++i)
    {
        MapObject* Obj = vObjs[i];
        
        if((Obj->mObjectFrame.size() < 5))
            continue;

        if((Obj->mvpMapObjectMappoints.size() < 10) || (Obj->bBadErase == true))
            continue;

        // std::cout << "draw asso points" << std::endl;
        // draw associated map points
        {
            // std::vector<MapPoint *> owned_mappoints = Obj->mvpMapObjectMappoints; 
            std::vector<MapPoint *> owned_mappoints = Obj->mvpMapObjectMappointsFilter; 
            glPointSize(mPointSize * 5);
            glBegin(GL_POINTS);
            Eigen::Vector3f box_color = box_colors[Obj->mnId % box_colors.size()];
			glColor4f(box_color(0), box_color(1), box_color(2), 1.0f);
			for (size_t pt_id = 0; pt_id < owned_mappoints.size(); pt_id++)
			{
				MapPoint *mpt = owned_mappoints[pt_id];

				if (!mpt->isBad())
				{
					cv::Mat pos;
                    pos = mpt->GetWorldPos();
					if (pos.rows == 0)
						continue;
					glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                    // std::cout << "point xyz:" << pos << std::endl;
				}
			}
            // for(vector<MapPoint*>::iterator vit=owned_mappoints.begin(), vend=owned_mappoints.end(); vit!=vend; vit++)
            // {
            //     if((*vit)->isBad())
            //         continue;
            //     cv::Mat pos = (*vit)->GetWorldPos();
            //     glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
            // }
		    glEnd();
        }
    }
}

void MapDrawer::DrawOptiMapObjects()
{
    const vector<MapObject*> &vObjs = mpMap->GetAllMapObjects();
    for(size_t i = 0; i < vObjs.size(); ++i)
    {
        MapObject* Obj = vObjs[i];
        
        if((Obj->mObjectFrame.size() < 5))
            continue;

        if((Obj->mvpMapObjectMappoints.size() < 10) || (Obj->bBadErase == true))
            continue;

        // std::cout << "draw cuboid 1" << std::endl;
        // *************************************
        //    STEP 1. [EAO-SLAM] Draw cubes.   *
        // *************************************
        // if(((Obj->mnClass == 73) || (Obj->mnClass == 64) || (Obj->mnClass == 65) 
        //         || (Obj->mnClass == 66) || (Obj->mnClass == 56) || (Obj->mnClass == 72)))
        {
            bool bObjAsOrigin = true;

            if (Obj->pose.rows == 0)
                continue;
            // object center.
            if(bObjAsOrigin)
            {
                cv::Mat Twobj_t = Obj->pose.t();
                glPushMatrix();
                glMultMatrixf(Twobj_t.ptr<GLfloat>(0));
            }

            // draw object center.
            glPointSize(4*mPointSize);
            Eigen::Vector3f box_color = box_colors[Obj->mnId % box_colors.size()];
			glColor4f(box_color(0), box_color(1), box_color(2), 1.0f);
            glBegin(GL_POINTS);
            if(bObjAsOrigin)
                glVertex3f(0, 0, 0);
            else
                glVertex3f(Obj->mCenter3D.at<float>(0), Obj->mCenter3D.at<float>(1), Obj->mCenter3D.at<float>(2));
            glEnd();

            // ******************************************
            //                 7------6                 *
            //                /|     /|                 *
            //               / |    / |                 *
            //              4------5  |                 *
            //              |  3---|--2                 *
            //              | /    | /                  *
            //              0------1                    *
            // ******************************************

            glBegin(GL_LINES);
            if(bObjAsOrigin)
            {
                float lenth = Obj->lenth/2;
                float width = Obj->width/2;
                float height = Obj->height/2;

                if(Obj->mnClass == 0)
                {
                    glVertex3f(-lenth, -width, 0);      // 5
                    glVertex3f(lenth, -width, 0);       // 6

                    glVertex3f(lenth, -width, 0);       // 6
                    glVertex3f(lenth, width, 0);        // 7

                    glVertex3f(lenth, width, 0);        // 7
                    glVertex3f(-lenth, width, 0);       // 8

                    glVertex3f(-lenth, width, 0);       // 8
                    glVertex3f(-lenth, -width, 0);      // 5


                    glVertex3f(-lenth, -width, -height);    // 1
                    glVertex3f(-lenth, -width, 0);          // 5

                    glVertex3f(lenth, -width, -height);     // 2
                    glVertex3f(lenth, -width, 0);           // 6

                    glVertex3f(lenth, width, height);       // 9
                    glVertex3f(-lenth, width, height);      // 10

                    glVertex3f(lenth, width, -height);      // 3
                    glVertex3f(lenth, width, height);       // 9

                    glVertex3f(-lenth, width, -height);     // 4
                    glVertex3f(-lenth, width, height);      // 10
                }
                else
                {
                    // // chair, fixed scale, for better visulazation.
                    // if(Obj->mnClass == 56)
                    // {
                    //     lenth = 0.09;
                    //     width = 0.08;
                    //     height = 0.12;
                    // }

                    glVertex3f(-lenth, -width, -height);    // 1
                    glVertex3f(lenth, -width, -height);     // 2

                    glVertex3f(lenth, -width, -height);     // 2
                    glVertex3f(lenth, width, -height);      // 3

                    glVertex3f(lenth, width, -height);      // 3
                    glVertex3f(-lenth, width, -height);     // 4

                    glVertex3f(-lenth, width, -height);     // 4
                    glVertex3f(-lenth, -width, -height);    // 1

                    glVertex3f(-lenth, -width, height);     // 5
                    glVertex3f(lenth, -width, height);      // 6

                    glVertex3f(lenth, -width, height);      // 6
                    glVertex3f(lenth, width, height);       // 7

                    glVertex3f(lenth, width, height);       // 7
                    glVertex3f(-lenth, width, height);      // 8

                    glVertex3f(-lenth, width, height);      // 8
                    glVertex3f(-lenth, -width, height);     // 5

                    glVertex3f(-lenth, -width, -height);    // 1
                    glVertex3f(-lenth, -width, height);     // 5

                    glVertex3f(lenth, -width, -height);     // 2
                    glVertex3f(lenth, -width, height);      // 6

                    glVertex3f(lenth, width, -height);      // 3
                    glVertex3f(lenth, width, height);       // 7

                    glVertex3f(-lenth, width, -height);     // 4
                    glVertex3f(-lenth, width, height);      // 8
                }
                glEnd();
                glPopMatrix();
            }
        } // draw cubes END ----------------------------------------------------------------------------


        /*
        // std::cout << "draw quadrics" << std::endl;
        // // if(QuadricObj && !((Obj->mnClass == 73) || (Obj->mnClass == 64) || (Obj->mnClass == 65) 
        // //         || (Obj->mnClass == 66) || (Obj->mnClass == 56) || (Obj->mnClass == 72)))
        {
            // half axial length.
            float lenth = Obj->lenth/2;
            float width = Obj->width/2;
            float height = Obj->height/2;

            // tvmonitor, fixed scale, for better visulazation.
            // if(Obj->mnClass == 62)
            // {
            //     lenth = 0.13;
            //     width = 0.035;
            //     height = 0.08;
            // }
            // if(Obj->mnClass == 75)
            // {
            //     lenth = 0.08;
            //     width = 0.08;
            //     height = 0.08;
            // }

            cv::Mat axe = cv::Mat::zeros(3,1,CV_32F);
            axe.at<float>(0) = lenth;
            axe.at<float>(1) = width;
            axe.at<float>(2) = height;

            // quadrcis pose.
            cv::Mat Twq = cv::Mat::zeros(4,4,CV_32F);
            Twq.at<float>(0, 0) = 1;
            Twq.at<float>(0, 1) = 0;
            Twq.at<float>(0, 2) = 0;
            //Twq.at<float>(0, 3) = Obj->mCenter3D.at<float>(0);
            Twq.at<float>(0, 3) = Obj->cuboidCenter[0];
            Twq.at<float>(1, 0) = 0;
            Twq.at<float>(1, 1) = 1;
            Twq.at<float>(1, 2) = 0;
            //Twq.at<float>(1, 3) = Obj->mCenter3D.at<float>(1);
            Twq.at<float>(1, 3) = Obj->cuboidCenter[1];
            Twq.at<float>(2, 0) = 0;
            Twq.at<float>(2, 1) = 0;
            Twq.at<float>(2, 2) = 1;
            //Twq.at<float>(2, 3) = Obj->mCenter3D.at<float>(2);
            Twq.at<float>(2, 3) = Obj->cuboidCenter[2];
            Twq.at<float>(3, 0) = 0;
            Twq.at<float>(3, 1) = 0;
            Twq.at<float>(3, 2) = 0;
            Twq.at<float>(3, 3) = 1;

            // create a quadric.
            GLUquadricObj *pObj = gluNewQuadric();
            cv::Mat Twq_t = Twq.t();

            // color
            // cv::Scalar sc;
            // // sc = cv::Scalar(0, 255, 0);
            // sc = cv::Scalar(box_color(0), box_color(1), box_color(2));

            // add to display list
            glPushMatrix();
            glMultMatrixf(Twq_t.ptr<GLfloat >(0));
            glScalef(
                    (GLfloat)(axe.at<float>(0,0)),
                    (GLfloat)(axe.at<float>(0,1)),
                    (GLfloat)(axe.at<float>(0,2))
                    );

        std::cout << "draw cuboid 4" << std::endl;
            gluQuadricDrawStyle(pObj, GLU_LINE);
            gluQuadricNormals(pObj, GLU_NONE);
            glBegin(GL_COMPILE);
            Eigen::Vector3f box_color = box_colors[Obj->mnId % box_colors.size()];
			glColor4f(box_color(0), box_color(1), box_color(2), 1.0f);
            gluSphere(pObj, 1., 15, 10);// draw a sphere with radius 1.0, center (0,0,0), slices 26, and stacks 13.

            glEnd();
            glPopMatrix();
            // draw quadrics END ---------------------------------------------------------------------
        }
        */

    }
}

void MapDrawer::DrawOptiMapQuadrics()
{
    const vector<MapObject*> &vObjs = mpMap->GetAllMapObjects();
    for(size_t i = 0; i < vObjs.size(); ++i)
    {
        MapObject* Obj = vObjs[i];
        
        if((Obj->mObjectFrame.size() < 5))
            continue;

        if((Obj->mvpMapObjectMappoints.size() < 10) || (Obj->bBadErase == true))
            continue;


        // std::cout << "draw quadrics" << std::endl;
        // if(QuadricObj && !((Obj->mnClass == 73) || (Obj->mnClass == 64) || (Obj->mnClass == 65) 
        //         || (Obj->mnClass == 66) || (Obj->mnClass == 56) || (Obj->mnClass == 72)))
        {
            // half axial length.
            float lenth = Obj->lenth/2;
            float width = Obj->width/2;
            float height = Obj->height/2;

            // if(Obj->mnClass == 56)
            // {
            //     lenth = 0.09;
            //     width = 0.08;
            //     height = 0.12;
            // }
            // // tvmonitor, fixed scale, for better visulazation.
            // if(Obj->mnClass == 62)
            // {
            //     lenth = 0.13;
            //     width = 0.035;
            //     height = 0.08;
            // }
            // if(Obj->mnClass == 75)
            // {
            //     lenth = 0.08;
            //     width = 0.08;
            //     height = 0.08;
            // }

            cv::Mat axe = cv::Mat::zeros(3,1,CV_32F);
            axe.at<float>(0) = lenth;
            axe.at<float>(1) = width;
            axe.at<float>(2) = height;

            // quadrcis pose.
            cv::Mat Twq = cv::Mat::zeros(4,4,CV_32F);
            Twq.at<float>(0, 0) = 1;
            Twq.at<float>(0, 1) = 0;
            Twq.at<float>(0, 2) = 0;
            //Twq.at<float>(0, 3) = Obj->mCenter3D.at<float>(0);
            Twq.at<float>(0, 3) = Obj->cuboidCenter(0);
            Twq.at<float>(1, 0) = 0;
            Twq.at<float>(1, 1) = 1;
            Twq.at<float>(1, 2) = 0;
            //Twq.at<float>(1, 3) = Obj->mCenter3D.at<float>(1);
            Twq.at<float>(1, 3) = Obj->cuboidCenter(1);
            Twq.at<float>(2, 0) = 0;
            Twq.at<float>(2, 1) = 0;
            Twq.at<float>(2, 2) = 1;
            //Twq.at<float>(2, 3) = Obj->mCenter3D.at<float>(2);
            Twq.at<float>(2, 3) = Obj->cuboidCenter(2);
            Twq.at<float>(3, 0) = 0;
            Twq.at<float>(3, 1) = 0;
            Twq.at<float>(3, 2) = 0;
            Twq.at<float>(3, 3) = 1;

            // create a quadric.
            GLUquadricObj *pObj = gluNewQuadric();
            cv::Mat Twq_t = Twq.t();

            // color
            // cv::Scalar sc;
            // // sc = cv::Scalar(0, 255, 0);
            // sc = cv::Scalar(box_color(0), box_color(1), box_color(2));

            // add to display list
            glPushMatrix();
            glMultMatrixf(Twq_t.ptr<GLfloat >(0));
            glScalef(
                    (GLfloat)(axe.at<float>(0,0)),
                    (GLfloat)(axe.at<float>(0,1)),
                    (GLfloat)(axe.at<float>(0,2))
                    );

            gluQuadricDrawStyle(pObj, GLU_LINE);
            gluQuadricNormals(pObj, GLU_NONE);
            glBegin(GL_COMPILE);
            Eigen::Vector3f box_color = box_colors[Obj->mnId % box_colors.size()];
			glColor4f(box_color(0), box_color(1), box_color(2), 1.0f);
            gluSphere(pObj, 1., 15, 10);// draw a sphere with radius 1.0, center (0,0,0), slices 26, and stacks 13.

            glEnd();
            glPopMatrix();
            // draw quadrics END ---------------------------------------------------------------------
        }
    }
}


void MapDrawer::SaveMapObjects()
{
    string cuboid_file = "MapObjects.txt";
    string cuboid_asso_points_file = "MapPoints.txt";
    string cuboid_keyframe = "MapKeyframe.txt";
    vector<MapObject*> all_map_obj;
    const vector<MapObject*> original_map_obj = mpMap->GetAllMapObjects();
    for (size_t i = 0; i < original_map_obj.size(); i++)
        if(original_map_obj[i]->bBadErase == false && original_map_obj[i]->mvpMapObjectMappoints.size()>10 && original_map_obj[i]->mObjectFrame.size()>5)
            all_map_obj.push_back(original_map_obj[i]);
    for (size_t i = 0; i < all_map_obj.size(); i++)
        for (size_t j = i+1; j < all_map_obj.size(); j++)
    {
        if(all_map_obj[i]->mnId > all_map_obj[j]->mnId)
        {
            MapObject* tmp = all_map_obj[i];
            all_map_obj[i] = all_map_obj[j];
            all_map_obj[j] = tmp;
        }
    }

    ofstream f_cuboid;
    ofstream f_points;
    ofstream f_frame;
    f_cuboid.open(cuboid_file.c_str());
    f_points.open(cuboid_asso_points_file.c_str());
    f_frame.open(cuboid_keyframe.c_str());

    for (size_t obj_idd = 0; obj_idd < all_map_obj.size(); obj_idd++)
    {
        MapObject *obj_tmp = all_map_obj[obj_idd];
        cout << "points in the object " << obj_tmp->mnId << " " << obj_tmp->mvpMapObjectMappointsFilter.size() << endl;
        f_cuboid << obj_tmp->mnClass << " " << obj_tmp->cuboidCenter(0) << " " << obj_tmp->cuboidCenter(1) << " " << obj_tmp->cuboidCenter(2) 
                << " " << obj_tmp->rotY << " " << obj_tmp->lenth << " " << obj_tmp->width << " " << obj_tmp->height << " " << obj_tmp->mnId << endl;
        for (int pt_idd = 0; pt_idd < obj_tmp->mvpMapObjectMappointsFilter.size()-1; pt_idd++)
        {
            MapPoint *pMP = obj_tmp->mvpMapObjectMappointsFilter[pt_idd];
            if(pMP->isBad())
                continue;
            cv::Mat pointPos = pMP->GetWorldPos();
            // cout << obj_tmp->mnClass << " " << pointPos.at<float>(0) << " " << pointPos.at<float>(1) << " " << pointPos.at<float>(2) 
            //     << " " << pMP->object_class << " " << pMP->object_id  << " " << pMP->mnFirstKFid << " " << pMP->mnId << endl;
            f_points << obj_tmp->mnClass << " " << pointPos.at<float>(0) << " " << pointPos.at<float>(1) << " " << pointPos.at<float>(2) 
                << " " << pMP->object_class << " " << pMP->object_id  << " " << pMP->mnFirstKFid << " " << pMP->mnId << endl;
        }
    }

    std::vector<KeyFrame*> all_key_frames = mpMap->GetAllKeyFrames();
    sort(all_key_frames.begin(),all_key_frames.end(),KeyFrame::lId);
    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    for (size_t frame_idd = 0; frame_idd < all_key_frames.size(); frame_idd++)
    {
        KeyFrame* pKFi = all_key_frames[frame_idd]; 
        if(pKFi->isBad())
            continue;
        cv::Mat R = pKFi->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKFi->GetCameraCenter();
        f_frame << setprecision(6) << pKFi->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        // cv::Mat framePos = pKFi->GetPoseInverse();
        // // cout << pKFi->mnId << " " << framePos.at<float>(0,3) << " " << framePos.at<float>(1,3) << " " << framePos.at<float>(2,3) << endl;
        // f_frame << pKFi->mnId << " " << framePos.at<float>(0,3) << " " << framePos.at<float>(1,3) << " " << framePos.at<float>(2,3) << endl;
    }

    f_cuboid.close();
    f_points.close();
    f_frame.close();
    cout << "object and points saved!" << endl;
}

/*
void MapDrawer::DrawOptiMapObjects() 
{
    const vector<MapObject*> &vObjs = mpMap->GetAllMapObjects();
    for(size_t i = 0; i < vObjs.size(); ++i)
    {
        MapObject* Obj = vObjs[i];
        
        if((Obj->mObjectFrame.size() < 5))
            continue;

        if((Obj->mvpMapObjectMappoints.size() < 10) || (Obj->bBadErase == true))
            continue;

        // g2o::ellipsoid e = obj_landmark->GetWorldPos();
        // g2o::SE3Quat TmwSE3 = e.pose.inverse();
        // Eigen::Vector3d scale = e.scale;

        g2o::SE3Quat TmwSE3 = Converter::toSE3Quat(Obj->pose).inverse();
        Eigen::Vector3d scale = Eigen::Vector3d(Obj->lenth, Obj->width, Obj->height);

        glPushMatrix();
        glLineWidth(mGraphLineWidth * 1);
        glColor3f(0.0f,1.0f,0.0f);

        GLUquadricObj* pObj;
        pObj = gluNewQuadric();
        gluQuadricDrawStyle(pObj,GLU_LINE);

        pangolin::OpenGlMatrix Twm;
        SE3ToOpenGLCameraMatrix(TmwSE3, Twm);
        glMultMatrixd(Twm.m);
        glScaled(scale[0],scale[1],scale[2]);

        gluSphere(pObj,1.0,16,8);
        //drawAxisNormal();
        //glPopMatrix();

        gluDeleteQuadric(pObj);
        glEnd();

        glPopMatrix();
    }

}

void MapDrawer::SE3ToOpenGLCameraMatrix(g2o::SE3Quat &matInSe3, pangolin::OpenGlMatrix &M)
{
    // eigen to cv
    Eigen::Matrix4d matEigen = matInSe3.to_homogeneous_matrix();
    cv::Mat matIn;
    cv::eigen2cv(matEigen, matIn);

    if(!matIn.empty())
    {
        cv::Mat Rwc(3,3,CV_64F);
        cv::Mat twc(3,1,CV_64F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = matIn.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*matIn.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<double>(0,0);
        M.m[1] = Rwc.at<double>(1,0);
        M.m[2] = Rwc.at<double>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<double>(0,1);
        M.m[5] = Rwc.at<double>(1,1);
        M.m[6] = Rwc.at<double>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<double>(0,2);
        M.m[9] = Rwc.at<double>(1,2);
        M.m[10] = Rwc.at<double>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<double>(0);
        M.m[13] = twc.at<double>(1);
        M.m[14] = twc.at<double>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}
*/

void MapDrawer::DrawMapTruthCameraPose()
{
    // read truth pose in tracking.h
    // Eigen::MatrixXd truth_poses(4,8); 
    // truth_poses << 
    // 1617728169.41141939, 0.010638, -0.000009, 0.000000, 0.000000, 0.000000, -0.000494, 1.000000,
    // 1617728176.45573926, 0.803737, -0.000614, 0.000000, 0.000000, 0.000000, -0.000199, 1.000000,
    // 1617728192.57064152, 3.331697, -0.003343, 0.000000, 0.000000, 0.000000, -0.000105, 1.000000,
    // 1617728192.57064152, 3.361697, -0.003343, 0.000000, 0.000000, 0.000000, -0.000105, 1.000000;

    if (truth_poses.rows() > 0)
    {
        glLineWidth(mGraphLineWidth * 2);
        glBegin(GL_LINE_STRIP); // line strip connects adjacent points
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        for (int pt_id = 0; pt_id < truth_poses.rows(); pt_id++)
        {
            // glVertex3f(truth_poses(pt_id, 0), truth_poses(pt_id, 1), truth_poses(pt_id, 2));
            glVertex3f(truth_poses(pt_id, 1), truth_poses(pt_id, 2), truth_poses(pt_id, 3));
        }
        glEnd();
    }
}

} //namespace ORB_SLAM
