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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

// rank.
typedef Eigen::Matrix<float,5,1> VI;
bool VIC(const VI& lhs, const VI& rhs) 
{
    int index = 2;
    return lhs[index] > rhs[index];
}

// [graph matching] add truth cam pose for visualization
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];

    // [object_graph] add initial setting
    // std::string data_folder = strSettingPath.substr(0, strSettingPath.find_last_of("/"));
    data_folder = strSettingPath.substr(0, strSettingPath.find_last_of("/")); // define as tracking::data_folder

    // read truth camera pose for visualization
    std::string truth_camera_file = data_folder + "/Ground-Truth.txt";
    Eigen::MatrixXd truth_frame_poses(100,8);// time, xyz, qwxyz
    truth_frame_poses.resize(100,8);
    if (!read_all_number_txt(truth_camera_file,truth_frame_poses))
       exit(-1);
    std::cout<< "truth_camera_pose file: " << truth_camera_file << " data size:  "<< truth_frame_poses.rows()<<std::endl;
	mpMapDrawer->truth_poses.resizeLike(truth_frame_poses);
	mpMapDrawer->truth_poses = truth_frame_poses;

    // the init pose should equal camera truth pose
	Eigen::MatrixXd init_pose_truth = truth_frame_poses.row(0).tail<7>(); // xyz, q1234
	// Eigen::MatrixXd init_pose_truth(7,1);
    // init_pose_truth << 0,0.203,0, -0.757679, -0.325457, 0.26267, 0.501004; // self annotation 20220122
    Eigen::Matrix4d Twc;
	Twc.setIdentity();
	Twc.block(0,0,3,3) = Eigen::Quaterniond(init_pose_truth(6),init_pose_truth(3),init_pose_truth(4),init_pose_truth(5)).toRotationMatrix();
	Twc.col(3).head(3) = Eigen::Vector3d(init_pose_truth(0), init_pose_truth(1), init_pose_truth(2));
    std::cout << "init_pose_truth:  " << init_pose_truth << std::endl;
	InitToGround = cv::Mat::eye(4, 4, CV_32F);
	for (int row = 0; row < 4; row++)
		for (int col = 0; col < 4; col++)
			InitToGround.at<float>(row, col) = float(Twc(row, col));
    std::cout << " InitToGround: \n" << InitToGround << std::endl;

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    // Retrieve bbox yolo name with timestep
    std::vector<std::string> vstrBbox; // define as tracking param
    std::string time_ex = to_string(timestamp);
    std::replace(time_ex.begin(), time_ex.end(), ',', '.');
    std::string bbox_2d_file = data_folder+"/yolo_txts/" + time_ex + ".txt";
    std::vector<double> vframe_id;
    // // read offline bbox: each row:  [x1y1wh]
    Eigen::MatrixXd bbox_offline_yolo(1,6);  // yolo detection [class, x1y1x2y2, error]
    bbox_offline_yolo.setZero();
    if (!read_all_number_txt(bbox_2d_file, bbox_offline_yolo))
    {    std::cout << "\033[31m bbox_offline_yolo: \033[0m" << bbox_offline_yolo << "\033[0m" << std::endl;
        exit(-1);}
    // std::cout << "bbox_offline_yolo: \n" << bbox_offline_yolo << std::endl;

    // convert 2d bbox to class Box_2D
    // std::vector<Box2D*> frame_box_2d_tmp; // defined in Tracking::frame_box_2d_tmp offline
    frame_box_2d_tmp.clear();
    for (int k = 0; k < bbox_offline_yolo.rows(); k++)
    {

        Box2D* box = new Box2D;
        box->class_id = bbox_offline_yolo(k,0);
        box->x = bbox_offline_yolo(k,1);
        box->y = bbox_offline_yolo(k,2);
        box->width = bbox_offline_yolo(k,3);
        box->height = bbox_offline_yolo(k,4);
        box->score = bbox_offline_yolo(k,5);
        box->mRectBBox = cv::Rect(box->x, box->y, box->width, box->height);
        // filter 2d box detection with name and size
        // if(box->class_id!=56 && box->class_id!=57 && box->class_id!=60 && 
        //     box->class_id!=73 && box->class_id!=66 && box->class_id!=58 )
        //     continue; // 56: char, 57: sofa/couch, 60: table //73: book 66: keyboard 58 plant
        // if(box->score<0.5)
        //     continue;  
        
        if(box->class_id!=56 && box->class_id!=57 && box->class_id!=60)
            continue;
        if(box->width<150 || box->height<150)
            continue; 
        frame_box_2d_tmp.push_back(box);
    }
    std::sort(frame_box_2d_tmp.begin(), frame_box_2d_tmp.end(), 
        [](Box2D* a, Box2D* b) -> bool { return a->score > b->score; });

    // // save to current frame, here only 2d bbox, will be updata again
    mCurrentFrame.mBbox.clear();
    mCurrentFrame.mBbox = frame_box_2d_tmp;


    // note: [EAO-SLAM] line detection.
	line_lbd_detect line_lbd_obj;
	line_lbd_obj.use_LSD = true;
	line_lbd_obj.line_length_thres = 15; // remove short edges
	cv::Mat all_lines_mat;
	line_lbd_obj.detect_filter_lines(mImGray, all_lines_mat);
	Eigen::MatrixXd all_lines_raw(all_lines_mat.rows, 4);
	for (int rr = 0; rr < all_lines_mat.rows; rr++)
		for (int cc = 0; cc < 4; cc++)
			all_lines_raw(rr, cc) = all_lines_mat.at<float>(rr, cc);
	// std::cout << "all_lines_raw: " << all_lines_raw << std::endl;
    // save to frame.
    mCurrentFrame.all_lines_eigen = all_lines_raw;

    Track();

    return mCurrentFrame.mTcw.clone();
}

// [graph matching] the mainly modifications are in the TrackWithMotionModel().
void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            mInitialSecendFrame = Frame(mCurrentFrame); // [EAO] the second frame when initialization.
            CreateInitialMapMonocular();
        }
    }
}

// [graph matchng] rotate the world coordinate to the initial frame.
void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

	std::cout << "\033[33m Created Mono Initial Map using 1st keyframe " << pKFini->mnId << " normal frame ID  " << pKFini->mnFrameId << "\033[0m" << std::endl;
	std::cout << "\033[33m Created Mono Initial Map using 2rd keyframe " << pKFcur->mnId << " normal frame ID  " << pKFcur->mnFrameId << "\033[0m" << std::endl;

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    std::cout << "Optimizer: Tracking, GlobalBundleAdjustemnt CreateInitialMapMonocular" << endl;
    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    // [graph matching] rotate the world coordinate to the initial frame (groundtruth provides the normal vector of the ground).
    // only use the groundtruth of the first frame.
    std::cout << "InitToGround \n" << InitToGround << std::endl;
    cv::Mat R = InitToGround.rowRange(0, 3).colRange(0, 3);
    cv::Mat t = InitToGround.rowRange(0, 3).col(3);
    cv::Mat Rinv = R.t();
    cv::Mat Ow = -Rinv * t;
    cv::Mat GroundToInit = cv::Mat::eye(4, 4, CV_32F);
    Rinv.copyTo(GroundToInit.rowRange(0, 3).colRange(0, 3));
    Ow.copyTo(GroundToInit.rowRange(0, 3).col(3));

    // transform initial pose and map to ground frame
    pKFini->SetPose(pKFini->GetPose() * GroundToInit);
    pKFcur->SetPose(pKFcur->GetPose() * GroundToInit);
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
    {
        if (vpAllMapPoints[iMP])
        {
            MapPoint *pMP = vpAllMapPoints[iMP];
            // pMP->SetWorldPos(pMP->GetWorldPos()*scaling_ratio);
            pMP->SetWorldPos(InitToGround.rowRange(0, 3).colRange(0, 3) * pMP->GetWorldPos() + InitToGround.rowRange(0, 3).col(3));
        }
    }
    // [graph matching] rotate the world coordinate to the initial frame -----------------------------------------------------------


    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

// [graph matching] inital object and track
bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // *****************************
    // STEP 1. construct 2D object *
    // *****************************
    std::vector<Box2D *> objs_2d = frame_box_2d_tmp;
    // std::cout << "Tracking: 1 objs_2d size : " << objs_2d.size() << std::endl;
    for (auto &obj : objs_2d)
    {
        obj->sum_pos_3d = cv::Mat::zeros(3, 1, CV_32F); // init zero for sum
        obj->bad = false;                           // init false for erase
    }

    // ***************************************
    // STEP 2. associate objects with points *
    // ***************************************
    AssociateBBoxWithPoints(objs_2d);

    // ***************************************
    // STEP 3. associate objects with lines *
    // ***************************************
    AssociateBBoxWithLines(objs_2d);

    // // ***************************************************
    // // add to STEP 2
    // // STEP 4. compute the mean and standard of points.*
    // // Erase outliers (camera frame) by boxplot.*
    // // **************************************************
    // for (auto &obj : objs_2d)
    // {
    //     // compute the mean and standard.
    //     obj->ComputeMeanAndStandardBbox();

    //     // If the object has too few points, ignore.
    //     if (obj->Obj_c_MapPonits.size() < 8)
    //         continue;

    //     // Erase outliers by boxplot.
    //     obj->RemoveOutliersByBoxPlot(mCurrentFrame);
    // }
    // // Erase outliers of obj_2d END ----------------------

    // **************************************************************************
    // STEP 5. construct the bounding box by object feature points in the image.*
    // **************************************************************************
    // bounding box detected by yolo |  bounding box constructed by object points.
    //  _______________                 //  _____________
    // |   *        *  |                // |   *        *|
    // |    *  *       |                // |    *  *     |
    // |*      *  *    |                // |*      *  *  |
    // | *   *    *    |                // | *   *    *  |
    // |   *       *   |                // |___*_______*_|
    // |_______________|
    ComputeNewBBoxByFeaturePoints(objs_2d);
    // construct bounding box by feature points END -----------------------------------

    // **********************************************************************************************
    // STEP 6. remove 2d bad bounding boxes.
    // Due to the complex scene and Yolo error detection, some poor quality objects need to be removed.
    // The strategy can be adjusted and is not unique, such as:
    // 1. objects overlap with too many object;
    // 2. objects with too few points;
    // 3. objects with too few points and on the edge of the image;
    // 4. objects too large and take up more than half of the image;
    // TODO and so on ......
    // **********************************************************************************************
    RemoveBoxByOverlapNamePointSize(objs_2d);
    // remove 2d bad bounding boxes END ------------------------------------------------------
    std::cout << "Tracking: 2d bbox size " << objs_2d.size() << std::endl;

    // ************************************
    // STEP 9. Initialize the object map  *
    // ************************************
    if (mbObjectIni == false && mCurrentFrame.mnId > mInitialSecendFrame.mnId)
    {
        std::cout << "Initialization: InitObjectMap(objs_2d) " << std::endl;
        InitObjectMap(objs_2d); // init once
    }

    // *************************************************************
    // STEP 7. copy objects in the last frame after initialization.*
    // STEP 8. Merges objects with 5-10 points  between two adjacent frames.
    // Advantage: Small objects with too few points, can be merged to keep them from being eliminated.
    // (The effect is not very significant.)
    // *************************************************************
    if ((mbObjectIni == true) && (mCurrentFrame.mnId > mnObjectIniFrameID))
    {
        // copy objects in the last frame.
        mCurrentFrame.mvLastFrameBbox = mLastFrame.mvFrameBbox;

        // // copy objects in the penultimate frame.
        // if (!mLastFrame.mvLastFrameBbox.empty())
        //     mCurrentFrame.mvLastLastFrameBbox = mLastFrame.mvLastFrameBbox;
    
        bool bMergeSmallBbox = true;
        if (bMergeSmallBbox)
        {
            // object in current frame.
            for (size_t k = 0; k < objs_2d.size(); ++k)
            {
                // ignore objects with more than 10 points.
                if (objs_2d[k]->Obj_c_MapPonits.size() >= 10)
                    continue;
                // object in last frame.
                for (size_t l = 0; l < mCurrentFrame.mvLastFrameBbox.size(); ++l)
                {
                    // ignore objects with more than 10 points.
                    if (mCurrentFrame.mvLastFrameBbox[l]->Obj_c_MapPonits.size() >= 10)
                        continue;
                    // merge two objects.
                    if (bboxOverlapratio(objs_2d[k]->mRectBBox, mCurrentFrame.mvLastFrameBbox[l]->mRectBBox) > 0.5)
                    {
                        objs_2d[k]->MergeTwoFrameBbox(mCurrentFrame.mvLastFrameBbox[l]);
                        break;
                    }
                }
            }
        }
    
    }
    // copy objects END -------------------------------------------------------


    // **************************************************************
    // STEP 10. Data association after initializing the object map. *
    // **************************************************************
    string mflag = "IoU";
    if((mbObjectIni == true) && (mCurrentFrame.mnId > mnObjectIniFrameID))
    {
        std::cout << "Tracking: ObjectDataAssociation " << std::endl;
        ObjectDataAssociation(objs_2d);
    }
    
    // **************************************************************
    // STEP 11. Update Map Object. *
    // step 10.3 remove objects with too few observations.
    // **************************************************************
    if ((mbObjectIni == true) && (mCurrentFrame.mnId > mnObjectIniFrameID))
    {
        std::vector<MapObject *> all_objects = mpMap->GetAllMapObjects();
        std::cout << "Tracking: object num in map: " << all_objects.size() << std::endl;

        for (int i = (int)all_objects.size() - 1; i >= 0; i--)
        {
            if (all_objects[i]->bBadErase)
                continue;

            int nObsFrame = (int)all_objects[i]->mObjectFrame.size();
            // observed more than 10 frame, keep 
            if(nObsFrame > 10) 
                continue;
            // not been observed in the last 30 frames.
            else if (all_objects[i]->mnLastAddID < (mCurrentFrame.mnId - 30))
            {
                // not been observed in last 30 frames, and observation < 5
                if (nObsFrame < 5)
                    all_objects[i]->bBadErase = true;
                else // check overlap, when overlap, bad
                {
                    bool overlap = false;
                    for (int j = (int)all_objects.size() - 1; j >= 0; j--)
                    {
                        if (all_objects[j]->bBadErase || (i == j))
                            continue;
                        if (all_objects[i]->WhetherOverlap(all_objects[j]))
                        {
                            overlap = true;
                            break;
                        }
                    }
                    if (overlap)
                        all_objects[i]->bBadErase = true;
                }
            }
        }

        // step 10.4 Update the co-view relationship between objects. (appears in the same frame).
        for (int i = (int)all_objects.size() - 1; i >= 0; i--)
        {
            if (all_objects[i]->mnLastAddID == mCurrentFrame.mnId)
            {
                for (int j = (int)all_objects.size() - 1; j >= 0; j--)
                {
                    if (i == j)
                        continue;

                    if (all_objects[j]->mnLastAddID == mCurrentFrame.mnId)
                    {
                        int nObjId = all_objects[j]->mnId;

                        map<int, int>::iterator sit;
                        sit = all_objects[i]->mmAppearSametime.find(nObjId);

                        if (sit != all_objects[i]->mmAppearSametime.end())
                        {
                            int sit_sec = sit->second;
                            all_objects[i]->mmAppearSametime.erase(nObjId);
                            all_objects[i]->mmAppearSametime.insert(make_pair(nObjId, sit_sec + 1));
                        }
                        else
                            all_objects[i]->mmAppearSametime.insert(make_pair(nObjId, 1));   // first co-view.
                    }
                }
            }
        }

        // step 10.5 Merge potential associate objects (see mapping thread).

        // step 10.6 Estimate the orientation of objects.
        for (int i = (int)all_objects.size() - 1; i >= 0; i--)
        {
            // map object.
            MapObject* objMap = all_objects[i];

            if (objMap->bBadErase)
                continue;

            if (objMap->mnLastAddID < mCurrentFrame.mnId - 5)
                continue;

            // estimate only regular objects.
            // if (((objMap->mnClass == 73) || (objMap->mnClass == 64) || (objMap->mnClass == 65) 
            //     || (objMap->mnClass == 57) || (objMap->mnClass == 56)|| (objMap->mnClass == 72) ))
            {
                // // objects appear in current frame.
                if(objMap->mnLastAddID == mCurrentFrame.mnId)
                {
                    std::cout << "Tracking: obj info: " << objMap->mnClass << " id " << objMap->mnLastAddID << " obj yaw before" << objMap->rotY << std::endl;
                    SampleObjYaw(objMap);   // note: sample object yaw.
                    std::cout << "Tracking: obj info: " << objMap->mnClass << " id " << objMap->mnLastAddID << " obj yaw after" << objMap->rotY << std::endl;
                }
                // std::cout << "Tracking: obj info: " << objMap->mnClass << " obj yaw " << objMap->rotY << std::endl;
                // mCurrentFrame.m3DObject.push_back(objMap); // do it when init
            }

        }

    } // data association END ----------------------------------------------------------------




    
    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}

bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

// [graph matching] associate objects with feature points that inside the bounding box.
void Tracking::AssociateBBoxWithPoints(vector<Box2D *> objs_2d)
{
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    // associate keypoint to bbox
    for (int i = 0; i < mCurrentFrame.mvpMapPoints.size(); i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
            if (!pMP->isBad())
            {
                for (size_t k = 0; k < objs_2d.size(); ++k)
                {
                    if (objs_2d[k]->mRectBBox.contains(mCurrentFrame.mvKeysUn[i].pt))// in rect.
                    {
                        cv::KeyPoint feature = mCurrentFrame.mvKeysUn[i];
                        // object points.
                        objs_2d[k]->Obj_c_MapPonits.push_back(pMP);
                        objs_2d[k]->Obj_Feature_2D.push_back(feature);

                        // summation the position of points.
                        cv::Mat PointPosWorld = pMP->GetWorldPos();                 // world frame.
                        // cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;         // camera frame.
                        // pMP->feature = mCurrentFrame.mvKeysUn[i]; // coordinate in current frame.
                        objs_2d[k]->sum_pos_3d += PointPosWorld;
                    }
                }
            }
        }
    }

    // Erase outliers by boxplot.
    for (size_t k = 0; k < objs_2d.size(); ++k)
    {
       Box2D *obj = objs_2d[k];
       obj->RemoveOutliersByBoxPlot(mCurrentFrame);
    }

} // AssociateObjAndPoints() END -----------------------------------

// BRIEF [EAO] associate objects with lines.
void Tracking::AssociateBBoxWithLines(vector<Box2D *> objs_2d)
{
    
    // all lines in current frame.
    Eigen::MatrixXd AllLinesEigen = mCurrentFrame.all_lines_eigen;

    // step 1 make sure edges start from left to right.
    // align_left_right_edges(AllLinesEigen);
    // // make sure edges start from left to right
    // void align_left_right_edges(MatrixXd& all_lines)
    // {
    for (int line_id=0; line_id < AllLinesEigen.rows(); line_id++)
    {
        if (AllLinesEigen(line_id,2) < AllLinesEigen(line_id,0))
        {
            Vector2d temp = AllLinesEigen.row(line_id).tail<2>();
            AllLinesEigen.row(line_id).tail<2>() = AllLinesEigen.row(line_id).head<2>();
            AllLinesEigen.row(line_id).head<2>() = temp;
        }
    }
    // }
    for(int i = 0; i < (int)objs_2d.size(); i++)
    {
        Box2D* bbox = objs_2d[i];

        // step 2. expand the bounding box.
        double dLeftExpand = std::max(0.0, bbox->x - 15.0);
        double dRightExpand = std::min(float(mImageWidth), bbox->x + bbox->width + 15);
        double dTopExpand = std::max(0.0, bbox->y - 15.0);
        double dBottomExpand = std::min(float(mImageHeight), bbox->y + bbox->height + 15);
        // cv::Rect ExpandBBox = cv::Rect(dLeftExpand, dTopExpand, dRightExpand-dLeftExpand, dBottomExpand-dRightExpand)
        Eigen::Vector2d ExpanLeftTop = Vector2d(dLeftExpand, dTopExpand);			// lefttop.
		Eigen::Vector2d ExpanRightBottom = Vector2d(dRightExpand, dBottomExpand);  // rightbottom.

        // step 3. associate object with lines.
        Eigen::MatrixXd ObjectLines(AllLinesEigen.rows(),AllLinesEigen.cols()); 
		int nInsideLinesNum = 0;
		for (int line_id = 0; line_id < AllLinesEigen.rows(); line_id++)
        {
            // // check endpoints of the lines, whether inside the box.
            // if(ExpandBBox.contains(pt_left) && ExpandBBox.contains(pt_righ) )
            if (check_inside_box(   AllLinesEigen.row(line_id).head<2>(), 
                                    ExpanLeftTop, 
                                    ExpanRightBottom ))
            {
                if(check_inside_box(AllLinesEigen.row(line_id).tail<2>(),
                                    ExpanLeftTop, 
                                    ExpanRightBottom ))
                {
                    ObjectLines.row(nInsideLinesNum) = AllLinesEigen.row(line_id);
                    nInsideLinesNum++;
                }
            }
        }

        // step 4. merge lines.
        double pre_merge_dist_thre = 20; 
		double pre_merge_angle_thre = 5; 
		double edge_length_threshold = 30;
	    Eigen::MatrixXd ObjectLinesAfterMerge;
		merge_break_lines(	ObjectLines.topRows(nInsideLinesNum), 
							ObjectLinesAfterMerge, 		// output lines after merge.
							pre_merge_dist_thre,		// the distance threshold between two line, 20 pixels.
							pre_merge_angle_thre, 		// angle threshold between two line, 5°.
							edge_length_threshold);		// length threshold, 30 pixels.

        // step 5. save object lines.
        bbox->mObjLinesEigen = ObjectLinesAfterMerge;
        mCurrentFrame.vObjsLines.push_back(ObjectLinesAfterMerge);
    }
} // AssociateObjAndLines() END ----------------------------------.

void Tracking::ComputeNewBBoxByFeaturePoints(vector<Box2D *> objs_2d)
{    
    for (auto &obj : objs_2d)
    {
        // record the coordinates of each point in the xy(uv) directions.
        vector<float> x_pt;
        vector<float> y_pt;
        for (size_t ttt = 0; ttt < obj->Obj_Feature_2D.size(); ttt++)
        {
            float u = obj->Obj_Feature_2D[ttt].pt.x;
            float v = obj->Obj_Feature_2D[ttt].pt.y;
            x_pt.push_back(u);
            y_pt.push_back(v);
        }
        
        if (x_pt.size() < 4) // ignore.
            continue;

        // extremum in xy(uv) direction
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];
        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        // make insure in the image.
        if (x_min < 0)
            x_min = 0;
        if (y_min < 0)
            y_min = 0;
        if (x_max > mImageWidth)
            x_max = mImageWidth;
        if (y_max > mImageHeight)
            y_max = mImageHeight;

        // the bounding box constructed by object feature points.
        obj->mRectFeaturePoints = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
    }
}

void Tracking::RemoveBoxByOverlapNamePointSize(vector<Box2D *> objs_2d)
{
    // overlap with too many objects.
    for (size_t f = 0; f < objs_2d.size(); ++f)
    {
        int num = 0;
        for (size_t l = 0; l < objs_2d.size(); ++l)
        {
            if (f == l)
                continue;
            if (bboxOverlapratioLatter(objs_2d[f]->mRectBBox, objs_2d[l]->mRectBBox) > 0.05)
                num++;
        }
        // overlap with more than 3 objects.
        if (num > 3)
            objs_2d[f]->bad = true;
    }
    // class name, point size.
    for (size_t f = 0; f < objs_2d.size(); ++f)
    {
        if (objs_2d[f]->bad)
            continue;

        // ignore the error detect by yolo.
        if ((objs_2d[f]->class_id == 0) || (objs_2d[f]->class_id == 63) || (objs_2d[f]->class_id == 15))
            objs_2d[f]->bad = true;

        // too large in the image.
        if ((float)objs_2d[f]->mRectBBox.area() / (float)(mImageWidth * mImageHeight) > 0.5)
            objs_2d[f]->bad = true;

        // too few object points.
        if (objs_2d[f]->Obj_c_MapPonits.size() < 5)
            objs_2d[f]->bad = true;

        // object points too few and the object on the edge of the image.
        else if ((objs_2d[f]->Obj_c_MapPonits.size() >= 5) && (objs_2d[f]->Obj_c_MapPonits.size() < 10))
        {
            if ((objs_2d[f]->mRectBBox.x < 20) || (objs_2d[f]->mRectBBox.y < 20) ||
                (objs_2d[f]->mRectBBox.x + objs_2d[f]->mRectBBox.width > mImageWidth - 20) ||
                (objs_2d[f]->mRectBBox.y + objs_2d[f]->mRectBBox.height > mImageHeight - 20))
            {
                objs_2d[f]->bad = true;
            }
        }

        // mark the object that on the edge of the image.
        if (((objs_2d[f]->mRectBBox.x < 5) || (objs_2d[f]->mRectBBox.y < 5) ||
            (objs_2d[f]->mRectBBox.x + objs_2d[f]->mRectBBox.width > mImageWidth - 5) ||
            (objs_2d[f]->mRectBBox.y + objs_2d[f]->mRectBBox.height > mImageHeight - 5)))
        {
            objs_2d[f]->bOnEdge = true;
        }

        // when the overlap is large, only one object remains.
        for (size_t l = 0; l < objs_2d.size(); ++l)
        {
            if (objs_2d[l]->bad)
                continue;

            if (f == l)
                continue;

            // retain objects which with high probability.
            if (bboxOverlapratio(objs_2d[f]->mRectBBox, objs_2d[l]->mRectBBox) > 0.3)
            {
                if (objs_2d[f]->score < objs_2d[l]->score)
                    objs_2d[f]->bad = true;

                else if (objs_2d[f]->score >= objs_2d[l]->score)
                    objs_2d[l]->bad = true;
            }
            // if one object surrounds another, keep the larger one.
            if (bboxOverlapratio(objs_2d[f]->mRectBBox, objs_2d[l]->mRectBBox) > 0.05)
            {
                if (bboxOverlapratioFormer(objs_2d[f]->mRectBBox, objs_2d[l]->mRectBBox) > 0.85)
                    objs_2d[f]->bad = true;
                if (bboxOverlapratioLatter(objs_2d[f]->mRectBBox, objs_2d[l]->mRectBBox) > 0.85)
                    objs_2d[l]->bad = true;
            }
        }
    }
    // erase the bad object.
    for (vector<Box2D *>::iterator it = objs_2d.begin(); it != objs_2d.end(); )
    {
        if ((*it)->bad == true)
            it = objs_2d.erase(it); // erase.
        else
        {
           ++it;
        }
    }
}

// BRIEF [EAO] Initialize the object map.
void Tracking::InitObjectMap(vector<Box2D *> objs_2d)
{
    int GoodObj = -1;
    for (auto &obj : objs_2d)
    {
        // Initialize the object map need enough points.
        if (obj->Obj_c_MapPonits.size() < 10)
        {
            std::cout << "Initialization: Init Object failed, less than 10 map points : " << obj->Obj_c_MapPonits.size() << std::endl;
            continue;
        }
        mbObjectIni = true;
        mnObjectIniFrameID = mCurrentFrame.mnId;
        
        InitSingleObject(obj);

        // GoodObj ++;
        // // Create an object in the map.
        // MapObject *MapObjectSingle = new MapObject();
        // MapObjectSingle->mObjectFrame.push_back(obj);   // 2D objects in each frame associated with this 3D map object.
        // MapObjectSingle->mnId = GoodObj;             // 3d objects in the map.
        // MapObjectSingle->mnClass = obj->class_id;      // object class.
        // MapObjectSingle->mnConfidence = 1;              // object confidence = mObjectFrame.size().
        // MapObjectSingle->mnLastAddID = mCurrentFrame.mnId;      // last added id.
        // MapObjectSingle->mnLastLastAddID = mCurrentFrame.mnId;  // last last added id.
        // MapObjectSingle->mLastRect = obj->mRectBBox;             // last rect.
        // MapObjectSingle->mSumPointsPos = obj->sum_pos_3d;       // accumulated coordinates of object points.
        // MapObjectSingle->mCenter3D = obj->ave_pos_3d;                 // 3d centre.
        // // add map point and save it to the object.
        // for (size_t i = 0; i < obj->Obj_c_MapPonits.size(); i++)
        // {
        //     MapPoint *pMP = obj->Obj_c_MapPonits[i];
        //     if(!pMP->isBad())
        //     {
        //         pMP->object_id = MapObjectSingle->mnId;                            
        //         pMP->object_class = MapObjectSingle->mnClass;                      
        //         pMP->object_id_vector.insert(make_pair(MapObjectSingle->mnId, 1)); // the point is first observed by the object.
        //         MapObjectSingle->mvpMapObjectMappoints.push_back(pMP);                
        //     }
        // }
        // // calcuate center after adding map points
        // MapObjectSingle->ComputeMeanAndStandard(); // 3D object init, add pose
        // std::cout << "Initialization: Init Object success object center : " << MapObjectSingle->mCenter3D.t() 
        //     << "pose \n " << MapObjectSingle->pose << std::endl;
        // // update 2d object, associate current frame
        // obj->mnId = MapObjectSingle->mnId;
        // // update current frame, add frame bbox and 3D object
        // mCurrentFrame.mvFrameBbox.push_back(obj);
        // mCurrentFrame.m3DObject.push_back(MapObjectSingle);
        // // update map 
        // // mpMap->mspMapObjects.push_back(MapObjectSingle);
        // mpMap->AddMapObject(MapObjectSingle);// add to  mpMap->mspMapCuboids
        // std::cout << "Initialization: Init Object Map success mpMap objects : " << mpMap->MapObjectsInMap() << std::endl;
    }
} // initialize the object map. END -----------------------------------------------------

void Tracking::InitSingleObject(Box2D *obj)
{
    int obj_exit_num = mpMap->MapObjectsInMap();

    // Create an object in the map.
    MapObject *MapObjectSingle = new MapObject();
    MapObjectSingle->mObjectFrame.push_back(obj);   // 2D objects in each frame associated with this 3D map object.
    MapObjectSingle->mnId = obj_exit_num;             // 3d objects in the map.
    MapObjectSingle->mnClass = obj->class_id;      // object class.
    MapObjectSingle->mnConfidence = 1;              // object confidence = mObjectFrame.size().
    MapObjectSingle->mnLastAddID = mCurrentFrame.mnId;      // last added id.
    MapObjectSingle->mnLastLastAddID = mCurrentFrame.mnId;  // last last added id.
    MapObjectSingle->mLastRect = obj->mRectBBox;             // last rect.
    MapObjectSingle->mSumPointsPos = obj->sum_pos_3d;       // accumulated coordinates of object points.
    MapObjectSingle->mCenter3D = obj->ave_pos_3d;                 // 3d centre.
    MapObjectSingle->mObjectFrameId.push_back(mCurrentFrame.mnId);      
    
    // add map point and save it to the object.
    for (size_t i = 0; i < obj->Obj_c_MapPonits.size(); i++)
    {
        MapPoint *pMP = obj->Obj_c_MapPonits[i];
        if(!pMP->isBad())
        {
            pMP->object_id = MapObjectSingle->mnId;                            
            pMP->object_class = MapObjectSingle->mnClass;                      
            pMP->object_id_vector.insert(make_pair(MapObjectSingle->mnId, 1)); // the point is first observed by the object.
            MapObjectSingle->mvpMapObjectMappoints.push_back(pMP);                
        }
    }
    // calcuate center after adding map points
    MapObjectSingle->ComputeMeanAndStandard(); // 3D object init, add pose
    std::cout << "Initialization: Init Object success object center : " << MapObjectSingle->mCenter3D.t() 
        << "pose \n " << MapObjectSingle->pose << std::endl;

    // update 2d object, associate current frame
    obj->mnId = MapObjectSingle->mnId;
    // update current frame, add frame bbox and 3D object
    mCurrentFrame.mvFrameBbox.push_back(obj);
    mCurrentFrame.m3DObject.push_back(MapObjectSingle);

    // update map 
    // mpMap->mspMapObjects.push_back(MapObjectSingle);
    mpMap->AddMapObject(MapObjectSingle);// add to  mpMap->mspMapCuboids
    std::cout << "Initialization: Init Object Map success mpMap objects : " << mpMap->MapObjectsInMap() << std::endl;
}

void Tracking::ObjectDataAssociation(vector<Box2D *> objs_2d)
{
    // // step 10.1 points of the object that appeared in the last 30 frames 
    // // are projected into the image to form a projection bounding box.
    // for (int i = 0; i < (int)all_objects.size(); i++)
    // {
    //     if (all_objects[i]->bBadErase)
    //         continue;
    //     // object appeared in the last 30 frames.
    //     if (all_objects[i]->mnLastAddID > mCurrentFrame.mnId - 30)
    //         all_objects[i]->ComputeProjectRectFrame(mImageWidth, mImageHeight, mCurrentFrame);
    //     // else
    //     {
    //         all_objects[i]->mRectProject = cv::Rect(0, 0, 0, 0);
    //     }
    // }

    // step 10.2 data association.
    for (auto &obj : objs_2d)
    {
        // ignore object with less than 5 points.
        if (obj->Obj_c_MapPonits.size() < 5)
        {
            // obj->few_mappoint = true;
            // obj->current = false;
            std::cout << "Asso: Obj_c_MapPonits<5, skip association " << std::endl;
            continue;
        }

        cv::Rect RectCurrent = obj->mRectBBox;    // object bounding box in current frame.
        cv::Rect RectPredict;               // predicted bounding box according to last frame and next to last frame.
        // cv::Rect RectProject;               // bounding box constructed by projecting points.
        float IouMax = 0;
        bool bAssoByIou = false;            // whether associated by IoU.
        // int nAssoByIouId = -1;              // the associated map object ID.
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

        for (int i = 0; i < (int)all_objects.size(); i++)
        {
            if (all_objects[i]->mnClass != obj->class_id)
            {
                // std::cout << "Asso: object filter by class id " << std::endl;
                continue;
            }
            if (all_objects[i]->bBadErase)
            {
                // std::cout << "Asso: object filter by bBadErase " << std::endl;
                continue;                
            }

            // std::cout << "Asso: all_objects[i]->mnLastAddID: " << all_objects[i]->mnLastAddID << std::endl;
            // std::cout << "Asso: mCurrentFrame.mnId: " << mCurrentFrame.mnId  << std::endl;
            // if (objMap->mnLastAddID < mCurrentFrame.mnId - 5)
            if (all_objects[i]->mnLastAddID > mCurrentFrame.mnId - 20)// seen with in 20 frame
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
                    if (left_top_x > mImageWidth)
                        right_down_x = mImageWidth;
                    float right_down_y = (all_objects[i]->mLastRect.y + all_objects[i]->mLastRect.height) * 2 - (all_objects[i]->mLastLastRect.y + all_objects[i]->mLastLastRect.height);
                    if (left_top_y > mImageHeight)
                        right_down_y = mImageHeight;

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
        
        // step 1.3 if the association is successful, update the map object.
        std::cout << "IouMax: " << IouMax  << " IouMaxObjID: " << IouMaxObjID << std::endl;
        if ((IouMax > 0) && (IouMaxObjID >= 0))
            bAssoByIou = true;              // associated by IoU.
        
        if (bAssoByIou)
        {
            std::cout << "Asso: associated by existing object, update objects " << std::endl;
            bool bFlag = all_objects[IouMaxObjID]->DataAssociateUpdate(obj, mCurrentFrame, mImageWidth, mImageHeight, 1);
        }
        else
        {
            // If the object appears at the edge of the image, ignore.
            if ((obj->x < 10) || (obj->y < 10) ||
                (obj->x + obj->width > mImageWidth - 10) ||
                (obj->y + obj->height > mImageHeight - 10))
            {
                std::cout << "Asso: object at edge, skip association " << std::endl;
                obj->bad = true;
                break;
            }
            std::cout << "Asso: associated failed, init new objects " << std::endl;
            InitSingleObject(obj);
        }
        
        
        /*
        if ((IouMax > 0) && (IouMaxObjID >= 0))
        {
            // update.
            std::cout << "Asso: DataAssociateUpdate "  << std::endl;
            bool bFlag = all_objects[IouMaxObjID]->DataAssociateUpdate(obj, mCurrentFrame, mImageWidth, mImageHeight, 1);
            if (bFlag)
            {
                bAssoByIou = true;              // associated by IoU.
                // nAssoByIouId = IouMaxObjID;     // associated map object id.
            }
        }

        // Iou data association END ----------------------------------------------------------------------------

        if (bAssoByIou)
        {
            std::cout << "Asso: associated by existing object, not create new objects " << std::endl;
            break;
        }
        
        // If the object appears at the edge of the image, ignore.
        if ((obj->x < 10) || (obj->y < 10) ||
            (obj->x + obj->width > mImageWidth - 10) ||
            (obj->y + obj->height > mImageHeight - 10))
        {
            obj->bad = true;
            break;
        }

        // create a 3d object in the map.
        MapObject *MapObjectSingle = new MapObject();
        MapObjectSingle->mObjectFrame.push_back(obj);     
        MapObjectSingle->mnId = all_objects.size(); 
        MapObjectSingle->mnClass = obj->class_id;             
        MapObjectSingle->mnConfidence = 1;              
        // MapObjectSingle->mbFirstObserve = true;            
        // MapObjectSingle->mnAddedID = mCurrentFrame.mnId;
        MapObjectSingle->mnLastAddID = mCurrentFrame.mnId;
        MapObjectSingle->mnLastLastAddID = mCurrentFrame.mnId;
        MapObjectSingle->mLastRect = obj->mRectBBox;                 
        // MapObjectSingle->msFrameId.insert(mCurrentFrame.mnId); 
        MapObjectSingle->mSumPointsPos = obj->sum_pos_3d;           
        MapObjectSingle->mCenter3D = obj->ave_pos_3d;
        // this->mAssMapObjCenter = this->ave_pos_3d;
        std::cout << "Asso: add new object " << MapObjectSingle->mCenter3D.t() << std::endl;
        // add properties of the point and save it to the object.
        for (size_t i = 0; i < obj->Obj_c_MapPonits.size(); i++)
        {
            MapPoint *pMP = obj->Obj_c_MapPonits[i];
            if(!pMP->isBad())
            {
                pMP->object_id = MapObjectSingle->mnId;                           
                pMP->object_class = MapObjectSingle->mnClass;                    
                pMP->object_id_vector.insert(make_pair(MapObjectSingle->mnId, 1));
                // if (MapObjectSingle->mbFirstObserve == true) 
                //     pMP->First_obj_view = true;
                // save to the object.
                MapObjectSingle->mvpMapObjectMappoints.push_back(pMP);                
            }

        }
        MapObjectSingle->ComputeMeanAndStandard();
        obj->mnId = MapObjectSingle->mnId;
        // mnWhichTime = MapObjectSingle->mnConfidence;
        // current = true;

        // save this 2d object to current frame (associates with a 3d object in the map).
        mCurrentFrame.mvFrameBbox.push_back(obj);
        // mCurrentFrame.AppearNewObject = true;

        // will be added after filter. 
        mCurrentFrame.m3DObject.push_back(MapObjectSingle); 

        // update object map.
        // MapObjectSingle->IsolationForestDeleteOutliers();
        // all_objects.push_back(MapObjectSingle);
        mpMap->AddMapObject(MapObjectSingle);// add to  mpMap->mspMapCuboids
        std::cout << "Asso: mpMap->AddMapObject(MapObjectSingle) "  << std::endl;
        // create a new object END ------------------------------------------------------
    
        */
    }
}


// BRIEF [EAO] Estimate object orientation.
void Tracking::SampleObjYaw(MapObject* objMap)
{
    // // demo 1: compare the results without estimating the orientation.
    // if((mflag == "None") || (mflag == "iForest"))
    //     return;

    int numMax = 0;
    float fError = 0.0; // no used in this version.
    float fErrorYaw = 0.0;  // average angle error.
    // float minErrorYaw = 360.0;
    float sampleYaw = 0.0;
    int nAllLineNum = objMap->mObjectFrame.back()->mObjLinesEigen.rows();
    // std::cout << "nAllLineNum " << nAllLineNum << "\n" << objMap->mObjectFrame.back()->mObjLinesEigen << std::endl;
    
    for(int i = 0; i < 30; i++)
    {
        // initial angle.
        float roll, pitch, yaw;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        float error = 0.0;
        float errorYaw = 0.0;

        // 1 -> 15: -45° - 0°
        // 16 -> 30: 0° - 45°
        if(i < 15)
            yaw = (0.0 - i*3.0)/180.0 * M_PI;
        else
            yaw = (0.0 + (i-15)*3.0)/180.0 * M_PI;

        // object pose in object frame. (Ryaw)
        float cp = cos(pitch);
        float sp = sin(pitch);
        float sr = sin(roll);
        float cr = cos(roll);
        float sy = sin(yaw);
        float cy = cos(yaw);
        Eigen::Matrix<double,3,3> REigen;
        REigen<<   cp*cy, (sr*sp*cy)-(cr*sy), (cr*sp*cy)+(sr*sy),
                cp*sy, (sr*sp*sy)+(cr*cy), (cr*sp*sy)-(sr*cy),
                    -sp,    sr*cp,              cr * cp;
        cv::Mat Ryaw = Converter::toCvMat(REigen);

        // 8 vertices of the 3D box, world --> object frame.
        cv::Mat corner_1 = Converter::toCvMat(objMap->corner_1) - Converter::toCvMat(objMap->cuboidCenter);
        cv::Mat corner_2 = Converter::toCvMat(objMap->corner_2) - Converter::toCvMat(objMap->cuboidCenter);
        cv::Mat corner_3 = Converter::toCvMat(objMap->corner_3) - Converter::toCvMat(objMap->cuboidCenter);
        cv::Mat corner_4 = Converter::toCvMat(objMap->corner_4) - Converter::toCvMat(objMap->cuboidCenter);
        cv::Mat corner_5 = Converter::toCvMat(objMap->corner_5) - Converter::toCvMat(objMap->cuboidCenter);
        cv::Mat corner_6 = Converter::toCvMat(objMap->corner_6) - Converter::toCvMat(objMap->cuboidCenter);
        cv::Mat corner_7 = Converter::toCvMat(objMap->corner_7) - Converter::toCvMat(objMap->cuboidCenter);
        cv::Mat corner_8 = Converter::toCvMat(objMap->corner_8) - Converter::toCvMat(objMap->cuboidCenter);

        // rotate in object frame  + object frame --> world frame.
        corner_1 = Ryaw * corner_1 + Converter::toCvMat(objMap->cuboidCenter);
        corner_2 = Ryaw * corner_2 + Converter::toCvMat(objMap->cuboidCenter);
        corner_3 = Ryaw * corner_3 + Converter::toCvMat(objMap->cuboidCenter);
        corner_4 = Ryaw * corner_4 + Converter::toCvMat(objMap->cuboidCenter);
        corner_5 = Ryaw * corner_5 + Converter::toCvMat(objMap->cuboidCenter);
        corner_6 = Ryaw * corner_6 + Converter::toCvMat(objMap->cuboidCenter);
        corner_7 = Ryaw * corner_7 + Converter::toCvMat(objMap->cuboidCenter);
        corner_8 = Ryaw * corner_8 + Converter::toCvMat(objMap->cuboidCenter);

        // step 1. project 8 vertices to image.
        cv::Point2f point1, point2, point3, point4, point5, point6, point7, point8;
        point1 = WorldToImg(corner_1);
        point2 = WorldToImg(corner_2);
        point3 = WorldToImg(corner_3);
        point4 = WorldToImg(corner_4);
        point5 = WorldToImg(corner_5);
        point6 = WorldToImg(corner_6);
        point7 = WorldToImg(corner_7);
        point8 = WorldToImg(corner_8);

        // step 2. angle of 3 edges(lenth, width, height).
        float angle1;
        float angle2;
        float angle3;
        // left -> right.
        if(point6.x > point5.x)
            angle1 = atan2(point6.y - point5.y, point6.x - point5.x);
        else
            angle1 = atan2(point5.y - point6.y, point5.x - point6.x);
        float lenth1 = sqrt((point6.y - point5.y) * (point6.y - point5.y) + (point6.x - point5.x) * (point6.x - point5.x));

        if(point7.x > point6.x)
            angle2 = atan2(point7.y - point6.y, point7.x - point6.x);
        else
            angle2 = atan2(point6.y - point7.y, point6.x - point7.x);
        float lenth2 = sqrt((point7.y - point6.y) * (point7.y - point6.y) + (point7.x - point6.x) * (point7.x - point6.x));

        if(point6.x > point2.x)
            angle3 = atan2(point6.y - point2.y, point6.x - point2.x);
        else
            angle3 = atan2(point2.y - point6.y, point2.x - point6.x);
        float lenth3 = sqrt((point6.y - point2.y) * (point6.y - point2.y) + (point6.x - point2.x) * (point6.x - point2.x));

        // step 3. compute angle between detected lines and cube edges.
        int num = 0;
        for(int line_id = 0; line_id < objMap->mObjectFrame.back()->mObjLinesEigen.rows(); line_id++)
        {
            // angle of detected lines.
            double x1 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 0);
            double y1 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 1);
            double x2 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 2);
            double y2 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 3);
            float angle = atan2(y2 - y1, x2 - x1);

            // lenth.
            // float lenth = sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1));

            // angle between line and 3 edges.
            float dis_angle1 = abs(angle * 180/M_PI - angle1 * 180/M_PI);
            float dis_angle2 = abs(angle * 180/M_PI - angle2 * 180/M_PI);
            float dis_angle3 = abs(angle * 180/M_PI - angle3 * 180/M_PI);

            float th = 5.0;             // threshold of the angle.
            if(objMap->mnClass == 56)   // chair.
            {
                if((dis_angle2 < th) || (dis_angle3 < th))    
                    num++;
                if(dis_angle1 < th)
                {
                    num+=3;
                }
            }
            else
            {
                // the shortest edge is lenth1.
                if( min(min(lenth1, lenth2), lenth3) == lenth1)
                {
                    // error with other two edges.
                    if((dis_angle2 < th) || (dis_angle3 < th))
                    {
                        num++;
                        if(dis_angle2 < th)
                            error += dis_angle2;
                        if(dis_angle3 < th)
                            error += dis_angle3;
                    }

                    // angle error.
                    errorYaw+=min(dis_angle2, dis_angle3);
                }
                // the shortest edge is lenth2.
                if( min(min(lenth1, lenth2), lenth3) == lenth2)
                {
                    if((dis_angle1 < th) || (dis_angle3 < th))    
                    {
                        num++;
                        if(dis_angle1 < th)
                            error += dis_angle1;
                        if(dis_angle3 < th)
                            error += dis_angle3;
                    }
                    errorYaw+=min(dis_angle3, dis_angle1);
                }
                // the shortest edge is lenth3.
                if( min(min(lenth1, lenth2), lenth3) == lenth3)
                {
                    if((dis_angle1 < th) || (dis_angle2 < th))  
                    {
                        num++;
                        if(dis_angle1 < th)
                            error += dis_angle1;
                        if(dis_angle2 < th)
                            error += dis_angle2;
                    }
                    errorYaw+=min(dis_angle2, dis_angle1);
                }
            }
        }
        if(num == 0)
        {
            num = 1;
            errorYaw = 10.0;
        }

        // record the angle with max number parallel lines.
        if(num > numMax)
        {
            numMax = num;
            sampleYaw = yaw;

            fError = error; // no used in this version.
            // average angle error.
            fErrorYaw = (errorYaw/(float)num)/10.0;
        }
    }

    // step 4. scoring.
    float fScore;
    fScore = ((float)numMax / (float)nAllLineNum) * (1.0 - 0.1 * fErrorYaw);
    if(isinf(fScore))
        fScore = 0.0;

    // measurement： yaw, times, score, angle, angle error.
    Eigen::Matrix<float,5,1> AngleTimesAndScore;
    AngleTimesAndScore[0] = sampleYaw;
    AngleTimesAndScore[1] = 1.0;
    AngleTimesAndScore[2] = fScore;
    AngleTimesAndScore[3] = fError;     // no used in this version.
    AngleTimesAndScore[4] = fErrorYaw;

    // update multi-frame measurement.
    bool bNewMeasure = true;
    for (auto &row : objMap->mvAngleTimesAndScore)
    {
        if(row[0] == AngleTimesAndScore[0])
        {   
            row[1] += 1.0;
            row[2] = AngleTimesAndScore[2] * (1/row[1]) + row[2] * (1 - 1/row[1]);
            row[3] = AngleTimesAndScore[3] * (1/row[1]) + row[3] * (1 - 1/row[1]);
            row[4] = AngleTimesAndScore[4] * (1/row[1]) + row[4] * (1 - 1/row[1]);
            bNewMeasure = false;
        }
    }
    if(bNewMeasure == true)
    {
        objMap->mvAngleTimesAndScore.push_back(AngleTimesAndScore);
    }

    // step 5. rank.
    std::sort(objMap->mvAngleTimesAndScore.begin(),objMap->mvAngleTimesAndScore.end(),VIC);
    // for (auto &row : objMap->mvAngleTimesAndScore)
    // {
    //     std::cout << row[0] * 180.0 / M_PI  << "\t" <<  row[1] << "\t" <<  row[2] << std::endl;
    // }
    // the best yaw.
    int best_num = 0;
    float best_score = 0;
    for(int i = 0; i < std::min(3, (int)objMap->mvAngleTimesAndScore.size()); i++)
    {
        float fScore = objMap->mvAngleTimesAndScore[i][2];
        if(fScore >= best_score)
        {
            best_score = fScore;
            best_num = i;
        }
    }

    // step 6. update object yaw.
    objMap->rotY = objMap->mvAngleTimesAndScore[best_num][0];
    objMap->mfErrorParallel = objMap->mvAngleTimesAndScore[best_num][3];
    objMap->mfErrorYaw = objMap->mvAngleTimesAndScore[best_num][4];
    // std::cout << "best score " << objMap->rotY << std::endl;
    // objMap->rotY = 0.0;
    // objMap->mfErrorParallel = 0.0;
    // objMap->mfErrorYaw = 0.0;
} // SampleObjYaw() END -------------------------------------------------------------------------

// BRIEF [EAO] project points to image.
cv::Point2f Tracking::WorldToImg(cv::Mat &PointPosWorld)
{
    // world.
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    // camera.
    cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

    const float xc = PointPosCamera.at<float>(0);
    const float yc = PointPosCamera.at<float>(1);
    const float invzc = 1.0 / PointPosCamera.at<float>(2);

    // image.
    float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
    float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

    return cv::Point2f(u, v);
} // WorldToImg(cv::Mat &PointPosWorld) END ------------------------------



} //namespace ORB_SLAM
