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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM2
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0)
{
    mnCovisibilityConsistencyTh = 3;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }

            // add loop closure here
            if(DetectObjectLoop())
            {
                // Perform loop fusion and pose graph optimization
                CorrectObjectLoop();
            }
        }       

        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }

    mpCurrentKF->SetErase();
    return false;
}

bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    mpMap->InformNewBigChange();

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId;   
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);
            }
        }
    }
}


void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    int idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }            

            mpMap->InformNewBigChange();

            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LoopClosing::DistanceMatrix(vector<vector<float> > &centerpoint, Eigen::MatrixXi& label_matrix, Eigen::MatrixXf& distance_matrix)
{
    int num = centerpoint.size();
    distance_matrix.resize(num, num);
    label_matrix.resize(num, num);
    for (size_t i = 0; i < centerpoint.size(); i++)
    {
        Eigen::VectorXi node_vec_label(num);
        Eigen::VectorXf node_vec_dist(num);
        float x0 = centerpoint[i][0];
        float y0 = centerpoint[i][1];
        float z0 = centerpoint[i][2];
        for (size_t j = 0; j < centerpoint.size(); j++)
        {
            float x1 = centerpoint[j][0];
            float y1 = centerpoint[j][1];
            float z1 = centerpoint[j][2];
            // node_vec_dist(j) = sqrt(pow(x0-x1,2)+ pow(y0-y1,2) + pow(z0-z1,2));
            node_vec_dist(j) = sqrt(pow(x0-x1,2)+ pow(y0-y1,2));
            // node_vec_label(j) = j;
            node_vec_label(j) = centerpoint[j][3];
        }
        // std::cout << "node_vec_dist: " << node_vec_dist.transpose() << std::endl;
        // sort the vector by distance
        for (size_t m = 0; m < node_vec_dist.size(); m++)
            for (size_t n = m+1; n < node_vec_dist.size(); n++)
        {
            if(node_vec_dist(m) > node_vec_dist(n))
            {
                float dis_tmp = node_vec_dist(m);
                node_vec_dist(m) = node_vec_dist(n);
                node_vec_dist(n) = dis_tmp;
                int label_tmp = node_vec_label(m);
                node_vec_label(m) = node_vec_label(n);
                node_vec_label(n) = label_tmp;
            }
        }
        distance_matrix.row(i) = node_vec_dist;
        label_matrix.row(i) = node_vec_label;
        // std::cout << "node_vec_label: " << node_vec_label.transpose() << std::endl;
    }
}

void LoopClosing::OrientHistDes(vector<vector<float> > &centerpoint, Eigen::MatrixXi& orient_matrix)
{
    int num = centerpoint.size();
    orient_matrix.resize(num, 48);
    for (size_t i = 0; i < centerpoint.size(); i++)
    {
        Eigen::VectorXi hist_vec(48);
        hist_vec.setZero();
        Eigen::VectorXi node_vec_label(num);
        Eigen::VectorXf node_vec_orient(num);
        float x0 = centerpoint[i][0];
        float y0 = centerpoint[i][1];
        float z0 = centerpoint[i][2];
        for (size_t j = 0; j < centerpoint.size(); j++)
        {
            float x1 = centerpoint[j][0];
            float y1 = centerpoint[j][1];
            float z1 = centerpoint[j][2];
            // node_vec_dist(j) = sqrt(pow(x0-x1,2)+ pow(y0-y1,2) + pow(z0-z1,2));
            float orient_diff = atan2(y1-y0,x1-x0); // atan2 = gives angle value between -pi and pi
            int idx = centerpoint[j][3];
            // node_vec_label(j) = j;
            node_vec_label(j) = centerpoint[j][3];

            int div=8;
            for (size_t k = 0; k < div; k++)
            {
                if(orient_diff >= -M_PI+k*2*M_PI/div && orient_diff < -M_PI+(k+1)*2*M_PI/div)
                    hist_vec(k*6+idx) += 1;
            }

        }
        orient_matrix.row(i) = hist_vec;
        // std::cout << "hist_vec: " << hist_vec.transpose() << std::endl;
    }

}

void LoopClosing::getTransform(MatrixXf source, MatrixXf target, MatrixXf& R, MatrixXf& T, float& Scale, int iteNum)
{

    int size = source.rows();
    MatrixXf avergae_source(1, 3);
    MatrixXf avergae_target(1, 3);
    MatrixXf converage_source(size, 3);
    MatrixXf converage_target(size, 3);
    MatrixXf A(3, 3);

    MatrixXf R0(3, 3);
    MatrixXf T0(3, 1);
    float S0 = 0.0;
    
    Scale = 1.0;
    R.resize(3, 3);
    T.resize(3, 1);

    R.setIdentity();
    T.setZero();

    for(int ite=0; ite<iteNum; ite++){

        //initiallize some values
        avergae_source.setZero();
        avergae_target.setZero();
        converage_source.setZero();
        converage_target.setZero();

        //calculate the average value
        for(int i=0; i<size; i++){

            avergae_source(0, 0) = avergae_source(0, 0) + (source(i, 0)/size);
            avergae_source(0, 1) = avergae_source(0, 1) + (source(i, 1)/size);
            avergae_source(0, 2) = avergae_source(0, 2) + (source(i, 2)/size);

            avergae_target(0, 0) = avergae_target(0, 0) + (target(i, 0)/size);
            avergae_target(0, 1) = avergae_target(0, 1) + (target(i, 1)/size);
            avergae_target(0, 2) = avergae_target(0, 2) + (target(i, 2)/size);

        }

        //cout<<"source Average: "<<avergae_source<<endl;
        //cout<<"target Average: "<<avergae_target<<endl;

        //obtain the convaraience value of the points
        for(int i=0; i<size; i++){

            converage_source(i, 0) = source(i, 0) - avergae_source(0, 0);
            converage_source(i, 1) = source(i, 1) - avergae_source(0, 1);
            converage_source(i, 2) = source(i, 2) - avergae_source(0, 2);

            converage_target(i, 0) = target(i, 0) - avergae_target(0, 0);
            converage_target(i, 1) = target(i, 1) - avergae_target(0, 1);
            converage_target(i, 2) = target(i, 2) - avergae_target(0, 2);
        }

        // cout<<"source Converage: "<<converage_source<<endl;
        // cout<<"target Converage: "<<converage_target<<endl;

        A = converage_source.transpose() * converage_target;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();
        R0 = V * U.transpose();

        Eigen::MatrixXf rot_source(size, 3);
        for(int i=0; i<size; i++)
        {
            rot_source.row(i) = (R0*converage_source.row(i).transpose()).transpose();
        }
        // cout<<"rot_source: "<<rot_source<<endl;

        float top=0.0, bottom=0.0;
        for(int i=0; i<rot_source.rows(); i++)
        {
            Eigen::MatrixXf ttt, bbb;
            ttt = converage_target.row(i)*rot_source.row(i).transpose();
            bbb = rot_source.row(i)*rot_source.row(i).transpose();
            // cout<<"ttt: "<<ttt<<endl;
            // cout<<"bbb: "<<bbb<<endl;
            top += ttt(0,0);
            bottom += bbb(0,0);
        }
        S0 = top /bottom;

        T0 = avergae_target.transpose() - S0 * R0 * (avergae_source.transpose());
        // cout<<"S0: "<<S0<<endl;
        // cout<<"R0: "<<R0<<endl;
        // cout<<"T0: "<<T0.transpose()<<endl;
        // cout<<"source: "<<source<<endl;
        // cout<<"target: "<<target<<endl;

        for (int i = 0; i < source.rows(); i++) {
            source.row(i) = (S0*R0*source.row(i).transpose() + T0).transpose();
        }
        R = R0 * R;
        T = T0 + T;
        Scale = S0 * Scale;
        // cout<<"final: "<<source<<endl;
    }
}

bool LoopClosing::DetectObjectLoop()
{

    std::cout << "current keyframe Twc every: " << mpCurrentKF->GetPoseInverse().rowRange(0,3).col(3).t() << std::endl;
    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+20)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // sort object by id and remove bad object
    vector<MapObject*> all_map_obj;
    const vector<MapObject*> original_map_obj = mpMap->GetAllMapObjects();
    for (size_t i = 0; i < original_map_obj.size(); i++)
        if(original_map_obj[i]->bBadErase == false)
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

    //If the map contains less than 3 Map Objects, 3 for match, totally at least 6 objects
    if(all_map_obj.size() <= 6)
    {
        for (size_t obj_idd = 0; obj_idd < all_map_obj.size(); obj_idd++)
        {
            MapObject *obj_tmp = all_map_obj[obj_idd];
            cout << "points in the object " << obj_tmp->mnId << " " << obj_tmp->mvpMapObjectMappointsFilter.size() << endl;
            for (int pt_idd = 0; pt_idd < obj_tmp->mvpMapObjectMappointsFilter.size()-1; pt_idd++)
            {
                MapPoint *pMP = obj_tmp->mvpMapObjectMappointsFilter[pt_idd];
                if(pMP->isBad())
                    continue;
                cv::Mat pointPos = pMP->GetWorldPos();
                cout << obj_tmp->mnClass << " " << pointPos.at<float>(0) << " " << pointPos.at<float>(1) << " " << pointPos.at<float>(2) 
                 << " " << pMP->object_class << " " << pMP->object_id  << " " << pMP->mnFirstKFid << " " << pMP->mnId << endl;
            }
        }
        std::cout << "print map keyframe info " << std::endl;
        std::vector<KeyFrame*> all_key_frames = mpMap->GetAllKeyFrames();
        for (size_t frame_idd = 0; frame_idd < all_key_frames.size(); frame_idd++)
        {
            KeyFrame* pKFi = all_key_frames[frame_idd]; 
            if(pKFi->isBad())
                continue;
            
            cv::Mat framePos = pKFi->GetPoseInverse();
            cout << pKFi->mnId << " " << framePos.at<float>(0,3) << " " << framePos.at<float>(1,3) << " " << framePos.at<float>(2,3) << endl;
        }
        return false;
    }

    std::cout << "LoopClosure: start object loop detection " << std::endl;
    bool print_info = false;
    if(print_info && mpCurrentKF->mnFrameId > 2050)
    {
        std::cout << "print map object associated points info " << std::endl;
        for (size_t obj_idd = 0; obj_idd < all_map_obj.size(); obj_idd++)
        {
            MapObject *obj_tmp = all_map_obj[obj_idd];
            // cout << "points in the object " << obj_tmp->mnId << " " << obj_tmp->mvpMapObjectMappointsFilter.size() << endl;
            for (int pt_idd = 0; pt_idd < obj_tmp->mvpMapObjectMappointsFilter.size()-1; pt_idd++)
            {
                MapPoint *pMP = obj_tmp->mvpMapObjectMappointsFilter[pt_idd];
                if(pMP->isBad())
                    continue;
                cv::Mat pointPos = pMP->GetWorldPos();
                cout << obj_tmp->mnClass << " " << pointPos.at<float>(0) << " " << pointPos.at<float>(1) << " " << pointPos.at<float>(2) 
                 << " " << pMP->object_class << " " << pMP->object_id  << " " << pMP->mnFirstKFid << " " << pMP->mnId << endl;
            }
        }
        std::cout << "print map keyframe info " << std::endl;
        std::vector<KeyFrame*> all_key_frames = mpMap->GetAllKeyFrames();
        for (size_t frame_idd = 0; frame_idd < all_key_frames.size(); frame_idd++)
        {
            KeyFrame* pKFi = all_key_frames[frame_idd]; 
            if(pKFi->isBad())
                continue;
            
            cv::Mat framePos = pKFi->GetPoseInverse();
            cout << pKFi->mnId << " " << framePos.at<float>(0,3) << " " << framePos.at<float>(1,3) << " " << framePos.at<float>(2,3) << endl;
        }
        return false;
    }

    // divided map object into two group, recent 6 object and previous ...
    // Compute object similarity score
    vector<vector<float> > centerpoint1;
    vector<vector<float> > centerpoint2;
    size_t map_obj_cnt = all_map_obj.size();
    int cpoint1_num = map_obj_cnt/2;
    for (size_t i = 0; i < cpoint1_num; i++)
    {
        vector<float> point_tmp; // x,y,z,label
        point_tmp.resize(5);
        point_tmp[0] = all_map_obj[i]->cuboidCenter(0);
        point_tmp[1] = all_map_obj[i]->cuboidCenter(1);
        point_tmp[2] = all_map_obj[i]->cuboidCenter(2);
        point_tmp[3] = all_map_obj[i]->mnClass-55; // label
        point_tmp[4] = all_map_obj[i]->mnId; // id
        centerpoint1.push_back(point_tmp);
        std::cout << "cpoint1: " << point_tmp[0] << " " << point_tmp[1] << " " << point_tmp[2] 
                    << " " << point_tmp[3] << " " << point_tmp[4] << std::endl;
    }
    for (size_t j = cpoint1_num; j < map_obj_cnt; j++)
    {
        vector<float> point_tmp; // x,y,z,label
        point_tmp.resize(5);
        point_tmp[0] = all_map_obj[j]->cuboidCenter(0);
        point_tmp[1] = all_map_obj[j]->cuboidCenter(1);
        point_tmp[2] = all_map_obj[j]->cuboidCenter(2);
        point_tmp[3] = all_map_obj[j]->mnClass-55; // label
        point_tmp[4] = all_map_obj[j]->mnId; // id
        centerpoint2.push_back(point_tmp);
        // std::cout << "cpoint2: " << point_tmp[4] << " " << point_tmp[1] << std::endl;
        std::cout << "cpoint2: " << point_tmp[0] << " " << point_tmp[1] << " " << point_tmp[2] 
                    << " " << point_tmp[3] << " " << point_tmp[4] << std::endl;
    }
    
    
    // check loop detection
    // step 1: construct distance vector
    Eigen::MatrixXi label_matrix1, label_matrix2;
    Eigen::MatrixXf distance_matrix1, distance_matrix2;
    DistanceMatrix(centerpoint1, label_matrix1, distance_matrix1);
    DistanceMatrix(centerpoint2, label_matrix2, distance_matrix2);

    // step 2: construct orient histogram descriptor
    Eigen::MatrixXi orient_mat_1, orient_mat_2;
    OrientHistDes(centerpoint1, orient_mat_1);
    OrientHistDes(centerpoint2, orient_mat_2);
    // std::cout << "orient_mat_1: \n" << orient_mat_1 << std::endl;
    // std::cout << "orient_mat_2: \n" << orient_mat_2 << std::endl;
    
    
    // step 3: get a coarse matches
    std::vector<Eigen::Vector3f> coarse_matches; // id1, id2, similarity_score
    // return coarse_matches;
    // FindCoarseMatches(centerpoint1, centerpoint2, orient_mat_1, orient_mat_2);
    int Num1 = orient_mat_1.rows();
    int Num2 = orient_mat_2.rows();
    for (size_t i = 0; i < Num1; i++)
    {
        float score = 0;
        int label_1 = centerpoint1[i][3];
        int idx_1 = centerpoint1[i][4];
        Eigen::VectorXi vec1 = orient_mat_1.row(i);
        // std::cout << "vec 1: " << vec1.transpose() << std::endl;
        for (size_t j = 0; j < Num2; j++)
        {
            int label_2 = centerpoint2[j][3];
            int idx_2 = centerpoint2[j][4];
            Eigen::VectorXi vec2 = orient_mat_2.row(j);
            // if(label_1 != label_2 ) // check if label are the same and over 4 different id
            if(label_1 != label_2 || idx_2-idx_1 < 20) // check if label are the same and over 4 different id
            {
                // std::cout << "label_1 " << label_1 << " label_2 " << label_2 << std::endl;
                // std::cout << "idx_1 " << idx_1 << " idx_2 " << idx_2 << std::endl;
                score=0;
                continue;
            }

            // calculate the similarity by dot production: s=sum(A*B)/sqrt(sum(A^2)*sum(B^2))
            float SumTop = 0;
            for(int row = 0; row<vec1.size(); row++)
            {
                SumTop = SumTop + (vec1(row)*vec2(row));
            }
            float SumBottomLeft = 0;
            float SumBottomRight = 0;
            for(int row = 0; row<vec1.size(); row++)
            {
                SumBottomLeft = SumBottomLeft + pow(vec1(row), 2);
                SumBottomRight = SumBottomRight + pow(vec2(row), 2);
            }
            float SumBottom = sqrt(SumBottomLeft*SumBottomRight);
            if(SumBottom==0)
                score = 0;
            else
                score = abs(SumTop/SumBottom);

            Eigen::Vector3f score_mat(i,j,score);
            coarse_matches.push_back(score_mat);
            cout<< "score_mat(" << i << "," << j << "): "<< score <<endl;
        }
    }

    // checkpoint 2: coase match size()
    if(coarse_matches.size()<3)
    {
        std::cout << "coarse_matches < 3, no loop closure" << std::endl;
        return false;
    }
    
    int topk = coarse_matches.size();
    if(topk > 8)
        topk = 8;
    float min_similarity_thre = 0.3; // min_similarity score
    std::vector<Eigen::Vector3f> coarse_matches_topk;
    // FindTopKMatch(coarse_matches, topk, min_similarity_thre);
    for (size_t i = 0; i < coarse_matches.size(); i++)
        for (size_t j = i+1; j < coarse_matches.size(); j++)
    {
        if(coarse_matches[i](2) < coarse_matches[j](2))
        {
            Eigen::VectorXf tmp = coarse_matches[i];
            coarse_matches[i] = coarse_matches[j];
            coarse_matches[j] = tmp;
        }
    }
    // all matches have high similarity and at most 8 matches.
    for (size_t i = 0; i < topk; i++)
    {
        if(coarse_matches[i](2)>min_similarity_thre)
        {
            coarse_matches_topk.push_back(coarse_matches[i]);
            std::cout << "top_k_coarse_match: " << coarse_matches[i].transpose() << std::endl;
        }
        else
            break;
    }


    // step 4: geometry verification
    // step 4.1: check the scale of corresponding distance are similar
    std::vector<Eigen::VectorXf> geometry_matches; // match_id(2*3), error, scale 
    // FindGeometryMatches(centerpoint1, centerpoint2, coarse_matches_topk)
    size_t coarse_match_cnt = coarse_matches_topk.size();
    for (size_t i = 0; i < coarse_match_cnt; i++)
        for (size_t j = i+1; j < coarse_match_cnt; j++)
            for (size_t k = j+1; k < coarse_match_cnt; k++)
    {
        // check point id not duplicated
        Eigen::Vector3f match_i = coarse_matches_topk[i];
        Eigen::Vector3f match_j = coarse_matches_topk[j];
        Eigen::Vector3f match_k = coarse_matches_topk[k];
        if(match_i(0) == match_j(0) || match_i(1) == match_j(1) ||
        match_j(0) == match_k(0) || match_j(1) == match_k(1) ||
        match_i(0) == match_k(0) || match_i(1) == match_k(1) )
        {
            // std::cout << "wrong loop" << std::endl;
            continue;
        }
        // compute 2D distance and compare the scale
        float x0 = centerpoint1[match_i(0)][0];
        float y0 = centerpoint1[match_i(0)][1];
        float x1 = centerpoint1[match_j(0)][0];
        float y1 = centerpoint1[match_j(0)][1];
        float x2 = centerpoint1[match_k(0)][0];
        float y2 = centerpoint1[match_k(0)][1];
        float dist_1 = sqrt(pow(x0-x1,2)+ pow(y0-y1,2));
        float dist_2 = sqrt(pow(x1-x2,2)+ pow(y1-y2,2));
        float dist_3 = sqrt(pow(x0-x2,2)+ pow(y0-y2,2));
        float u0 = centerpoint2[match_i(1)][0];
        float v0 = centerpoint2[match_i(1)][1];
        float u1 = centerpoint2[match_j(1)][0];
        float v1 = centerpoint2[match_j(1)][1];
        float u2 = centerpoint2[match_k(1)][0];
        float v2 = centerpoint2[match_k(1)][1];
        float len_1 = sqrt(pow(u0-u1,2)+ pow(v0-v1,2));
        float len_2 = sqrt(pow(u1-u2,2)+ pow(v1-v2,2));
        float len_3 = sqrt(pow(u0-u2,2)+ pow(v0-v2,2));
        float scale_1 = dist_1/len_1;
        float scale_2 = dist_2/len_2;
        float scale_3 = dist_3/len_3;
        Eigen::Vector3f scale_vec(scale_1, scale_2, scale_3);
        float scale_ave = scale_vec.sum()/scale_vec.size();
        float error = pow(scale_1-scale_ave,2) + pow(scale_2-scale_ave,2) + pow(scale_3-scale_ave,2);
        std::cout << "id: " << match_i.head(2).transpose() << " " << match_j.head(2).transpose() << " " << match_k.head(2).transpose() << " ";
        std::cout << "scale: " << scale_1 << " " << scale_2 << " " << scale_3 << " " << error << std::endl;

        Eigen::VectorXf scale_mat(8); 
        scale_mat << match_i(0), match_i(1), match_j(0), match_j(1),
                    match_k(0), match_k(1), error, scale_ave;
        geometry_matches.push_back(scale_mat);
    }
    // checkpoint 3: geometry matches size()
    if(geometry_matches.size()<1)
    {
        std::cout << "geometry_matches < 1, no loop closure" << std::endl;
        return false;
    }

    // sort by error
    std::vector<Eigen::VectorXf> geometry_matches_topk;
    int topk2 = geometry_matches.size();
    float min_scale_thre = 0.01; // min_similarity score
    // FindTopKMatch(coarse_matches, topk2, min_similarity_thre);
    for (size_t i = 0; i < geometry_matches.size(); i++)
        for (size_t j = i+1; j < geometry_matches.size(); j++)
    {
        if(geometry_matches[i](6) > geometry_matches[j](6))
        {
            Eigen::VectorXf tmp = geometry_matches[i];
            geometry_matches[i] = geometry_matches[j];
            geometry_matches[j] = tmp;
        }
    }
    for (size_t i = 0; i < topk2; i++)
    {
        if(geometry_matches[i](6)<min_scale_thre)
        {
            geometry_matches_topk.push_back(geometry_matches[i]);
            std::cout << "top_k_geometry_match: " << geometry_matches_topk[i].transpose() << std::endl;
        }
        else
            break;
    }


    // step 5: collect all possible matches
    std::vector<Eigen::Vector3f> final_matches; //id1, id2, scale
    // FindFinalMatches(geometry_matches_topk)
    for (size_t i = 0; i < geometry_matches_topk.size(); i++)
        for (size_t j = 0; j < 3; j++) // every topḱ match has 3 id paar
    {
        Eigen::Vector3f id_tmp;
        bool label_exist = false;
        id_tmp(0) = geometry_matches_topk[i](2*j); // 0,2,4
        id_tmp(1) = geometry_matches_topk[i](2*j+1); // 1,3,5
        id_tmp(2) = geometry_matches_topk[i](7);
        // check if matched paar already exist
        for (size_t row = 0; row < final_matches.size(); row++)
        {
            if(id_tmp(0)==final_matches[row](0) || id_tmp(1)==final_matches[row](1))
            {
                label_exist = true;
                continue;
            }
        }
        if(label_exist)
            label_exist = false;
        else
            final_matches.push_back(id_tmp);
    }

    std::vector<int> print_id_list;
    for (size_t i = 0; i < final_matches.size(); i++)
    {
        std::cout << "final_matches: " << final_matches[i].transpose() << std::endl;
        print_id_list.push_back(final_matches[i](0));
        print_id_list.push_back(final_matches[i](1)+cpoint1_num);
    }
    
    for (size_t i = 0; i < final_matches.size(); i++)
        std::cout << "final_matches_id: " << centerpoint1[final_matches[i](0)][4] << " " << centerpoint2[final_matches[i](1)][4]<< std::endl;

    // checkpoint 4: inlier size,
    if(final_matches.size()>=3)
    {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!! detect object loop !!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        // // step 5: register all matches paar and calculate R and T
        // Eigen::MatrixXf Rotation; Eigen::MatrixXf Translation; float Scale;
        // // Alignment(centerpoint1, centerpoint2, final_matches)
        // int trans_cnt = final_matches.size();
        // Eigen::MatrixXf sourcePoints(trans_cnt, 3);
        // Eigen::MatrixXf targetPoints(trans_cnt, 3);
        // //generate the source cloud and target cloud
        // // for(int i=0; i<3; i++)
        // for(int i=0; i<trans_cnt; i++)
        // {
        //     int row1 = final_matches[i](0);
        //     int row2 = final_matches[i](1);
        //     targetPoints(i, 0) = centerpoint1[row1][0];
        //     targetPoints(i, 1) = centerpoint1[row1][1];
        //     targetPoints(i, 2) = centerpoint1[row1][2];
        //     sourcePoints(i, 0) = centerpoint2[row2][0];
        //     sourcePoints(i, 1) = centerpoint2[row2][1];
        //     sourcePoints(i, 2) = centerpoint2[row2][2];
        // }
        // getTransform(sourcePoints, targetPoints, Rotation, Translation, Scale, 100);
        // //getWeightedTransform(sourceInliers, targetInliers, Rotation, Translation, labelVector, 1000)
        // cout<<"Scale: "<<Scale<<endl;
        // cout<<"R: "<<Rotation<<endl;
        // cout<<"T: "<<Translation.transpose()<<endl;
        // Eigen::Vector3f eulerAngle;
        // rot_to_euler_zyx(Rotation, eulerAngle(0), eulerAngle(1), eulerAngle(2));
        // cout<<"Eular angle: "<<eulerAngle.transpose()/3.1415926*180<<endl;
        // corr_scale_init = Scale;
        // rot_21_init = Rotation;
        // trans_21_init = Translation;
        // corr_T21_init.setIdentity();
        // corr_T21_init.block(0,0,3,3) = Rotation;
        // corr_T21_init.col(3).head(3) = Translation;
        // cout<<"corr_T21_init:  "<<corr_T21_init<<endl;
        // for (size_t obj_idd = 0; obj_idd < print_id_list.size(); obj_idd++)
        // {
        //     int object_idddddd = print_id_list[obj_idd];
        //     MapObject *obj_tmp = all_map_obj[object_idddddd];
        //     cout << "points in the object " << obj_tmp->mnId << " " << obj_tmp->mvpMapObjectMappointsFilter.size() << endl;
        //     for (int pt_idd = 0; pt_idd < obj_tmp->mvpMapObjectMappointsFilter.size()-1; pt_idd++)
        //     {
        //         MapPoint *pMP = obj_tmp->mvpMapObjectMappointsFilter[pt_idd];
        //         if(pMP->isBad())
        //             continue;
        //         cv::Mat pointPos = pMP->GetWorldPos();
        //         cout << obj_tmp->mnClass << " " << pointPos.at<float>(0) << " " << pointPos.at<float>(1) << " " << pointPos.at<float>(2) 
        //          << " " << pMP->object_class << " " << pMP->object_id  << " " << pMP->mnFirstKFid << " " << pMP->mnId << endl;
        //     }
        // }
        
        Eigen::MatrixXf Rotation; 
        Eigen::MatrixXf Translation;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
        vector<vector<float> > cluster_center_1; // calculate scale
        vector<vector<float> > cluster_center_2; // calculate scale
        // generate the source cloud and target cloud
        int trans_cnt = final_matches.size();
        for(int i=0; i<trans_cnt; i++)
        {
            pcl::PointXYZRGB pt_tmp; // x,y,z,rgb
            int row1 = final_matches[i](0);
            int row2 = final_matches[i](1)+cpoint1_num;
            MapObject *obj_tmp = all_map_obj[row1];
            for (int pt_idd = 0; pt_idd < obj_tmp->mvpMapObjectMappointsFilter.size()-1; pt_idd++)
            {
                MapPoint *pMP = obj_tmp->mvpMapObjectMappointsFilter[pt_idd];
                if(pMP->isBad())
                    continue;
                cv::Mat pointPos = pMP->GetWorldPos();
                // cout << obj_tmp->mnClass << " " << pointPos.at<float>(0) << " " << pointPos.at<float>(1) << " " << pointPos.at<float>(2) 
                //  << " " << pMP->object_class << " " << pMP->object_id  << " " << pMP->mnFirstKFid << " " << pMP->mnId << endl;
                pt_tmp.x = pointPos.at<float>(0);
                pt_tmp.y = pointPos.at<float>(1);
                pt_tmp.z = pointPos.at<float>(2);
                cloud1->points.push_back(pt_tmp);
            }

            vector<float> point_center; // x,y,z,label
            point_center.resize(3);
            point_center[0] = (float)obj_tmp->cuboidCenter(0);
            point_center[1] = (float)obj_tmp->cuboidCenter(1);
            point_center[2] = (float)obj_tmp->cuboidCenter(2);
            cluster_center_1.push_back(point_center);

            obj_tmp = all_map_obj[row2];
            for (int pt_idd = 0; pt_idd < obj_tmp->mvpMapObjectMappointsFilter.size()-1; pt_idd++)
            {
                MapPoint *pMP = obj_tmp->mvpMapObjectMappointsFilter[pt_idd];
                if(pMP->isBad())
                    continue;
                cv::Mat pointPos = pMP->GetWorldPos();
                // cout << obj_tmp->mnClass << " " << pointPos.at<float>(0) << " " << pointPos.at<float>(1) << " " << pointPos.at<float>(2) 
                //  << " " << pMP->object_class << " " << pMP->object_id  << " " << pMP->mnFirstKFid << " " << pMP->mnId << endl;
                pt_tmp.x = pointPos.at<float>(0);
                pt_tmp.y = pointPos.at<float>(1);
                pt_tmp.z = pointPos.at<float>(2);
                cloud2->points.push_back(pt_tmp);
            }

            point_center[0] = (float)obj_tmp->cuboidCenter(0);
            point_center[1] = (float)obj_tmp->cuboidCenter(1);
            point_center[2] = (float)obj_tmp->cuboidCenter(2);
            cluster_center_2.push_back(point_center);
        }

        // calculate scale 
        float scale_ave = 0.0f;
        {
            float x0 = cluster_center_1[0][0];
            float y0 = cluster_center_1[0][1];
            float z0 = cluster_center_1[0][2];
            float x1 = cluster_center_1[1][0];
            float y1 = cluster_center_1[1][1];
            float z1 = cluster_center_1[1][2];
            float x2 = cluster_center_1[2][0];
            float y2 = cluster_center_1[2][1];
            float z2 = cluster_center_1[2][2];
            float dist_1 = sqrt(pow(x0-x1,2)+ pow(y0-y1,2) + pow(z0-z1,2));
            float dist_2 = sqrt(pow(x1-x2,2)+ pow(y1-y2,2) + pow(z1-z2,2));
            float dist_3 = sqrt(pow(x0-x2,2)+ pow(y0-y2,2) + pow(z0-z2,2));
            float u0 = cluster_center_2[0][0];
            float v0 = cluster_center_2[0][1];
            float w0 = cluster_center_2[0][2];
            float u1 = cluster_center_2[1][0];
            float v1 = cluster_center_2[1][1];
            float w1 = cluster_center_2[1][2];
            float u2 = cluster_center_2[2][0];
            float v2 = cluster_center_2[2][1];
            float w2 = cluster_center_2[2][2];
            float len_1 = sqrt(pow(u0-u1,2)+ pow(v0-v1,2) + pow(w0-w1,2));
            float len_2 = sqrt(pow(u1-u2,2)+ pow(v1-v2,2) + pow(w1-w2,2));
            float len_3 = sqrt(pow(u0-u2,2)+ pow(v0-v2,2) + pow(w0-w2,2));
            if(len_1==0 || len_2==0 || len_3==0)
                return false;
            float scale_1 = dist_1/len_1;
            float scale_2 = dist_2/len_2;
            float scale_3 = dist_3/len_3;
            Eigen::Vector3f scale_vec(scale_1, scale_2, scale_3);
            scale_ave = scale_vec.sum()/scale_vec.size();
            float error = pow(scale_1-scale_ave,2) + pow(scale_2-scale_ave,2) + pow(scale_3-scale_ave,2);
            std::cout << "scale: " << scale_1 << " " << scale_2 << " " << scale_3 << " " << error << std::endl;

            for (size_t i = 0; i < cloud2->points.size(); i++)
            {
                cloud2->points[i].x = cloud2->points[i].x * scale_ave;
                cloud2->points[i].y = cloud2->points[i].y * scale_ave;
                cloud2->points[i].z = cloud2->points[i].z * scale_ave;
            }
        }

        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(cloud2);         //设置源点云
        icp.setInputTarget(cloud1);       //设置目标点云
        pcl::PointCloud<pcl::PointXYZRGB> Final;//存储经过配准变换源点云后的点云
        icp.align(Final);                    //执行匹配存储变换后的源点云到Final

        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        Eigen::MatrixXi inlierID(3,2);
        inlierID.setZero();
        Rotation = icp.getFinalTransformation().block(0,0,3,3);
        Translation = icp.getFinalTransformation().col(3).head(3);
        std::cout << "Translation: \n" << Translation.transpose() << std::endl;
        std::cout << "Rotation: \n" << Rotation << std::endl;
        
        Eigen::Vector3f eulerAngle;
        rot_to_euler_zyx(Rotation, eulerAngle(0), eulerAngle(1), eulerAngle(2));
        eulerAngle(0) = (eulerAngle(0)/3.1415926)*180;
        eulerAngle(1) = (eulerAngle(1)/3.1415926)*180;
        eulerAngle(2) = (eulerAngle(2)/3.1415926)*180;
        cout<<"Eular angle: "<<eulerAngle.transpose()<<endl;
       
        if(Translation(0)<0 || Translation(1)>0 ||  Translation(2)>0 ||
            std::abs(eulerAngle(0))>15 || std::abs(eulerAngle(1))>15 || std::abs(eulerAngle(2))>15)
            return false;

        corr_scale_init = scale_ave;
        rot_21_init = Rotation;
        trans_21_init = Translation;
        corr_T21_init.setIdentity();
        corr_T21_init.block(0,0,3,3) = Rotation;
        corr_T21_init.col(3).head(3) = Translation;
        final_matches_tmp = final_matches;


        icp.setInputSource(cloud1);         //设置源点云
        icp.setInputTarget(cloud2);       //设置目标点云
        icp.align(Final);                    //执行匹配存储变换后的源点云到Final
        scale_12_init = 1.0/corr_scale_init;
        rot_12_init = icp.getFinalTransformation().block(0,0,3,3);
        trans_12_init = icp.getFinalTransformation().col(3).head(3);
        std::cout << "Translation: \n" << trans_12_init.transpose() << std::endl;
        std::cout << "Rotation: \n" << rot_12_init << std::endl;
        
        return true;
    }
    else
    {
        std::cout << "no loop, add object in the map" << std::endl;
        return false;
    }

}

void LoopClosing::CorrectObjectLoop()
{
    cout << "Loop Object detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);



    // // add scale, trans, rotation
    // // std::cout << "current keyframe GetPose(): \n" << mpCurrentKF->GetPose() << std::endl;
    // std::cout << "current keyframe GetPoseInverse(): \n" << mpCurrentKF->GetPoseInverse() << std::endl;
    // cv::Mat Twc_tmp = mpCurrentKF->GetPoseInverse();
    // Eigen::Matrix4f Twc_mat; 
	// for (int row = 0; row < 4; row++)
	// 	for (int col = 0; col < 4; col++)
    //         Twc_mat(row, col) = Twc_tmp.at<float>(row, col);
    // Eigen::Vector3f trans_new = corr_scale_init*rot_21_init*Twc_mat.col(3).head(3)+trans_21_init;
    // Eigen::Matrix3f rot_new = rot_21_init * Twc_mat.block(0,0,3,3);
    // std::cout << "current keyframe Twc after 1: " << trans_new.transpose() << std::endl;
    // cv::Mat Twc_final = cv::Mat::eye(4, 4, CV_32F);
	// for (int row = 0; row < 3; row++)
	// 	for (int col = 0; col < 3; col++)
	// 		Twc_final.at<float>(row, col) = float(rot_new(row, col));
    // Twc_final.at<float>(0, 3) = trans_new(0);
    // Twc_final.at<float>(1, 3) = trans_new(1);
    // Twc_final.at<float>(2, 3) = trans_new(2);
   
    
    // for (int row = 0; row < 4; row++)
	// 	for (int col = 0; col < 4; col++)
    //         Twc_mat(row, col) = Twc_tmp.at<float>(row, col);
    // Eigen::Matrix4f Twc_new_2 = Twc_mat * corr_T21_init.inverse();
    // std::cout << "current keyframe Twc after 2: " << Twc_new_2.col(3).transpose() << std::endl;

	// cv::Mat trans_mat = cv::Mat::eye(4, 4, CV_32F);
	// for (int row = 0; row < 4; row++)
	// 	for (int col = 0; col < 4; col++)
	// 		trans_mat.at<float>(row, col) = float(corr_T21_init(row, col));
    // cv::Mat R = trans_mat.rowRange(0, 3).colRange(0, 3);
    // cv::Mat t = trans_mat.rowRange(0, 3).col(3);
    // cv::Mat Rinv = R.t();
    // cv::Mat Ow = -Rinv * t;
    // cv::Mat trans_mat_12 = cv::Mat::eye(4, 4, CV_32F);
    // Rinv.copyTo(trans_mat_12.rowRange(0, 3).colRange(0, 3));
    // Ow.copyTo(trans_mat_12.rowRange(0, 3).col(3));
	// for (int row = 0; row < 3; row++)
	// 	for (int col = 0; col < 3; col++)
	// 		trans_mat_12.at<float>(row, col) = corr_scale_init*trans_mat_12.at<float>(row, col);
    // cv::Mat Twc_new_3 = Twc_tmp*trans_mat_12;
    // std::cout << "current keyframe Twc after 3: " << Twc_new_3 << std::endl;

    // mpCurrentKF->SetPose(Twc_final);
    // std::cout << "current keyframe Twc after: " << mpCurrentKF->GetPose() << std::endl;

    // Eigen::Matrix4d trans_curr;
	// for (int row = 0; row < 4; row++)
	// 	for (int col = 0; col < 4; col++)
    //         trans_curr(row, col) = Twc_final.at<float>(row, col);
    // Eigen::Matrix3d rot_curr = trans_curr.block(0,0,3,3);
    // Eigen::Vector3d tra_curr = trans_curr.col(3).head(3);

    // g2o::Sim3 rotation_init(rot_curr, tra_curr, (double)corr_scale_init);
    // g2o::Sim3 rotation_init(rot_new.cast<double>(), trans_new.cast<double>(), (double)corr_scale_init);
    // g2o::Sim3 rotation_init(rot_21_init.cast<double>(), trans_21_init.cast<double>(), (double)corr_scale_init);

    std::cout << "corr_scale_init: \n" << corr_scale_init << std::endl;
    std::cout << "rot_21_init: \n" << rot_21_init << std::endl;
    std::cout << "trans_21_init: \n" << trans_21_init.transpose() << std::endl;

    // std::cout << "scale_12_init: \n" << scale_12_init << std::endl;
    // std::cout << "rot_12_init: \n" << rot_12_init << std::endl;
    // std::cout << "trans_12_init: \n" << trans_12_init.transpose() << std::endl;
    // Eigen::Matrix4f T_12_new = corr_T21_init.inverse();
    // rot_12_init = T_12_new.block(0,0,3,3);
    // trans_12_init = T_12_new.col(3).head(3);
    // std::cout << "rot_12_init: \n" << rot_12_init << std::endl;
    // std::cout << "trans_12_init: \n" << trans_12_init.transpose() << std::endl;


    cv::Mat Twc = mpCurrentKF->GetPoseInverse();
    // Eigen::Matrix4f Twc_eigen;
    // for (int row = 0; row < 4; row++)
    //     for (int col = 0; col < 4; col++)
    //         Twc_eigen(row, col) = Twc.at<float>(row, col);
    // Eigen::Matrix3f rot_eigen = rot_21_init * Twc_eigen.block(0,0,3,3);
    // Eigen::Vector3f tran_eigen = corr_scale_init*rot_21_init*Twc_eigen.col(3).head(3)+trans_21_init;
    // cv::Mat Twc_new = cv::Mat::eye(4, 4, CV_32F);
    // for (int row = 0; row < 3; row++)
    //     for (int col = 0; col < 3; col++)
    //         Twc_new.at<float>(row, col) = float(rot_eigen(row, col));
    // Twc_new.at<float>(0, 3) = tran_eigen(0);
    // Twc_new.at<float>(1, 3) = tran_eigen(1);
    // Twc_new.at<float>(2, 3) = tran_eigen(2);
    // g2o::Sim3 gStt(rot_eigen, tran_eigen, 1.0);
    // mg2oScw = gStt.inverse();

    g2o::Sim3 gScm(rot_21_init.cast<double>(), trans_21_init.cast<double>(), (double)corr_scale_init);
    // g2o::Sim3 gScm(rot_12_init.cast<double>(), trans_12_init.cast<double>(), (double)scale_12_init);
    // g2o::Sim3 gSmw(Converter::toMatrix3d(mpCurrentKF->GetRotation()),Converter::toVector3d(mpCurrentKF->GetTranslation()),1.0);
    g2o::Sim3 gSmw(Converter::toMatrix3d(Twc.rowRange(0,3).colRange(0,3)),Converter::toVector3d(Twc.rowRange(0,3).col(3)),1.0);
    mg2oScw = gScm*gSmw;
    mg2oScw = mg2oScw.inverse();
    // cv::Mat gScm_mat = Converter::toCvMat(gScm);
    // cv::Mat gScm_t = cv::Mat::eye(4,4,gScm_mat.type());
    // cv::Mat RRcw = gScm_mat.rowRange(0,3).colRange(0,3);
    // cv::Mat ttcw = gScm_mat.rowRange(0,3).col(3);
    // cv::Mat RRwc = RRcw.t();
    // cv::Mat OOw = -RRwc*ttcw;
    // RRwc.copyTo(gScm_t.rowRange(0,3).colRange(0,3));
    // OOw.copyTo(gScm_t.rowRange(0,3).col(3));
    // g2o::Sim3 gScm_new(Converter::toMatrix3d(RRwc),Converter::toVector3d(OOw),1.0);
    // mg2oScw = gScm_new*gSmw;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    // std::cout << "current keyframe Twc: " << Twc << std::endl;
    {

        // // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));
                // Eigen::Vector3f point_tmp; 
                // for (int row = 0; row < 3; row++)
                //         point_tmp(row) = P3Dw.at<float>(row);
                // Eigen::Vector3f trans_new = corr_scale_init*rot_21_init*point_tmp+trans_21_init;
                // Eigen::Matrix<double,3,1> eigCorrectedP3Dw = trans_new.cast<double>();

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();

                if(iMP==0)
                {
                    std::cout << "point pose origin: " << P3Dw.t() << std::endl;
                    std::cout << "point pose after: " << cvCorrectedP3Dw.t() << std::endl;
                    Eigen::Vector3f point_tmp; 
                    for (int row = 0; row < 3; row++)
                            point_tmp(row) = P3Dw.at<float>(row);
                    Eigen::Vector3f trans_new = corr_scale_init*rot_21_init*point_tmp+trans_21_init;
                    std::cout << "point pose after: " << trans_new.transpose() << std::endl;
                }
            
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            std::cout << "mpCurrentKF before: " << pKFi->GetPoseInverse().rowRange(0, 3).col(3).t()<< std::endl;
            pKFi->SetPose(correctedTiw);
            cv::Mat Twc_cam = pKFi->GetPoseInverse();
            Eigen::Matrix4f Twc_cam_mat; 
            for (int row = 0; row < 4; row++)
                for (int col = 0; col < 4; col++)
                    Twc_cam_mat(row, col) = Twc_cam.at<float>(row, col);
            Eigen::Vector3f trans_new = corr_scale_init*rot_21_init*Twc_cam_mat.col(3).head(3)+trans_21_init;
            Eigen::Matrix3f rot_new = rot_21_init * Twc_cam_mat.block(0,0,3,3);
            cv::Mat Twc_cam_final = cv::Mat::eye(4, 4, CV_32F);
            for (int row = 0; row < 3; row++)
                for (int col = 0; col < 3; col++)
                    Twc_cam_final.at<float>(row, col) = float(rot_new(row, col));
            Twc_cam_final.at<float>(0, 3) = trans_new(0);
            Twc_cam_final.at<float>(1, 3) = trans_new(1);
            Twc_cam_final.at<float>(2, 3) = trans_new(2);

            cv::Mat Tcw_final = cv::Mat::eye(4,4,Twc_cam_final.type());
            cv::Mat Rcw = Twc_cam_final.rowRange(0,3).colRange(0,3);
            cv::Mat tcw = Twc_cam_final.rowRange(0,3).col(3);
            cv::Mat Rwc = Rcw.t();
            cv::Mat Ow = -Rwc*tcw;
            Rwc.copyTo(Tcw_final.rowRange(0,3).colRange(0,3));
            Ow.copyTo(Tcw_final.rowRange(0,3).col(3));

            // pKFi->SetPose(Tcw_final);
            std::cout << "Twc_cam_final: " << Twc_cam_final.rowRange(0, 3).col(3).t() << std::endl;
            std::cout << "mpCurrentKF after: " << pKFi->GetPoseInverse().rowRange(0, 3).col(3).t()<< std::endl;
            
            // Make sure connections are updated
            pKFi->UpdateConnections();
        }


        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);

    std::cout << "current keyframe Twc fuse: " << mpCurrentKF->GetPoseInverse().rowRange(0, 3).col(3).t() << std::endl;
    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();
        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame*> &spConnections = mit->second;
        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            std::cout << " connection keyframe " << pKF->mnId << " to " << nIDj << std::endl;
        }
    }

    // Optimize graph
    std::cout << "current keyframe Twc loop: " << mpCurrentKF->mnId << " " << mpCurrentKF->GetPoseInverse().rowRange(0, 3).col(3).t() << std::endl;
    // mpMatchedKF = mpCurrentKF;
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);
    std::cout << "current keyframe Twc loop: " << mpCurrentKF->mnId << mpCurrentKF->GetPoseInverse().rowRange(0, 3).col(3).t() << std::endl;

    // mpMap->InformNewBigChange();

    // // // Add loop edge
    // mpMatchedKF->AddLoopEdge(mpCurrentKF);
    // mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    std::cout << "current keyframe Twc final: " << mpCurrentKF->GetPoseInverse().rowRange(0,3).col(3).t() << std::endl;
    mLastLoopKFid = mpCurrentKF->mnId;   
}

} //namespace ORB_SLAM
