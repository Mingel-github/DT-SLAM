#include <opencv2/imgproc/types_c.h>
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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):
    N(0),mbOnlyTracking(false),mnTracked(0),
    mnORBFeatures(0),mnSemanticFeatures(0),mnGeometryRemoved(0),
    mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys;
    vector<int> vMatches;
    vector<cv::KeyPoint> vCurrentKeys;
    vector<bool> vbMap;
    vector<unsigned char> vbSemanticDynamic;
    vector<unsigned char> vbGeometryTrackingRemoved;
    cv::Mat mask;
    int state;

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);
        if(!mImMask.empty())
            mImMask.copyTo(mask);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbMap = mvbMap;
            vbSemanticDynamic = mvbSemanticDynamic;
            vbGeometryTrackingRemoved =
                mvbGeometryTrackingRemoved;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    if(im.channels()<3)
        cvtColor(im,im,CV_GRAY2BGR);

    // DT-SLAM: 动态mask红色半透明叠加（透明度85%/15%，隐约可见不遮挡画面）
    if(!mask.empty())
    {
        cv::Mat overlay = im.clone();
        overlay.setTo(cv::Scalar(0,0,255), mask);
        cv::addWeighted(im, 0.85, overlay, 0.15, 0, im);
    }

    //Draw
    if(state==Tracking::NOT_INITIALIZED)
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK)
    {
        mnTracked=0;
        mnORBFeatures=static_cast<int>(vCurrentKeys.size());
        mnSemanticFeatures=0;
        mnGeometryRemoved=0;
        const float r = 5;
        const int n = vCurrentKeys.size();

        // Keep the normal view deliberately simple: green means that the
        // association survived tracking and is backed by an established
        // MapPoint. VO-only and intermediate geometry states are hidden.
        for(int i=0;i<n;i++)
        {
            if(i<static_cast<int>(vbMap.size()) && vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                const cv::Scalar green(0,255,0);
                cv::rectangle(im,pt1,pt2,green);
                cv::circle(im,vCurrentKeys[i].pt,2,green,-1);
                mnTracked++;
            }
        }

        const auto drawFilteredPoint =
            [&im](const cv::Point2f &point,const cv::Scalar &color)
        {
            cv::circle(im,point,3,color,-1);
        };

        // Red points show every extracted ORB feature covered by the
        // semantic mask, including features whose associations were removed
        // before FrameDrawer::Update().
        for(int i=0;i<n;i++)
        {
            if(i<static_cast<int>(vbSemanticDynamic.size()) &&
               vbSemanticDynamic[i]!=0)
            {
                drawFilteredPoint(
                    vCurrentKeys[i].pt,cv::Scalar(0,0,255));
                mnSemanticFeatures++;
            }
        }

        // Yellow points show only associations that G1-F1 actually
        // removed. Raw residual candidates and mapping-only diagnostics stay
        // out of the normal viewer to avoid visual clutter.
        for(int i=0;i<n;i++)
        {
            if(i<static_cast<int>(vbGeometryTrackingRemoved.size()) &&
               vbGeometryTrackingRemoved[i]!=0)
            {
                drawFilteredPoint(
                    vCurrentKeys[i].pt,cv::Scalar(0,255,255));
                mnGeometryRemoved++;
            }
        }
    }

    // DT-SLAM: 画YOLO检测框+置信度
    std::vector<Detection> dets;
    {
        unique_lock<mutex> lock(mMutex);
        dets = mDetections;
    }
    for (const auto& d : dets)
    {
        cv::rectangle(im, d.box, cv::Scalar(0,0,255), 2);
        std::string label = cv::format("person %.2f", d.confidence);
        cv::putText(im, label, cv::Point(d.box.x, d.box.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1);
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs
          << ", ORB: " << mnORBFeatures
          << ", Matches: " << mnTracked
          << ", Sem: " << mnSemanticFeatures
          << ", Geo removed: " << mnGeometryRemoved;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::UpdateMask(const cv::Mat &mask)
{
    unique_lock<mutex> lock(mMutex);
    if(!mask.empty())
        mask.copyTo(mImMask);
    else
        mImMask.release();
}

void FrameDrawer::UpdateDetections(const std::vector<Detection> &detections)
{
    unique_lock<mutex> lock(mMutex);
    mDetections = detections;
}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbMap = vector<bool>(N,false);
    mvbSemanticDynamic =
        pTracker->mCurrentFrame.mvbSemanticDynamic;
    if(mvbSemanticDynamic.size()!=static_cast<std::size_t>(N))
        mvbSemanticDynamic.assign(N,0);
    mvbGeometryTrackingRemoved =
        pTracker->GetCurrentSparseFlowRemovedAssociations();
    if(mvbGeometryTrackingRemoved.size()!=static_cast<std::size_t>(N))
        mvbGeometryTrackingRemoved.assign(N,0);
    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
