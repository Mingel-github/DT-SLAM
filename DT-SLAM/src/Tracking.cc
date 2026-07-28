#include <opencv2/imgproc/types_c.h>
#include <unistd.h>
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
#include<opencv2/imgcodecs.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"

#include <algorithm>
#include <stdexcept>
#include"PnPsolver.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include<iostream>
#include <sstream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

namespace
{

void LoadGeometryCameraMatrix(const cv::FileStorage &settings,
                              const cv::Mat &trackingK,
                              cv::Mat &geometryK,
                              bool &usesDedicatedCameraModel)
{
    const cv::FileNode fxNode = settings["Geometry.Camera.fx"];
    const cv::FileNode fyNode = settings["Geometry.Camera.fy"];
    const cv::FileNode cxNode = settings["Geometry.Camera.cx"];
    const cv::FileNode cyNode = settings["Geometry.Camera.cy"];
    const int configuredValues =
        static_cast<int>(!fxNode.empty())+
        static_cast<int>(!fyNode.empty())+
        static_cast<int>(!cxNode.empty())+
        static_cast<int>(!cyNode.empty());

    if(configuredValues==0)
    {
        trackingK.copyTo(geometryK);
        usesDedicatedCameraModel = false;
        return;
    }
    if(configuredValues!=4)
    {
        throw std::invalid_argument(
            "Geometry.Camera.fx/fy/cx/cy must either all be provided or all be omitted");
    }

    const float fx = static_cast<float>(fxNode);
    const float fy = static_cast<float>(fyNode);
    const float cx = static_cast<float>(cxNode);
    const float cy = static_cast<float>(cyNode);
    if(!std::isfinite(fx) || !std::isfinite(fy) ||
       !std::isfinite(cx) || !std::isfinite(cy) ||
       fx<=0.0f || fy<=0.0f)
    {
        throw std::invalid_argument(
            "Geometry.Camera intrinsics must be finite with positive focal lengths");
    }

    geometryK = cv::Mat::eye(3,3,CV_32F);
    geometryK.at<float>(0,0) = fx;
    geometryK.at<float>(1,1) = fy;
    geometryK.at<float>(0,2) = cx;
    geometryK.at<float>(1,2) = cy;
    usesDedicatedCameraModel = true;
}

GeometrySemanticProxyStats ComputeSemanticProxyStats(
    const GeometricWarpResult &result,
    const cv::Mat &semanticMask)
{
    GeometrySemanticProxyStats stats;
    stats.semanticPixels =
        static_cast<std::size_t>(cv::countNonZero(semanticMask));
    stats.validComparisonPixels = result.stats.validComparisons;
    stats.positiveSeedPixels = result.stats.positiveSeedPixels;

    cv::Mat semanticValid;
    cv::bitwise_and(
        semanticMask,result.validComparisonMask,semanticValid);
    stats.semanticValidPixels =
        static_cast<std::size_t>(cv::countNonZero(semanticValid));

    cv::Mat positiveInsideSemantic;
    cv::bitwise_and(
        semanticMask,result.positiveSeedMask,positiveInsideSemantic);
    stats.positiveInsideSemanticPixels =
        static_cast<std::size_t>(cv::countNonZero(positiveInsideSemantic));
    stats.positiveOutsideSemanticPixels =
        stats.positiveSeedPixels-stats.positiveInsideSemanticPixels;

    if(stats.semanticPixels>0)
    {
        stats.semanticValidCoverage =
            static_cast<double>(stats.semanticValidPixels)/
            static_cast<double>(stats.semanticPixels);
    }
    if(stats.positiveSeedPixels>0)
    {
        stats.proxyPrecision =
            static_cast<double>(stats.positiveInsideSemanticPixels)/
            static_cast<double>(stats.positiveSeedPixels);
    }
    if(stats.semanticValidPixels>0)
    {
        stats.conditionalRecall =
            static_cast<double>(stats.positiveInsideSemanticPixels)/
            static_cast<double>(stats.semanticValidPixels);
    }

    const std::size_t staticValidPixels =
        stats.validComparisonPixels-stats.semanticValidPixels;
    if(staticValidPixels>0)
    {
        stats.staticBackgroundFpr =
            static_cast<double>(stats.positiveOutsideSemanticPixels)/
            static_cast<double>(staticValidPixels);
    }
    return stats;
}

std::vector<GeometryFeatureShadowStats> ComputeFeatureShadowStats(
    const GeometricWarpResult &result,
    const Frame &frame,
    const cv::Mat &semanticMask)
{
    static const int radii[] = {0,1,2,3};
    std::vector<GeometryFeatureShadowStats> results;
    results.reserve(sizeof(radii)/sizeof(radii[0]));

    for(std::size_t radiusIndex=0;
        radiusIndex<sizeof(radii)/sizeof(radii[0]); ++radiusIndex)
    {
        const int radius = radii[radiusIndex];
        cv::Mat eligibleMask;
        cv::Mat candidateMask;
        if(radius==0)
        {
            eligibleMask = result.validComparisonMask;
            candidateMask = result.positiveSeedMask;
        }
        else
        {
            const cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_RECT,cv::Size(2*radius+1,2*radius+1));
            cv::dilate(
                result.validComparisonMask,eligibleMask,kernel);
            cv::dilate(
                result.positiveSeedMask,candidateMask,kernel);
        }

        GeometryFeatureShadowStats stats;
        stats.radiusPixels = radius;
        stats.featureCount = frame.mvKeys.size();
        for(std::size_t index=0; index<frame.mvKeys.size(); ++index)
        {
            const int u = static_cast<int>(frame.mvKeys[index].pt.x);
            const int v = static_cast<int>(frame.mvKeys[index].pt.y);
            if(u<0 || u>=eligibleMask.cols || v<0 || v>=eligibleMask.rows)
                continue;

            const bool semantic =
                !semanticMask.empty() && semanticMask.at<uchar>(v,u)!=0;
            const bool eligible = eligibleMask.at<uchar>(v,u)!=0;
            const bool candidate = candidateMask.at<uchar>(v,u)!=0;
            if(semantic)
                ++stats.semanticFeatureCount;
            if(eligible)
            {
                ++stats.eligibleFeatureCount;
                if(semantic)
                    ++stats.semanticEligibleFeatureCount;
            }
            if(candidate)
            {
                ++stats.candidateFeatureCount;
                if(semantic)
                    ++stats.candidateInsideSemanticFeatureCount;
                else
                    ++stats.candidateOutsideSemanticFeatureCount;
            }
        }

        if(stats.featureCount>0)
        {
            stats.eligibleCoverage =
                static_cast<double>(stats.eligibleFeatureCount)/
                static_cast<double>(stats.featureCount);
        }
        if(stats.semanticFeatureCount>0)
        {
            stats.semanticEligibleCoverage =
                static_cast<double>(stats.semanticEligibleFeatureCount)/
                static_cast<double>(stats.semanticFeatureCount);
        }
        if(stats.candidateFeatureCount>0)
        {
            stats.proxyPrecision =
                static_cast<double>(
                    stats.candidateInsideSemanticFeatureCount)/
                static_cast<double>(stats.candidateFeatureCount);
        }
        if(stats.semanticEligibleFeatureCount>0)
        {
            stats.conditionalRecall =
                static_cast<double>(
                    stats.candidateInsideSemanticFeatureCount)/
                static_cast<double>(stats.semanticEligibleFeatureCount);
        }
        const std::size_t proxyBackgroundEligible =
            stats.eligibleFeatureCount-stats.semanticEligibleFeatureCount;
        if(proxyBackgroundEligible>0)
        {
            stats.proxyBackgroundRate =
                static_cast<double>(
                    stats.candidateOutsideSemanticFeatureCount)/
                static_cast<double>(proxyBackgroundEligible);
        }
        results.push_back(stats);
    }

    return results;
}

} // namespace

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap),
    mbGeometryShadowEnabled(false), mbGeometryDebugSaveEnabled(false),
    mbGeometryUsesDedicatedCameraModel(false),
    mnGeometryLogEveryN(30), mnGeometryDebugEveryN(30),
    mnGeometryComputedFrames(0),
    mbJiGeometryShadowEnabled(false),
    mbJiGeometryReprojectionStatsEnabled(false),
    mbJiGeometryDebugSaveEnabled(false),
    mnJiGeometryLogEveryN(30), mnJiGeometryDebugEveryN(30),
    mnJiGeometryComputedFrames(0), mnLastRelocFrameId(0)
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

    const cv::FileNode geometryEnableNode = fSettings["Geometry.Enable"];
    if(!geometryEnableNode.empty())
        mbGeometryShadowEnabled = static_cast<int>(geometryEnableNode)!=0;

    const cv::FileNode jiGeometryEnableNode =
        fSettings["JiGeometry.Enable"];
    if(!jiGeometryEnableNode.empty())
    {
        mbJiGeometryShadowEnabled =
            static_cast<int>(jiGeometryEnableNode)!=0;
    }
    const cv::FileNode jiReprojectionStatsNode =
        fSettings["JiGeometry.EnableReprojectionStats"];
    if(!jiReprojectionStatsNode.empty())
    {
        mbJiGeometryReprojectionStatsEnabled =
            static_cast<int>(jiReprojectionStatsNode)!=0;
    }
    if(mbJiGeometryReprojectionStatsEnabled &&
       !mbJiGeometryShadowEnabled)
    {
        throw std::invalid_argument(
            "JiGeometry.EnableReprojectionStats=1 requires "
            "JiGeometry.Enable=1");
    }

    LoadGeometryCameraMatrix(
        fSettings,mK,mGeometryK,mbGeometryUsesDedicatedCameraModel);
    mGeometricDetector.SetCameraMatrix(mGeometryK);
    mGeometricGroundTruthDetector.SetCameraMatrix(mGeometryK);
    mJiGeometryBaseline.SetCameraMatrix(mGeometryK);

    if((mbGeometryShadowEnabled || mbJiGeometryShadowEnabled) &&
       !mbGeometryUsesDedicatedCameraModel &&
       cv::norm(mDistCoef,cv::NORM_INF)>1e-8)
    {
        throw std::invalid_argument(
            "Geometry.Enable=1 or JiGeometry.Enable=1 requires zero distortion "
            "in the current raw-pixel pinhole implementation. Provide the "
            "dataset-authorized raw registered Geometry.Camera.fx/fy/cx/cy "
            "model, or rectify RGB/depth/masks together.");
    }

    const cv::FileNode geometryLogEveryNode = fSettings["Geometry.LogEveryN"];
    if(!geometryLogEveryNode.empty())
        mnGeometryLogEveryN = std::max(1,static_cast<int>(geometryLogEveryNode));

    const cv::FileNode geometryResidualThresholdNode =
        fSettings["Geometry.ResidualThresholdM"];
    if(!geometryResidualThresholdNode.empty())
    {
        mGeometricDetector.SetResidualThresholdMeters(
            static_cast<float>(geometryResidualThresholdNode));
        mGeometricGroundTruthDetector.SetResidualThresholdMeters(
            static_cast<float>(geometryResidualThresholdNode));
    }

    const cv::FileNode geometryRegionGrowNode =
        fSettings["Geometry.RegionGrowEnable"];
    if(!geometryRegionGrowNode.empty())
    {
        mGeometricDetector.SetRegionGrowEnabled(
            static_cast<int>(geometryRegionGrowNode)!=0);
        mGeometricGroundTruthDetector.SetRegionGrowEnabled(
            static_cast<int>(geometryRegionGrowNode)!=0);
    }

    const cv::FileNode geometryRegionDepthThresholdNode =
        fSettings["Geometry.RegionDepthThresholdM"];
    if(!geometryRegionDepthThresholdNode.empty())
    {
        mGeometricDetector.SetRegionDepthThresholdMeters(
            static_cast<float>(geometryRegionDepthThresholdNode));
        mGeometricGroundTruthDetector.SetRegionDepthThresholdMeters(
            static_cast<float>(geometryRegionDepthThresholdNode));
    }

    const cv::FileNode geometryDebugSaveNode = fSettings["Geometry.DebugSave"];
    if(!geometryDebugSaveNode.empty())
        mbGeometryDebugSaveEnabled = static_cast<int>(geometryDebugSaveNode)!=0;

    const cv::FileNode geometryDebugEveryNode = fSettings["Geometry.DebugEveryN"];
    if(!geometryDebugEveryNode.empty())
        mnGeometryDebugEveryN = std::max(1,static_cast<int>(geometryDebugEveryNode));

    if(mbGeometryDebugSaveEnabled)
    {
        const char *debugOutputDir = std::getenv("DT_SLAM_GEOMETRY_DEBUG_DIR");
        if(debugOutputDir && debugOutputDir[0]!='\0')
        {
            mGeometryDebugOutputDir = debugOutputDir;
        }
        else
        {
            cerr << "[Geometry G0-2V] Geometry.DebugSave requested, but "
                 << "DT_SLAM_GEOMETRY_DEBUG_DIR is not set; debug saving disabled"
                 << endl;
            mbGeometryDebugSaveEnabled = false;
        }
    }

    const char *poseDiagnosticCsv =
        std::getenv("DT_SLAM_GT_DIAGNOSTIC_CSV");
    if(poseDiagnosticCsv && poseDiagnosticCsv[0]!='\0')
        mGeometryPoseDiagnosticCsvPath = poseDiagnosticCsv;

    const char *semanticProxyCsv =
        std::getenv("DT_SLAM_GEOMETRY_PROXY_CSV");
    if(semanticProxyCsv && semanticProxyCsv[0]!='\0')
        mGeometrySemanticProxyCsvPath = semanticProxyCsv;

    const char *featureShadowCsv =
        std::getenv("DT_SLAM_GEOMETRY_FEATURE_CSV");
    if(featureShadowCsv && featureShadowCsv[0]!='\0')
        mGeometryFeatureShadowCsvPath = featureShadowCsv;

    const cv::FileNode jiClustersNode =
        fSettings["JiGeometry.Clusters"];
    if(!jiClustersNode.empty())
    {
        mJiGeometryBaseline.SetClusterCount(
            static_cast<int>(jiClustersNode));
    }

    int jiKMeansMaxIterations = mJiGeometryBaseline.MaxIterations();
    double jiKMeansEpsilon = mJiGeometryBaseline.Epsilon();
    const cv::FileNode jiMaxIterationsNode =
        fSettings["JiGeometry.KMeansMaxIterations"];
    if(!jiMaxIterationsNode.empty())
    {
        jiKMeansMaxIterations =
            static_cast<int>(jiMaxIterationsNode);
    }
    const cv::FileNode jiEpsilonNode =
        fSettings["JiGeometry.KMeansEpsilon"];
    if(!jiEpsilonNode.empty())
        jiKMeansEpsilon = static_cast<double>(jiEpsilonNode);
    mJiGeometryBaseline.SetKMeansCriteria(
        jiKMeansMaxIterations,jiKMeansEpsilon);

    const cv::FileNode jiAttemptsNode =
        fSettings["JiGeometry.KMeansAttempts"];
    if(!jiAttemptsNode.empty())
    {
        mJiGeometryBaseline.SetKMeansAttempts(
            static_cast<int>(jiAttemptsNode));
    }

    const cv::FileNode jiRandomSeedNode =
        fSettings["JiGeometry.RandomSeed"];
    if(!jiRandomSeedNode.empty())
    {
        const int randomSeed = static_cast<int>(jiRandomSeedNode);
        if(randomSeed<0)
        {
            throw std::invalid_argument(
                "JiGeometry.RandomSeed must be non-negative");
        }
        mJiGeometryBaseline.SetRandomSeed(
            static_cast<std::uint64_t>(randomSeed));
    }

    const cv::FileNode jiLogEveryNode =
        fSettings["JiGeometry.LogEveryN"];
    if(!jiLogEveryNode.empty())
    {
        mnJiGeometryLogEveryN =
            std::max(1,static_cast<int>(jiLogEveryNode));
    }

    const cv::FileNode jiDebugSaveNode =
        fSettings["JiGeometry.DebugSave"];
    if(!jiDebugSaveNode.empty())
    {
        mbJiGeometryDebugSaveEnabled =
            static_cast<int>(jiDebugSaveNode)!=0;
    }

    const cv::FileNode jiDebugEveryNode =
        fSettings["JiGeometry.DebugEveryN"];
    if(!jiDebugEveryNode.empty())
    {
        mnJiGeometryDebugEveryN =
            std::max(1,static_cast<int>(jiDebugEveryNode));
    }

    if(mbJiGeometryDebugSaveEnabled)
    {
        const char *debugOutputDir =
            std::getenv("DT_SLAM_JI_DEBUG_DIR");
        if(debugOutputDir && debugOutputDir[0]!='\0')
        {
            mJiGeometryDebugOutputDir = debugOutputDir;
        }
        else
        {
            cerr << "[Ji GJ-1] JiGeometry.DebugSave requested, but "
                 << "DT_SLAM_JI_DEBUG_DIR is not set; debug saving disabled"
                 << endl;
            mbJiGeometryDebugSaveEnabled = false;
        }
    }

    const char *jiClusterCsv =
        std::getenv("DT_SLAM_JI_CLUSTER_CSV");
    if(jiClusterCsv && jiClusterCsv[0]!='\0')
        mJiGeometryClusterCsvPath = jiClusterCsv;

    const char *jiReprojectionCsv =
        std::getenv("DT_SLAM_JI_REPROJECTION_CSV");
    if(jiReprojectionCsv && jiReprojectionCsv[0]!='\0')
        mJiGeometryReprojectionCsvPath = jiReprojectionCsv;

    cout << endl << "Geometry Shadow: "
         << (mbGeometryShadowEnabled ? "enabled" : "disabled") << endl;
    if(mbGeometryShadowEnabled)
    {
        cout << "- pixel domain: raw registered RGB/depth pixels" << endl;
        cout << "- camera model: "
             << (mbGeometryUsesDedicatedCameraModel ?
                 "dedicated raw registered pinhole K" :
                 "tracking pinhole K")
             << " with zero distortion" << endl;
        cout << "- geometry fx: " << mGeometryK.at<float>(0,0) << endl;
        cout << "- geometry fy: " << mGeometryK.at<float>(1,1) << endl;
        cout << "- geometry cx: " << mGeometryK.at<float>(0,2) << endl;
        cout << "- geometry cy: " << mGeometryK.at<float>(1,2) << endl;
        cout << "- semantic/feature labeling domain: Frame::mvKeys" << endl;
        cout << "- optimizer feature domain: Frame::mvKeysUn" << endl;
        cout << "- log every: " << mnGeometryLogEveryN << " computed frames" << endl;
        cout << "- provisional residual threshold: "
             << mGeometricDetector.ResidualThresholdMeters() << " m" << endl;
        cout << "- region candidate generation: "
             << (mGeometricDetector.RegionGrowEnabled() ?
                 "enabled" : "disabled") << endl;
        if(mGeometricDetector.RegionGrowEnabled())
        {
            cout << "- provisional local depth threshold: "
                 << mGeometricDetector.RegionDepthThresholdMeters()
                 << " m" << endl;
        }
        cout << "- debug image saving: "
             << (mbGeometryDebugSaveEnabled ? "enabled" : "disabled") << endl;
        if(mbGeometryDebugSaveEnabled)
        {
            cout << "- debug every: " << mnGeometryDebugEveryN
                 << " computed frames" << endl;
            cout << "- debug output directory: " << mGeometryDebugOutputDir << endl;
        }
        if(!mGeometrySemanticProxyCsvPath.empty())
        {
            cout << "- semantic proxy diagnostics: "
                 << mGeometrySemanticProxyCsvPath << endl;
        }
        if(!mGeometryFeatureShadowCsvPath.empty())
        {
            cout << "- feature shadow diagnostics (r=0,1,2,3 px): "
                 << mGeometryFeatureShadowCsvPath << endl;
        }
    }

    cout << endl << "Ji Geometry GJ-1 Shadow: "
         << (mbJiGeometryShadowEnabled ? "enabled" : "disabled") << endl;
    if(mbJiGeometryShadowEnabled)
    {
        cout << "- scope: 3D depth K-means labels only; no dynamic decision"
             << endl;
        cout << "- paper parameter: clusters="
             << mJiGeometryBaseline.ClusterCount() << endl;
        cout << "- engineering choices: KMEANS_PP_CENTERS, attempts="
             << mJiGeometryBaseline.Attempts()
             << ", max_iterations=" << mJiGeometryBaseline.MaxIterations()
             << ", epsilon=" << mJiGeometryBaseline.Epsilon()
             << ", random_seed=" << mJiGeometryBaseline.RandomSeed()
             << endl;
        cout << "- pixel domain: raw registered depth pixels" << endl;
        cout << "- output labels: CV_16SC1 (-1 invalid, 0..K-1 cluster)"
             << endl;
        cout << "- SLAM state mutation: none; synchronous runtime remains "
             << "part of end-to-end latency" << endl;
        cout << "- initial-pose cluster reprojection statistics: "
             << (mbJiGeometryReprojectionStatsEnabled ?
                 "enabled" : "disabled") << endl;
        cout << "- log every: " << mnJiGeometryLogEveryN
             << " computed frames" << endl;
        cout << "- debug image saving: "
             << (mbJiGeometryDebugSaveEnabled ? "enabled" : "disabled")
             << endl;
        if(mbJiGeometryDebugSaveEnabled)
        {
            cout << "- debug every: " << mnJiGeometryDebugEveryN
                 << " computed frames" << endl;
            cout << "- debug output directory: "
                 << mJiGeometryDebugOutputDir << endl;
        }
        if(!mJiGeometryClusterCsvPath.empty())
        {
            cout << "- cluster diagnostics: "
                 << mJiGeometryClusterCsvPath << endl;
        }
        if(mbJiGeometryReprojectionStatsEnabled &&
           !mJiGeometryReprojectionCsvPath.empty())
        {
            cout << "- reprojection diagnostics: "
                 << mJiGeometryReprojectionCsvPath << endl;
        }
    }

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

void Tracking::SetGroundTruthPoseForGeometry(const cv::Mat &TcwGroundTruth)
{
    if(TcwGroundTruth.empty())
    {
        mCurrentGroundTruthTcw.release();
        return;
    }
    if(TcwGroundTruth.rows!=4 || TcwGroundTruth.cols!=4 ||
       TcwGroundTruth.channels()!=1 || !cv::checkRange(TcwGroundTruth))
    {
        throw std::invalid_argument(
            "geometry ground-truth Tcw must be an empty matrix or a finite 4x4 matrix");
    }
    TcwGroundTruth.convertTo(mCurrentGroundTruthTcw,CV_32F);
}

void Tracking::SaveGeometryPoseDiagnostics()
{
    if(!mGeometryPoseDiagnosticCsvPath.empty() &&
       !mvGeometryPoseDiagnostics.empty())
    {
        ofstream stream(mGeometryPoseDiagnosticCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G0-2P] failed to open diagnostic CSV: "
                 << mGeometryPoseDiagnosticCsvPath << endl;
        }
        else
        {
            stream << "frame,reference,current_timestamp,reference_timestamp,dt_s,"
                   << "slam_comparisons,gt_comparisons,"
                   << "slam_compare_coverage,gt_compare_coverage,"
                   << "slam_mean_abs_m,gt_mean_abs_m,"
                   << "slam_positive_ratio,gt_positive_ratio,"
                   << "slam_negative_ratio,gt_negative_ratio,"
                   << "slam_total_ms,gt_total_ms\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometryPoseDiagnostics.size(); ++index)
            {
                const GeometryPoseDiagnosticRecord &record =
                    mvGeometryPoseDiagnostics[index];
                stream << record.frameId << ","
                       << record.referenceFrameId << ","
                       << record.timestamp << ","
                       << record.referenceTimestamp << ","
                       << record.timestamp-record.referenceTimestamp << ","
                       << record.slam.validComparisons << ","
                       << record.groundTruth.validComparisons << ","
                       << record.slam.comparisonCoverageRatio << ","
                       << record.groundTruth.comparisonCoverageRatio << ","
                       << record.slam.residualMeanAbs << ","
                       << record.groundTruth.residualMeanAbs << ","
                       << record.slam.positiveSeedRatio << ","
                       << record.groundTruth.positiveSeedRatio << ","
                       << record.slam.negativeDiagnosticRatio << ","
                       << record.groundTruth.negativeDiagnosticRatio << ","
                       << record.slam.totalMs << ","
                       << record.groundTruth.totalMs << "\n";
            }
            stream.close();
            cout << "[Geometry G0-2P] saved "
                 << mvGeometryPoseDiagnostics.size()
                 << " paired diagnostics to "
                 << mGeometryPoseDiagnosticCsvPath << endl;
        }
    }

    if(!mGeometrySemanticProxyCsvPath.empty() &&
       !mvGeometrySemanticProxyDiagnostics.empty())
    {
        ofstream stream(mGeometrySemanticProxyCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G0-2A] failed to open semantic proxy CSV: "
                 << mGeometrySemanticProxyCsvPath << endl;
        }
        else
        {
            stream << "frame,reference,timestamp,has_gt,semantic_pixels,"
                   << "slam_valid,slam_semantic_valid,slam_positive,"
                   << "slam_positive_inside,slam_positive_outside,"
                   << "slam_semantic_coverage,slam_proxy_precision,"
                   << "slam_conditional_recall,slam_static_fpr,"
                   << "gt_valid,gt_semantic_valid,gt_positive,"
                   << "gt_positive_inside,gt_positive_outside,"
                   << "gt_semantic_coverage,gt_proxy_precision,"
                   << "gt_conditional_recall,gt_static_fpr\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometrySemanticProxyDiagnostics.size(); ++index)
            {
                const GeometrySemanticProxyRecord &record =
                    mvGeometrySemanticProxyDiagnostics[index];
                stream << record.frameId << ","
                       << record.referenceFrameId << ","
                       << record.timestamp << ","
                       << static_cast<int>(record.hasGroundTruth) << ","
                       << record.slam.semanticPixels << ","
                       << record.slam.validComparisonPixels << ","
                       << record.slam.semanticValidPixels << ","
                       << record.slam.positiveSeedPixels << ","
                       << record.slam.positiveInsideSemanticPixels << ","
                       << record.slam.positiveOutsideSemanticPixels << ","
                       << record.slam.semanticValidCoverage << ","
                       << record.slam.proxyPrecision << ","
                       << record.slam.conditionalRecall << ","
                       << record.slam.staticBackgroundFpr << ","
                       << record.groundTruth.validComparisonPixels << ","
                       << record.groundTruth.semanticValidPixels << ","
                       << record.groundTruth.positiveSeedPixels << ","
                       << record.groundTruth.positiveInsideSemanticPixels << ","
                       << record.groundTruth.positiveOutsideSemanticPixels << ","
                       << record.groundTruth.semanticValidCoverage << ","
                       << record.groundTruth.proxyPrecision << ","
                       << record.groundTruth.conditionalRecall << ","
                       << record.groundTruth.staticBackgroundFpr << "\n";
            }
            stream.close();
            cout << "[Geometry G0-2A] saved "
                 << mvGeometrySemanticProxyDiagnostics.size()
                 << " semantic proxy diagnostics to "
                 << mGeometrySemanticProxyCsvPath << endl;
        }
    }

    if(!mGeometryFeatureShadowCsvPath.empty() &&
       !mvGeometryFeatureShadowDiagnostics.empty())
    {
        ofstream stream(mGeometryFeatureShadowCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G0-4F] failed to open feature shadow CSV: "
                 << mGeometryFeatureShadowCsvPath << endl;
        }
        else
        {
            stream << "frame,reference,timestamp,pose_source,radius_px,"
                   << "features,semantic_features,eligible,"
                   << "semantic_eligible,candidates,"
                   << "candidates_inside_semantic,"
                   << "candidates_outside_semantic,"
                   << "eligible_coverage,semantic_eligible_coverage,"
                   << "proxy_precision,conditional_recall,"
                   << "proxy_background_rate\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometryFeatureShadowDiagnostics.size(); ++index)
            {
                const GeometryFeatureShadowRecord &record =
                    mvGeometryFeatureShadowDiagnostics[index];
                const GeometryFeatureShadowStats &stats = record.stats;
                stream << record.frameId << ","
                       << record.referenceFrameId << ","
                       << record.timestamp << ","
                       << (record.groundTruthPose ? "gt" : "slam") << ","
                       << stats.radiusPixels << ","
                       << stats.featureCount << ","
                       << stats.semanticFeatureCount << ","
                       << stats.eligibleFeatureCount << ","
                       << stats.semanticEligibleFeatureCount << ","
                       << stats.candidateFeatureCount << ","
                       << stats.candidateInsideSemanticFeatureCount << ","
                       << stats.candidateOutsideSemanticFeatureCount << ","
                       << stats.eligibleCoverage << ","
                       << stats.semanticEligibleCoverage << ","
                       << stats.proxyPrecision << ","
                       << stats.conditionalRecall << ","
                       << stats.proxyBackgroundRate << "\n";
            }
            stream.close();
            cout << "[Geometry G0-4F] saved "
                 << mvGeometryFeatureShadowDiagnostics.size()
                 << " feature shadow rows to "
                 << mGeometryFeatureShadowCsvPath << endl;
        }
    }

    if(!mJiGeometryClusterCsvPath.empty() &&
       !mvJiGeometryClusterDiagnostics.empty())
    {
        ofstream stream(mJiGeometryClusterCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Ji GJ-1] failed to open cluster CSV: "
                 << mJiGeometryClusterCsvPath << endl;
        }
        else
        {
            stream << "frame,timestamp,cluster_id,valid_depth,"
                   << "requested_clusters,produced_clusters,"
                   << "cluster_pixels,centroid_x_m,centroid_y_m,"
                   << "centroid_z_m,smallest_cluster_pixels,"
                   << "largest_cluster_pixels,compactness,"
                   << "prepare_ms,kmeans_ms,label_ms,total_ms\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvJiGeometryClusterDiagnostics.size(); ++index)
            {
                const JiGeometryClusterRecord &record =
                    mvJiGeometryClusterDiagnostics[index];
                const JiDepthClusteringStats &stats =
                    record.frameStats;
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.cluster.id << ","
                       << stats.validDepthPixels << ","
                       << stats.requestedClusters << ","
                       << stats.producedClusters << ","
                       << record.cluster.pixels << ","
                       << record.cluster.centroid[0] << ","
                       << record.cluster.centroid[1] << ","
                       << record.cluster.centroid[2] << ","
                       << stats.smallestClusterPixels << ","
                       << stats.largestClusterPixels << ","
                       << stats.compactness << ","
                       << stats.prepareMs << ","
                       << stats.kmeansMs << ","
                       << stats.labelMs << ","
                       << stats.totalMs << "\n";
            }
            stream.close();
            cout << "[Ji GJ-1] saved "
                 << mvJiGeometryClusterDiagnostics.size()
                 << " cluster rows to "
                 << mJiGeometryClusterCsvPath << endl;
        }
    }

    if(!mJiGeometryReprojectionCsvPath.empty() &&
       !mvJiGeometryReprojectionDiagnostics.empty())
    {
        ofstream stream(mJiGeometryReprojectionCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Ji GJ-2] failed to open reprojection CSV: "
                 << mJiGeometryReprojectionCsvPath << endl;
        }
        else
        {
            stream << "frame,timestamp,initial_pose_available,cluster_id,"
                   << "depth_pixels,features,matched_map_support,"
                   << "optimizer_outlier_support,"
                   << "valid_reprojection_support,invalid_projections,"
                   << "evidence_state,mean_squared_error_px2,"
                   << "mean_error_px,median_error_px,p90_error_px,"
                   << "maximum_error_px,frame_features,"
                   << "features_assigned,frame_matched_observations,"
                   << "frame_optimizer_outliers_assigned,"
                   << "matches_assigned,frame_valid_reprojections,"
                   << "clusters_without_evidence,feature_assignment_ms,"
                   << "reprojection_ms,aggregate_ms,total_ms\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvJiGeometryReprojectionDiagnostics.size(); ++index)
            {
                const JiGeometryReprojectionRecord &record =
                    mvJiGeometryReprojectionDiagnostics[index];
                const JiReprojectionFrameStats &frame =
                    record.frameStats;
                const JiClusterReprojectionStats &cluster =
                    record.clusterStats;
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << static_cast<int>(frame.initialPoseAvailable) << ","
                       << cluster.clusterId << ","
                       << record.depthCluster.pixels << ","
                       << cluster.featureCount << ","
                       << cluster.matchedMapSupport << ","
                       << cluster.optimizerOutlierSupport << ","
                       << cluster.validReprojectionSupport << ","
                       << cluster.invalidProjectionCount << ","
                       << (cluster.hasGeometryEvidence ?
                           "measured" : "unknown") << ","
                       << cluster.meanSquaredErrorPixels2 << ","
                       << cluster.meanErrorPixels << ","
                       << cluster.medianErrorPixels << ","
                       << cluster.p90ErrorPixels << ","
                       << cluster.maximumErrorPixels << ","
                       << frame.featureCount << ","
                       << frame.featuresAssignedToClusters << ","
                       << frame.matchedObservations << ","
                       << frame.optimizerOutlierObservations << ","
                       << frame.matchesAssignedToClusters << ","
                       << frame.validReprojections << ","
                       << frame.clustersWithoutEvidence << ","
                       << frame.featureAssignmentMs << ","
                       << frame.reprojectionMs << ","
                       << frame.aggregateMs << ","
                       << frame.totalMs << "\n";
            }
            stream.close();
            cout << "[Ji GJ-2] saved "
                 << mvJiGeometryReprojectionDiagnostics.size()
                 << " cluster reprojection rows to "
                 << mJiGeometryReprojectionCsvPath << endl;
        }
    }

    mvGeometryPoseDiagnostics.clear();
    mvGeometrySemanticProxyDiagnostics.clear();
    mvGeometryFeatureShadowDiagnostics.clear();
    mvJiGeometryClusterDiagnostics.clear();
    mvJiGeometryReprojectionDiagnostics.clear();
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


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const cv::Mat &mask, const double &timestamp)
{
    if(imRGB.empty() || imD.empty())
        throw std::invalid_argument("RGB-D input images must be non-empty");
    if(imRGB.size()!=imD.size())
        throw std::invalid_argument(
            "RGB and registered depth images must have the same pixel dimensions");
    if(!std::isfinite(timestamp))
        throw std::invalid_argument("RGB-D timestamp must be finite");

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

    if(mbGeometryShadowEnabled || mbJiGeometryShadowEnabled)
        mCurrentDepthMeters = imDepth;
    if((mbGeometryShadowEnabled && mbGeometryDebugSaveEnabled) ||
       (mbJiGeometryShadowEnabled && mbJiGeometryDebugSaveEnabled))
    {
        mCurrentGeometryDebugImage = imRGB;
    }

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    cv::Mat semanticMask;
    if(!mask.empty())
    {
        if(mask.type()!=CV_8UC1 || mask.size()!=mImGray.size())
            throw std::invalid_argument("RGB-D semantic mask must be CV_8UC1 and match the RGB image size");

        cv::compare(mask,0,semanticMask,cv::CMP_NE);
    }
    UpdateDynamicFeaturesFromMask(mCurrentFrame,semanticMask);

    // DT-SLAM: 传递mask和检测列表给FrameDrawer用于Pangolin可视化
    mpFrameDrawer->UpdateMask(mCurrentFrame.mSemanticMask);

    if(mbJiGeometryReprojectionStatsEnabled)
    {
        mJiInitialTcw.release();
        mvJiInitialObservations.clear();
    }
    Track();
    if(mbJiGeometryReprojectionStatsEnabled && mState!=OK)
    {
        mJiInitialTcw.release();
        mvJiInitialObservations.clear();
    }
    RunJiGeometryShadow();

    if(mbGeometryShadowEnabled)
    {
        if(mState==OK && !mCurrentFrame.mTcw.empty())
        {
            cv::Mat referenceDepth = imDepth.clone();
            if(!semanticMask.empty())
                referenceDepth.setTo(0.0f,semanticMask);
            mGeometricDetector.UpdateReference(
                referenceDepth,mCurrentFrame.mTcw,mCurrentFrame.mnId,
                mCurrentFrame.mTimeStamp);
            if(!mCurrentGroundTruthTcw.empty())
            {
                mGeometricGroundTruthDetector.UpdateReference(
                    referenceDepth,mCurrentGroundTruthTcw,mCurrentFrame.mnId,
                    mCurrentFrame.mTimeStamp);
            }
            else
            {
                mGeometricGroundTruthDetector.ResetReference();
            }
        }
        else
        {
            mGeometricDetector.ResetReference();
            mGeometricGroundTruthDetector.ResetReference();
        }
    }

    mCurrentGeometryDebugImage.release();
    return mCurrentFrame.mTcw.clone();
}

void Tracking::UpdateDynamicFeaturesFromMask(Frame &frame, const cv::Mat &mask)
{
    frame.mSemanticMask = mask.empty() ? cv::Mat() : mask.clone();
    frame.mvbSemanticDynamic.assign(frame.N,0);
    frame.mvbDynamic.assign(frame.N,0);

    if(mask.empty())
        return;

    for(int i=0; i<frame.N; i++)
    {
        const int u = static_cast<int>(frame.mvKeys[i].pt.x);
        const int v = static_cast<int>(frame.mvKeys[i].pt.y);
        if(u>=0 && u<mask.cols && v>=0 && v<mask.rows && mask.at<uchar>(v,u)!=0)
        {
            frame.mvbSemanticDynamic[i] = 1;
            frame.mvbDynamic[i] = 1;
        }
    }
}

int Tracking::RemoveDynamicAssociations(Frame &frame)
{
    int removed = 0;
    const size_t count = std::min(frame.mvpMapPoints.size(),frame.mvbDynamic.size());
    for(size_t i=0; i<count; i++)
    {
        if(frame.mvbDynamic[i] && frame.mvpMapPoints[i])
        {
            frame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            removed++;
        }
    }
    return removed;
}

void Tracking::RunGeometryShadow()
{
    if(!mbGeometryShadowEnabled || mSensor!=System::RGBD ||
       mCurrentDepthMeters.empty() || mCurrentFrame.mTcw.empty() ||
       !mGeometricDetector.HasReference())
    {
        return;
    }

    GeometricWarpResult result;
    if(!mGeometricDetector.Compute(
           mCurrentDepthMeters,mCurrentFrame.mTcw,result))
    {
        return;
    }

    ++mnGeometryComputedFrames;
    GeometricWarpResult groundTruthResult;
    const bool hasGroundTruthResult =
        !mCurrentGroundTruthTcw.empty() &&
        mGeometricGroundTruthDetector.HasReference() &&
        mGeometricGroundTruthDetector.Compute(
            mCurrentDepthMeters,mCurrentGroundTruthTcw,groundTruthResult);
    if(hasGroundTruthResult)
    {
        GeometryPoseDiagnosticRecord record;
        record.frameId = mCurrentFrame.mnId;
        record.referenceFrameId =
            mGeometricGroundTruthDetector.ReferenceFrameId();
        record.timestamp = mCurrentFrame.mTimeStamp;
        record.referenceTimestamp =
            mGeometricGroundTruthDetector.ReferenceTimestampSeconds();
        record.slam = result.stats;
        record.groundTruth = groundTruthResult.stats;
        mvGeometryPoseDiagnostics.push_back(record);
    }

    GeometrySemanticProxyStats semanticProxyStats;
    GeometrySemanticProxyStats groundTruthSemanticProxyStats;
    const bool hasSemanticProxy =
        !mCurrentFrame.mSemanticMask.empty() &&
        mCurrentFrame.mSemanticMask.type()==CV_8UC1 &&
        mCurrentFrame.mSemanticMask.size()==result.validComparisonMask.size();
    if(hasSemanticProxy)
    {
        semanticProxyStats =
            ComputeSemanticProxyStats(result,mCurrentFrame.mSemanticMask);
        if(hasGroundTruthResult)
        {
            groundTruthSemanticProxyStats =
                ComputeSemanticProxyStats(
                    groundTruthResult,mCurrentFrame.mSemanticMask);
        }

        GeometrySemanticProxyRecord proxyRecord;
        proxyRecord.frameId = mCurrentFrame.mnId;
        proxyRecord.referenceFrameId =
            mGeometricDetector.ReferenceFrameId();
        proxyRecord.timestamp = mCurrentFrame.mTimeStamp;
        proxyRecord.hasGroundTruth = hasGroundTruthResult;
        proxyRecord.slam = semanticProxyStats;
        proxyRecord.groundTruth = groundTruthSemanticProxyStats;
        mvGeometrySemanticProxyDiagnostics.push_back(proxyRecord);
    }

    std::vector<GeometryFeatureShadowStats> featureShadowStats;
    if(!mGeometryFeatureShadowCsvPath.empty())
    {
        featureShadowStats = ComputeFeatureShadowStats(
            result,mCurrentFrame,mCurrentFrame.mSemanticMask);
        for(std::size_t index=0; index<featureShadowStats.size(); ++index)
        {
            GeometryFeatureShadowRecord featureRecord;
            featureRecord.frameId = mCurrentFrame.mnId;
            featureRecord.referenceFrameId =
                mGeometricDetector.ReferenceFrameId();
            featureRecord.timestamp = mCurrentFrame.mTimeStamp;
            featureRecord.groundTruthPose = false;
            featureRecord.stats = featureShadowStats[index];
            mvGeometryFeatureShadowDiagnostics.push_back(featureRecord);
        }
        if(hasGroundTruthResult)
        {
            const std::vector<GeometryFeatureShadowStats>
                groundTruthFeatureShadowStats =
                    ComputeFeatureShadowStats(
                        groundTruthResult,mCurrentFrame,
                        mCurrentFrame.mSemanticMask);
            for(std::size_t index=0;
                index<groundTruthFeatureShadowStats.size(); ++index)
            {
                GeometryFeatureShadowRecord featureRecord;
                featureRecord.frameId = mCurrentFrame.mnId;
                featureRecord.referenceFrameId =
                    mGeometricGroundTruthDetector.ReferenceFrameId();
                featureRecord.timestamp = mCurrentFrame.mTimeStamp;
                featureRecord.groundTruthPose = true;
                featureRecord.stats = groundTruthFeatureShadowStats[index];
                mvGeometryFeatureShadowDiagnostics.push_back(featureRecord);
            }
        }
    }

    if(mbGeometryDebugSaveEnabled &&
       (mnGeometryComputedFrames==1 ||
        mnGeometryComputedFrames%static_cast<long unsigned int>(mnGeometryDebugEveryN)==0))
    {
        SaveGeometryDebugImages(result);
    }

    if(mnGeometryComputedFrames==1 ||
       mnGeometryComputedFrames%static_cast<long unsigned int>(mnGeometryLogEveryN)==0)
    {
        cout << "[Geometry G0] frame=" << mCurrentFrame.mnId
             << " ref=" << mGeometricDetector.ReferenceFrameId()
             << " current_ts=" << std::setprecision(15)
             << mCurrentFrame.mTimeStamp
             << " ref_ts=" << mGeometricDetector.ReferenceTimestampSeconds()
             << " dt_s="
             << mCurrentFrame.mTimeStamp-
                mGeometricDetector.ReferenceTimestampSeconds()
             << std::setprecision(6)
             << " ref_valid=" << result.stats.referenceValidPixels
             << " projected=" << result.stats.projectedSamples
             << " zbuffer=" << result.stats.zbufferValidPixels
             << " current_valid=" << result.stats.currentValidPixels
             << " comparisons=" << result.stats.validComparisons
             << " pred_coverage=" << result.stats.predictionCoverageRatio
             << " compare_coverage=" << result.stats.comparisonCoverageRatio
             << " residual_mean_m=" << result.stats.residualMean
             << " residual_mean_abs_m=" << result.stats.residualMeanAbs
             << " residual_max_abs_m=" << result.stats.residualMaxAbs
             << " consistent=" << result.stats.consistentEvidencePixels
             << " positive_seed=" << result.stats.positiveSeedPixels
             << " negative_diag=" << result.stats.negativeDiagnosticPixels
             << " consistent_ratio=" << result.stats.consistentEvidenceRatio
             << " positive_ratio=" << result.stats.positiveSeedRatio
             << " negative_ratio=" << result.stats.negativeDiagnosticRatio
             << " regions=" << result.stats.depthRegionCount
             << " region_pixels=" << result.stats.regionCandidatePixels
             << " largest_region=" << result.stats.largestRegionPixels
             << " growth_ratio=" << result.stats.regionGrowthRatio
             << " warp_ms=" << result.stats.warpMs
             << " residual_ms=" << result.stats.residualMs
             << " evidence_ms=" << result.stats.evidenceMs
             << " region_ms=" << result.stats.regionGrowMs
             << " total_ms=" << result.stats.totalMs
             << endl;

        if(!result.depthRegions.empty())
        {
            std::vector<std::size_t> regionOrder(
                result.depthRegions.size());
            for(std::size_t index=0; index<regionOrder.size(); ++index)
                regionOrder[index] = index;
            std::sort(
                regionOrder.begin(),regionOrder.end(),
                [&result](const std::size_t left, const std::size_t right)
                {
                    const GeometricDepthRegionStats &leftRegion =
                        result.depthRegions[left];
                    const GeometricDepthRegionStats &rightRegion =
                        result.depthRegions[right];
                    if(leftRegion.positiveSeedPixels!=
                       rightRegion.positiveSeedPixels)
                    {
                        return leftRegion.positiveSeedPixels>
                               rightRegion.positiveSeedPixels;
                    }
                    return leftRegion.pixels>rightRegion.pixels;
                });

            const std::size_t reportCount =
                std::min<std::size_t>(3,regionOrder.size());
            for(std::size_t rank=0; rank<reportCount; ++rank)
            {
                const GeometricDepthRegionStats &region =
                    result.depthRegions[regionOrder[rank]];
                cout << "[Geometry G0-3R] frame=" << mCurrentFrame.mnId
                     << " rank=" << rank+1
                     << " pixels=" << region.pixels
                     << " positive=" << region.positiveSeedPixels
                     << " negative=" << region.negativeDiagnosticPixels
                     << " positive_ratio=" << region.positiveSeedRatio
                     << " negative_ratio=" << region.negativeDiagnosticRatio
                     << " residual_median_m="
                     << region.signedResidualMedian
                     << endl;
            }
        }

        if(hasGroundTruthResult)
        {
            cout << "[Geometry G0-2P] frame=" << mCurrentFrame.mnId
                 << " ref=" << mGeometricGroundTruthDetector.ReferenceFrameId()
                 << " slam_comparisons=" << result.stats.validComparisons
                 << " gt_comparisons="
                 << groundTruthResult.stats.validComparisons
                 << " slam_compare_coverage="
                 << result.stats.comparisonCoverageRatio
                 << " gt_compare_coverage="
                 << groundTruthResult.stats.comparisonCoverageRatio
                 << " slam_mean_abs_m=" << result.stats.residualMeanAbs
                 << " gt_mean_abs_m="
                 << groundTruthResult.stats.residualMeanAbs
                 << " slam_positive_ratio="
                 << result.stats.positiveSeedRatio
                 << " gt_positive_ratio="
                 << groundTruthResult.stats.positiveSeedRatio
                 << " slam_negative_ratio="
                 << result.stats.negativeDiagnosticRatio
                 << " gt_negative_ratio="
                 << groundTruthResult.stats.negativeDiagnosticRatio
                 << " gt_total_ms=" << groundTruthResult.stats.totalMs
                 << endl;
        }
        if(hasSemanticProxy)
        {
            cout << "[Geometry G0-2A] frame=" << mCurrentFrame.mnId
                 << " semantic_pixels=" << semanticProxyStats.semanticPixels
                 << " semantic_valid="
                 << semanticProxyStats.semanticValidPixels
                 << " positive_inside="
                 << semanticProxyStats.positiveInsideSemanticPixels
                 << " positive_outside="
                 << semanticProxyStats.positiveOutsideSemanticPixels
                 << " proxy_precision="
                 << semanticProxyStats.proxyPrecision
                 << " conditional_recall="
                 << semanticProxyStats.conditionalRecall
                 << " static_fpr="
                 << semanticProxyStats.staticBackgroundFpr
                 << endl;
        }
        if(!featureShadowStats.empty())
        {
            const GeometryFeatureShadowStats &center =
                featureShadowStats.front();
            const GeometryFeatureShadowStats &radiusTwo =
                featureShadowStats[2];
            cout << "[Geometry G0-4F] frame=" << mCurrentFrame.mnId
                 << " features=" << center.featureCount
                 << " semantic_features=" << center.semanticFeatureCount
                 << " r0_eligible=" << center.eligibleFeatureCount
                 << " r0_candidates=" << center.candidateFeatureCount
                 << " r0_proxy_precision=" << center.proxyPrecision
                 << " r0_conditional_recall=" << center.conditionalRecall
                 << " r0_proxy_background_rate="
                 << center.proxyBackgroundRate
                 << " r2_eligible=" << radiusTwo.eligibleFeatureCount
                 << " r2_candidates=" << radiusTwo.candidateFeatureCount
                 << " r2_proxy_precision=" << radiusTwo.proxyPrecision
                 << " r2_conditional_recall="
                 << radiusTwo.conditionalRecall
                 << " r2_proxy_background_rate="
                 << radiusTwo.proxyBackgroundRate
                 << endl;
        }
    }
}

void Tracking::CaptureJiInitialTrackingSnapshot()
{
    if(!mbJiGeometryReprojectionStatsEnabled ||
       mSensor!=System::RGBD || mCurrentFrame.mTcw.empty())
    {
        return;
    }

    mJiInitialTcw = mCurrentFrame.mTcw.clone();
    mvJiInitialObservations.clear();
    const std::size_t count = std::min(
        std::min(mCurrentFrame.mvKeys.size(),
                 mCurrentFrame.mvKeysUn.size()),
        std::min(mCurrentFrame.mvpMapPoints.size(),
                 mCurrentFrame.mvbOutlier.size()));
    mvJiInitialObservations.reserve(count);
    for(std::size_t index=0; index<count; ++index)
    {
        MapPoint *mapPoint = mCurrentFrame.mvpMapPoints[index];
        if(!mapPoint || mapPoint->isBad() ||
           mapPoint->Observations()<1)
        {
            continue;
        }

        const cv::Mat worldPosition = mapPoint->GetWorldPos();
        if(worldPosition.empty() || worldPosition.total()!=3 ||
           worldPosition.channels()!=1 ||
           !cv::checkRange(worldPosition))
        {
            continue;
        }
        cv::Mat worldPositionFloat;
        worldPosition.convertTo(worldPositionFloat,CV_32F);
        worldPositionFloat = worldPositionFloat.reshape(1,3);

        JiReprojectionObservation observation;
        observation.rawPixel = mCurrentFrame.mvKeys[index].pt;
        observation.observedPinholePixel =
            mCurrentFrame.mvKeysUn[index].pt;
        observation.worldPoint = cv::Point3f(
            worldPositionFloat.at<float>(0),
            worldPositionFloat.at<float>(1),
            worldPositionFloat.at<float>(2));
        observation.optimizerOutlier =
            mCurrentFrame.mvbOutlier[index];
        mvJiInitialObservations.push_back(observation);
    }
}

void Tracking::RunJiGeometryShadow()
{
    if(!mbJiGeometryShadowEnabled || mSensor!=System::RGBD ||
       mCurrentDepthMeters.empty())
    {
        return;
    }

    JiDepthClusteringResult result;
    try
    {
        if(!mJiGeometryBaseline.ComputeDepthClusters(
               mCurrentDepthMeters,result))
        {
            cerr << "[Ji GJ-1] frame=" << mCurrentFrame.mnId
                 << " clustering skipped: invalid input or fewer valid "
                 << "depth pixels than requested clusters" << endl;
            return;
        }
    }
    catch(const cv::Exception &error)
    {
        cerr << "[Ji GJ-1] frame=" << mCurrentFrame.mnId
             << " OpenCV K-means failed: " << error.what() << endl;
        return;
    }
    catch(const std::exception &error)
    {
        cerr << "[Ji GJ-1] frame=" << mCurrentFrame.mnId
             << " clustering failed: " << error.what() << endl;
        return;
    }

    ++mnJiGeometryComputedFrames;
    if(!mJiGeometryClusterCsvPath.empty())
    {
        for(std::size_t index=0; index<result.clusters.size(); ++index)
        {
            JiGeometryClusterRecord record;
            record.frameId = mCurrentFrame.mnId;
            record.timestamp = mCurrentFrame.mTimeStamp;
            record.frameStats = result.stats;
            record.cluster = result.clusters[index];
            mvJiGeometryClusterDiagnostics.push_back(record);
        }
    }

    JiReprojectionFrameStats reprojectionFrameStats;
    bool reprojectionStatsComputed = false;
    if(mbJiGeometryReprojectionStatsEnabled)
    {
        std::vector<cv::Point2f> rawFeaturePixels;
        rawFeaturePixels.reserve(mCurrentFrame.mvKeys.size());
        for(std::size_t index=0;
            index<mCurrentFrame.mvKeys.size(); ++index)
        {
            rawFeaturePixels.push_back(
                mCurrentFrame.mvKeys[index].pt);
        }

        std::vector<JiClusterReprojectionStats> clusterStats;
        reprojectionStatsComputed =
            mJiGeometryBaseline.ComputeClusterReprojectionStats(
                result,rawFeaturePixels,mvJiInitialObservations,
                mJiInitialTcw,mK,clusterStats,reprojectionFrameStats);
        if(!reprojectionStatsComputed)
        {
            cerr << "[Ji GJ-2] frame=" << mCurrentFrame.mnId
                 << " reprojection statistics failed input validation"
                 << endl;
        }
        else if(!mJiGeometryReprojectionCsvPath.empty())
        {
            const std::size_t count = std::min(
                result.clusters.size(),clusterStats.size());
            for(std::size_t index=0; index<count; ++index)
            {
                JiGeometryReprojectionRecord record;
                record.frameId = mCurrentFrame.mnId;
                record.timestamp = mCurrentFrame.mTimeStamp;
                record.depthCluster = result.clusters[index];
                record.frameStats = reprojectionFrameStats;
                record.clusterStats = clusterStats[index];
                mvJiGeometryReprojectionDiagnostics.push_back(record);
            }
        }
    }

    if(mbJiGeometryDebugSaveEnabled &&
       (mnJiGeometryComputedFrames==1 ||
        mnJiGeometryComputedFrames%
            static_cast<long unsigned int>(mnJiGeometryDebugEveryN)==0))
    {
        SaveJiGeometryDebugImages(result);
    }

    if(mnJiGeometryComputedFrames==1 ||
       mnJiGeometryComputedFrames%
           static_cast<long unsigned int>(mnJiGeometryLogEveryN)==0)
    {
        cout << "[Ji GJ-1] frame=" << mCurrentFrame.mnId
             << " valid_depth=" << result.stats.validDepthPixels
             << " clusters=" << result.stats.producedClusters
             << " smallest_cluster="
             << result.stats.smallestClusterPixels
             << " largest_cluster="
             << result.stats.largestClusterPixels
             << " compactness=" << result.stats.compactness
             << " prepare_ms=" << result.stats.prepareMs
             << " kmeans_ms=" << result.stats.kmeansMs
             << " label_ms=" << result.stats.labelMs
             << " total_ms=" << result.stats.totalMs
             << " direct_slam_state_mutation=none" << endl;
        if(reprojectionStatsComputed)
        {
            cout << "[Ji GJ-2] frame=" << mCurrentFrame.mnId
                 << " initial_pose="
                 << static_cast<int>(
                     reprojectionFrameStats.initialPoseAvailable)
                 << " features="
                 << reprojectionFrameStats.featureCount
                 << " assigned_features="
                 << reprojectionFrameStats.featuresAssignedToClusters
                 << " initial_matches="
                 << reprojectionFrameStats.matchedObservations
                 << " optimizer_outliers_assigned="
                 << reprojectionFrameStats.optimizerOutlierObservations
                 << " assigned_matches="
                 << reprojectionFrameStats.matchesAssignedToClusters
                 << " valid_reprojections="
                 << reprojectionFrameStats.validReprojections
                 << " unknown_clusters="
                 << reprojectionFrameStats.clustersWithoutEvidence
                 << " assignment_ms="
                 << reprojectionFrameStats.featureAssignmentMs
                 << " reprojection_ms="
                 << reprojectionFrameStats.reprojectionMs
                 << " aggregate_ms="
                 << reprojectionFrameStats.aggregateMs
                 << " total_ms="
                 << reprojectionFrameStats.totalMs
                 << " dynamic_decision=none" << endl;
        }
    }
}

void Tracking::SaveJiGeometryDebugImages(
    const JiDepthClusteringResult &result)
{
    if(mCurrentGeometryDebugImage.empty() ||
       mJiGeometryDebugOutputDir.empty() ||
       result.labelImage.empty())
    {
        return;
    }
    if(result.labelImage.type()!=CV_16SC1 ||
       result.labelImage.size()!=mCurrentGeometryDebugImage.size())
    {
        cerr << "[Ji GJ-1] frame=" << mCurrentFrame.mnId
             << " debug images skipped because RGB and label domains differ"
             << endl;
        return;
    }

    cv::Mat baseImage;
    if(mCurrentGeometryDebugImage.channels()==1)
    {
        cv::cvtColor(
            mCurrentGeometryDebugImage,baseImage,CV_GRAY2BGR);
    }
    else if(mCurrentGeometryDebugImage.channels()==3)
    {
        baseImage = mCurrentGeometryDebugImage.clone();
    }
    else if(mCurrentGeometryDebugImage.channels()==4)
    {
        cv::cvtColor(
            mCurrentGeometryDebugImage,baseImage,CV_BGRA2BGR);
    }
    else
    {
        cerr << "[Ji GJ-1] frame=" << mCurrentFrame.mnId
             << " debug images skipped because RGB has "
             << mCurrentGeometryDebugImage.channels() << " channels"
             << endl;
        return;
    }

    cv::Mat hsv(
        result.labelImage.size(),CV_8UC3,cv::Scalar(0,0,0));
    const int clusterCount =
        std::max(1,result.stats.producedClusters);
    for(int v=0; v<result.labelImage.rows; ++v)
    {
        const short *labelRow = result.labelImage.ptr<short>(v);
        cv::Vec3b *hsvRow = hsv.ptr<cv::Vec3b>(v);
        for(int u=0; u<result.labelImage.cols; ++u)
        {
            const int label = labelRow[u];
            if(label<0)
                continue;
            hsvRow[u] = cv::Vec3b(
                static_cast<uchar>((label*179)/clusterCount),
                220,255);
        }
    }

    cv::Mat clusterColor;
    cv::cvtColor(hsv,clusterColor,CV_HSV2BGR);
    cv::Mat overlay;
    cv::addWeighted(baseImage,0.55,clusterColor,0.45,0.0,overlay);

    std::ostringstream filenamePrefix;
    filenamePrefix << mJiGeometryDebugOutputDir;
    if(mJiGeometryDebugOutputDir[
           mJiGeometryDebugOutputDir.size()-1]!='/')
    {
        filenamePrefix << "/";
    }
    filenamePrefix << "frame_" << std::setfill('0') << std::setw(6)
                   << mCurrentFrame.mnId;
    const std::string prefix = filenamePrefix.str();

    bool saved = false;
    try
    {
        saved =
            cv::imwrite(prefix+"_ji_clusters.png",clusterColor) &&
            cv::imwrite(prefix+"_ji_cluster_overlay.png",overlay);
    }
    catch(const cv::Exception &error)
    {
        cerr << "[Ji GJ-1] frame=" << mCurrentFrame.mnId
             << " debug image write failed: " << error.what() << endl;
        return;
    }

    if(saved)
    {
        cout << "[Ji GJ-1] frame=" << mCurrentFrame.mnId
             << " saved cluster visualization prefix=" << prefix << endl;
    }
    else
    {
        cerr << "[Ji GJ-1] frame=" << mCurrentFrame.mnId
             << " cluster visualization could not be written under "
             << mJiGeometryDebugOutputDir << endl;
    }
}

void Tracking::SaveGeometryDebugImages(const GeometricWarpResult &result)
{
    if(mCurrentGeometryDebugImage.empty() || mGeometryDebugOutputDir.empty())
        return;

    if(result.validComparisonMask.size()!=mCurrentGeometryDebugImage.size() ||
       result.positiveSeedMask.size()!=mCurrentGeometryDebugImage.size() ||
       result.negativeDiagnosticMask.size()!=mCurrentGeometryDebugImage.size() ||
       (!result.regionCandidateMask.empty() &&
        result.regionCandidateMask.size()!=mCurrentGeometryDebugImage.size()) ||
       (!result.regionPositiveSupport.empty() &&
        result.regionPositiveSupport.size()!=mCurrentGeometryDebugImage.size()))
    {
        cerr << "[Geometry G0-2V] frame=" << mCurrentFrame.mnId
             << " debug images skipped because image and mask sizes differ" << endl;
        return;
    }

    const std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();

    cv::Mat baseImage;
    if(mCurrentGeometryDebugImage.channels()==1)
    {
        cv::cvtColor(mCurrentGeometryDebugImage,baseImage,CV_GRAY2BGR);
    }
    else if(mCurrentGeometryDebugImage.channels()==3)
    {
        baseImage = mCurrentGeometryDebugImage.clone();
    }
    else if(mCurrentGeometryDebugImage.channels()==4)
    {
        cv::cvtColor(mCurrentGeometryDebugImage,baseImage,CV_BGRA2BGR);
    }
    else
    {
        cerr << "[Geometry G0-2V] frame=" << mCurrentFrame.mnId
             << " debug images skipped because the input image has "
             << mCurrentGeometryDebugImage.channels() << " channels" << endl;
        return;
    }

    cv::Mat colorImage = baseImage.clone();
    colorImage.setTo(cv::Scalar(0,0,255),result.positiveSeedMask);
    colorImage.setTo(cv::Scalar(255,0,0),result.negativeDiagnosticMask);

    cv::Mat overlay;
    cv::addWeighted(baseImage,0.65,colorImage,0.35,0.0,overlay);

    cv::Mat regionOverlay;
    if(!result.regionCandidateMask.empty())
    {
        cv::Mat regionColorImage = baseImage.clone();
        regionColorImage.setTo(
            cv::Scalar(0,255,0),result.regionCandidateMask);
        regionColorImage.setTo(
            cv::Scalar(0,0,255),result.positiveSeedMask);
        regionColorImage.setTo(
            cv::Scalar(255,0,0),result.negativeDiagnosticMask);
        cv::addWeighted(
            baseImage,0.65,regionColorImage,0.35,0.0,regionOverlay);
    }

    std::ostringstream filenamePrefix;
    filenamePrefix << mGeometryDebugOutputDir;
    if(mGeometryDebugOutputDir[mGeometryDebugOutputDir.size()-1]!='/')
        filenamePrefix << "/";
    filenamePrefix << "frame_" << std::setfill('0') << std::setw(6)
                   << mCurrentFrame.mnId;
    const std::string prefix = filenamePrefix.str();

    bool saved = false;
    try
    {
        saved =
            cv::imwrite(prefix+"_valid.png",result.validComparisonMask) &&
            cv::imwrite(prefix+"_positive.png",result.positiveSeedMask) &&
            cv::imwrite(prefix+"_negative.png",result.negativeDiagnosticMask) &&
            cv::imwrite(prefix+"_overlay.png",overlay);
        if(saved && !mCurrentFrame.mSemanticMask.empty())
        {
            cv::Mat proxyColorImage = baseImage.clone();
            proxyColorImage.setTo(
                cv::Scalar(0,255,255),mCurrentFrame.mSemanticMask);
            proxyColorImage.setTo(
                cv::Scalar(0,0,255),result.positiveSeedMask);
            cv::Mat positiveInsideSemantic;
            cv::bitwise_and(
                mCurrentFrame.mSemanticMask,result.positiveSeedMask,
                positiveInsideSemantic);
            proxyColorImage.setTo(
                cv::Scalar(255,0,255),positiveInsideSemantic);
            cv::Mat proxyOverlay;
            cv::addWeighted(
                baseImage,0.65,proxyColorImage,0.35,0.0,proxyOverlay);
            saved =
                cv::imwrite(
                    prefix+"_semantic_proxy.png",
                    mCurrentFrame.mSemanticMask) &&
                cv::imwrite(
                    prefix+"_proxy_overlay.png",proxyOverlay);
        }
        if(saved && !result.regionCandidateMask.empty())
        {
            saved =
                cv::imwrite(
                    prefix+"_region.png",result.regionCandidateMask) &&
                cv::imwrite(
                    prefix+"_region_overlay.png",regionOverlay);
        }
        if(saved && !result.regionPositiveSupport.empty())
        {
            cv::Mat regionSupportImage;
            result.regionPositiveSupport.convertTo(
                regionSupportImage,CV_8UC1,255.0);
            saved = cv::imwrite(
                prefix+"_region_support.png",regionSupportImage);
        }
    }
    catch(const cv::Exception &error)
    {
        cerr << "[Geometry G0-2V] frame=" << mCurrentFrame.mnId
             << " debug image write failed: " << error.what() << endl;
        return;
    }

    const std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    const double writeMs =
        std::chrono::duration<double,std::milli>(end-start).count();

    if(saved)
    {
        cout << "[Geometry G0-2V] frame=" << mCurrentFrame.mnId
             << " debug_write_ms=" << writeMs
             << " prefix=" << prefix << endl;
    }
    else
    {
        cerr << "[Geometry G0-2V] frame=" << mCurrentFrame.mnId
             << " one or more debug images could not be written under "
             << mGeometryDebugOutputDir << endl;
    }
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

    Track();

    return mCurrentFrame.mTcw.clone();
}

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
                        if(mbJiGeometryReprojectionStatsEnabled)
                        {
                            mJiInitialTcw.release();
                            mvJiInitialObservations.clear();
                        }
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(bOK)
        {
            if(mJiInitialTcw.empty())
                CaptureJiInitialTrackingSnapshot();
            RunGeometryShadow();
        }

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
    int nStaticFeatures = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(!mCurrentFrame.mvbDynamic[i])
            nStaticFeatures++;
    }

    if(nStaticFeatures>500)
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
            if(z>0 && !mCurrentFrame.mvbDynamic[i])
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

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


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

    nmatches -= RemoveDynamicAssociations(mCurrentFrame);
    if(nmatches<15)
        return false;

    Optimizer::PoseOptimization(&mCurrentFrame);
    CaptureJiInitialTrackingSnapshot();

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
        if(z>0 && !mLastFrame.mvbDynamic[i])
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

    nmatches -= RemoveDynamicAssociations(mCurrentFrame);
    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);
    CaptureJiInitialTrackingSnapshot();

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

    RemoveDynamicAssociations(mCurrentFrame);

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
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth &&
               !mCurrentFrame.mvbDynamic[i])
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

    RemoveDynamicAssociations(mCurrentFrame);

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
            if(z>0 && !mCurrentFrame.mvbDynamic[i])
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
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                RemoveDynamicAssociations(mCurrentFrame);
                for(int j=0; j<mCurrentFrame.N; j++)
                    if(mCurrentFrame.mvpMapPoints[j])
                        sFound.insert(mCurrentFrame.mvpMapPoints[j]);

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
                        RemoveDynamicAssociations(mCurrentFrame);
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
                                RemoveDynamicAssociations(mCurrentFrame);
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
    mGeometricDetector.ResetReference();
    mGeometricGroundTruthDetector.ResetReference();
    mCurrentDepthMeters.release();
    mCurrentGeometryDebugImage.release();
    mCurrentGroundTruthTcw.release();
    mvGeometryPoseDiagnostics.clear();
    mvGeometrySemanticProxyDiagnostics.clear();
    mvGeometryFeatureShadowDiagnostics.clear();
    mnGeometryComputedFrames = 0;

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

    LoadGeometryCameraMatrix(
        fSettings,mK,mGeometryK,mbGeometryUsesDedicatedCameraModel);
    mGeometricDetector.SetCameraMatrix(mGeometryK);
    mGeometricGroundTruthDetector.SetCameraMatrix(mGeometryK);
    mJiGeometryBaseline.SetCameraMatrix(mGeometryK);
    if((mbGeometryShadowEnabled || mbJiGeometryShadowEnabled) &&
       !mbGeometryUsesDedicatedCameraModel &&
       cv::norm(mDistCoef,cv::NORM_INF)>1e-8)
    {
        throw std::invalid_argument(
            "geometry calibration change requires zero distortion in the "
            "current raw-pixel pinhole implementation unless a dedicated "
            "Geometry.Camera model is configured");
    }

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
