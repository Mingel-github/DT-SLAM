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
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include<iostream>
#include <limits>
#include <numeric>
#include <sstream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

namespace
{

template<typename T>
std::string JoinDiagnosticValues(const std::vector<T> &values)
{
    std::ostringstream stream;
    for(std::size_t index=0; index<values.size(); ++index)
    {
        if(index>0)
            stream << ";";
        stream << values[index];
    }
    return stream.str();
}

std::set<long unsigned int> ParseFrameIdFilter(const char *value)
{
    std::set<long unsigned int> frameIds;
    if(!value || value[0]=='\0')
        return frameIds;

    std::stringstream stream(value);
    std::string token;
    while(std::getline(stream,token,','))
    {
        if(token.empty())
            throw std::invalid_argument(
                "geometry feature frame filter contains an empty item");
        char *end = NULL;
        const unsigned long parsed =
            std::strtoul(token.c_str(),&end,10);
        if(end==token.c_str() || *end!='\0')
        {
            throw std::invalid_argument(
                "geometry feature frame filter must contain "
                "comma-separated non-negative integers");
        }
        frameIds.insert(static_cast<long unsigned int>(parsed));
    }
    return frameIds;
}

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
    mbSInStyleShadowEnabled(false),
    mbSInStyleNativeInitialRegionsEnabled(false),
    mbSInStyleNativeGradientSplitEnabled(false),
    mbSInStyleNativePlaneEdgeEnabled(false),
    mbSInStyleNativeRAGMergeEnabled(false),
    mbSInStyleDenseFlowResidualEnabled(false),
    mbSInStyleRegionDynamicEnabled(false),
    mbSInStyleRegionFeatureFilterEnabled(false),
    mbSInStyleDepthFilterEnabled(false),
    mnSInStyleRegionFeatureFilterMinimumRemainingFeatures(250),
    mnSInStyleLogEveryN(30),
    mSInStyleDepthFilterMaskMode("semantic_or_geometry"),
    mbCurrentSInRegionTrackingFailOpen(false),
    mbCurrentSInGeometryEvidenceAvailable(false),
    mbCurrentSInDepthMappingAdmissible(false),
    mnSInStyleComputedFrames(0), mnSInStyleInputFrameIndex(0),
    mnSInStyleResetEpoch(0),
    mnSInStyleDepthFilterInputFrameIndex(0),
    mbGeometryShadowEnabled(false),
    mbGeometrySingleReferenceShadowEnabled(true),
    mbGeometryDebugSaveEnabled(false),
    mbGeometryUsesDedicatedCameraModel(false),
    mnGeometryLogEveryN(30), mnGeometryDebugEveryN(30),
    mnGeometryComputedFrames(0),
    mbGeometryMultiReferenceShadowEnabled(false),
    mnGeometryMultiReferenceMaxReferences(5),
    mnGeometryMultiReferenceHistorySize(20),
    mGeometryMultiReferenceSelectionPolicy("recent"),
    mGeometryMultiReferenceSamplingPolicy("dense"),
    mnGeometryMultiReferenceGridStride(4),
    mnGeometryMultiReferencePyramidScale(2),
    mbGeometryMultiReferenceDenseAuditEnabled(false),
    mnGeometryMultiReferenceComputedFrames(0),
    mbGeometrySparseEgoFlowShadowEnabled(false),
    mnGeometrySparseFlowComputedFrames(0),
    mbGeometrySparseFlowCounterfactualShadowEnabled(false),
    mbGeometrySparseFlowTrackingFilterEnabled(false),
    mfGeometrySparseFlowTrackingFilterQ(10.0f),
    mfGeometrySparseFlowTrackingFilterMaximumAssociationFraction(0.05f),
    mnGeometrySparseFlowTrackingFilterMinimumAssociations(30),
    mnGeometrySparseFlowTrackingFilterMinimumScaleSupport(20),
    mbCurrentSparseFlowTrackingSafeguardsPassed(false),
    mCurrentSparseFlowTrackingSafeguardState("not_evaluated"),
    mbGeometrySparseFlowMappingCounterfactualEnabled(false),
    mbGeometrySparseFlowMappingFilterEnabled(false),
    mfGeometrySparseFlowMappingFilterMaximumFeatureFraction(0.05f),
    mfGeometrySparseFlowMappingFilterMaximumDepthFraction(0.05f),
    mnGeometrySparseFlowMappingFilterMinimumRemainingDepthFeatures(100),
    mbGeometrySparseFlowMapQualityAuditEnabled(false),
    mbGeometryLocalRigidityShadowEnabled(false),
    mnGeometryLocalRigidityComputedFrames(0),
    mbGeometryRigidHypothesisShadowEnabled(false),
    mnGeometryRigidHypothesisComputedFrames(0),
    mbGeometryRegionEvidenceShadowEnabled(false),
    mbGeometryRegionRiskDiagnosticsEnabled(false),
    mbGeometryLowResolutionRegionShadowEnabled(false),
    mGeometryRegionRelativeThreshold(0.025f),
    mGeometryRegionAbsoluteThresholdMeters(0.08f),
    mnGeometryRegionEvidenceComputedFrames(0),
    mbJiGeometryShadowEnabled(false),
    mbJiGeometryReprojectionStatsEnabled(false),
    mbJiGeometryDebugSaveEnabled(false),
    mbJiGeometryDebugRawLabelsOnly(false),
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

    SInStyleDetectorConfig sinStyleConfig;
    const cv::FileNode sinStyleEnableNode =
        fSettings["SInStyle.ShadowEnable"];
    if(!sinStyleEnableNode.empty())
        mbSInStyleShadowEnabled = static_cast<int>(sinStyleEnableNode)!=0;
    sinStyleConfig.enabled = mbSInStyleShadowEnabled;

    const cv::FileNode sinStyleBackendNode =
        fSettings["SInStyle.Backend"];
    if(!sinStyleBackendNode.empty())
        sinStyleBackendNode >> sinStyleConfig.backend;

    const cv::FileNode sinStyleReferenceDirectoryNode =
        fSettings["SInStyle.ReferenceDirectory"];
    if(!sinStyleReferenceDirectoryNode.empty())
        sinStyleReferenceDirectoryNode >> sinStyleConfig.referenceDirectory;
    const char *sinStyleReferenceDirectoryOverride =
        std::getenv("DT_SLAM_SIN_REFERENCE_DIR");
    if(sinStyleReferenceDirectoryOverride &&
       sinStyleReferenceDirectoryOverride[0]!='\0')
    {
        sinStyleConfig.referenceDirectory =
            sinStyleReferenceDirectoryOverride;
    }

    const cv::FileNode sinStyleReferenceBackendNode =
        fSettings["SInStyle.ReferenceBackend"];
    if(!sinStyleReferenceBackendNode.empty())
        sinStyleReferenceBackendNode >> mSInStyleReferenceBackend;
    if(mbSInStyleShadowEnabled && mSInStyleReferenceBackend.empty())
        throw std::invalid_argument(
            "SInStyle.ReferenceBackend must identify deepflow_cpu or brox_cuda");
    if(mbSInStyleShadowEnabled &&
       mSInStyleReferenceBackend!="deepflow_cpu" &&
       mSInStyleReferenceBackend!="brox_cuda")
    {
        throw std::invalid_argument(
            "SInStyle.ReferenceBackend must be deepflow_cpu or brox_cuda");
    }

    const cv::FileNode sinStyleMaskSuffixNode =
        fSettings["SInStyle.ReferenceMaskSuffix"];
    if(!sinStyleMaskSuffixNode.empty())
        sinStyleMaskSuffixNode >> sinStyleConfig.referenceMaskSuffix;

    const cv::FileNode sinStyleRequireLabelsNode =
        fSettings["SInStyle.RequireLabels"];
    if(!sinStyleRequireLabelsNode.empty())
    {
        sinStyleConfig.requireLabels =
            static_cast<int>(sinStyleRequireLabelsNode)!=0;
    }
    const cv::FileNode sinStyleRegionValidSuffixNode =
        fSettings["SInStyle.ReferenceRegionValidSuffix"];
    if(!sinStyleRegionValidSuffixNode.empty())
    {
        sinStyleRegionValidSuffixNode >>
            sinStyleConfig.referenceRegionValidSuffix;
    }
    const cv::FileNode sinStyleRegionDynamicEnableNode =
        fSettings["SInStyle.RegionDynamicShadowEnable"];
    if(!sinStyleRegionDynamicEnableNode.empty())
    {
        mbSInStyleRegionDynamicEnabled =
            static_cast<int>(sinStyleRegionDynamicEnableNode)!=0;
    }
    mSInStyleRegionDynamicLabelSource = "reference_replay";
    const cv::FileNode sinStyleRegionDynamicLabelSourceNode =
        fSettings["SInStyle.RegionDynamicLabelSource"];
    if(!sinStyleRegionDynamicLabelSourceNode.empty())
    {
        sinStyleRegionDynamicLabelSourceNode >>
            mSInStyleRegionDynamicLabelSource;
    }
    if(mSInStyleRegionDynamicLabelSource!="reference_replay" &&
       mSInStyleRegionDynamicLabelSource!="native_rag")
    {
        throw std::invalid_argument(
            "SInStyle.RegionDynamicLabelSource must be reference_replay "
            "or native_rag");
    }
    const cv::FileNode sinStyleRegionDynamicOutputDirNode =
        fSettings["SInStyle.RegionDynamicOutputDirectory"];
    if(!sinStyleRegionDynamicOutputDirNode.empty())
    {
        sinStyleRegionDynamicOutputDirNode >>
            mSInStyleRegionDynamicOutputDir;
    }
    const char *sinStyleRegionDynamicOutputDirOverride =
        std::getenv("DT_SLAM_SIN_REGION_DYNAMIC_DIR");
    if(sinStyleRegionDynamicOutputDirOverride &&
       sinStyleRegionDynamicOutputDirOverride[0]!='\0')
    {
        mSInStyleRegionDynamicOutputDir =
            sinStyleRegionDynamicOutputDirOverride;
    }
    sinStyleConfig.requireRegionValidity =
        mbSInStyleRegionDynamicEnabled &&
        mSInStyleRegionDynamicLabelSource=="reference_replay";

    const cv::FileNode sinStyleLogEveryNode =
        fSettings["SInStyle.LogEveryN"];
    if(!sinStyleLogEveryNode.empty())
        mnSInStyleLogEveryN = static_cast<int>(sinStyleLogEveryNode);
    if(mnSInStyleLogEveryN<0)
        throw std::invalid_argument("SInStyle.LogEveryN must be non-negative");

    const cv::FileNode sinStyleCsvNode =
        fSettings["SInStyle.CsvPath"];
    if(!sinStyleCsvNode.empty())
        sinStyleCsvNode >> mSInStyleShadowCsvPath;
    const char *sinStyleCsvOverride =
        std::getenv("DT_SLAM_SIN_SHADOW_FRAME_CSV");
    if(sinStyleCsvOverride && sinStyleCsvOverride[0]!='\0')
        mSInStyleShadowCsvPath = sinStyleCsvOverride;

    if(mbSInStyleShadowEnabled && sensor!=System::RGBD)
        throw std::invalid_argument(
            "SInStyle.ShadowEnable=1 requires RGB-D input");
    mSInStyleDetector.Configure(sinStyleConfig);

    SInStyleInitialRegionConfig sinStyleInitialRegionConfig;
    const cv::FileNode sinStyleNativeInitialEnableNode =
        fSettings["SInStyle.NativeInitialRegionsEnable"];
    if(!sinStyleNativeInitialEnableNode.empty())
    {
        mbSInStyleNativeInitialRegionsEnabled =
            static_cast<int>(sinStyleNativeInitialEnableNode)!=0;
    }
    sinStyleInitialRegionConfig.enabled =
        mbSInStyleNativeInitialRegionsEnabled;

    const cv::FileNode sinStyleClusterPixelDivisorNode =
        fSettings["SInStyle.NativeClusterPixelDivisor"];
    if(!sinStyleClusterPixelDivisorNode.empty())
    {
        sinStyleInitialRegionConfig.clusterPixelDivisor =
            static_cast<double>(sinStyleClusterPixelDivisorNode);
    }
    const cv::FileNode sinStyleMaximumDepthNode =
        fSettings["SInStyle.NativeMaximumDepthM"];
    if(!sinStyleMaximumDepthNode.empty())
    {
        sinStyleInitialRegionConfig.maximumDepthMeters =
            static_cast<float>(sinStyleMaximumDepthNode);
    }
    const cv::FileNode sinStyleKMeansMaxIterationsNode =
        fSettings["SInStyle.NativeKMeansMaxIterations"];
    if(!sinStyleKMeansMaxIterationsNode.empty())
    {
        sinStyleInitialRegionConfig.maximumIterations =
            static_cast<int>(sinStyleKMeansMaxIterationsNode);
    }
    const cv::FileNode sinStyleKMeansEpsilonNode =
        fSettings["SInStyle.NativeKMeansEpsilon"];
    if(!sinStyleKMeansEpsilonNode.empty())
    {
        sinStyleInitialRegionConfig.epsilon =
            static_cast<double>(sinStyleKMeansEpsilonNode);
    }
    const cv::FileNode sinStyleKMeansAttemptsNode =
        fSettings["SInStyle.NativeKMeansAttempts"];
    if(!sinStyleKMeansAttemptsNode.empty())
    {
        sinStyleInitialRegionConfig.attempts =
            static_cast<int>(sinStyleKMeansAttemptsNode);
    }
    const cv::FileNode sinStyleKMeansSeedNode =
        fSettings["SInStyle.NativeKMeansSeed"];
    if(!sinStyleKMeansSeedNode.empty())
    {
        const double seedValue =
            static_cast<double>(sinStyleKMeansSeedNode);
        if(!std::isfinite(seedValue) || seedValue<0.0)
            throw std::invalid_argument(
                "SInStyle.NativeKMeansSeed must be non-negative");
        sinStyleInitialRegionConfig.randomSeed =
            static_cast<std::uint64_t>(seedValue);
    }
    const cv::FileNode sinStyleCoarseToFineNode =
        fSettings["SInStyle.NativeCoarseToFine"];
    if(!sinStyleCoarseToFineNode.empty())
    {
        sinStyleInitialRegionConfig.coarseToFine =
            static_cast<int>(sinStyleCoarseToFineNode)!=0;
    }
    const cv::FileNode sinStylePyramidLevelsNode =
        fSettings["SInStyle.NativePyramidLevels"];
    if(!sinStylePyramidLevelsNode.empty())
    {
        sinStyleInitialRegionConfig.pyramidLevels =
            static_cast<int>(sinStylePyramidLevelsNode);
    }
    const cv::FileNode sinStyleTemporalInitializationNode =
        fSettings["SInStyle.NativeTemporalInitialization"];
    if(!sinStyleTemporalInitializationNode.empty())
    {
        sinStyleInitialRegionConfig.temporalInitialization =
            static_cast<int>(sinStyleTemporalInitializationNode)!=0;
    }
    const cv::FileNode sinStyleTemporalCommitStartNode =
        fSettings["SInStyle.NativeTemporalCommitStartInputIndex"];
    if(!sinStyleTemporalCommitStartNode.empty())
    {
        const double startIndex =
            static_cast<double>(sinStyleTemporalCommitStartNode);
        if(!std::isfinite(startIndex) || startIndex<0.0)
            throw std::invalid_argument(
                "SInStyle.NativeTemporalCommitStartInputIndex must be non-negative");
        sinStyleInitialRegionConfig.temporalCommitStartInputIndex =
            static_cast<long unsigned int>(startIndex);
    }
    const cv::FileNode sinStyleNativeInitialOutputDirNode =
        fSettings["SInStyle.NativeInitialOutputDirectory"];
    if(!sinStyleNativeInitialOutputDirNode.empty())
    {
        sinStyleNativeInitialOutputDirNode >>
            mSInStyleNativeInitialOutputDir;
    }
    const char *sinStyleNativeInitialOutputDirOverride =
        std::getenv("DT_SLAM_SIN_NATIVE_INITIAL_DIR");
    if(sinStyleNativeInitialOutputDirOverride &&
       sinStyleNativeInitialOutputDirOverride[0]!='\0')
    {
        mSInStyleNativeInitialOutputDir =
            sinStyleNativeInitialOutputDirOverride;
    }
    if(mbSInStyleNativeInitialRegionsEnabled && !mbSInStyleShadowEnabled)
        throw std::invalid_argument(
            "SInStyle.NativeInitialRegionsEnable=1 requires SInStyle.ShadowEnable=1");
    mSInStyleInitialRegionClusterer.Configure(
        sinStyleInitialRegionConfig,mK);

    SInStyleGradientSplitConfig sinStyleGradientSplitConfig;
    sinStyleGradientSplitConfig.maximumDepthMeters =
        sinStyleInitialRegionConfig.maximumDepthMeters;
    const cv::FileNode sinStyleGradientSplitEnableNode =
        fSettings["SInStyle.NativeGradientSplitEnable"];
    if(!sinStyleGradientSplitEnableNode.empty())
    {
        mbSInStyleNativeGradientSplitEnabled =
            static_cast<int>(sinStyleGradientSplitEnableNode)!=0;
    }
    sinStyleGradientSplitConfig.enabled =
        mbSInStyleNativeGradientSplitEnabled;
    const cv::FileNode sinStyleGradientRelativeThresholdNode =
        fSettings["SInStyle.NativeGradientRelativeThreshold"];
    if(!sinStyleGradientRelativeThresholdNode.empty())
    {
        sinStyleGradientSplitConfig.relativeThreshold =
            static_cast<float>(sinStyleGradientRelativeThresholdNode);
    }
    const cv::FileNode sinStyleGradientAbsoluteThresholdNode =
        fSettings["SInStyle.NativeGradientAbsoluteThresholdM"];
    if(!sinStyleGradientAbsoluteThresholdNode.empty())
    {
        sinStyleGradientSplitConfig.absoluteThresholdMeters =
            static_cast<float>(sinStyleGradientAbsoluteThresholdNode);
    }
    const cv::FileNode sinStyleGradientMedianRadiusNode =
        fSettings["SInStyle.NativeGradientMedianRadius"];
    if(!sinStyleGradientMedianRadiusNode.empty())
    {
        sinStyleGradientSplitConfig.medianRadius =
            static_cast<int>(sinStyleGradientMedianRadiusNode);
    }
    const cv::FileNode sinStyleGradientMinimumSupportNode =
        fSettings["SInStyle.NativeGradientMinimumSupport"];
    if(!sinStyleGradientMinimumSupportNode.empty())
    {
        sinStyleGradientSplitConfig.minimumMedianSupport =
            static_cast<int>(sinStyleGradientMinimumSupportNode);
    }
    const cv::FileNode sinStyleGradientConnectivityNode =
        fSettings["SInStyle.NativeGradientConnectivity"];
    if(!sinStyleGradientConnectivityNode.empty())
    {
        sinStyleGradientSplitConfig.connectivity =
            static_cast<int>(sinStyleGradientConnectivityNode);
    }
    const cv::FileNode sinStyleGradientSmallComponentNode =
        fSettings["SInStyle.NativeGradientSmallComponentAuditPixels"];
    if(!sinStyleGradientSmallComponentNode.empty())
    {
        const double smallComponentPixels =
            static_cast<double>(sinStyleGradientSmallComponentNode);
        if(!std::isfinite(smallComponentPixels) || smallComponentPixels<0.0)
            throw std::invalid_argument(
                "SInStyle.NativeGradientSmallComponentAuditPixels "
                "must be non-negative");
        sinStyleGradientSplitConfig.smallComponentAuditPixels =
            static_cast<std::size_t>(smallComponentPixels);
    }
    const cv::FileNode sinStyleGradientOutputDirNode =
        fSettings["SInStyle.NativeGradientOutputDirectory"];
    if(!sinStyleGradientOutputDirNode.empty())
        sinStyleGradientOutputDirNode >> mSInStyleNativeGradientOutputDir;
    const char *sinStyleGradientOutputDirOverride =
        std::getenv("DT_SLAM_SIN_NATIVE_GRADIENT_DIR");
    if(sinStyleGradientOutputDirOverride &&
       sinStyleGradientOutputDirOverride[0]!='\0')
    {
        mSInStyleNativeGradientOutputDir =
            sinStyleGradientOutputDirOverride;
    }
    if(mbSInStyleNativeGradientSplitEnabled &&
       !mbSInStyleNativeInitialRegionsEnabled)
    {
        throw std::invalid_argument(
            "SInStyle.NativeGradientSplitEnable=1 requires "
            "SInStyle.NativeInitialRegionsEnable=1");
    }
    mSInStyleGradientRegionSplitter.Configure(
        sinStyleGradientSplitConfig);

    SInStylePlaneEdgeSplitConfig sinStylePlaneEdgeConfig;
    sinStylePlaneEdgeConfig.maximumDepthMeters =
        sinStyleInitialRegionConfig.maximumDepthMeters;
    const cv::FileNode sinStylePlaneEdgeEnableNode =
        fSettings["SInStyle.NativePlaneEdgeEnable"];
    if(!sinStylePlaneEdgeEnableNode.empty())
    {
        mbSInStyleNativePlaneEdgeEnabled =
            static_cast<int>(sinStylePlaneEdgeEnableNode)!=0;
    }
    sinStylePlaneEdgeConfig.enabled = mbSInStyleNativePlaneEdgeEnabled;
#define READ_SIN_PLANE_INT(KEY, FIELD) \
    do { \
        const cv::FileNode node = fSettings[KEY]; \
        if(!node.empty()) sinStylePlaneEdgeConfig.FIELD = \
            static_cast<int>(node); \
    } while(false)
#define READ_SIN_PLANE_DOUBLE(KEY, FIELD) \
    do { \
        const cv::FileNode node = fSettings[KEY]; \
        if(!node.empty()) sinStylePlaneEdgeConfig.FIELD = \
            static_cast<double>(node); \
    } while(false)
    READ_SIN_PLANE_INT("SInStyle.NativePlaneBlockSize",blockSize);
    READ_SIN_PLANE_INT(
        "SInStyle.NativePlaneMinimumPixels",minimumPlanePixels);
    READ_SIN_PLANE_DOUBLE(
        "SInStyle.NativePlaneDistanceThresholdM",
        distanceThresholdMeters);
    READ_SIN_PLANE_DOUBLE(
        "SInStyle.NativePlaneSensorErrorA",sensorErrorA);
    READ_SIN_PLANE_DOUBLE(
        "SInStyle.NativePlaneSensorErrorB",sensorErrorB);
    READ_SIN_PLANE_DOUBLE(
        "SInStyle.NativePlaneSensorErrorC",sensorErrorC);
    READ_SIN_PLANE_INT(
        "SInStyle.NativePlaneEndpointRadius",endpointRadius);
    READ_SIN_PLANE_INT(
        "SInStyle.NativePlaneEndpointMaximumSupportExclusive",
        endpointMaximumSupportExclusive);
    READ_SIN_PLANE_INT(
        "SInStyle.NativePlaneEndpointAssociationRadius",
        endpointAssociationRadius);
    READ_SIN_PLANE_INT(
        "SInStyle.NativePlaneMinimumEndpointCountExclusive",
        minimumEndpointCountExclusive);
    READ_SIN_PLANE_INT(
        "SInStyle.NativePlaneConnectivity",connectivity);
#undef READ_SIN_PLANE_INT
#undef READ_SIN_PLANE_DOUBLE
    const cv::FileNode sinStylePlaneOutputDirNode =
        fSettings["SInStyle.NativePlaneOutputDirectory"];
    if(!sinStylePlaneOutputDirNode.empty())
        sinStylePlaneOutputDirNode >> mSInStyleNativePlaneOutputDir;
    const char *sinStylePlaneOutputDirOverride =
        std::getenv("DT_SLAM_SIN_NATIVE_PLANE_DIR");
    if(sinStylePlaneOutputDirOverride &&
       sinStylePlaneOutputDirOverride[0]!='\0')
    {
        mSInStyleNativePlaneOutputDir =
            sinStylePlaneOutputDirOverride;
    }
    if(mbSInStyleNativePlaneEdgeEnabled &&
       !mbSInStyleNativeGradientSplitEnabled)
    {
        throw std::invalid_argument(
            "SInStyle.NativePlaneEdgeEnable=1 requires "
            "SInStyle.NativeGradientSplitEnable=1");
    }
    mSInStylePlaneEdgeRegionSplitter.Configure(
        sinStylePlaneEdgeConfig,mK);

    SInStyleRAGMergeConfig sinStyleRAGConfig;
    sinStyleRAGConfig.maximumDepthMeters =
        sinStyleInitialRegionConfig.maximumDepthMeters;
    const cv::FileNode sinStyleRAGEnableNode =
        fSettings["SInStyle.NativeRAGMergeEnable"];
    if(!sinStyleRAGEnableNode.empty())
        mbSInStyleNativeRAGMergeEnabled =
            static_cast<int>(sinStyleRAGEnableNode)!=0;
    sinStyleRAGConfig.enabled = mbSInStyleNativeRAGMergeEnabled;
#define READ_SIN_RAG_FLOAT(KEY, FIELD) \
    do { \
        const cv::FileNode node = fSettings[KEY]; \
        if(!node.empty()) sinStyleRAGConfig.FIELD = static_cast<float>(node); \
    } while(false)
#define READ_SIN_RAG_INT(KEY, FIELD) \
    do { \
        const cv::FileNode node = fSettings[KEY]; \
        if(!node.empty()) sinStyleRAGConfig.FIELD = static_cast<int>(node); \
    } while(false)
    READ_SIN_RAG_INT(
        "SInStyle.NativeRAGAdjacencyDilationRadius",
        adjacencyDilationRadius);
    READ_SIN_RAG_INT("SInStyle.NativeRAGHistogramBins",histogramBins);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGAdjacencyThresholdPixels",
        adjacencyThresholdPixels);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGAreaDepthScoreWeight",areaDepthScoreWeight);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGFakeEdgeWeight",fakeEdgeWeight);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGLargeRegionWeight",largeRegionWeight);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGMiddleRegionWeight",middleRegionWeight);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGSmallRegionWeight",smallRegionWeight);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGMergeThreshold",mergeThreshold);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGDepthRejectThreshold",depthRejectThreshold);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGHighMiddleFraction",highMiddleFraction);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGLargeFraction",largeFraction);
    READ_SIN_RAG_FLOAT(
        "SInStyle.NativeRAGSmallFraction",smallFraction);
#undef READ_SIN_RAG_FLOAT
#undef READ_SIN_RAG_INT
    const cv::FileNode sinStyleRAGOutputDirNode =
        fSettings["SInStyle.NativeRAGOutputDirectory"];
    if(!sinStyleRAGOutputDirNode.empty())
        sinStyleRAGOutputDirNode >> mSInStyleNativeRAGOutputDir;
    const char *sinStyleRAGOutputDirOverride =
        std::getenv("DT_SLAM_SIN_NATIVE_RAG_DIR");
    if(sinStyleRAGOutputDirOverride &&
       sinStyleRAGOutputDirOverride[0]!='\0')
    {
        mSInStyleNativeRAGOutputDir = sinStyleRAGOutputDirOverride;
    }
    if(mbSInStyleNativeRAGMergeEnabled &&
       !mbSInStyleNativeGradientSplitEnabled)
    {
        throw std::invalid_argument(
            "SInStyle.NativeRAGMergeEnable=1 requires "
            "SInStyle.NativeGradientSplitEnable=1");
    }
    mSInStyleRAGRegionMerger.Configure(sinStyleRAGConfig);

    SInStyleDenseFlowResidualConfig sinStyleDenseFlowConfig;
    const cv::FileNode sinStyleDenseFlowEnableNode =
        fSettings["SInStyle.DenseFlowResidualEnable"];
    if(!sinStyleDenseFlowEnableNode.empty())
    {
        mbSInStyleDenseFlowResidualEnabled =
            static_cast<int>(sinStyleDenseFlowEnableNode)!=0;
    }
    sinStyleDenseFlowConfig.enabled = mbSInStyleDenseFlowResidualEnabled;
    const cv::FileNode sinStyleDenseFlowBackendNode =
        fSettings["SInStyle.DenseFlowResidualBackend"];
    if(!sinStyleDenseFlowBackendNode.empty())
        sinStyleDenseFlowBackendNode >> sinStyleDenseFlowConfig.backend;
    const cv::FileNode sinStyleDenseFlowDirectoryNode =
        fSettings["SInStyle.DenseFlowResidualReferenceDirectory"];
    if(!sinStyleDenseFlowDirectoryNode.empty())
    {
        sinStyleDenseFlowDirectoryNode >>
            sinStyleDenseFlowConfig.referenceDirectory;
    }
    const char *sinStyleDenseFlowDirectoryOverride =
        std::getenv("DT_SLAM_SIN_DENSE_FLOW_REFERENCE_DIR");
    if(sinStyleDenseFlowDirectoryOverride &&
       sinStyleDenseFlowDirectoryOverride[0]!='\0')
    {
        sinStyleDenseFlowConfig.referenceDirectory =
            sinStyleDenseFlowDirectoryOverride;
    }
    const cv::FileNode sinStyleDenseFlowRequireNode =
        fSettings["SInStyle.DenseFlowResidualRequireReference"];
    if(!sinStyleDenseFlowRequireNode.empty())
    {
        sinStyleDenseFlowConfig.requireReference =
            static_cast<int>(sinStyleDenseFlowRequireNode)!=0;
    }
    const cv::FileNode sinStyleDenseFlowTemporalPriorNode =
        fSettings["SInStyle.DenseFlowUseTemporalRegionPrior"];
    if(!sinStyleDenseFlowTemporalPriorNode.empty())
    {
        sinStyleDenseFlowConfig.useTemporalRegionPrior =
            static_cast<int>(sinStyleDenseFlowTemporalPriorNode)!=0;
    }
    if(mbSInStyleDenseFlowResidualEnabled && !mbSInStyleShadowEnabled)
    {
        throw std::invalid_argument(
            "SInStyle.DenseFlowResidualEnable=1 requires "
            "SInStyle.ShadowEnable=1");
    }
    mSInStyleDenseFlowResidualEstimator.Configure(
        sinStyleDenseFlowConfig);

    SInStyleRegionDynamicConfig sinStyleRegionDynamicConfig;
    sinStyleRegionDynamicConfig.enabled = mbSInStyleRegionDynamicEnabled;
    if(mbSInStyleRegionDynamicEnabled &&
       !mbSInStyleDenseFlowResidualEnabled)
    {
        throw std::invalid_argument(
            "SInStyle.RegionDynamicShadowEnable=1 requires "
            "SInStyle.DenseFlowResidualEnable=1");
    }
    if(mbSInStyleRegionDynamicEnabled &&
       mSInStyleRegionDynamicLabelSource=="native_rag" &&
       !mbSInStyleNativeRAGMergeEnabled)
    {
        throw std::invalid_argument(
            "SInStyle.RegionDynamicLabelSource=native_rag requires "
            "SInStyle.NativeRAGMergeEnable=1");
    }
    if(sinStyleDenseFlowConfig.useTemporalRegionPrior &&
       !mbSInStyleRegionDynamicEnabled)
    {
        throw std::invalid_argument(
            "SInStyle.DenseFlowUseTemporalRegionPrior=1 requires "
            "SInStyle.RegionDynamicShadowEnable=1");
    }
    const cv::FileNode sinStyleRegionFeatureFilterEnableNode =
        fSettings["SInStyle.RegionFeatureFilterEnable"];
    if(!sinStyleRegionFeatureFilterEnableNode.empty())
    {
        mbSInStyleRegionFeatureFilterEnabled =
            static_cast<int>(sinStyleRegionFeatureFilterEnableNode)!=0;
    }
    const char *sinStyleRegionFeatureFilterOverride =
        std::getenv("DT_SLAM_SIN_REGION_FEATURE_FILTER");
    if(sinStyleRegionFeatureFilterOverride &&
       sinStyleRegionFeatureFilterOverride[0]!='\0')
    {
        mbSInStyleRegionFeatureFilterEnabled =
            std::string(sinStyleRegionFeatureFilterOverride)!="0";
    }
    const cv::FileNode sinStyleRegionFeatureMinimumNode =
        fSettings[
            "SInStyle.RegionFeatureFilterMinimumRemainingFeatures"];
    if(!sinStyleRegionFeatureMinimumNode.empty())
    {
        mnSInStyleRegionFeatureFilterMinimumRemainingFeatures =
            static_cast<int>(sinStyleRegionFeatureMinimumNode);
    }
    const char *sinStyleRegionFeatureMinimumOverride =
        std::getenv("DT_SLAM_SIN_REGION_FEATURE_MIN_REMAINING");
    if(sinStyleRegionFeatureMinimumOverride &&
       sinStyleRegionFeatureMinimumOverride[0]!='\0')
    {
        char *end = NULL;
        errno = 0;
        const long parsed = std::strtol(
            sinStyleRegionFeatureMinimumOverride,&end,10);
        if(errno!=0 || end==sinStyleRegionFeatureMinimumOverride ||
           *end!='\0' || parsed<1 || parsed>INT_MAX)
        {
            throw std::invalid_argument(
                "DT_SLAM_SIN_REGION_FEATURE_MIN_REMAINING must be a "
                "positive integer");
        }
        mnSInStyleRegionFeatureFilterMinimumRemainingFeatures =
            static_cast<int>(parsed);
    }
    if(mnSInStyleRegionFeatureFilterMinimumRemainingFeatures<1)
    {
        throw std::invalid_argument(
            "SInStyle.RegionFeatureFilterMinimumRemainingFeatures "
            "must be positive");
    }
    if(mbSInStyleRegionFeatureFilterEnabled &&
       (!mbSInStyleRegionDynamicEnabled ||
        mSInStyleRegionDynamicLabelSource!="native_rag"))
    {
        throw std::invalid_argument(
            "SInStyle.RegionFeatureFilterEnable=1 requires native_rag "
            "region dynamic shadow");
    }
    mSInStyleRegionDynamicClassifier.Configure(
        sinStyleRegionDynamicConfig);

    SInStyleDepthFilterConfig sinStyleDepthFilterConfig;
    const cv::FileNode sinStyleDepthFilterEnableNode =
        fSettings["SInStyle.DepthFilterEnable"];
    if(!sinStyleDepthFilterEnableNode.empty())
    {
        mbSInStyleDepthFilterEnabled =
            static_cast<int>(sinStyleDepthFilterEnableNode)!=0;
    }
    const char *sinStyleDepthFilterEnableOverride =
        std::getenv("DT_SLAM_SIN_DEPTH_FILTER");
    if(sinStyleDepthFilterEnableOverride &&
       sinStyleDepthFilterEnableOverride[0]!='\0')
    {
        mbSInStyleDepthFilterEnabled =
            std::string(sinStyleDepthFilterEnableOverride)!="0";
    }
    const cv::FileNode sinStyleDepthFilterModeNode =
        fSettings["SInStyle.DepthFilterMaskMode"];
    if(!sinStyleDepthFilterModeNode.empty())
        sinStyleDepthFilterModeNode >> mSInStyleDepthFilterMaskMode;
    const char *sinStyleDepthFilterModeOverride =
        std::getenv("DT_SLAM_SIN_DEPTH_FILTER_MODE");
    if(sinStyleDepthFilterModeOverride &&
       sinStyleDepthFilterModeOverride[0]!='\0')
    {
        mSInStyleDepthFilterMaskMode =
            sinStyleDepthFilterModeOverride;
    }
    const cv::FileNode sinStyleDepthFilterCsvNode =
        fSettings["SInStyle.DepthFilterCsvPath"];
    if(!sinStyleDepthFilterCsvNode.empty())
        sinStyleDepthFilterCsvNode >> mSInStyleDepthFilterCsvPath;
    const char *sinStyleDepthFilterCsvOverride =
        std::getenv("DT_SLAM_SIN_DEPTH_FILTER_CSV");
    if(sinStyleDepthFilterCsvOverride &&
       sinStyleDepthFilterCsvOverride[0]!='\0')
    {
        mSInStyleDepthFilterCsvPath = sinStyleDepthFilterCsvOverride;
    }
    const cv::FileNode sinStyleDepthFilterOutputNode =
        fSettings["SInStyle.DepthFilterOutputDirectory"];
    if(!sinStyleDepthFilterOutputNode.empty())
        sinStyleDepthFilterOutputNode >> mSInStyleDepthFilterOutputDir;
    const char *sinStyleDepthFilterOutputOverride =
        std::getenv("DT_SLAM_SIN_DEPTH_FILTER_OUTPUT_DIR");
    if(sinStyleDepthFilterOutputOverride &&
       sinStyleDepthFilterOutputOverride[0]!='\0')
    {
        mSInStyleDepthFilterOutputDir =
            sinStyleDepthFilterOutputOverride;
    }
    if(mbSInStyleDepthFilterEnabled && sensor!=System::RGBD)
    {
        throw std::invalid_argument(
            "SInStyle.DepthFilterEnable=1 requires RGB-D input");
    }
    if(mbSInStyleDepthFilterEnabled &&
       mSInStyleDepthFilterMaskMode=="geometry_only" &&
       !mbSInStyleRegionDynamicEnabled)
    {
        throw std::invalid_argument(
            "SInStyle.DepthFilterMaskMode=geometry_only requires the "
            "SIn-style region dynamic detector");
    }
    sinStyleDepthFilterConfig.enabled = mbSInStyleDepthFilterEnabled;
    sinStyleDepthFilterConfig.maskMode = mSInStyleDepthFilterMaskMode;
    mSInStyleDepthFilter.Configure(sinStyleDepthFilterConfig);

    if(mbSInStyleShadowEnabled)
    {
        cout << endl
             << "[SIn S1] shadow-only region reference enabled" << endl
             << "[SIn S1] backend: " << sinStyleConfig.backend << endl
             << "[SIn S1] reference backend: "
             << mSInStyleReferenceBackend << endl
             << "[SIn S1] reference directory: "
             << sinStyleConfig.referenceDirectory << endl
             << "[SIn S1] native initial regions: "
             << mbSInStyleNativeInitialRegionsEnabled << endl
             << "[SIn S1] native coarse-to-fine: "
             << sinStyleInitialRegionConfig.coarseToFine << endl
             << "[SIn S1] native gradient split: "
             << mbSInStyleNativeGradientSplitEnabled << endl
             << "[SIn S1] OpenCV plane-edge substitute: "
             << mbSInStyleNativePlaneEdgeEnabled << endl
             << "[SIn S1] native RAG merge: "
             << mbSInStyleNativeRAGMergeEnabled << endl
             << "- dense flow residual replay: "
             << mbSInStyleDenseFlowResidualEnabled << endl
             << "[SIn S1] region dynamic classifier shadow: "
             << mbSInStyleRegionDynamicEnabled << endl
             << "[SIn S1] region dynamic label source: "
             << mSInStyleRegionDynamicLabelSource << endl
             << "[SIn S2] region feature filter: "
             << mbSInStyleRegionFeatureFilterEnabled << endl
             << "[SIn S2] minimum remaining features: "
             << mnSInStyleRegionFeatureFilterMinimumRemainingFeatures
             << endl
             << "[SIn S3] mapping-side dynamic depth filter: "
             << mbSInStyleDepthFilterEnabled << endl
             << "[SIn S3] mapping mask mode: "
             << mSInStyleDepthFilterMaskMode << endl
             << "[SIn S1] native initial output directory: "
             << (mSInStyleNativeInitialOutputDir.empty() ?
                    "disabled" : mSInStyleNativeInitialOutputDir) << endl
             << "[SIn S1] native gradient output directory: "
             << (mSInStyleNativeGradientOutputDir.empty() ?
                    "disabled" : mSInStyleNativeGradientOutputDir) << endl
             << "[SIn S1] native plane output directory: "
             << (mSInStyleNativePlaneOutputDir.empty() ?
                    "disabled" : mSInStyleNativePlaneOutputDir) << endl
             << "[SIn S1] native RAG output directory: "
             << (mSInStyleNativeRAGOutputDir.empty() ?
                    "disabled" : mSInStyleNativeRAGOutputDir) << endl
             << "[SIn S1] region dynamic output directory: "
             << (mSInStyleRegionDynamicOutputDir.empty() ?
                    "disabled" : mSInStyleRegionDynamicOutputDir) << endl
             << "[SIn S1] dynamic_decision=shadow_only, "
             << "direct_slam_state_mutation=none" << endl;
    }
    else if(mbSInStyleDepthFilterEnabled)
    {
        cout << endl
             << "[SIn S3] mapping-side dynamic depth filter enabled"
             << endl
             << "[SIn S3] mapping mask mode: "
             << mSInStyleDepthFilterMaskMode << endl
             << "[SIn S3] tracking_state_mutation=none" << endl;
    }

    const cv::FileNode geometryEnableNode = fSettings["Geometry.Enable"];
    if(!geometryEnableNode.empty())
        mbGeometryShadowEnabled = static_cast<int>(geometryEnableNode)!=0;

    const cv::FileNode geometrySingleReferenceEnableNode =
        fSettings["Geometry.SingleReferenceShadowEnable"];
    if(!geometrySingleReferenceEnableNode.empty())
    {
        mbGeometrySingleReferenceShadowEnabled =
            static_cast<int>(geometrySingleReferenceEnableNode)!=0;
    }

    const cv::FileNode geometryMultiReferenceEnableNode =
        fSettings["Geometry.MultiReferenceShadowEnable"];
    if(!geometryMultiReferenceEnableNode.empty())
    {
        mbGeometryMultiReferenceShadowEnabled =
            static_cast<int>(geometryMultiReferenceEnableNode)!=0;
    }
    if(mbGeometryMultiReferenceShadowEnabled &&
       !mbGeometryShadowEnabled)
    {
        throw std::invalid_argument(
            "Geometry.MultiReferenceShadowEnable=1 requires "
            "Geometry.Enable=1");
    }

    const cv::FileNode geometrySparseEgoFlowEnableNode =
        fSettings["Geometry.SparseEgoFlowShadowEnable"];
    if(!geometrySparseEgoFlowEnableNode.empty())
    {
        mbGeometrySparseEgoFlowShadowEnabled =
            static_cast<int>(
                geometrySparseEgoFlowEnableNode)!=0;
    }
    if(mbGeometrySparseEgoFlowShadowEnabled &&
       (!mbGeometryShadowEnabled || sensor!=System::RGBD))
    {
        throw std::invalid_argument(
            "Geometry.SparseEgoFlowShadowEnable=1 requires "
            "Geometry.Enable=1 and RGB-D input");
    }
    const cv::FileNode geometrySparseFlowCounterfactualNode =
        fSettings[
            "Geometry.SparseFlowCounterfactualShadowEnable"];
    if(!geometrySparseFlowCounterfactualNode.empty())
    {
        mbGeometrySparseFlowCounterfactualShadowEnabled =
            static_cast<int>(
                geometrySparseFlowCounterfactualNode)!=0;
    }
    const char *geometrySparseFlowCounterfactualOverride =
        std::getenv(
            "DT_SLAM_GEOMETRY_ASSOCIATION_COUNTERFACTUAL");
    if(geometrySparseFlowCounterfactualOverride &&
       geometrySparseFlowCounterfactualOverride[0]!='\0')
    {
        mbGeometrySparseFlowCounterfactualShadowEnabled =
            std::string(
                geometrySparseFlowCounterfactualOverride)!="0";
    }
    if(mbGeometrySparseFlowCounterfactualShadowEnabled &&
       !mbGeometrySparseEgoFlowShadowEnabled)
    {
        throw std::invalid_argument(
            "Geometry.SparseFlowCounterfactualShadowEnable=1 "
            "requires Geometry.SparseEgoFlowShadowEnable=1");
    }
    const cv::FileNode geometrySparseFlowTrackingFilterNode =
        fSettings["Geometry.SparseFlowTrackingFilterEnable"];
    if(!geometrySparseFlowTrackingFilterNode.empty())
    {
        mbGeometrySparseFlowTrackingFilterEnabled =
            static_cast<int>(
                geometrySparseFlowTrackingFilterNode)!=0;
    }
    const char *geometrySparseFlowTrackingFilterOverride =
        std::getenv("DT_SLAM_GEOMETRY_TRACKING_FILTER");
    if(geometrySparseFlowTrackingFilterOverride &&
       geometrySparseFlowTrackingFilterOverride[0]!='\0')
    {
        mbGeometrySparseFlowTrackingFilterEnabled =
            std::string(
                geometrySparseFlowTrackingFilterOverride)!="0";
    }
    const cv::FileNode geometrySparseFlowTrackingFilterQNode =
        fSettings["Geometry.SparseFlowTrackingFilterQ"];
    if(!geometrySparseFlowTrackingFilterQNode.empty())
    {
        mfGeometrySparseFlowTrackingFilterQ =
            static_cast<float>(
                geometrySparseFlowTrackingFilterQNode);
    }
    const char *geometrySparseFlowTrackingFilterQOverride =
        std::getenv("DT_SLAM_GEOMETRY_TRACKING_FILTER_Q");
    if(geometrySparseFlowTrackingFilterQOverride &&
       geometrySparseFlowTrackingFilterQOverride[0]!='\0')
    {
        std::size_t parsedCharacters = 0;
        try
        {
            mfGeometrySparseFlowTrackingFilterQ =
                std::stof(
                    geometrySparseFlowTrackingFilterQOverride,
                    &parsedCharacters);
        }
        catch(const std::exception &)
        {
            throw std::invalid_argument(
                "DT_SLAM_GEOMETRY_TRACKING_FILTER_Q "
                "must be 6, 8, or 10");
        }
        if(parsedCharacters!=
           std::string(
               geometrySparseFlowTrackingFilterQOverride).size())
        {
            throw std::invalid_argument(
                "DT_SLAM_GEOMETRY_TRACKING_FILTER_Q "
                "must be 6, 8, or 10");
        }
    }
    const bool supportedTrackingFilterQ =
        std::abs(mfGeometrySparseFlowTrackingFilterQ-6.0f)<1e-6f ||
        std::abs(mfGeometrySparseFlowTrackingFilterQ-8.0f)<1e-6f ||
        std::abs(mfGeometrySparseFlowTrackingFilterQ-10.0f)<1e-6f;
    if(!supportedTrackingFilterQ)
    {
        throw std::invalid_argument(
            "Geometry.SparseFlowTrackingFilterQ must be 6, 8, or 10");
    }
    const cv::FileNode geometrySparseFlowTrackingFilterMaximumNode =
        fSettings[
            "Geometry.SparseFlowTrackingFilterMaximumAssociationFraction"];
    if(!geometrySparseFlowTrackingFilterMaximumNode.empty())
    {
        mfGeometrySparseFlowTrackingFilterMaximumAssociationFraction =
            static_cast<float>(
                geometrySparseFlowTrackingFilterMaximumNode);
    }
    const cv::FileNode geometrySparseFlowTrackingFilterMinimumNode =
        fSettings[
            "Geometry.SparseFlowTrackingFilterMinimumAssociations"];
    if(!geometrySparseFlowTrackingFilterMinimumNode.empty())
    {
        mnGeometrySparseFlowTrackingFilterMinimumAssociations =
            static_cast<int>(
                geometrySparseFlowTrackingFilterMinimumNode);
    }
    const cv::FileNode geometrySparseFlowTrackingFilterScaleSupportNode =
        fSettings[
            "Geometry.SparseFlowTrackingFilterMinimumScaleSupport"];
    if(!geometrySparseFlowTrackingFilterScaleSupportNode.empty())
    {
        const int configuredScaleSupport =
            static_cast<int>(
                geometrySparseFlowTrackingFilterScaleSupportNode);
        if(configuredScaleSupport<=0)
        {
            throw std::invalid_argument(
                "Geometry.SparseFlowTrackingFilterMinimumScaleSupport "
                "must be positive");
        }
        mnGeometrySparseFlowTrackingFilterMinimumScaleSupport =
            static_cast<std::size_t>(configuredScaleSupport);
    }
    if(mbGeometrySparseFlowTrackingFilterEnabled &&
       (!mbGeometrySparseEgoFlowShadowEnabled ||
        sensor!=System::RGBD))
    {
        throw std::invalid_argument(
            "Geometry.SparseFlowTrackingFilterEnable=1 requires "
            "Geometry.SparseEgoFlowShadowEnable=1 and RGB-D input");
    }
    if(mbGeometrySparseFlowTrackingFilterEnabled &&
       mbGeometrySparseFlowCounterfactualShadowEnabled)
    {
        throw std::invalid_argument(
            "G1-F1 tracking filter and G1-F0B counterfactual "
            "snapshots cannot be enabled together");
    }
    if(!std::isfinite(
           mfGeometrySparseFlowTrackingFilterMaximumAssociationFraction) ||
       mfGeometrySparseFlowTrackingFilterMaximumAssociationFraction<=0.0f ||
       mfGeometrySparseFlowTrackingFilterMaximumAssociationFraction>1.0f ||
       mnGeometrySparseFlowTrackingFilterMinimumAssociations<1)
    {
        throw std::invalid_argument(
            "G1-F1 association safeguards are invalid");
    }
    const cv::FileNode geometrySparseFlowMappingCounterfactualNode =
        fSettings[
            "Geometry.SparseFlowMappingCounterfactualEnable"];
    if(!geometrySparseFlowMappingCounterfactualNode.empty())
    {
        mbGeometrySparseFlowMappingCounterfactualEnabled =
            static_cast<int>(
                geometrySparseFlowMappingCounterfactualNode)!=0;
    }
    const char *geometrySparseFlowMappingCounterfactualOverride =
        std::getenv(
            "DT_SLAM_GEOMETRY_MAPPING_COUNTERFACTUAL");
    if(geometrySparseFlowMappingCounterfactualOverride &&
       geometrySparseFlowMappingCounterfactualOverride[0]!='\0')
    {
        mbGeometrySparseFlowMappingCounterfactualEnabled =
            std::string(
                geometrySparseFlowMappingCounterfactualOverride)!="0";
    }
    if(mbGeometrySparseFlowMappingCounterfactualEnabled &&
       (!mbGeometrySparseFlowTrackingFilterEnabled ||
        std::abs(
            mfGeometrySparseFlowTrackingFilterQ-10.0f)>1e-6f ||
        sensor!=System::RGBD))
    {
        throw std::invalid_argument(
            "G1-M0 mapping counterfactual requires RGB-D and "
            "G1-F1 tracking filter q10");
    }
    const cv::FileNode geometrySparseFlowMappingFilterNode =
        fSettings[
            "Geometry.SparseFlowMappingFilterEnable"];
    if(!geometrySparseFlowMappingFilterNode.empty())
    {
        mbGeometrySparseFlowMappingFilterEnabled =
            static_cast<int>(
                geometrySparseFlowMappingFilterNode)!=0;
    }
    const char *geometrySparseFlowMappingFilterOverride =
        std::getenv("DT_SLAM_GEOMETRY_MAPPING_FILTER");
    if(geometrySparseFlowMappingFilterOverride &&
       geometrySparseFlowMappingFilterOverride[0]!='\0')
    {
        mbGeometrySparseFlowMappingFilterEnabled =
            std::string(
                geometrySparseFlowMappingFilterOverride)!="0";
    }
    const cv::FileNode mappingMaximumFeatureFractionNode =
        fSettings[
            "Geometry.SparseFlowMappingFilterMaximumFeatureFraction"];
    if(!mappingMaximumFeatureFractionNode.empty())
    {
        mfGeometrySparseFlowMappingFilterMaximumFeatureFraction =
            static_cast<float>(
                mappingMaximumFeatureFractionNode);
    }
    const cv::FileNode mappingMaximumDepthFractionNode =
        fSettings[
            "Geometry.SparseFlowMappingFilterMaximumDepthFraction"];
    if(!mappingMaximumDepthFractionNode.empty())
    {
        mfGeometrySparseFlowMappingFilterMaximumDepthFraction =
            static_cast<float>(
                mappingMaximumDepthFractionNode);
    }
    const cv::FileNode mappingMinimumRemainingDepthNode =
        fSettings[
            "Geometry.SparseFlowMappingFilterMinimumRemainingDepthFeatures"];
    if(!mappingMinimumRemainingDepthNode.empty())
    {
        const int minimumRemainingDepth =
            static_cast<int>(
                mappingMinimumRemainingDepthNode);
        if(minimumRemainingDepth<1)
        {
            throw std::invalid_argument(
                "Geometry.SparseFlowMappingFilterMinimumRemainingDepthFeatures "
                "must be positive");
        }
        mnGeometrySparseFlowMappingFilterMinimumRemainingDepthFeatures =
            static_cast<std::size_t>(
                minimumRemainingDepth);
    }
    if(!std::isfinite(
           mfGeometrySparseFlowMappingFilterMaximumFeatureFraction) ||
       mfGeometrySparseFlowMappingFilterMaximumFeatureFraction<=0.0f ||
       mfGeometrySparseFlowMappingFilterMaximumFeatureFraction>1.0f ||
       !std::isfinite(
           mfGeometrySparseFlowMappingFilterMaximumDepthFraction) ||
       mfGeometrySparseFlowMappingFilterMaximumDepthFraction<=0.0f ||
       mfGeometrySparseFlowMappingFilterMaximumDepthFraction>1.0f)
    {
        throw std::invalid_argument(
            "G1-M1 mapping safeguard fractions are invalid");
    }
    if(mbGeometrySparseFlowMappingFilterEnabled &&
       (!mbGeometrySparseFlowTrackingFilterEnabled ||
        std::abs(
            mfGeometrySparseFlowTrackingFilterQ-10.0f)>1e-6f ||
        sensor!=System::RGBD))
    {
        throw std::invalid_argument(
            "G1-M1 mapping filter requires RGB-D and "
            "G1-F1 tracking filter q10");
    }
    if(mbGeometrySparseFlowMappingFilterEnabled &&
       mbGeometrySparseFlowMappingCounterfactualEnabled)
    {
        throw std::invalid_argument(
            "G1-M0 counterfactual and G1-M1 mapping filter "
            "cannot be enabled together");
    }
    const cv::FileNode mapQualityAuditNode =
        fSettings[
            "Geometry.SparseFlowMapQualityAuditEnable"];
    if(!mapQualityAuditNode.empty())
    {
        mbGeometrySparseFlowMapQualityAuditEnabled =
            static_cast<int>(mapQualityAuditNode)!=0;
    }
    const char *mapQualityAuditOverride =
        std::getenv("DT_SLAM_GEOMETRY_MAP_QUALITY_AUDIT");
    if(mapQualityAuditOverride &&
       mapQualityAuditOverride[0]!='\0')
    {
        mbGeometrySparseFlowMapQualityAuditEnabled =
            std::string(mapQualityAuditOverride)!="0";
    }
    if(mbGeometrySparseFlowMapQualityAuditEnabled &&
       (!mbGeometrySparseFlowTrackingFilterEnabled ||
        std::abs(
            mfGeometrySparseFlowTrackingFilterQ-10.0f)>1e-6f ||
        sensor!=System::RGBD ||
        (mbGeometrySparseFlowMappingCounterfactualEnabled==
         mbGeometrySparseFlowMappingFilterEnabled)))
    {
        throw std::invalid_argument(
            "map-quality audit requires RGB-D, G1-F1 q10, "
            "and exactly one of G1-M0 or G1-M1");
    }
    const cv::FileNode geometryLocalRigidityEnableNode =
        fSettings["Geometry.LocalRigidityShadowEnable"];
    if(!geometryLocalRigidityEnableNode.empty())
    {
        mbGeometryLocalRigidityShadowEnabled =
            static_cast<int>(
                geometryLocalRigidityEnableNode)!=0;
    }
    if(mbGeometryLocalRigidityShadowEnabled &&
       !mbGeometrySparseEgoFlowShadowEnabled)
    {
        throw std::invalid_argument(
            "Geometry.LocalRigidityShadowEnable=1 requires "
            "Geometry.SparseEgoFlowShadowEnable=1");
    }
    const cv::FileNode geometryRigidHypothesisEnableNode =
        fSettings["Geometry.RigidHypothesisShadowEnable"];
    if(!geometryRigidHypothesisEnableNode.empty())
    {
        mbGeometryRigidHypothesisShadowEnabled =
            static_cast<int>(
                geometryRigidHypothesisEnableNode)!=0;
    }
    if(mbGeometryRigidHypothesisShadowEnabled &&
       !mbGeometryLocalRigidityShadowEnabled)
    {
        throw std::invalid_argument(
            "Geometry.RigidHypothesisShadowEnable=1 requires "
            "Geometry.LocalRigidityShadowEnable=1");
    }

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
    const cv::FileNode rgbdInputRectificationNode =
        fSettings["RGBD.InputRectification.Enable"];
    const bool rgbdInputRectificationEnabled =
        !rgbdInputRectificationNode.empty() &&
        static_cast<int>(rgbdInputRectificationNode)!=0;
    if(rgbdInputRectificationEnabled &&
       cv::norm(mDistCoef,cv::NORM_INF)>1e-8)
    {
        throw std::invalid_argument(
            "RGBD.InputRectification.Enable=1 requires zero Camera "
            "distortion in Tracking");
    }
    if(rgbdInputRectificationEnabled &&
       mbGeometryUsesDedicatedCameraModel &&
       cv::norm(mGeometryK-mK,cv::NORM_INF)>1e-6)
    {
        throw std::invalid_argument(
            "Rectified RGB-D input requires Geometry.Camera K to match "
            "the tracking Camera K");
    }
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

    const cv::FileNode geometryMultiReferenceCountNode =
        fSettings["Geometry.MultiReferenceCount"];
    if(!geometryMultiReferenceCountNode.empty())
    {
        mnGeometryMultiReferenceMaxReferences =
            static_cast<int>(geometryMultiReferenceCountNode);
    }
    const cv::FileNode geometryMultiReferenceHistoryNode =
        fSettings["Geometry.MultiReferenceHistory"];
    if(!geometryMultiReferenceHistoryNode.empty())
    {
        mnGeometryMultiReferenceHistorySize =
            static_cast<int>(geometryMultiReferenceHistoryNode);
    }
    const cv::FileNode geometryMultiReferencePolicyNode =
        fSettings["Geometry.MultiReferenceSelectionPolicy"];
    if(!geometryMultiReferencePolicyNode.empty())
    {
        geometryMultiReferencePolicyNode >>
            mGeometryMultiReferenceSelectionPolicy;
    }
    if(mGeometryMultiReferenceSelectionPolicy!="recent" &&
       mGeometryMultiReferenceSelectionPolicy!="covisibility")
    {
        throw std::invalid_argument(
            "Geometry.MultiReferenceSelectionPolicy must be "
            "'recent' or 'covisibility'");
    }
    const cv::FileNode geometryMultiReferenceSamplingNode =
        fSettings["Geometry.MultiReferenceSamplingPolicy"];
    if(!geometryMultiReferenceSamplingNode.empty())
    {
        geometryMultiReferenceSamplingNode >>
            mGeometryMultiReferenceSamplingPolicy;
    }
    if(mGeometryMultiReferenceSamplingPolicy!="dense" &&
       mGeometryMultiReferenceSamplingPolicy!="orb_depth" &&
       mGeometryMultiReferenceSamplingPolicy!="grid_depth" &&
       mGeometryMultiReferenceSamplingPolicy!="pyramid_dense")
    {
        throw std::invalid_argument(
            "Geometry.MultiReferenceSamplingPolicy must be "
            "'dense', 'orb_depth', 'grid_depth', or "
            "'pyramid_dense'");
    }
    const cv::FileNode geometryMultiReferenceGridStrideNode =
        fSettings["Geometry.MultiReferenceGridStride"];
    if(!geometryMultiReferenceGridStrideNode.empty())
    {
        mnGeometryMultiReferenceGridStride =
            static_cast<int>(geometryMultiReferenceGridStrideNode);
    }
    const char *gridStrideOverride =
        std::getenv("DT_SLAM_GEOMETRY_GRID_STRIDE");
    if(gridStrideOverride && gridStrideOverride[0]!='\0')
    {
        mnGeometryMultiReferenceGridStride =
            std::atoi(gridStrideOverride);
    }
    if(mnGeometryMultiReferenceGridStride<1 ||
       mnGeometryMultiReferenceGridStride>64)
    {
        throw std::invalid_argument(
            "Geometry.MultiReferenceGridStride must be in [1,64]");
    }
    const cv::FileNode geometryMultiReferencePyramidScaleNode =
        fSettings["Geometry.MultiReferencePyramidScale"];
    if(!geometryMultiReferencePyramidScaleNode.empty())
    {
        mnGeometryMultiReferencePyramidScale =
            static_cast<int>(
                geometryMultiReferencePyramidScaleNode);
    }
    if(mnGeometryMultiReferencePyramidScale!=2)
    {
        throw std::invalid_argument(
            "Geometry.MultiReferencePyramidScale currently supports "
            "only scale 2");
    }
    const char *denseSamplingAudit =
        std::getenv("DT_SLAM_GEOMETRY_DENSE_SAMPLING_AUDIT");
    if(denseSamplingAudit && denseSamplingAudit[0]!='\0' &&
       std::string(denseSamplingAudit)!="0")
    {
        mbGeometryMultiReferenceDenseAuditEnabled = true;
    }
    if(mbGeometryMultiReferenceDenseAuditEnabled &&
       mGeometryMultiReferenceSamplingPolicy=="dense")
    {
        throw std::invalid_argument(
            "DT_SLAM_GEOMETRY_DENSE_SAMPLING_AUDIT requires "
            "a non-dense Geometry.MultiReferenceSamplingPolicy");
    }
    const cv::FileNode geometryRegionEvidenceEnableNode =
        fSettings["Geometry.RegionEvidenceShadowEnable"];
    if(!geometryRegionEvidenceEnableNode.empty())
    {
        mbGeometryRegionEvidenceShadowEnabled =
            static_cast<int>(
                geometryRegionEvidenceEnableNode)!=0;
    }
    const cv::FileNode geometryRegionRiskDiagnosticsNode =
        fSettings["Geometry.RegionRiskDiagnosticsEnable"];
    if(!geometryRegionRiskDiagnosticsNode.empty())
    {
        mbGeometryRegionRiskDiagnosticsEnabled =
            static_cast<int>(
                geometryRegionRiskDiagnosticsNode)!=0;
    }
    const char *regionRiskDiagnosticsOverride =
        std::getenv(
            "DT_SLAM_GEOMETRY_REGION_RISK_DIAGNOSTICS");
    if(regionRiskDiagnosticsOverride &&
       regionRiskDiagnosticsOverride[0]!='\0')
    {
        mbGeometryRegionRiskDiagnosticsEnabled =
            std::string(regionRiskDiagnosticsOverride)!="0";
    }
    const cv::FileNode geometryLowResolutionRegionEnableNode =
        fSettings["Geometry.LowResolutionRegionShadowEnable"];
    if(!geometryLowResolutionRegionEnableNode.empty())
    {
        mbGeometryLowResolutionRegionShadowEnabled =
            static_cast<int>(
                geometryLowResolutionRegionEnableNode)!=0;
    }
    const cv::FileNode geometryRegionRelativeThresholdNode =
        fSettings["Geometry.RegionPartitionRelativeThreshold"];
    if(!geometryRegionRelativeThresholdNode.empty())
    {
        mGeometryRegionRelativeThreshold =
            static_cast<float>(
                geometryRegionRelativeThresholdNode);
    }
    const cv::FileNode geometryRegionAbsoluteThresholdNode =
        fSettings["Geometry.RegionPartitionAbsoluteThresholdM"];
    if(!geometryRegionAbsoluteThresholdNode.empty())
    {
        mGeometryRegionAbsoluteThresholdMeters =
            static_cast<float>(
                geometryRegionAbsoluteThresholdNode);
    }
    if(!std::isfinite(mGeometryRegionRelativeThreshold) ||
       mGeometryRegionRelativeThreshold<0.0f ||
       !std::isfinite(
           mGeometryRegionAbsoluteThresholdMeters) ||
       mGeometryRegionAbsoluteThresholdMeters<=0.0f)
    {
        throw std::invalid_argument(
            "Geometry region partition thresholds must be finite; "
            "relative must be non-negative and absolute must be positive");
    }
    if(mbGeometryRegionEvidenceShadowEnabled &&
       !mbGeometryMultiReferenceShadowEnabled)
    {
        throw std::invalid_argument(
            "Geometry.RegionEvidenceShadowEnable=1 requires "
            "Geometry.MultiReferenceShadowEnable=1");
    }
    if(mbGeometryRegionRiskDiagnosticsEnabled &&
       !mbGeometryRegionEvidenceShadowEnabled)
    {
        throw std::invalid_argument(
            "Geometry.RegionRiskDiagnosticsEnable=1 requires "
            "Geometry.RegionEvidenceShadowEnable=1");
    }
    if(mbGeometryLowResolutionRegionShadowEnabled &&
       (!mbGeometryRegionEvidenceShadowEnabled ||
        mGeometryMultiReferenceSamplingPolicy!="pyramid_dense"))
    {
        throw std::invalid_argument(
            "Geometry.LowResolutionRegionShadowEnable=1 requires "
            "Geometry.RegionEvidenceShadowEnable=1 and "
            "Geometry.MultiReferenceSamplingPolicy=pyramid_dense");
    }
    if(mnGeometryMultiReferenceMaxReferences<1 ||
       mnGeometryMultiReferenceMaxReferences>255)
    {
        throw std::invalid_argument(
            "Geometry.MultiReferenceCount must be in [1,255]");
    }
    if(mnGeometryMultiReferenceHistorySize<
       mnGeometryMultiReferenceMaxReferences)
    {
        throw std::invalid_argument(
            "Geometry.MultiReferenceHistory must be at least "
            "Geometry.MultiReferenceCount");
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

    const char *multiReferenceCsv =
        std::getenv("DT_SLAM_GEOMETRY_MULTIREF_CSV");
    if(multiReferenceCsv && multiReferenceCsv[0]!='\0')
        mGeometryMultiReferenceCsvPath = multiReferenceCsv;
    const char *multiReferenceFeatureCsv =
        std::getenv("DT_SLAM_GEOMETRY_MULTIREF_FEATURE_CSV");
    if(multiReferenceFeatureCsv &&
       multiReferenceFeatureCsv[0]!='\0')
    {
        mGeometryMultiReferenceFeatureCsvPath =
            multiReferenceFeatureCsv;
        mGeometryMultiReferenceFeatureFrameFilter =
            ParseFrameIdFilter(
                std::getenv(
                    "DT_SLAM_GEOMETRY_MULTIREF_FEATURE_FRAME_IDS"));
    }
    const char *sparseFlowCsv =
        std::getenv("DT_SLAM_GEOMETRY_SPARSE_FLOW_CSV");
    if(sparseFlowCsv && sparseFlowCsv[0]!='\0')
    {
        mGeometrySparseFlowCsvPath = sparseFlowCsv;
        mGeometrySparseFlowFrameFilter =
            ParseFrameIdFilter(
                std::getenv(
                    "DT_SLAM_GEOMETRY_SPARSE_FLOW_FRAME_IDS"));
    }
    const char *associationSnapshotCsv =
        std::getenv(
            "DT_SLAM_GEOMETRY_ASSOCIATION_SNAPSHOT_CSV");
    if(associationSnapshotCsv &&
       associationSnapshotCsv[0]!='\0')
    {
        mGeometryAssociationSnapshotCsvPath =
            associationSnapshotCsv;
    }
    if(mbGeometrySparseFlowCounterfactualShadowEnabled &&
       (mGeometrySparseFlowCsvPath.empty() ||
        mGeometryAssociationSnapshotCsvPath.empty()))
    {
        throw std::invalid_argument(
            "Geometry.SparseFlowCounterfactualShadowEnable=1 "
            "requires DT_SLAM_GEOMETRY_SPARSE_FLOW_CSV and "
            "DT_SLAM_GEOMETRY_ASSOCIATION_SNAPSHOT_CSV");
    }
    const char *trackingFilterCsv =
        std::getenv("DT_SLAM_GEOMETRY_TRACKING_FILTER_CSV");
    if(trackingFilterCsv && trackingFilterCsv[0]!='\0')
    {
        mGeometrySparseFlowTrackingFilterCsvPath =
            trackingFilterCsv;
    }
    if(mbGeometrySparseFlowTrackingFilterEnabled &&
       mGeometrySparseFlowTrackingFilterCsvPath.empty())
    {
        throw std::invalid_argument(
            "G1-F1 tracking filter requires "
            "DT_SLAM_GEOMETRY_TRACKING_FILTER_CSV");
    }
    const char *removedAssociationCsv =
        std::getenv(
            "DT_SLAM_GEOMETRY_TRACKING_FILTER_FEATURE_CSV");
    if(removedAssociationCsv && removedAssociationCsv[0]!='\0')
    {
        mGeometrySparseFlowRemovedAssociationCsvPath =
            removedAssociationCsv;
    }
    const char *candidateAssociationCsv =
        std::getenv(
            "DT_SLAM_GEOMETRY_TRACKING_FILTER_CANDIDATE_CSV");
    if(candidateAssociationCsv && candidateAssociationCsv[0]!='\0')
    {
        mGeometrySparseFlowCandidateAssociationCsvPath =
            candidateAssociationCsv;
    }
    const char *mappingCounterfactualCsv =
        std::getenv(
            "DT_SLAM_GEOMETRY_MAPPING_COUNTERFACTUAL_CSV");
    if(mappingCounterfactualCsv &&
       mappingCounterfactualCsv[0]!='\0')
    {
        mGeometrySparseFlowMappingCounterfactualCsvPath =
            mappingCounterfactualCsv;
    }
    if(mbGeometrySparseFlowMappingCounterfactualEnabled &&
       mGeometrySparseFlowMappingCounterfactualCsvPath.empty())
    {
        throw std::invalid_argument(
            "G1-M0 mapping counterfactual requires "
            "DT_SLAM_GEOMETRY_MAPPING_COUNTERFACTUAL_CSV");
    }
    const char *mappingFilterCsv =
        std::getenv("DT_SLAM_GEOMETRY_MAPPING_FILTER_CSV");
    if(mappingFilterCsv && mappingFilterCsv[0]!='\0')
    {
        mGeometrySparseFlowMappingFilterCsvPath =
            mappingFilterCsv;
    }
    if(mbGeometrySparseFlowMappingFilterEnabled &&
       mGeometrySparseFlowMappingFilterCsvPath.empty())
    {
        throw std::invalid_argument(
            "G1-M1 mapping filter requires "
            "DT_SLAM_GEOMETRY_MAPPING_FILTER_CSV");
    }
    const char *mapQualityPrefix =
        std::getenv("DT_SLAM_GEOMETRY_MAP_QUALITY_PREFIX");
    if(mapQualityPrefix && mapQualityPrefix[0]!='\0')
        mGeometrySparseFlowMapQualityPrefix = mapQualityPrefix;
    if(mbGeometrySparseFlowMapQualityAuditEnabled &&
       mGeometrySparseFlowMapQualityPrefix.empty())
    {
        throw std::invalid_argument(
            "map-quality audit requires "
            "DT_SLAM_GEOMETRY_MAP_QUALITY_PREFIX");
    }
    const char *localRigidityCsv =
        std::getenv("DT_SLAM_GEOMETRY_LOCAL_RIGIDITY_CSV");
    if(localRigidityCsv && localRigidityCsv[0]!='\0')
    {
        mGeometryLocalRigidityCsvPath = localRigidityCsv;
        mGeometryLocalRigidityFrameFilter =
            ParseFrameIdFilter(
                std::getenv(
                    "DT_SLAM_GEOMETRY_LOCAL_RIGIDITY_FRAME_IDS"));
    }
    const char *rigidHypothesisCsv =
        std::getenv("DT_SLAM_GEOMETRY_RIGID_HYPOTHESIS_CSV");
    if(rigidHypothesisCsv && rigidHypothesisCsv[0]!='\0')
    {
        mGeometryRigidHypothesisCsvPath = rigidHypothesisCsv;
        mGeometryRigidHypothesisFrameFilter =
            ParseFrameIdFilter(
                std::getenv(
                    "DT_SLAM_GEOMETRY_RIGID_HYPOTHESIS_FRAME_IDS"));
    }
    const char *referenceSelectionCsv =
        std::getenv("DT_SLAM_GEOMETRY_REFERENCE_SELECTION_CSV");
    if(referenceSelectionCsv && referenceSelectionCsv[0]!='\0')
        mGeometryReferenceSelectionCsvPath = referenceSelectionCsv;
    const char *regionEvidenceCsv =
        std::getenv("DT_SLAM_GEOMETRY_REGION_EVIDENCE_CSV");
    if(regionEvidenceCsv && regionEvidenceCsv[0]!='\0')
        mGeometryRegionEvidenceCsvPath = regionEvidenceCsv;
    const char *multiReferenceDebugDir =
        std::getenv("DT_SLAM_GEOMETRY_MULTIREF_DEBUG_DIR");
    if(multiReferenceDebugDir &&
       multiReferenceDebugDir[0]!='\0')
    {
        mGeometryMultiReferenceDebugOutputDir =
            multiReferenceDebugDir;
    }

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
    const char *jiDebugEveryEnvironment =
        std::getenv("DT_SLAM_JI_DEBUG_EVERY_N");
    if(jiDebugEveryEnvironment &&
       jiDebugEveryEnvironment[0]!='\0')
    {
        char *end = static_cast<char*>(NULL);
        const long parsed = std::strtol(
            jiDebugEveryEnvironment,&end,10);
        if(!end || end==jiDebugEveryEnvironment || *end!='\0' ||
           parsed<1 || parsed>std::numeric_limits<int>::max())
        {
            throw std::invalid_argument(
                "DT_SLAM_JI_DEBUG_EVERY_N must be a positive integer");
        }
        mnJiGeometryDebugEveryN = static_cast<int>(parsed);
    }
    const char *jiDebugRawLabelsOnlyEnvironment =
        std::getenv("DT_SLAM_JI_DEBUG_RAW_LABELS_ONLY");
    if(jiDebugRawLabelsOnlyEnvironment &&
       std::string(jiDebugRawLabelsOnlyEnvironment)=="1")
    {
        mbJiGeometryDebugRawLabelsOnly = true;
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
        cout << "- G0 single-reference shadow: "
             << (mbGeometrySingleReferenceShadowEnabled ?
                 "enabled" : "disabled") << endl;
        cout << "- pixel domain: "
             << (rgbdInputRectificationEnabled ?
                 "jointly rectified RGB/registered-depth input pixels" :
                 "raw registered RGB/depth pixels")
             << endl;
        cout << "- camera model: "
             << (rgbdInputRectificationEnabled ?
                 "shared rectified tracking/geometry pinhole K" :
                 (mbGeometryUsesDedicatedCameraModel ?
                 "dedicated raw registered pinhole K" :
                 "tracking pinhole K"))
             << " with zero distortion" << endl;
        cout << "- geometry fx: " << mGeometryK.at<float>(0,0) << endl;
        cout << "- geometry fy: " << mGeometryK.at<float>(1,1) << endl;
        cout << "- geometry cx: " << mGeometryK.at<float>(0,2) << endl;
        cout << "- geometry cy: " << mGeometryK.at<float>(1,2) << endl;
        cout << "- semantic/feature labeling domain: Frame::mvKeys"
             << (rgbdInputRectificationEnabled ?
                 " in rectified input" : "") << endl;
        cout << "- optimizer feature domain: Frame::mvKeysUn"
             << (rgbdInputRectificationEnabled ?
                 " (same pixels because Camera distortion is zero)" : "")
             << endl;
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
        cout << "- G2-4F1 sparse ego-flow shadow: "
             << (mbGeometrySparseEgoFlowShadowEnabled
                     ? "enabled" : "disabled") << endl;
        cout << "- G2-4F3 local rigidity shadow: "
             << (mbGeometryLocalRigidityShadowEnabled
                     ? "enabled" : "disabled") << endl;
        cout << "- G2-MH1 local 3-D rigid hypotheses: "
             << (mbGeometryRigidHypothesisShadowEnabled
                     ? "enabled" : "disabled") << endl;
        if(mbGeometryRigidHypothesisShadowEnabled)
        {
            cout << "- G2-MH1 local point count: 7" << endl;
            cout << "- motion-group independent local validation "
                 << "point count: 7" << endl;
            cout << "- G2-MH1 output: continuous hypothesis fit; "
                 << "dynamic_decision=none; "
                 << "direct_slam_state_mutation=none" << endl;
            if(!mGeometryRigidHypothesisCsvPath.empty())
            {
                cout << "- G2-MH1 diagnostics: "
                     << mGeometryRigidHypothesisCsvPath << endl;
            }
        }
        cout << "- G1-F0B association counterfactual snapshots: "
             << (mbGeometrySparseFlowCounterfactualShadowEnabled
                     ? "enabled" : "disabled") << endl;
        if(mbGeometrySparseFlowCounterfactualShadowEnabled)
        {
            cout << "- G1-F0B output: raw association state only; "
                 << "dynamic_decision=none; "
                 << "direct_slam_state_mutation=none" << endl;
        }
        cout << "- G1-F1 sparse-flow tracking association filter: "
             << (mbGeometrySparseFlowTrackingFilterEnabled
                     ? "enabled" : "disabled") << endl;
        if(mbGeometrySparseFlowTrackingFilterEnabled)
        {
            cout << "- G1-F1 q threshold: "
                 << mfGeometrySparseFlowTrackingFilterQ << endl;
            cout << "- G1-F1 maximum association removal fraction: "
                 << mfGeometrySparseFlowTrackingFilterMaximumAssociationFraction
                 << endl;
            cout << "- G1-F1 minimum remaining associations: "
                 << mnGeometrySparseFlowTrackingFilterMinimumAssociations
                 << endl;
            cout << "- G1-F1 minimum scale support: "
                 << mnGeometrySparseFlowTrackingFilterMinimumScaleSupport
                 << endl;
            cout << "- G1-F1 output: "
                 << mGeometrySparseFlowTrackingFilterCsvPath << endl;
            if(!mGeometrySparseFlowRemovedAssociationCsvPath.empty())
            {
                cout << "- G1-F1 exact removed-association audit: "
                     << mGeometrySparseFlowRemovedAssociationCsvPath
                     << endl;
            }
            if(!mGeometrySparseFlowCandidateAssociationCsvPath.empty())
            {
                cout << "- G1-F1 post-search candidate-association audit: "
                     << mGeometrySparseFlowCandidateAssociationCsvPath
                     << endl;
            }
            cout << "- G1-F1 scope: TrackLocalMap associations only; "
                 << "pose_reoptimization=none; mapping_veto=none"
                 << endl;
        }
        cout << "- G1-M0 MapPoint admission counterfactual: "
             << (mbGeometrySparseFlowMappingCounterfactualEnabled
                     ? "enabled" : "disabled") << endl;
        if(mbGeometrySparseFlowMappingCounterfactualEnabled)
        {
            cout << "- G1-M0 output: "
                 << mGeometrySparseFlowMappingCounterfactualCsvPath
                 << endl;
            cout << "- G1-M0 scope: read-only initialization/"
                 << "CreateNewKeyFrame audit; mapping_veto=none"
                 << endl;
        }
        cout << "- G1-M1 sparse-flow MapPoint admission filter: "
             << (mbGeometrySparseFlowMappingFilterEnabled
                     ? "enabled" : "disabled") << endl;
        if(mbGeometrySparseFlowMappingFilterEnabled)
        {
            cout << "- G1-M1 maximum candidate feature fraction: "
                 << mfGeometrySparseFlowMappingFilterMaximumFeatureFraction
                 << endl;
            cout << "- G1-M1 maximum candidate depth fraction: "
                 << mfGeometrySparseFlowMappingFilterMaximumDepthFraction
                 << endl;
            cout << "- G1-M1 minimum remaining depth features: "
                 << mnGeometrySparseFlowMappingFilterMinimumRemainingDepthFeatures
                 << endl;
            cout << "- G1-M1 output: "
                 << mGeometrySparseFlowMappingFilterCsvPath
                 << endl;
            cout << "- G1-M1 scope: existing mvbDynamic/"
                 << "MapPoint admission path; pose_reoptimization=none"
                 << endl;
        }
        cout << "- G1 sparse-map quality audit: "
             << (mbGeometrySparseFlowMapQualityAuditEnabled
                     ? "enabled" : "disabled") << endl;
        if(mbGeometrySparseFlowMapQualityAuditEnabled)
        {
            cout << "- map-quality output prefix: "
                 << mGeometrySparseFlowMapQualityPrefix << endl;
            cout << "- map-quality scope: read-only candidate "
                 << "MapPoint lifecycle/final-map summary" << endl;
        }
        cout << "- G2-1 multi-reference evidence: "
             << (mbGeometryMultiReferenceShadowEnabled ?
                 "enabled" : "disabled") << endl;
        if(mbGeometryMultiReferenceShadowEnabled)
        {
            cout << "- G2 reference policy: "
                 << mGeometryMultiReferenceSelectionPolicy << endl;
            cout << "- G2 reference depth sampling: "
                 << mGeometryMultiReferenceSamplingPolicy << endl;
            if(mGeometryMultiReferenceSamplingPolicy=="grid_depth")
            {
                cout << "- G2 regular-grid stride: "
                     << mnGeometryMultiReferenceGridStride
                     << " pixels" << endl;
            }
            if(mGeometryMultiReferenceSamplingPolicy=="pyramid_dense")
            {
                cout << "- G2 boundary-preserving pyramid scale: "
                     << mnGeometryMultiReferencePyramidScale
                     << endl;
            }
            cout << "- G2 sampling same-reference dense audit: "
                 << (mbGeometryMultiReferenceDenseAuditEnabled
                         ? "enabled" : "disabled") << endl;
            cout << "- G2 requested references: "
                 << mnGeometryMultiReferenceMaxReferences
                 << " from cached history "
                 << mnGeometryMultiReferenceHistorySize << endl;
            cout << "- G2-1 output: raw comparison/evidence counts; "
                 << "dynamic decision=none" << endl;
            if(!mGeometryMultiReferenceCsvPath.empty())
            {
                 cout << "- G2-1 histogram diagnostics: "
                     << mGeometryMultiReferenceCsvPath << endl;
            }
            if(!mGeometryReferenceSelectionCsvPath.empty())
            {
                cout << "- G2-2R reference selection diagnostics: "
                     << mGeometryReferenceSelectionCsvPath << endl;
            }
            if(!mGeometryMultiReferenceDebugOutputDir.empty())
            {
                cout << "- G2-1 raw count images: "
                     << mGeometryMultiReferenceDebugOutputDir << endl;
            }
            cout << "- G2-3R1 region evidence aggregation: "
                 << (mbGeometryRegionEvidenceShadowEnabled
                         ? "enabled" : "disabled") << endl;
            if(mbGeometryRegionEvidenceShadowEnabled)
            {
                cout << "- G2-4A region risk diagnostics: "
                     << (mbGeometryRegionRiskDiagnosticsEnabled
                             ? "enabled" : "disabled") << endl;
                cout << "- G2-3R4 low-resolution region candidate: "
                     << (mbGeometryLowResolutionRegionShadowEnabled
                             ? "enabled" : "disabled") << endl;
                cout << "- G2-3R1 depth boundary thresholds: relative="
                     << mGeometryRegionRelativeThreshold
                     << ", absolute="
                     << mGeometryRegionAbsoluteThresholdMeters
                     << " m" << endl;
                cout << "- G2-3R1 output: per-region evidence "
                     << "distributions; dynamic decision=none" << endl;
                if(!mGeometryRegionEvidenceCsvPath.empty())
                {
                    cout << "- G2-3R1 region diagnostics: "
                         << mGeometryRegionEvidenceCsvPath << endl;
                }
            }
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
            cout << "- debug output: "
                 << (mbJiGeometryDebugRawLabelsOnly
                         ? "raw labels only" : "labels and visualization")
                 << endl;
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

const std::vector<unsigned char>&
Tracking::GetCurrentSparseFlowRemovedAssociations() const
{
    return mvbCurrentSparseFlowRemovedAssociations;
}

void Tracking::SaveSInStyleShadowDiagnostics()
{
    if(mSInStyleShadowCsvPath.empty() ||
       mvSInStyleShadowDiagnostics.empty())
    {
        return;
    }

    ofstream stream(mSInStyleShadowCsvPath.c_str());
    if(!stream.is_open())
    {
        cerr << "[SIn S1] failed to open shadow CSV: "
             << mSInStyleShadowCsvPath << endl;
        return;
    }

    stream << "input_index,frame,reset_epoch,timestamp,backend,"
           << "region_dynamic_label_source,"
           << "reference_available,labels_available,"
           << "pixels,depth_valid_pixels,raw_unknown_pixels,raw_static_pixels,"
           << "raw_dynamic_pixels,valid_pixels,static_pixels,"
           << "dynamic_pixels,unknown_pixels,positive_label_count,"
           << "positive_label_pixels,depth_supported_positive_label_pixels,"
           << "positive_label_component_count,"
           << "author_dynamic_pixels_on_positive_labels,"
           << "author_dynamic_pixels_on_label_zero,"
           << "author_dynamic_pixels_with_labels_unavailable,raw_orb_count,"
           << "author_dynamic_mask_hit_on_dt_orb_set,"
           << "depth_supported_dynamic_orb_count,"
           << "valid_orb_count,unknown_orb_count,"
           << "semantic_dynamic_orb_count,"
           << "semantic_author_dynamic_overlap_on_dt_orb_set,"
           << "would_keep_orb_count,counterfactual_fallback_on_dt_orb_set,"
           << "counterfactual_removed_on_dt_orb_set,actual_slam_removed,"
           << "tracking_state_after,load_ms,state_conversion_ms,"
           << "region_statistics_ms,total_ms,dynamic_decision,"
           << "direct_slam_state_mutation,"
           << "dense_flow_enabled,dense_flow_available,"
           << "dense_flow_dynamic_state_available,"
           << "dense_flow_failure_reason,dense_flow_backend,"
           << "dense_flow_frame_index,dense_flow_intended_reference_lag,"
           << "dense_flow_reference_index,dense_flow_actual_reference_lag,"
           << "dense_flow_large_motion,dense_flow_image_scale,"
           << "dense_flow_homography_samples,dense_flow_max_flow_px,"
           << "dense_flow_temporal_prior_used,"
           << "dense_flow_temporal_unknown_samples,"
           << "dense_flow_temporal_static_samples,"
           << "dense_flow_temporal_dynamic_samples,"
           << "dense_flow_max_residual_px,dense_flow_otsu_threshold_u8,"
           << "dense_flow_triangle_threshold_u8,"
           << "dense_flow_low_threshold_u8,"
           << "dense_flow_high_threshold_u8,dense_flow_low_threshold_px,"
           << "dense_flow_high_threshold_px,dense_flow_low_pixels,"
           << "dense_flow_high_pixels,dense_flow_residual_recompute_error_px,"
           << "dense_flow_normalized_recompute_error,dense_flow_load_ms,"
           << "dense_flow_validate_ms,dense_flow_total_ms,"
           << "dense_flow_dynamic_decision,"
           << "dense_flow_direct_slam_state_mutation,"
           << "native_initial_enabled,native_initial_available,"
           << "native_dynamic_state_available,native_image_pixels,"
           << "native_input_depth_valid_pixels,"
           << "native_clustering_depth_valid_pixels,"
           << "native_excluded_far_depth_pixels,"
           << "native_requested_clusters,native_produced_clusters,"
           << "native_smallest_region_pixels,"
           << "native_largest_region_pixels,native_compactness,"
           << "native_prepare_ms,native_kmeans_ms,"
           << "native_label_conversion_ms,native_total_ms,"
           << "native_initial_orb_assigned_count,"
           << "native_initial_labels_written,native_coarse_to_fine,"
           << "native_pyramid_levels,native_initialization_source,"
           << "native_previous_prior_samples,native_grid_fallback_samples,"
           << "native_previous_prior_coverage,"
           << "native_temporal_prior_committed,native_level_shapes,"
           << "native_level_valid_samples,native_level_prior_samples,"
           << "native_level_grid_fallback_samples,"
           << "native_level_compactness,native_level_prepare_ms,"
           << "native_level_kmeans_ms,native_level_label_ms,"
           << "native_gradient_enabled,native_gradient_available,"
           << "native_gradient_dynamic_state_available,"
           << "native_gradient_image_pixels,"
           << "native_gradient_input_depth_valid_pixels,"
           << "native_gradient_initial_region_pixels,"
           << "native_gradient_median_valid_pixels,"
           << "native_gradient_insufficient_support_pixels,"
           << "native_gradient_raw_edge_pixels,"
           << "native_gradient_split_boundary_pixels,"
           << "native_gradient_split_core_pixels,"
           << "native_gradient_initial_region_count,"
           << "native_gradient_split_component_count,"
           << "native_gradient_split_initial_region_count,"
           << "native_gradient_fully_consumed_initial_region_count,"
           << "native_gradient_median_fragmentation,"
           << "native_gradient_maximum_fragmentation,"
           << "native_gradient_small_component_count,"
           << "native_gradient_small_component_pixels,"
           << "native_gradient_median_filter_ms,"
           << "native_gradient_edge_ms,native_gradient_components_ms,"
           << "native_gradient_total_ms,native_gradient_edge_written,"
           << "native_gradient_split_labels_written,"
           << "native_gradient_dynamic_decision,"
           << "native_plane_enabled,native_plane_available,"
           << "native_plane_dynamic_state_available,"
           << "native_plane_opencv_substitute,"
           << "native_plane_image_pixels,"
           << "native_plane_input_depth_valid_pixels,"
           << "native_plane_initial_region_pixels,"
           << "native_plane_pixels,native_plane_count,"
           << "native_plane_raw_boundary_pixels,"
           << "native_plane_gradient_overlap_pixels,"
           << "native_plane_candidate_boundary_pixels,"
           << "native_plane_gradient_endpoint_pixels,"
           << "native_plane_boundary_segment_count,"
           << "native_plane_retained_segment_count,"
           << "native_plane_unsupported_segment_count,"
           << "native_plane_retained_boundary_pixels,"
           << "native_plane_combined_edge_pixels,"
           << "native_plane_combined_core_pixels,"
           << "native_plane_initial_region_count,"
           << "native_plane_combined_component_count,"
           << "native_plane_split_initial_region_count,"
           << "native_plane_fully_consumed_initial_region_count,"
           << "native_plane_maximum_fragmentation,"
           << "native_plane_extraction_ms,native_plane_boundary_ms,"
           << "native_plane_endpoint_filter_ms,"
           << "native_plane_components_ms,native_plane_total_ms,"
           << "native_plane_raw_boundary_written,"
           << "native_plane_retained_boundary_written,"
           << "native_plane_combined_edge_written,"
           << "native_plane_combined_labels_written,"
           << "native_plane_dynamic_decision,"
           << "native_rag_enabled,native_rag_available,"
           << "native_rag_dynamic_state_available,"
           << "native_rag_plane_rejection_available,"
           << "native_rag_image_pixels,native_rag_input_core_pixels,"
           << "native_rag_output_core_pixels,"
           << "native_rag_input_component_count,"
           << "native_rag_output_region_count,native_rag_total_pairs,"
           << "native_rag_spatial_adjacent_pairs,"
           << "native_rag_shared_fake_edge_pairs,"
           << "native_rag_depth_rejected_pairs,"
           << "native_rag_eligible_pairs,"
           << "native_rag_high_middle_merges,native_rag_low_merges,"
           << "native_rag_unmerged_low_regions,"
           << "native_rag_cross_gradient_merge_violations,"
           << "native_rag_mean_hist_similarity,"
           << "native_rag_max_hist_similarity,"
           << "native_rag_mean_eligible_score,"
           << "native_rag_max_eligible_score,"
           << "native_rag_median_group_components,"
           << "native_rag_max_group_components,"
           << "native_rag_smallest_region_pixels,"
           << "native_rag_largest_region_pixels,"
           << "native_rag_attribute_ms,native_rag_graph_ms,"
           << "native_rag_merge_ms,native_rag_total_ms,"
           << "native_rag_labels_written,native_rag_dynamic_decision,"
           << "native_dynamic_decision,"
           << "region_dynamic_enabled,region_dynamic_available,"
           << "region_dynamic_state_available,region_dynamic_failure_reason,"
           << "region_dynamic_frame_index,region_dynamic_valid_pixels,"
           << "region_dynamic_unknown_pixels,region_dynamic_above_low_pixels,"
           << "region_dynamic_high_pixels,region_dynamic_temporal_added_pixels,"
           << "region_dynamic_region_count,region_dynamic_regions_with_high,"
           << "region_dynamic_eligible_contours,region_dynamic_valid_seeds,"
           << "region_dynamic_whole_regions,region_dynamic_partial_regions,"
           << "region_dynamic_core_pixels,region_dynamic_author_pixels,"
           << "region_dynamic_valid_dynamic_pixels,region_dynamic_static_pixels,"
           << "region_dynamic_prepare_ms,region_dynamic_classify_ms,"
           << "region_dynamic_total_ms,region_dynamic_orb_count,"
           << "region_dynamic_author_overlap_pixels,"
           << "region_dynamic_author_union_pixels,"
           << "region_dynamic_author_overlap_orb_count,"
           << "region_feature_filter_enabled,"
           << "region_feature_filter_applied,"
           << "region_feature_filter_state,"
           << "region_feature_filter_candidate_features,"
           << "region_feature_filter_semantic_overlap,"
           << "region_feature_filter_new_dynamic_features,"
           << "region_feature_filter_remaining_features,"
           << "region_feature_filter_actual_removed_associations,"
           << "region_feature_filter_tracking_fail_open,"
           << "region_feature_filter_tracking_fail_open_stage,"
           << "region_feature_filter_tracking_fail_open_cleared_features,"
           << "region_feature_filter_mapping_flags_restored,"
           << "region_dynamic_decision,region_dynamic_direct_slam_mutation\n";
    stream << std::setprecision(15);

    for(std::size_t index=0;
        index<mvSInStyleShadowDiagnostics.size(); ++index)
    {
        const SInStyleShadowRecord &record =
            mvSInStyleShadowDiagnostics[index];
        std::vector<std::string> nativeLevelShapes;
        std::vector<std::size_t> nativeLevelValidSamples;
        std::vector<std::size_t> nativeLevelPriorSamples;
        std::vector<std::size_t> nativeLevelGridFallbackSamples;
        std::vector<double> nativeLevelCompactness;
        std::vector<double> nativeLevelPrepareMs;
        std::vector<double> nativeLevelKMeansMs;
        std::vector<double> nativeLevelLabelMs;
        for(std::size_t levelIndex=0;
            levelIndex<record.nativeInitialStats.levels.size(); ++levelIndex)
        {
            const SInStyleInitialRegionLevelStats &level =
                record.nativeInitialStats.levels[levelIndex];
            std::ostringstream shape;
            shape << level.level << ":" << level.cols << "x" << level.rows;
            nativeLevelShapes.push_back(shape.str());
            nativeLevelValidSamples.push_back(level.validSamples);
            nativeLevelPriorSamples.push_back(level.priorInitializedSamples);
            nativeLevelGridFallbackSamples.push_back(
                level.gridFallbackSamples);
            nativeLevelCompactness.push_back(level.compactness);
            nativeLevelPrepareMs.push_back(level.prepareMs);
            nativeLevelKMeansMs.push_back(level.kmeansMs);
            nativeLevelLabelMs.push_back(level.labelMs);
        }
        stream << record.inputFrameIndex << ","
               << record.frameId << ","
               << record.resetEpoch << ","
               << record.timestamp << ","
               << record.referenceBackend << ","
               << record.regionDynamicLabelSource << ","
               << static_cast<int>(record.stats.referenceAvailable) << ","
               << static_cast<int>(record.stats.labelsAvailable) << ","
               << record.stats.pixelCount << ","
               << record.stats.depthValidPixels << ","
               << record.stats.rawUnknownPixels << ","
               << record.stats.rawStaticPixels << ","
               << record.stats.rawDynamicPixels << ","
               << record.stats.validPixels << ","
               << record.stats.staticPixels << ","
               << record.stats.dynamicPixels << ","
               << record.stats.unknownPixels << ","
               << record.stats.positiveLabelCount << ","
               << record.stats.positiveLabelPixels << ","
               << record.stats.depthSupportedPositiveLabelPixels << ","
               << record.stats.positiveLabelComponentCount << ","
               << record.stats.authorDynamicPixelsOnPositiveLabels << ","
               << record.stats.authorDynamicPixelsOnLabelZero << ","
               << record.stats.authorDynamicPixelsWithLabelsUnavailable << ","
               << record.rawOrbCount << ","
               << record.authorDynamicMaskHitOnDtOrbSet << ","
               << record.depthSupportedDynamicOrbCount << ","
               << record.validOrbCount << ","
               << record.unknownOrbCount << ","
               << record.semanticDynamicOrbCount << ","
               << record.semanticAuthorDynamicOverlapOnDtOrbSet << ","
               << record.wouldKeepOrbCount << ","
               << static_cast<int>(record.counterfactualFallbackOnDtOrbSet) << ","
               << record.counterfactualRemovedOnDtOrbSet << ","
               << 0 << ","
               << record.trackingStateAfter << ","
               << record.runtime.loadMs << ","
               << record.runtime.stateConversionMs << ","
               << record.runtime.regionStatisticsMs << ","
               << record.runtime.totalMs << ","
               << "shadow_only,none,"
               << static_cast<int>(record.denseFlowStats.enabled) << ","
               << static_cast<int>(record.denseFlowStats.available) << ","
               << static_cast<int>(
                      record.denseFlowStats.dynamicStateAvailable) << ","
               << record.denseFlowStats.failureReason << ","
               << record.denseFlowStats.backend << ","
               << record.denseFlowStats.frameIndex << ","
               << record.denseFlowStats.intendedReferenceLag << ","
               << record.denseFlowStats.referenceIndex << ","
               << record.denseFlowStats.actualReferenceLag << ","
               << static_cast<int>(record.denseFlowStats.largeMotion) << ","
               << record.denseFlowStats.imageScale << ","
               << record.denseFlowStats.homographySampleCount << ","
               << record.denseFlowStats.maxObservedFlowPx << ","
               << static_cast<int>(
                      record.denseFlowStats.temporalRegionPriorUsed) << ","
               << record.denseFlowStats.temporalUnknownSamples << ","
               << record.denseFlowStats.temporalStaticSamples << ","
               << record.denseFlowStats.temporalDynamicSamples << ","
               << record.denseFlowStats.maxResidualPx << ","
               << record.denseFlowStats.otsuThresholdU8 << ","
               << record.denseFlowStats.triangleThresholdU8 << ","
               << record.denseFlowStats.lowThresholdU8 << ","
               << record.denseFlowStats.highThresholdU8 << ","
               << record.denseFlowStats.lowThresholdPx << ","
               << record.denseFlowStats.highThresholdPx << ","
               << record.denseFlowStats.lowPixels << ","
               << record.denseFlowStats.highPixels << ","
               << record.denseFlowStats.residualRecomputeMaxAbsPx << ","
               << record.denseFlowStats.normalizedRecomputeMaxAbs << ","
               << record.denseFlowStats.loadMs << ","
               << record.denseFlowStats.validateMs << ","
               << record.denseFlowStats.totalMs << ","
               << "none,none,"
               << static_cast<int>(record.nativeInitialStats.enabled) << ","
               << static_cast<int>(record.nativeInitialStats.available) << ","
               << static_cast<int>(
                      record.nativeInitialStats.dynamicStateAvailable) << ","
               << record.nativeInitialStats.imagePixels << ","
               << record.nativeInitialStats.inputDepthValidPixels << ","
               << record.nativeInitialStats.clusteringDepthValidPixels << ","
               << record.nativeInitialStats.excludedFarDepthPixels << ","
               << record.nativeInitialStats.requestedClusters << ","
               << record.nativeInitialStats.producedClusters << ","
               << record.nativeInitialStats.smallestRegionPixels << ","
               << record.nativeInitialStats.largestRegionPixels << ","
               << record.nativeInitialStats.compactness << ","
               << record.nativeInitialStats.prepareMs << ","
               << record.nativeInitialStats.kmeansMs << ","
               << record.nativeInitialStats.labelConversionMs << ","
               << record.nativeInitialStats.totalMs << ","
               << record.nativeInitialOrbAssignedCount << ","
               << static_cast<int>(record.nativeInitialLabelsWritten) << ","
               << static_cast<int>(record.nativeInitialStats.coarseToFine)
               << ","
               << record.nativeInitialStats.pyramidLevels << ","
               << record.nativeInitialStats.initializationSource << ","
               << record.nativeInitialStats.previousPriorSamples << ","
               << record.nativeInitialStats.gridFallbackSamples << ","
               << record.nativeInitialStats.previousPriorCoverage << ","
               << static_cast<int>(
                      record.nativeInitialStats.temporalPriorCommitted) << ","
               << JoinDiagnosticValues(nativeLevelShapes) << ","
               << JoinDiagnosticValues(nativeLevelValidSamples) << ","
               << JoinDiagnosticValues(nativeLevelPriorSamples) << ","
               << JoinDiagnosticValues(nativeLevelGridFallbackSamples) << ","
               << JoinDiagnosticValues(nativeLevelCompactness) << ","
               << JoinDiagnosticValues(nativeLevelPrepareMs) << ","
               << JoinDiagnosticValues(nativeLevelKMeansMs) << ","
               << JoinDiagnosticValues(nativeLevelLabelMs) << ","
               << static_cast<int>(record.nativeGradientStats.enabled) << ","
               << static_cast<int>(record.nativeGradientStats.available) << ","
               << static_cast<int>(
                      record.nativeGradientStats.dynamicStateAvailable) << ","
               << record.nativeGradientStats.imagePixels << ","
               << record.nativeGradientStats.inputDepthValidPixels << ","
               << record.nativeGradientStats.initialRegionPixels << ","
               << record.nativeGradientStats.medianValidPixels << ","
               << record.nativeGradientStats.insufficientSupportPixels << ","
               << record.nativeGradientStats.rawGradientEdgePixels << ","
               << record.nativeGradientStats.splitBoundaryPixels << ","
               << record.nativeGradientStats.splitCorePixels << ","
               << record.nativeGradientStats.initialRegionCount << ","
               << record.nativeGradientStats.splitComponentCount << ","
               << record.nativeGradientStats.splitInitialRegionCount << ","
               << record.nativeGradientStats.fullyConsumedInitialRegionCount << ","
               << record.nativeGradientStats.medianFragmentation << ","
               << record.nativeGradientStats.maximumFragmentation << ","
               << record.nativeGradientStats.smallComponentCount << ","
               << record.nativeGradientStats.smallComponentPixels << ","
               << record.nativeGradientStats.medianFilterMs << ","
               << record.nativeGradientStats.gradientEdgeMs << ","
               << record.nativeGradientStats.connectedComponentsMs << ","
               << record.nativeGradientStats.totalMs << ","
               << static_cast<int>(record.nativeGradientEdgeWritten) << ","
               << static_cast<int>(
                      record.nativeGradientSplitLabelsWritten) << ","
               << "none,"
               << static_cast<int>(record.nativePlaneStats.enabled) << ","
               << static_cast<int>(record.nativePlaneStats.available) << ","
               << static_cast<int>(
                      record.nativePlaneStats.dynamicStateAvailable) << ","
               << static_cast<int>(
                      record.nativePlaneStats.opencvPlaneSubstitute) << ","
               << record.nativePlaneStats.imagePixels << ","
               << record.nativePlaneStats.inputDepthValidPixels << ","
               << record.nativePlaneStats.initialRegionPixels << ","
               << record.nativePlaneStats.planePixels << ","
               << record.nativePlaneStats.planeCount << ","
               << record.nativePlaneStats.rawPlaneBoundaryPixels << ","
               << record.nativePlaneStats.gradientOverlapPixels << ","
               << record.nativePlaneStats.planeCandidateBoundaryPixels << ","
               << record.nativePlaneStats.gradientEndpointPixels << ","
               << record.nativePlaneStats.planeBoundarySegmentCount << ","
               << record.nativePlaneStats.retainedPlaneBoundarySegmentCount
               << ","
               << record.nativePlaneStats.unsupportedPlaneBoundarySegmentCount
               << ","
               << record.nativePlaneStats.retainedPlaneBoundaryPixels << ","
               << record.nativePlaneStats.combinedEdgePixels << ","
               << record.nativePlaneStats.combinedCorePixels << ","
               << record.nativePlaneStats.initialRegionCount << ","
               << record.nativePlaneStats.combinedComponentCount << ","
               << record.nativePlaneStats.splitInitialRegionCount << ","
               << record.nativePlaneStats.fullyConsumedInitialRegionCount
               << ","
               << record.nativePlaneStats.maximumFragmentation << ","
               << record.nativePlaneStats.planeExtractionMs << ","
               << record.nativePlaneStats.boundaryBuildMs << ","
               << record.nativePlaneStats.endpointFilterMs << ","
               << record.nativePlaneStats.connectedComponentsMs << ","
               << record.nativePlaneStats.totalMs << ","
               << static_cast<int>(record.nativePlaneRawBoundaryWritten)
               << ","
               << static_cast<int>(record.nativePlaneRetainedBoundaryWritten)
               << ","
               << static_cast<int>(record.nativeCombinedEdgeWritten) << ","
               << static_cast<int>(record.nativeCombinedSplitLabelsWritten)
               << ","
               << "none,"
               << static_cast<int>(record.nativeRAGStats.enabled) << ","
               << static_cast<int>(record.nativeRAGStats.available) << ","
               << static_cast<int>(
                      record.nativeRAGStats.dynamicStateAvailable) << ","
               << static_cast<int>(
                      record.nativeRAGStats.planeRejectionAvailable) << ","
               << record.nativeRAGStats.imagePixels << ","
               << record.nativeRAGStats.inputCorePixels << ","
               << record.nativeRAGStats.outputCorePixels << ","
               << record.nativeRAGStats.inputComponentCount << ","
               << record.nativeRAGStats.outputRegionCount << ","
               << record.nativeRAGStats.totalPairCount << ","
               << record.nativeRAGStats.spatialAdjacentPairCount << ","
               << record.nativeRAGStats.sharedFakeEdgePairCount << ","
               << record.nativeRAGStats.depthRejectedPairCount << ","
               << record.nativeRAGStats.eligiblePairCount << ","
               << record.nativeRAGStats.highMiddleMergeCount << ","
               << record.nativeRAGStats.lowScoreMergeCount << ","
               << record.nativeRAGStats.unmergedLowScoreRegionCount << ","
               << record.nativeRAGStats.crossGradientMergeViolationCount
               << ","
               << record.nativeRAGStats.meanHistogramSimilarityOnAdjacentPairs
               << ","
               << record.nativeRAGStats.maximumHistogramSimilarityOnAdjacentPairs
               << ","
               << record.nativeRAGStats.meanTotalScoreOnEligiblePairs << ","
               << record.nativeRAGStats.maximumTotalScoreOnEligiblePairs << ","
               << record.nativeRAGStats.medianMergedGroupComponents << ","
               << record.nativeRAGStats.maximumMergedGroupComponents << ","
               << record.nativeRAGStats.smallestMergedRegionPixels << ","
               << record.nativeRAGStats.largestMergedRegionPixels << ","
               << record.nativeRAGStats.attributeMs << ","
               << record.nativeRAGStats.ragMs << ","
               << record.nativeRAGStats.mergeMs << ","
               << record.nativeRAGStats.totalMs << ","
               << static_cast<int>(record.nativeRAGMergedLabelsWritten) << ","
               << "none,"
               << "none,"
               << static_cast<int>(record.regionDynamicStats.enabled) << ","
               << static_cast<int>(record.regionDynamicStats.available) << ","
               << static_cast<int>(
                      record.regionDynamicStats.dynamicStateAvailable) << ","
               << record.regionDynamicStats.failureReason << ","
               << record.regionDynamicStats.frameIndex << ","
               << record.regionDynamicStats.validRegionPixels << ","
               << record.regionDynamicStats.unknownPixels << ","
               << record.regionDynamicStats.lowResidualPixels << ","
               << record.regionDynamicStats.highResidualPixels << ","
               << record.regionDynamicStats.temporalHighPixelsAdded << ","
               << record.regionDynamicStats.regionCount << ","
               << record.regionDynamicStats.regionsWithHighSupport << ","
               << record.regionDynamicStats.eligibleContourCount << ","
               << record.regionDynamicStats.validSeedContourCount << ","
               << record.regionDynamicStats.wholeDynamicRegionCount << ","
               << record.regionDynamicStats.partialDynamicRegionCount << ","
               << record.regionDynamicStats.dynamicPixelsBeforeDilation << ","
               << record.regionDynamicStats.authorStyleDynamicPixels << ","
               << record.regionDynamicStats.depthSupportedDynamicPixels << ","
               << record.regionDynamicStats.staticPixels << ","
               << record.regionDynamicStats.prepareMs << ","
               << record.regionDynamicStats.classifyMs << ","
               << record.regionDynamicStats.totalMs << ","
               << record.regionDynamicOrbCount << ","
               << record.regionDynamicAuthorOverlapPixels << ","
               << record.regionDynamicAuthorUnionPixels << ","
               << record.regionDynamicAuthorOverlapOrbCount << ","
               << static_cast<int>(record.regionFeatureFilterEnabled) << ","
               << static_cast<int>(record.regionFeatureFilterApplied) << ","
               << record.regionFeatureFilterState << ","
               << record.regionFeatureFilterCandidateFeatures << ","
               << record.regionFeatureFilterSemanticOverlap << ","
               << record.regionFeatureFilterNewDynamicFeatures << ","
               << record.regionFeatureFilterRemainingFeatures << ","
               << record.regionFeatureFilterActualRemovedAssociations << ","
               << static_cast<int>(
                      record.regionFeatureFilterTrackingFailOpen) << ","
               << record.regionFeatureFilterTrackingFailOpenStage << ","
               << record.
                      regionFeatureFilterTrackingFailOpenClearedFeatures
               << ","
               << static_cast<int>(
                      record.regionFeatureFilterMappingFlagsRestored) << ","
               << (record.regionFeatureFilterTrackingFailOpen ?
                      "region_feature_filter_tracking_fail_open" :
                      (record.regionFeatureFilterApplied ?
                      "region_feature_filter" :
                      (record.regionDynamicStats.available ?
                           "shadow_only" : "none"))) << ","
               << (record.regionFeatureFilterTrackingFailOpen ?
                      "mapping_dynamic_flags_only" :
                      (record.regionFeatureFilterApplied ?
                      "frame_dynamic_flags" : "none")) << "\n";
    }
    stream.close();
    cout << "[SIn S1] saved " << mvSInStyleShadowDiagnostics.size()
         << " shadow rows to " << mSInStyleShadowCsvPath << endl;
}

void Tracking::SaveSInStyleDepthFilterDiagnostics()
{
    if(mSInStyleDepthFilterCsvPath.empty() ||
       mvSInStyleDepthFilterDiagnostics.empty())
    {
        return;
    }

    ofstream stream(mSInStyleDepthFilterCsvPath.c_str());
    if(!stream.is_open())
    {
        cerr << "[SIn S3] failed to open depth-filter CSV: "
             << mSInStyleDepthFilterCsvPath << endl;
        return;
    }

    stream << "input_index,frame,timestamp,enabled,available,mask_mode,"
           << "state,geometry_evidence_available,input_valid_depth_pixels,"
           << "semantic_dynamic_pixels,geometry_dynamic_pixels,"
           << "union_dynamic_pixels,rejected_valid_depth_pixels,"
           << "output_valid_depth_pixels,filter_ms,tracking_state_after,"
           << "mapping_output_available,mask_written,"
           << "tracking_state_mutation\n";
    stream << std::setprecision(17);
    for(std::size_t index=0;
        index<mvSInStyleDepthFilterDiagnostics.size(); ++index)
    {
        const SInStyleDepthFilterRecord &record =
            mvSInStyleDepthFilterDiagnostics[index];
        const SInStyleDepthFilterStats &stats = record.stats;
        stream << record.inputFrameIndex << ","
               << record.frameId << ","
               << record.timestamp << ","
               << stats.enabled << ","
               << stats.available << ","
               << stats.maskMode << ","
               << stats.state << ","
               << stats.geometryEvidenceAvailable << ","
               << stats.inputValidDepthPixels << ","
               << stats.semanticDynamicPixels << ","
               << stats.geometryDynamicPixels << ","
               << stats.unionDynamicPixels << ","
               << stats.rejectedValidDepthPixels << ","
               << stats.outputValidDepthPixels << ","
               << stats.totalMs << ","
               << record.trackingStateAfter << ","
               << record.mappingOutputAvailable << ","
               << record.maskWritten << ",none\n";
    }
    cout << "[SIn S3] depth-filter diagnostics saved: "
         << mSInStyleDepthFilterCsvPath << endl;
}

cv::Mat Tracking::GetCurrentDynamicDepthMaskForMapping() const
{
    if(!mbCurrentSInDepthMappingAdmissible)
        return cv::Mat();
    return mCurrentSInDepthFilterResult.dynamicDepthMask.clone();
}

cv::Mat Tracking::GetCurrentStaticDepthForMapping() const
{
    if(!mbCurrentSInDepthMappingAdmissible)
        return cv::Mat();
    return mCurrentSInDepthFilterResult.staticDepthMeters.clone();
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

    if(!mGeometryMultiReferenceCsvPath.empty() &&
       !mvGeometryMultiReferenceHistogram.empty())
    {
        ofstream stream(mGeometryMultiReferenceCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-1] failed to open histogram CSV: "
                 << mGeometryMultiReferenceCsvPath << endl;
        }
        else
        {
            stream << "frame,timestamp,sampling_policy,reference_count,"
                   << "comparison_count,"
                   << "positive_count,pixel_count,semantic_pixel_count,"
                   << "pixels_with_comparison,total_comparisons,"
                   << "pixels_with_positive,total_positive_votes,"
                   << "total_negative_votes,total_consistent_votes,"
                   << "warp_evidence_ms,aggregate_ms,preprocess_ms,"
                   << "expand_ms,total_ms\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometryMultiReferenceHistogram.size(); ++index)
            {
                const GeometryMultiReferenceHistogramRecord &record =
                    mvGeometryMultiReferenceHistogram[index];
                const GeometricMultiReferenceStats &stats =
                    record.frameStats;
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.samplingPolicy << ","
                       << record.referenceCount << ","
                       << record.comparisonCount << ","
                       << record.positiveCount << ","
                       << record.pixelCount << ","
                       << record.semanticPixelCount << ","
                       << stats.pixelsWithComparison << ","
                       << stats.totalComparisons << ","
                       << stats.pixelsWithPositiveEvidence << ","
                       << stats.totalPositiveVotes << ","
                       << stats.totalNegativeVotes << ","
                       << stats.totalConsistentVotes << ","
                       << stats.warpAndEvidenceMs << ","
                       << stats.aggregateMs << ","
                       << stats.preprocessMs << ","
                       << stats.expandMs << ","
                       << stats.totalMs << "\n";
            }
            stream.close();
            cout << "[Geometry G2-1] saved "
                 << mvGeometryMultiReferenceHistogram.size()
                 << " histogram rows to "
                 << mGeometryMultiReferenceCsvPath << endl;
        }
    }

    if(!mGeometryMultiReferenceFeatureCsvPath.empty() &&
       !mvGeometryMultiReferenceFeatureDiagnostics.empty())
    {
        ofstream stream(
            mGeometryMultiReferenceFeatureCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-4F0] failed to open feature evidence CSV: "
                 << mGeometryMultiReferenceFeatureCsvPath << endl;
        }
        else
        {
            stream << "frame,timestamp,sampling_policy,feature_index,"
                   << "u_raw,v_raw,octave,has_mappoint,"
                   << "current_frame_outlier_flag,semantic_nonzero,native_scale,"
                   << "native_u,native_v,comparison_count,"
                   << "positive_count,negative_count,consistent_count\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometryMultiReferenceFeatureDiagnostics.size();
                ++index)
            {
                const GeometryMultiReferenceFeatureRecord &record =
                    mvGeometryMultiReferenceFeatureDiagnostics[index];
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.samplingPolicy << ","
                       << record.featureIndex << ","
                       << record.imageU << ","
                       << record.imageV << ","
                       << record.octave << ","
                       << (record.hasMapPoint ? 1 : 0) << ","
                       << (record.currentFrameOutlierFlag ? 1 : 0) << ","
                       << (record.semanticNonzero ? 1 : 0) << ","
                       << record.nativeScale << ","
                       << record.nativeU << ","
                       << record.nativeV << ","
                       << static_cast<int>(record.comparisonCount) << ","
                       << static_cast<int>(record.positiveCount) << ","
                       << static_cast<int>(record.negativeCount) << ","
                       << static_cast<int>(record.consistentCount) << "\n";
            }
            stream.close();
            cout << "[Geometry G2-4F0] saved "
                 << mvGeometryMultiReferenceFeatureDiagnostics.size()
                 << " feature evidence rows to "
                 << mGeometryMultiReferenceFeatureCsvPath << endl;
        }
    }

    if(!mGeometrySparseFlowCsvPath.empty() &&
       !mvGeometrySparseFlowFeatureDiagnostics.empty())
    {
        ofstream stream(mGeometrySparseFlowCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-4F1] failed to open sparse-flow CSV: "
                 << mGeometrySparseFlowCsvPath << endl;
        }
        else
        {
            stream
                << "frame,timestamp,reference_frame,"
                << "reference_timestamp,dt_seconds,feature_index,"
                << "u_current,v_current,octave,has_mappoint,"
                << "semantic_nonzero,quality_eligible,q_candidate,"
                << "normalized_residual_q,backward_lk_status,"
                << "forward_lk_status,u_reference,v_reference,"
                << "u_forward_back,v_forward_back,"
                << "lk_error_backward,lk_error_forward,"
                << "forward_backward_error_px,"
                << "reference_depth_valid,reference_depth_m,"
                << "reference_depth_boundary_d1,"
                << "reference_depth_boundary_d2,"
                << "reference_invalid_depth_d1,"
                << "reference_invalid_depth_d2,"
                << "slam_ego_projection_valid,slam_u_ego,"
                << "slam_v_ego,slam_residual_x_px,"
                << "slam_residual_y_px,"
                << "slam_residual_magnitude_px,"
                << "gt_pose_available,gt_ego_projection_valid,"
                << "gt_u_ego,gt_v_ego,gt_residual_x_px,"
                << "gt_residual_y_px,"
                << "gt_residual_magnitude_px,evidence_state\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<
                    mvGeometrySparseFlowFeatureDiagnostics.size();
                ++index)
            {
                const GeometrySparseFlowFeatureRecord &record =
                    mvGeometrySparseFlowFeatureDiagnostics[index];
                const GeometricSparseFlowSample &sample =
                    record.sample;
                const bool hasReference =
                    sample.evidenceState!=
                    GeometricSparseFlowEvidenceState::
                        ReferenceUnavailable;
                stream << record.frameId << ","
                       << record.timestamp << ",";
                if(hasReference)
                {
                    stream << record.referenceFrameId << ","
                           << record.referenceTimestamp << ","
                           << record.timestamp-
                              record.referenceTimestamp << ",";
                }
                else
                {
                    stream << ",,,";
                }
                stream << sample.featureIndex << ","
                       << sample.currentPixel.x << ","
                       << sample.currentPixel.y << ","
                       << record.octave << ","
                       << (record.hasMapPoint ? 1 : 0) << ","
                       << (record.semanticNonzero ? 1 : 0) << ","
                       << (record.qualityEligible ? 1 : 0) << ","
                       << (record.candidate ? 1 : 0) << ",";
                if(std::isfinite(record.normalizedResidualQ))
                    stream << record.normalizedResidualQ;
                stream << ","
                       << (sample.backwardLkValid ? 1 : 0) << ","
                       << (sample.forwardLkValid ? 1 : 0) << ",";
                if(sample.backwardLkValid)
                {
                    stream << sample.referencePixel.x << ","
                           << sample.referencePixel.y << ",";
                }
                else
                {
                    stream << ",,";
                }
                if(sample.forwardLkValid)
                {
                    stream << sample.forwardBackPixel.x << ","
                           << sample.forwardBackPixel.y << ",";
                }
                else
                {
                    stream << ",,";
                }
                if(sample.backwardLkValid)
                    stream << sample.backwardLkError;
                stream << ",";
                if(sample.forwardLkValid)
                    stream << sample.forwardLkError;
                stream << ",";
                if(sample.backwardLkValid &&
                   sample.forwardLkValid)
                {
                    stream
                        << sample.forwardBackwardErrorPixels;
                }
                stream << ","
                       << (sample.referenceDepthValid ? 1 : 0)
                       << ",";
                if(sample.referenceDepthValid)
                    stream << sample.referenceDepthMeters;
                stream << ","
                       << (sample.referenceDepthBoundaryWithinOnePixel
                           ? 1 : 0)
                       << ","
                       << (sample.referenceDepthBoundaryWithinTwoPixels
                           ? 1 : 0)
                       << ","
                       << (sample.referenceInvalidDepthWithinOnePixel
                           ? 1 : 0)
                       << ","
                       << (sample.referenceInvalidDepthWithinTwoPixels
                           ? 1 : 0)
                       << ","
                       << (sample.slamProjectionValid ? 1 : 0)
                       << ",";
                if(sample.slamProjectionValid)
                {
                    stream << sample.slamEgoPixel.x << ","
                           << sample.slamEgoPixel.y << ","
                           << sample.slamResidualPixels.x << ","
                           << sample.slamResidualPixels.y << ","
                           << sample.slamResidualMagnitudePixels;
                }
                else
                {
                    stream << ",,,,";
                }
                stream << ","
                       << (sample.groundTruthPoseAvailable ? 1 : 0)
                       << ","
                       << (sample.groundTruthProjectionValid ? 1 : 0)
                       << ",";
                if(sample.groundTruthProjectionValid)
                {
                    stream
                        << sample.groundTruthEgoPixel.x << ","
                        << sample.groundTruthEgoPixel.y << ","
                        << sample.groundTruthResidualPixels.x << ","
                        << sample.groundTruthResidualPixels.y << ","
                        << sample.groundTruthResidualMagnitudePixels;
                }
                else
                {
                    stream << ",,,,";
                }
                stream << ","
                       << GeometricDynamicDetector::
                          SparseFlowEvidenceStateName(
                              sample.evidenceState)
                       << "\n";
            }
            stream.close();
            cout << "[Geometry G2-4F1] saved "
                 << mvGeometrySparseFlowFeatureDiagnostics.size()
                 << " sparse-flow feature rows to "
                 << mGeometrySparseFlowCsvPath << endl;
        }
    }

    if(!mGeometrySparseFlowCsvPath.empty() &&
       !mvGeometrySparseFlowFrameDiagnostics.empty())
    {
        const std::string frameCsvPath =
            mGeometrySparseFlowCsvPath+".frames.csv";
        ofstream stream(frameCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-4F1] failed to open sparse-flow "
                 << "frame CSV: " << frameCsvPath << endl;
        }
        else
        {
            stream
                << "frame,timestamp,reference_frame,"
                << "reference_timestamp,dt_seconds,"
                << "reference_available,domain_valid,features,"
                << "backward_lk_valid,forward_lk_valid,"
                << "reference_depth_valid,slam_residual_valid,"
                << "gt_residual_valid,slam_residual_median_px,"
                << "slam_residual_p90_px,slam_residual_p95_px,"
                << "gt_residual_median_px,"
                << "gt_residual_p90_px,gt_residual_p95_px,"
                << "backward_lk_ms,forward_lk_ms,"
                << "depth_projection_ms,record_ms,"
                << "compute_total_ms,active_total_ms,"
                << "dynamic_decision,"
                << "direct_slam_state_mutation\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometrySparseFlowFrameDiagnostics.size();
                ++index)
            {
                const GeometrySparseFlowFrameRecord &record =
                    mvGeometrySparseFlowFrameDiagnostics[index];
                const GeometricSparseFlowStats &stats =
                    record.stats;
                stream << record.frameId << ","
                       << record.timestamp << ",";
                if(record.referenceAvailable)
                {
                    stream << record.referenceFrameId << ","
                           << record.referenceTimestamp << ","
                           << record.timestamp-
                              record.referenceTimestamp << ",";
                }
                else
                {
                    stream << ",,,";
                }
                stream
                    << (record.referenceAvailable ? 1 : 0)
                    << ","
                    << (record.domainValid ? 1 : 0) << ","
                    << stats.featureCount << ","
                    << stats.backwardLkValidCount << ","
                    << stats.forwardLkValidCount << ","
                    << stats.referenceDepthValidCount << ","
                    << stats.slamResidualValidCount << ","
                    << stats.groundTruthResidualValidCount << ","
                    << stats.slamResidualMedianPixels << ","
                    << stats.slamResidualP90Pixels << ","
                    << stats.slamResidualP95Pixels << ","
                    << stats.groundTruthResidualMedianPixels << ","
                    << stats.groundTruthResidualP90Pixels << ","
                    << stats.groundTruthResidualP95Pixels << ","
                    << stats.backwardLkMs << ","
                    << stats.forwardLkMs << ","
                    << stats.depthAndProjectionMs << ","
                    << record.recordMs << ","
                    << stats.totalMs << ","
                    << record.activeTotalMs << ",none,none\n";
            }
            stream.close();
            cout << "[Geometry G2-4F1] saved "
                 << mvGeometrySparseFlowFrameDiagnostics.size()
                 << " sparse-flow frame rows to "
                 << frameCsvPath << endl;
        }
    }

    if(!mGeometryAssociationSnapshotCsvPath.empty() &&
       !mvGeometryAssociationSnapshotDiagnostics.empty())
    {
        ofstream stream(
            mGeometryAssociationSnapshotCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G1-F0B] failed to open association "
                 << "snapshot CSV: "
                 << mGeometryAssociationSnapshotCsvPath << endl;
        }
        else
        {
            stream
                << "frame,timestamp,stage,feature_index,"
                << "has_mappoint,mappoint_bad,mappoint_observations,"
                << "current_frame_outlier,semantic_nonzero,"
                << "only_tracking,within_relocalization_window,"
                << "counted_tracking_inlier,tracking_inliers,"
                << "counterfactual_only,dynamic_decision,"
                << "direct_slam_state_mutation,pose_reoptimization\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<
                    mvGeometryAssociationSnapshotDiagnostics.size();
                ++index)
            {
                const GeometryAssociationSnapshotRecord &record =
                    mvGeometryAssociationSnapshotDiagnostics[index];
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.stage << ","
                       << record.featureIndex << ","
                       << (record.hasMapPoint ? 1 : 0) << ","
                       << (record.mapPointBad ? 1 : 0) << ","
                       << record.mapPointObservations << ","
                       << (record.currentFrameOutlier ? 1 : 0)
                       << ","
                       << (record.semanticNonzero ? 1 : 0) << ","
                       << (record.onlyTracking ? 1 : 0) << ","
                       << (record.withinRelocalizationWindow
                               ? 1 : 0)
                       << ","
                       << (record.countedTrackingInlier
                               ? 1 : 0)
                       << ","
                       << record.trackingInliers
                       << ",true,none,none,none\n";
            }
            stream.close();
            cout << "[Geometry G1-F0B] saved "
                 << mvGeometryAssociationSnapshotDiagnostics.size()
                 << " raw association snapshot rows to "
                 << mGeometryAssociationSnapshotCsvPath << endl;
        }
    }

    if(!mGeometrySparseFlowTrackingFilterCsvPath.empty() &&
       !mvGeometrySparseFlowTrackingFilterDiagnostics.empty())
    {
        ofstream stream(
            mGeometrySparseFlowTrackingFilterCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G1-F1] failed to open tracking "
                 << "filter CSV: "
                 << mGeometrySparseFlowTrackingFilterCsvPath
                 << endl;
        }
        else
        {
            stream
                << "frame,timestamp,q_threshold,scale_valid,"
                << "frame_scale_px,scale_support,"
                << "quality_eligible_features,candidate_features,"
                << "baseline_associations,candidate_associations,"
                << "removed_associations,remaining_associations,"
                << "candidate_association_fraction,"
                << "within_relocalization_window,applied,state,"
                << "pose_reoptimization,mapping_veto\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<
                    mvGeometrySparseFlowTrackingFilterDiagnostics.
                        size(); ++index)
            {
                const GeometrySparseFlowTrackingFilterRecord &record =
                    mvGeometrySparseFlowTrackingFilterDiagnostics[
                        index];
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.qThreshold << ","
                       << (record.scaleValid ? 1 : 0) << ","
                       << record.frameScalePixels << ","
                       << record.scaleSupport << ","
                       << record.qualityEligibleFeatures << ","
                       << record.candidateFeatures << ","
                       << record.baselineAssociations << ","
                       << record.candidateAssociations << ","
                       << record.removedAssociations << ","
                       << record.remainingAssociations << ","
                       << record.candidateAssociationFraction << ","
                       << (record.withinRelocalizationWindow
                               ? 1 : 0)
                       << ","
                       << (record.applied ? 1 : 0) << ","
                       << record.state
                       << ",none,none\n";
            }
            stream.close();
            cout << "[Geometry G1-F1] saved "
                 << mvGeometrySparseFlowTrackingFilterDiagnostics.
                        size()
                 << " tracking-filter frame rows to "
                 << mGeometrySparseFlowTrackingFilterCsvPath
                 << endl;
        }
    }

    if(!mGeometrySparseFlowRemovedAssociationCsvPath.empty())
    {
        ofstream stream(
            mGeometrySparseFlowRemovedAssociationCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G1-F1] failed to open exact removed-"
                 << "association CSV: "
                 << mGeometrySparseFlowRemovedAssociationCsvPath
                 << endl;
        }
        else
        {
            stream
                << "frame,timestamp,feature_index,u,v,mappoint_id,"
                << "q_threshold,frame_scale_px,semantic_dynamic,"
                << "filter_state,direct_slam_state_mutation\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<
                    mvGeometrySparseFlowRemovedAssociationDiagnostics.
                        size(); ++index)
            {
                const GeometrySparseFlowRemovedAssociationRecord &record =
                    mvGeometrySparseFlowRemovedAssociationDiagnostics[
                        index];
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.featureIndex << ","
                       << record.pixelX << ","
                       << record.pixelY << ","
                       << record.mapPointId << ","
                       << record.qThreshold << ","
                       << record.frameScalePixels << ","
                       << (record.semanticDynamic ? 1 : 0)
                       << ",applied,association_removed\n";
            }
            stream.close();
            cout << "[Geometry G1-F1] saved "
                 << mvGeometrySparseFlowRemovedAssociationDiagnostics.
                        size()
                 << " exact removed-association rows to "
                 << mGeometrySparseFlowRemovedAssociationCsvPath
                 << endl;
        }
    }

    if(!mGeometrySparseFlowCandidateAssociationCsvPath.empty())
    {
        ofstream stream(
            mGeometrySparseFlowCandidateAssociationCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G1-F1] failed to open post-search "
                 << "candidate-association CSV: "
                 << mGeometrySparseFlowCandidateAssociationCsvPath
                 << endl;
        }
        else
        {
            stream
                << "frame,timestamp,feature_index,u,v,mappoint_id,"
                << "q_threshold,frame_scale_px,semantic_dynamic,"
                << "removed,filter_state\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<
                    mvGeometrySparseFlowCandidateAssociationDiagnostics.
                        size(); ++index)
            {
                const GeometrySparseFlowCandidateAssociationRecord &record =
                    mvGeometrySparseFlowCandidateAssociationDiagnostics[
                        index];
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.featureIndex << ","
                       << record.pixelX << ","
                       << record.pixelY << ","
                       << record.mapPointId << ","
                       << record.qThreshold << ","
                       << record.frameScalePixels << ","
                       << (record.semanticDynamic ? 1 : 0) << ","
                       << (record.removed ? 1 : 0) << ","
                       << record.filterState << "\n";
            }
            stream.close();
            cout << "[Geometry G1-F1] saved "
                 << mvGeometrySparseFlowCandidateAssociationDiagnostics.
                        size()
                 << " post-search candidate-association rows to "
                 << mGeometrySparseFlowCandidateAssociationCsvPath
                 << endl;
        }
    }

    if(!mGeometrySparseFlowMappingCounterfactualCsvPath.empty() &&
       !mvGeometrySparseFlowMappingAdmissionDiagnostics.empty())
    {
        ofstream stream(
            mGeometrySparseFlowMappingCounterfactualCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G1-M0] failed to open mapping "
                 << "counterfactual CSV: "
                 << mGeometrySparseFlowMappingCounterfactualCsvPath
                 << endl;
        }
        else
        {
            stream
                << "frame,timestamp,stage,q_threshold,scale_valid,"
                << "candidate_vector_valid,candidate_state,"
                << "feature_count,candidate_features,"
                << "candidate_associations_before_mapping,"
                << "candidate_tracking_removals,"
                << "valid_depth_features,"
                << "candidate_valid_depth_features,"
                << "depth_admission_features,"
                << "candidate_depth_admission_features,"
                << "created_mappoints,"
                << "candidate_created_mappoints,"
                << "recreated_after_tracking_removal,"
                << "counterfactual_only,"
                << "direct_mapping_state_mutation,mapping_veto\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<
                    mvGeometrySparseFlowMappingAdmissionDiagnostics.
                        size(); ++index)
            {
                const GeometrySparseFlowMappingAdmissionRecord &record =
                    mvGeometrySparseFlowMappingAdmissionDiagnostics[
                        index];
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.stage << ","
                       << record.qThreshold << ","
                       << (record.scaleValid ? 1 : 0) << ","
                       << (record.candidateVectorValid ? 1 : 0)
                       << ","
                       << record.candidateState << ","
                       << record.featureCount << ","
                       << record.candidateFeatures << ","
                       << record.
                              candidateAssociationsBeforeMapping
                       << ","
                       << record.candidateTrackingRemovals << ","
                       << record.validDepthFeatures << ","
                       << record.candidateValidDepthFeatures << ","
                       << record.depthAdmissionFeatures << ","
                       << record.
                              candidateDepthAdmissionFeatures
                       << ","
                       << record.createdMapPoints << ","
                       << record.candidateCreatedMapPoints << ","
                       << record.recreatedAfterTrackingRemoval
                       << ",true,none,none\n";
            }
            stream.close();
            cout << "[Geometry G1-M0] saved "
                 << mvGeometrySparseFlowMappingAdmissionDiagnostics.
                        size()
                 << " mapping-admission rows to "
                 << mGeometrySparseFlowMappingCounterfactualCsvPath
                 << endl;
        }
    }

    if(!mGeometrySparseFlowMappingFilterCsvPath.empty() &&
       !mvGeometrySparseFlowMappingFilterDiagnostics.empty())
    {
        ofstream stream(
            mGeometrySparseFlowMappingFilterCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G1-M1] failed to open mapping "
                 << "filter CSV: "
                 << mGeometrySparseFlowMappingFilterCsvPath
                 << endl;
        }
        else
        {
            stream
                << "frame,timestamp,stage,q_threshold,scale_valid,"
                << "candidate_vector_valid,"
                << "tracking_safeguards_passed,"
                << "tracking_safeguard_state,"
                << "feature_count,available_features,"
                << "candidate_features,candidate_feature_fraction,"
                << "valid_depth_features,"
                << "candidate_valid_depth_features,"
                << "candidate_depth_fraction,"
                << "maximum_feature_fraction,"
                << "maximum_depth_fraction,"
                << "minimum_remaining_depth_features,"
                << "remaining_valid_depth_features,"
                << "candidate_associations_before_veto,"
                << "candidate_tracking_removals,"
                << "new_dynamic_flags,removed_associations,"
                << "vetoed_depth_features,created_mappoints,"
                << "candidate_created_mappoints,applied,state,"
                << "mapping_filter,pose_reoptimization\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<
                    mvGeometrySparseFlowMappingFilterDiagnostics.
                        size(); ++index)
            {
                const GeometrySparseFlowMappingFilterRecord &record =
                    mvGeometrySparseFlowMappingFilterDiagnostics[
                        index];
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.stage << ","
                       << record.qThreshold << ","
                       << (record.scaleValid ? 1 : 0) << ","
                       << (record.candidateVectorValid ? 1 : 0)
                       << ","
                       << (record.trackingSafeguardsPassed
                               ? 1 : 0)
                       << ","
                       << record.trackingSafeguardState << ","
                       << record.featureCount << ","
                       << record.availableFeatures << ","
                       << record.candidateFeatures << ","
                       << record.candidateFeatureFraction << ","
                       << record.validDepthFeatures << ","
                       << record.candidateValidDepthFeatures
                       << ","
                       << record.candidateDepthFraction << ","
                       << record.maximumFeatureFraction << ","
                       << record.maximumDepthFraction << ","
                       << record.minimumRemainingDepthFeatures
                       << ","
                       << record.remainingValidDepthFeatures << ","
                       << record.candidateAssociationsBeforeVeto
                       << ","
                       << record.candidateTrackingRemovals << ","
                       << record.newDynamicFlags << ","
                       << record.removedAssociations << ","
                       << record.vetoedDepthFeatures << ","
                       << record.createdMapPoints << ","
                       << record.candidateCreatedMapPoints << ","
                       << (record.applied ? 1 : 0) << ","
                       << record.state
                       << ",mvbDynamic,none\n";
            }
            stream.close();
            cout << "[Geometry G1-M1] saved "
                 << mvGeometrySparseFlowMappingFilterDiagnostics.
                        size()
                 << " mapping-filter rows to "
                 << mGeometrySparseFlowMappingFilterCsvPath
                 << endl;
        }
    }

    if(mbGeometrySparseFlowMapQualityAuditEnabled &&
       !mGeometrySparseFlowMapQualityPrefix.empty())
    {
        const std::string lifecyclePath =
            mGeometrySparseFlowMapQualityPrefix+
            "_candidate_lifecycle.csv";
        const std::string summaryPath =
            mGeometrySparseFlowMapQualityPrefix+
            "_summary.csv";
        const vector<MapPoint*> finalMapPoints =
            mpMap->GetAllMapPoints();
        const vector<KeyFrame*> finalKeyFrames =
            mpMap->GetAllKeyFrames();
        const std::set<MapPoint*> finalMapPointSet(
            finalMapPoints.begin(),finalMapPoints.end());

        std::vector<int> finalObservationCounts;
        finalObservationCounts.reserve(finalMapPoints.size());
        std::size_t finalObservationTotal = 0;
        for(std::size_t index=0;
            index<finalMapPoints.size(); ++index)
        {
            const int observations =
                finalMapPoints[index]->Observations();
            finalObservationCounts.push_back(observations);
            if(observations>0)
            {
                finalObservationTotal +=
                    static_cast<std::size_t>(observations);
            }
        }
        std::sort(finalObservationCounts.begin(),
                  finalObservationCounts.end());
        double medianObservations = 0.0;
        if(!finalObservationCounts.empty())
        {
            const std::size_t middle =
                finalObservationCounts.size()/2;
            medianObservations =
                finalObservationCounts.size()%2!=0
                ? static_cast<double>(
                      finalObservationCounts[middle])
                : 0.5*static_cast<double>(
                      finalObservationCounts[middle-1]+
                      finalObservationCounts[middle]);
        }
        const double meanObservations =
            finalMapPoints.empty()
            ? 0.0
            : static_cast<double>(finalObservationTotal)/
                static_cast<double>(finalMapPoints.size());

        std::size_t originalSurvivors = 0;
        std::size_t resolvedSurvivors = 0;
        std::size_t replacementSurvivors = 0;
        ofstream lifecycleStream(lifecyclePath.c_str());
        if(!lifecycleStream.is_open())
        {
            cerr << "[Geometry Map Quality] failed to open "
                 << lifecyclePath << endl;
        }
        else
        {
            lifecycleStream
                << "frame,timestamp,feature_index,pixel_x,pixel_y,"
                << "depth_m,mode,mapping_state,original_mappoint_id,"
                << "original_in_final_map,original_bad,"
                << "original_observations,replacement_depth,"
                << "resolved_mappoint_id,resolved_in_final_map,"
                << "resolved_bad,resolved_observations,"
                << "proxy_survived\n";
            lifecycleStream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometrySparseFlowCandidateMapPoints.size();
                ++index)
            {
                const GeometrySparseFlowCandidateMapPointRecord &record =
                    mvGeometrySparseFlowCandidateMapPoints[index];
                MapPoint *original = record.originalMapPoint;
                const bool originalInFinal =
                    original &&
                    finalMapPointSet.count(original)>0;
                const bool originalBad =
                    !original || original->isBad();
                const int originalObservations =
                    original ? original->Observations() : 0;

                MapPoint *resolved = original;
                std::set<MapPoint*> replacementChain;
                std::size_t replacementDepth = 0;
                while(resolved && replacementDepth<32 &&
                      replacementChain.insert(resolved).second)
                {
                    MapPoint *replacement =
                        resolved->GetReplaced();
                    if(!replacement)
                        break;
                    resolved = replacement;
                    ++replacementDepth;
                }
                const bool resolvedInFinal =
                    resolved &&
                    finalMapPointSet.count(resolved)>0;
                const bool resolvedBad =
                    !resolved || resolved->isBad();
                const int resolvedObservations =
                    resolved ? resolved->Observations() : 0;
                const bool proxySurvived =
                    resolvedInFinal && !resolvedBad;
                if(originalInFinal && !originalBad)
                    ++originalSurvivors;
                if(proxySurvived)
                    ++resolvedSurvivors;
                if(proxySurvived && replacementDepth>0)
                    ++replacementSurvivors;

                lifecycleStream
                    << record.frameId << ","
                    << record.timestamp << ","
                    << record.featureIndex << ","
                    << record.pixelX << ","
                    << record.pixelY << ","
                    << record.depthMeters << ","
                    << record.mode << ","
                    << record.mappingState << ","
                    << record.originalMapPointId << ","
                    << (originalInFinal ? 1 : 0) << ","
                    << (originalBad ? 1 : 0) << ","
                    << originalObservations << ","
                    << replacementDepth << ","
                    << (resolved ? resolved->mnId : 0) << ","
                    << (resolvedInFinal ? 1 : 0) << ","
                    << (resolvedBad ? 1 : 0) << ","
                    << resolvedObservations << ","
                    << (proxySurvived ? 1 : 0) << "\n";
            }
            lifecycleStream.close();
        }

        ofstream summaryStream(summaryPath.c_str());
        if(!summaryStream.is_open())
        {
            cerr << "[Geometry Map Quality] failed to open "
                 << summaryPath << endl;
        }
        else
        {
            const std::size_t candidateCreated =
                mvGeometrySparseFlowCandidateMapPoints.size();
            const std::size_t candidateCulled =
                candidateCreated>=resolvedSurvivors
                ? candidateCreated-resolvedSurvivors : 0;
            const double survivorRatio =
                candidateCreated>0
                ? static_cast<double>(resolvedSurvivors)/
                    static_cast<double>(candidateCreated)
                : 0.0;
            summaryStream
                << "mode,final_mappoints,final_keyframes,"
                << "final_observation_total,mean_observations,"
                << "median_observations,candidate_created,"
                << "candidate_original_survivors,"
                << "candidate_resolved_survivors,"
                << "candidate_replacement_survivors,"
                << "candidate_culled_or_not_surviving,"
                << "candidate_proxy_survival_ratio,"
                << "read_only,direct_map_mutation\n";
            summaryStream << std::setprecision(15)
                << (mbGeometrySparseFlowMappingCounterfactualEnabled
                        ? "g1_m0_counterfactual"
                        : "g1_m1_filter") << ","
                << finalMapPoints.size() << ","
                << finalKeyFrames.size() << ","
                << finalObservationTotal << ","
                << meanObservations << ","
                << medianObservations << ","
                << candidateCreated << ","
                << originalSurvivors << ","
                << resolvedSurvivors << ","
                << replacementSurvivors << ","
                << candidateCulled << ","
                << survivorRatio
                << ",true,none\n";
            summaryStream.close();
            cout << "[Geometry Map Quality] final_mappoints="
                 << finalMapPoints.size()
                 << " final_keyframes=" << finalKeyFrames.size()
                 << " candidate_created=" << candidateCreated
                 << " candidate_survived=" << resolvedSurvivors
                 << " candidate_survival_ratio=" << survivorRatio
                 << " read_only=true direct_map_mutation=none"
                 << endl;
        }
    }

    if(!mGeometryLocalRigidityCsvPath.empty() &&
       !mvGeometryLocalRigidityNodeDiagnostics.empty())
    {
        ofstream stream(mGeometryLocalRigidityCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-4F3] failed to open local-rigidity "
                 << "node CSV: "
                 << mGeometryLocalRigidityCsvPath << endl;
        }
        else
        {
            stream
                << "frame,timestamp,reference_frame,"
                << "reference_timestamp,dt_seconds,feature_index,"
                << "u_current,v_current,u_reference,v_reference,"
                << "octave,has_mappoint,semantic_nonzero,"
                << "forward_backward_error_px,"
                << "flow_residual_magnitude_px,"
                << "x_reference_m,y_reference_m,z_reference_m,"
                << "x_current_m,y_current_m,z_current_m,"
                << "reference_depth_uncertainty_std_m,"
                << "current_depth_uncertainty_std_m,"
                << "reference_depth_neighborhood_valid_weight,"
                << "current_depth_neighborhood_valid_weight,"
                << "valid_neighbors,"
                << "absolute_strain_median_m,"
                << "absolute_strain_p90_m,"
                << "relative_strain_median,"
                << "relative_strain_p90,"
                << "uncertainty_normalized_strain_median,"
                << "uncertainty_normalized_strain_p90,"
                << "evidence_state\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<
                    mvGeometryLocalRigidityNodeDiagnostics.size();
                ++index)
            {
                const GeometryLocalRigidityNodeRecord &record =
                    mvGeometryLocalRigidityNodeDiagnostics[index];
                const GeometricRigidityNodeSample &sample =
                    record.sample;
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.referenceFrameId << ","
                       << record.referenceTimestamp << ","
                       << record.timestamp-
                          record.referenceTimestamp << ","
                       << sample.featureIndex << ","
                       << sample.currentPixel.x << ","
                       << sample.currentPixel.y << ","
                       << sample.referencePixel.x << ","
                       << sample.referencePixel.y << ","
                       << record.octave << ","
                       << (record.hasMapPoint ? 1 : 0) << ","
                       << (record.semanticNonzero ? 1 : 0) << ","
                       << sample.forwardBackwardErrorPixels << ","
                       << sample.flowResidualMagnitudePixels << ","
                       << sample.referencePointMeters.x << ","
                       << sample.referencePointMeters.y << ","
                       << sample.referencePointMeters.z << ","
                       << sample.currentPointMeters.x << ","
                       << sample.currentPointMeters.y << ","
                       << sample.currentPointMeters.z << ","
                       << sample.
                          referenceDepthUncertaintyStdMeters << ","
                       << sample.
                          currentDepthUncertaintyStdMeters << ","
                       << sample.
                          referenceDepthNeighborhoodValidWeight << ","
                       << sample.
                          currentDepthNeighborhoodValidWeight << ","
                       << sample.validNeighborCount << ","
                       << sample.
                          incidentAbsoluteStrainMedianMeters << ","
                       << sample.
                          incidentAbsoluteStrainP90Meters << ","
                       << sample.incidentRelativeStrainMedian << ","
                       << sample.incidentRelativeStrainP90 << ","
                       << sample.
                          incidentUncertaintyNormalizedStrainMedian
                       << ","
                       << sample.
                          incidentUncertaintyNormalizedStrainP90
                       << ","
                       << GeometricDynamicDetector::
                          RigidityNodeStateName(sample.state)
                       << "\n";
            }
            stream.close();
            cout << "[Geometry G2-4F3] saved "
                 << mvGeometryLocalRigidityNodeDiagnostics.size()
                 << " local-rigidity node rows to "
                 << mGeometryLocalRigidityCsvPath << endl;
        }
    }

    if(!mGeometryLocalRigidityCsvPath.empty() &&
       !mvGeometryLocalRigidityEdgeDiagnostics.empty())
    {
        const std::string edgeCsvPath =
            mGeometryLocalRigidityCsvPath+".edges.csv";
        ofstream stream(edgeCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-4F3] failed to open local-rigidity "
                 << "edge CSV: " << edgeCsvPath << endl;
        }
        else
        {
            stream
                << "frame,timestamp,reference_frame,"
                << "reference_timestamp,dt_seconds,"
                << "feature_index_a,feature_index_b,"
                << "reference_distance_m,current_distance_m,"
                << "absolute_strain_m,relative_strain,"
                << "delta_length_uncertainty_std_m,"
                << "uncertainty_normalized_strain,"
                << "flow_residual_a_px,flow_residual_b_px,"
                << "forward_backward_error_a_px,"
                << "forward_backward_error_b_px,"
                << "has_mappoint_a,has_mappoint_b,"
                << "semantic_nonzero_a,semantic_nonzero_b,"
                << "dynamic_decision,"
                << "direct_slam_state_mutation\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<
                    mvGeometryLocalRigidityEdgeDiagnostics.size();
                ++index)
            {
                const GeometryLocalRigidityEdgeRecord &record =
                    mvGeometryLocalRigidityEdgeDiagnostics[index];
                const GeometricRigidityEdgeSample &sample =
                    record.sample;
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.referenceFrameId << ","
                       << record.referenceTimestamp << ","
                       << record.timestamp-
                          record.referenceTimestamp << ","
                       << sample.featureIndexA << ","
                       << sample.featureIndexB << ","
                       << sample.referenceDistanceMeters << ","
                       << sample.currentDistanceMeters << ","
                       << sample.absoluteStrainMeters << ","
                       << sample.relativeStrain << ","
                       << sample.deltaLengthUncertaintyStdMeters
                       << ","
                       << sample.uncertaintyNormalizedStrain
                       << ","
                       << sample.flowResidualMagnitudePixelsA
                       << ","
                       << sample.flowResidualMagnitudePixelsB
                       << ","
                       << sample.forwardBackwardErrorPixelsA
                       << ","
                       << sample.forwardBackwardErrorPixelsB
                       << ","
                       << (record.hasMapPointA ? 1 : 0) << ","
                       << (record.hasMapPointB ? 1 : 0) << ","
                       << (record.semanticNonzeroA ? 1 : 0)
                       << ","
                       << (record.semanticNonzeroB ? 1 : 0)
                       << ",none,none\n";
            }
            stream.close();
            cout << "[Geometry G2-4F3] saved "
                 << mvGeometryLocalRigidityEdgeDiagnostics.size()
                 << " local-rigidity edge rows to "
                 << edgeCsvPath << endl;
        }
    }

    if(!mGeometryLocalRigidityCsvPath.empty() &&
       !mvGeometryLocalRigidityFrameDiagnostics.empty())
    {
        const std::string frameCsvPath =
            mGeometryLocalRigidityCsvPath+".frames.csv";
        ofstream stream(frameCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-4F3] failed to open local-rigidity "
                 << "frame CSV: " << frameCsvPath << endl;
        }
        else
        {
            stream
                << "frame,timestamp,reference_frame,"
                << "reference_timestamp,dt_seconds,"
                << "reference_available,domain_valid,input_features,"
                << "sparse_flow_measured,fb_rejected,"
                << "semantic_excluded,current_depth_invalid,"
                << "uncertainty_invalid,"
                << "outside_image,duplicate_image_point,"
                << "eligible_nodes,nodes_with_edges,valid_edges,"
                << "uncertainty_normalized_edges,"
                << "uncertainty_floor_uses,"
                << "axial_depth_noise_coefficient_per_m,"
                << "uncertainty_denominator_floor_m,"
                << "graph_ms,metric_ms,total_ms,"
                << "dynamic_decision,"
                << "direct_slam_state_mutation\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<
                    mvGeometryLocalRigidityFrameDiagnostics.size();
                ++index)
            {
                const GeometryLocalRigidityFrameRecord &record =
                    mvGeometryLocalRigidityFrameDiagnostics[index];
                const GeometricRigidityStats &stats =
                    record.stats;
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.referenceFrameId << ","
                       << record.referenceTimestamp << ","
                       << record.timestamp-
                          record.referenceTimestamp << ","
                       << (record.referenceAvailable ? 1 : 0)
                       << ","
                       << (record.domainValid ? 1 : 0) << ","
                       << stats.inputFeatureCount << ","
                       << stats.sparseFlowMeasuredCount << ","
                       << stats.forwardBackwardRejectedCount << ","
                       << stats.semanticExcludedCount << ","
                       << stats.currentDepthInvalidCount << ","
                       << stats.uncertaintyInvalidCount << ","
                       << stats.outsideImageCount << ","
                       << stats.duplicateImagePointCount << ","
                       << stats.eligibleNodeCount << ","
                       << stats.nodeWithEdgeCount << ","
                       << stats.validEdgeCount << ","
                       << stats.uncertaintyNormalizedEdgeCount
                       << ","
                       << stats.uncertaintyFloorUseCount << ","
                       << stats.axialDepthNoiseCoefficientPerMeter
                       << ","
                       << stats.uncertaintyDenominatorFloorMeters
                       << ","
                       << stats.graphMs << ","
                       << stats.metricMs << ","
                       << stats.totalMs
                       << ",none,none\n";
            }
            stream.close();
            cout << "[Geometry G2-4F3] saved "
                 << mvGeometryLocalRigidityFrameDiagnostics.size()
                 << " local-rigidity frame rows to "
                 << frameCsvPath << endl;
        }
    }

    if(!mGeometryRigidHypothesisCsvPath.empty() &&
       !mvGeometryRigidHypothesisDiagnostics.empty())
    {
        ofstream stream(mGeometryRigidHypothesisCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-MH1] failed to open hypothesis CSV: "
                 << mGeometryRigidHypothesisCsvPath << endl;
        }
        else
        {
            stream
                << "frame,timestamp,reference_frame,"
                << "reference_timestamp,dt_seconds,anchor_feature_index,"
                << "u_current,v_current,u_reference,v_reference,"
                << "member_feature_indices,member_count,"
                << "validation_feature_indices,validation_count,"
                << "h00,h01,h02,h03,h10,h11,h12,h13,"
                << "h20,h21,h22,h23,"
                << "local_fit_median_m,local_fit_rms_m,local_fit_p90_m,"
                << "background_fit_median_m,background_fit_rms_m,"
                << "background_fit_p90_m,median_improvement_m,"
                << "background_to_local_rms_ratio,"
                << "relative_translation_m,relative_rotation_rad,"
                << "maximum_image_radius_px,reference_depth_span_m,"
                << "current_depth_span_m,"
                << "reference_second_to_first_singular_ratio,"
                << "validation_local_fit_median_m,"
                << "validation_local_fit_rms_m,"
                << "validation_local_fit_p90_m,"
                << "validation_background_fit_median_m,"
                << "validation_background_fit_rms_m,"
                << "validation_background_fit_p90_m,"
                << "validation_median_improvement_m,"
                << "validation_background_to_local_rms_ratio,"
                << "validation_local_better_fraction,"
                << "global_validation_count,global_local_better_count,"
                << "global_local_better_fraction,"
                << "global_median_improvement_m,"
                << "evidence_state,validation_state,dynamic_decision,"
                << "direct_slam_state_mutation\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometryRigidHypothesisDiagnostics.size();
                ++index)
            {
                const GeometryRigidHypothesisRecord &record =
                    mvGeometryRigidHypothesisDiagnostics[index];
                const GeometricRigidHypothesisSample &sample =
                    record.sample;
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.referenceFrameId << ","
                       << record.referenceTimestamp << ","
                       << record.timestamp-
                          record.referenceTimestamp << ","
                       << sample.anchorFeatureIndex << ","
                       << sample.anchorCurrentPixel.x << ","
                       << sample.anchorCurrentPixel.y << ","
                       << sample.anchorReferencePixel.x << ","
                       << sample.anchorReferencePixel.y << ","
                       << JoinDiagnosticValues(
                              sample.memberFeatureIndices) << ","
                       << sample.memberFeatureIndices.size() << ","
                       << JoinDiagnosticValues(
                              sample.validationFeatureIndices) << ","
                       << sample.validationFeatureIndices.size() << ","
                       << sample.referenceToCurrent(0,0) << ","
                       << sample.referenceToCurrent(0,1) << ","
                       << sample.referenceToCurrent(0,2) << ","
                       << sample.referenceToCurrent(0,3) << ","
                       << sample.referenceToCurrent(1,0) << ","
                       << sample.referenceToCurrent(1,1) << ","
                       << sample.referenceToCurrent(1,2) << ","
                       << sample.referenceToCurrent(1,3) << ","
                       << sample.referenceToCurrent(2,0) << ","
                       << sample.referenceToCurrent(2,1) << ","
                       << sample.referenceToCurrent(2,2) << ","
                       << sample.referenceToCurrent(2,3) << ","
                       << sample.localFitMedianMeters << ","
                       << sample.localFitRmsMeters << ","
                       << sample.localFitP90Meters << ","
                       << sample.backgroundFitMedianMeters << ","
                       << sample.backgroundFitRmsMeters << ","
                       << sample.backgroundFitP90Meters << ","
                       << sample.medianImprovementMeters << ","
                       << sample.backgroundToLocalRmsRatio << ","
                       << sample.relativeTranslationMeters << ","
                       << sample.relativeRotationRadians << ","
                       << sample.maximumImageRadiusPixels << ","
                       << sample.referenceDepthSpanMeters << ","
                       << sample.currentDepthSpanMeters << ","
                       << sample.referenceSecondToFirstSingularRatio
                       << ","
                       << sample.validationLocalFitMedianMeters << ","
                       << sample.validationLocalFitRmsMeters << ","
                       << sample.validationLocalFitP90Meters << ","
                       << sample.validationBackgroundFitMedianMeters
                       << ","
                       << sample.validationBackgroundFitRmsMeters
                       << ","
                       << sample.validationBackgroundFitP90Meters
                       << ","
                       << sample.validationMedianImprovementMeters
                       << ","
                       << sample.
                          validationBackgroundToLocalRmsRatio << ","
                       << sample.validationLocalBetterFraction << ","
                       << sample.globalValidationCount << ","
                       << sample.globalLocalBetterCount << ","
                       << sample.globalLocalBetterFraction << ","
                       << sample.globalMedianImprovementMeters << ","
                       << GeometricDynamicDetector::
                          RigidHypothesisStateName(sample.state)
                       << ","
                       << GeometricDynamicDetector::
                          RigidHypothesisValidationStateName(
                              sample.validationState)
                       << ",none,none\n";
            }
            stream.close();
            cout << "[Geometry G2-MH1] saved "
                 << mvGeometryRigidHypothesisDiagnostics.size()
                 << " hypothesis rows to "
                 << mGeometryRigidHypothesisCsvPath << endl;
        }
    }

    if(!mGeometryRigidHypothesisCsvPath.empty() &&
       !mvGeometryRigidHypothesisFrameDiagnostics.empty())
    {
        const std::string frameCsvPath =
            mGeometryRigidHypothesisCsvPath+".frames.csv";
        ofstream stream(frameCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-MH1] failed to open frame CSV: "
                 << frameCsvPath << endl;
        }
        else
        {
            stream
                << "frame,timestamp,reference_frame,"
                << "reference_timestamp,dt_seconds,"
                << "reference_available,domain_valid,input_nodes,"
                << "eligible_nodes,valid_hypotheses,"
                << "insufficient_local_support,degenerate_geometry,"
                << "numeric_failure,local_point_count,"
                << "local_validation_point_count,valid_validations,"
                << "insufficient_validation_support,"
                << "numeric_validation_failure,"
                << "neighbor_search_ms,fit_ms,support_evaluation_ms,"
                << "total_ms,"
                << "dynamic_decision,direct_slam_state_mutation\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometryRigidHypothesisFrameDiagnostics.size();
                ++index)
            {
                const GeometryRigidHypothesisFrameRecord &record =
                    mvGeometryRigidHypothesisFrameDiagnostics[index];
                const GeometricRigidHypothesisStats &stats =
                    record.stats;
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.referenceFrameId << ","
                       << record.referenceTimestamp << ","
                       << record.timestamp-
                          record.referenceTimestamp << ","
                       << (record.referenceAvailable ? 1 : 0) << ","
                       << (record.domainValid ? 1 : 0) << ","
                       << stats.inputNodeCount << ","
                       << stats.eligibleNodeCount << ","
                       << stats.validHypothesisCount << ","
                       << stats.insufficientLocalSupportCount << ","
                       << stats.degenerateGeometryCount << ","
                       << stats.numericFailureCount << ","
                       << stats.localPointCount << ","
                       << stats.localValidationPointCount << ","
                       << stats.validValidationCount << ","
                       << stats.insufficientValidationSupportCount
                       << ","
                       << stats.numericValidationFailureCount << ","
                       << stats.neighborSearchMs << ","
                       << stats.fitMs << ","
                       << stats.supportEvaluationMs << ","
                       << stats.totalMs << ",none,none\n";
            }
            stream.close();
            cout << "[Geometry G2-MH1] saved "
                 << mvGeometryRigidHypothesisFrameDiagnostics.size()
                 << " frame rows to " << frameCsvPath << endl;
        }
    }

    if(!mGeometryRegionEvidenceCsvPath.empty() &&
       !mvGeometryRegionEvidenceDiagnostics.empty())
    {
        ofstream stream(mGeometryRegionEvidenceCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-3R1] failed to open region CSV: "
                 << mGeometryRegionEvidenceCsvPath << endl;
        }
        else
        {
            stream << "frame,timestamp,sampling_policy,region_label,"
                   << "region_pixels,semantic_proxy_pixels,"
                   << "semantic_comparison_pixels,"
                   << "semantic_positive_presence_pixels,"
                   << "semantic_negative_presence_pixels,"
                   << "semantic_consistent_presence_pixels,"
                   << "semantic_proxy_region_ratio,"
                   << "semantic_comparison_coverage,"
                   << "semantic_positive_compared_pixel_ratio,"
                   << "comparison_pixels,"
                   << "positive_presence_pixels,"
                   << "negative_presence_pixels,"
                   << "consistent_presence_pixels,"
                   << "comparison_votes,positive_votes,negative_votes,"
                   << "consistent_votes,comparison_coverage,"
                   << "positive_compared_pixel_ratio,"
                   << "negative_compared_pixel_ratio,"
                   << "consistent_compared_pixel_ratio,"
                   << "positive_vote_ratio,negative_vote_ratio,"
                   << "consistent_vote_ratio,"
                   << "single_reference_comparison_pixels,"
                   << "multi_reference_comparison_pixels,"
                   << "single_reference_positive_presence_pixels,"
                   << "multi_reference_positive_presence_pixels,"
                   << "unanimous_positive_pixels,"
                   << "boundary_d1_region_pixels,"
                   << "boundary_d1_comparison_pixels,"
                   << "boundary_d1_positive_presence_pixels,"
                   << "boundary_d1_comparison_votes,"
                   << "boundary_d1_positive_votes,"
                   << "boundary_d2_region_pixels,"
                   << "boundary_d2_comparison_pixels,"
                   << "boundary_d2_positive_presence_pixels,"
                   << "boundary_d2_comparison_votes,"
                   << "boundary_d2_positive_votes,"
                   << "invalid_d1_region_pixels,"
                   << "invalid_d1_comparison_pixels,"
                   << "invalid_d1_positive_presence_pixels,"
                   << "invalid_d1_comparison_votes,"
                   << "invalid_d1_positive_votes,"
                   << "invalid_d2_region_pixels,"
                   << "invalid_d2_comparison_pixels,"
                   << "invalid_d2_positive_presence_pixels,"
                   << "invalid_d2_comparison_votes,"
                   << "invalid_d2_positive_votes,"
                   << "valid_depth_pixels,"
                   << "boundary_pixels,partition_region_count,"
                   << "largest_region_valid_ratio,"
                   << "top_five_region_valid_ratio,domain_scale,"
                   << "partition_ms,mapping_ms,online_region_ms,"
                   << "regions_with_comparison,"
                   << "regions_with_positive,aggregation_ms\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometryRegionEvidenceDiagnostics.size();
                ++index)
            {
                const GeometryRegionEvidenceRecord &record =
                    mvGeometryRegionEvidenceDiagnostics[index];
                const GeometricRegionEvidenceStats &region =
                    record.region;
                const GeometricRegionPartitionStats &partition =
                    record.partitionStats;
                const GeometricRegionEvidenceAggregationStats
                    &aggregation = record.aggregationStats;
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.samplingPolicy << ","
                       << region.regionLabel << ","
                       << region.regionPixels << ","
                       << region.semanticProxyPixels << ","
                       << region.semanticComparisonPixels << ","
                       << region.semanticPositivePresencePixels << ","
                       << region.semanticNegativePresencePixels << ","
                       << region.semanticConsistentPresencePixels << ","
                       << region.semanticProxyRegionRatio << ","
                       << region.semanticComparisonCoverage << ","
                       << region.semanticPositiveComparedPixelRatio << ","
                       << region.comparisonPixels << ","
                       << region.positivePresencePixels << ","
                       << region.negativePresencePixels << ","
                       << region.consistentPresencePixels << ","
                       << region.comparisonVotes << ","
                       << region.positiveVotes << ","
                       << region.negativeVotes << ","
                       << region.consistentVotes << ","
                       << region.comparisonCoverage << ","
                       << region.positiveComparedPixelRatio << ","
                       << region.negativeComparedPixelRatio << ","
                       << region.consistentComparedPixelRatio << ","
                       << region.positiveVoteRatio << ","
                       << region.negativeVoteRatio << ","
                       << region.consistentVoteRatio << ","
                       << region.singleReferenceComparisonPixels << ","
                       << region.multiReferenceComparisonPixels << ","
                       << region.singleReferencePositivePresencePixels << ","
                       << region.multiReferencePositivePresencePixels << ","
                       << region.unanimousPositivePixels << ","
                       << region.boundaryWithinOnePixel.regionPixels << ","
                       << region.boundaryWithinOnePixel.comparisonPixels << ","
                       << region.boundaryWithinOnePixel.
                              positivePresencePixels << ","
                       << region.boundaryWithinOnePixel.comparisonVotes << ","
                       << region.boundaryWithinOnePixel.positiveVotes << ","
                       << region.boundaryWithinTwoPixels.regionPixels << ","
                       << region.boundaryWithinTwoPixels.comparisonPixels << ","
                       << region.boundaryWithinTwoPixels.
                              positivePresencePixels << ","
                       << region.boundaryWithinTwoPixels.comparisonVotes << ","
                       << region.boundaryWithinTwoPixels.positiveVotes << ","
                       << region.invalidWithinOnePixel.regionPixels << ","
                       << region.invalidWithinOnePixel.comparisonPixels << ","
                       << region.invalidWithinOnePixel.
                              positivePresencePixels << ","
                       << region.invalidWithinOnePixel.comparisonVotes << ","
                       << region.invalidWithinOnePixel.positiveVotes << ","
                       << region.invalidWithinTwoPixels.regionPixels << ","
                       << region.invalidWithinTwoPixels.comparisonPixels << ","
                       << region.invalidWithinTwoPixels.
                              positivePresencePixels << ","
                       << region.invalidWithinTwoPixels.comparisonVotes << ","
                       << region.invalidWithinTwoPixels.positiveVotes << ","
                       << partition.validDepthPixels << ","
                       << partition.boundaryPixels << ","
                       << partition.regionCount << ","
                       << partition.largestRegionValidRatio << ","
                       << partition.topFiveRegionValidRatio << ","
                       << partition.domainScale << ","
                       << partition.totalMs << ","
                       << partition.mappingMs << ","
                       << partition.onlineTotalMs << ","
                       << aggregation.regionsWithComparison << ","
                       << aggregation.regionsWithPositiveEvidence << ","
                       << aggregation.totalMs << "\n";
            }
            stream.close();
            cout << "[Geometry G2-3R1] saved "
                 << mvGeometryRegionEvidenceDiagnostics.size()
                 << " region rows to "
                 << mGeometryRegionEvidenceCsvPath << endl;
        }
    }

    if(!mGeometryReferenceSelectionCsvPath.empty() &&
       !mvGeometryReferenceSelectionDiagnostics.empty())
    {
        ofstream stream(mGeometryReferenceSelectionCsvPath.c_str());
        if(!stream.is_open())
        {
            cerr << "[Geometry G2-2R] failed to open selection CSV: "
                 << mGeometryReferenceSelectionCsvPath << endl;
        }
        else
        {
            stream << "frame,timestamp,policy,sampling_policy,"
                   << "requested_reference_count,"
                   << "candidate_count,cached_reference_match_count,"
                   << "selected_reference_count,evidence_computed,"
                   << "selected_frame_ids,selected_covisibility_weights,"
                   << "selected_frame_ages,"
                   << "per_reference_valid_reference_samples,"
                   << "per_reference_projected_samples,"
                   << "per_reference_valid_comparisons,"
                   << "per_reference_prediction_coverage,"
                   << "per_reference_comparison_coverage,"
                   << "per_reference_total_ms,dense_audit_computed,"
                   << "sampled_comparison_pixels,"
                   << "dense_comparison_on_sampled_pixels,"
                   << "sampled_positive_presence_pixels,"
                   << "dense_positive_on_sampled_pixels,"
                   << "both_positive_pixels,"
                   << "positive_presence_agreement_pixels,"
                   << "exact_vote_agreement_pixels\n";
            stream << std::setprecision(15);
            for(std::size_t index=0;
                index<mvGeometryReferenceSelectionDiagnostics.size();
                ++index)
            {
                const GeometryReferenceSelectionRecord &record =
                    mvGeometryReferenceSelectionDiagnostics[index];
                std::vector<std::size_t> projectedSamples;
                std::vector<std::size_t> validReferenceSamples;
                std::vector<std::size_t> validComparisons;
                std::vector<double> predictionCoverage;
                std::vector<double> comparisonCoverage;
                std::vector<double> totalMs;
                for(std::size_t referenceIndex=0;
                    referenceIndex<record.perReference.size();
                    ++referenceIndex)
                {
                    const GeometricWarpStats &stats =
                        record.perReference[referenceIndex].warp;
                    validReferenceSamples.push_back(
                        stats.referenceValidPixels);
                    projectedSamples.push_back(stats.projectedSamples);
                    validComparisons.push_back(stats.validComparisons);
                    predictionCoverage.push_back(
                        stats.predictionCoverageRatio);
                    comparisonCoverage.push_back(
                        stats.comparisonCoverageRatio);
                    totalMs.push_back(stats.totalMs);
                }
                stream << record.frameId << ","
                       << record.timestamp << ","
                       << record.policy << ","
                       << record.samplingPolicy << ","
                       << record.requestedReferenceCount << ","
                       << record.stats.candidateCount << ","
                       << record.stats.cachedReferenceMatchCount << ","
                       << record.stats.selectedReferenceCount << ","
                       << (record.evidenceComputed ? 1 : 0) << ","
                       << JoinDiagnosticValues(
                              record.selectedFrameIds) << ","
                       << JoinDiagnosticValues(
                              record.selectedCovisibilityWeights) << ","
                       << JoinDiagnosticValues(
                              record.selectedFrameAges) << ","
                       << JoinDiagnosticValues(
                              validReferenceSamples) << ","
                       << JoinDiagnosticValues(projectedSamples) << ","
                       << JoinDiagnosticValues(validComparisons) << ","
                       << JoinDiagnosticValues(predictionCoverage) << ","
                       << JoinDiagnosticValues(comparisonCoverage) << ","
                       << JoinDiagnosticValues(totalMs) << ","
                       << (record.denseAuditComputed ? 1 : 0) << ","
                       << record.sampledComparisonPixels << ","
                       << record.denseComparisonOnSampledPixels << ","
                       << record.sampledPositivePresencePixels << ","
                       << record.densePositiveOnSampledPixels << ","
                       << record.bothPositivePixels << ","
                       << record.positivePresenceAgreementPixels << ","
                       << record.exactVoteAgreementPixels << "\n";
            }
            stream.close();
            cout << "[Geometry G2-2R] saved "
                 << mvGeometryReferenceSelectionDiagnostics.size()
                 << " selection rows to "
                 << mGeometryReferenceSelectionCsvPath << endl;
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
    mvGeometryMultiReferenceHistogram.clear();
    mvGeometryMultiReferenceFeatureDiagnostics.clear();
    mvGeometrySparseFlowFeatureDiagnostics.clear();
    mvGeometrySparseFlowFrameDiagnostics.clear();
    mvGeometryAssociationSnapshotDiagnostics.clear();
    mvGeometrySparseFlowTrackingFilterDiagnostics.clear();
    mvGeometrySparseFlowRemovedAssociationDiagnostics.clear();
    mvGeometrySparseFlowCandidateAssociationDiagnostics.clear();
    mvGeometrySparseFlowMappingAdmissionDiagnostics.clear();
    mvGeometrySparseFlowMappingFilterDiagnostics.clear();
    mvGeometrySparseFlowCandidateMapPoints.clear();
    mvGeometryRegionEvidenceDiagnostics.clear();
    mvGeometryReferenceSelectionDiagnostics.clear();
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

    cv::Mat sinStyleDenseFlowGray;
    if(mbSInStyleDenseFlowResidualEnabled)
    {
        if(imRGB.channels()==3)
            cvtColor(imRGB,sinStyleDenseFlowGray,CV_BGR2GRAY);
        else if(imRGB.channels()==4)
            cvtColor(imRGB,sinStyleDenseFlowGray,CV_BGRA2GRAY);
        else
            sinStyleDenseFlowGray = imRGB;
    }

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

    if(mbGeometryShadowEnabled ||
       mbGeometrySparseEgoFlowShadowEnabled ||
       mbJiGeometryShadowEnabled)
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
    mCurrentSInGeometryDynamicMask.release();
    mbCurrentSInGeometryEvidenceAvailable = false;
    RunSInStyleRegionShadow(imDepth,sinStyleDenseFlowGray);
    RunSInStyleDepthFilter(imDepth,semanticMask);

    // DT-SLAM: 传递mask和检测列表给FrameDrawer用于Pangolin可视化
    mpFrameDrawer->UpdateMask(mCurrentFrame.mSemanticMask);

    if(mbJiGeometryReprojectionStatsEnabled)
    {
        mJiInitialTcw.release();
        mvJiInitialObservations.clear();
    }
    Track();
    FinalizeSInStyleDepthFilterForMapping();
    if(mbSInStyleShadowEnabled &&
       !mvSInStyleShadowDiagnostics.empty() &&
       mvSInStyleShadowDiagnostics.back().inputFrameIndex+1==
           mnSInStyleInputFrameIndex)
    {
        mvSInStyleShadowDiagnostics.back().trackingStateAfter =
            static_cast<int>(mState);
    }
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
            const bool updateMultiReference =
                mbGeometryMultiReferenceShadowEnabled &&
                mnLastKeyFrameId==mCurrentFrame.mnId;
            cv::Mat referenceDepth;
            if(mbGeometrySingleReferenceShadowEnabled ||
               updateMultiReference)
            {
                referenceDepth = imDepth.clone();
                if(!semanticMask.empty())
                    referenceDepth.setTo(0.0f,semanticMask);
                if(mbGeometrySingleReferenceShadowEnabled)
                {
                    mGeometricDetector.UpdateReference(
                        referenceDepth,mCurrentFrame.mTcw,
                        mCurrentFrame.mnId,mCurrentFrame.mTimeStamp);
                }
                if(updateMultiReference)
                    UpdateMultiReferenceGeometryHistory(referenceDepth);
            }
            if(mbGeometrySingleReferenceShadowEnabled &&
               !mCurrentGroundTruthTcw.empty())
            {
                mGeometricGroundTruthDetector.UpdateReference(
                    referenceDepth,mCurrentGroundTruthTcw,mCurrentFrame.mnId,
                    mCurrentFrame.mTimeStamp);
            }
            else if(mbGeometrySingleReferenceShadowEnabled)
            {
                mGeometricGroundTruthDetector.ResetReference();
            }
        }
        else if(mbGeometrySingleReferenceShadowEnabled)
        {
            mGeometricDetector.ResetReference();
            mGeometricGroundTruthDetector.ResetReference();
        }
    }

    if(mbGeometrySparseEgoFlowShadowEnabled)
    {
        if(mState==OK && !mCurrentFrame.mTcw.empty())
            UpdateSparseEgoFlowReference();
        else
        {
            mSparseFlowReference.gray.release();
            mSparseFlowReference.depthMeters.release();
            mSparseFlowReference.TcwFinal.release();
            mSparseFlowReference.TcwGroundTruth.release();
            mSparseFlowReference.valid = false;
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

void Tracking::RunSInStyleDepthFilter(
    const cv::Mat &depthMeters, const cv::Mat &semanticDynamicMask)
{
    mCurrentSInDepthFilterResult = SInStyleDepthFilterResult();
    mbCurrentSInDepthMappingAdmissible = false;
    if(!mbSInStyleDepthFilterEnabled)
        return;

    mCurrentSInDepthFilterResult = mSInStyleDepthFilter.Filter(
        depthMeters,semanticDynamicMask,mCurrentSInGeometryDynamicMask,
        mbCurrentSInGeometryEvidenceAvailable);

    SInStyleDepthFilterRecord record;
    record.inputFrameIndex = mnSInStyleDepthFilterInputFrameIndex++;
    record.frameId = mCurrentFrame.mnId;
    record.timestamp = mCurrentFrame.mTimeStamp;
    record.stats = mCurrentSInDepthFilterResult.stats;
    record.trackingStateAfter = -1;
    record.mappingOutputAvailable = false;
    record.maskWritten = false;
    mvSInStyleDepthFilterDiagnostics.push_back(record);
}

void Tracking::FinalizeSInStyleDepthFilterForMapping()
{
    if(!mbSInStyleDepthFilterEnabled)
        return;

    mbCurrentSInDepthMappingAdmissible =
        mCurrentSInDepthFilterResult.stats.available &&
        mState==OK && !mCurrentFrame.mTcw.empty() &&
        !mCurrentSInDepthFilterResult.dynamicDepthMask.empty() &&
        !mCurrentSInDepthFilterResult.staticDepthMeters.empty();

    if(mvSInStyleDepthFilterDiagnostics.empty() ||
       mvSInStyleDepthFilterDiagnostics.back().frameId!=mCurrentFrame.mnId)
    {
        return;
    }

    SInStyleDepthFilterRecord &record =
        mvSInStyleDepthFilterDiagnostics.back();
    record.trackingStateAfter = static_cast<int>(mState);
    record.mappingOutputAvailable =
        mbCurrentSInDepthMappingAdmissible;

    if(mbCurrentSInDepthMappingAdmissible &&
       !mSInStyleDepthFilterOutputDir.empty())
    {
        std::ostringstream path;
        path << mSInStyleDepthFilterOutputDir;
        if(mSInStyleDepthFilterOutputDir[
               mSInStyleDepthFilterOutputDir.size()-1]!='/')
        {
            path << "/";
        }
        path << "frame_" << std::setfill('0') << std::setw(6)
             << record.inputFrameIndex << "_dynamic_depth_mask.png";
        try
        {
            record.maskWritten = cv::imwrite(
                path.str(),mCurrentSInDepthFilterResult.dynamicDepthMask);
            if(!record.maskWritten)
            {
                cerr << "[SIn S3] dynamic-depth mask was not written: "
                     << path.str() << endl;
            }
        }
        catch(const cv::Exception &error)
        {
            cerr << "[SIn S3] dynamic-depth mask write failed: "
                 << error.what() << endl;
        }
    }

    if(mnSInStyleLogEveryN>0 &&
       (mCurrentFrame.mnId%
        static_cast<long unsigned int>(mnSInStyleLogEveryN)==0))
    {
        cout << "[SIn S3] frame=" << mCurrentFrame.mnId
             << " mode=" << record.stats.maskMode
             << " state=" << record.stats.state
             << " geometry_available="
             << record.stats.geometryEvidenceAvailable
             << " rejected_valid_depth="
             << record.stats.rejectedValidDepthPixels
             << " output_valid_depth="
             << record.stats.outputValidDepthPixels
             << " mapping_output_available="
             << record.mappingOutputAvailable
             << " filter_ms=" << record.stats.totalMs
             << " tracking_state_mutation=none" << endl;
    }
}

void Tracking::RunSInStyleRegionShadow(
    const cv::Mat &depthMeters, const cv::Mat &denseFlowGray)
{
    if(!mbSInStyleShadowEnabled)
        return;

    const SInStyleShadowResult result = mSInStyleDetector.Process(
        mImGray,depthMeters,mnSInStyleInputFrameIndex,
        mCurrentFrame.mTimeStamp);
    const SInStyleDenseFlowResidualResult denseFlowResult =
        mSInStyleDenseFlowResidualEstimator.Process(
            mnSInStyleInputFrameIndex,denseFlowGray);
    const SInStyleInitialRegionResult nativeInitialResult =
        mSInStyleInitialRegionClusterer.Compute(
            depthMeters,mnSInStyleInputFrameIndex);
    const SInStyleGradientSplitResult nativeGradientResult =
        mSInStyleGradientRegionSplitter.Compute(
            depthMeters,nativeInitialResult.labels);
    const SInStylePlaneEdgeSplitResult nativePlaneResult =
        mSInStylePlaneEdgeRegionSplitter.Compute(
            depthMeters,nativeInitialResult.labels,
            nativeGradientResult.rawGradientEdgeMask);
    const cv::Mat &ragSplitLabels = nativePlaneResult.stats.available ?
        nativePlaneResult.combinedCoreLabels :
        nativeGradientResult.splitCoreLabels;
    const cv::Mat &ragRealEdgeMask = nativePlaneResult.stats.available ?
        nativePlaneResult.combinedEdgeMask :
        nativeGradientResult.rawGradientEdgeMask;
    const SInStyleRAGMergeResult nativeRAGResult =
        mSInStyleRAGRegionMerger.Compute(
            depthMeters,nativeInitialResult.labels,
            ragSplitLabels,ragRealEdgeMask);
    const cv::Mat &regionDecisionLabels =
        mSInStyleRegionDynamicLabelSource=="native_rag" ?
        nativeRAGResult.mergedLabels : result.regionLabels;
    const cv::Mat &regionDecisionValidity =
        mSInStyleRegionDynamicLabelSource=="native_rag" ?
        nativeInitialResult.validMask : result.referenceRegionValidMask;
    const SInStyleRegionDynamicResult regionDynamicResult =
        mSInStyleRegionDynamicClassifier.Compute(
            mnSInStyleInputFrameIndex,regionDecisionLabels,
            regionDecisionValidity,
            denseFlowResult.lowResidualMask,
            denseFlowResult.highResidualMask);
    if(regionDynamicResult.stats.available &&
       regionDynamicResult.stats.dynamicStateAvailable &&
       !regionDynamicResult.dynamicMask.empty())
    {
        mCurrentSInGeometryDynamicMask =
            regionDynamicResult.dynamicMask.clone();
        mbCurrentSInGeometryEvidenceAvailable = true;
    }
    if(regionDynamicResult.stats.available)
    {
        mSInStyleDenseFlowResidualEstimator.CommitTemporalRegionPrior(
            mnSInStyleInputFrameIndex,
            regionDynamicResult.rawStateMask,
            regionDecisionLabels);
    }

    SInStyleShadowRecord record;
    record.inputFrameIndex = mnSInStyleInputFrameIndex;
    record.frameId = mCurrentFrame.mnId;
    record.resetEpoch = mnSInStyleResetEpoch;
    record.timestamp = mCurrentFrame.mTimeStamp;
    record.referenceBackend = mSInStyleReferenceBackend;
    record.regionDynamicLabelSource =
        mSInStyleRegionDynamicLabelSource;
    record.stats = result.stats;
    record.runtime = result.runtime;
    record.nativeInitialStats = nativeInitialResult.stats;
    record.nativeGradientStats = nativeGradientResult.stats;
    record.nativePlaneStats = nativePlaneResult.stats;
    record.nativeRAGStats = nativeRAGResult.stats;
    record.denseFlowStats = denseFlowResult.stats;
    record.regionDynamicStats = regionDynamicResult.stats;
    record.regionDynamicOrbCount = 0;
    record.regionDynamicAuthorOverlapPixels = 0;
    record.regionDynamicAuthorUnionPixels = 0;
    record.regionDynamicAuthorOverlapOrbCount = 0;
    if(regionDynamicResult.stats.available &&
       result.stats.referenceAvailable)
    {
        cv::Mat overlap;
        cv::Mat unionMask;
        cv::bitwise_and(regionDynamicResult.authorStyleDynamicMask,
                        result.authorDynamicMask,overlap);
        cv::bitwise_or(regionDynamicResult.authorStyleDynamicMask,
                       result.authorDynamicMask,unionMask);
        record.regionDynamicAuthorOverlapPixels =
            static_cast<std::size_t>(cv::countNonZero(overlap));
        record.regionDynamicAuthorUnionPixels =
            static_cast<std::size_t>(cv::countNonZero(unionMask));
    }
    if(regionDynamicResult.stats.available &&
       !mSInStyleRegionDynamicOutputDir.empty())
    {
        std::ostringstream prefix;
        prefix << mSInStyleRegionDynamicOutputDir;
        if(mSInStyleRegionDynamicOutputDir[
               mSInStyleRegionDynamicOutputDir.size()-1]!='/')
        {
            prefix << "/";
        }
        prefix << "frame_" << std::setfill('0') << std::setw(6)
               << mnSInStyleInputFrameIndex;
        try
        {
            cv::imwrite(prefix.str()+"_region_valid.png",
                        regionDynamicResult.validRegionMask);
            cv::imwrite(prefix.str()+"_region_core.png",
                        regionDynamicResult.filledDynamicMaskBeforeDilation);
            cv::imwrite(prefix.str()+"_region_author_style.png",
                        regionDynamicResult.authorStyleDynamicMask);
            cv::imwrite(prefix.str()+"_region_valid_dynamic.png",
                        regionDynamicResult.dynamicMask);
            cv::imwrite(prefix.str()+"_region_low_support.png",
                        regionDynamicResult.lowResidualSupportMask);
        }
        catch(const cv::Exception &error)
        {
            cerr << "[SIn S1] region dynamic audit write failed: "
                 << error.what() << endl;
        }
    }
    record.nativeInitialOrbAssignedCount = 0;
    record.nativeInitialLabelsWritten = false;
    record.nativeGradientEdgeWritten = false;
    record.nativeGradientSplitLabelsWritten = false;
    record.nativePlaneRawBoundaryWritten = false;
    record.nativePlaneRetainedBoundaryWritten = false;
    record.nativeCombinedEdgeWritten = false;
    record.nativeCombinedSplitLabelsWritten = false;
    record.nativeRAGMergedLabelsWritten = false;
    record.rawOrbCount = static_cast<std::size_t>(mCurrentFrame.N);
    record.authorDynamicMaskHitOnDtOrbSet = 0;
    record.depthSupportedDynamicOrbCount = 0;
    record.validOrbCount = 0;
    record.unknownOrbCount = 0;
    record.semanticDynamicOrbCount = 0;
    record.semanticAuthorDynamicOverlapOnDtOrbSet = 0;
    record.regionFeatureFilterEnabled =
        mbSInStyleRegionFeatureFilterEnabled;
    record.regionFeatureFilterApplied = false;
    record.regionFeatureFilterState =
        mbSInStyleRegionFeatureFilterEnabled ?
            "not_evaluated" : "disabled";
    record.regionFeatureFilterCandidateFeatures = 0;
    record.regionFeatureFilterSemanticOverlap = 0;
    record.regionFeatureFilterNewDynamicFeatures = 0;
    record.regionFeatureFilterRemainingFeatures =
        static_cast<std::size_t>(mCurrentFrame.N);
    record.regionFeatureFilterActualRemovedAssociations = 0;
    record.regionFeatureFilterTrackingFailOpen = false;
    record.regionFeatureFilterTrackingFailOpenStage = "none";
    record.regionFeatureFilterTrackingFailOpenClearedFeatures = 0;
    record.regionFeatureFilterMappingFlagsRestored = false;
    mvbCurrentSInRegionDynamicFeatures.assign(
        static_cast<std::size_t>(mCurrentFrame.N),0);
    mvbCurrentSInRegionNewDynamicFeatures.assign(
        static_cast<std::size_t>(mCurrentFrame.N),0);
    mvbCurrentSInRegionRemovedAssociations.assign(
        static_cast<std::size_t>(mCurrentFrame.N),0);
    mbCurrentSInRegionTrackingFailOpen = false;

    for(int index=0; index<mCurrentFrame.N; ++index)
    {
        const int u = static_cast<int>(mCurrentFrame.mvKeys[index].pt.x);
        const int v = static_cast<int>(mCurrentFrame.mvKeys[index].pt.y);
        const bool inImage =
            u>=0 && u<result.dynamicMask.cols &&
            v>=0 && v<result.dynamicMask.rows;
        const bool geometryDynamic =
            inImage && result.dynamicMask.at<unsigned char>(v,u)!=0;
        const bool authorDynamic =
            inImage && result.authorDynamicMask.at<unsigned char>(v,u)!=0;
        const bool geometryValid =
            inImage && result.validMask.at<unsigned char>(v,u)!=0;
        const bool semanticDynamic =
            index<static_cast<int>(mCurrentFrame.mvbSemanticDynamic.size()) &&
            mCurrentFrame.mvbSemanticDynamic[index]!=0;

        if(authorDynamic)
            ++record.authorDynamicMaskHitOnDtOrbSet;
        if(geometryDynamic)
            ++record.depthSupportedDynamicOrbCount;
        if(geometryValid)
            ++record.validOrbCount;
        else
            ++record.unknownOrbCount;
        if(semanticDynamic)
            ++record.semanticDynamicOrbCount;
        if(semanticDynamic && authorDynamic)
            ++record.semanticAuthorDynamicOverlapOnDtOrbSet;
        if(u>=0 && u<regionDynamicResult.dynamicMask.cols &&
           v>=0 && v<regionDynamicResult.dynamicMask.rows &&
           regionDynamicResult.dynamicMask.at<unsigned char>(v,u)!=0)
        {
            mvbCurrentSInRegionDynamicFeatures[
                static_cast<std::size_t>(index)] = 1;
            ++record.regionDynamicOrbCount;
            ++record.regionFeatureFilterCandidateFeatures;
            if(semanticDynamic)
                ++record.regionFeatureFilterSemanticOverlap;
            if(authorDynamic)
                ++record.regionDynamicAuthorOverlapOrbCount;
        }

        if(u>=0 && u<nativeInitialResult.labels.cols &&
           v>=0 && v<nativeInitialResult.labels.rows &&
           nativeInitialResult.labels.at<int>(v,u)>0)
        {
            ++record.nativeInitialOrbAssignedCount;
        }
    }

    if(mbSInStyleRegionFeatureFilterEnabled)
    {
        if(!regionDynamicResult.stats.available ||
           !regionDynamicResult.stats.dynamicStateAvailable)
        {
            record.regionFeatureFilterState =
                "region_unavailable_fail_open";
        }
        else if(record.regionFeatureFilterCandidateFeatures==0)
        {
            record.regionFeatureFilterState = "no_candidates";
        }
        else
        {
            std::size_t combinedDynamic = 0;
            for(int index=0; index<mCurrentFrame.N; ++index)
            {
                const bool semanticOrExisting =
                    index<static_cast<int>(mCurrentFrame.mvbDynamic.size()) &&
                    mCurrentFrame.mvbDynamic[index]!=0;
                const bool geometry =
                    mvbCurrentSInRegionDynamicFeatures[
                        static_cast<std::size_t>(index)]!=0;
                if(semanticOrExisting || geometry)
                    ++combinedDynamic;
            }
            const std::size_t featureCount =
                static_cast<std::size_t>(mCurrentFrame.N);
            const std::size_t remainingWithGeometry =
                combinedDynamic<=featureCount ?
                    featureCount-combinedDynamic : 0;
            if(remainingWithGeometry<static_cast<std::size_t>(
                   mnSInStyleRegionFeatureFilterMinimumRemainingFeatures))
            {
                record.regionFeatureFilterState =
                    "minimum_remaining_features_fail_open";
            }
            else
            {
                for(int index=0; index<mCurrentFrame.N; ++index)
                {
                    if(mvbCurrentSInRegionDynamicFeatures[
                           static_cast<std::size_t>(index)]==0)
                    {
                        continue;
                    }
                    if(mCurrentFrame.mvbDynamic[index]==0)
                    {
                        mCurrentFrame.mvbDynamic[index] = 1;
                        mvbCurrentSInRegionNewDynamicFeatures[
                            static_cast<std::size_t>(index)] = 1;
                        ++record.regionFeatureFilterNewDynamicFeatures;
                    }
                }
                record.regionFeatureFilterApplied =
                    record.regionFeatureFilterNewDynamicFeatures>0;
                record.regionFeatureFilterState =
                    record.regionFeatureFilterApplied ?
                        "applied" : "semantic_overlap_only";
            }
        }

        std::size_t remaining = 0;
        for(int index=0; index<mCurrentFrame.N; ++index)
        {
            if(index>=static_cast<int>(mCurrentFrame.mvbDynamic.size()) ||
               mCurrentFrame.mvbDynamic[index]==0)
            {
                ++remaining;
            }
        }
        record.regionFeatureFilterRemainingFeatures = remaining;
    }

    if(nativeInitialResult.stats.available &&
       !mSInStyleNativeInitialOutputDir.empty())
    {
        cv::Mat encodedLabels(
            nativeInitialResult.labels.size(),CV_16UC1,cv::Scalar(0));
        for(int row=0; row<nativeInitialResult.labels.rows; ++row)
        {
            const int *source = nativeInitialResult.labels.ptr<int>(row);
            unsigned short *destination =
                encodedLabels.ptr<unsigned short>(row);
            for(int col=0; col<nativeInitialResult.labels.cols; ++col)
            {
                if(source[col]>0 &&
                   source[col]<=static_cast<int>(
                       std::numeric_limits<unsigned short>::max()))
                {
                    destination[col] =
                        static_cast<unsigned short>(source[col]);
                }
            }
        }
        std::ostringstream path;
        path << mSInStyleNativeInitialOutputDir;
        if(mSInStyleNativeInitialOutputDir[
               mSInStyleNativeInitialOutputDir.size()-1]!='/')
        {
            path << "/";
        }
        path << "frame_" << std::setfill('0') << std::setw(6)
             << mnSInStyleInputFrameIndex << "_native_initial_labels.png";
        try
        {
            record.nativeInitialLabelsWritten =
                cv::imwrite(path.str(),encodedLabels);
        }
        catch(const cv::Exception &error)
        {
            cerr << "[SIn S1] native initial label write failed: "
                 << error.what() << endl;
        }
    }

    if(nativeGradientResult.stats.available &&
       !mSInStyleNativeGradientOutputDir.empty())
    {
        std::ostringstream edgePath;
        edgePath << mSInStyleNativeGradientOutputDir;
        if(mSInStyleNativeGradientOutputDir[
               mSInStyleNativeGradientOutputDir.size()-1]!='/')
            edgePath << "/";
        edgePath << "frame_" << std::setfill('0') << std::setw(6)
                 << mnSInStyleInputFrameIndex
                 << "_native_gradient_edge.png";
        try
        {
            record.nativeGradientEdgeWritten = cv::imwrite(
                edgePath.str(),nativeGradientResult.rawGradientEdgeMask);
        }
        catch(const cv::Exception &error)
        {
            cerr << "[SIn S1] native gradient edge write failed: "
                 << error.what() << endl;
        }

        if(nativeGradientResult.stats.splitComponentCount<=
           static_cast<int>(std::numeric_limits<unsigned short>::max()))
        {
            cv::Mat encodedLabels(
                nativeGradientResult.splitCoreLabels.size(),
                CV_16UC1,cv::Scalar(0));
            for(int row=0;
                row<nativeGradientResult.splitCoreLabels.rows; ++row)
            {
                const int *source =
                    nativeGradientResult.splitCoreLabels.ptr<int>(row);
                unsigned short *destination =
                    encodedLabels.ptr<unsigned short>(row);
                for(int col=0;
                    col<nativeGradientResult.splitCoreLabels.cols; ++col)
                {
                    if(source[col]>0)
                        destination[col] =
                            static_cast<unsigned short>(source[col]);
                }
            }
            std::ostringstream labelPath;
            labelPath << mSInStyleNativeGradientOutputDir;
            if(mSInStyleNativeGradientOutputDir[
                   mSInStyleNativeGradientOutputDir.size()-1]!='/')
                labelPath << "/";
            labelPath << "frame_" << std::setfill('0') << std::setw(6)
                      << mnSInStyleInputFrameIndex
                      << "_native_gradient_split_labels.png";
            try
            {
                record.nativeGradientSplitLabelsWritten = cv::imwrite(
                    labelPath.str(),encodedLabels);
            }
            catch(const cv::Exception &error)
            {
                cerr << "[SIn S1] native gradient label write failed: "
                     << error.what() << endl;
            }
        }
    }

    if(nativePlaneResult.stats.available &&
       !mSInStyleNativePlaneOutputDir.empty())
    {
        std::string planePrefix = mSInStyleNativePlaneOutputDir;
        if(planePrefix[planePrefix.size()-1]!='/')
            planePrefix += "/";
        std::ostringstream baseName;
        baseName << planePrefix << "frame_" << std::setfill('0')
                 << std::setw(6) << mnSInStyleInputFrameIndex;
        try
        {
            record.nativePlaneRawBoundaryWritten = cv::imwrite(
                baseName.str()+"_native_plane_raw_boundary.png",
                nativePlaneResult.rawPlaneBoundaryMask);
            record.nativePlaneRetainedBoundaryWritten = cv::imwrite(
                baseName.str()+"_native_plane_retained_boundary.png",
                nativePlaneResult.retainedPlaneBoundaryMask);
            record.nativeCombinedEdgeWritten = cv::imwrite(
                baseName.str()+"_native_combined_edge.png",
                nativePlaneResult.combinedEdgeMask);
        }
        catch(const cv::Exception &error)
        {
            cerr << "[SIn S1] native plane edge write failed: "
                 << error.what() << endl;
        }

        if(nativePlaneResult.stats.combinedComponentCount<=
           static_cast<int>(std::numeric_limits<unsigned short>::max()))
        {
            cv::Mat encodedLabels(
                nativePlaneResult.combinedCoreLabels.size(),
                CV_16UC1,cv::Scalar(0));
            for(int row=0;
                row<nativePlaneResult.combinedCoreLabels.rows; ++row)
            {
                const int *source =
                    nativePlaneResult.combinedCoreLabels.ptr<int>(row);
                unsigned short *destination =
                    encodedLabels.ptr<unsigned short>(row);
                for(int col=0;
                    col<nativePlaneResult.combinedCoreLabels.cols; ++col)
                {
                    if(source[col]>0)
                        destination[col] =
                            static_cast<unsigned short>(source[col]);
                }
            }
            try
            {
                record.nativeCombinedSplitLabelsWritten = cv::imwrite(
                    baseName.str()+"_native_combined_split_labels.png",
                    encodedLabels);
            }
            catch(const cv::Exception &error)
            {
                cerr << "[SIn S1] native combined label write failed: "
                     << error.what() << endl;
            }
        }
    }

    if(nativeRAGResult.stats.available &&
       !mSInStyleNativeRAGOutputDir.empty() &&
       nativeRAGResult.stats.outputRegionCount<=
           static_cast<int>(std::numeric_limits<unsigned short>::max()))
    {
        cv::Mat encodedLabels(
            nativeRAGResult.mergedLabels.size(),CV_16UC1,cv::Scalar(0));
        for(int row=0; row<nativeRAGResult.mergedLabels.rows; ++row)
        {
            const int *source = nativeRAGResult.mergedLabels.ptr<int>(row);
            unsigned short *destination =
                encodedLabels.ptr<unsigned short>(row);
            for(int col=0; col<nativeRAGResult.mergedLabels.cols; ++col)
            {
                if(source[col]>0)
                    destination[col] =
                        static_cast<unsigned short>(source[col]);
            }
        }
        std::ostringstream labelPath;
        labelPath << mSInStyleNativeRAGOutputDir;
        if(mSInStyleNativeRAGOutputDir[
               mSInStyleNativeRAGOutputDir.size()-1]!='/')
        {
            labelPath << "/";
        }
        labelPath << "frame_" << std::setfill('0') << std::setw(6)
                  << mnSInStyleInputFrameIndex
                  << "_native_rag_merged_labels.png";
        try
        {
            record.nativeRAGMergedLabelsWritten = cv::imwrite(
                labelPath.str(),encodedLabels);
        }
        catch(const cv::Exception &error)
        {
            cerr << "[SIn S1] native RAG label write failed: "
                 << error.what() << endl;
        }
    }

    record.wouldKeepOrbCount =
        record.rawOrbCount-record.authorDynamicMaskHitOnDtOrbSet;
    // This mirrors the public SIn ORB-extractor safety rule only as a
    // counterfactual statistic over DT-SLAM's unfiltered keypoints.
    record.counterfactualFallbackOnDtOrbSet = record.wouldKeepOrbCount<250;
    record.counterfactualRemovedOnDtOrbSet =
        record.counterfactualFallbackOnDtOrbSet ?
        0 : record.authorDynamicMaskHitOnDtOrbSet;
    record.trackingStateAfter = -1;
    mvSInStyleShadowDiagnostics.push_back(record);
    ++mnSInStyleComputedFrames;
    ++mnSInStyleInputFrameIndex;

    if(mnSInStyleLogEveryN>0 &&
       (mCurrentFrame.mnId%
        static_cast<long unsigned int>(mnSInStyleLogEveryN)==0))
    {
        cout << "[SIn S1] input_index=" << record.inputFrameIndex
             << " frame=" << record.frameId
             << " reference=" << record.stats.referenceAvailable
             << " dynamic_px=" << record.stats.dynamicPixels
             << " unknown_px=" << record.stats.unknownPixels
             << " raw_orb=" << record.rawOrbCount
             << " mask_hit_dt_orb="
             << record.authorDynamicMaskHitOnDtOrbSet
             << " counterfactual_fallback="
             << record.counterfactualFallbackOnDtOrbSet
             << " native_regions="
             << record.nativeInitialStats.producedClusters
             << " native_orb_assigned="
             << record.nativeInitialOrbAssignedCount
             << " gradient_edges="
             << record.nativeGradientStats.rawGradientEdgePixels
             << " split_regions="
             << record.nativeGradientStats.splitComponentCount
             << " plane_count="
             << record.nativePlaneStats.planeCount
             << " retained_plane_edges="
             << record.nativePlaneStats.retainedPlaneBoundaryPixels
             << " combined_regions="
             << record.nativePlaneStats.combinedComponentCount
             << " rag_regions="
             << record.nativeRAGStats.outputRegionCount
             << " rag_merges="
             << record.nativeRAGStats.highMiddleMergeCount+
                    record.nativeRAGStats.lowScoreMergeCount
             << " dense_flow_available="
             << record.denseFlowStats.available
             << " dense_flow_low_px="
             << record.denseFlowStats.lowPixels
             << " dense_flow_high_px="
             << record.denseFlowStats.highPixels
             << " region_dynamic_available="
             << record.regionDynamicStats.available
             << " region_dynamic_px="
             << record.regionDynamicStats.depthSupportedDynamicPixels
             << " region_dynamic_orb="
             << record.regionDynamicOrbCount
             << " region_filter_state="
             << record.regionFeatureFilterState
             << " region_filter_new_dynamic="
             << record.regionFeatureFilterNewDynamicFeatures
             << " actual_removed_so_far="
             << record.regionFeatureFilterActualRemovedAssociations
             << " total_ms=" << record.runtime.totalMs << endl;
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
            if(mbSInStyleRegionFeatureFilterEnabled &&
               i<mvbCurrentSInRegionDynamicFeatures.size() &&
               mvbCurrentSInRegionDynamicFeatures[i]!=0 &&
               i<mvbCurrentSInRegionRemovedAssociations.size() &&
               mvbCurrentSInRegionRemovedAssociations[i]==0)
            {
                mvbCurrentSInRegionRemovedAssociations[i] = 1;
                if(!mvSInStyleShadowDiagnostics.empty() &&
                   mvSInStyleShadowDiagnostics.back().frameId==frame.mnId)
                {
                    ++mvSInStyleShadowDiagnostics.back().
                        regionFeatureFilterActualRemovedAssociations;
                }
            }
            frame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            removed++;
        }
    }
    return removed;
}

bool Tracking::FailOpenSInRegionFeatureFilterForTracking(
    const std::string &stage)
{
    if(!mbSInStyleRegionFeatureFilterEnabled ||
       mbCurrentSInRegionTrackingFailOpen ||
       mvbCurrentSInRegionNewDynamicFeatures.size()!=
           static_cast<std::size_t>(mCurrentFrame.N) ||
       mCurrentFrame.mvbDynamic.size()!=
           static_cast<std::size_t>(mCurrentFrame.N))
    {
        return false;
    }

    std::size_t cleared = 0;
    for(int index=0; index<mCurrentFrame.N; ++index)
    {
        if(mvbCurrentSInRegionNewDynamicFeatures[
               static_cast<std::size_t>(index)]==0)
        {
            continue;
        }
        if(mCurrentFrame.mvbDynamic[index]!=0)
        {
            mCurrentFrame.mvbDynamic[index] = 0;
            ++cleared;
        }
    }
    if(cleared==0)
        return false;

    mbCurrentSInRegionTrackingFailOpen = true;
    if(!mvSInStyleShadowDiagnostics.empty() &&
       mvSInStyleShadowDiagnostics.back().frameId==mCurrentFrame.mnId)
    {
        SInStyleShadowRecord &record =
            mvSInStyleShadowDiagnostics.back();
        record.regionFeatureFilterTrackingFailOpen = true;
        record.regionFeatureFilterTrackingFailOpenStage = stage;
        record.regionFeatureFilterTrackingFailOpenClearedFeatures =
            cleared;
    }

    cout << "[SIn S2] frame=" << mCurrentFrame.mnId
         << " tracking_fail_open_stage=" << stage
         << " cleared_geometry_flags=" << cleared
         << " semantic_flags_preserved=true"
         << " pose_reoptimization=none" << endl;
    return true;
}

void Tracking::RestoreSInRegionFeatureFilterForMapping()
{
    if(!mbCurrentSInRegionTrackingFailOpen ||
       mvbCurrentSInRegionNewDynamicFeatures.size()!=
           static_cast<std::size_t>(mCurrentFrame.N) ||
       mCurrentFrame.mvbDynamic.size()!=
           static_cast<std::size_t>(mCurrentFrame.N))
    {
        return;
    }

    for(int index=0; index<mCurrentFrame.N; ++index)
    {
        if(mvbCurrentSInRegionNewDynamicFeatures[
               static_cast<std::size_t>(index)]!=0)
        {
            mCurrentFrame.mvbDynamic[index] = 1;
        }
    }
    if(!mvSInStyleShadowDiagnostics.empty() &&
       mvSInStyleShadowDiagnostics.back().frameId==mCurrentFrame.mnId)
    {
        mvSInStyleShadowDiagnostics.back().
            regionFeatureFilterMappingFlagsRestored = true;
    }
}

void Tracking::RunSparseEgoFlowShadow()
{
    mCurrentSparseFlowFilterResult =
        GeometricSparseFlowFilterResult();
    mCurrentSparseFlowFilterResult.qualityEligibleMask.assign(
        static_cast<std::size_t>(mCurrentFrame.N),0);
    mCurrentSparseFlowFilterResult.candidateMask.assign(
        static_cast<std::size_t>(mCurrentFrame.N),0);
    mvbCurrentSparseFlowRemovedAssociations.assign(
        static_cast<std::size_t>(mCurrentFrame.N),0);
    mbCurrentSparseFlowTrackingSafeguardsPassed = false;
    mCurrentSparseFlowTrackingSafeguardState =
        "not_evaluated";
    if(!mbGeometrySparseEgoFlowShadowEnabled ||
       mSensor!=System::RGBD ||
       mCurrentFrame.mTcw.empty())
    {
        return;
    }

    const std::chrono::steady_clock::time_point activeStart =
        std::chrono::steady_clock::now();
    const bool referenceAvailable =
        mSparseFlowReference.valid &&
        mSparseFlowReference.frameId+1==mCurrentFrame.mnId &&
        mCurrentFrame.mTimeStamp>
            mSparseFlowReference.timestamp;
    const bool domainValid =
        cv::norm(mDistCoef,cv::NORM_INF)<=1e-8 &&
        mImGray.type()==CV_8UC1 &&
        (!referenceAvailable ||
         (mSparseFlowReference.gray.type()==CV_8UC1 &&
          mSparseFlowReference.depthMeters.type()==CV_32FC1 &&
          mSparseFlowReference.gray.size()==mImGray.size() &&
          mSparseFlowReference.depthMeters.size()==mImGray.size()));

    std::vector<cv::Point2f> currentPixels;
    currentPixels.reserve(mCurrentFrame.mvKeys.size());
    for(std::size_t index=0;
        index<mCurrentFrame.mvKeys.size(); ++index)
    {
        currentPixels.push_back(
            mCurrentFrame.mvKeys[index].pt);
    }

    GeometricSparseFlowResult result;
    result.stats.featureCount = currentPixels.size();
    if(referenceAvailable && domainValid)
    {
        const bool pairedGroundTruth =
            !mSparseFlowReference.TcwGroundTruth.empty() &&
            !mCurrentGroundTruthTcw.empty();
        result =
            GeometricDynamicDetector::ComputeSparseEgoFlow(
                mImGray,mSparseFlowReference.gray,
                mSparseFlowReference.depthMeters,
                currentPixels,
                mSparseFlowReference.TcwFinal,
                mCurrentFrame.mTcw,mGeometryK,
                pairedGroundTruth
                    ? mSparseFlowReference.TcwGroundTruth
                    : cv::Mat(),
                pairedGroundTruth
                    ? mCurrentGroundTruthTcw
                    : cv::Mat(),
                mGeometryRegionRelativeThreshold,
                mGeometryRegionAbsoluteThresholdMeters);
    }
    else
    {
        result.samples.resize(currentPixels.size());
        for(std::size_t index=0;
            index<currentPixels.size(); ++index)
        {
            result.samples[index].featureIndex = index;
            result.samples[index].currentPixel =
                currentPixels[index];
            result.samples[index].evidenceState =
                referenceAvailable
                ? GeometricSparseFlowEvidenceState::DomainInvalid
                : GeometricSparseFlowEvidenceState::
                    ReferenceUnavailable;
        }
    }

    if(mbGeometrySparseFlowTrackingFilterEnabled)
    {
        mCurrentSparseFlowFilterResult =
            GeometricDynamicDetector::
                SelectSparseFlowHighResidualCandidates(
                    result,mCurrentFrame.mvbSemanticDynamic,
                    mfGeometrySparseFlowTrackingFilterQ,
                    0.25f,
                    mnGeometrySparseFlowTrackingFilterMinimumScaleSupport,
                    1.4826f,0.001f);
    }

    GeometricRigidityResult rigidityResult;
    if(mbGeometryLocalRigidityShadowEnabled)
    {
        if(referenceAvailable && domainValid &&
           mCurrentDepthMeters.type()==CV_32FC1 &&
           mCurrentDepthMeters.size()==mImGray.size())
        {
            rigidityResult =
                GeometricDynamicDetector::ComputeLocalRigidity(
                    mSparseFlowReference.depthMeters,
                    mCurrentDepthMeters,mGeometryK,result,
                    mCurrentFrame.mvbSemanticDynamic);
        }
        else
        {
            rigidityResult.stats.inputFeatureCount =
                result.samples.size();
            rigidityResult.nodes.resize(
                result.samples.size());
            for(std::size_t index=0;
                index<result.samples.size(); ++index)
            {
                rigidityResult.nodes[index].featureIndex =
                    result.samples[index].featureIndex;
                rigidityResult.nodes[index].currentPixel =
                    result.samples[index].currentPixel;
                rigidityResult.nodes[index].referencePixel =
                    result.samples[index].referencePixel;
                rigidityResult.nodes[index].state =
                    GeometricRigidityNodeState::
                        SparseFlowInvalid;
            }
        }
    }

    GeometricRigidHypothesisResult rigidHypothesisResult;
    if(mbGeometryRigidHypothesisShadowEnabled)
    {
        if(referenceAvailable && domainValid &&
           !mSparseFlowReference.TcwFinal.empty())
        {
            rigidHypothesisResult =
                GeometricDynamicDetector::ComputeLocalRigidHypotheses(
                    rigidityResult,
                    mSparseFlowReference.TcwFinal,
                    mCurrentFrame.mTcw,7);
        }
        else
        {
            rigidHypothesisResult.stats.inputNodeCount =
                rigidityResult.nodes.size();
            rigidHypothesisResult.stats.localPointCount = 7;
            rigidHypothesisResult.stats.localValidationPointCount = 7;
            rigidHypothesisResult.hypotheses.resize(
                rigidityResult.nodes.size());
            for(std::size_t index=0;
                index<rigidityResult.nodes.size(); ++index)
            {
                rigidHypothesisResult.hypotheses[index].
                    anchorFeatureIndex =
                        rigidityResult.nodes[index].featureIndex;
                rigidHypothesisResult.hypotheses[index].state =
                    GeometricRigidHypothesisState::SparseFlowInvalid;
            }
        }
    }

    const bool recordFeatures =
        !mGeometrySparseFlowCsvPath.empty() &&
        (mGeometrySparseFlowFrameFilter.empty() ||
         mGeometrySparseFlowFrameFilter.count(
             mCurrentFrame.mnId)>0);
    const std::chrono::steady_clock::time_point recordStart =
        std::chrono::steady_clock::now();
    if(recordFeatures)
    {
        for(std::size_t index=0;
            index<result.samples.size(); ++index)
        {
            GeometrySparseFlowFeatureRecord record;
            record.frameId = mCurrentFrame.mnId;
            record.timestamp = mCurrentFrame.mTimeStamp;
            record.referenceFrameId =
                referenceAvailable
                ? mSparseFlowReference.frameId : 0;
            record.referenceTimestamp =
                referenceAvailable
                ? mSparseFlowReference.timestamp : 0.0;
            record.octave =
                index<mCurrentFrame.mvKeys.size()
                ? mCurrentFrame.mvKeys[index].octave : -1;
            record.hasMapPoint =
                index<mCurrentFrame.mvpMapPoints.size() &&
                mCurrentFrame.mvpMapPoints[index]!=NULL;
            record.semanticNonzero =
                index<mCurrentFrame.mvbSemanticDynamic.size() &&
                mCurrentFrame.mvbSemanticDynamic[index]!=0;
            record.qualityEligible =
                index<
                    mCurrentSparseFlowFilterResult.
                        qualityEligibleMask.size() &&
                mCurrentSparseFlowFilterResult.
                    qualityEligibleMask[index]!=0;
            record.candidate =
                index<
                    mCurrentSparseFlowFilterResult.
                        candidateMask.size() &&
                mCurrentSparseFlowFilterResult.
                    candidateMask[index]!=0;
            record.normalizedResidualQ =
                record.qualityEligible &&
                mCurrentSparseFlowFilterResult.scaleValid
                ? result.samples[index].
                    slamResidualMagnitudePixels/
                    mCurrentSparseFlowFilterResult.
                        frameScalePixels
                : std::numeric_limits<float>::quiet_NaN();
            record.sample = result.samples[index];
            mvGeometrySparseFlowFeatureDiagnostics.push_back(
                record);
        }
    }

    const bool recordRigidity =
        mbGeometryLocalRigidityShadowEnabled &&
        !mGeometryLocalRigidityCsvPath.empty() &&
        (mGeometryLocalRigidityFrameFilter.empty() ||
         mGeometryLocalRigidityFrameFilter.count(
             mCurrentFrame.mnId)>0);
    if(recordRigidity)
    {
        for(std::size_t index=0;
            index<rigidityResult.nodes.size(); ++index)
        {
            const GeometricRigidityNodeSample &sample =
                rigidityResult.nodes[index];
            const std::size_t featureIndex =
                sample.featureIndex;
            GeometryLocalRigidityNodeRecord record;
            record.frameId = mCurrentFrame.mnId;
            record.timestamp = mCurrentFrame.mTimeStamp;
            record.referenceFrameId =
                referenceAvailable
                ? mSparseFlowReference.frameId : 0;
            record.referenceTimestamp =
                referenceAvailable
                ? mSparseFlowReference.timestamp : 0.0;
            record.octave =
                featureIndex<mCurrentFrame.mvKeys.size()
                ? mCurrentFrame.mvKeys[featureIndex].octave : -1;
            record.hasMapPoint =
                featureIndex<mCurrentFrame.mvpMapPoints.size() &&
                mCurrentFrame.mvpMapPoints[featureIndex]!=NULL;
            record.semanticNonzero =
                featureIndex<
                    mCurrentFrame.mvbSemanticDynamic.size() &&
                mCurrentFrame.mvbSemanticDynamic[
                    featureIndex]!=0;
            record.sample = sample;
            mvGeometryLocalRigidityNodeDiagnostics.push_back(
                record);
        }
        for(std::size_t index=0;
            index<rigidityResult.edges.size(); ++index)
        {
            GeometryLocalRigidityEdgeRecord record;
            record.frameId = mCurrentFrame.mnId;
            record.timestamp = mCurrentFrame.mTimeStamp;
            record.referenceFrameId =
                referenceAvailable
                ? mSparseFlowReference.frameId : 0;
            record.referenceTimestamp =
                referenceAvailable
                ? mSparseFlowReference.timestamp : 0.0;
            record.sample = rigidityResult.edges[index];
            const std::size_t featureA =
                record.sample.featureIndexA;
            const std::size_t featureB =
                record.sample.featureIndexB;
            record.hasMapPointA =
                featureA<mCurrentFrame.mvpMapPoints.size() &&
                mCurrentFrame.mvpMapPoints[featureA]!=NULL;
            record.hasMapPointB =
                featureB<mCurrentFrame.mvpMapPoints.size() &&
                mCurrentFrame.mvpMapPoints[featureB]!=NULL;
            record.semanticNonzeroA =
                featureA<
                    mCurrentFrame.mvbSemanticDynamic.size() &&
                mCurrentFrame.mvbSemanticDynamic[featureA]!=0;
            record.semanticNonzeroB =
                featureB<
                    mCurrentFrame.mvbSemanticDynamic.size() &&
                mCurrentFrame.mvbSemanticDynamic[featureB]!=0;
            mvGeometryLocalRigidityEdgeDiagnostics.push_back(
                record);
        }
    }
    const bool recordRigidHypotheses =
        mbGeometryRigidHypothesisShadowEnabled &&
        !mGeometryRigidHypothesisCsvPath.empty() &&
        (mGeometryRigidHypothesisFrameFilter.empty() ||
         mGeometryRigidHypothesisFrameFilter.count(
             mCurrentFrame.mnId)>0);
    if(recordRigidHypotheses)
    {
        for(std::size_t index=0;
            index<rigidHypothesisResult.hypotheses.size(); ++index)
        {
            GeometryRigidHypothesisRecord record;
            record.frameId = mCurrentFrame.mnId;
            record.timestamp = mCurrentFrame.mTimeStamp;
            record.referenceFrameId =
                referenceAvailable
                ? mSparseFlowReference.frameId : 0;
            record.referenceTimestamp =
                referenceAvailable
                ? mSparseFlowReference.timestamp : 0.0;
            record.sample =
                rigidHypothesisResult.hypotheses[index];
            mvGeometryRigidHypothesisDiagnostics.push_back(record);
        }
    }
    const double recordMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-recordStart).count();
    const double activeTotalMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-activeStart).count();

    if(!mGeometrySparseFlowCsvPath.empty())
    {
        GeometrySparseFlowFrameRecord frameRecord;
        frameRecord.frameId = mCurrentFrame.mnId;
        frameRecord.timestamp = mCurrentFrame.mTimeStamp;
        frameRecord.referenceFrameId =
            referenceAvailable
            ? mSparseFlowReference.frameId : 0;
        frameRecord.referenceTimestamp =
            referenceAvailable
            ? mSparseFlowReference.timestamp : 0.0;
        frameRecord.referenceAvailable = referenceAvailable;
        frameRecord.domainValid = domainValid;
        frameRecord.recordMs = recordMs;
        frameRecord.activeTotalMs = activeTotalMs;
        frameRecord.stats = result.stats;
        mvGeometrySparseFlowFrameDiagnostics.push_back(
            frameRecord);
    }
    if(mbGeometryLocalRigidityShadowEnabled &&
       !mGeometryLocalRigidityCsvPath.empty())
    {
        GeometryLocalRigidityFrameRecord frameRecord;
        frameRecord.frameId = mCurrentFrame.mnId;
        frameRecord.timestamp = mCurrentFrame.mTimeStamp;
        frameRecord.referenceFrameId =
            referenceAvailable
            ? mSparseFlowReference.frameId : 0;
        frameRecord.referenceTimestamp =
            referenceAvailable
            ? mSparseFlowReference.timestamp : 0.0;
        frameRecord.referenceAvailable = referenceAvailable;
        frameRecord.domainValid = domainValid;
        frameRecord.stats = rigidityResult.stats;
        mvGeometryLocalRigidityFrameDiagnostics.push_back(
            frameRecord);
    }
    if(mbGeometryRigidHypothesisShadowEnabled &&
       !mGeometryRigidHypothesisCsvPath.empty())
    {
        GeometryRigidHypothesisFrameRecord frameRecord;
        frameRecord.frameId = mCurrentFrame.mnId;
        frameRecord.timestamp = mCurrentFrame.mTimeStamp;
        frameRecord.referenceFrameId =
            referenceAvailable
            ? mSparseFlowReference.frameId : 0;
        frameRecord.referenceTimestamp =
            referenceAvailable
            ? mSparseFlowReference.timestamp : 0.0;
        frameRecord.referenceAvailable = referenceAvailable;
        frameRecord.domainValid = domainValid;
        frameRecord.stats = rigidHypothesisResult.stats;
        mvGeometryRigidHypothesisFrameDiagnostics.push_back(
            frameRecord);
    }

    ++mnGeometrySparseFlowComputedFrames;
    if(mnGeometrySparseFlowComputedFrames==1 ||
       mnGeometrySparseFlowComputedFrames%
           static_cast<long unsigned int>(
               mnGeometryLogEveryN)==0)
    {
        cout << "[Geometry G2-4F1] frame="
             << mCurrentFrame.mnId
             << " ref="
             << (referenceAvailable
                 ? std::to_string(
                       mSparseFlowReference.frameId)
                 : "none")
             << " features=" << result.stats.featureCount
             << " backward_lk="
             << result.stats.backwardLkValidCount
             << " forward_lk="
             << result.stats.forwardLkValidCount
             << " depth_valid="
             << result.stats.referenceDepthValidCount
             << " measured="
             << result.stats.slamResidualValidCount
             << " gt_measured="
             << result.stats.groundTruthResidualValidCount
             << " median_px="
             << result.stats.slamResidualMedianPixels
             << " p90_px="
             << result.stats.slamResidualP90Pixels
             << " p95_px="
             << result.stats.slamResidualP95Pixels
             << " active_ms=" << activeTotalMs
             << " dynamic_decision=none"
             << " direct_slam_state_mutation=none"
             << endl;
    }
    if(mbGeometryLocalRigidityShadowEnabled)
    {
        ++mnGeometryLocalRigidityComputedFrames;
        if(mnGeometryLocalRigidityComputedFrames==1 ||
           mnGeometryLocalRigidityComputedFrames%
               static_cast<long unsigned int>(
                   mnGeometryLogEveryN)==0)
        {
            cout << "[Geometry G2-4F3] frame="
                 << mCurrentFrame.mnId
                 << " eligible_nodes="
                 << rigidityResult.stats.eligibleNodeCount
                 << " nodes_with_edges="
                 << rigidityResult.stats.nodeWithEdgeCount
                 << " edges="
                 << rigidityResult.stats.validEdgeCount
                 << " duplicate_nodes="
                 << rigidityResult.stats.
                    duplicateImagePointCount
                 << " total_ms="
                 << rigidityResult.stats.totalMs
                 << " dynamic_decision=none"
                 << " direct_slam_state_mutation=none"
                 << endl;
        }
    }
    if(mbGeometryRigidHypothesisShadowEnabled)
    {
        ++mnGeometryRigidHypothesisComputedFrames;
        if(mnGeometryRigidHypothesisComputedFrames==1 ||
           mnGeometryRigidHypothesisComputedFrames%
               static_cast<long unsigned int>(
                   mnGeometryLogEveryN)==0)
        {
            cout << "[Geometry G2-MH1] frame="
                 << mCurrentFrame.mnId
                 << " eligible_nodes="
                 << rigidHypothesisResult.stats.eligibleNodeCount
                 << " valid_hypotheses="
                 << rigidHypothesisResult.stats.validHypothesisCount
                 << " valid_validations="
                 << rigidHypothesisResult.stats.validValidationCount
                 << " insufficient="
                 << rigidHypothesisResult.stats.
                    insufficientLocalSupportCount
                 << " degenerate="
                 << rigidHypothesisResult.stats.
                    degenerateGeometryCount
                 << " numeric_failure="
                 << rigidHypothesisResult.stats.numericFailureCount
                 << " validation_insufficient="
                 << rigidHypothesisResult.stats.
                    insufficientValidationSupportCount
                 << " validation_numeric_failure="
                 << rigidHypothesisResult.stats.
                    numericValidationFailureCount
                 << " neighbor_ms="
                 << rigidHypothesisResult.stats.neighborSearchMs
                 << " fit_ms="
                 << rigidHypothesisResult.stats.fitMs
                 << " support_ms="
                 << rigidHypothesisResult.stats.supportEvaluationMs
                 << " total_ms="
                 << rigidHypothesisResult.stats.totalMs
                 << " dynamic_decision=none"
                 << " direct_slam_state_mutation=none"
                 << endl;
        }
    }
}

int Tracking::ApplySparseFlowTrackingFilter()
{
    if(!mbGeometrySparseFlowTrackingFilterEnabled)
        return 0;

    GeometrySparseFlowTrackingFilterRecord record;
    record.frameId = mCurrentFrame.mnId;
    record.timestamp = mCurrentFrame.mTimeStamp;
    record.qThreshold = mfGeometrySparseFlowTrackingFilterQ;
    record.scaleValid =
        mCurrentSparseFlowFilterResult.scaleValid;
    record.frameScalePixels =
        mCurrentSparseFlowFilterResult.frameScalePixels;
    record.scaleSupport =
        mCurrentSparseFlowFilterResult.scaleSupport;
    record.qualityEligibleFeatures =
        mCurrentSparseFlowFilterResult.
            qualityEligibleFeatureCount;
    record.candidateFeatures =
        mCurrentSparseFlowFilterResult.candidateFeatureCount;
    record.baselineAssociations = 0;
    record.candidateAssociations = 0;
    record.removedAssociations = 0;
    record.remainingAssociations = 0;
    record.candidateAssociationFraction = 0.0;
    record.withinRelocalizationWindow =
        mCurrentFrame.mnId<
            mnLastRelocFrameId+
                static_cast<long unsigned int>(mMaxFrames);
    record.applied = false;
    record.state = "unknown";

    const std::size_t count =
        static_cast<std::size_t>(mCurrentFrame.N);
    const bool vectorSizesValid =
        mCurrentFrame.mvpMapPoints.size()==count &&
        mCurrentFrame.mvbOutlier.size()==count &&
        mCurrentSparseFlowFilterResult.
            candidateMask.size()==count;
    if(!vectorSizesValid)
    {
        record.state = "vector_size_mismatch_fail_open";
        mbCurrentSparseFlowTrackingSafeguardsPassed = false;
        mCurrentSparseFlowTrackingSafeguardState =
            record.state;
        mvGeometrySparseFlowTrackingFilterDiagnostics.
            push_back(record);
        return 0;
    }

    std::vector<GeometrySparseFlowCandidateAssociationRecord>
        candidateAssociationRecords;
    std::vector<std::size_t> candidateAssociationRecordIndices(
        count,std::numeric_limits<std::size_t>::max());

    for(std::size_t index=0; index<count; ++index)
    {
        MapPoint *mapPoint =
            mCurrentFrame.mvpMapPoints[index];
        if(!mapPoint || mapPoint->isBad())
            continue;
        ++record.baselineAssociations;
        if(mCurrentSparseFlowFilterResult.
               candidateMask[index]!=0)
        {
            ++record.candidateAssociations;
            if(!mGeometrySparseFlowCandidateAssociationCsvPath.empty())
            {
                GeometrySparseFlowCandidateAssociationRecord
                    candidateRecord;
                candidateRecord.frameId = mCurrentFrame.mnId;
                candidateRecord.timestamp =
                    mCurrentFrame.mTimeStamp;
                candidateRecord.featureIndex = index;
                candidateRecord.pixelX =
                    mCurrentFrame.mvKeys[index].pt.x;
                candidateRecord.pixelY =
                    mCurrentFrame.mvKeys[index].pt.y;
                candidateRecord.mapPointId = mapPoint->mnId;
                candidateRecord.qThreshold =
                    mfGeometrySparseFlowTrackingFilterQ;
                candidateRecord.frameScalePixels =
                    mCurrentSparseFlowFilterResult.
                        frameScalePixels;
                candidateRecord.semanticDynamic =
                    index<mCurrentFrame.mvbSemanticDynamic.size() &&
                    mCurrentFrame.mvbSemanticDynamic[index]!=0;
                candidateRecord.removed = false;
                candidateRecord.filterState = "unknown";
                candidateAssociationRecordIndices[index] =
                    candidateAssociationRecords.size();
                candidateAssociationRecords.push_back(
                    candidateRecord);
            }
        }
    }
    record.remainingAssociations =
        record.baselineAssociations;
    if(record.baselineAssociations>0)
    {
        record.candidateAssociationFraction =
            static_cast<double>(
                record.candidateAssociations)/
            static_cast<double>(
                record.baselineAssociations);
    }

    if(!record.scaleValid)
        record.state = "scale_invalid_fail_open";
    else if(record.withinRelocalizationWindow)
        record.state = "relocalization_window_fail_open";
    else if(record.baselineAssociations<
            mnGeometrySparseFlowTrackingFilterMinimumAssociations)
        record.state = "insufficient_associations_fail_open";
    else if(record.candidateAssociations==0)
        record.state = "no_candidate_association";
    else if(record.candidateAssociationFraction>
            mfGeometrySparseFlowTrackingFilterMaximumAssociationFraction)
        record.state = "maximum_fraction_fail_open";
    else if(record.baselineAssociations-
                record.candidateAssociations<
            mnGeometrySparseFlowTrackingFilterMinimumAssociations)
        record.state = "minimum_remaining_fail_open";
    else
    {
        for(std::size_t index=0; index<count; ++index)
        {
            if(mCurrentSparseFlowFilterResult.
                   candidateMask[index]==0)
            {
                continue;
            }
            MapPoint *mapPoint =
                mCurrentFrame.mvpMapPoints[index];
            if(!mapPoint || mapPoint->isBad())
                continue;
            if(!mGeometrySparseFlowRemovedAssociationCsvPath.empty())
            {
                GeometrySparseFlowRemovedAssociationRecord
                    removedRecord;
                removedRecord.frameId = mCurrentFrame.mnId;
                removedRecord.timestamp =
                    mCurrentFrame.mTimeStamp;
                removedRecord.featureIndex = index;
                removedRecord.pixelX =
                    mCurrentFrame.mvKeys[index].pt.x;
                removedRecord.pixelY =
                    mCurrentFrame.mvKeys[index].pt.y;
                removedRecord.mapPointId = mapPoint->mnId;
                removedRecord.qThreshold =
                    mfGeometrySparseFlowTrackingFilterQ;
                removedRecord.frameScalePixels =
                    mCurrentSparseFlowFilterResult.frameScalePixels;
                removedRecord.semanticDynamic =
                    index<mCurrentFrame.mvbSemanticDynamic.size() &&
                    mCurrentFrame.mvbSemanticDynamic[index]!=0;
                mvGeometrySparseFlowRemovedAssociationDiagnostics.
                    push_back(removedRecord);
            }
            mCurrentFrame.mvpMapPoints[index] =
                static_cast<MapPoint*>(NULL);
            mCurrentFrame.mvbOutlier[index] = false;
            mvbCurrentSparseFlowRemovedAssociations[index] = 1;
            const std::size_t candidateRecordIndex =
                candidateAssociationRecordIndices[index];
            if(candidateRecordIndex!=
                   std::numeric_limits<std::size_t>::max())
            {
                candidateAssociationRecords[
                    candidateRecordIndex].removed = true;
            }
            ++record.removedAssociations;
        }
        record.remainingAssociations =
            record.baselineAssociations-
            record.removedAssociations;
        record.applied = record.removedAssociations>0;
        record.state =
            record.applied ? "applied" :
                "no_candidate_association";
    }

    for(std::size_t index=0;
        index<candidateAssociationRecords.size(); ++index)
    {
        candidateAssociationRecords[index].filterState =
            record.state;
        mvGeometrySparseFlowCandidateAssociationDiagnostics.
            push_back(candidateAssociationRecords[index]);
    }

    mbCurrentSparseFlowTrackingSafeguardsPassed =
        record.state=="applied" ||
        record.state=="no_candidate_association";
    mCurrentSparseFlowTrackingSafeguardState =
        record.state;
    mvGeometrySparseFlowTrackingFilterDiagnostics.
        push_back(record);
    if(record.applied ||
       mCurrentFrame.mnId%
           static_cast<long unsigned int>(
               mnGeometryLogEveryN)==0)
    {
        cout << "[Geometry G1-F1] frame="
             << record.frameId
             << " q=" << record.qThreshold
             << " scale=" << record.frameScalePixels
             << " support=" << record.scaleSupport
             << " candidate_features="
             << record.candidateFeatures
             << " baseline_assoc="
             << record.baselineAssociations
             << " candidate_assoc="
             << record.candidateAssociations
             << " removed="
             << record.removedAssociations
             << " remaining="
             << record.remainingAssociations
             << " state=" << record.state
             << " pose_reoptimization=none"
             << " mapping_veto=none"
             << endl;
    }
    return record.removedAssociations;
}

void Tracking::RecordSparseFlowAssociationSnapshot(
    const std::string &stage,
    const int trackingInliers,
    const std::vector<unsigned char> &countedTrackingInliers)
{
    if(!mbGeometrySparseFlowCounterfactualShadowEnabled ||
       mGeometryAssociationSnapshotCsvPath.empty() ||
       (!mGeometrySparseFlowFrameFilter.empty() &&
        mGeometrySparseFlowFrameFilter.count(
            mCurrentFrame.mnId)==0))
    {
        return;
    }
    if(stage!="post_search_pre_pose" &&
       stage!="post_existing_pose")
    {
        throw std::invalid_argument(
            "unsupported sparse-flow association snapshot stage");
    }
    if((stage=="post_search_pre_pose" &&
        (trackingInliers!=-1 ||
         !countedTrackingInliers.empty())) ||
       (stage=="post_existing_pose" &&
        (trackingInliers<0 ||
         countedTrackingInliers.size()!=
             static_cast<std::size_t>(
                 mCurrentFrame.N))))
    {
        throw std::invalid_argument(
            "association snapshot tracking-inlier state is invalid");
    }
    if(stage=="post_existing_pose")
    {
        const int counted = std::accumulate(
            countedTrackingInliers.begin(),
            countedTrackingInliers.end(),0);
        if(counted!=trackingInliers)
        {
            throw std::logic_error(
                "association snapshot inlier mask/count mismatch");
        }
    }

    const std::size_t count =
        static_cast<std::size_t>(mCurrentFrame.N);
    if(mCurrentFrame.mvpMapPoints.size()!=count ||
       mCurrentFrame.mvbOutlier.size()!=count ||
       mCurrentFrame.mvbSemanticDynamic.size()!=count)
    {
        throw std::logic_error(
            "Frame feature association vectors must match Frame::N");
    }

    const bool withinRelocalizationWindow =
        mCurrentFrame.mnId<
            mnLastRelocFrameId+
                static_cast<long unsigned int>(mMaxFrames);
    for(std::size_t index=0; index<count; ++index)
    {
        MapPoint *mapPoint =
            mCurrentFrame.mvpMapPoints[index];
        GeometryAssociationSnapshotRecord record;
        record.frameId = mCurrentFrame.mnId;
        record.timestamp = mCurrentFrame.mTimeStamp;
        record.stage = stage;
        record.featureIndex = index;
        record.hasMapPoint = mapPoint!=NULL;
        record.mapPointBad =
            mapPoint ? mapPoint->isBad() : false;
        record.mapPointObservations =
            mapPoint ? mapPoint->Observations() : 0;
        record.currentFrameOutlier =
            mCurrentFrame.mvbOutlier[index];
        record.semanticNonzero =
            mCurrentFrame.mvbSemanticDynamic[index]!=0;
        record.onlyTracking = mbOnlyTracking;
        record.withinRelocalizationWindow =
            withinRelocalizationWindow;
        record.countedTrackingInlier =
            stage=="post_existing_pose" &&
            countedTrackingInliers[index]!=0;
        record.trackingInliers = trackingInliers;
        mvGeometryAssociationSnapshotDiagnostics.push_back(
            record);
    }
}

void Tracking::UpdateSparseEgoFlowReference()
{
    if(!mbGeometrySparseEgoFlowShadowEnabled ||
       mImGray.empty() ||
       mCurrentDepthMeters.empty() ||
       mCurrentFrame.mTcw.empty())
    {
        mSparseFlowReference.valid = false;
        return;
    }

    mSparseFlowReference.gray = mImGray.clone();
    mSparseFlowReference.depthMeters =
        mCurrentDepthMeters.clone();
    if(!mCurrentFrame.mSemanticMask.empty())
    {
        mSparseFlowReference.depthMeters.setTo(
            0.0f,mCurrentFrame.mSemanticMask);
    }
    mSparseFlowReference.TcwFinal =
        mCurrentFrame.mTcw.clone();
    mSparseFlowReference.TcwGroundTruth =
        mCurrentGroundTruthTcw.empty()
        ? cv::Mat() : mCurrentGroundTruthTcw.clone();
    mSparseFlowReference.frameId = mCurrentFrame.mnId;
    mSparseFlowReference.timestamp =
        mCurrentFrame.mTimeStamp;
    mSparseFlowReference.valid = true;
}

void Tracking::RunGeometryShadow()
{
    if(!mbGeometryShadowEnabled || mSensor!=System::RGBD ||
       mCurrentDepthMeters.empty() || mCurrentFrame.mTcw.empty())
    {
        return;
    }

    if(!mbGeometrySingleReferenceShadowEnabled)
    {
        RunMultiReferenceGeometryShadow();
        return;
    }

    if(!mGeometricDetector.HasReference())
        return;

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

    RunMultiReferenceGeometryShadow();

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

void Tracking::RunMultiReferenceGeometryShadow()
{
    if(!mbGeometryMultiReferenceShadowEnabled ||
       mCurrentDepthMeters.empty() || mCurrentFrame.mTcw.empty() ||
       mqGeometryKeyframeReferences.empty())
    {
        return;
    }

    std::string samplingPolicyLabel =
        mGeometryMultiReferenceSamplingPolicy;
    if(mGeometryMultiReferenceSamplingPolicy=="grid_depth")
    {
        std::ostringstream label;
        label << "grid_depth_s"
              << mnGeometryMultiReferenceGridStride;
        samplingPolicyLabel = label.str();
    }
    else if(mGeometryMultiReferenceSamplingPolicy=="pyramid_dense")
    {
        std::ostringstream label;
        label << "pyramid_dense_s"
              << mnGeometryMultiReferencePyramidScale;
        samplingPolicyLabel = label.str();
    }

    std::vector<GeometricReferenceFrame> cachedReferences(
        mqGeometryKeyframeReferences.begin(),
        mqGeometryKeyframeReferences.end());
    std::vector<long unsigned int> candidateFrameIds;
    std::vector<int> candidateCovisibilityWeights;

    if(mGeometryMultiReferenceSelectionPolicy=="recent")
    {
        candidateFrameIds.reserve(cachedReferences.size());
        candidateCovisibilityWeights.reserve(cachedReferences.size());
        for(std::vector<GeometricReferenceFrame>::const_reverse_iterator
                reference=cachedReferences.rbegin();
            reference!=cachedReferences.rend(); ++reference)
        {
            candidateFrameIds.push_back(reference->frameId);
            candidateCovisibilityWeights.push_back(-1);
        }
    }
    else
    {
        KeyFrame *primaryReference = mCurrentFrame.mpReferenceKF;
        if(primaryReference && !primaryReference->isBad())
        {
            int currentOverlapCount = 0;
            for(int featureIndex=0;
                featureIndex<mCurrentFrame.N; ++featureIndex)
            {
                MapPoint *mapPoint =
                    mCurrentFrame.mvpMapPoints[featureIndex];
                if(!mapPoint || mapPoint->isBad())
                    continue;
                const map<KeyFrame*,size_t> observations =
                    mapPoint->GetObservations();
                if(observations.count(primaryReference)>0)
                    ++currentOverlapCount;
            }
            candidateFrameIds.push_back(
                primaryReference->mnFrameId);
            candidateCovisibilityWeights.push_back(
                currentOverlapCount);

            const vector<KeyFrame*> covisibleKeyFrames =
                primaryReference->GetVectorCovisibleKeyFrames();
            for(std::size_t index=0;
                index<covisibleKeyFrames.size(); ++index)
            {
                KeyFrame *candidate = covisibleKeyFrames[index];
                if(!candidate || candidate->isBad())
                    continue;
                const long unsigned int candidateFrameId =
                    candidate->mnFrameId;
                if(std::find(candidateFrameIds.begin(),
                             candidateFrameIds.end(),
                             candidateFrameId)!=
                   candidateFrameIds.end())
                {
                    continue;
                }
                candidateFrameIds.push_back(candidateFrameId);
                candidateCovisibilityWeights.push_back(
                    primaryReference->GetWeight(candidate));
            }
        }
    }

    const GeometricReferenceSelectionResult selection =
        GeometricDynamicDetector::SelectCachedReferences(
            cachedReferences,candidateFrameIds,
            static_cast<std::size_t>(
                mnGeometryMultiReferenceMaxReferences));
    const std::vector<GeometricReferenceFrame> &references =
        selection.references;

    GeometryReferenceSelectionRecord selectionRecord;
    selectionRecord.frameId = mCurrentFrame.mnId;
    selectionRecord.timestamp = mCurrentFrame.mTimeStamp;
    selectionRecord.policy =
        mGeometryMultiReferenceSelectionPolicy;
    selectionRecord.samplingPolicy =
        samplingPolicyLabel;
    selectionRecord.requestedReferenceCount =
        mnGeometryMultiReferenceMaxReferences;
    selectionRecord.stats = selection.stats;
    selectionRecord.evidenceComputed = false;
    selectionRecord.denseAuditComputed = false;
    selectionRecord.sampledComparisonPixels = 0;
    selectionRecord.denseComparisonOnSampledPixels = 0;
    selectionRecord.sampledPositivePresencePixels = 0;
    selectionRecord.densePositiveOnSampledPixels = 0;
    selectionRecord.bothPositivePixels = 0;
    selectionRecord.positivePresenceAgreementPixels = 0;
    selectionRecord.exactVoteAgreementPixels = 0;
    for(std::size_t referenceIndex=0;
        referenceIndex<references.size(); ++referenceIndex)
    {
        const long unsigned int selectedFrameId =
            references[referenceIndex].frameId;
        selectionRecord.selectedFrameIds.push_back(
            selectedFrameId);
        const std::vector<long unsigned int>::const_iterator
            candidatePosition =
                std::find(candidateFrameIds.begin(),
                          candidateFrameIds.end(),
                          selectedFrameId);
        int covisibilityWeight = -1;
        if(candidatePosition!=candidateFrameIds.end())
        {
            const std::size_t candidateIndex =
                static_cast<std::size_t>(
                    candidatePosition-candidateFrameIds.begin());
            if(candidateIndex<
               candidateCovisibilityWeights.size())
            {
                covisibilityWeight =
                    candidateCovisibilityWeights[candidateIndex];
            }
        }
        selectionRecord.selectedCovisibilityWeights.push_back(
            covisibilityWeight);
        selectionRecord.selectedFrameAges.push_back(
            static_cast<long int>(mCurrentFrame.mnId)-
            static_cast<long int>(selectedFrameId));
    }

    if(references.size()!=
       static_cast<std::size_t>(
           mnGeometryMultiReferenceMaxReferences))
    {
        mvGeometryReferenceSelectionDiagnostics.push_back(
            selectionRecord);
        const std::size_t diagnosticCount =
            mvGeometryReferenceSelectionDiagnostics.size();
        if(diagnosticCount==1 ||
           diagnosticCount%
               static_cast<std::size_t>(
                   mnGeometryLogEveryN)==0)
        {
            cout << "[Geometry G2-2R] frame="
                 << mCurrentFrame.mnId
                 << " policy="
                 << mGeometryMultiReferenceSelectionPolicy
                 << " sampling="
                 << samplingPolicyLabel
                 << " requested="
                 << mnGeometryMultiReferenceMaxReferences
                 << " candidates="
                 << selection.stats.candidateCount
                 << " cached_matches="
                 << selection.stats.cachedReferenceMatchCount
                 << " selected="
                 << selection.stats.selectedReferenceCount
                 << " evidence_computed=0"
                 << " reason=insufficient_cached_policy_references"
                 << endl;
        }
        return;
    }

    GeometricMultiReferenceResult result;
    try
    {
        if(mGeometryMultiReferenceSamplingPolicy=="pyramid_dense")
        {
            result =
                GeometricDynamicDetector::
                    ComputePyramidMultiReferenceEvidence(
                        references,mCurrentDepthMeters,
                        mCurrentFrame.mTcw,mGeometryK,
                        mGeometricDetector.
                            ResidualThresholdMeters(),
                        mnGeometryMultiReferencePyramidScale,
                        mGeometryRegionRelativeThreshold,
                        mGeometryRegionAbsoluteThresholdMeters);
        }
        else
        {
            result =
                GeometricDynamicDetector::
                    ComputeMultiReferenceEvidence(
                        references,mCurrentDepthMeters,
                        mCurrentFrame.mTcw,mGeometryK,
                        mGeometricDetector.
                            ResidualThresholdMeters(),
                        mGeometryMultiReferenceSamplingPolicy==
                            "orb_depth"
                            ? GeometricReferenceSamplingPolicy::
                                OrbDepth
                            : (mGeometryMultiReferenceSamplingPolicy==
                                    "grid_depth"
                                ? GeometricReferenceSamplingPolicy::
                                    GridDepth
                                : GeometricReferenceSamplingPolicy::
                                    Dense));
        }
    }
    catch(const std::exception &error)
    {
        cerr << "[Geometry G2-1] frame=" << mCurrentFrame.mnId
             << " multi-reference evidence failed: "
             << error.what() << endl;
        mvGeometryReferenceSelectionDiagnostics.push_back(
            selectionRecord);
        return;
    }

    selectionRecord.evidenceComputed = true;
    selectionRecord.perReference = result.perReference;
    ++mnGeometryMultiReferenceComputedFrames;

    if(!mGeometryMultiReferenceFeatureCsvPath.empty() &&
       (mGeometryMultiReferenceFeatureFrameFilter.empty() ||
        mGeometryMultiReferenceFeatureFrameFilter.count(
            mCurrentFrame.mnId)>0))
    {
        std::vector<cv::Point2f> featurePixels;
        featurePixels.reserve(mCurrentFrame.mvKeys.size());
        for(std::size_t featureIndex=0;
            featureIndex<mCurrentFrame.mvKeys.size(); ++featureIndex)
        {
            featurePixels.push_back(
                mCurrentFrame.mvKeys[featureIndex].pt);
        }
        const std::vector<GeometricFeatureEvidenceSample> samples =
            GeometricDynamicDetector::
                SampleMultiReferenceEvidenceAtFeatures(
                    result,featurePixels);
        const bool hasSemantic =
            !mCurrentFrame.mSemanticMask.empty() &&
            mCurrentFrame.mSemanticMask.type()==CV_8UC1 &&
            mCurrentFrame.mSemanticMask.size()==
                result.comparisonCount.size();
        for(std::size_t sampleIndex=0;
            sampleIndex<samples.size(); ++sampleIndex)
        {
            const GeometricFeatureEvidenceSample &sample =
                samples[sampleIndex];
            GeometryMultiReferenceFeatureRecord record;
            record.frameId = mCurrentFrame.mnId;
            record.timestamp = mCurrentFrame.mTimeStamp;
            record.samplingPolicy = samplingPolicyLabel;
            record.featureIndex = sample.featureIndex;
            record.imageU = sample.imageU;
            record.imageV = sample.imageV;
            record.octave =
                mCurrentFrame.mvKeys[sample.featureIndex].octave;
            record.hasMapPoint =
                sample.featureIndex<
                    mCurrentFrame.mvpMapPoints.size() &&
                mCurrentFrame.mvpMapPoints[sample.featureIndex]!=NULL;
            record.currentFrameOutlierFlag =
                sample.featureIndex<
                    mCurrentFrame.mvbOutlier.size() &&
                mCurrentFrame.mvbOutlier[sample.featureIndex];
            record.semanticNonzero =
                hasSemantic &&
                sample.imageU>=0 &&
                sample.imageV>=0 &&
                sample.imageU<
                    mCurrentFrame.mSemanticMask.cols &&
                sample.imageV<
                    mCurrentFrame.mSemanticMask.rows &&
                mCurrentFrame.mSemanticMask.at<unsigned char>(
                    sample.imageV,sample.imageU)!=0;
            record.nativeScale = sample.nativeScale;
            record.nativeU = sample.nativeU;
            record.nativeV = sample.nativeV;
            record.comparisonCount = sample.comparisonCount;
            record.positiveCount = sample.positiveCount;
            record.negativeCount = sample.negativeCount;
            record.consistentCount = sample.consistentCount;
            mvGeometryMultiReferenceFeatureDiagnostics.push_back(
                record);
        }
    }

    GeometricMultiReferenceResult denseAuditResult;
    bool hasDenseAuditResult = false;
    if(mbGeometryMultiReferenceDenseAuditEnabled)
    {
        try
        {
            denseAuditResult =
                GeometricDynamicDetector::ComputeMultiReferenceEvidence(
                    references,mCurrentDepthMeters,mCurrentFrame.mTcw,
                    mGeometryK,
                    mGeometricDetector.ResidualThresholdMeters(),
                    GeometricReferenceSamplingPolicy::Dense);
            hasDenseAuditResult = true;
        }
        catch(const std::exception &error)
        {
            cerr << "[Geometry G2-2] frame="
                 << mCurrentFrame.mnId
                 << " same-reference dense audit failed: "
                 << error.what() << endl;
        }
    }
    if(hasDenseAuditResult)
    {
        selectionRecord.denseAuditComputed = true;
        for(int v=0; v<result.comparisonCount.rows; ++v)
        {
            const unsigned char *sampledComparison =
                result.comparisonCount.ptr<unsigned char>(v);
            const unsigned char *sampledPositive =
                result.positiveCount.ptr<unsigned char>(v);
            const unsigned char *denseComparison =
                denseAuditResult.comparisonCount.ptr<unsigned char>(v);
            const unsigned char *densePositive =
                denseAuditResult.positiveCount.ptr<unsigned char>(v);
            for(int u=0; u<result.comparisonCount.cols; ++u)
            {
                if(sampledComparison[u]==0)
                    continue;
                ++selectionRecord.sampledComparisonPixels;
                if(denseComparison[u]>0)
                {
                    ++selectionRecord.denseComparisonOnSampledPixels;
                }
                if(sampledPositive[u]>0)
                    ++selectionRecord.sampledPositivePresencePixels;
                if(densePositive[u]>0)
                    ++selectionRecord.densePositiveOnSampledPixels;
                if(sampledPositive[u]>0 && densePositive[u]>0)
                    ++selectionRecord.bothPositivePixels;
                if((sampledPositive[u]>0)==(densePositive[u]>0))
                {
                    ++selectionRecord.positivePresenceAgreementPixels;
                }
                if(sampledComparison[u]==denseComparison[u] &&
                   sampledPositive[u]==densePositive[u])
                {
                    ++selectionRecord.exactVoteAgreementPixels;
                }
            }
        }
    }
    mvGeometryReferenceSelectionDiagnostics.push_back(
        selectionRecord);

    GeometricRegionPartitionResult regionPartition;
    GeometricRegionEvidenceAggregationResult regionAggregation;
    GeometricRegionEvidenceAggregationResult
        denseRegionAggregation;
    bool hasRegionEvidenceAggregation = false;
    bool hasDenseRegionEvidenceAggregation = false;
    if(mbGeometryRegionEvidenceShadowEnabled)
    {
        try
        {
            if(mbGeometryLowResolutionRegionShadowEnabled)
            {
                if(result.nativeDepthMeters.empty() ||
                   result.nativeComparisonCount.empty() ||
                   result.nativeScale!=
                       mnGeometryMultiReferencePyramidScale)
                {
                    throw std::logic_error(
                        "low-resolution region candidate requires "
                        "native pyramid depth and evidence");
                }

                regionPartition =
                    GeometricDynamicDetector::
                        PartitionDepthByDiscontinuity(
                            result.nativeDepthMeters,
                            mGeometryRegionRelativeThreshold,
                            mGeometryRegionAbsoluteThresholdMeters);
                regionPartition.stats.domainScale =
                    result.nativeScale;

                GeometricMultiReferenceResult nativeEvidence;
                nativeEvidence.comparisonCount =
                    result.nativeComparisonCount;
                nativeEvidence.positiveCount =
                    result.nativePositiveCount;
                nativeEvidence.negativeCount =
                    result.nativeNegativeCount;
                nativeEvidence.consistentCount =
                    result.nativeConsistentCount;

                cv::Mat nativeSemanticProxy;
                if(!mCurrentFrame.mSemanticMask.empty())
                {
                    nativeSemanticProxy =
                        GeometricDynamicDetector::
                            DownsampleMaskAny(
                                mCurrentFrame.mSemanticMask,
                                result.nativeScale);
                }
                regionAggregation =
                    GeometricDynamicDetector::
                        AggregateMultiReferenceEvidenceByRegion(
                            regionPartition,nativeEvidence,
                            nativeSemanticProxy,
                            mbGeometryRegionRiskDiagnosticsEnabled);

                const std::chrono::steady_clock::time_point
                    mappingStart =
                        std::chrono::steady_clock::now();
                cv::Mat mappedLabels;
                cv::Mat mappedBoundary;
                cv::Mat repeatedLabels;
                cv::Mat repeatedBoundary;
                cv::repeat(
                    regionPartition.labels,result.nativeScale,
                    result.nativeScale,repeatedLabels);
                cv::repeat(
                    regionPartition.boundaryMask,
                    result.nativeScale,result.nativeScale,
                    repeatedBoundary);
                repeatedLabels(
                    cv::Rect(
                        0,0,mCurrentDepthMeters.cols,
                        mCurrentDepthMeters.rows)).copyTo(
                            mappedLabels);
                repeatedBoundary(
                    cv::Rect(
                        0,0,mCurrentDepthMeters.cols,
                        mCurrentDepthMeters.rows)).copyTo(
                            mappedBoundary);
                regionPartition.stats.mappingMs =
                    std::chrono::duration<double,std::milli>(
                        std::chrono::steady_clock::now()-
                        mappingStart).count();
                regionPartition.stats.onlineTotalMs =
                    regionPartition.stats.totalMs+
                    regionAggregation.stats.totalMs+
                    regionPartition.stats.mappingMs;
            }
            else
            {
                regionPartition =
                    GeometricDynamicDetector::
                        PartitionDepthByDiscontinuity(
                            mCurrentDepthMeters,
                            mGeometryRegionRelativeThreshold,
                            mGeometryRegionAbsoluteThresholdMeters);
                regionAggregation =
                    GeometricDynamicDetector::
                        AggregateMultiReferenceEvidenceByRegion(
                            regionPartition,result,
                            mCurrentFrame.mSemanticMask,
                            mbGeometryRegionRiskDiagnosticsEnabled);
                regionPartition.stats.onlineTotalMs =
                    regionPartition.stats.totalMs+
                    regionAggregation.stats.totalMs;
            }
            hasRegionEvidenceAggregation = true;
            ++mnGeometryRegionEvidenceComputedFrames;

            for(std::size_t regionIndex=0;
                regionIndex<regionAggregation.regions.size();
                ++regionIndex)
            {
                GeometryRegionEvidenceRecord record;
                record.frameId = mCurrentFrame.mnId;
                record.timestamp = mCurrentFrame.mTimeStamp;
                record.samplingPolicy = samplingPolicyLabel;
                record.partitionStats = regionPartition.stats;
                record.aggregationStats =
                    regionAggregation.stats;
                record.region =
                    regionAggregation.regions[regionIndex];
                mvGeometryRegionEvidenceDiagnostics.push_back(
                    record);
            }

            if(hasDenseAuditResult &&
               !mbGeometryLowResolutionRegionShadowEnabled)
            {
                denseRegionAggregation =
                    GeometricDynamicDetector::
                        AggregateMultiReferenceEvidenceByRegion(
                            regionPartition,denseAuditResult,
                            mCurrentFrame.mSemanticMask,
                            mbGeometryRegionRiskDiagnosticsEnabled);
                hasDenseRegionEvidenceAggregation = true;

                for(std::size_t regionIndex=0;
                    regionIndex<
                        denseRegionAggregation.regions.size();
                    ++regionIndex)
                {
                    GeometryRegionEvidenceRecord record;
                    record.frameId = mCurrentFrame.mnId;
                    record.timestamp =
                        mCurrentFrame.mTimeStamp;
                    record.samplingPolicy =
                        "dense_same_reference_audit";
                    record.partitionStats =
                        regionPartition.stats;
                    record.aggregationStats =
                        denseRegionAggregation.stats;
                    record.region =
                        denseRegionAggregation.regions[
                            regionIndex];
                    mvGeometryRegionEvidenceDiagnostics.push_back(
                        record);
                }
            }
        }
        catch(const std::exception &error)
        {
            cerr << "[Geometry G2-3R1/G2-3R2] frame="
                 << mCurrentFrame.mnId
                 << " region evidence aggregation failed: "
                 << error.what() << endl;
        }
    }

    if(!mGeometryMultiReferenceDebugOutputDir.empty())
    {
        std::ostringstream prefix;
        prefix << mGeometryMultiReferenceDebugOutputDir
               << "/frame_" << std::setw(6) << std::setfill('0')
               << mCurrentFrame.mnId;
        const bool comparisonWritten = cv::imwrite(
            prefix.str()+"_comparisons.png",
            result.comparisonCount);
        const bool positiveWritten = cv::imwrite(
            prefix.str()+"_positives.png",
            result.positiveCount);
        if(!comparisonWritten || !positiveWritten)
        {
            cerr << "[Geometry G2-1] frame=" << mCurrentFrame.mnId
                 << " failed to save raw count images to "
                 << mGeometryMultiReferenceDebugOutputDir << endl;
        }
    }

    const int referenceCount =
        static_cast<int>(references.size());
    const auto appendHistogram =
        [&](const GeometricMultiReferenceResult &evidence,
            const std::string &samplingPolicy)
        {
            if(mGeometryMultiReferenceCsvPath.empty())
                return;

            std::vector<std::vector<std::size_t> > histogram(
                static_cast<std::size_t>(referenceCount+1),
                std::vector<std::size_t>(
                    static_cast<std::size_t>(referenceCount+1),0));
            std::vector<std::vector<std::size_t> > semanticHistogram(
                static_cast<std::size_t>(referenceCount+1),
                std::vector<std::size_t>(
                    static_cast<std::size_t>(referenceCount+1),0));

            const bool hasSemanticProxy =
                !mCurrentFrame.mSemanticMask.empty() &&
                mCurrentFrame.mSemanticMask.type()==CV_8UC1 &&
                mCurrentFrame.mSemanticMask.size()==
                    evidence.comparisonCount.size();
            for(int v=0; v<evidence.comparisonCount.rows; ++v)
            {
                const unsigned char *comparisonRow =
                    evidence.comparisonCount.ptr<unsigned char>(v);
                const unsigned char *positiveRow =
                    evidence.positiveCount.ptr<unsigned char>(v);
                const unsigned char *semanticRow =
                    hasSemanticProxy
                        ? mCurrentFrame.mSemanticMask.ptr<unsigned char>(v)
                        : static_cast<const unsigned char*>(NULL);
                for(int u=0; u<evidence.comparisonCount.cols; ++u)
                {
                    const int comparisons = comparisonRow[u];
                    const int positives = positiveRow[u];
                    if(comparisons<0 ||
                       comparisons>referenceCount ||
                       positives<0 ||
                       positives>comparisons)
                    {
                        throw std::logic_error(
                            "G2 histogram received invalid evidence counts");
                    }
                    ++histogram[
                        static_cast<std::size_t>(comparisons)]
                        [static_cast<std::size_t>(positives)];
                    if(semanticRow && semanticRow[u]!=0)
                    {
                        ++semanticHistogram[
                            static_cast<std::size_t>(comparisons)]
                            [static_cast<std::size_t>(positives)];
                    }
                }
            }

            for(int comparisons=0;
                comparisons<=referenceCount; ++comparisons)
            {
                for(int positives=0;
                    positives<=comparisons; ++positives)
                {
                    const std::size_t pixelCount =
                        histogram[
                            static_cast<std::size_t>(comparisons)]
                            [static_cast<std::size_t>(positives)];
                    const std::size_t semanticPixelCount =
                        semanticHistogram[
                            static_cast<std::size_t>(comparisons)]
                            [static_cast<std::size_t>(positives)];
                    if(pixelCount==0 && semanticPixelCount==0)
                        continue;

                    GeometryMultiReferenceHistogramRecord record;
                    record.frameId = mCurrentFrame.mnId;
                    record.timestamp = mCurrentFrame.mTimeStamp;
                    record.samplingPolicy = samplingPolicy;
                    record.referenceCount = referenceCount;
                    record.comparisonCount = comparisons;
                    record.positiveCount = positives;
                    record.pixelCount = pixelCount;
                    record.semanticPixelCount = semanticPixelCount;
                    record.frameStats = evidence.stats;
                    mvGeometryMultiReferenceHistogram.push_back(
                        record);
                }
            }
        };
    appendHistogram(result,samplingPolicyLabel);
    if(hasDenseAuditResult)
        appendHistogram(denseAuditResult,"dense_same_reference_audit");

    if(mnGeometryMultiReferenceComputedFrames==1 ||
       mnGeometryMultiReferenceComputedFrames%
           static_cast<long unsigned int>(
               mnGeometryLogEveryN)==0)
    {
        cout << "[Geometry G2-1] frame=" << mCurrentFrame.mnId
             << " policy="
             << mGeometryMultiReferenceSelectionPolicy
             << " sampling="
             << samplingPolicyLabel
             << " references=" << referenceCount
             << " selected_ids="
             << JoinDiagnosticValues(
                    selectionRecord.selectedFrameIds)
             << " selected_weights="
             << JoinDiagnosticValues(
                    selectionRecord.selectedCovisibilityWeights)
             << " selected_ages="
             << JoinDiagnosticValues(
                    selectionRecord.selectedFrameAges)
             << " pixels_with_comparison="
             << result.stats.pixelsWithComparison
             << " total_comparisons="
             << result.stats.totalComparisons
             << " pixels_with_positive="
             << result.stats.pixelsWithPositiveEvidence
             << " positive_votes="
             << result.stats.totalPositiveVotes
             << " negative_votes="
             << result.stats.totalNegativeVotes
             << " consistent_votes="
             << result.stats.totalConsistentVotes
             << " warp_evidence_ms="
             << result.stats.warpAndEvidenceMs
             << " aggregate_ms="
             << result.stats.aggregateMs
             << " preprocess_ms="
             << result.stats.preprocessMs
             << " expand_ms="
             << result.stats.expandMs
             << " total_ms="
             << result.stats.totalMs
             << " same_reference_dense_audit_ms="
             << (hasDenseAuditResult
                     ? denseAuditResult.stats.totalMs : -1.0)
             << " region_partition_ms="
             << (hasRegionEvidenceAggregation
                     ? regionPartition.stats.totalMs : -1.0)
             << " region_aggregation_ms="
             << (hasRegionEvidenceAggregation
                     ? regionAggregation.stats.totalMs : -1.0)
             << " dense_region_aggregation_ms="
             << (hasDenseRegionEvidenceAggregation
                     ? denseRegionAggregation.stats.totalMs : -1.0)
             << " region_count="
             << (hasRegionEvidenceAggregation
                     ? regionAggregation.stats.regionCount : 0)
             << " regions_with_comparison="
             << (hasRegionEvidenceAggregation
                     ? regionAggregation.stats.regionsWithComparison : 0)
             << " regions_with_positive="
             << (hasRegionEvidenceAggregation
                     ? regionAggregation.stats.regionsWithPositiveEvidence : 0)
             << " dynamic_decision=none"
             << " direct_slam_state_mutation=none"
             << endl;
    }
}

void Tracking::UpdateMultiReferenceGeometryHistory(
    const cv::Mat &referenceDepth)
{
    if(!mbGeometryMultiReferenceShadowEnabled)
        return;
    if(referenceDepth.empty() || referenceDepth.type()!=CV_32FC1 ||
       mCurrentFrame.mTcw.empty())
    {
        throw std::invalid_argument(
            "G2-1 reference history requires CV_32FC1 depth and a pose");
    }
    if(!mqGeometryKeyframeReferences.empty() &&
       mqGeometryKeyframeReferences.back().frameId==
           mCurrentFrame.mnId)
    {
        return;
    }

    GeometricReferenceFrame reference;
    reference.depthMeters = referenceDepth.clone();
    if(mGeometryMultiReferenceSamplingPolicy=="pyramid_dense")
    {
        reference.pyramidDepthMeters =
            GeometricDynamicDetector::
                DownsampleDepthBoundaryAware(
                    referenceDepth,
                    mnGeometryMultiReferencePyramidScale,
                    mGeometryRegionRelativeThreshold,
                    mGeometryRegionAbsoluteThresholdMeters);
    }
    reference.Tcw = mCurrentFrame.mTcw.clone();
    reference.frameId = mCurrentFrame.mnId;
    reference.timestampSeconds = mCurrentFrame.mTimeStamp;
    if(mGeometryMultiReferenceSamplingPolicy=="orb_depth")
    {
        cv::Mat sampledPixels =
            cv::Mat::zeros(referenceDepth.size(),CV_8UC1);
        reference.featureDepthPixels.reserve(
            mCurrentFrame.mvKeys.size());
        for(std::size_t featureIndex=0;
            featureIndex<mCurrentFrame.mvKeys.size(); ++featureIndex)
        {
            const int u = static_cast<int>(
                mCurrentFrame.mvKeys[featureIndex].pt.x);
            const int v = static_cast<int>(
                mCurrentFrame.mvKeys[featureIndex].pt.y);
            if(u<0 || u>=referenceDepth.cols ||
               v<0 || v>=referenceDepth.rows ||
               sampledPixels.at<unsigned char>(v,u)!=0)
            {
                continue;
            }
            sampledPixels.at<unsigned char>(v,u) = 255;
            const float depth = referenceDepth.at<float>(v,u);
            if(std::isfinite(depth) && depth>0.0f)
            {
                reference.featureDepthPixels.push_back(
                    cv::Point2i(u,v));
            }
        }
    }
    else if(mGeometryMultiReferenceSamplingPolicy=="grid_depth")
    {
        const int stride = mnGeometryMultiReferenceGridStride;
        reference.gridDepthPixels.reserve(
            static_cast<std::size_t>(
                (referenceDepth.rows+stride-1)/stride)*
            static_cast<std::size_t>(
                (referenceDepth.cols+stride-1)/stride));
        for(int v=0; v<referenceDepth.rows; v+=stride)
        {
            for(int u=0; u<referenceDepth.cols; u+=stride)
            {
                const float depth = referenceDepth.at<float>(v,u);
                if(std::isfinite(depth) && depth>0.0f)
                {
                    reference.gridDepthPixels.push_back(
                        cv::Point2i(u,v));
                }
            }
        }
    }
    mqGeometryKeyframeReferences.push_back(reference);
    while(mqGeometryKeyframeReferences.size()>
          static_cast<std::size_t>(
              mnGeometryMultiReferenceHistorySize))
    {
        mqGeometryKeyframeReferences.pop_front();
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
    cv::Mat encodedLabels(
        result.labelImage.size(),CV_16UC1,cv::Scalar(0));
    for(int v=0; v<result.labelImage.rows; ++v)
    {
        const short *labelRow = result.labelImage.ptr<short>(v);
        unsigned short *encodedRow =
            encodedLabels.ptr<unsigned short>(v);
        for(int u=0; u<result.labelImage.cols; ++u)
        {
            if(labelRow[u]>=0)
            {
                encodedRow[u] =
                    static_cast<unsigned short>(labelRow[u]+1);
            }
        }
    }
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
        saved = cv::imwrite(
            prefix+"_ji_labels_u16.png",encodedLabels);
        if(saved && !mbJiGeometryDebugRawLabelsOnly)
        {
            saved =
                cv::imwrite(prefix+"_ji_clusters.png",clusterColor) &&
                cv::imwrite(
                    prefix+"_ji_cluster_overlay.png",overlay);
        }
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
             << " saved "
             << (mbJiGeometryDebugRawLabelsOnly
                     ? "raw cluster labels" : "cluster visualization")
             << " prefix=" << prefix << endl;
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

        // Relocalization must remain fail-open with respect to the optional
        // SIn-style geometry filter. Semantic flags are preserved, and the
        // geometry flags are restored after tracking for mapping admission.
        if(mState==LOST)
        {
            FailOpenSInRegionFeatureFilterForTracking(
                "relocalization");
        }

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
            RunSparseEgoFlowShadow();
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

        // A tracking fail-open must not turn into a mapping fail-open. Restore
        // exactly the SIn geometry flags introduced for this frame before any
        // new KeyFrame/MapPoint admission is considered below.
        RestoreSInRegionFeatureFilterForMapping();

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

        GeometrySparseFlowMappingAdmissionRecord mappingRecord;
        const bool recordMappingAdmission =
            mbGeometrySparseFlowMappingCounterfactualEnabled;
        if(recordMappingAdmission)
        {
            mappingRecord.frameId = mCurrentFrame.mnId;
            mappingRecord.timestamp = mCurrentFrame.mTimeStamp;
            mappingRecord.stage = "stereo_initialization";
            mappingRecord.qThreshold =
                mfGeometrySparseFlowTrackingFilterQ;
            mappingRecord.scaleValid = false;
            mappingRecord.candidateVectorValid = false;
            mappingRecord.candidateState =
                "reference_unavailable";
            mappingRecord.featureCount =
                static_cast<std::size_t>(mCurrentFrame.N);
            mappingRecord.candidateFeatures = 0;
            mappingRecord.candidateAssociationsBeforeMapping = 0;
            mappingRecord.candidateTrackingRemovals = 0;
            mappingRecord.validDepthFeatures = 0;
            mappingRecord.candidateValidDepthFeatures = 0;
            mappingRecord.depthAdmissionFeatures = 0;
            mappingRecord.candidateDepthAdmissionFeatures = 0;
            mappingRecord.createdMapPoints = 0;
            mappingRecord.candidateCreatedMapPoints = 0;
            mappingRecord.recreatedAfterTrackingRemoval = 0;
        }

        GeometrySparseFlowMappingFilterRecord mappingFilterRecord =
            GeometrySparseFlowMappingFilterRecord();
        const bool recordMappingFilter =
            mbGeometrySparseFlowMappingFilterEnabled;
        if(recordMappingFilter)
        {
            mappingFilterRecord.frameId = mCurrentFrame.mnId;
            mappingFilterRecord.timestamp =
                mCurrentFrame.mTimeStamp;
            mappingFilterRecord.stage =
                "stereo_initialization";
            mappingFilterRecord.qThreshold =
                mfGeometrySparseFlowTrackingFilterQ;
            mappingFilterRecord.scaleValid = false;
            mappingFilterRecord.candidateVectorValid = false;
            mappingFilterRecord.trackingSafeguardsPassed =
                false;
            mappingFilterRecord.trackingSafeguardState =
                "reference_unavailable";
            mappingFilterRecord.featureCount =
                static_cast<std::size_t>(mCurrentFrame.N);
            mappingFilterRecord.availableFeatures =
                static_cast<std::size_t>(nStaticFeatures);
            mappingFilterRecord.maximumFeatureFraction =
                mfGeometrySparseFlowMappingFilterMaximumFeatureFraction;
            mappingFilterRecord.maximumDepthFraction =
                mfGeometrySparseFlowMappingFilterMaximumDepthFraction;
            mappingFilterRecord.minimumRemainingDepthFeatures =
                mnGeometrySparseFlowMappingFilterMinimumRemainingDepthFeatures;
            mappingFilterRecord.state =
                "reference_unavailable_fail_open";
        }

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0 && !mCurrentFrame.mvbDynamic[i])
            {
                if(recordMappingAdmission)
                {
                    ++mappingRecord.validDepthFeatures;
                    ++mappingRecord.depthAdmissionFeatures;
                }
                if(recordMappingFilter)
                {
                    ++mappingFilterRecord.validDepthFeatures;
                    ++mappingFilterRecord.
                        remainingValidDepthFeatures;
                }
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
                if(recordMappingAdmission)
                    ++mappingRecord.createdMapPoints;
                if(recordMappingFilter)
                    ++mappingFilterRecord.createdMapPoints;
            }
        }
        if(recordMappingAdmission)
        {
            mvGeometrySparseFlowMappingAdmissionDiagnostics.
                push_back(mappingRecord);
        }
        if(recordMappingFilter)
        {
            mvGeometrySparseFlowMappingFilterDiagnostics.
                push_back(mappingFilterRecord);
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

    int nmatches = matcher.SearchByBoW(
        mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15 &&
       FailOpenSInRegionFeatureFilterForTracking(
           "reference_keyframe_pre_pose"))
    {
        vpMapPointMatches.clear();
        nmatches = matcher.SearchByBoW(
            mpReferenceKF,mCurrentFrame,vpMapPointMatches);
    }

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

    if(nmatches<20 &&
       FailOpenSInRegionFeatureFilterForTracking(
           "motion_model_pre_pose"))
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),
             mCurrentFrame.mvpMapPoints.end(),
             static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(
            mCurrentFrame,mLastFrame,2*th,
            mSensor==System::MONOCULAR);
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

    const int requiredPotentialInliers =
        mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames ? 50 : 30;
    int potentialMapInliers = 0;
    for(int index=0; index<mCurrentFrame.N; ++index)
    {
        MapPoint *mapPoint = mCurrentFrame.mvpMapPoints[index];
        if(!mapPoint || mapPoint->isBad())
            continue;
        if(mbOnlyTracking || mapPoint->Observations()>0)
            ++potentialMapInliers;
    }
    if(potentialMapInliers<requiredPotentialInliers &&
       FailOpenSInRegionFeatureFilterForTracking(
           "local_map_pre_pose"))
    {
        SearchLocalPoints();
        RemoveDynamicAssociations(mCurrentFrame);
    }

    ApplySparseFlowTrackingFilter();
    RecordSparseFlowAssociationSnapshot(
        "post_search_pre_pose");

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;
    std::vector<unsigned char> countedTrackingInliers(
        static_cast<std::size_t>(mCurrentFrame.N),0);

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
                    {
                        mnMatchesInliers++;
                        countedTrackingInliers[
                            static_cast<std::size_t>(i)] = 1;
                    }
                }
                else
                {
                    mnMatchesInliers++;
                    countedTrackingInliers[
                        static_cast<std::size_t>(i)] = 1;
                }
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }
    RecordSparseFlowAssociationSnapshot(
        "post_existing_pose",mnMatchesInliers,
        countedTrackingInliers);

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

    GeometrySparseFlowMappingAdmissionRecord mappingRecord;
    const bool recordMappingAdmission =
        mbGeometrySparseFlowMappingCounterfactualEnabled;
    bool candidateVectorValid = false;
    if(recordMappingAdmission)
    {
        mappingRecord.frameId = mCurrentFrame.mnId;
        mappingRecord.timestamp = mCurrentFrame.mTimeStamp;
        mappingRecord.stage = "create_new_keyframe";
        mappingRecord.qThreshold =
            mfGeometrySparseFlowTrackingFilterQ;
        mappingRecord.scaleValid =
            mCurrentSparseFlowFilterResult.scaleValid;
        mappingRecord.featureCount =
            static_cast<std::size_t>(mCurrentFrame.N);
        candidateVectorValid =
            mCurrentSparseFlowFilterResult.candidateMask.size()==
                mappingRecord.featureCount &&
            mvbCurrentSparseFlowRemovedAssociations.size()==
                mappingRecord.featureCount &&
            mCurrentFrame.mvpMapPoints.size()==
                mappingRecord.featureCount;
        mappingRecord.candidateVectorValid =
            candidateVectorValid;
        mappingRecord.candidateState =
            !mappingRecord.scaleValid
                ? "scale_invalid"
                : (candidateVectorValid
                    ? "measured" : "vector_size_mismatch");
        mappingRecord.candidateFeatures = 0;
        mappingRecord.candidateAssociationsBeforeMapping = 0;
        mappingRecord.candidateTrackingRemovals = 0;
        mappingRecord.validDepthFeatures = 0;
        mappingRecord.candidateValidDepthFeatures = 0;
        mappingRecord.depthAdmissionFeatures = 0;
        mappingRecord.candidateDepthAdmissionFeatures = 0;
        mappingRecord.createdMapPoints = 0;
        mappingRecord.candidateCreatedMapPoints = 0;
        mappingRecord.recreatedAfterTrackingRemoval = 0;

        if(candidateVectorValid)
        {
            for(std::size_t index=0;
                index<mappingRecord.featureCount; ++index)
            {
                if(mCurrentSparseFlowFilterResult.
                       candidateMask[index]==0)
                {
                    continue;
                }
                ++mappingRecord.candidateFeatures;
                MapPoint *mapPoint =
                    mCurrentFrame.mvpMapPoints[index];
                if(mapPoint && !mapPoint->isBad())
                {
                    ++mappingRecord.
                        candidateAssociationsBeforeMapping;
                }
                if(mvbCurrentSparseFlowRemovedAssociations[
                       index]!=0)
                {
                    ++mappingRecord.candidateTrackingRemovals;
                }
            }
        }
    }

    GeometrySparseFlowMappingFilterRecord mappingFilterRecord =
        GeometrySparseFlowMappingFilterRecord();
    const bool applyMappingFilter =
        mbGeometrySparseFlowMappingFilterEnabled;
    bool mappingFilterCandidateVectorValid = false;
    if(applyMappingFilter)
    {
        mappingFilterRecord.frameId = mCurrentFrame.mnId;
        mappingFilterRecord.timestamp =
            mCurrentFrame.mTimeStamp;
        mappingFilterRecord.stage = "create_new_keyframe";
        mappingFilterRecord.qThreshold =
            mfGeometrySparseFlowTrackingFilterQ;
        mappingFilterRecord.scaleValid =
            mCurrentSparseFlowFilterResult.scaleValid;
        mappingFilterRecord.trackingSafeguardsPassed =
            mbCurrentSparseFlowTrackingSafeguardsPassed;
        mappingFilterRecord.trackingSafeguardState =
            mCurrentSparseFlowTrackingSafeguardState;
        mappingFilterRecord.featureCount =
            static_cast<std::size_t>(mCurrentFrame.N);
        mappingFilterRecord.maximumFeatureFraction =
            mfGeometrySparseFlowMappingFilterMaximumFeatureFraction;
        mappingFilterRecord.maximumDepthFraction =
            mfGeometrySparseFlowMappingFilterMaximumDepthFraction;
        mappingFilterRecord.minimumRemainingDepthFeatures =
            mnGeometrySparseFlowMappingFilterMinimumRemainingDepthFeatures;

        mappingFilterCandidateVectorValid =
            mCurrentSparseFlowFilterResult.candidateMask.size()==
                mappingFilterRecord.featureCount &&
            mvbCurrentSparseFlowRemovedAssociations.size()==
                mappingFilterRecord.featureCount &&
            mCurrentFrame.mvpMapPoints.size()==
                mappingFilterRecord.featureCount &&
            mCurrentFrame.mvbOutlier.size()==
                mappingFilterRecord.featureCount &&
            mCurrentFrame.mvbDynamic.size()==
                mappingFilterRecord.featureCount &&
            mCurrentFrame.mvDepth.size()==
                mappingFilterRecord.featureCount;
        mappingFilterRecord.candidateVectorValid =
            mappingFilterCandidateVectorValid;

        if(mappingFilterCandidateVectorValid)
        {
            for(std::size_t index=0;
                index<mappingFilterRecord.featureCount; ++index)
            {
                const bool currentlyDynamic =
                    mCurrentFrame.mvbDynamic[index]!=0;
                if(!currentlyDynamic)
                    ++mappingFilterRecord.availableFeatures;

                const bool candidate =
                    mCurrentSparseFlowFilterResult.
                        candidateMask[index]!=0 &&
                    !currentlyDynamic;
                if(candidate)
                {
                    ++mappingFilterRecord.candidateFeatures;
                    if(mCurrentFrame.mvpMapPoints[index])
                    {
                        ++mappingFilterRecord.
                            candidateAssociationsBeforeVeto;
                    }
                    if(mvbCurrentSparseFlowRemovedAssociations[
                           index]!=0)
                    {
                        ++mappingFilterRecord.
                            candidateTrackingRemovals;
                    }
                }

                const bool validDepth =
                    mCurrentFrame.mvDepth[index]>0.0f &&
                    !currentlyDynamic;
                if(validDepth)
                {
                    ++mappingFilterRecord.validDepthFeatures;
                    if(candidate)
                    {
                        ++mappingFilterRecord.
                            candidateValidDepthFeatures;
                    }
                }
            }
        }

        if(mappingFilterRecord.availableFeatures>0)
        {
            mappingFilterRecord.candidateFeatureFraction =
                static_cast<double>(
                    mappingFilterRecord.candidateFeatures)/
                static_cast<double>(
                    mappingFilterRecord.availableFeatures);
        }
        if(mappingFilterRecord.validDepthFeatures>0)
        {
            mappingFilterRecord.candidateDepthFraction =
                static_cast<double>(
                    mappingFilterRecord.
                        candidateValidDepthFeatures)/
                static_cast<double>(
                    mappingFilterRecord.validDepthFeatures);
        }
        const std::size_t prospectiveRemainingValidDepth =
            mappingFilterRecord.validDepthFeatures>=
                mappingFilterRecord.candidateValidDepthFeatures
            ? mappingFilterRecord.validDepthFeatures-
                mappingFilterRecord.candidateValidDepthFeatures
            : 0;
        mappingFilterRecord.remainingValidDepthFeatures =
            mappingFilterRecord.validDepthFeatures;

        if(!mappingFilterCandidateVectorValid)
        {
            mappingFilterRecord.state =
                "vector_size_mismatch_fail_open";
        }
        else if(!mappingFilterRecord.scaleValid)
        {
            mappingFilterRecord.state =
                "scale_invalid_fail_open";
        }
        else if(!mappingFilterRecord.trackingSafeguardsPassed)
        {
            mappingFilterRecord.state =
                "tracking_safeguard_fail_open";
        }
        else if(mappingFilterRecord.candidateFeatures==0)
        {
            mappingFilterRecord.state = "no_candidates";
        }
        else if(mappingFilterRecord.candidateFeatureFraction>
                mappingFilterRecord.maximumFeatureFraction)
        {
            mappingFilterRecord.state =
                "maximum_feature_fraction_fail_open";
        }
        else if(mappingFilterRecord.candidateDepthFraction>
                mappingFilterRecord.maximumDepthFraction)
        {
            mappingFilterRecord.state =
                "maximum_depth_fraction_fail_open";
        }
        else if(prospectiveRemainingValidDepth<
                mappingFilterRecord.
                    minimumRemainingDepthFeatures)
        {
            mappingFilterRecord.state =
                "minimum_remaining_depth_fail_open";
        }
        else
        {
            for(std::size_t index=0;
                index<mappingFilterRecord.featureCount; ++index)
            {
                if(mCurrentSparseFlowFilterResult.
                       candidateMask[index]==0 ||
                   mCurrentFrame.mvbDynamic[index]!=0)
                {
                    continue;
                }
                mCurrentFrame.mvbDynamic[index] = 1;
                ++mappingFilterRecord.newDynamicFlags;
                if(mCurrentFrame.mvDepth[index]>0.0f)
                    ++mappingFilterRecord.vetoedDepthFeatures;
            }
            mappingFilterRecord.applied =
                mappingFilterRecord.newDynamicFlags>0;
            mappingFilterRecord.remainingValidDepthFeatures =
                prospectiveRemainingValidDepth;
            mappingFilterRecord.state =
                mappingFilterRecord.applied
                    ? "applied" : "no_candidates";
        }
    }

    RemoveDynamicAssociations(mCurrentFrame);
    if(applyMappingFilter && mappingFilterRecord.applied)
    {
        mappingFilterRecord.removedAssociations =
            mappingFilterRecord.candidateAssociationsBeforeVeto;
    }

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
                if(recordMappingAdmission)
                {
                    ++mappingRecord.validDepthFeatures;
                    if(candidateVectorValid &&
                       mCurrentSparseFlowFilterResult.
                           candidateMask[
                               static_cast<std::size_t>(i)]!=0)
                    {
                        ++mappingRecord.
                            candidateValidDepthFeatures;
                    }
                }
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
                const std::size_t featureIndex =
                    static_cast<std::size_t>(i);
                const bool geometryCandidate =
                    recordMappingAdmission &&
                    candidateVectorValid &&
                    mCurrentSparseFlowFilterResult.
                        candidateMask[featureIndex]!=0;
                const bool removedByTracking =
                    geometryCandidate &&
                    mvbCurrentSparseFlowRemovedAssociations[
                        featureIndex]!=0;
                const bool mappingFilterCandidate =
                    applyMappingFilter &&
                    mappingFilterCandidateVectorValid &&
                    mCurrentSparseFlowFilterResult.
                        candidateMask[featureIndex]!=0;
                if(recordMappingAdmission)
                {
                    ++mappingRecord.depthAdmissionFeatures;
                    if(geometryCandidate)
                    {
                        ++mappingRecord.
                            candidateDepthAdmissionFeatures;
                    }
                }

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
                    if(recordMappingAdmission &&
                       geometryCandidate)
                    {
                        ++mappingRecord.
                            candidateCreatedMapPoints;
                        if(removedByTracking)
                        {
                            ++mappingRecord.
                                recreatedAfterTrackingRemoval;
                        }
                    }
                    if(applyMappingFilter &&
                       mappingFilterCandidate)
                    {
                        ++mappingFilterRecord.
                            candidateCreatedMapPoints;
                    }
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    if(mbGeometrySparseFlowMapQualityAuditEnabled &&
                       (geometryCandidate ||
                        mappingFilterCandidate))
                    {
                        GeometrySparseFlowCandidateMapPointRecord
                            lifecycleRecord;
                        lifecycleRecord.frameId =
                            mCurrentFrame.mnId;
                        lifecycleRecord.timestamp =
                            mCurrentFrame.mTimeStamp;
                        lifecycleRecord.featureIndex =
                            featureIndex;
                        lifecycleRecord.pixelX =
                            mCurrentFrame.mvKeys[i].pt.x;
                        lifecycleRecord.pixelY =
                            mCurrentFrame.mvKeys[i].pt.y;
                        lifecycleRecord.depthMeters =
                            mCurrentFrame.mvDepth[i];
                        lifecycleRecord.mode =
                            recordMappingAdmission
                                ? "g1_m0_counterfactual"
                                : "g1_m1_fail_open";
                        lifecycleRecord.mappingState =
                            recordMappingAdmission
                                ? "counterfactual_no_veto"
                                : mappingFilterRecord.state;
                        lifecycleRecord.originalMapPointId =
                            pNewMP->mnId;
                        lifecycleRecord.originalMapPoint =
                            pNewMP;
                        mvGeometrySparseFlowCandidateMapPoints.
                            push_back(lifecycleRecord);
                    }

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                    if(recordMappingAdmission)
                        ++mappingRecord.createdMapPoints;
                    if(applyMappingFilter)
                        ++mappingFilterRecord.createdMapPoints;
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

    if(recordMappingAdmission)
    {
        mvGeometrySparseFlowMappingAdmissionDiagnostics.
            push_back(mappingRecord);
        cout << "[Geometry G1-M0] frame="
             << mappingRecord.frameId
             << " state=" << mappingRecord.candidateState
             << " candidates="
             << mappingRecord.candidateFeatures
             << " candidate_assoc="
             << mappingRecord.
                    candidateAssociationsBeforeMapping
             << " tracking_removed="
             << mappingRecord.candidateTrackingRemovals
             << " created=" << mappingRecord.createdMapPoints
             << " candidate_created="
             << mappingRecord.candidateCreatedMapPoints
             << " recreated_after_tracking="
             << mappingRecord.
                    recreatedAfterTrackingRemoval
             << " counterfactual_only=true"
             << " mapping_veto=none"
             << endl;
    }
    if(applyMappingFilter)
    {
        mvGeometrySparseFlowMappingFilterDiagnostics.
            push_back(mappingFilterRecord);
        cout << "[Geometry G1-M1] frame="
             << mappingFilterRecord.frameId
             << " state=" << mappingFilterRecord.state
             << " tracking_state="
             << mappingFilterRecord.trackingSafeguardState
             << " candidates="
             << mappingFilterRecord.candidateFeatures
             << " candidate_feature_fraction="
             << mappingFilterRecord.candidateFeatureFraction
             << " candidate_depth="
             << mappingFilterRecord.candidateValidDepthFeatures
             << " candidate_depth_fraction="
             << mappingFilterRecord.candidateDepthFraction
             << " new_dynamic_flags="
             << mappingFilterRecord.newDynamicFlags
             << " vetoed_depth="
             << mappingFilterRecord.vetoedDepthFeatures
             << " created="
             << mappingFilterRecord.createdMapPoints
             << " candidate_created="
             << mappingFilterRecord.candidateCreatedMapPoints
             << " mapping_filter_applied="
             << (mappingFilterRecord.applied ? "true" : "false")
             << " pose_reoptimization=none"
             << endl;
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
    mSInStyleDetector.Reset();
    mSInStyleInitialRegionClusterer.Reset();
    mSInStyleDenseFlowResidualEstimator.Reset();
    mSInStyleRegionDynamicClassifier.Reset();
    mvbCurrentSInRegionDynamicFeatures.clear();
    mvbCurrentSInRegionNewDynamicFeatures.clear();
    mvbCurrentSInRegionRemovedAssociations.clear();
    mbCurrentSInRegionTrackingFailOpen = false;
    mCurrentSInGeometryDynamicMask.release();
    mbCurrentSInGeometryEvidenceAvailable = false;
    mCurrentSInDepthFilterResult = SInStyleDepthFilterResult();
    mbCurrentSInDepthMappingAdmissible = false;
    ++mnSInStyleResetEpoch;
    mGeometricDetector.ResetReference();
    mGeometricGroundTruthDetector.ResetReference();
    mCurrentDepthMeters.release();
    mCurrentGeometryDebugImage.release();
    mCurrentGroundTruthTcw.release();
    mvGeometryPoseDiagnostics.clear();
    mvGeometrySemanticProxyDiagnostics.clear();
    mvGeometryFeatureShadowDiagnostics.clear();
    mvGeometryMultiReferenceHistogram.clear();
    mvGeometryMultiReferenceFeatureDiagnostics.clear();
    mvGeometrySparseFlowFeatureDiagnostics.clear();
    mvGeometrySparseFlowFrameDiagnostics.clear();
    mvGeometryAssociationSnapshotDiagnostics.clear();
    mvGeometrySparseFlowTrackingFilterDiagnostics.clear();
    mvGeometrySparseFlowRemovedAssociationDiagnostics.clear();
    mvGeometrySparseFlowCandidateAssociationDiagnostics.clear();
    mvGeometrySparseFlowMappingAdmissionDiagnostics.clear();
    mvGeometrySparseFlowMappingFilterDiagnostics.clear();
    mvGeometrySparseFlowCandidateMapPoints.clear();
    mCurrentSparseFlowFilterResult =
        GeometricSparseFlowFilterResult();
    mvbCurrentSparseFlowRemovedAssociations.clear();
    mbCurrentSparseFlowTrackingSafeguardsPassed = false;
    mCurrentSparseFlowTrackingSafeguardState =
        "not_evaluated";
    mvGeometryLocalRigidityNodeDiagnostics.clear();
    mvGeometryLocalRigidityEdgeDiagnostics.clear();
    mvGeometryLocalRigidityFrameDiagnostics.clear();
    mvGeometryRigidHypothesisDiagnostics.clear();
    mvGeometryRigidHypothesisFrameDiagnostics.clear();
    mvGeometryRegionEvidenceDiagnostics.clear();
    mvGeometryReferenceSelectionDiagnostics.clear();
    mqGeometryKeyframeReferences.clear();
    mSparseFlowReference.gray.release();
    mSparseFlowReference.depthMeters.release();
    mSparseFlowReference.TcwFinal.release();
    mSparseFlowReference.TcwGroundTruth.release();
    mSparseFlowReference.valid = false;
    mnGeometryComputedFrames = 0;
    mnGeometryMultiReferenceComputedFrames = 0;
    mnGeometrySparseFlowComputedFrames = 0;
    mnGeometryLocalRigidityComputedFrames = 0;
    mnGeometryRigidHypothesisComputedFrames = 0;
    mnGeometryRegionEvidenceComputedFrames = 0;

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
